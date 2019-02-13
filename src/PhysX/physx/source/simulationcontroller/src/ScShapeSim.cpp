//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "ScBodySim.h"
#include "ScStaticSim.h"
#include "ScScene.h"
#include "ScElementSimInteraction.h"
#include "ScShapeInteraction.h"
#include "ScTriggerInteraction.h"
#include "ScSimStats.h"
#include "ScObjectIDTracker.h"
#include "GuHeightFieldUtil.h"
#include "GuTriangleMesh.h"
#include "GuConvexMeshData.h"
#include "GuHeightField.h"
#include "PxsContext.h"
#include "PxsTransformCache.h"
#include "CmTransformUtils.h"
#include "GuBounds.h"
#include "PxsRigidBody.h"
#include "ScSqBoundsManager.h"
#include "PxsSimulationController.h"
#include "common/PxProfileZone.h"

using namespace physx;
using namespace Gu;
using namespace Sc;

// PT: keep local functions in cpp, no need to pollute the header. Don't force conversions to bool if not necessary.
static PX_FORCE_INLINE PxU32 hasTriggerFlags(PxShapeFlags flags)	{ return PxU32(flags) & PxU32(PxShapeFlag::eTRIGGER_SHAPE);									}
static PX_FORCE_INLINE PxU32 isBroadPhase(PxShapeFlags flags)		{ return PxU32(flags) & PxU32(PxShapeFlag::eTRIGGER_SHAPE|PxShapeFlag::eSIMULATION_SHAPE);	}

static PX_FORCE_INLINE void resetElementID(Scene& scene, ShapeSim& shapeSim)
{
	PX_ASSERT(!shapeSim.isInBroadPhase());

	scene.getDirtyShapeSimMap().reset(shapeSim.getElementID());

	if(shapeSim.getSqBoundsId() != PX_INVALID_U32)
		shapeSim.destroySqBounds();
}

void ShapeSim::initSubsystemsDependingOnElementID()
{
	Scene& scScene = getScene();

	Bp::BoundsArray& boundsArray = scScene.getBoundsArray();
	const PxU32 index = getElementID();

	PX_ALIGN(16, PxTransform absPos);
	getAbsPoseAligned(&absPos);

	PxsTransformCache& cache = scScene.getLowLevelContext()->getTransformCache();
	cache.initEntry(index);
	cache.setTransformCache(absPos, 0, index);

	boundsArray.updateBounds(absPos, mCore.getGeometryUnion(), index);
	
	{
		PX_PROFILE_ZONE("API.simAddShapeToBroadPhase", scScene.getContextId());
		if(isBroadPhase(mCore.getFlags()))
			internalAddToBroadPhase();			
		else
			scScene.getAABBManager()->reserveSpaceForBounds(index);
		scScene.updateContactDistance(index, getContactOffset());
	}

	if(scScene.getDirtyShapeSimMap().size() <= index)
		scScene.getDirtyShapeSimMap().resize(PxMax(index+1, (scScene.getDirtyShapeSimMap().size()+1) * 2u));

	RigidSim& owner = getRbSim();
	if(owner.isDynamicRigid() && static_cast<BodySim&>(owner).isActive())
		createSqBounds();

	// Init LL shape
	{
		mLLShape.mElementIndex = index;
		mLLShape.mShapeCore = const_cast<PxsShapeCore*>(&mCore.getCore());
	
		if(owner.getActorType()==PxActorType::eRIGID_STATIC)
		{
			mLLShape.mBodySimIndex = IG::NodeIndex(IG_INVALID_NODE);
		}
		else
		{
			BodySim& bodySim = static_cast<BodySim&>(getActor());
			mLLShape.mBodySimIndex = bodySim.getNodeIndex();
			//mLLShape.mLocalBound = computeBounds(mCore.getGeometry(), PxTransform(PxIdentity));
		}
	}
}

ShapeSim::ShapeSim(RigidSim& owner, const ShapeCore& core) :
	ElementSim	(owner),
	mCore		(core),
	mSqBoundsId	(PX_INVALID_U32)
{
	// sizeof(ShapeSim) = 32 bytes
	Scene& scScene = getScene();

	mId = scScene.getShapeIDTracker().createID();

	initSubsystemsDependingOnElementID();
}

ShapeSim::~ShapeSim()
{
	Scene& scScene = getScene();
	resetElementID(scScene, *this);
	scScene.getShapeIDTracker().releaseID(mId);
}

Bp::FilterGroup::Enum ShapeSim::getBPGroup() const
{
	bool isKinematic = false;
	BodySim* bs = getBodySim();
	if(bs)
		isKinematic = bs->isKinematic();

	RigidSim& rbSim = getRbSim();
	return Bp::getFilterGroup(rbSim.getActorType()==PxActorType::eRIGID_STATIC, rbSim.getRigidID(), isKinematic);
}

PX_FORCE_INLINE void ShapeSim::internalAddToBroadPhase()
{
	PX_ASSERT(!isInBroadPhase());

	addToAABBMgr(mCore.getContactOffset(), getBPGroup(), Ps::IntBool(mCore.getCore().mShapeFlags & PxShapeFlag::eTRIGGER_SHAPE));
}

PX_FORCE_INLINE void ShapeSim::internalRemoveFromBroadPhase(bool wakeOnLostTouch)
{
	PX_ASSERT(isInBroadPhase());
	removeFromAABBMgr();

	Scene& scene = getScene();
	PxsContactManagerOutputIterator outputs = scene.getLowLevelContext()->getNphaseImplementationContext()->getContactManagerOutputs();
	scene.getNPhaseCore()->onVolumeRemoved(this, wakeOnLostTouch ? PxU32(PairReleaseFlag::eWAKE_ON_LOST_TOUCH) : 0, outputs, scene.getPublicFlags() & PxSceneFlag::eADAPTIVE_FORCE);
}

void ShapeSim::removeFromBroadPhase(bool wakeOnLostTouch)
{
	if(isInBroadPhase())
		internalRemoveFromBroadPhase(wakeOnLostTouch);
}

void ShapeSim::reinsertBroadPhase()
{
	if(isInBroadPhase())
		internalRemoveFromBroadPhase();
//	internalAddToBroadPhase();

	Scene& scene = getScene();

	// Scene::removeShape
	{
		//unregisterShapeFromNphase(shape.getCore());

		// PT: "getID" is const but the addShape call used LLShape, which uses elementID, so....
		scene.getSimulationController()->removeShape(getID());
	}

	// Call ShapeSim dtor
	{
		resetElementID(scene, *this);
	}

	// Call ElementSim dtor
	{
		releaseID();
	}

	// Call ElementSim ctor
	{
		initID();
	}

	// Call ShapeSim ctor
	{
		initSubsystemsDependingOnElementID();
	}

	// Scene::addShape
	{
		scene.getSimulationController()->addShape(&getLLShapeSim(), getID());
		// PT: TODO: anything else needed here?
		//registerShapeInNphase(shapeCore);
	}
}

void ShapeSim::onFilterDataChange()
{
	setElementInteractionsDirty(InteractionDirtyFlag::eFILTER_STATE, InteractionFlag::eFILTERABLE);
}

void ShapeSim::onResetFiltering()
{
	if(isInBroadPhase())
		reinsertBroadPhase();
}

void ShapeSim::onMaterialChange()
{
	setElementInteractionsDirty(InteractionDirtyFlag::eMATERIAL, InteractionFlag::eRB_ELEMENT);
}

void ShapeSim::onRestOffsetChange()
{
	setElementInteractionsDirty(InteractionDirtyFlag::eREST_OFFSET, InteractionFlag::eRB_ELEMENT);
}

void ShapeSim::onContactOffsetChange()
{
	if(isInBroadPhase())
		getScene().getAABBManager()->setContactOffset(getElementID(), mCore.getContactOffset());
}

void ShapeSim::onFlagChange(PxShapeFlags oldFlags)
{
	const PxShapeFlags newFlags = mCore.getFlags();

	const bool oldBp = isBroadPhase(oldFlags)!=0;
	const bool newBp = isBroadPhase(newFlags)!=0;

	// Change of collision shape flags requires removal/add to broadphase
	if(oldBp != newBp)
	{
		if(!oldBp && newBp)
		{
			// A.B. if a trigger was removed and inserted within the same frame we need to reinsert
			if(hasTriggerFlags(newFlags) && getScene().getAABBManager()->isMarkedForRemove(getElementID()))
			{
				reinsertBroadPhase();
			}
			else
			{
				internalAddToBroadPhase();
			}
		}
		else
			internalRemoveFromBroadPhase();
	}
	else
	{
		const bool wasTrigger = hasTriggerFlags(oldFlags)!=0;
		const bool isTrigger = hasTriggerFlags(newFlags)!=0;
		if(wasTrigger != isTrigger)
			reinsertBroadPhase();  // re-insertion is necessary because trigger pairs get killed
	}

	const PxShapeFlags hadSq = oldFlags & PxShapeFlag::eSCENE_QUERY_SHAPE;
	const PxShapeFlags hasSq = newFlags & PxShapeFlag::eSCENE_QUERY_SHAPE;
	if(hasSq && !hadSq)
	{
		BodySim* body = getBodySim();
		if(body &&  body->isActive())
			createSqBounds();
	}
	else if(hadSq && !hasSq)
		destroySqBounds();
}

void ShapeSim::getAbsPoseAligned(PxTransform* PX_RESTRICT globalPose) const
{
	const PxTransform& shape2Actor = mCore.getCore().transform;
	const PxTransform* actor2World = NULL;
	if(getActor().getActorType()==PxActorType::eRIGID_STATIC)
	{
		PxsRigidCore& core = static_cast<StaticSim&>(getActor()).getStaticCore().getCore();
		actor2World = &core.body2World;
	}
	else
	{
		PxsBodyCore& core = static_cast<BodySim&>(getActor()).getBodyCore().getCore();
		if(!core.mIdtBody2Actor)
		{
			Cm::getDynamicGlobalPoseAligned(core.body2World, shape2Actor, core.getBody2Actor(), *globalPose);
			return;
		}
		actor2World = &core.body2World;
	}
	Cm::getStaticGlobalPoseAligned(*actor2World, shape2Actor, *globalPose);
}

BodySim* ShapeSim::getBodySim() const
{ 
	ActorSim& a = getActor();
	return a.isDynamicRigid() ? static_cast<BodySim*>(&a) : NULL;
}

PxsRigidCore& ShapeSim::getPxsRigidCore() const
{
	ActorSim& a = getActor();
	return a.isDynamicRigid() ? static_cast<BodySim&>(a).getBodyCore().getCore()
							  : static_cast<StaticSim&>(a).getStaticCore().getCore();
}

void ShapeSim::updateCached(PxU32 transformCacheFlags, Cm::BitMapPinned* shapeChangedMap)
{
	PX_ALIGN(16, PxTransform absPose);
	getAbsPoseAligned(&absPose);

	Scene& scene = getScene();
	const PxU32 index = getElementID();

	scene.getLowLevelContext()->getTransformCache().setTransformCache(absPose, transformCacheFlags, index);
	scene.getBoundsArray().updateBounds(absPose, mCore.getGeometryUnion(), index);
	if (shapeChangedMap && isInBroadPhase())
		shapeChangedMap->growAndSet(index);
}

void ShapeSim::updateCached(PxsTransformCache& transformCache, Bp::BoundsArray& boundsArray)
{
	const PxU32 index = getElementID();

	PxsCachedTransform& ct = transformCache.getTransformCache(index);
	Ps::prefetchLine(&ct);

	getAbsPoseAligned(&ct.transform);

	ct.flags = 0;

	PxBounds3& b = boundsArray.begin()[index];
	Gu::computeBounds(b, mCore.getGeometryUnion().getGeometry(), ct.transform, 0.0f, NULL, 1.0f);
}

void ShapeSim::updateContactDistance(PxReal* contactDistance, const PxReal inflation, const PxVec3 angVel, const PxReal dt, Bp::BoundsArray& boundsArray)
{
	const PxU32 index = getElementID();

	const PxBounds3& bounds = boundsArray.getBounds(index);

	PxReal radius = bounds.getExtents().magnitude();

	//Heuristic for angular velocity...
	PxReal angularInflation = angVel.magnitude() * dt * radius;

	contactDistance[index] = getContactOffset() + inflation + angularInflation;
}

Ps::IntBool ShapeSim::updateSweptBounds()
{
	Vec3p endOrigin, endExtent;
	const ShapeCore& shapeCore = mCore;
	const PxTransform& endPose = getScene().getLowLevelContext()->getTransformCache().getTransformCache(getElementID()).transform;
	PxReal ccdThreshold = computeBoundsWithCCDThreshold(endOrigin, endExtent, shapeCore.getGeometry(), endPose, NULL);

	PxBounds3 bounds = PxBounds3::centerExtents(endOrigin, endExtent);

	BodySim* body = getBodySim();
	PxcRigidBody& rigidBody = body->getLowLevelBody();
	PxsBodyCore& bodyCore = body->getBodyCore().getCore();
	PX_ALIGN(16, PxTransform shape2World);
	Cm::getDynamicGlobalPoseAligned(rigidBody.mLastTransform, shapeCore.getShape2Actor(), bodyCore.getBody2Actor(), shape2World);
	PxBounds3 startBounds = computeBounds(shapeCore.getGeometry(), shape2World);

	const Ps::IntBool isFastMoving = (startBounds.getCenter() - endOrigin).magnitudeSquared() >= ccdThreshold * ccdThreshold ? 1 : 0;

	if (isFastMoving)
		bounds.include(startBounds);

	PX_ASSERT(bounds.minimum.x <= bounds.maximum.x
		&&	  bounds.minimum.y <= bounds.maximum.y
		&&	  bounds.minimum.z <= bounds.maximum.z);

	getScene().getBoundsArray().setBounds(bounds, getElementID());

	return isFastMoving;	
}

void ShapeSim::updateBPGroup()
{
	if(isInBroadPhase())
	{
		Sc::Scene& scene = getScene();
		scene.getAABBManager()->setBPGroup(getElementID(), getBPGroup());

		reinsertBroadPhase();
//		internalRemoveFromBroadPhase();
//		internalAddToBroadPhase();
	}
}

void ShapeSim::markBoundsForUpdate(bool forceBoundsUpdate)
{
	Scene& scene = getScene();
	if(forceBoundsUpdate)
		updateCached(0, &scene.getAABBManager()->getChangedAABBMgActorHandleMap());
	else if(isInBroadPhase())
		scene.getDirtyShapeSimMap().growAndSet(getElementID());
}

static PX_FORCE_INLINE void updateInteraction(Scene& scene, Interaction* i, const bool isDynamic, const bool isAsleep)
{
	if(i->getType() == InteractionType::eOVERLAP)
	{
		ShapeInteraction* si = static_cast<ShapeInteraction*>(i);
		si->resetManagerCachedState();
			
		if (isAsleep)
			si->onShapeChangeWhileSleeping(isDynamic);
	}
	else if(i->getType() == InteractionType::eTRIGGER)
		(static_cast<TriggerInteraction*>(i))->forceProcessingThisFrame(scene);  // trigger pairs need to be checked next frame
}

void ShapeSim::onVolumeOrTransformChange(bool forceBoundsUpdate)
{
	Scene& scene = getScene();
	BodySim* body = getBodySim();
	const bool isDynamic = (body != NULL);
	const bool isAsleep = body ? !body->isActive() : true;

	ElementSim::ElementInteractionIterator iter = getElemInteractions();
	ElementSimInteraction* i = iter.getNext();
	while(i)
	{
		updateInteraction(scene, i, isDynamic, isAsleep);
		i = iter.getNext();
	}

	markBoundsForUpdate(forceBoundsUpdate);
}

void notifyActorInteractionsOfTransformChange(ActorSim& actor)
{
	bool isDynamic;
	bool isAsleep;
	if(actor.isDynamicRigid())
	{
		isDynamic = true;
		isAsleep = !static_cast<BodySim&>(actor).isActive();
	}
	else
	{
		isDynamic = false;
		isAsleep = true;
	}

	Scene& scene = actor.getScene();

	PxU32 nbInteractions = actor.getActorInteractionCount();
	Interaction** interactions = actor.getActorInteractions();
	while(nbInteractions--)
		updateInteraction(scene, *interactions++, isDynamic, isAsleep);
}

void ShapeSim::createSqBounds()
{
	if(mSqBoundsId!=PX_INVALID_U32)
		return;

	BodySim* bodySim = getBodySim();
	PX_ASSERT(bodySim);

	if(bodySim->usingSqKinematicTarget() || bodySim->isFrozen() || !bodySim->isActive() || bodySim->readInternalFlag(BodySim::BF_IS_COMPOUND_RIGID))
		return;

	if(mCore.getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
		getScene().getSqBoundsManager().addShape(*this);
}

void ShapeSim::destroySqBounds()
{
	if(mSqBoundsId!=PX_INVALID_U32)
		getScene().getSqBoundsManager().removeShape(*this);
}

