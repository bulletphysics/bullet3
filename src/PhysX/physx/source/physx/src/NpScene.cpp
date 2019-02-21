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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "PxSimulationEventCallback.h"

#include "NpScene.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulation.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationLink.h"
#include "NpArticulationJoint.h"
#include "NpAggregate.h"
#include "NpBatchQuery.h"
#include "SqPruner.h"
#include "SqPruningStructure.h"
#include "SqSceneQueryManager.h"
#include "GuBVHStructure.h"

#include "ScbNpDeps.h"
#include "ScArticulationSim.h"
#include "ScConstraintSim.h"
#include "CmCollection.h"
#include "CmUtils.h"


#if PX_SUPPORT_GPU_PHYSX
#include "task/PxGpuDispatcher.h"
#endif

#include "extensions/PxJoint.h"

#include "PxsIslandSim.h"
#include "common/PxProfileZone.h"

using namespace physx;

// enable thread checks in all debug builds
#if PX_DEBUG || PX_CHECKED
#define NP_ENABLE_THREAD_CHECKS 1
#else
#define NP_ENABLE_THREAD_CHECKS 0
#endif

using namespace shdfnd;
using namespace Sq;

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE bool removeFromSceneCheck(NpScene* npScene, PxScene* scene, const char* name)
{
	if (scene == static_cast<PxScene*>(npScene))
	{
		return true;
	}
	else
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "%s not assigned to scene or assigned to another scene. Call will be ignored!", name);
		return false;
	}
}

///////////////////////////////////////////////////////////////////////////////


NpSceneQueries::NpSceneQueries(const PxSceneDesc& desc) : 
	mScene					(desc, getContextId()),
	mSQManager				(mScene, desc.staticStructure, desc.dynamicStructure, desc.dynamicTreeRebuildRateHint, desc.limits),
	mCachedRaycastFuncs		(Gu::getRaycastFuncTable()),
	mCachedSweepFuncs		(Gu::getSweepFuncTable()),
	mCachedOverlapFuncs		(Gu::getOverlapFuncTable()),
	mSceneQueriesStaticPrunerUpdate		(getContextId(), 0, "NpSceneQueries.sceneQueriesStaticPrunerUpdate"),
	mSceneQueriesDynamicPrunerUpdate(getContextId(), 0, "NpSceneQueries.sceneQueriesDynamicPrunerUpdate"),
	mSceneQueryUpdateMode	(desc.sceneQueryUpdateMode)
#if PX_SUPPORT_PVD
	, mSingleSqCollector	(mScene, false),
	mBatchedSqCollector		(mScene, true)
#endif
{
	mSceneQueriesStaticPrunerUpdate.setObject(this);
	mSceneQueriesDynamicPrunerUpdate.setObject(this);
}

NpScene::NpScene(const PxSceneDesc& desc) :
	NpSceneQueries			(desc),
	mConstraints			(PX_DEBUG_EXP("sceneConstraints")),
	mRigidActors			(PX_DEBUG_EXP("sceneRigidActors")),
	mArticulations			(PX_DEBUG_EXP("sceneArticulations")),
	mAggregates				(PX_DEBUG_EXP("sceneAggregates")),
	mSanityBounds			(desc.sanityBounds),
	mNbClients				(1),			//we always have the default client.
	mClientBehaviorFlags	(PX_DEBUG_EXP("sceneBehaviorFlags")),
	mSceneCompletion		(getContextId(), mPhysicsDone),
	mCollisionCompletion	(getContextId(), mCollisionDone),
	mSceneQueriesCompletion	(getContextId(), mSceneQueriesDone),
	mSceneExecution			(getContextId(), 0, "NpScene.execution"),
	mSceneCollide			(getContextId(), 0, "NpScene.collide"),
	mSceneAdvance			(getContextId(), 0, "NpScene.solve"),
	mControllingSimulation	(false),
	mSimThreadStackSize		(0),
	mConcurrentWriteCount	(0),
	mConcurrentReadCount	(0),
	mConcurrentErrorCount	(0),	
	mCurrentWriter			(0),
	mSceneQueriesUpdateRunning	(false),
	mHasSimulatedOnce		(false),
	mBetweenFetchResults	(false),
	mBuildFrozenActors		(false)
{
	mSceneExecution.setObject(this);
	mSceneCollide.setObject(this);
	mSceneAdvance.setObject(this);

	mTaskManager = mScene.getScScene().getTaskManagerPtr();
	mThreadReadWriteDepth = Ps::TlsAlloc();

	updatePhysXIndicator();

}

NpSceneQueries::~NpSceneQueries()
{
}

NpScene::~NpScene()
{
	// PT: we need to do that one first, now that we don't release the objects anymore. Otherwise we end up with a sequence like:
	// - actor is part of an aggregate, and part of a scene
	// - actor gets removed from the scene. This does *not* remove it from the aggregate.
	// - aggregate gets removed from the scene, sees that one contained actor ain't in the scene => we get a warning message
	PxU32 aggregateCount = mAggregates.size();
	while(aggregateCount--)
		removeAggregate(*mAggregates.getEntries()[aggregateCount], false);

	PxU32 rigidActorCount = mRigidActors.size();
	while(rigidActorCount--)
		removeActor(*mRigidActors[rigidActorCount], false);

	PxU32 articCount = mArticulations.size();
	while(articCount--)
		removeArticulation(*mArticulations.getEntries()[articCount], false);

	bool unlock = mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK;

#if PX_SUPPORT_PVD
	getSingleSqCollector().release();
	getBatchedSqCollector().release();
#endif

	// release batch queries
	PxU32 numSq = mBatchQueries.size();
	while(numSq--)
		PX_DELETE(mBatchQueries[numSq]);
	mBatchQueries.clear();

	mScene.release();

	// unlock the lock taken in release(), must unlock before 
	// mRWLock is destroyed otherwise behavior is undefined
	if (unlock)
		unlockWrite();

	TlsFree(mThreadReadWriteDepth);
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::release()
{
	// need to acquire lock for release, note this is unlocked in the destructor
	if (mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK)
		lockWrite(__FILE__, __LINE__);

	// It will be hard to do a write check here since all object release calls in the scene destructor do it and would mess
	// up the test. If we really want it on scene destruction as well, we need to either have internal and external release
	// calls or come up with a different approach (for example using thread ID as detector variable).

	if(getSimulationStage() != Sc::SimulationStage::eCOMPLETE)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::release(): Scene is still being simulated! PxScene::fetchResults() is called implicitly.");
		
		if(getSimulationStage() == Sc::SimulationStage::eCOLLIDE)
		{
			fetchCollision(true);
		}

		if(getSimulationStage() == Sc::SimulationStage::eFETCHCOLLIDE)  // need to call getSimulationStage() again beacause fetchCollision() might change the value.
		{
			// this is for split sim
			advance(NULL);
		}

		fetchResults(true, NULL);
	}
	NpPhysics::getInstance().releaseSceneInternal(*this);
}

///////////////////////////////////////////////////////////////////////////////

bool NpScene::loadFromDesc(const PxSceneDesc& desc)
{
	{
		if(desc.limits.maxNbActors)
			mRigidActors.reserve(desc.limits.maxNbActors);

		//const PxU32 totalNbShapes = desc.limits.maxNbStaticShapes + desc.limits.maxNbDynamicShapes;
		mScene.getScScene().preAllocate(desc.limits.maxNbActors, desc.limits.maxNbBodies, desc.limits.maxNbStaticShapes, desc.limits.maxNbDynamicShapes);
	}

	userData = desc.userData;

	return true;
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::setGravity(const PxVec3& g)
{
	NP_WRITE_CHECK(this);
	mScene.setGravity(g);
}

PxVec3 NpScene::getGravity() const
{
	NP_READ_CHECK(this);
	return mScene.getGravity();
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::setBounceThresholdVelocity(const PxReal t)
{
	NP_WRITE_CHECK(this);
	mScene.setBounceThresholdVelocity(t);
}

PxReal NpScene::getBounceThresholdVelocity() const
{
	NP_READ_CHECK(this)
	return mScene.getBounceThresholdVelocity();
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::setLimits(const PxSceneLimits& limits)
{
	NP_WRITE_CHECK(this);

	if(limits.maxNbActors)
		mRigidActors.reserve(limits.maxNbActors);

	mScene.getScScene().preAllocate(limits.maxNbActors, limits.maxNbBodies, limits.maxNbStaticShapes, limits.maxNbDynamicShapes);
	mScene.setLimits(limits);

	mSQManager.preallocate(limits.maxNbStaticShapes, limits.maxNbDynamicShapes);
}

//////////////////////////////////////////////////////////////////////////

PxSceneLimits NpScene::getLimits() const
{
	NP_READ_CHECK(this);

	return mScene.getLimits();
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::setFlag(PxSceneFlag::Enum flag, bool value)
{
	NP_WRITE_CHECK(this);

	// this call supports mutable flags only
	PX_CHECK_AND_RETURN(PxSceneFlags(flag) & PxSceneFlags(PxSceneFlag::eMUTABLE_FLAGS),
						"PxScene::setFlag: This flag is not mutable - you can only set it once in PxSceneDesc at startup!");

	PxSceneFlags currentFlags = mScene.getFlags();

	if(value)
		currentFlags |= flag;
	else
		currentFlags &= ~PxSceneFlags(flag);

	mScene.setFlags(currentFlags);
}

PxSceneFlags NpScene::getFlags() const
{
	NP_READ_CHECK(this);
	return mScene.getFlags();
}

///////////////////////////////////////////////////////////////////////////////

// PT: make sure we always add to array and set the array index properly / at the same time
template<class T>
static PX_FORCE_INLINE void addRigidActorToArray(T& a, Ps::Array<PxRigidActor*>& rigidActors)
{
	a.setRigidActorArrayIndex(rigidActors.size());
	rigidActors.pushBack(&a);
}

void NpScene::addActor(PxActor& actor, const PxBVHStructure* bvhStructure)
{
	PX_PROFILE_ZONE("API.addActor", getContextId());
	NP_WRITE_CHECK(this);
	PX_SIMD_GUARD;

	PxRigidStatic* a = actor.is<PxRigidStatic>();
	if(a)
	{
#if PX_CHECKED
		if(!static_cast<NpRigidStatic*>(a)->checkConstraintValidity())
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addActor(): actor has invalid constraint and may not be added to scene");
			return;
		}
#endif
		if(static_cast<NpRigidStatic*>(a)->getShapeManager().getPruningStructure())
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addActor(): actor is in a pruning structure and cannot be added to a scene directly, use addActors(const PxPruningStructure& )");
			return;
		}
	}

	PxRigidDynamic* aD = actor.is<PxRigidDynamic>();
	if(aD && static_cast<NpRigidDynamic*>(aD)->getShapeManager().getPruningStructure())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addActor(): actor is in a pruning structure and cannot be added to a scene directly, use addActors(const PxPruningStructure& )");
		return;
	}

	const Scb::ControlState::Enum cs = NpActor::getScbFromPxActor(actor).getControlState();
	if((cs == Scb::ControlState::eNOT_IN_SCENE) || ((cs == Scb::ControlState::eREMOVE_PENDING) && (NpActor::getOwnerScene(actor) == this)))
		addActorInternal(actor, bvhStructure);
	else
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addActor(): Actor already assigned to a scene. Call will be ignored!");
}

void NpScene::addActorInternal(PxActor& actor, const PxBVHStructure* bvhStructure)
{
	// BvhStructure check
	if(bvhStructure)
	{
		const PxRigidActor* rigidActor = actor.is<PxRigidActor>();
		if(!rigidActor || bvhStructure->getNbBounds() == 0 || bvhStructure->getNbBounds() > rigidActor->getNbShapes())
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxRigidActor::setBVHStructure structure is empty or does not match shapes in the actor.");
			return;
		}
	}

	switch(actor.getConcreteType())
	{
		case PxConcreteType::eRIGID_STATIC:
		{
			NpRigidStatic& npStatic = static_cast<NpRigidStatic&>(actor);
#if PX_CHECKED
			checkPositionSanity(npStatic, npStatic.getGlobalPose(), "PxScene::addActor or PxScene::addAggregate");
#endif
			addRigidStatic(npStatic, static_cast<const Gu::BVHStructure*>(bvhStructure));
		}
		break;

		case PxConcreteType::eRIGID_DYNAMIC:
		{
			NpRigidDynamic& npDynamic = static_cast<NpRigidDynamic&>(actor);
#if PX_CHECKED
			checkPositionSanity(npDynamic, npDynamic.getGlobalPose(), "PxScene::addActor or PxScene::addAggregate");
#endif
			addRigidDynamic(npDynamic, static_cast<const Gu::BVHStructure*>(bvhStructure));
		}
		break;

		case PxConcreteType::eARTICULATION_LINK:
		{
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::addActor(): Individual articulation links can not be added to the scene");
		}
		break;

		default:
			PX_ASSERT(0);
	}
}

void NpScene::updateScbStateAndSetupSq(const PxRigidActor& rigidActor, Scb::Actor& scbActor, NpShapeManager& shapeManager, bool actorDynamic, const PxBounds3* bounds, bool hasPrunerStructure)
{
	// all the things Scb does in non-buffered insertion
	SceneQueryManager& sqManager = getSceneQueryManagerFast();

	scbActor.setScbScene(&mScene);
	scbActor.setControlState(Scb::ControlState::eIN_SCENE);
	NpShape*const * shapes = shapeManager.getShapes();
	PxU32 nbShapes = shapeManager.getNbShapes();

	for(PxU32 i=0;i<nbShapes;i++)
	{
		NpShape& shape = *shapes[i];
		const PxShapeFlags shapeFlags = shape.getFlagsUnbuffered();	// PT: note that the regular code reads buffered flags

		shape.incRefCount();
		if(shape.isExclusiveFast())
		{
			shape.getScbShape().setScbScene(&mScene);
			shape.getScbShape().setControlState(Scb::ControlState::eIN_SCENE);
		}

		// PT: this part is copied from 'NpShapeManager::setupAllSceneQuery'
		if(shapeFlags & PxShapeFlag::eSCENE_QUERY_SHAPE)	// PT: TODO: refactor with 'isSceneQuery' in shape manager?
			shapeManager.addPrunerShape(sqManager, i, shape, rigidActor, actorDynamic, bounds ? bounds+i : NULL, hasPrunerStructure);
	}			
}

PX_FORCE_INLINE	void NpScene::updateScbStateAndSetupSq(const PxRigidActor& rigidActor, Scb::Body& body, NpShapeManager& shapeManager, bool actorDynamic, const PxBounds3* bounds, bool hasPrunerStructure)
{
	body.initBufferedState();
	updateScbStateAndSetupSq(rigidActor, static_cast<Scb::Actor&>(body), shapeManager, actorDynamic, bounds, hasPrunerStructure);
}

void NpScene::addActors(PxActor*const* actors, PxU32 nbActors)
{
	addActorsInternal(actors, nbActors, NULL);
}

void NpScene::addActors(const PxPruningStructure& ps)
{
	const Sq::PruningStructure& prunerStructure = static_cast<const Sq::PruningStructure&>(ps);
	if(!prunerStructure.isValid())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
			"PxScene::addActors(): Provided pruning structure is not valid.");
		return;
	}
	addActorsInternal(prunerStructure.getActors(), prunerStructure.getNbActors(), &prunerStructure);
}

void NpScene::addActorsInternal(PxActor*const* PX_RESTRICT actors, PxU32 nbActors, const Sq::PruningStructure* pS)
{
	PX_PROFILE_ZONE("API.addActors", getContextId());
	NP_WRITE_CHECK(this);	
	PX_SIMD_GUARD;

	if(getSimulationStage() != Sc::SimulationStage::eCOMPLETE) 
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, 
			"PxScene::addActors() not allowed while simulation is running.");
		return;
	}

	const bool hasPrunerStructure = pS ? true : false;
	Sc::Scene& scScene = mScene.getScScene();
	PxU32 actorsDone;

	Sc::BatchInsertionState scState;
	scScene.startBatchInsertion(scState);

	scState.staticActorOffset		= ptrdiff_t(size_t(&(reinterpret_cast<NpRigidStatic*>(0)->getScbRigidStaticFast().getScStatic())));
	scState.staticShapeTableOffset	= ptrdiff_t(size_t(&(reinterpret_cast<NpRigidStatic*>(0)->getShapeManager().getShapeTable())));
	scState.dynamicActorOffset		= ptrdiff_t(size_t(&(reinterpret_cast<NpRigidDynamic*>(0)->getScbBodyFast().getScBody())));
	scState.dynamicShapeTableOffset = ptrdiff_t(size_t(&(reinterpret_cast<NpRigidDynamic*>(0)->getShapeManager().getShapeTable())));
	scState.shapeOffset				= ptrdiff_t(NpShapeGetScPtrOffset());

	Ps::InlineArray<PxBounds3, 8> shapeBounds;
	for(actorsDone=0; actorsDone<nbActors; actorsDone++)
	{
		if(actorsDone+1<nbActors)
			Ps::prefetch(actors[actorsDone+1], sizeof(NpRigidDynamic));	// worst case: PxRigidStatic is smaller

		const Scb::ControlState::Enum cs = NpActor::getScbFromPxActor(*actors[actorsDone]).getControlState();
		if (!((cs == Scb::ControlState::eNOT_IN_SCENE) || ((cs == Scb::ControlState::eREMOVE_PENDING) && (NpActor::getOwnerScene(*actors[actorsDone]) == this))))
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addActors(): Actor already assigned to a scene. Call will be ignored!");
			break;
		}

		const PxType type = actors[actorsDone]->getConcreteType();
		if(type == PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic& a = *static_cast<NpRigidStatic*>(actors[actorsDone]);
#if PX_CHECKED
			if(!a.checkConstraintValidity())
			{
				Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addActors(): actor has invalid constraint and may not be added to scene");
				break;
			}
			checkPositionSanity(a, a.getGlobalPose(), "PxScene::addActors");
#endif
			if(!hasPrunerStructure && a.getShapeManager().getPruningStructure())
			{
				Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addActors(): actor is in a pruning structure and cannot be added to a scene directly, use addActors(const PxPruningStructure& )");
				break;
			}

			if(!(a.getScbRigidStaticFast().getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
			{
				shapeBounds.resizeUninitialized(a.NpRigidStatic::getNbShapes()+1);	// PT: +1 for safe reads in addPrunerData/inflateBounds
				scScene.addStatic(&a, scState, shapeBounds.begin());
				updateScbStateAndSetupSq(a, a.getScbActorFast(), a.getShapeManager(), false, shapeBounds.begin(), hasPrunerStructure);
				addRigidActorToArray(a, mRigidActors);
				a.addConstraintsToScene();
			}
			else
				addRigidStatic(a, NULL, hasPrunerStructure);
		}
		else if(type == PxConcreteType::eRIGID_DYNAMIC)
		{
			NpRigidDynamic& a = *static_cast<NpRigidDynamic*>(actors[actorsDone]);
#if PX_CHECKED
			checkPositionSanity(a, a.getGlobalPose(), "PxScene::addActors");
#endif
			if(!hasPrunerStructure && a.getShapeManager().getPruningStructure())
			{
				Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addActors(): actor is in a pruning structure and cannot be added to a scene directly, use addActors(const PxPruningStructure& )");
				break;
			}

			if(!(a.getScbBodyFast().getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
			{
				shapeBounds.resizeUninitialized(a.NpRigidDynamic::getNbShapes()+1);	// PT: +1 for safe reads in addPrunerData/inflateBounds
				scScene.addBody(&a, scState, shapeBounds.begin(), false);
				updateScbStateAndSetupSq(a, a.getScbBodyFast(), a.getShapeManager(), true, shapeBounds.begin(), hasPrunerStructure);
				addRigidActorToArray(a, mRigidActors);
				a.addConstraintsToScene();
			}
			else
				addRigidDynamic(a, NULL, hasPrunerStructure);
		}
		else
		{
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::addRigidActors(): articulation link not permitted");
			break;
		}
	}
	// merge sq PrunerStructure
	if(pS)
	{		
		mSQManager.addPruningStructure(*pS);
	}
	scScene.finishBatchInsertion(scState);

	// if we failed, still complete everything for the successful inserted actors before backing out	
#if PX_SUPPORT_PVD
	for(PxU32 i=0;i<actorsDone;i++)
	{
		if ((actors[i]->getConcreteType()==PxConcreteType::eRIGID_STATIC) && (!(static_cast<NpRigidStatic*>(actors[i])->getScbRigidStaticFast().getActorFlags() & PxActorFlag::eDISABLE_SIMULATION)))
			mScene.getScenePvdClient().addStaticAndShapesToPvd(static_cast<NpRigidStatic*>(actors[i])->getScbRigidStaticFast());
		else if ((actors[i]->getConcreteType() == PxConcreteType::eRIGID_DYNAMIC) && (!(static_cast<NpRigidDynamic*>(actors[i])->getScbBodyFast().getActorFlags() & PxActorFlag::eDISABLE_SIMULATION)))
			mScene.getScenePvdClient().addBodyAndShapesToPvd(static_cast<NpRigidDynamic*>(actors[i])->getScbBodyFast());
	}
#endif

	if(actorsDone<nbActors)	// Everything is consistent up to the failure point, so just use removeActor to back out gracefully if necessary
	{
		for(PxU32 j=0;j<actorsDone;j++)
			removeActorInternal(*actors[j], false, true);
	}
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::removeActors(PxActor*const* PX_RESTRICT actors, PxU32 nbActors, bool wakeOnLostTouch)
{
	PX_PROFILE_ZONE("API.removeActors", getContextId());
	NP_WRITE_CHECK(this);	
	
	Sc::Scene& scScene = mScene.getScScene();
	// resize the bitmap so it does not allocate each remove actor call
	scScene.resizeReleasedBodyIDMaps(mRigidActors.size(),nbActors);
	Sc::BatchRemoveState removeState;
	scScene.setBatchRemove(&removeState);
	 
	for(PxU32 actorsDone=0; actorsDone<nbActors; actorsDone++)
	{
		if(actorsDone+1<nbActors)
			Ps::prefetch(actors[actorsDone+1], sizeof(NpRigidDynamic));	// worst case: PxRigidStatic is smaller

		PxType type = actors[actorsDone]->getConcreteType();
		if (!removeFromSceneCheck(this, actors[actorsDone]->getScene(), "PxScene::removeActors(): Actor"))
		{			
			break;
		}
					
		removeState.bufferedShapes.clear();
		removeState.removedShapes.clear();		

		if(type == PxConcreteType::eRIGID_STATIC)
		{			
			NpRigidStatic& actor = *static_cast<NpRigidStatic*>(actors[actorsDone]);
			const PxActorFlags actorFlags = actor.getScbRigidStaticFast().getActorFlags();

			if(actor.getShapeManager().getNbShapes())
				Ps::prefetch(actor.getShapeManager().getShapes()[0],sizeof(NpShape));
			scScene.prefetchForRemove(actor.getScbRigidStaticFast().getScStatic());
			Ps::prefetch(mRigidActors[mRigidActors.size()-1],sizeof(NpRigidDynamic));

			const bool noSimBuffered = actorFlags.isSet(PxActorFlag::eDISABLE_SIMULATION);
			if (!noSimBuffered)
				actor.removeConstraintsFromScene();

			actor.getShapeManager().teardownAllSceneQuery(getSceneQueryManagerFast(), actor);

			Scb::RigidStatic& rs = actor.getScbRigidStaticFast();
			mScene.removeActor(rs, wakeOnLostTouch, rs.isSimDisabledInternally());
			removeFromRigidActorList(actor.getRigidActorArrayIndex());
		}
		else if(type == PxConcreteType::eRIGID_DYNAMIC)
		{			
			NpRigidDynamic& actor = *static_cast<NpRigidDynamic*>(actors[actorsDone]);	
			const PxActorFlags actorFlags = actor.getScbBodyFast().getActorFlags();

			if(actor.getShapeManager().getNbShapes())
				Ps::prefetch(actor.getShapeManager().getShapes()[0],sizeof(NpShape));
			scScene.prefetchForRemove(actor.getScbBodyFast().getScBody());
			Ps::prefetch(mRigidActors[mRigidActors.size()-1],sizeof(NpRigidDynamic));

			const bool noSimBuffered = actorFlags.isSet(PxActorFlag::eDISABLE_SIMULATION);			
			if (!noSimBuffered)
				actor.removeConstraintsFromScene();

			actor.getShapeManager().teardownAllSceneQuery(getSceneQueryManagerFast(), actor);

			Scb::Body& b = actor.getScbBodyFast();
			mScene.removeActor(b, wakeOnLostTouch, b.isSimDisabledInternally());
			removeFromRigidActorList(actor.getRigidActorArrayIndex());
		}
		else
		{
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::removeActor(): Individual articulation links can not be removed from the scene");
			break;
		}
	}	

	scScene.setBatchRemove(NULL);
}

void NpScene::removeActor(PxActor& actor, bool wakeOnLostTouch)
{
	PX_PROFILE_ZONE("API.removeActor", getContextId());
	NP_WRITE_CHECK(this);	
	if (removeFromSceneCheck(this, actor.getScene(), "PxScene::removeActor(): Actor"))
	{
		removeActorInternal(actor, wakeOnLostTouch, true);
	}
}

void NpScene::removeActorInternal(PxActor& actor, bool wakeOnLostTouch, bool removeFromAggregate)
{
	switch(actor.getType())
	{
		case PxActorType::eRIGID_STATIC:
		{
			NpRigidStatic& npStatic = static_cast<NpRigidStatic&>(actor);
			removeRigidStatic(npStatic, wakeOnLostTouch, removeFromAggregate);
		}
		break;

		case PxActorType::eRIGID_DYNAMIC:
		{
			NpRigidDynamic& npDynamic = static_cast<NpRigidDynamic&>(actor);
			removeRigidDynamic(npDynamic, wakeOnLostTouch, removeFromAggregate);
		}
		break;

		case PxActorType::eARTICULATION_LINK:
		{
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::removeActor(): Individual articulation links can not be removed from the scene");
		}
		break;
		
		case PxActorType::eACTOR_COUNT:
		case PxActorType::eACTOR_FORCE_DWORD:
			PX_ASSERT(0);
	}
}

///////////////////////////////////////////////////////////////////////////////

// PT: TODO: inline this one in the header for consistency
void NpScene::removeFromRigidActorList(const PxU32& index)
{
	PX_ASSERT(index != 0xFFFFFFFF);
	PX_ASSERT(index < mRigidActors.size());

	{
		const PxU32 size = mRigidActors.size() - 1;
		mRigidActors.replaceWithLast(index);
		if(size && size != index)
		{
			PxRigidActor& rigidActor = *mRigidActors[index];
			switch(rigidActor.getType())
			{
			case PxActorType::eRIGID_STATIC:
				{
					NpRigidStatic& npStatic = static_cast<NpRigidStatic&>(rigidActor);
					npStatic.setRigidActorArrayIndex(index);
				}
				break;
			case PxActorType::eRIGID_DYNAMIC:
				{
					NpRigidDynamic& npDynamic = static_cast<NpRigidDynamic&>(rigidActor);
					npDynamic.setRigidActorArrayIndex(index);
				}
				break;
			case PxActorType::eARTICULATION_LINK:
			case PxActorType::eACTOR_COUNT:
			case PxActorType::eACTOR_FORCE_DWORD:
				PX_ASSERT(0);
				break;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

template<class T, class T2>
static PX_FORCE_INLINE void addActorT(T& actor, T2& scbActor, Ps::Array<PxRigidActor*>& actors, NpScene* scene, const Gu::BVHStructure* bvhStructure, bool hasPrunerStructure)
{
	const bool noSimBuffered = scbActor.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION);

	PxBounds3 bounds[8+1];	// PT: +1 for safe reads in addPrunerData/inflateBounds
	const bool canReuseBounds = !noSimBuffered && !scene->getScene().isPhysicsBuffering() && actor.getShapeManager().getNbShapes()<=8;
	PxBounds3* uninflatedBounds = canReuseBounds ? bounds : NULL;

	scene->getScene().addActor(scbActor, noSimBuffered, uninflatedBounds, bvhStructure);

	actor.getShapeManager().setupAllSceneQuery(scene, actor, hasPrunerStructure, uninflatedBounds, bvhStructure);
	if(!noSimBuffered)
		actor.addConstraintsToScene();
	addRigidActorToArray(actor, actors);
}

void NpScene::addRigidStatic(NpRigidStatic& actor, const Gu::BVHStructure* bvhStructure, bool hasPrunerStructure)
{
	addActorT(actor, actor.getScbRigidStaticFast(), mRigidActors, this, bvhStructure, hasPrunerStructure);
}

void NpScene::addRigidDynamic(NpRigidDynamic& body, const Gu::BVHStructure* bvhStructure, bool hasPrunerStructure)
{
	addActorT(body, body.getScbBodyFast(), mRigidActors, this, bvhStructure, hasPrunerStructure);
}

///////////////////////////////////////////////////////////////////////////////

template<class T, class T2>
static PX_FORCE_INLINE void removeActorT(T& actor, T2& scbActor, NpScene* scene, bool wakeOnLostTouch, bool removeFromAggregate)
{
	PX_ASSERT(NpActor::getAPIScene(actor) == scene);
	const bool noSimBuffered = scbActor.getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION);

	if(removeFromAggregate)
	{
		PxU32 index = 0xffffffff;
		NpAggregate* aggregate = actor.getNpAggregate(index);
		if(aggregate)
		{
			aggregate->removeActorAndReinsert(actor, false);
			PX_ASSERT(!actor.getAggregate());
		}
	}

	actor.getShapeManager().teardownAllSceneQuery(scene->getSceneQueryManagerFast(), actor);
	if(!noSimBuffered)
		actor.removeConstraintsFromScene();

	scene->getScene().removeActor(scbActor, wakeOnLostTouch, scbActor.isSimDisabledInternally());
	scene->removeFromRigidActorList(actor.getRigidActorArrayIndex());
}

void NpScene::removeRigidStatic(NpRigidStatic& actor, bool wakeOnLostTouch, bool removeFromAggregate)
{
	removeActorT(actor, actor.getScbRigidStaticFast(), this, wakeOnLostTouch, removeFromAggregate);
}

void NpScene::removeRigidDynamic(NpRigidDynamic& body, bool wakeOnLostTouch, bool removeFromAggregate)
{
	removeActorT(body, body.getScbBodyFast(), this, wakeOnLostTouch, removeFromAggregate);
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::addArticulation(PxArticulationBase& articulation)
{
	PX_PROFILE_ZONE("API.addArticulation", getContextId());
	NP_WRITE_CHECK(this);

	PX_CHECK_AND_RETURN(articulation.getNbLinks()>0, "PxScene::addArticulation: empty articulations may not be added to simulation.");
	PX_SIMD_GUARD;

	if (this->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addArticulation(): Articulations are not currently supported when PxSceneFlag::eENABLE_GPU_DYNAMICS is set!");
		return;
	}

	if (getSimulationStage() != Sc::SimulationStage::eCOMPLETE && articulation.getType() == PxArticulationBase::eReducedCoordinate)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addArticulation(): this call is not allowed while the simulation is running. Call will be ignored!");
		return;
	}

	PxArticulationImpl& npa = *reinterpret_cast<PxArticulationImpl*>(articulation.getImpl());

	Scb::Articulation& art = npa.getArticulation();
	const Scb::ControlState::Enum cs = art.getControlState();
	if ((cs == Scb::ControlState::eNOT_IN_SCENE) || ((cs == Scb::ControlState::eREMOVE_PENDING) && (art.getScbScene()->getPxScene() == this)))
		addArticulationInternal(articulation);
	else
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addArticulation(): Articulation already assigned to a scene. Call will be ignored!");
}

static void checkArticulationLink(NpScene* scene, NpArticulationLink* link)
{
#if PX_CHECKED
	scene->checkPositionSanity(*link, link->getGlobalPose(), "PxScene::addArticulation or PxScene::addAggregate");
#else
	PX_UNUSED(scene);
#endif
	if(link->getMass()==0.0f)
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::addArticulation(): Articulation link with zero mass added to scene; defaulting mass to 1");
		link->setMass(1.0f);
	}

	const PxVec3 inertia0 = link->getMassSpaceInertiaTensor();
	if(inertia0.x == 0.0f || inertia0.y == 0.0f || inertia0.z == 0.0f)
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::addArticulation(): Articulation link with zero moment of inertia added to scene; defaulting inertia to (1,1,1)");
		link->setMassSpaceInertiaTensor(PxVec3(1.0f, 1.0f, 1.0f));
	}
}

void NpScene::addArticulationInternal(PxArticulationBase& npa)
{
	// Add root link first
	PxU32 nbLinks = npa.getNbLinks();
	PX_ASSERT(nbLinks > 0);
	PxArticulationImpl* impl = reinterpret_cast<PxArticulationImpl*>(npa.getImpl());
	NpArticulationLink* rootLink = static_cast<NpArticulationLink*>(impl->getRoot());

	checkArticulationLink(this, rootLink);

	bool linkTriggersWakeUp = !rootLink->getScbBodyFast().checkSleepReadinessBesidesWakeCounter();
	
	addArticulationLinkBody(*rootLink);

	// Add articulation
	PxArticulationImpl* npaImpl = reinterpret_cast<PxArticulationImpl*>(npa.getImpl());
	Scb::Articulation& scbArt = npaImpl->getArticulation();
	scbArt.setArticulationType(npa.getType());
	mScene.addArticulation(scbArt);

	Sc::ArticulationCore& scArtCore = scbArt.getScArticulation();
	Sc::ArticulationSim* scArtSim = scArtCore.getSim();

	if (scArtSim)
	{
		PxU32 handle = scArtSim->findBodyIndex(*rootLink->getScbBodyFast().getScBody().getSim());
		rootLink->setLLIndex(handle);
	}
	rootLink->setInboundJointDof(0);

	addArticulationLinkConstraint(*rootLink);
	
	// Add links & joints
	PX_ALLOCA(linkStack, NpArticulationLink*, nbLinks);
	linkStack[0] = rootLink;
	PxU32 curLink = 0;
	PxU32 stackSize = 1;
	while(curLink < (nbLinks-1))
	{
		PX_ASSERT(curLink < stackSize);
		NpArticulationLink* l = linkStack[curLink];
		NpArticulationLink*const* children = l->getChildren();

		for(PxU32 i=0; i < l->getNbChildren(); i++)
		{
			NpArticulationLink* child = children[i];

			checkArticulationLink(this, child);

			linkTriggersWakeUp = linkTriggersWakeUp || (!child->getScbBodyFast().checkSleepReadinessBesidesWakeCounter());

			addArticulationLink(*child);  // Adds joint too
			if (scArtSim)
			{
				PxU32 cHandle = scArtSim->findBodyIndex(*child->getScbBodyFast().getScBody().getSim());
				child->setLLIndex(cHandle);
			}
			//child->setInboundJointDof(scArtSim->getDof(cHandle));

			linkStack[stackSize] = child;
			stackSize++;
		}

		curLink++;
	}

	if ((scbArt.getWakeCounter() == 0.0f) && linkTriggersWakeUp)
	{
		// this is for the buffered insert case, where the articulation needs to wake up, if one of the links triggers activation.
		npaImpl->wakeUpInternal(true, false);
	}

	mArticulations.insert(&npa);

	if (scArtSim)
	{
		scArtSim->checkResize();

		linkStack[0] = rootLink;
		curLink = 0;
		stackSize = 1;

		while (curLink < (nbLinks - 1))
		{
			PX_ASSERT(curLink < stackSize);
			NpArticulationLink* l = linkStack[curLink];
			NpArticulationLink*const* children = l->getChildren();

			for (PxU32 i = 0; i < l->getNbChildren(); i++)
			{
				NpArticulationLink* child = children[i];

				child->setInboundJointDof(scArtSim->getDof(child->getLinkIndex()));

				if (npa.getType() == PxArticulationBase::eReducedCoordinate)
				{
					PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(child->getInboundJoint());
					PxArticulationJointType::Enum jointType = joint->getJointType();

					if (jointType == PxArticulationJointType::eUNDEFINED)
					{
						Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::addArticulation(): The application need to set joint type. defaulting joint type to eFix");
						joint->setJointType(PxArticulationJointType::eFIX);
						child->setInboundJointDof(0);
					}

					if (jointType != PxArticulationJointType::eFIX)
					{

						PxArticulationMotion::Enum motionX = joint->getMotion(PxArticulationAxis::eX);
						PxArticulationMotion::Enum motionY = joint->getMotion(PxArticulationAxis::eY);
						PxArticulationMotion::Enum motionZ = joint->getMotion(PxArticulationAxis::eZ);

						PxArticulationMotion::Enum motionSwing1 = joint->getMotion(PxArticulationAxis::eSWING1);
						PxArticulationMotion::Enum motionSwing2 = joint->getMotion(PxArticulationAxis::eSWING2);
						PxArticulationMotion::Enum motionTwist = joint->getMotion(PxArticulationAxis::eTWIST);

						//PxArticulationMotion::eLOCKED is 0 
						if (!(motionX | motionY | motionZ | motionSwing1 | motionSwing2 | motionTwist))
						{
							//if all axis are locked, which means the user doesn't set the motion. In this case, we should change the joint type to be
							//fix to avoid crash in the solver
							Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::addArticulation(): The application need to set joint motion. defaulting joint type to eFix");
							joint->setJointType(PxArticulationJointType::eFIX);
							child->setInboundJointDof(0);
						}
					}
				}

				linkStack[stackSize] = child;
				stackSize++;
			}

			curLink++;
		}
	}

	//add loop joints
	if (npa.getType() == PxArticulationBase::eReducedCoordinate)
	{
		//This method will prepare link data for the gpu 
		mScene.getScScene().addArticulationSimControl(scbArt.getScArticulation());

		NpArticulationReducedCoordinate* npaRC = static_cast<NpArticulationReducedCoordinate*>(&npa);

		for (PxU32 i = 0; i < npaRC->mLoopJoints.size(); ++i)
		{
			PxJoint* joint = npaRC->mLoopJoints[i];
			NpConstraint* constraint = static_cast<NpConstraint*>(joint->getConstraint());
			Sc::ConstraintSim* cSim = constraint->getScbConstraint().getScConstraint().getSim();
			scArtSim->addLoopConstraint(cSim);
		}
	}
}

void NpScene::removeArticulation(PxArticulationBase& articulation, bool wakeOnLostTouch)
{
	PX_PROFILE_ZONE("API.removeArticulation", getContextId());
	NP_WRITE_CHECK(this);

	if (removeFromSceneCheck(this, articulation.getScene(), "PxScene::removeArticulation(): Articulation"))
	{
		NpArticulation& npa = static_cast<NpArticulation&>(articulation);
		removeArticulationInternal(npa, wakeOnLostTouch, true);
	}
}

void NpScene::removeArticulationInternal(PxArticulationBase& npa, bool wakeOnLostTouch,  bool removeFromAggregate)
{
	PxU32 nbLinks = npa.getNbLinks();
	PX_ASSERT(nbLinks > 0);

	if(removeFromAggregate && npa.getAggregate())
	{
		static_cast<NpAggregate*>(npa.getAggregate())->removeArticulationAndReinsert(npa, false);
		PX_ASSERT(!npa.getAggregate());
	}

	//!!!AL
	// Inefficient. We might want to introduce a LL method to kill the whole LL articulation together with all joints in one go, then
	// the order of removing the links/joints does not matter anymore.

	// Remove links & joints
	PX_ALLOCA(linkStack, NpArticulationLink*, nbLinks);
	linkStack[0] = reinterpret_cast<PxArticulationImpl*>(npa.getImpl())->getLinks()[0];
	PxU32 curLink = 0, stackSize = 1;

	while(curLink < (nbLinks-1))
	{
		PX_ASSERT(curLink < stackSize);
		NpArticulationLink* l = linkStack[curLink];
		NpArticulationLink*const* children = l->getChildren();

		for(PxU32 i=0; i < l->getNbChildren(); i++)
		{
			linkStack[stackSize] = children[i];
			stackSize++;
		}

		curLink++;
	}

	PxRigidBodyFlags flag;
	for(PxI32 j=PxI32(nbLinks); j-- > 0; )
	{
		flag |=linkStack[j]->getScbBodyFast().getScBody().getCore().mFlags;
		removeArticulationLink(*linkStack[j], wakeOnLostTouch);
	}

	if (flag & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
	{
		PxArticulationImpl* impl = reinterpret_cast<PxArticulationImpl*>(npa.getImpl());
		IG::NodeIndex index = impl->getScbArticulation().getScArticulation().getIslandNodeIndex();
		if (index.isValid())
			mScene.getScScene().resetSpeculativeCCDArticulationLink(index.index());
	}
	// Remove articulation
	mScene.removeArticulation(reinterpret_cast<PxArticulationImpl*>(npa.getImpl())->getArticulation());


	removeFromArticulationList(npa);
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::addArticulationLinkBody(NpArticulationLink& link)
{
	mScene.addActor(link.getScbBodyFast(), false, NULL, NULL);
	link.getShapeManager().setupAllSceneQuery(this, link, false);
}

void NpScene::addArticulationLinkConstraint(NpArticulationLink& link)
{
	PxArticulationJointBase* j = link.getInboundJoint();
	if (j)
	{
		PxArticulationJointImpl* impl = j->getImpl();
		mScene.addArticulationJoint(impl->getScbArticulationJoint());
	}

	link.addConstraintsToScene();
}

void NpScene::addArticulationLink(NpArticulationLink& link)
{
	addArticulationLinkBody(link);
	addArticulationLinkConstraint(link);
}

void NpScene::removeArticulationLink(NpArticulationLink& link, bool wakeOnLostTouch)
{
	PxArticulationJointBase* j =link.getInboundJoint();

	link.removeConstraintsFromScene();
	link.getShapeManager().teardownAllSceneQuery(getSceneQueryManagerFast(), link);

	if (j)
		mScene.removeArticulationJoint(j->getImpl()->getScbArticulationJoint());

	mScene.removeActor(link.getScbBodyFast(), wakeOnLostTouch, false);
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::addAggregate(PxAggregate& aggregate)
{
	PX_PROFILE_ZONE("API.addAggregate", getContextId());
	NP_WRITE_CHECK(this);
	PX_SIMD_GUARD;

	NpAggregate& np = static_cast<NpAggregate&>(aggregate);

	const PxU32 nb = np.getCurrentSizeFast();
#if PX_CHECKED
	for(PxU32 i=0;i<nb;i++)
	{
		PxRigidStatic* a = np.getActorFast(i)->is<PxRigidStatic>();
		if(a && !static_cast<NpRigidStatic*>(a)->checkConstraintValidity())
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addAggregate(): Aggregate contains an actor with an invalid constraint!");
			return;
		}
	}	
#endif

	Scb::Aggregate& agg = np.getScbAggregate(); 
	const Scb::ControlState::Enum cs = agg.getControlState();
	if ((cs == Scb::ControlState::eNOT_IN_SCENE) || ((cs == Scb::ControlState::eREMOVE_PENDING) && (agg.getScbScene()->getPxScene() == this)))
	{
		mScene.addAggregate(agg);

		for(PxU32 i=0;i<nb;i++)
		{
			PX_ASSERT(np.getActorFast(i));
			PxActor& actor = *np.getActorFast(i);

			//A.B. check if a bvh structure was connected to that actor, we will use it for the insert and remove it
			NpActor& npActor = NpActor::getFromPxActor(actor);
			Gu::BVHStructure* bvhStructure = NULL;			
			if(npActor.getConnectors<Gu::BVHStructure>(NpConnectorType::eBvhStructure, &bvhStructure, 1))
			{
				npActor.removeConnector(actor, NpConnectorType::eBvhStructure, bvhStructure, "PxBVHStructure connector could not have been removed!");				
			}

			np.addActorInternal(actor, *this, bvhStructure);

			// if a bvh structure was used dec ref count, we increased the ref count when adding the actor connection
			if(bvhStructure)
			{
				bvhStructure->decRefCount();
			}
		}

		mAggregates.insert(&aggregate);
	}
	else
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addAggregate(): Aggregate already assigned to a scene. Call will be ignored!");
}

void NpScene::removeAggregate(PxAggregate& aggregate, bool wakeOnLostTouch)
{
	PX_PROFILE_ZONE("API.removeAggregate", getContextId());
	NP_WRITE_CHECK(this);	
	if(!removeFromSceneCheck(this, aggregate.getScene(), "PxScene::removeAggregate(): Aggregate"))
		return;

	NpAggregate& np = static_cast<NpAggregate&>(aggregate);
	if(np.getScene()!=this)
		return;

	const PxU32 nb = np.getCurrentSizeFast();
	for(PxU32 j=0;j<nb;j++)
	{
		PxActor* a = np.getActorFast(j);
		PX_ASSERT(a);

		if (a->getType() != PxActorType::eARTICULATION_LINK)
		{
			Scb::Actor& scb = NpActor::getScbFromPxActor(*a);

			np.getScbAggregate().removeActor(scb, false);  // This is only here to make sure the aggregateID gets set to invalid on sync

			removeActorInternal(*a, wakeOnLostTouch, false);
		}
		else if (a->getScene())
		{
			NpArticulationLink& al = static_cast<NpArticulationLink&>(*a);
			PxArticulationBase& npArt = al.getRoot();
			PxArticulationImpl* impl = reinterpret_cast<PxArticulationImpl*>(npArt.getImpl());
			NpArticulationLink* const* links = impl->getLinks();
			for(PxU32 i=0; i < npArt.getNbLinks(); i++)
			{
				np.getScbAggregate().removeActor(links[i]->getScbActorFast(), false);  // This is only here to make sure the aggregateID gets set to invalid on sync
			}

			removeArticulationInternal(npArt, wakeOnLostTouch, false);
		}
	}

	mScene.removeAggregate(np.getScbAggregate());

	removeFromAggregateList(aggregate);
}

PxU32 NpScene::getNbAggregates() const
{
	NP_READ_CHECK(this);
	return mAggregates.size();
}

PxU32 NpScene::getAggregates(PxAggregate** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mAggregates.getEntries(), mAggregates.size());
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::addCollection(const PxCollection& collection)
{
	PX_PROFILE_ZONE("API.addCollection", getContextId());
	const Cm::Collection& col = static_cast<const Cm::Collection&>(collection);

	PxU32 nb = col.internalGetNbObjects();
#if PX_CHECKED
	for(PxU32 i=0;i<nb;i++)
	{
		PxRigidStatic* a = col.internalGetObject(i)->is<PxRigidStatic>();
		if(a && !static_cast<NpRigidStatic*>(a)->checkConstraintValidity())
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::addCollection(): collection contains an actor with an invalid constraint!");
			return;
		}
	}	
#endif

	Ps::Array<PxActor*> actorsToInsert;
	actorsToInsert.reserve(nb);

	struct Local
	{
		static void addActorIfNeeded(PxActor* actor, Ps::Array<PxActor*>& actorArray)
		{
			if(actor->getAggregate())
				return;	// The actor will be added when the aggregate is added
			actorArray.pushBack(actor);			
		}
	};

	for(PxU32 i=0;i<nb;i++)
	{
		PxBase* s = col.internalGetObject(i);
		const PxType serialType = s->getConcreteType();

		//NpArticulationLink, NpArticulationJoint are added with the NpArticulation
		//Actors and Articulations that are members of an Aggregate are added with the NpAggregate

		if(serialType==PxConcreteType::eRIGID_DYNAMIC)
		{
			NpRigidDynamic* np = static_cast<NpRigidDynamic*>(s);
			// if pruner structure exists for the actor, actor will be added with the pruner structure
			if(!np->getShapeManager().getPruningStructure())
				Local::addActorIfNeeded(np, actorsToInsert);
		}
		else if(serialType==PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic* np = static_cast<NpRigidStatic*>(s);
			// if pruner structure exists for the actor, actor will be added with the pruner structure
			if(!np->getShapeManager().getPruningStructure())
				Local::addActorIfNeeded(np, actorsToInsert);
		}
		else if(serialType==PxConcreteType::eSHAPE)
		{			
		}
		else if(serialType==PxConcreteType::eARTICULATION)
		{
			NpArticulation* np = static_cast<NpArticulation*>(s);
			if (!np->getAggregate()) // The actor will be added when the aggregate is added
			{
				if (np->mType == PxArticulationBase::eMaximumCoordinate)
					addArticulation(static_cast<PxArticulation&>(*np));
				/*else
					addArticulation(static_cast<PxArticulationReducedCoordinate&>(*np));*/
			}
		}
		else if(serialType==PxConcreteType::eAGGREGATE)
		{
			NpAggregate* np = static_cast<NpAggregate*>(s);
			addAggregate(*np);
		}
		else if(serialType == PxConcreteType::ePRUNING_STRUCTURE)
		{
			PxPruningStructure* ps = static_cast<PxPruningStructure*>(s);
			addActors(*ps);
		}
	}

	if(!actorsToInsert.empty())
		addActorsInternal(&actorsToInsert[0], actorsToInsert.size(), NULL);
}

///////////////////////////////////////////////////////////////////////////////

PxU32 NpScene::getNbActors(PxActorTypeFlags types) const
{
	NP_READ_CHECK(this);
	PxU32 nbActors = 0;

	if (types & PxActorTypeFlag::eRIGID_STATIC)
	{
		for(PxU32 i=mRigidActors.size(); i--;)
		{
			if (mRigidActors[i]->is<PxRigidStatic>())
				nbActors++;
		}
	}

	if (types & PxActorTypeFlag::eRIGID_DYNAMIC)
	{
		for(PxU32 i=mRigidActors.size(); i--;)
		{
			if (mRigidActors[i]->is<PxRigidDynamic>())
				nbActors++;
		}
	}

	return nbActors;
}

PxU32 NpScene::getActors(PxActorTypeFlags types, PxActor** buffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);

	PxU32 writeCount = 0;
	PxU32 virtualIndex = 0;	// PT: virtual index of actor, continuous across different actor containers.

	if(types & (PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC))
	{
		const PxU32 size = mRigidActors.size();
		for(PxU32 i=0; (i < size) && (writeCount < bufferSize); i++)
		{
			if ((types & PxActorTypeFlag::eRIGID_STATIC ) && mRigidActors[i]->is<PxRigidStatic>())
			{
				if (virtualIndex >= startIndex)
					buffer[writeCount++] = mRigidActors[i];
				virtualIndex++;
			}
			else if ((types & PxActorTypeFlag::eRIGID_DYNAMIC) && mRigidActors[i]->is<PxRigidDynamic>())
			{
				if (virtualIndex >= startIndex)
					buffer[writeCount++] = mRigidActors[i];
				virtualIndex++;
			}
		}
	}

	return writeCount;
}

///////////////////////////////////////////////////////////////////////////////

PxActor** NpScene::getActiveActors(PxU32& nbActorsOut)
{
	NP_READ_CHECK(this);
	return mScene.getActiveActors(nbActorsOut);
}

PxActor** NpScene::getFrozenActors(PxU32& nbActorsOut)
{
	NP_READ_CHECK(this);
	return mScene.getFrozenActors(nbActorsOut);
}

void NpScene::setFrozenActorFlag(const bool buildFrozenActors)
{
#if PX_CHECKED
	PxSceneFlags combinedFlag(PxSceneFlag::eENABLE_ACTIVE_ACTORS | PxSceneFlag::eENABLE_STABILIZATION);

	PX_CHECK_AND_RETURN((getFlags() & combinedFlag)== combinedFlag,
		"NpScene::setFrozenActorFlag: Cannot raise BuildFrozenActors if PxSceneFlag::eENABLE_STABILIZATION and PxSceneFlag::eENABLE_ACTIVE_ACTORS is not raised!");
#endif
	mBuildFrozenActors = buildFrozenActors;
}

///////////////////////////////////////////////////////////////////////////////

PxU32 NpScene::getNbArticulations() const
{
	NP_READ_CHECK(this);
	return mArticulations.size();
}

PxU32 NpScene::getArticulations(PxArticulationBase** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mArticulations.getEntries(), mArticulations.size());
}

///////////////////////////////////////////////////////////////////////////////

PxU32 NpScene::getNbConstraints() const
{
	NP_READ_CHECK(this);
	return mConstraints.size();
}

PxU32 NpScene::getConstraints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mConstraints.getEntries(), mConstraints.size());
}

///////////////////////////////////////////////////////////////////////////////

const PxRenderBuffer& NpScene::getRenderBuffer()
{
	if (getSimulationStage() != Sc::SimulationStage::eCOMPLETE) 
	{
		// will be reading the Sc::Scene renderable which is getting written 
		// during the sim, hence, avoid call while simulation is running.
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, 
			"PxScene::getRenderBuffer() not allowed while simulation is running.");
	}

	return mRenderBuffer;
}

void NpScene::visualize()
{
	NP_READ_CHECK(this);

	PX_PROFILE_ZONE("NpScene::visualize", getContextId());

	mRenderBuffer.clear(); // clear last frame visualizations 

#if PX_ENABLE_DEBUG_VISUALIZATION
	if(getVisualizationParameter(PxVisualizationParameter::eSCALE) == 0.0f)
		return;

	Cm::RenderOutput out(mRenderBuffer);

	// Visualize scene axis
	const PxReal worldAxes = getVisualizationParameter(PxVisualizationParameter::eWORLD_AXES);
	if (worldAxes != 0)
		out << Cm::DebugBasis(PxVec3(worldAxes));

	// Visualize articulations
	for(PxU32 i=0;i<mArticulations.size();i++)
		static_cast<NpArticulation *>(mArticulations.getEntries()[i])->visualize(out, this);

	// Visualize rigid actors and rigid bodies
	PxRigidActor*const* rigidActors = mRigidActors.begin();
	const PxU32 rigidActorCount = mRigidActors.size();

	for(PxU32 i=0; i < rigidActorCount; i++)
	{
		PxRigidActor* a = rigidActors[i];
		if (a->getType() == PxActorType::eRIGID_DYNAMIC)
			static_cast<NpRigidDynamic*>(a)->visualize(out, this);
		else
			static_cast<NpRigidStatic*>(a)->visualize(out, this);
	}

	// Visualize pruning structures
	const bool visStatic = getVisualizationParameter(PxVisualizationParameter::eCOLLISION_STATIC) != 0.0f;
	const bool visDynamic = getVisualizationParameter(PxVisualizationParameter::eCOLLISION_DYNAMIC) != 0.0f;
	//flushQueryUpdates(); // DE7834
	if(visStatic && mSQManager.get(PruningIndex::eSTATIC).pruner())
		mSQManager.get(PruningIndex::eSTATIC).pruner()->visualize(out, PxU32(PxDebugColor::eARGB_BLUE));
	if(visDynamic && mSQManager.get(PruningIndex::eDYNAMIC).pruner())
		mSQManager.get(PruningIndex::eDYNAMIC).pruner()->visualize(out, PxU32(PxDebugColor::eARGB_RED));

	if(getVisualizationParameter(PxVisualizationParameter::eMBP_REGIONS) != 0.0f)
	{
		out << PxTransform(PxIdentity);

		const PxU32 nbRegions = mScene.getNbBroadPhaseRegions();
		for(PxU32 i=0;i<nbRegions;i++)
		{
			PxBroadPhaseRegionInfo info;
			mScene.getBroadPhaseRegions(&info, 1, i);

			if(info.active)
				out << PxU32(PxDebugColor::eARGB_YELLOW);
			else
				out << PxU32(PxDebugColor::eARGB_BLACK);
			out << Cm::DebugBox(info.region.bounds);
		}
	}

	if(getVisualizationParameter(PxVisualizationParameter::eCULL_BOX)!=0.0f)
	{
		const PxBounds3& cullbox = getScene().getVisualizationCullingBox();
		if(!cullbox.isEmpty())
		{
			out << PxU32(PxDebugColor::eARGB_YELLOW);
			out << Cm::DebugBox(cullbox);
		}
	}

#if PX_SUPPORT_PVD
	mScene.getScenePvdClient().visualize(mRenderBuffer);
#endif
#endif
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::getSimulationStatistics(PxSimulationStatistics& s) const
{
	NP_READ_CHECK(this);

	if (getSimulationStage() == Sc::SimulationStage::eCOMPLETE)
	{
#if PX_ENABLE_SIM_STATS
		mScene.getStats(s);
#else
		PX_UNUSED(s);
#endif
	}
	else
	{
		//will be reading data that is getting written during the sim, hence, avoid call while simulation is running.
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::getSimulationStatistics() not allowed while simulation is running. Call will be ignored.");
	}
}

///////////////////////////////////////////////////////////////////////////////

//Multiclient 

PxClientID NpScene::createClient()
{
	NP_WRITE_CHECK(this);

	PX_CHECK_AND_RETURN_NULL(mNbClients < PX_MAX_CLIENTS, "PxScene::createClient: Maximum number of clients reached! No new client created.");
	mNbClients++;		//track this just for error checking 
	return mScene.createClient();
}

///////////////////////////////////////////////////////////////////////////////

//FrictionModel 

void NpScene::setFrictionType(PxFrictionType::Enum frictionType)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN(!mHasSimulatedOnce, "PxScene::setFrictionType: This flag can only be set before calling Simulate() or Collide() for the first time");
	mScene.setFrictionType(frictionType);
}

PxFrictionType::Enum NpScene::getFrictionType() const
{
	NP_READ_CHECK(this);
	return mScene.getFrictionType();
}

///////////////////////////////////////////////////////////////////////////////

// Callbacks

void NpScene::setSimulationEventCallback(PxSimulationEventCallback* callback)
{
	NP_WRITE_CHECK(this);
	mScene.setSimulationEventCallback(callback);
}

PxSimulationEventCallback* NpScene::getSimulationEventCallback() const
{
	NP_READ_CHECK(this);
	return mScene.getSimulationEventCallback();
}

void NpScene::setContactModifyCallback(PxContactModifyCallback* callback)
{
	NP_WRITE_CHECK(this);
	mScene.setContactModifyCallback(callback);
}

PxContactModifyCallback* NpScene::getContactModifyCallback() const
{
	NP_READ_CHECK(this);
	return mScene.getContactModifyCallback();
}

void NpScene::setCCDContactModifyCallback(PxCCDContactModifyCallback* callback)
{
	NP_WRITE_CHECK(this);
	mScene.setCCDContactModifyCallback(callback);
}

PxCCDContactModifyCallback* NpScene::getCCDContactModifyCallback() const
{
	NP_READ_CHECK(this);
	return mScene.getCCDContactModifyCallback();
}

void NpScene::setBroadPhaseCallback(PxBroadPhaseCallback* callback)
{
	NP_WRITE_CHECK(this);
	mScene.setBroadPhaseCallback(callback);
}

PxBroadPhaseCallback* NpScene::getBroadPhaseCallback() const
{
	NP_READ_CHECK(this);
	return mScene.getBroadPhaseCallback();
}

void NpScene::setCCDMaxPasses(PxU32 ccdMaxPasses)
{
	NP_WRITE_CHECK(this);
	mScene.setCCDMaxPasses(ccdMaxPasses);
}

PxU32 NpScene::getCCDMaxPasses() const
{
	NP_READ_CHECK(this);
	return mScene.getCCDMaxPasses();
}

PxBroadPhaseType::Enum NpScene::getBroadPhaseType() const
{
	NP_READ_CHECK(this);
	return mScene.getBroadPhaseType();
}

bool NpScene::getBroadPhaseCaps(PxBroadPhaseCaps& caps) const
{
	NP_READ_CHECK(this);
	return mScene.getBroadPhaseCaps(caps);
}

PxU32 NpScene::getNbBroadPhaseRegions() const
{
	NP_READ_CHECK(this);
	return mScene.getNbBroadPhaseRegions();
}

PxU32 NpScene::getBroadPhaseRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(this);
	return mScene.getBroadPhaseRegions(userBuffer, bufferSize, startIndex);
}

PxU32 NpScene::addBroadPhaseRegion(const PxBroadPhaseRegion& region, bool populateRegion)
{
	PX_PROFILE_ZONE("BroadPhase.addBroadPhaseRegion", getContextId());

	NP_WRITE_CHECK(this);

	PX_CHECK_MSG(region.bounds.isValid(), "PxScene::addBroadPhaseRegion(): invalid bounds provided!");
	if(region.bounds.isEmpty())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxScene::addBroadPhaseRegion(): region bounds are empty. Call will be ignored.");
		return 0xffffffff;
	}

	return mScene.addBroadPhaseRegion(region, populateRegion);
}

bool NpScene::removeBroadPhaseRegion(PxU32 handle)
{
	NP_WRITE_CHECK(this);
	return mScene.removeBroadPhaseRegion(handle);
}

///////////////////////////////////////////////////////////////////////////////

// Filtering
void NpScene::setFilterShaderData(const void* data, PxU32 dataSize)
{
	NP_WRITE_CHECK(this);

	PX_CHECK_AND_RETURN((	((dataSize == 0) && (data == NULL)) ||
							((dataSize > 0) && (data != NULL)) ), "PxScene::setFilterShaderData(): data pointer must not be NULL unless the specified data size is 0 too and vice versa.");

	mScene.setFilterShaderData(data, dataSize);
}

const void*	NpScene::getFilterShaderData() const
{
	NP_READ_CHECK(this);
	return mScene.getFilterShaderData();
}

PxU32 NpScene::getFilterShaderDataSize() const
{
	NP_READ_CHECK(this);
	return mScene.getFilterShaderDataSize();
}

PxSimulationFilterShader NpScene::getFilterShader() const
{
	NP_READ_CHECK(this);
	return mScene.getFilterShader();
}

PxSimulationFilterCallback*	NpScene::getFilterCallback() const
{
	NP_READ_CHECK(this);
	return mScene.getFilterCallback();
}

void NpScene::resetFiltering(PxActor& actor)
{
	NP_WRITE_CHECK(this);

	PX_CHECK_AND_RETURN(NpActor::getAPIScene(actor) && (NpActor::getAPIScene(actor) == this), "PxScene::resetFiltering(): actor not in scene!");

	switch(actor.getConcreteType())
	{
		case PxConcreteType::eRIGID_STATIC:
		{
			NpRigidStatic& npStatic = static_cast<NpRigidStatic&>(actor);
			npStatic.resetFiltering(npStatic.getScbRigidStaticFast(), NULL, 0);
		}
		break;

		case PxConcreteType::eRIGID_DYNAMIC:
		{
			NpRigidDynamic& npDynamic = static_cast<NpRigidDynamic&>(actor);
			if (npDynamic.resetFiltering(npDynamic.getScbBodyFast(), NULL, 0))
				npDynamic.wakeUpInternal();
		}
		break;

		case PxConcreteType::eARTICULATION_LINK:
		{
			NpArticulationLink& npLink = static_cast<NpArticulationLink&>(actor);
			if (npLink.resetFiltering(npLink.getScbBodyFast(), NULL, 0))
			{
				PxArticulationImpl* impl = reinterpret_cast<PxArticulationImpl*>(npLink.getRoot().getImpl());
				impl->wakeUpInternal(false, true);
			}
		}
		break;

		default:
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxScene::resetFiltering(): only PxRigidActor supports this operation!");
	}
}

void NpScene::resetFiltering(PxRigidActor& actor, PxShape*const* shapes, PxU32 shapeCount)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN(NpActor::getAPIScene(actor) && (NpActor::getAPIScene(actor) == this), "PxScene::resetFiltering(): actor not in scene!");
	PX_SIMD_GUARD;

	switch(actor.getConcreteType())
	{
		case PxConcreteType::eRIGID_STATIC:
		{
			NpRigidStatic& npStatic = static_cast<NpRigidStatic&>(actor);
			npStatic.resetFiltering(npStatic.getScbRigidStaticFast(), shapes, shapeCount);
		}
		break;

		case PxConcreteType::eRIGID_DYNAMIC:
		{
			NpRigidDynamic& npDynamic = static_cast<NpRigidDynamic&>(actor);
			if (npDynamic.resetFiltering(npDynamic.getScbBodyFast(), shapes, shapeCount))
				npDynamic.wakeUpInternal();
		}
		break;

		case PxConcreteType::eARTICULATION_LINK:
		{
			NpArticulationLink& npLink = static_cast<NpArticulationLink&>(actor);
			if (npLink.resetFiltering(npLink.getScbBodyFast(), shapes, shapeCount))
			{
				PxArticulationImpl* impl = reinterpret_cast<PxArticulationImpl*>(npLink.getRoot().getImpl());
				impl->wakeUpInternal(false, true);
			}
		}
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////

PxPhysics& NpScene::getPhysics()
{
	return NpPhysics::getInstance();
}

void NpScene::updateDirtyShaders()
{
	PX_PROFILE_ZONE("Sim.updateDirtyShaders", getContextId());
	// this should continue to be done in the Np layer even after SC has taken over
	// all vital simulation functions, because it needs to complete before simulate()
	// returns to the application

	// However, the implementation needs fixing so that it does work proportional to
	// the number of dirty shaders

	PxConstraint*const* constraints = mConstraints.getEntries();
	for(PxU32 i=0;i<mConstraints.size();i++)
	{
		static_cast<NpConstraint*>(constraints[i])->updateConstants();
	}
}

///////////////////////////////////////////////////////////////////////////////
void NpScene::simulateOrCollide(PxReal elapsedTime, physx::PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation, const char* invalidCallMsg, Sc::SimulationStage::Enum simStage)
{
	PX_SIMD_GUARD;

	{
		// write guard must end before simulation kicks off worker threads
		// otherwise the simulation callbacks could overlap with this function
		// and perform API reads,triggering an error
		NP_WRITE_CHECK(this);

		PX_PROFILE_START_CROSSTHREAD("Basic.simulate", getContextId());

		if(getSimulationStage() != Sc::SimulationStage::eCOMPLETE)
		{
			//fetchResult doesn't get called
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, invalidCallMsg);
			return;
		}

		PX_CHECK_AND_RETURN(elapsedTime > 0, "PxScene::collide/simulate: The elapsed time must be positive!");

		PX_CHECK_AND_RETURN((reinterpret_cast<size_t>(scratchBlock)&15) == 0, "PxScene::simulate: scratch block must be 16-byte aligned!");
	
		PX_CHECK_AND_RETURN((scratchBlockSize&16383) == 0, "PxScene::simulate: scratch block size must be a multiple of 16K");
	
#if PX_SUPPORT_PVD		
		//signal the frame is starting.	
		mScene.getScenePvdClient().frameStart(elapsedTime);
#endif

#if PX_ENABLE_DEBUG_VISUALIZATION
		visualize();
#endif

		updateDirtyShaders();

#if PX_SUPPORT_PVD
		mScene.getScenePvdClient().updateJoints();			
#endif

		mScene.getScScene().setScratchBlock(scratchBlock, scratchBlockSize);

		mElapsedTime = elapsedTime;
		if (simStage == Sc::SimulationStage::eCOLLIDE)
			mScene.getScScene().setElapsedTime(elapsedTime);

		mControllingSimulation = controlSimulation;

		//sync all the material events
		NpPhysics& physics = static_cast<NpPhysics&>(this->getPhysics());
		NpMaterialManager& manager = physics.getMaterialManager();
		NpMaterial** materials = manager.getMaterials();
		mScene.updateLowLevelMaterial(materials);

		setSimulationStage(simStage);
		mScene.setPhysicsBuffering(true);
		mHasSimulatedOnce = true;
	}

	{
		PX_PROFILE_ZONE("Sim.taskFrameworkSetup", getContextId());

		if (controlSimulation)
		{
			{
				PX_PROFILE_ZONE("Sim.resetDependencies", getContextId());
				// Only reset dependencies, etc if we own the TaskManager. Will be false
				// when an NpScene is controlled by an APEX scene.
				mTaskManager->resetDependencies();
			}
			mTaskManager->startSimulation();
		}

		if (simStage == Sc::SimulationStage::eCOLLIDE)
		{
			mCollisionCompletion.setContinuation(*mTaskManager, completionTask);
			mSceneCollide.setContinuation(&mCollisionCompletion);
			//Initialize scene completion task
			mSceneCompletion.setContinuation(*mTaskManager, NULL);

			mCollisionCompletion.removeReference();
			mSceneCollide.removeReference();
		}
		else
		{
			mSceneCompletion.setContinuation(*mTaskManager, completionTask);
			mSceneExecution.setContinuation(*mTaskManager, &mSceneCompletion);

			mSceneCompletion.removeReference();
			mSceneExecution.removeReference();
		}
	}
}

void NpScene::simulate(PxReal elapsedTime, physx::PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation)
{
	simulateOrCollide(	elapsedTime, completionTask, scratchBlock, scratchBlockSize, controlSimulation, 
						"PxScene::simulate: Simulation is still processing last simulate call, you should call fetchResults()!", Sc::SimulationStage::eADVANCE);
}

void NpScene::advance( physx::PxBaseTask* completionTask)
{
	NP_WRITE_CHECK(this);
	//issue error if advance() doesn't get called between fetchCollision() and fetchResult()
	if(getSimulationStage() != Sc::SimulationStage::eFETCHCOLLIDE)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::advance: advance() called illegally! advance() needed to be called after fetchCollision() and before fetchResult()!!");
		return;
	}

	//apply buffering for forces, velocities, kinematic targets and wake-up events
	mScene.syncWriteThroughProperties();

	//if mSimulateStage == eFETCHCOLLIDE, which means collide() has been kicked off and finished running, we can run advance() safely
	{
		//change the mSimulateStaget to eADVANCE to indicate the next stage to run is fetchResult()
		setSimulationStage(Sc::SimulationStage::eADVANCE);

		{
			PX_PROFILE_ZONE("Sim.taskFrameworkSetup", getContextId());

			mSceneCompletion.setDependent(completionTask);
			mSceneAdvance.setContinuation(*mTaskManager, &mSceneCompletion);
			mSceneCompletion.removeReference();
			mSceneAdvance.removeReference();
		}
	}
}

void NpScene::collide(PxReal elapsedTime, physx::PxBaseTask* completionTask, void* scratchBlock, PxU32 scratchBlockSize, bool controlSimulation)
{
	simulateOrCollide(	elapsedTime, 
						completionTask,
						scratchBlock,
						scratchBlockSize,
						controlSimulation,
						"PxScene::collide: collide() called illegally! If it isn't the first frame, collide() needed to be called between fetchResults() and fetchCollision(). Otherwise, collide() needed to be called before fetchCollision()", 
						Sc::SimulationStage::eCOLLIDE);
}

bool NpScene::checkResultsInternal(bool block)
{
	PX_PROFILE_ZONE("Basic.checkResults", getContextId());
	return mPhysicsDone.wait(block ? Ps::Sync::waitForever : 0);
}

bool NpScene::checkCollisionInternal(bool block)
{
	PX_PROFILE_ZONE("Basic.checkCollision", getContextId());
	return mCollisionDone.wait(block ? Ps::Sync::waitForever : 0);
}

bool NpScene::checkResults(bool block)
{
	return checkResultsInternal(block);
}

bool NpScene::checkCollision(bool block)
{
	return checkCollisionInternal(block);
}

void NpScene::fireOutOfBoundsCallbacks()
{
	PX_PROFILE_ZONE("Sim.fireOutOfBoundsCallbacks", getContextId());

	// Fire broad-phase callbacks
	{
		Sc::Scene& scene = mScene.getScScene();
		using namespace physx::Sc;

		bool outputWarning = scene.fireOutOfBoundsCallbacks();

		// Aggregates
		{
			void** outAgg = scene.getOutOfBoundsAggregates();
			const PxU32 nbOut1 = scene.getNbOutOfBoundsAggregates();

			PxBroadPhaseCallback* cb = scene.getBroadPhaseCallback();

			for(PxU32 i=0;i<nbOut1;i++)
			{
				PxAggregate* px = reinterpret_cast<PxAggregate*>(outAgg[i]);
				NpAggregate* np = static_cast<NpAggregate*>(px);
				if(np->getScbAggregate().getControlState()==Scb::ControlState::eREMOVE_PENDING)
					continue;

				if(cb)
					cb->onObjectOutOfBounds(*px);
				else
					outputWarning = true;
			}
			scene.clearOutOfBoundsAggregates();
		}

		if(outputWarning)
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "At least one object is out of the broadphase bounds. To manage those objects, define a PxBroadPhaseCallback for each used client.");
	}
}

bool NpScene::fetchCollision(bool block)
{
	if(getSimulationStage() != Sc::SimulationStage::eCOLLIDE)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::fetchCollision: fetchCollision() should be called after collide() and before advance()!");
		return false;
	}

	//if collision isn't finish running (and block is false), then return false
	if(!checkCollisionInternal(block))
		return false;

	// take write check *after* collision() finished, otherwise 
	// we will block fetchCollision() from using the API
	NP_WRITE_CHECK_NOREENTRY(this);

	setSimulationStage(Sc::SimulationStage::eFETCHCOLLIDE);

	return true;
}

class SqRefFinder: public Sc::SqRefFinder
{
public:
	virtual	Sq::PrunerHandle find(const PxRigidBody* body, const PxShape* shape)
	{		
		const Sq::PrunerData prunerdata = NpActor::getShapeManager(*body)->findSceneQueryData(*static_cast<const NpShape*>(shape));
		return Sq::getPrunerHandle(prunerdata);
	}
private:
};

// The order of the following operations is important!
// 1. Process object deletions which were carried out while the simulation was running (since these effect contact and trigger reports)
// 2. Write contact reports to global stream (taking pending deletions into account), clear some simulation buffers (deleted objects etc.), ...
// 3. Send reports which have to be done before the data is synced (contact & trigger reports etc.) such that the user gets the old state.
// 4. Mark the simulation as not running internally to allow reading data which should not be read otherwise
// 5. Synchronize the simulation and user state
// 6. Fire callbacks which need to reflect the synchronized object state

void NpScene::fetchResultsPreContactCallbacks()
{
#if PX_SUPPORT_PVD	
	mScene.getScenePvdClient().updateContacts();
#endif

	mScene.prepareOutOfBoundsCallbacks();
	mScene.processPendingRemove();
	mScene.endSimulation();

	{
		PX_PROFILE_ZONE("Sim.fireCallbacksPreSync", getContextId());
		fireOutOfBoundsCallbacks();		// fire out-of-bounds callbacks
		mScene.fireBrokenConstraintCallbacks();
		mScene.fireTriggerCallbacks();
	}
}

void NpScene::fetchResultsPostContactCallbacks()
{
	mScene.postCallbacksPreSync();
	mScene.syncEntireScene();	// double buffering

	SqRefFinder sqRefFinder;
	mScene.getScScene().syncSceneQueryBounds(mSQManager.getDynamicBoundsSync(), sqRefFinder);

	mSQManager.updateCompoundActors(mScene.getScScene().getActiveCompoundBodiesArray(), mScene.getScScene().getNumActiveCompoundBodies());
	mSQManager.afterSync(getSceneQueryUpdateModeFast());

#if PX_SUPPORT_PVD
	mScene.getScenePvdClient().updateSceneQueries();

	getSingleSqCollector().clear();
	getBatchedSqCollector().clear();
#endif

	// fire sleep and wake-up events
	// we do this after buffer-swapping so that the events have the new state
	{
		PX_PROFILE_ZONE("Sim.fireCallbacksPostSync", getContextId());
		mScene.fireCallBacksPostSync();
	}

	mScene.postReportsCleanup();

	// build the list of active actors
	{
		PX_PROFILE_ZONE("Sim.buildActiveActors", getContextId());

		const bool buildActiveActors = mScene.getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS;
		
		if (buildActiveActors && mBuildFrozenActors)
			mScene.buildActiveAndFrozenActors();
		else if (buildActiveActors)
			mScene.buildActiveActors();
	}

	mRenderBuffer.append(mScene.getScScene().getRenderBuffer());

	PX_ASSERT(getSimulationStage() != Sc::SimulationStage::eCOMPLETE);
	if (mControllingSimulation)
	{
		mTaskManager->stopSimulation();
	}

	setSimulationStage(Sc::SimulationStage::eCOMPLETE);

	mPhysicsDone.reset();				// allow Physics to run again
	mCollisionDone.reset();
}

bool NpScene::fetchResults(bool block, PxU32* errorState)
{
	if(getSimulationStage() != Sc::SimulationStage::eADVANCE)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::fetchResults: fetchResults() called illegally! It must be called after advance() or simulate()");
		return false;
	}	

	if(!checkResultsInternal(block))
		return false;

	{
		PX_SIMD_GUARD;

		// take write check *after* simulation has finished, otherwise 
		// we will block simulation callbacks from using the API
		// disallow re-entry to detect callbacks making write calls
		NP_WRITE_CHECK_NOREENTRY(this);

		// we use cross thread profile here, to show the event in cross thread view
		// PT: TODO: why do we want to show it in the cross thread view?
		PX_PROFILE_START_CROSSTHREAD("Basic.fetchResults", getContextId());
		PX_PROFILE_ZONE("Sim.fetchResults", getContextId());

		fetchResultsPreContactCallbacks();

		{
			// PT: TODO: why a cross-thread event here?
			PX_PROFILE_START_CROSSTHREAD("Basic.processCallbacks", getContextId());
			mScene.fireQueuedContactCallbacks();
			PX_PROFILE_STOP_CROSSTHREAD("Basic.processCallbacks", getContextId());
		}

		fetchResultsPostContactCallbacks();
	
		PX_PROFILE_STOP_CROSSTHREAD("Basic.fetchResults", getContextId());
		PX_PROFILE_STOP_CROSSTHREAD("Basic.simulate", getContextId());

		if(errorState)
			*errorState = 0;
	}

#if PX_SUPPORT_PVD
	{
		PX_SIMD_GUARD;
		mScene.getScenePvdClient().frameEnd();
	}
#endif
	return true;
}

bool NpScene::fetchResultsStart(const PxContactPairHeader*& contactPairs, PxU32& nbContactPairs, bool block)
{
	if (getSimulationStage() != Sc::SimulationStage::eADVANCE)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PXScene::fetchResultsStart: fetchResultsStart() called illegally! It must be called after advance() or simulate()");
		return false;
	}

	if (!checkResultsInternal(block))
		return false;

	PX_SIMD_GUARD;
	NP_WRITE_CHECK(this);

	// we use cross thread profile here, to show the event in cross thread view
	PX_PROFILE_START_CROSSTHREAD("Basic.fetchResults", getContextId());
	PX_PROFILE_ZONE("Sim.fetchResultsStart", getContextId());

	fetchResultsPreContactCallbacks();
	const Ps::Array<PxContactPairHeader>& pairs = mScene.getQueuedContactPairHeaders();
	nbContactPairs = pairs.size();
	contactPairs = pairs.begin();

	mBetweenFetchResults = true;
	return true;
}

void NpContactCallbackTask::setData(NpScene* scene, const PxContactPairHeader* contactPairHeaders, const uint32_t nbContactPairHeaders)
{
	mScene = scene;
	mContactPairHeaders = contactPairHeaders;
	mNbContactPairHeaders = nbContactPairHeaders;
}

void NpContactCallbackTask::run()
{
	physx::PxSimulationEventCallback* callback = mScene->getSimulationEventCallback();
	if(!callback)
		return;

	mScene->lockRead();
	for(uint32_t i=0; i<mNbContactPairHeaders; ++i)
	{
		const physx::PxContactPairHeader& pairHeader = mContactPairHeaders[i];
		callback->onContact(pairHeader, pairHeader.pairs, pairHeader.nbPairs);
	}
	mScene->unlockRead();
}

void NpScene::processCallbacks(physx::PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.processCallbacks", getContextId());
	PX_PROFILE_ZONE("Sim.processCallbacks", getContextId());
	//ML: because Apex destruction callback isn't thread safe so that we make this run single thread first
	const Ps::Array<PxContactPairHeader>& pairs = mScene.getQueuedContactPairHeaders();
	const PxU32 nbPairs = pairs.size();
	const PxContactPairHeader* contactPairs = pairs.begin();
	const PxU32 nbToProcess = 256;

	Cm::FlushPool* flushPool = mScene.getScScene().getFlushPool();

	for (PxU32 i = 0; i < nbPairs; i += nbToProcess)
	{
		NpContactCallbackTask* task = PX_PLACEMENT_NEW(flushPool->allocate(sizeof(NpContactCallbackTask)), NpContactCallbackTask)();
		task->setData(this, contactPairs+i, PxMin(nbToProcess, nbPairs - i));
		task->setContinuation(continuation);
		task->removeReference();
	}
}

void NpScene::fetchResultsFinish(PxU32* errorState)
{
	{
		PX_SIMD_GUARD;
		PX_PROFILE_STOP_CROSSTHREAD("Basic.processCallbacks", getContextId());
		PX_PROFILE_ZONE("Basic.fetchResultsFinish", getContextId());

		mBetweenFetchResults = false;
		NP_WRITE_CHECK(this);
		
		fetchResultsPostContactCallbacks();

		if (errorState)
			*errorState = 0;

		PX_PROFILE_STOP_CROSSTHREAD("Basic.fetchResults", getContextId());
		PX_PROFILE_STOP_CROSSTHREAD("Basic.simulate", getContextId());
	}

#if PX_SUPPORT_PVD
	mScene.getScenePvdClient().frameEnd();
#endif
}

void NpScene::flushSimulation(bool sendPendingReports)
{
	PX_PROFILE_ZONE("API.flushSimulation", getContextId());
	NP_WRITE_CHECK_NOREENTRY(this);
	PX_SIMD_GUARD;

	if (getSimulationStage() != Sc::SimulationStage::eCOMPLETE)
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, 
			"PxScene::flushSimulation(): This call is not allowed while the simulation is running. Call will be ignored");
		return;
	}

	mScene.flush(sendPendingReports);
	mSQManager.flushMemory();

	//!!! TODO: Shrink all NpObject lists?
}

void NpScene::flushQueryUpdates()
{
	// DS: how do we profile const methods??????
	PX_PROFILE_ZONE("API.flushQueryUpdates", getContextId());
	NP_READ_CHECK(this);
	PX_SIMD_GUARD;

	if (getSimulationStage() != Sc::SimulationStage::eCOMPLETE)
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, 
			"PxScene::flushQueryUpdates(): This call is not allowed while the simulation is running. Call will be ignored");
		return;
	}

	mSQManager.flushUpdates();
}

/*
Replaces finishRun() with the addition of appropriate thread sync(pulled out of PhysicsThread())

Note: this function can be called from the application thread or the physics thread, depending on the
scene flags.
*/
void NpScene::executeScene(PxBaseTask* continuation)
{
	mScene.simulate(mElapsedTime, continuation);
}

void NpScene::executeCollide(PxBaseTask* continuation)
{
	mScene.collide(mElapsedTime, continuation);
}

void NpScene::executeAdvance(PxBaseTask* continuation)
{
	mScene.advance(mElapsedTime, continuation);
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::addMaterial(const NpMaterial& mat)
{
	mScene.addMaterial(mat.getScMaterial());
}

void NpScene::updateMaterial(const NpMaterial& mat)
{
	//PxU32 index = mat.getTableIndex();
	mScene.updateMaterial(mat.getScMaterial());
}

void NpScene::removeMaterial(const NpMaterial& mat)
{
	//PxU32 index = mat.getTableIndex();
	mScene.removeMaterial(mat.getScMaterial());
}

///////////////////////////////////////////////////////////////////////////////

void NpScene::setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((group1 < PX_MAX_DOMINANCE_GROUP && group2 < PX_MAX_DOMINANCE_GROUP), 
		"PxScene::setDominanceGroupPair: invalid params! Groups must be <= 31!");
	//can't change matrix diagonal 
	PX_CHECK_AND_RETURN(group1 != group2, "PxScene::setDominanceGroupPair: invalid params! Groups must be unequal! Can't change matrix diagonal!");
	PX_CHECK_AND_RETURN(
		((dominance.dominance0) == 1.0f && (dominance.dominance1 == 1.0f))
		||	((dominance.dominance0) == 1.0f && (dominance.dominance1 == 0.0f))
		||	((dominance.dominance0) == 0.0f && (dominance.dominance1 == 1.0f))
		, "PxScene::setDominanceGroupPair: invalid params! dominance must be one of (1,1), (1,0), or (0,1)!");

	mScene.setDominanceGroupPair(group1, group2, dominance);
}

PxDominanceGroupPair NpScene::getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const
{
	NP_READ_CHECK(this);
	PX_CHECK_AND_RETURN_VAL((group1 < PX_MAX_DOMINANCE_GROUP && group2 < PX_MAX_DOMINANCE_GROUP), 
		"PxScene::getDominanceGroupPair: invalid params! Groups must be <= 31!", PxDominanceGroupPair(PxU8(1u), PxU8(1u)));
	return mScene.getDominanceGroupPair(group1, group2);
}

///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_GPU_PHYSX

void NpScene::updatePhysXIndicator()
{
	Ps::IntBool isGpu = mScene.getScScene().isUsingGpuRigidBodies();

	mPhysXIndicator.setIsGpu(isGpu != 0);
}
#endif	//PX_SUPPORT_GPU_PHYSX


///////////////////////////////////////////////////////////////////////////////

void NpScene::setSceneQueryUpdateMode(PxSceneQueryUpdateMode::Enum updateMode)
{
	NP_WRITE_CHECK(this);
	mSceneQueryUpdateMode = updateMode;
}

PxSceneQueryUpdateMode::Enum NpScene::getSceneQueryUpdateMode() const
{
	NP_READ_CHECK(this);
	return mSceneQueryUpdateMode;
}

void NpScene::setDynamicTreeRebuildRateHint(PxU32 dynamicTreeRebuildRateHint)
{
	PX_CHECK_AND_RETURN((dynamicTreeRebuildRateHint >= 4), "PxScene::setDynamicTreeRebuildRateHint(): Param has to be >= 4!");
	mSQManager.setDynamicTreeRebuildRateHint(dynamicTreeRebuildRateHint);
}

PxU32 NpScene::getDynamicTreeRebuildRateHint() const
{
	NP_READ_CHECK(this);
	return mSQManager.getDynamicTreeRebuildRateHint();
}

void NpScene::forceDynamicTreeRebuild(bool rebuildStaticStructure, bool rebuildDynamicStructure)
{
	PX_PROFILE_ZONE("API.forceDynamicTreeRebuild", getContextId());
	NP_WRITE_CHECK(this);
	PX_SIMD_GUARD;
	mSQManager.forceDynamicTreeRebuild(rebuildStaticStructure, rebuildDynamicStructure);
}

void NpScene::setSolverBatchSize(PxU32 solverBatchSize)
{
	NP_WRITE_CHECK(this);
	mScene.setSolverBatchSize(solverBatchSize);
}

PxU32 NpScene::getSolverBatchSize(void) const
{
	NP_READ_CHECK(this);
	// get from our local copy
	return mScene.getSolverBatchSize();
}

///////////////////////////////////////////////////////////////////////////////

bool NpScene::setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN_VAL(PxIsFinite(value), "PxScene::setVisualizationParameter: value is not valid.", false);

	if (param >= PxVisualizationParameter::eNUM_VALUES)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "setVisualizationParameter: parameter out of range.");
		return false;
	}
	else if (value < 0.0f)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "setVisualizationParameter: value must be larger or equal to 0.");
		return false;
	}
	else
	{
		mScene.setVisualizationParameter(param, value);
		return true;
	}
}

PxReal NpScene::getVisualizationParameter(PxVisualizationParameter::Enum param) const
{
	if (param < PxVisualizationParameter::eNUM_VALUES)
		return mScene.getVisualizationParameter(param);
	else
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "getVisualizationParameter: param is not an enum.");

	return 0.0f;
}

void NpScene::setVisualizationCullingBox(const PxBounds3& box)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_MSG(box.isValid(), "PxScene::setVisualizationCullingBox(): invalid bounds provided!");
	mScene.setVisualizationCullingBox(box);
}

PxBounds3 NpScene::getVisualizationCullingBox() const
{
	NP_READ_CHECK(this);
	const PxBounds3& bounds = mScene.getVisualizationCullingBox();
	PX_ASSERT(bounds.isValid());
	return bounds;
}

void NpScene::setNbContactDataBlocks(PxU32 numBlocks)
{
	PX_CHECK_AND_RETURN((getSimulationStage() == Sc::SimulationStage::eCOMPLETE), 
		"PxScene::setNbContactDataBlock: This call is not allowed while the simulation is running. Call will be ignored!");
	
	mScene.getScScene().setNbContactDataBlocks(numBlocks);
}

PxU32 NpScene::getNbContactDataBlocksUsed() const
{
	PX_CHECK_AND_RETURN_VAL((getSimulationStage() == Sc::SimulationStage::eCOMPLETE), 
		"PxScene::getNbContactDataBlocksUsed: This call is not allowed while the simulation is running. Returning 0.", 0);
	
	return mScene.getScScene().getNbContactDataBlocksUsed();
}

PxU32 NpScene::getMaxNbContactDataBlocksUsed() const
{
	PX_CHECK_AND_RETURN_VAL((getSimulationStage() == Sc::SimulationStage::eCOMPLETE), 
		"PxScene::getMaxNbContactDataBlocksUsed: This call is not allowed while the simulation is running. Returning 0.", 0);
	
	return mScene.getScScene().getMaxNbContactDataBlocksUsed();
}

PxU32 NpScene::getTimestamp() const
{
	return mScene.getScScene().getTimeStamp();
}

PxU32 NpScene::getSceneQueryStaticTimestamp() const
{
	return mSQManager.get(PruningIndex::eSTATIC).timestamp();
}

PxCpuDispatcher* NpScene::getCpuDispatcher() const
{
	return getTaskManager()->getCpuDispatcher();
}

PxGpuDispatcher* NpScene::getGpuDispatcher() const
{
	return getTaskManager()->getGpuDispatcher();
}

PxPruningStructureType::Enum NpScene::getStaticStructure() const
{
	return mSQManager.get(PruningIndex::eSTATIC).type();
}

PxPruningStructureType::Enum NpScene::getDynamicStructure() const
{
	return mSQManager.get(PruningIndex::eDYNAMIC).type();
}

PxReal NpScene::getFrictionOffsetThreshold() const
{
	return mScene.getScScene().getFrictionOffsetThreshold();
}

PxU32 NpScene::getContactReportStreamBufferSize() const
{
	return mScene.getScScene().getDefaultContactReportStreamBufferSize();
}

#if PX_CHECKED
void NpScene::checkPositionSanity(const PxRigidActor& a, const PxTransform& pose, const char* fnName) const
{
	if(!mSanityBounds.contains(pose.p))
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
			"%s: actor pose for %lp is outside sanity bounds\n", fnName, &a);
}
#endif

namespace 
{
	struct ThreadReadWriteCount
	{
		ThreadReadWriteCount(const size_t data)
			:	readDepth(data & 0xFF),
				writeDepth((data >> 8) & 0xFF),
				readLockDepth((data >> 16) & 0xFF),
				writeLockDepth((data >> 24) & 0xFF)
		{
			
		}

		size_t getData() const { return size_t(writeLockDepth) << 24 |  size_t(readLockDepth) << 16 | size_t(writeDepth) << 8 | size_t(readDepth); }

		PxU8 readDepth;			// depth of re-entrant reads
		PxU8 writeDepth;		// depth of re-entrant writes 

		PxU8 readLockDepth;		// depth of read-locks
		PxU8 writeLockDepth;	// depth of write-locks
	};
}

#if NP_ENABLE_THREAD_CHECKS

NpScene::StartWriteResult::Enum NpScene::startWrite(bool allowReentry)
{ 
	PX_COMPILE_TIME_ASSERT(sizeof(ThreadReadWriteCount) == 4);

	if (mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK)
	{
		ThreadReadWriteCount localCounts(TlsGetValue(mThreadReadWriteDepth));

		if (mBetweenFetchResults)
			return StartWriteResult::eIN_FETCHRESULTS;

		// ensure we already have the write lock
		return localCounts.writeLockDepth > 0 ? StartWriteResult::eOK : StartWriteResult::eNO_LOCK;
	}
	
	{
		ThreadReadWriteCount localCounts(TlsGetValue(mThreadReadWriteDepth));
		StartWriteResult::Enum result;

		if (mBetweenFetchResults)
			result = StartWriteResult::eIN_FETCHRESULTS;

		// check we are the only thread reading (allows read->write order on a single thread) and no other threads are writing
		else if (mConcurrentReadCount != localCounts.readDepth || mConcurrentWriteCount != localCounts.writeDepth)
			result = StartWriteResult::eRACE_DETECTED;
		
		else
			result = StartWriteResult::eOK;

		// increment shared write counter
		Ps::atomicIncrement(&mConcurrentWriteCount);

		// in the normal case (re-entry is allowed) then we simply increment
		// the writeDepth by 1, otherwise (re-entry is not allowed) increment
		// by 2 to force subsequent writes to fail by creating a mismatch between
		// the concurrent write counter and the local counter, any value > 1 will do
		localCounts.writeDepth += allowReentry ? 1 : 2;
		TlsSetValue(mThreadReadWriteDepth, localCounts.getData());

		if (result != StartWriteResult::eOK)
			Ps::atomicIncrement(&mConcurrentErrorCount);

		return result;
	}
}

void NpScene::stopWrite(bool allowReentry) 
{ 
	if (!(mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK))
	{
		Ps::atomicDecrement(&mConcurrentWriteCount);

		// decrement depth of writes for this thread
		ThreadReadWriteCount localCounts (TlsGetValue(mThreadReadWriteDepth));

		// see comment in startWrite()
		if (allowReentry)
			localCounts.writeDepth--;
		else
			localCounts.writeDepth-=2;

		TlsSetValue(mThreadReadWriteDepth, localCounts.getData());
	}
}

bool NpScene::startRead() const 
{ 
	if (mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK)
	{
		ThreadReadWriteCount localCounts (TlsGetValue(mThreadReadWriteDepth));

		// ensure we already have the write or read lock
		return localCounts.writeLockDepth > 0 || localCounts.readLockDepth > 0;
	}
	else
	{
		Ps::atomicIncrement(&mConcurrentReadCount);

		// update current threads read depth
		ThreadReadWriteCount localCounts (TlsGetValue(mThreadReadWriteDepth));
		localCounts.readDepth++;
		TlsSetValue(mThreadReadWriteDepth, localCounts.getData());

		// success if the current thread is already performing a write (API re-entry) or no writes are in progress
		bool success = (localCounts.writeDepth > 0 || mConcurrentWriteCount == 0); 

		if (!success)
			Ps::atomicIncrement(&mConcurrentErrorCount);

		return success;
	}
} 

void NpScene::stopRead() const 
{
	if (!(mScene.getFlags() & PxSceneFlag::eREQUIRE_RW_LOCK))
	{
		Ps::atomicDecrement(&mConcurrentReadCount); 

		// update local threads read depth
		ThreadReadWriteCount localCounts (TlsGetValue(mThreadReadWriteDepth));
		localCounts.readDepth--;
		TlsSetValue(mThreadReadWriteDepth, localCounts.getData());
	}
}

#else 

NpScene::StartWriteResult::Enum NpScene::startWrite(bool) { PX_ASSERT(0); return NpScene::StartWriteResult::eOK; }
void NpScene::stopWrite(bool) {}

bool NpScene::startRead() const { PX_ASSERT(0); return false; }
void NpScene::stopRead() const {}

#endif // NP_ENABLE_THREAD_CHECKS

void NpScene::lockRead(const char* /*file*/, PxU32 /*line*/)
{
	// increment this threads read depth
	ThreadReadWriteCount localCounts (TlsGetValue(mThreadReadWriteDepth));
	localCounts.readLockDepth++;
	TlsSetValue(mThreadReadWriteDepth, localCounts.getData());

	// if we are the current writer then increment the reader count but don't actually lock (allow reading from threads with write ownership)
	if(localCounts.readLockDepth == 1)
		mRWLock.lockReader(mCurrentWriter != Thread::getId());
}

void NpScene::unlockRead()
{
	// increment this threads read depth
	ThreadReadWriteCount localCounts (TlsGetValue(mThreadReadWriteDepth));
	if(localCounts.readLockDepth < 1)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::unlockRead() called without matching call to PxScene::lockRead(), behaviour will be undefined.");
		return;
	}
	localCounts.readLockDepth--;
	TlsSetValue(mThreadReadWriteDepth, localCounts.getData());

	// only unlock on last read
	if(localCounts.readLockDepth == 0)
		mRWLock.unlockReader();
}

void NpScene::lockWrite(const char* file, PxU32 line)
{
	// increment this threads write depth
	ThreadReadWriteCount localCounts (TlsGetValue(mThreadReadWriteDepth));
	if (localCounts.writeLockDepth == 0 && localCounts.readLockDepth > 0)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, file?file:__FILE__, file?int(line):__LINE__, "PxScene::lockWrite() detected after a PxScene::lockRead(), lock upgrading is not supported, behaviour will be undefined.");
		return;
	}
	localCounts.writeLockDepth++;
	TlsSetValue(mThreadReadWriteDepth, localCounts.getData());

	// only lock on first call
	if (localCounts.writeLockDepth == 1)
		mRWLock.lockWriter();

	PX_ASSERT(mCurrentWriter == 0 || mCurrentWriter == Thread::getId());

	// set ourselves as the current writer
	mCurrentWriter = Thread::getId();
}

void NpScene::unlockWrite()
{
	// increment this thread's write depth
	ThreadReadWriteCount localCounts (TlsGetValue(mThreadReadWriteDepth));
	if (localCounts.writeLockDepth < 1)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::unlockWrite() called without matching call to PxScene::lockWrite(), behaviour will be undefined.");
		return;
	}
	localCounts.writeLockDepth--;
	TlsSetValue(mThreadReadWriteDepth, localCounts.getData());

	PX_ASSERT(mCurrentWriter == Thread::getId());

	if (localCounts.writeLockDepth == 0)
	{
		mCurrentWriter = 0;	
		mRWLock.unlockWriter();
	}
}

PxReal NpScene::getWakeCounterResetValue() const
{
	NP_READ_CHECK(this);

	return getWakeCounterResetValueInteral();
}

static PX_FORCE_INLINE void shiftRigidActor(PxRigidActor* a, const PxVec3& shift)
{
	PxActorType::Enum t = a->getType();
	if (t == PxActorType::eRIGID_DYNAMIC)
	{
		NpRigidDynamic* rd = static_cast<NpRigidDynamic*>(a);
		rd->getScbBodyFast().onOriginShift(shift);
	}
	else if (t == PxActorType::eRIGID_STATIC)
	{
		NpRigidStatic* rs = static_cast<NpRigidStatic*>(a);
		rs->getScbRigidStaticFast().onOriginShift(shift);
	}
	else
	{
		PX_ASSERT(t == PxActorType::eARTICULATION_LINK);
		NpArticulationLink* al = static_cast<NpArticulationLink*>(a);
		al->getScbBodyFast().onOriginShift(shift);
	}
}

void NpScene::shiftOrigin(const PxVec3& shift)
{
	PX_PROFILE_ZONE("API.shiftOrigin", getContextId());
	NP_WRITE_CHECK(this);

	if(mScene.isPhysicsBuffering())
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::shiftOrigin() not allowed while simulation is running. Call will be ignored.");
		return;
	}
	
	PX_SIMD_GUARD;

	const PxU32 prefetchLookAhead = 4;
	PxU32 rigidCount = mRigidActors.size();
	PxRigidActor*const* rigidActors = mRigidActors.begin();
	PxU32 batchIterCount = rigidCount / prefetchLookAhead;
	
	PxU32 idx = 0;
	for(PxU32 i=0; i < batchIterCount; i++)
	{
		// prefetch elements for next batch
		if (i < (batchIterCount-1))
		{
			Ps::prefetchLine(rigidActors[idx + prefetchLookAhead]);
			Ps::prefetchLine(reinterpret_cast<PxU8*>(rigidActors[idx + prefetchLookAhead]) + 128);  // for the buffered pose
			Ps::prefetchLine(rigidActors[idx + prefetchLookAhead + 1]);
			Ps::prefetchLine(reinterpret_cast<PxU8*>(rigidActors[idx + prefetchLookAhead + 1]) + 128);
			Ps::prefetchLine(rigidActors[idx + prefetchLookAhead + 2]);
			Ps::prefetchLine(reinterpret_cast<PxU8*>(rigidActors[idx + prefetchLookAhead + 2]) + 128);
			Ps::prefetchLine(rigidActors[idx + prefetchLookAhead + 3]);
			Ps::prefetchLine(reinterpret_cast<PxU8*>(rigidActors[idx + prefetchLookAhead + 3]) + 128);
		}
		else
		{
			for(PxU32 k=(idx + prefetchLookAhead); k < rigidCount; k++)
			{
				Ps::prefetchLine(rigidActors[k]);
				Ps::prefetchLine(reinterpret_cast<PxU8*>(rigidActors[k]) + 128);
			}
		}

		for(PxU32 j=idx; j < (idx + prefetchLookAhead); j++)
		{
			shiftRigidActor(rigidActors[j], shift);
		}

		idx += prefetchLookAhead;
	}
	// process remaining objects
	for(PxU32 i=idx; i < rigidCount; i++)
	{
		shiftRigidActor(rigidActors[i], shift);
	}

	PxArticulationBase*const* articulations = mArticulations.getEntries();
	for(PxU32 i=0; i < mArticulations.size(); i++)
	{
		PxArticulationBase* np = (articulations[i]);
		
		NpArticulationLink*const* links = reinterpret_cast<PxArticulationImpl*>(np->getImpl())->getLinks();

		for(PxU32 j=0; j < np->getNbLinks(); j++)
		{
			shiftRigidActor(links[j], shift);
		}
	}


	mScene.shiftOrigin(shift);


	//
	// shift scene query related data structures
	//
	mSQManager.shiftOrigin(shift);

#if PX_ENABLE_DEBUG_VISUALIZATION
	//
	// debug visualization
	//
	mRenderBuffer.shift(-shift);
#endif
}

#if PX_SUPPORT_PVD
PxPvdSceneClient* NpScene::getScenePvdClient()
{
	return &mScene.getScenePvdClient();
}
#else
PxPvdSceneClient* NpScene::getScenePvdClient()
{
	return NULL;
}
#endif

PxsSimulationController* NpScene::getSimulationController()
{
	return mScene.getScScene().getSimulationController();
}

void NpScene::setActiveActors(PxActor** actors, PxU32 nbActors)
{
	NP_WRITE_CHECK(this);
	mScene.setActiveActors(actors, nbActors);
}

void NpScene::forceSceneQueryRebuild()
{
	SqRefFinder sqRefFinder;
	mScene.getScScene().syncSceneQueryBounds(mSQManager.getDynamicBoundsSync(), sqRefFinder);

	mSQManager.afterSync(getSceneQueryUpdateModeFast());
}

void NpScene::sceneQueriesUpdate(physx::PxBaseTask* completionTask, bool controlSimulation)
{
	PX_SIMD_GUARD;

	bool runUpdateTasks[PruningIndex::eCOUNT] = {true, true};
	{
		// write guard must end before scene queries tasks kicks off worker threads
		NP_WRITE_CHECK(this);

		PX_PROFILE_START_CROSSTHREAD("Basic.sceneQueriesUpdate", getContextId());

		if(mSceneQueriesUpdateRunning)
		{
			//fetchSceneQueries doesn't get called
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::fetchSceneQueries was not called!");
			return;
		}
	
		// flush scene queries updates
		mSQManager.flushUpdates();

		// prepare scene queries for build - copy bounds
		runUpdateTasks[PruningIndex::eSTATIC] = mSQManager.prepareSceneQueriesUpdate(PruningIndex::eSTATIC);
		runUpdateTasks[PruningIndex::eDYNAMIC] = mSQManager.prepareSceneQueriesUpdate(PruningIndex::eDYNAMIC);

		mSceneQueriesUpdateRunning = true;
	}

	{
		PX_PROFILE_ZONE("Sim.sceneQueriesTaskSetup", getContextId());

		if (controlSimulation)
		{
			{
				PX_PROFILE_ZONE("Sim.resetDependencies", getContextId());
				// Only reset dependencies, etc if we own the TaskManager. Will be false
				// when an NpScene is controlled by an APEX scene.
				mTaskManager->resetDependencies();
			}
			mTaskManager->startSimulation();
		}

		mSceneQueriesCompletion.setContinuation(*mTaskManager, completionTask);
		if(runUpdateTasks[PruningIndex::eSTATIC])
			mSceneQueriesStaticPrunerUpdate.setContinuation(&mSceneQueriesCompletion);
		if(runUpdateTasks[PruningIndex::eDYNAMIC])
			mSceneQueriesDynamicPrunerUpdate.setContinuation(&mSceneQueriesCompletion);

		mSceneQueriesCompletion.removeReference();
		if(runUpdateTasks[PruningIndex::eSTATIC])
			mSceneQueriesStaticPrunerUpdate.removeReference();
		if(runUpdateTasks[PruningIndex::eDYNAMIC])
			mSceneQueriesDynamicPrunerUpdate.removeReference();
	}
}

bool NpScene::checkSceneQueriesInternal(bool block)
{
	PX_PROFILE_ZONE("Basic.checkSceneQueries", getContextId());
	return mSceneQueriesDone.wait(block ? Ps::Sync::waitForever : 0);
}

bool NpScene::checkQueries(bool block)
{
	return checkSceneQueriesInternal(block);
}

bool NpScene::fetchQueries(bool block)
{
	if(!mSceneQueriesUpdateRunning)
	{
		//fetchSceneQueries doesn't get called
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, 
			"PxScene::fetchQueries: fetchQueries() called illegally! It must be called after sceneQueriesUpdate()");
		return false;
	}

	if(!checkSceneQueriesInternal(block))
		return false;

	{
		PX_SIMD_GUARD;

		NP_WRITE_CHECK(this);

		// we use cross thread profile here, to show the event in cross thread view
		// PT: TODO: why do we want to show it in the cross thread view?
		PX_PROFILE_START_CROSSTHREAD("Basic.fetchQueries", getContextId());

		// flush updates and commit if work is done
		mSQManager.flushUpdates();
	
		PX_PROFILE_STOP_CROSSTHREAD("Basic.fetchQueries", getContextId());
		PX_PROFILE_STOP_CROSSTHREAD("Basic.sceneQueriesUpdate", getContextId());

		mSceneQueriesDone.reset();
		mSceneQueriesUpdateRunning = false;
	}
	return true;
}

void NpScene::frameEnd()
{
#if PX_SUPPORT_PVD
	mScene.getScenePvdClient().frameEnd();
#endif
}

PxBatchQuery* NpScene::createBatchQuery(const PxBatchQueryDesc& desc)
{
	PX_PROFILE_ZONE("API.createBatchQuery", getContextId());
	PX_CHECK_AND_RETURN_NULL(desc.isValid(),"Supplied PxBatchQueryDesc is not valid. createBatchQuery returns NULL.");

	NpBatchQuery* bq = PX_NEW(NpBatchQuery)(*this, desc);
	mBatchQueries.pushBack(bq);
	return bq;
}

void NpScene::releaseBatchQuery(PxBatchQuery* sq)
{
	PX_PROFILE_ZONE("API.releaseBatchQuery", getContextId());
	NpBatchQuery* npsq = static_cast<NpBatchQuery*>(sq);
	bool found = mBatchQueries.findAndReplaceWithLast(npsq);
	PX_UNUSED(found); PX_ASSERT(found);
	PX_DELETE_AND_RESET(npsq);
}

