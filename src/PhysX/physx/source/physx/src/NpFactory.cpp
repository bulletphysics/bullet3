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

#include "NpCast.h"
#include "NpFactory.h"
#include "NpPhysics.h"
#include "ScPhysics.h"
#include "ScbScene.h"
#include "ScbActor.h"
#include "GuHeightField.h"
#include "GuTriangleMesh.h"
#include "GuConvexMesh.h"

#include "NpConnector.h"
#include "NpPtrTableStorageManager.h"
#include "CmCollection.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationJointReducedCoordinate.h"

using namespace physx;

NpFactory::NpFactory()
: GuMeshFactory()
, mConnectorArrayPool(PX_DEBUG_EXP("connectorArrayPool"))
, mPtrTableStorageManager(PX_NEW(NpPtrTableStorageManager))
, mMaterialPool(PX_DEBUG_EXP("MaterialPool"))
#if PX_SUPPORT_PVD
	, mNpFactoryListener(NULL)
#endif	
{
}

namespace
{
	template <typename T> void releaseAll(Ps::HashSet<T*>& container)
	{
		// a bit tricky: release will call the factory back to remove the object from
		// the tracking array, immediately invalidating the iterator. Reconstructing the
		// iterator per delete can be expensive. So, we use a temporary object.
		//
		// a coalesced hash would be efficient too, but we only ever iterate over it
		// here so it's not worth the 2x remove penalty over the normal hash.

		Ps::Array<T*, Ps::ReflectionAllocator<T*> > tmp;
		tmp.reserve(container.size());
		for(typename Ps::HashSet<T*>::Iterator iter = container.getIterator(); !iter.done(); ++iter)
			tmp.pushBack(*iter);

		PX_ASSERT(tmp.size() == container.size());
		for(PxU32 i=0;i<tmp.size();i++)
			tmp[i]->release();
	}
}

NpFactory::~NpFactory()
{
	PX_DELETE(mPtrTableStorageManager);
}

void NpFactory::release()
{
	releaseAll(mAggregateTracking);
	releaseAll(mConstraintTracking);
	releaseAll(mArticulationTracking);
	releaseAll(mActorTracking);
	while(mShapeTracking.size())
		static_cast<NpShape*>(mShapeTracking.getEntries()[0])->releaseInternal();

	GuMeshFactory::release();  // deletes the class
}

void NpFactory::createInstance()
{
	PX_ASSERT(!mInstance);
	mInstance = PX_NEW(NpFactory)();
}

void NpFactory::destroyInstance()
{
	PX_ASSERT(mInstance);
	mInstance->release();
	mInstance = NULL;
}

NpFactory* NpFactory::mInstance = NULL;

///////////////////////////////////////////////////////////////////////////////

template <class T0, class T1>
static void addToTracking(T1& set, T0* element, Ps::Mutex& mutex, bool lock)
{
	if(!element)
		return;

	if(lock)
		mutex.lock();

	set.insert(element);

	if(lock)
		mutex.unlock();
}

/////////////////////////////////////////////////////////////////////////////// Actors

void NpFactory::addRigidStatic(PxRigidStatic* npActor, bool lock)
{
	addToTracking(mActorTracking, npActor, mTrackingMutex, lock);
}

void NpFactory::addRigidDynamic(PxRigidDynamic* npBody, bool lock)
{
	addToTracking(mActorTracking, npBody, mTrackingMutex, lock);
}

void NpFactory::addShape(PxShape* shape, bool lock)
{
	addToTracking(mShapeTracking, shape, mTrackingMutex, lock);
}

void NpFactory::onActorRelease(PxActor* a)
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	mActorTracking.erase(a);
}

void NpFactory::onShapeRelease(PxShape* a)
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	mShapeTracking.erase(a);
}

void NpFactory::addArticulation(PxArticulationBase* npArticulation, bool lock)
{
	addToTracking(mArticulationTracking, npArticulation, mTrackingMutex, lock);
}

namespace
{
	PxArticulationBase* createArticulation()
	{
		NpArticulation* npArticulation = NpFactory::getInstance().createNpArticulation();
		if (!npArticulation)
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Articulation initialization failed: returned NULL.");

		return npArticulation;
	}

	PxArticulationBase* createArticulationRC()
	{
		NpArticulationReducedCoordinate* npArticulation = NpFactory::getInstance().createNpArticulationRC();
		if (!npArticulation)
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Articulation initialization failed: returned NULL.");

		return npArticulation;
	}

	NpArticulationLink* createArticulationLink(PxArticulationBase&root, NpArticulationLink* parent, const PxTransform& pose)
	{
		PX_CHECK_AND_RETURN_NULL(pose.isValid(),"Supplied PxArticulation pose is not valid. Articulation link creation method returns NULL.");
		PX_CHECK_AND_RETURN_NULL((!parent || (&parent->getRoot() == &root)), "specified parent link is not part of the destination articulation. Articulation link creation method returns NULL.");
	
		NpArticulationLink* npArticulationLink = NpFactory::getInstance().createNpArticulationLink(root, parent, pose);
		if (!npArticulationLink)
		{
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, 
				"Articulation link initialization failed: returned NULL.");
			return NULL;
		}

		if (parent)
		{
			PxTransform parentPose = parent->getCMassLocalPose().transformInv(pose);
			PxTransform childPose = PxTransform(PxIdentity);
						
			PxArticulationJointBase* npArticulationJoint = root.createArticulationJoint(*parent, parentPose, *npArticulationLink, childPose);
			if (!npArticulationJoint)
			{
				PX_DELETE(npArticulationLink);
	
				Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, 
				"Articulation link initialization failed due to joint creation failure: returned NULL.");
				return NULL;
			}

			npArticulationLink->setInboundJoint(*npArticulationJoint);
		}

		return npArticulationLink;
	}

	// pointers to functions above, initialized during subsystem registration
	static PxArticulationBase* (*sCreateArticulationFn)() = 0;
	static PxArticulationBase* (*sCreateArticulationRCFn)() = 0;
	static NpArticulationLink* (*sCreateArticulationLinkFn)(PxArticulationBase&, NpArticulationLink* parent, const PxTransform& pose) = 0;
}

void NpFactory::registerArticulations()
{
	sCreateArticulationFn = &::createArticulation;
	sCreateArticulationLinkFn = &::createArticulationLink;
}

void NpFactory::registerArticulationRCs()
{
	sCreateArticulationRCFn = &::createArticulationRC;
	sCreateArticulationLinkFn = &::createArticulationLink;
}

void NpFactory::releaseArticulationToPool(PxArticulationBase& articulation)
{
	
	PX_ASSERT(articulation.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	if (articulation.getType() == PxArticulationBase::eMaximumCoordinate)
	{
		Ps::Mutex::ScopedLock lock(mArticulationPoolLock);
		mArticulationPool.destroy(static_cast<NpArticulation*>(&articulation));
	}
	else
	{
		Ps::Mutex::ScopedLock lock(mArticulationRCPoolLock);
		mArticulationRCPool.destroy(static_cast<NpArticulationReducedCoordinate*>(&articulation));
	}
}

NpArticulation* NpFactory::createNpArticulation()
{
	Ps::Mutex::ScopedLock lock(mArticulationPoolLock);
	return mArticulationPool.construct();
}

PxArticulation* NpFactory::createArticulation()
{
	if(!sCreateArticulationFn)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, 
			"Articulations not registered: returned NULL.");
		return NULL;
	}

	PxArticulationBase* npArticulation = (*sCreateArticulationFn)();
	if(npArticulation)
		addArticulation(npArticulation);

	return static_cast<PxArticulation*>(npArticulation);
}

PxArticulationReducedCoordinate* NpFactory::createArticulationRC()
{
	if (!sCreateArticulationRCFn)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__,
			"Articulations not registered: returned NULL.");
		return NULL;
	}

	PxArticulationBase* npArticulation = (*sCreateArticulationRCFn)();
	if (npArticulation)
		addArticulation(npArticulation);

	return static_cast<PxArticulationReducedCoordinate*>(npArticulation);
}

NpArticulationReducedCoordinate* NpFactory::createNpArticulationRC()
{
	Ps::Mutex::ScopedLock lock(mArticulationRCPoolLock);
	return mArticulationRCPool.construct();
}

void NpFactory::onArticulationRelease(PxArticulationBase* a)
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	mArticulationTracking.erase(a);
}

NpArticulationLink* NpFactory::createNpArticulationLink(PxArticulationBase&root, NpArticulationLink* parent, const PxTransform& pose)
{
	 NpArticulationLink* npArticulationLink;
	{
		Ps::Mutex::ScopedLock lock(mArticulationLinkPoolLock);		
		npArticulationLink = mArticulationLinkPool.construct(pose, root, parent);
	}
	return npArticulationLink;	
}

void NpFactory::releaseArticulationLinkToPool(NpArticulationLink& articulationLink)
{
	PX_ASSERT(articulationLink.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	Ps::Mutex::ScopedLock lock(mArticulationLinkPoolLock);
	mArticulationLinkPool.destroy(&articulationLink);
}

PxArticulationLink* NpFactory::createArticulationLink(PxArticulationBase& root, NpArticulationLink* parent, const PxTransform& pose)
{
	if(!sCreateArticulationLinkFn)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, 
			"Articulations not registered: returned NULL.");
		return NULL;
	}

	return (*sCreateArticulationLinkFn)(root, parent, pose);
}

NpArticulationJoint* NpFactory::createNpArticulationJoint(NpArticulationLink& parent, const PxTransform& parentFrame, NpArticulationLink& child, const PxTransform& childFrame)
{
	NpArticulationJoint* npArticulationJoint;
	{
		Ps::Mutex::ScopedLock lock(mArticulationJointPoolLock);		
		npArticulationJoint = mArticulationJointPool.construct(parent, parentFrame, child, childFrame);
	}
	return npArticulationJoint;	
}

void NpFactory::releaseArticulationJointToPool(NpArticulationJoint& articulationJoint)
{
	PX_ASSERT(articulationJoint.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	Ps::Mutex::ScopedLock lock(mArticulationJointPoolLock);
	mArticulationJointPool.destroy(&articulationJoint);
}

NpArticulationJointReducedCoordinate* NpFactory::createNpArticulationJointRC(NpArticulationLink& parent, const PxTransform& parentFrame, NpArticulationLink& child, const PxTransform& childFrame)
{
	NpArticulationJointReducedCoordinate* npArticulationJoint;
	{
		Ps::Mutex::ScopedLock lock(mArticulationJointRCPoolLock);
		npArticulationJoint = mArticulationRCJointPool.construct(parent, parentFrame, child, childFrame);
	}
	return npArticulationJoint;
}

void NpFactory::releaseArticulationJointRCToPool(NpArticulationJointReducedCoordinate& articulationJoint)
{
	PX_ASSERT(articulationJoint.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	Ps::Mutex::ScopedLock lock(mArticulationJointRCPoolLock);
	mArticulationRCJointPool.destroy(&articulationJoint);
}

/////////////////////////////////////////////////////////////////////////////// constraint

void NpFactory::addConstraint(PxConstraint* npConstraint, bool lock)
{
	addToTracking(mConstraintTracking, npConstraint, mTrackingMutex, lock);
}

PxConstraint* NpFactory::createConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize)
{
	PX_CHECK_AND_RETURN_NULL((actor0 && !actor0->is<PxRigidStatic>()) || (actor1 && !actor1->is<PxRigidStatic>()), "createConstraint: At least one actor must be dynamic or an articulation link");

	NpConstraint* npConstraint;
	{
		Ps::Mutex::ScopedLock lock(mConstraintPoolLock);
		npConstraint = mConstraintPool.construct(actor0, actor1, connector, shaders, dataSize);
	}
	addConstraint(npConstraint);
	return npConstraint;
}

void NpFactory::releaseConstraintToPool(NpConstraint& constraint)
{
	PX_ASSERT(constraint.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	Ps::Mutex::ScopedLock lock(mConstraintPoolLock);
	mConstraintPool.destroy(&constraint);
}

void NpFactory::onConstraintRelease(PxConstraint* c)
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	mConstraintTracking.erase(c);
}

/////////////////////////////////////////////////////////////////////////////// aggregate

void NpFactory::addAggregate(PxAggregate* npAggregate, bool lock)
{
	addToTracking(mAggregateTracking, npAggregate, mTrackingMutex, lock);
}

PxAggregate* NpFactory::createAggregate(PxU32 maxActors, bool selfCollisions)
{
	NpAggregate* npAggregate;
	{
		Ps::Mutex::ScopedLock lock(mAggregatePoolLock);
		npAggregate = mAggregatePool.construct(maxActors, selfCollisions);
	}

	addAggregate(npAggregate);
	return npAggregate;
}

void NpFactory::releaseAggregateToPool(NpAggregate& aggregate)
{
	PX_ASSERT(aggregate.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	Ps::Mutex::ScopedLock lock(mAggregatePoolLock);
	mAggregatePool.destroy(&aggregate);
}

void NpFactory::onAggregateRelease(PxAggregate* a)
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	mAggregateTracking.erase(a);
}

///////////////////////////////////////////////////////////////////////////////

PxMaterial* NpFactory::createMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution)
{
	PX_CHECK_AND_RETURN_NULL(dynamicFriction >= 0.0f, "createMaterial: dynamicFriction must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(staticFriction >= 0.0f, "createMaterial: staticFriction must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(restitution >= 0.0f || restitution <= 1.0f, "createMaterial: restitution must be between 0 and 1.");
	
	Sc::MaterialData data;
	data.staticFriction = staticFriction;
	data.dynamicFriction = dynamicFriction;
	data.restitution = restitution;

	NpMaterial* npMaterial;
	{
		Ps::Mutex::ScopedLock lock(mMaterialPoolLock);		
		npMaterial = mMaterialPool.construct(data);
	}
	return npMaterial;	
}

void NpFactory::releaseMaterialToPool(NpMaterial& material)
{
	PX_ASSERT(material.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	Ps::Mutex::ScopedLock lock(mMaterialPoolLock);
	mMaterialPool.destroy(&material);
}

///////////////////////////////////////////////////////////////////////////////

NpConnectorArray* NpFactory::acquireConnectorArray()
{
	Ps::MutexT<>::ScopedLock l(mConnectorArrayPoolLock);
	return mConnectorArrayPool.construct();
}

void NpFactory::releaseConnectorArray(NpConnectorArray* array)
{
	Ps::MutexT<>::ScopedLock l(mConnectorArrayPoolLock);
	mConnectorArrayPool.destroy(array);
}

///////////////////////////////////////////////////////////////////////////////

NpShape* NpFactory::createShape(const PxGeometry& geometry,
								PxShapeFlags shapeFlags,
								PxMaterial*const* materials,
								PxU16 materialCount,
								bool isExclusive)
{	
	switch(geometry.getType())
	{
		case PxGeometryType::eBOX:
			PX_CHECK_AND_RETURN_NULL(static_cast<const PxBoxGeometry&>(geometry).isValid(), "Supplied PxGeometry is not valid. Shape creation method returns NULL.");
			break;
		case PxGeometryType::eSPHERE:
			PX_CHECK_AND_RETURN_NULL(static_cast<const PxSphereGeometry&>(geometry).isValid(), "Supplied PxGeometry is not valid. Shape creation method returns NULL.");
			break;
		case PxGeometryType::eCAPSULE:
			PX_CHECK_AND_RETURN_NULL(static_cast<const PxCapsuleGeometry&>(geometry).isValid(), "Supplied PxGeometry is not valid. Shape creation method returns NULL.");
			break;
		case PxGeometryType::eCONVEXMESH:
			PX_CHECK_AND_RETURN_NULL(static_cast<const PxConvexMeshGeometry&>(geometry).isValid(), "Supplied PxGeometry is not valid. Shape creation method returns NULL.");
			break;
		case PxGeometryType::ePLANE:
			PX_CHECK_AND_RETURN_NULL(static_cast<const PxPlaneGeometry&>(geometry).isValid(), "Supplied PxGeometry is not valid. Shape creation method returns NULL.");
			break;
		case PxGeometryType::eHEIGHTFIELD:
			PX_CHECK_AND_RETURN_NULL(static_cast<const PxHeightFieldGeometry&>(geometry).isValid(), "Supplied PxGeometry is not valid. Shape creation method returns NULL.");
			break;
		case PxGeometryType::eTRIANGLEMESH:
			PX_CHECK_AND_RETURN_NULL(static_cast<const PxTriangleMeshGeometry&>(geometry).isValid(), "Supplied PxGeometry is not valid. Shape creation method returns NULL.");
			break;
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			PX_ASSERT(0);
	}

	//
	// Check for invalid material table setups
	//

#if PX_CHECKED
	if (!NpShape::checkMaterialSetup(geometry, "Shape creation", materials, materialCount))
		return NULL;
#endif

	Ps::InlineArray<PxU16, 4> materialIndices("NpFactory::TmpMaterialIndexBuffer");
	materialIndices.resize(materialCount);
	if(materialCount == 1)
		materialIndices[0] = Ps::to16((static_cast<NpMaterial*>(materials[0]))->getHandle());
	else
		NpMaterial::getMaterialIndices(materials, materialIndices.begin(), materialCount);

	NpShape* npShape;
	{
		Ps::Mutex::ScopedLock lock(mShapePoolLock);
		PxU16* mi = materialIndices.begin(); // required to placate pool constructor arg passing
		npShape = mShapePool.construct(geometry, shapeFlags, mi, materialCount, isExclusive);
	}

	if(!npShape)
		return NULL;

	for(PxU32 i=0; i < materialCount; i++)
		static_cast<NpMaterial*>(npShape->getMaterial(i))->incRefCount();

	addShape(npShape);

	return npShape;
}

void NpFactory::releaseShapeToPool(NpShape& shape)
{
	PX_ASSERT(shape.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	Ps::Mutex::ScopedLock lock(mShapePoolLock);
	mShapePool.destroy(&shape);
}

PxU32 NpFactory::getNbShapes() const
{
	// PT: TODO: isn't there a lock missing here? See usage in MeshFactory
	return mShapeTracking.size();
}

PxU32 NpFactory::getShapes(PxShape** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const
{
	// PT: TODO: isn't there a lock missing here? See usage in MeshFactory
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mShapeTracking.getEntries(), mShapeTracking.size());
}

///////////////////////////////////////////////////////////////////////////////

PxRigidStatic* NpFactory::createRigidStatic(const PxTransform& pose)
{
	PX_CHECK_AND_RETURN_NULL(pose.isValid(), "pose is not valid. createRigidStatic returns NULL.");

	NpRigidStatic* npActor;

	{
		Ps::Mutex::ScopedLock lock(mRigidStaticPoolLock);
		npActor = mRigidStaticPool.construct(pose);
	}

	addRigidStatic(npActor);
	return npActor;
}

void NpFactory::releaseRigidStaticToPool(NpRigidStatic& rigidStatic)
{
	PX_ASSERT(rigidStatic.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	Ps::Mutex::ScopedLock lock(mRigidStaticPoolLock);
	mRigidStaticPool.destroy(&rigidStatic);
}

///////////////////////////////////////////////////////////////////////////////

PxRigidDynamic* NpFactory::createRigidDynamic(const PxTransform& pose)
{
	PX_CHECK_AND_RETURN_NULL(pose.isValid(), "pose is not valid. createRigidDynamic returns NULL.");

	NpRigidDynamic* npBody;
	{
		Ps::Mutex::ScopedLock lock(mRigidDynamicPoolLock);
		npBody = mRigidDynamicPool.construct(pose);
	}
	addRigidDynamic(npBody);
	return npBody;
}

void NpFactory::releaseRigidDynamicToPool(NpRigidDynamic& rigidDynamic)
{
	PX_ASSERT(rigidDynamic.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	Ps::Mutex::ScopedLock lock(mRigidDynamicPoolLock);
	mRigidDynamicPool.destroy(&rigidDynamic);
}

///////////////////////////////////////////////////////////////////////////////

// PT: this function is here to minimize the amount of locks when deserializing a collection
void NpFactory::addCollection(const Cm::Collection& collection)
{
	
	PxU32 nb = collection.getNbObjects();
	const Ps::Pair<PxBase* const, PxSerialObjectId>* entries = collection.internalGetObjects();
	// PT: we take the lock only once, here
	Ps::Mutex::ScopedLock lock(mTrackingMutex);

	for(PxU32 i=0;i<nb;i++)
	{
		PxBase* s = entries[i].first;
		const PxType serialType = s->getConcreteType();
//////////////////////////
		if(serialType==PxConcreteType::eHEIGHTFIELD)
		{
			Gu::HeightField* gu = static_cast<Gu::HeightField*>(s);
			gu->setMeshFactory(this);
			addHeightField(gu, false);
		}
		else if(serialType==PxConcreteType::eCONVEX_MESH)
		{
			Gu::ConvexMesh* gu = static_cast<Gu::ConvexMesh*>(s);
			gu->setMeshFactory(this);
			addConvexMesh(gu, false);
		}
		else if(serialType==PxConcreteType::eTRIANGLE_MESH_BVH33 || serialType==PxConcreteType::eTRIANGLE_MESH_BVH34)
		{
			Gu::TriangleMesh* gu = static_cast<Gu::TriangleMesh*>(s);
			gu->setMeshFactory(this);
			addTriangleMesh(gu, false);
		}
		else if(serialType==PxConcreteType::eRIGID_DYNAMIC)
		{
			NpRigidDynamic* np = static_cast<NpRigidDynamic*>(s);
			addRigidDynamic(np, false);
		}
		else if(serialType==PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic* np = static_cast<NpRigidStatic*>(s);
			addRigidStatic(np, false);
		}
		else if(serialType==PxConcreteType::eSHAPE)
		{
			NpShape* np = static_cast<NpShape*>(s);
			addShape(np, false);
		}
		else if(serialType==PxConcreteType::eMATERIAL)
		{
		}
		else if(serialType==PxConcreteType::eCONSTRAINT)
		{
			NpConstraint* np = static_cast<NpConstraint*>(s);
			addConstraint(np, false);
		}
		else if(serialType==PxConcreteType::eAGGREGATE)
		{
			NpAggregate* np = static_cast<NpAggregate*>(s);
			addAggregate(np, false);

			// PT: TODO: double-check this.... is it correct?			
			for(PxU32 j=0;j<np->getCurrentSizeFast();j++)
			{
				PxBase* actor = np->getActorFast(j);
				const PxType serialType1 = actor->getConcreteType();

				if(serialType1==PxConcreteType::eRIGID_STATIC)
					addRigidStatic(static_cast<NpRigidStatic*>(actor), false);
				else if(serialType1==PxConcreteType::eRIGID_DYNAMIC)
					addRigidDynamic(static_cast<NpRigidDynamic*>(actor), false);
				else if(serialType1==PxConcreteType::eARTICULATION_LINK)
				{}
				else PX_ASSERT(0);
			}
		}
		else if(serialType==PxConcreteType::eARTICULATION)
		{
			NpArticulation* np = static_cast<NpArticulation*>(s);
			addArticulation(np, false);
		}
		else if(serialType==PxConcreteType::eARTICULATION_LINK)
		{
//			NpArticulationLink* np = static_cast<NpArticulationLink*>(s);
		}
		else if(serialType==PxConcreteType::eARTICULATION_JOINT)
		{
//			NpArticulationJoint* np = static_cast<NpArticulationJoint*>(s);
		}
		else
		{
//			assert(0);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_PVD
void NpFactory::setNpFactoryListener( NpFactoryListener& inListener)
{
	mNpFactoryListener = &inListener;
	addFactoryListener(inListener);
}
#endif

///////////////////////////////////////////////////////////////////////////////

// these calls are issued from the Scb layer when buffered deletes are issued. 
// TODO: we should really push these down as a virtual interface that is part of Scb's reqs
// to eliminate this link-time dep.

static void NpDestroyRigidActor(Scb::RigidStatic& scb)
{
	NpRigidStatic* np = const_cast<NpRigidStatic*>(getNpRigidStatic(&scb));

	void* ud = np->userData;

	if(np->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		NpFactory::getInstance().releaseRigidStaticToPool(*np);
	else
		np->~NpRigidStatic();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(np, ud);
}

static void NpDestroyRigidDynamic(Scb::Body& scb)
{
	NpRigidDynamic* np = const_cast<NpRigidDynamic*>(getNpRigidDynamic(&scb));

	void* ud = np->userData;
	if(np->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		NpFactory::getInstance().releaseRigidDynamicToPool(*np);
	else
		np->~NpRigidDynamic();
	NpPhysics::getInstance().notifyDeletionListenersMemRelease(np, ud);
}

static void NpDestroyArticulationLink(Scb::Body& scb)
{
	NpArticulationLink* np = const_cast<NpArticulationLink*>(getNpArticulationLink(&scb));

	void* ud = np->userData;
	if(np->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		NpFactory::getInstance().releaseArticulationLinkToPool(*np);
	else
		np->~NpArticulationLink();	
	NpPhysics::getInstance().notifyDeletionListenersMemRelease(np, ud);
}

static void NpDestroyArticulationJoint(Scb::ArticulationJoint& scb)
{
	PxArticulationJointBase* np = scb.getScArticulationJoint().getRoot();

	if (np->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		PxArticulationJointImpl* impl = np->getImpl();
		if (impl->mType == PxArticulationBase::eMaximumCoordinate)
		{
			NpFactory::getInstance().releaseArticulationJointToPool(*static_cast<NpArticulationJoint*>(np));
		}
		else
		{
			NpFactory::getInstance().releaseArticulationJointRCToPool(*static_cast<NpArticulationJointReducedCoordinate*>(np));
		}
	}
	else
		np->~PxArticulationJointBase();	
	NpPhysics::getInstance().notifyDeletionListenersMemRelease(np, NULL);
}

static void NpDestroyArticulation(Scb::Articulation& scb)
{
	PxArticulationBase* artic = const_cast<PxArticulationBase*>(static_cast<const PxArticulationBase*>(getNpArticulation(&scb)));
	
	void* ud = artic->PxArticulationBase::userData;
	if (artic->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		NpFactory::getInstance().releaseArticulationToPool(*artic);
	else
		artic->~PxArticulationBase();
	NpPhysics::getInstance().notifyDeletionListenersMemRelease(artic, ud);
}

static void NpDestroyAggregate(Scb::Aggregate& scb)
{
	NpAggregate* np = const_cast<NpAggregate*>(getNpAggregate(&scb));

	if(np->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		NpFactory::getInstance().releaseAggregateToPool(*np);
	else
		np->~NpAggregate();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(np, NULL);
}

static void NpDestroyShape(Scb::Shape& scb)
{
	NpShape* np = const_cast<NpShape*>(getNpShape(&scb));

	void* ud = np->userData;

	if(np->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		NpFactory::getInstance().releaseShapeToPool(*np);
	else
		np->~NpShape();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(np, ud);
}

static void NpDestroyConstraint(Scb::Constraint& scb)
{
	const size_t offset = size_t(&(reinterpret_cast<NpConstraint*>(0)->getScbConstraint()));
	NpConstraint* np = reinterpret_cast<NpConstraint*>(reinterpret_cast<char*>(&scb)-offset);
	if(np->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		NpFactory::getInstance().releaseConstraintToPool(*np);
	else
		np->~NpConstraint();
	NpPhysics::getInstance().notifyDeletionListenersMemRelease(np, NULL);
}

namespace physx
{
	void NpDestroy(Scb::Base& base)
	{
		switch(base.getScbType())
		{
			case ScbType::eSHAPE_EXCLUSIVE:
			case ScbType::eSHAPE_SHARED:				{ NpDestroyShape(static_cast<Scb::Shape&>(base));							}break;
			case ScbType::eBODY:						{ NpDestroyRigidDynamic(static_cast<Scb::Body&>(base));						}break;
			case ScbType::eBODY_FROM_ARTICULATION_LINK:	{ NpDestroyArticulationLink(static_cast<Scb::Body&>(base));					}break;
			case ScbType::eRIGID_STATIC:				{ NpDestroyRigidActor(static_cast<Scb::RigidStatic&>(base));				}break;
			case ScbType::eCONSTRAINT:					{ NpDestroyConstraint(static_cast<Scb::Constraint&>(base));					}break;
			case ScbType::eARTICULATION:				{ NpDestroyArticulation(static_cast<Scb::Articulation&>(base));				}break;
			case ScbType::eARTICULATION_JOINT:			{ NpDestroyArticulationJoint(static_cast<Scb::ArticulationJoint&>(base));	}break;
			case ScbType::eAGGREGATE:					{ NpDestroyAggregate(static_cast<Scb::Aggregate&>(base));					}break;
			case ScbType::eUNDEFINED:
			case ScbType::eTYPE_COUNT:
				PX_ALWAYS_ASSERT_MESSAGE("NpDestroy: missing type!");
				break;
		}
	}
}

