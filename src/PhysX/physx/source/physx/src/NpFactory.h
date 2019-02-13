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

#ifndef PX_PHYSICS_NP_FACTORY
#define PX_PHYSICS_NP_FACTORY

#include "PsPool.h"
#include "PsMutex.h"
#include "PsHashSet.h"

#include "GuMeshFactory.h"
#include "CmPhysXCommon.h"
#include "PxPhysXConfig.h"
#include "PxShape.h"
#include "PxArticulationBase.h"

namespace physx
{

class PxActor;

class PxRigidActor;

class PxRigidStatic;
class NpRigidStatic;

class PxRigidDynamic;
class NpRigidDynamic;

class NpConnectorArray;

struct PxConstraintShaderTable;
class PxConstraintConnector;
class PxConstraint;
class NpConstraint;

class PxArticulation;
class PxArticulationReducedCoordinate;
class NpArticulation;
class NpArticulationReducedCoordinate;
class PxArticulationLink;
class NpArticulationLink;
class NpArticulationJoint;
class NpArticulationJointReducedCoordinate;

class PxMaterial;
class NpMaterial;

class PxGeometry;

class NpShape;

class NpScene;

class PxAggregate;
class NpAggregate;

class NpConnectorArray;
class NpPtrTableStorageManager;

namespace Cm
{
   class Collection;
}

namespace Scb
{
	class RigidObject;
	class Articulation;
}

namespace pvdsdk
{
	class PvdDataStream;
	class PsPvd;
}

class NpFactoryListener : public GuMeshFactoryListener
{
protected:
	virtual ~NpFactoryListener(){}
};

class NpFactory : public GuMeshFactory
{
	PX_NOCOPY(NpFactory)
public:
														NpFactory();
private:
														~NpFactory();
public:
	static		void									createInstance();
	static		void									destroyInstance();
	static		void									registerArticulations();
	static		void									registerArticulationRCs();

				void									release();

				void									addCollection(const Cm::Collection& collection);

	PX_INLINE static NpFactory&							getInstance() { return *mInstance; }

				// Rigid dynamic
				PxRigidDynamic*							createRigidDynamic(const PxTransform& pose);
				void									addRigidDynamic(PxRigidDynamic*, bool lock=true);
				void									releaseRigidDynamicToPool(NpRigidDynamic&);
// PT: TODO: add missing functions
//				PxU32									getNbRigidDynamics() const;
//				PxU32									getRigidDynamics(PxRigidDynamic** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Rigid static
				PxRigidStatic*							createRigidStatic(const PxTransform& pose);
				void									addRigidStatic(PxRigidStatic*, bool lock=true);
				void									releaseRigidStaticToPool(NpRigidStatic&);
// PT: TODO: add missing functions
//				PxU32									getNbRigidStatics() const;
//				PxU32									getRigidStatics(PxRigidStatic** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Shapes
				NpShape*								createShape(const PxGeometry& geometry, PxShapeFlags shapeFlags, PxMaterial*const* materials, PxU16 materialCount, bool isExclusive);
				void									addShape(PxShape*, bool lock=true);
				void									releaseShapeToPool(NpShape&);
				PxU32									getNbShapes() const;
				PxU32									getShapes(PxShape** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Constraints
				PxConstraint*							createConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize);
				void									addConstraint(PxConstraint*, bool lock=true);
				void									releaseConstraintToPool(NpConstraint&);
// PT: TODO: add missing functions
//				PxU32									getNbConstraints() const;
//				PxU32									getConstraints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Articulations
				PxArticulation*							createArticulation();
				void									addArticulation(PxArticulationBase*, bool lock=true);
				void									releaseArticulationToPool(PxArticulationBase& articulation);
				PxArticulationReducedCoordinate*		createArticulationRC();
				NpArticulation*							createNpArticulation();
				NpArticulationReducedCoordinate*		createNpArticulationRC();
// PT: TODO: add missing functions
//				PxU32									getNbArticulations() const;
//				PxU32									getArticulations(PxArticulation** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Articulation links
				NpArticulationLink*						createNpArticulationLink(PxArticulationBase&root, NpArticulationLink* parent, const PxTransform& pose);
                void									releaseArticulationLinkToPool(NpArticulationLink& articulation);
				PxArticulationLink*						createArticulationLink(PxArticulationBase&, NpArticulationLink* parent, const PxTransform& pose);

				// Articulation joints
				NpArticulationJoint*					createNpArticulationJoint(NpArticulationLink& parent, const PxTransform& parentFrame, NpArticulationLink& child, const PxTransform& childFrame);
               	void									releaseArticulationJointToPool(NpArticulationJoint& articulationJoint);
				NpArticulationJointReducedCoordinate*	createNpArticulationJointRC(NpArticulationLink& parent, const PxTransform& parentFrame, NpArticulationLink& child, const PxTransform& childFrame);
				void									releaseArticulationJointRCToPool(NpArticulationJointReducedCoordinate& articulationJoint);

				// Aggregates
				PxAggregate*							createAggregate(PxU32 maxActors, bool selfCollisions);
				void									addAggregate(PxAggregate*, bool lock=true);
				void									releaseAggregateToPool(NpAggregate&);
// PT: TODO: add missing functions
//				PxU32									getNbAggregates() const;
//				PxU32									getAggregates(PxAggregate** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Materials
				PxMaterial*								createMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution);
				void									releaseMaterialToPool(NpMaterial& material);

				// It's easiest to track these uninvasively, so it's OK to use the Px pointers
				void									onActorRelease(PxActor*);
				void									onConstraintRelease(PxConstraint*);
				void									onAggregateRelease(PxAggregate*);
				void									onArticulationRelease(PxArticulationBase*);
				void									onShapeRelease(PxShape*);

				NpConnectorArray*						acquireConnectorArray();
				void									releaseConnectorArray(NpConnectorArray*);
				
				NpPtrTableStorageManager&				getPtrTableStorageManager()	{ return *mPtrTableStorageManager; }

#if PX_SUPPORT_PVD
				void									setNpFactoryListener( NpFactoryListener& );
#endif

private:
				void									releaseExclusiveShapeUserReferences();

				Ps::Pool<NpConnectorArray>				mConnectorArrayPool;
				Ps::Mutex								mConnectorArrayPoolLock;

				NpPtrTableStorageManager*				mPtrTableStorageManager;

				Ps::HashSet<PxAggregate*>				mAggregateTracking;
				Ps::HashSet<PxArticulationBase*>		mArticulationTracking;
				Ps::HashSet<PxConstraint*>				mConstraintTracking;
				Ps::HashSet<PxActor*>					mActorTracking;				
				Ps::CoalescedHashSet<PxShape*>			mShapeTracking;

				Ps::Pool2<NpRigidDynamic, 4096>			mRigidDynamicPool;
				Ps::Mutex								mRigidDynamicPoolLock;

				Ps::Pool2<NpRigidStatic, 4096>			mRigidStaticPool;
				Ps::Mutex								mRigidStaticPoolLock;

				Ps::Pool2<NpShape, 4096>				mShapePool;
				Ps::Mutex								mShapePoolLock;

				Ps::Pool2<NpAggregate, 4096>			mAggregatePool;
				Ps::Mutex								mAggregatePoolLock;

				Ps::Pool2<NpConstraint, 4096>			mConstraintPool;
				Ps::Mutex								mConstraintPoolLock;

				Ps::Pool2<NpMaterial, 4096>				mMaterialPool;
				Ps::Mutex								mMaterialPoolLock;

				Ps::Pool2<NpArticulation, 4096>			mArticulationPool;
				Ps::Mutex								mArticulationPoolLock;

				Ps::Pool2<NpArticulationReducedCoordinate, 4096>	mArticulationRCPool;
				Ps::Mutex											mArticulationRCPoolLock;

				Ps::Pool2<NpArticulationLink, 4096>		mArticulationLinkPool;
				Ps::Mutex								mArticulationLinkPoolLock;

				Ps::Pool2<NpArticulationJoint, 4096>	mArticulationJointPool;
				Ps::Mutex								mArticulationJointPoolLock;	

				Ps::Pool2<NpArticulationJointReducedCoordinate, 4096> mArticulationRCJointPool;
				Ps::Mutex								mArticulationJointRCPoolLock;

	static		NpFactory*								mInstance;

#if PX_SUPPORT_PVD
				NpFactoryListener*						mNpFactoryListener;
#endif
};

}

#endif
