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


#ifndef PX_PHYSICS_NP_PHYSICS
#define PX_PHYSICS_NP_PHYSICS

#include "PxPhysics.h"
#include "PsUserAllocated.h"
#include "GuMeshFactory.h"
#include "NpMaterial.h"
#include "NpPhysicsInsertionCallback.h"
#include "NpMaterialManager.h"
#include "ScPhysics.h"
#include "PsHashSet.h"
#include "PsHashMap.h"

#ifdef LINUX
#include <string.h>
#endif

#if PX_SUPPORT_GPU_PHYSX
#include "device/PhysXIndicator.h"
#endif

#include "PsPvd.h"

namespace physx
{
#if PX_SUPPORT_PVD
namespace Vd
{
	class PvdPhysicsClient;
}
#endif
	struct NpMaterialIndexTranslator
	{
		NpMaterialIndexTranslator() : indicesNeedTranslation(false) {}

		Ps::HashMap<PxU16, PxU16>	map;
		bool						indicesNeedTranslation;
	};

	class NpScene;	
	struct PxvOffsetTable;

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4996)	// We have to implement deprecated member functions, do not warn.
#endif

class NpPhysics : public PxPhysics, public Ps::UserAllocated
{
	NpPhysics& operator=(const NpPhysics&);
	NpPhysics(const NpPhysics &);

	struct NpDelListenerEntry : public UserAllocated
	{
		NpDelListenerEntry(const PxDeletionEventFlags& de, bool restrictedObjSet)
			: flags(de)
			, restrictedObjectSet(restrictedObjSet)
		{
		}

		Ps::HashSet<const PxBase*> registeredObjects;  // specifically registered objects for deletion events
		PxDeletionEventFlags flags;
		bool restrictedObjectSet;
	};


									NpPhysics(	const PxTolerancesScale& scale, 
												const PxvOffsetTable& pxvOffsetTable,
												bool trackOutstandingAllocations, 
                                                physx::pvdsdk::PsPvd* pvd);
	virtual							~NpPhysics();

public:
	
	static      NpPhysics*			createInstance(	PxU32 version, 
													PxFoundation& foundation, 
													const PxTolerancesScale& scale,
													bool trackOutstandingAllocations,
													physx::pvdsdk::PsPvd* pvd);

	static		PxU32			releaseInstance();

	static      NpPhysics&		getInstance() { return *mInstance; }

	virtual     void			release();

	virtual		PxScene*		createScene(const PxSceneDesc&);
				void			releaseSceneInternal(PxScene&);
	virtual		PxU32			getNbScenes()	const;
	virtual		PxU32			getScenes(PxScene** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const;

	virtual		PxRigidStatic*						createRigidStatic(const PxTransform&);
	virtual		PxRigidDynamic*						createRigidDynamic(const PxTransform&);
	virtual		PxArticulation*						createArticulation();
	virtual		PxArticulationReducedCoordinate*	createArticulationReducedCoordinate();
	virtual		PxConstraint*						createConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize);
	virtual		PxAggregate*						createAggregate(PxU32 maxSize, bool selfCollision);

	virtual		PxShape*			createShape(const PxGeometry&, PxMaterial*const *, PxU16, bool, PxShapeFlags shapeFlags);
	virtual		PxU32				getNbShapes()	const;
	virtual		PxU32				getShapes(PxShape** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

	virtual		PxMaterial*			createMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution);
	virtual		PxU32				getNbMaterials() const;
	virtual		PxU32				getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const;

	virtual		PxTriangleMesh*		createTriangleMesh(PxInputStream&);
	virtual		PxU32				getNbTriangleMeshes()	const;
	virtual		PxU32				getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex=0)	const;

	virtual		PxHeightField*		createHeightField(PxInputStream& stream);
	virtual		PxU32				getNbHeightFields()	const;
	virtual		PxU32				getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex=0)	const;

	virtual		PxConvexMesh*		createConvexMesh(PxInputStream&);
	virtual		PxU32				getNbConvexMeshes() const;
	virtual		PxU32				getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const;

	virtual		PxBVHStructure*		createBVHStructure(PxInputStream&);
	virtual		PxU32				getNbBVHStructures() const;
	virtual		PxU32				getBVHStructures(PxBVHStructure** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const;

#if PX_SUPPORT_GPU_PHYSX
	void							registerPhysXIndicatorGpuClient();
	void							unregisterPhysXIndicatorGpuClient();
#else
	PX_FORCE_INLINE void			registerPhysXIndicatorGpuClient() {}
	PX_FORCE_INLINE void			unregisterPhysXIndicatorGpuClient() {}
#endif

	virtual		PxPruningStructure*	createPruningStructure(PxRigidActor*const* actors, PxU32 nbActors);

	virtual		const PxTolerancesScale&		getTolerancesScale() const;

	virtual		PxFoundation&		getFoundation();

	PX_INLINE	NpScene*			getScene(PxU32 i) const { return mSceneArray[i]; }
	PX_INLINE	PxU32				getNumScenes() const { return mSceneArray.size(); }
#if PX_CHECKED
	static PX_INLINE	void		heightfieldsAreRegistered() { mHeightFieldsRegistered = true;  }
#endif

	virtual		void				registerDeletionListener(PxDeletionListener& observer, const PxDeletionEventFlags& deletionEvents, bool restrictedObjectSet);
	virtual		void				unregisterDeletionListener(PxDeletionListener& observer);
	virtual		void				registerDeletionListenerObjects(PxDeletionListener& observer, const PxBase* const* observables, PxU32 observableCount);
	virtual		void				unregisterDeletionListenerObjects(PxDeletionListener& observer, const PxBase* const* observables, PxU32 observableCount);

				void				notifyDeletionListeners(const PxBase*, void* userData, PxDeletionEventFlag::Enum deletionEvent);
	PX_FORCE_INLINE void			notifyDeletionListenersUserRelease(const PxBase* b, void* userData) { notifyDeletionListeners(b, userData, PxDeletionEventFlag::eUSER_RELEASE); }
	PX_FORCE_INLINE void			notifyDeletionListenersMemRelease(const PxBase* b, void* userData) { notifyDeletionListeners(b, userData, PxDeletionEventFlag::eMEMORY_RELEASE); }

	virtual		PxPhysicsInsertionCallback&	getPhysicsInsertionCallback() { return mObjectInsertion; }

				void				removeMaterialFromTable(NpMaterial&);
				void				updateMaterial(NpMaterial&);
				bool				sendMaterialTable(NpScene&);

				NpMaterialManager&	getMaterialManager()	{	return mMasterMaterialManager;	}

				NpMaterial*			addMaterial(NpMaterial* np);

	static		void				initOffsetTables(PxvOffsetTable& pxvOffsetTable);

	static bool apiReentryLock;

private:
				typedef Ps::CoalescedHashMap<PxDeletionListener*, NpDelListenerEntry*> DeletionListenerMap;

				Ps::Array<NpScene*>	mSceneArray;

				Sc::Physics					mPhysics;
				NpMaterialManager			mMasterMaterialManager;

				NpPhysicsInsertionCallback	mObjectInsertion;

				struct MeshDeletionListener: public GuMeshFactoryListener
				{
					void onGuMeshFactoryBufferRelease(const PxBase* object, PxType type)
					{
						PX_UNUSED(type);
						NpPhysics::getInstance().notifyDeletionListeners(object, NULL, PxDeletionEventFlag::eMEMORY_RELEASE);
					}
				};

				Ps::Mutex								mDeletionListenerMutex;
				DeletionListenerMap						mDeletionListenerMap;
				MeshDeletionListener					mDeletionMeshListener;
				bool									mDeletionListenersExist;

				Ps::Mutex								mSceneAndMaterialMutex;  // guarantees thread safety for API calls related to scene and material containers

#if PX_SUPPORT_GPU_PHYSX
				PhysXIndicator		mPhysXIndicator;
				PxU32				mNbRegisteredGpuClients;
				Ps::Mutex			mPhysXIndicatorMutex;
#endif
#if PX_SUPPORT_PVD	
				physx::pvdsdk::PsPvd*  mPvd;
                Vd::PvdPhysicsClient*   mPvdPhysicsClient;
#endif

	static		PxU32				mRefCount;
	static		NpPhysics*			mInstance;

#if PX_CHECKED
	static		bool				mHeightFieldsRegistered;	//just for error checking
#endif

	friend class NpCollection;
};

#if PX_VC
#pragma warning(pop)
#endif
}

#endif
