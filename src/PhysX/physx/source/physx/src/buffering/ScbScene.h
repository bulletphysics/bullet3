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

#ifndef PX_PHYSICS_SCB_SCENE
#define PX_PHYSICS_SCB_SCENE

#include "ScScene.h"

#include "ScbSceneBuffer.h"
#include "ScbType.h"
#include "PsFoundation.h"
#include "PsMutex.h"
#include "PsHashSet.h"

#if PX_SUPPORT_PVD
#include "PxPhysics.h"
#include "ScbScenePvdClient.h"
#endif

namespace physx
{

class NpMaterial;

namespace Sc
{
	class BodyDesc;
}

namespace Gu
{
	class BVHStructure;
}

namespace Scb
{
	class Base;
	class RigidObject;
	class RigidStatic;
	class Body;
	class Actor;
	class Shape;
	class Constraint;
	class Material;
	class Articulation;
	class ArticulationJoint;
	class Aggregate;

	struct ShapeBuffer;

	/**
	\brief Helper class to track inserted/removed/updated buffering objects.
	*/
	class ObjectTracker
	{
	public:
		 ObjectTracker() {}
		 
		/**
		\brief An object has been inserted while the simulation was running -> track it for insertion at sync point
		*/
		void scheduleForInsert(Base& element);

		/**
		\brief An object has been removed while the simulation was running -> track it for removal at sync point
		*/
		void scheduleForRemove(Base& element);

		/**
		\brief An object has been changed while the simulation was running -> track it for update at sync point
		*/
		void scheduleForUpdate(Base& element);
		
		/**
		\brief Get the list of dirty objects that require processing at a sync point.
		*/
		Base*const *		getBuffered()				{ return mBuffered.getEntries();	}

		/**
		\brief Get number of dirty objects that require processing at a sync point.
		*/
		PxU32				getBufferedCount()	const	{ return mBuffered.size();	}
		
		/**
		\brief Cleanup dirty objects after sync point.

		\li Transition pending insertion objects from eINSERT_PENDING to eIN_SCENE.
		\li Transition pending removal objects from eREMOVE_PENDING to eNOT_IN_SCENE.
		\li Destroy objects marked as eIS_RELEASED.
		\li Clear dirty list.
		*/
		void clear();
	
		void insert(Base& element);
		void remove(Base& element);
		
	private:
		Ps::CoalescedHashSet<Base*> mBuffered;	
	};

	typedef ObjectTracker	ShapeManager;
	typedef ObjectTracker	RigidStaticManager;
	typedef ObjectTracker	BodyManager;
	typedef ObjectTracker	ArticulationManager;
	typedef ObjectTracker	ConstraintManager;
	typedef ObjectTracker	ArticulationJointManager;
	typedef ObjectTracker	AggregateManager;

	enum MATERIAL_EVENT
	{
		MATERIAL_ADD,
		MATERIAL_UPDATE,
		MATERIAL_REMOVE
	};

	class MaterialEvent
	{
	public:
		PX_FORCE_INLINE	MaterialEvent(PxU32 handle, MATERIAL_EVENT type) : mHandle(handle), mType(type)	{}
		PX_FORCE_INLINE	MaterialEvent()																	{}

		PxU32			mHandle;//handle to the master material table
		MATERIAL_EVENT	mType;
	};

	class Scene : public Ps::UserAllocated
	{
		PX_NOCOPY(Scene)
	public:
		enum BufferFlag
		{
			BF_GRAVITY					= (1 << 0),
			BF_BOUNCETHRESHOLDVELOCITY  = (1 << 1),
			BF_FLAGS					= (1 << 2),
			BF_DOMINANCE_PAIRS			= (1 << 3),
			BF_SOLVER_BATCH_SIZE		= (1 << 4),
			BF_VISUALIZATION			= (1 << 5),
			BF_CULLING_BOX				= (1 << 6)
		};

	public:
											Scene(const PxSceneDesc& desc, PxU64 contextID);
											~Scene() {}	//use release() plz.

		//---------------------------------------------------------------------------------
		// Wrapper for Sc::Scene interface
		//---------------------------------------------------------------------------------
		void								release();

		PxScene*							getPxScene(); 

		PX_INLINE void						setGravity(const PxVec3& gravity);
		PX_INLINE PxVec3					getGravity() const;

		PX_INLINE void						setBounceThresholdVelocity(const PxReal t);
		PX_INLINE PxReal					getBounceThresholdVelocity() const;

		PX_INLINE void						setFlags(PxSceneFlags flags);
		PX_INLINE PxSceneFlags				getFlags() const;

		PX_INLINE void						setFrictionType(PxFrictionType::Enum type)	{ mScene.setFrictionType(type);		}
		PX_INLINE PxFrictionType::Enum		getFrictionType()					const	{ return mScene.getFrictionType();	}

		void 								addActor(Scb::RigidStatic&, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure);
		void 								removeActor(Scb::RigidStatic&, bool wakeOnLostTouch, bool noSim);

		void 								addActor(Scb::Body&, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure);
		void 								removeActor(Scb::Body&, bool wakeOnLostTouch, bool noSim);

		void								addConstraint(Scb::Constraint&);
		void								removeConstraint(Scb::Constraint&);

		void								addArticulation(Scb::Articulation&);
		void								removeArticulation(Scb::Articulation&);

		void								addArticulationJoint(Scb::ArticulationJoint&);
		void								removeArticulationJoint(Scb::ArticulationJoint&);

		void								addAggregate(Scb::Aggregate&);
		void								removeAggregate(Scb::Aggregate&);

		void								addMaterial(const Sc::MaterialCore& mat);
		void								updateMaterial(const Sc::MaterialCore& mat);
		void								removeMaterial(const Sc::MaterialCore& mat);
		void								updateLowLevelMaterial(NpMaterial** masterMaterials);
		// These methods are only to be called at fetchResults!
		PX_INLINE PxU32						getNumActiveBodies()	const	{ return mScene.getNumActiveBodies();	}
		PX_INLINE Sc::BodyCore* const*		getActiveBodiesArray()	const	{ return mScene.getActiveBodiesArray();	}

		PX_INLINE PxSimulationEventCallback*	getSimulationEventCallback() const;
		PX_INLINE void						setSimulationEventCallback(PxSimulationEventCallback* callback);
		PX_INLINE PxContactModifyCallback*	getContactModifyCallback() const;
		PX_INLINE void						setContactModifyCallback(PxContactModifyCallback* callback);
		PX_INLINE PxCCDContactModifyCallback*	getCCDContactModifyCallback() const;
		PX_INLINE void						setCCDContactModifyCallback(PxCCDContactModifyCallback* callback);
		PX_INLINE PxU32						getCCDMaxPasses() const;
		PX_INLINE void						setCCDMaxPasses(PxU32 ccdMaxPasses);
		PX_INLINE	void					setBroadPhaseCallback(PxBroadPhaseCallback* callback);
		PX_INLINE	PxBroadPhaseCallback*	getBroadPhaseCallback()		const;
					PxBroadPhaseType::Enum	getBroadPhaseType()																				const;
					bool					getBroadPhaseCaps(PxBroadPhaseCaps& caps)														const;
					PxU32					getNbBroadPhaseRegions()																		const;
					PxU32					getBroadPhaseRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;
					PxU32					addBroadPhaseRegion(const PxBroadPhaseRegion& region, bool populateRegion);
					bool					removeBroadPhaseRegion(PxU32 handle);

		// Collision filtering
		PX_INLINE void						setFilterShaderData(const void* data, PxU32 dataSize);
		PX_INLINE const void*				getFilterShaderData() const;
		PX_INLINE PxU32						getFilterShaderDataSize() const;
		PX_INLINE PxSimulationFilterShader	getFilterShader() const;
		PX_INLINE PxSimulationFilterCallback* getFilterCallback() const;

		// Groups
		PX_INLINE void						setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance);
		PX_INLINE PxDominanceGroupPair		getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const;

		PX_INLINE void						setSolverBatchSize(PxU32 solverBatchSize);
		PX_INLINE PxU32						getSolverBatchSize() const;

		PX_INLINE void						simulate(PxReal timeStep, PxBaseTask* continuation)	{ mScene.simulate(timeStep, continuation);	}
		PX_INLINE void						collide(PxReal timeStep, PxBaseTask* continuation)	{ mScene.collide(timeStep, continuation);	}
		PX_INLINE void						advance(PxReal timeStep, PxBaseTask* continuation)	{ mScene.advance(timeStep, continuation);	}
		PX_INLINE void						endSimulation()										{ mScene.endSimulation();					}

		PX_INLINE void						flush(bool sendPendingReports);
		PX_INLINE void						fireBrokenConstraintCallbacks()		{ mScene.fireBrokenConstraintCallbacks(); }
		PX_INLINE void						fireTriggerCallbacks()				{ mScene.fireTriggerCallbacks();  }
		PX_INLINE void						fireQueuedContactCallbacks()		{ mScene.fireQueuedContactCallbacks(false); }
		PX_INLINE const Ps::Array<PxContactPairHeader>&
											getQueuedContactPairHeaders()		{ return mScene.getQueuedContactPairHeaders(); }
		PX_FORCE_INLINE void				postCallbacksPreSync()				{ mScene.postCallbacksPreSync(); }  //cleanup tasks after the pre-sync callbacks have fired

		PX_INLINE void						fireCallBacksPostSync()				{ mScene.fireCallbacksPostSync(); }	//callbacks that are fired on the core side, after the buffers get synced
		PX_INLINE void						postReportsCleanup();

		PX_INLINE const PxSceneLimits&		getLimits()	const						{ return mScene.getLimits();	}
		PX_INLINE void						setLimits(const PxSceneLimits& limits)	{ mScene.setLimits(limits);		}

		PX_INLINE void						getStats(PxSimulationStatistics& stats) const;

		PX_INLINE void						buildActiveActors();			//build the list of active actors
		PX_INLINE void						buildActiveAndFrozenActors();	//build the list of active and frozen actors

		PX_INLINE PxActor**					getActiveActors(PxU32& nbActorsOut);
		PX_INLINE void						setActiveActors(PxActor** actors, PxU32 nbActors);
		PX_INLINE PxActor**					getFrozenActors(PxU32& nbActorsOut);


		PX_INLINE PxClientID				createClient();

		PX_INLINE void						setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value);
		PX_INLINE PxReal					getVisualizationParameter(PxVisualizationParameter::Enum param) const;

		PX_INLINE void						setVisualizationCullingBox(const PxBounds3& box);
		PX_INLINE const PxBounds3&			getVisualizationCullingBox() const;

		void								shiftOrigin(const PxVec3& shift);

		//---------------------------------------------------------------------------------
		// Data synchronization
		//---------------------------------------------------------------------------------
	public:
		void								syncWriteThroughProperties();
		void								syncEntireScene();
		void								processPendingRemove();

		PX_FORCE_INLINE	PxU16*				allocShapeMaterialBuffer(PxU32 count, PxU32& startIdx)	{ return allocArrayBuffer(mShapeMaterialBuffer, count, startIdx);	}
		PX_FORCE_INLINE	const PxU16*		getShapeMaterialBuffer(PxU32 startIdx)			const	{ return &mShapeMaterialBuffer[startIdx];							}
		PX_FORCE_INLINE	Scb::Shape**		allocShapeBuffer(PxU32 count, PxU32& startIdx)			{ return allocArrayBuffer(mShapePtrBuffer, count, startIdx);		}
		PX_FORCE_INLINE	Scb::Shape**		getShapeBuffer(PxU32 startIdx)							{ return &mShapePtrBuffer[startIdx];								}
		PX_FORCE_INLINE	Scb::Actor**		allocActorBuffer(PxU32 count, PxU32& startIdx)			{ return allocArrayBuffer(mActorPtrBuffer, count, startIdx);		}
		PX_FORCE_INLINE	Scb::Actor**		getActorBuffer(PxU32 startIdx)							{ return &mActorPtrBuffer[startIdx];								}

						void				scheduleForUpdate(Scb::Base& object);
						PxU8*				getStream(ScbType::Enum type);

		PX_FORCE_INLINE void				removeShapeFromPendingUpdateList(Scb::Base& shape) { mShapeManager.remove(shape); }

		PX_FORCE_INLINE	const Sc::Scene&	getScScene()					const	{ return mScene;						}
		PX_FORCE_INLINE	Sc::Scene&			getScScene()							{ return mScene;						}
		PX_FORCE_INLINE void				prepareOutOfBoundsCallbacks()			{ mScene.prepareOutOfBoundsCallbacks();	}

	private:
						void				syncState();
		PX_FORCE_INLINE	Ps::IntBool			isBuffered(BufferFlag f)	const	{ return Ps::IntBool(mBufferFlags& f);	}
		PX_FORCE_INLINE	void				markUpdated(BufferFlag f)			{ mBufferFlags |= f;					}

		//---------------------------------------------------------------------------------
		// Miscellaneous
		//---------------------------------------------------------------------------------
	public:

		PX_FORCE_INLINE bool				isPhysicsBuffering()			const	{ return mIsBuffering;					}
		PX_FORCE_INLINE void				setPhysicsBuffering(bool buffering)		{ mIsBuffering = buffering;				}

		PX_FORCE_INLINE Sc::SimulationStage::Enum getSimulationStage()		const	{ return mScene.getSimulationStage(); }
		PX_FORCE_INLINE void				setSimulationStage(Sc::SimulationStage::Enum stage) { mScene.setSimulationStage(stage); }

		PX_FORCE_INLINE bool				isValid()								const	{ return mScene.isValid();				}

		PX_FORCE_INLINE PxReal				getWakeCounterResetValue() const { return mWakeCounterResetValue; }

		void								switchRigidToNoSim(Scb::RigidObject&, bool isDynamic);
		void								switchRigidFromNoSim(Scb::RigidObject&, bool isDynamic);

		static size_t						getScOffset()	{ return reinterpret_cast<size_t>(&reinterpret_cast<Scene*>(0)->mScene); }

#if PX_SUPPORT_PVD
		PX_FORCE_INLINE	Vd::ScbScenePvdClient&			getScenePvdClient()			{ return mScenePvdClient; }
		PX_FORCE_INLINE	const Vd::ScbScenePvdClient&	getScenePvdClient() const	{ return mScenePvdClient; }
#endif
		PX_FORCE_INLINE	PxU64				getContextId() const { return mScene.getContextId(); }

	private:
						void				addShapeInternal(Scb::Shape&);
						void				addShapesInternal(PxU32 nbShapes, PxShape** PX_RESTRICT shapes, size_t scbOffset, PxActor** PX_RESTRICT owners, PxU32 offsetNpToCore, bool isDynamic);

		template<typename T> T*				allocArrayBuffer(Ps::Array<T>& buffer, PxU32 count, PxU32& startIdx);

	private:

	template<bool TIsDynamic, class T>
	PX_FORCE_INLINE void addActorT(T& actor, ObjectTracker& tracker, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure);

	template<typename T>												void add(T& v, ObjectTracker& tracker, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure);
	template<typename T>												void remove(T& v, ObjectTracker& tracker, bool wakeOnLostTouch = false);
	template<bool TIsDynamic, typename T>								void addRigidNoSim(T& v, ObjectTracker& tracker, const Gu::BVHStructure* bvhStructure);
	template<bool TIsDynamic, typename T>								void removeRigidNoSim(T& v, ObjectTracker& tracker);
	template<typename T, typename S>									void processSimUpdates(S*const * scObjects, PxU32 nbObjects);
	template<typename T>												void processUserUpdates(ObjectTracker& tracker);
	template<typename T, bool syncOnRemove, bool wakeOnLostTouchCheck>	void processRemoves(ObjectTracker& tracker);
	template<typename T>												void processShapeRemoves(ObjectTracker& tracker);

					Sc::Scene							mScene;
 
					Ps::Array<MaterialEvent>			mSceneMaterialBuffer;
					Ps::Mutex							mSceneMaterialBufferLock;
				
					bool								mSimulationRunning;
					bool								mIsBuffering;

					Cm::FlushPool						mStream;  // Pool for temporarily buffering user changes on objects
					ShapeManager						mShapeManager;
					Ps::Array<PxU16>					mShapeMaterialBuffer;  // Buffered setMaterial() call might need to track list of materials (for multi material shapes)
					Ps::Array<Scb::Shape*>				mShapePtrBuffer;  // List of shape pointers to track buffered calls to resetFiltering(), for example
					Ps::Array<Scb::Actor*>				mActorPtrBuffer;
					RigidStaticManager					mRigidStaticManager;
					BodyManager							mBodyManager;
					ConstraintManager					mConstraintManager;
					ArticulationManager					mArticulationManager;
					ArticulationJointManager			mArticulationJointManager;
					AggregateManager					mAggregateManager;
#if PX_SUPPORT_PVD
					Vd::ScbScenePvdClient				mScenePvdClient;
#endif

	PX_FORCE_INLINE	void								updatePvdProperties()
					{
#if PX_SUPPORT_PVD
						// PT: TODO: shouldn't we test PxPvdInstrumentationFlag::eDEBUG here?						
                        if(mScenePvdClient.isConnected())
                            mScenePvdClient.updatePvdProperties(); 
#endif
					}

					PxReal								mWakeCounterResetValue;

					// note: If deletion of rigid objects is done before the sync of the simulation data then we
					//       might wanna consider having a separate list for deleted rigid objects (for performance
					//       reasons)

					//---------------------------------------------------------------------------------
					// On demand buffered data (simulation read-only data)
					//---------------------------------------------------------------------------------
					Scb::SceneBuffer					mBufferedData;
					PxU32								mBufferFlags;
	};

}  // namespace Scb

template<typename T>
T* Scb::Scene::allocArrayBuffer(Ps::Array<T>& buffer, PxU32 count, PxU32& startIdx)
{
	PxU32 oldSize = buffer.size();
	buffer.resize(oldSize + count);
	startIdx = oldSize;
	return &buffer[oldSize];
}
		
PX_INLINE void Scb::Scene::setGravity(const PxVec3& gravity)
{
	if(!isPhysicsBuffering())
	{
		mScene.setGravity(gravity);
		updatePvdProperties();
	}
	else
	{
		mBufferedData.mGravity = gravity;
		markUpdated(BF_GRAVITY);
	}
}

PX_INLINE PxVec3 Scb::Scene::getGravity() const
{
	if(isBuffered(BF_GRAVITY))
		return mBufferedData.mGravity;
	else
		return mScene.getGravity();
}

void Scb::Scene::setBounceThresholdVelocity(const PxReal t)
{
	if(!isPhysicsBuffering())
	{
		mScene.setBounceThresholdVelocity(t);
		updatePvdProperties();
	}
	else
	{
		mBufferedData.mBounceThresholdVelocity = t;
		markUpdated(BF_BOUNCETHRESHOLDVELOCITY);
	}
}

PxReal Scb::Scene::getBounceThresholdVelocity() const
{
	if(isBuffered(BF_BOUNCETHRESHOLDVELOCITY))
		return mBufferedData.mBounceThresholdVelocity;
	else
		return mScene.getBounceThresholdVelocity();
}

PX_INLINE void Scb::Scene::setFlags(PxSceneFlags flags)
{
	if(!isPhysicsBuffering())
	{
		mScene.setPublicFlags(flags);
		const bool pcm = (flags & PxSceneFlag::eENABLE_PCM);
		mScene.setPCM(pcm);
		const bool contactCache = !(flags & PxSceneFlag::eDISABLE_CONTACT_CACHE);
		mScene.setContactCache(contactCache);
		updatePvdProperties();
	}
	else
	{
		mBufferedData.mFlags = flags;
		markUpdated(BF_FLAGS);
	}
}

PX_INLINE PxSceneFlags Scb::Scene::getFlags() const
{
	if(isBuffered(BF_FLAGS))
		return mBufferedData.mFlags;
	else
		return mScene.getPublicFlags();
}

///////////////////////////////////////////////////////////////////////////////

PX_INLINE PxSimulationEventCallback* Scb::Scene::getSimulationEventCallback() const
{
	return mScene.getSimulationEventCallback();
}

PX_INLINE void Scb::Scene::setSimulationEventCallback(PxSimulationEventCallback* callback)
{
	if(!isPhysicsBuffering())
		mScene.setSimulationEventCallback(callback);
	else
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::setSimulationEventCallback() not allowed while simulation is running. Call will be ignored.");
}

PX_INLINE PxContactModifyCallback* Scb::Scene::getContactModifyCallback() const
{
	return mScene.getContactModifyCallback();
}

PX_INLINE void Scb::Scene::setContactModifyCallback(PxContactModifyCallback* callback)
{
	if(!isPhysicsBuffering())
		mScene.setContactModifyCallback(callback);
	else
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::setContactModifyCallback() not allowed while simulation is running. Call will be ignored.");
}

PX_INLINE PxCCDContactModifyCallback* Scb::Scene::getCCDContactModifyCallback() const
{
	return mScene.getCCDContactModifyCallback();
}

PX_INLINE void Scb::Scene::setCCDContactModifyCallback(PxCCDContactModifyCallback* callback)
{
	if(!isPhysicsBuffering())
		mScene.setCCDContactModifyCallback(callback);
	else
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::setCCDContactModifyCallback() not allowed while simulation is running. Call will be ignored.");
}

PX_INLINE PxU32 Scb::Scene::getCCDMaxPasses() const
{
	return mScene.getCCDMaxPasses();
}

PX_INLINE void Scb::Scene::setCCDMaxPasses(PxU32 ccdMaxPasses)
{
	if(!isPhysicsBuffering())
		mScene.setCCDMaxPasses(ccdMaxPasses);
	else
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::setCCDMaxPasses() not allowed while simulation is running. Call will be ignored.");
}

PX_INLINE PxBroadPhaseCallback* Scb::Scene::getBroadPhaseCallback() const
{
	return mScene.getBroadPhaseCallback();
}

PX_INLINE void Scb::Scene::setBroadPhaseCallback(PxBroadPhaseCallback* callback)
{
	if(!isPhysicsBuffering())
		mScene.setBroadPhaseCallback(callback);
	else
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::setBroadPhaseCallback() not allowed while simulation is running. Call will be ignored.");
}

///////////////////////////////////////////////////////////////////////////////

PX_INLINE void Scb::Scene::setFilterShaderData(const void* data, PxU32 dataSize)
{
	if(!isPhysicsBuffering())
		mScene.setFilterShaderData(data, dataSize);
	else
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxScene::setFilterShaderData() not allowed while simulation is running. Call will be ignored.");
}

PX_INLINE const void* Scb::Scene::getFilterShaderData() const
{
	return mScene.getFilterShaderDataFast();
}

PX_INLINE PxU32 Scb::Scene::getFilterShaderDataSize() const
{
	return mScene.getFilterShaderDataSizeFast();
}

PX_INLINE PxSimulationFilterShader Scb::Scene::getFilterShader() const
{
	return mScene.getFilterShaderFast();
}

PX_INLINE PxSimulationFilterCallback* Scb::Scene::getFilterCallback() const
{
	return mScene.getFilterCallbackFast();
}

PX_INLINE void Scb::Scene::flush(bool sendPendingReports)
{
	PX_ASSERT(!isPhysicsBuffering());

	mShapeMaterialBuffer.reset();
	mShapePtrBuffer.reset();
	mActorPtrBuffer.reset();

	//!!! TODO: Clear all buffers used for double buffering changes (see ObjectTracker::mBufferPool)

	mScene.flush(sendPendingReports);
}

PX_INLINE void Scb::Scene::postReportsCleanup()
{
	PX_ASSERT(!isPhysicsBuffering());
	mScene.postReportsCleanup();
}

PX_INLINE void Scb::Scene::setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance)
{
	if(!isPhysicsBuffering())
	{
		mScene.setDominanceGroupPair(group1, group2, dominance);
		updatePvdProperties();
	}
	else
	{
		mBufferedData.setDominancePair(group1, group2, dominance);
		markUpdated(BF_DOMINANCE_PAIRS);
	}
}

PX_INLINE PxDominanceGroupPair Scb::Scene::getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const
{
	if(isBuffered(BF_DOMINANCE_PAIRS))
	{
		PxDominanceGroupPair dominance(0, 0);
		if(mBufferedData.getDominancePair(group1, group2, dominance))
			return dominance;
	}

	return mScene.getDominanceGroupPair(group1, group2);
}

PX_INLINE void Scb::Scene::setSolverBatchSize(PxU32 solverBatchSize)
{
	if(!isPhysicsBuffering())
	{
		mScene.setSolverBatchSize(solverBatchSize);
		updatePvdProperties();
	}
	else
	{
		mBufferedData.mSolverBatchSize = solverBatchSize;
		markUpdated(BF_SOLVER_BATCH_SIZE);
	}
}

PX_INLINE PxU32 Scb::Scene::getSolverBatchSize() const
{
	if(isBuffered(BF_SOLVER_BATCH_SIZE))
		return mBufferedData.mSolverBatchSize;
	else
		return mScene.getSolverBatchSize();
}

PX_INLINE void Scb::Scene::getStats(PxSimulationStatistics& stats) const
{
	PX_ASSERT(!isPhysicsBuffering());

	mScene.getStats(stats);
}

PX_INLINE void Scb::Scene::buildActiveActors()
{
	PX_ASSERT(!isPhysicsBuffering());

	mScene.buildActiveActors();
}

PX_INLINE void Scb::Scene::buildActiveAndFrozenActors()
{
	PX_ASSERT(!isPhysicsBuffering());

	mScene.buildActiveAndFrozenActors();
}

PX_INLINE PxActor** Scb::Scene::getActiveActors(PxU32& nbActorsOut)
{
	if(!isPhysicsBuffering())
		return mScene.getActiveActors(nbActorsOut);
	else
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::getActiveActors() not allowed while simulation is running. Call will be ignored.");
		nbActorsOut = 0;
		return NULL;
	}
}

PX_INLINE void Scb::Scene::setActiveActors(PxActor** actors, PxU32 nbActors)
{
	mScene.setActiveActors(actors, nbActors);
}

PX_INLINE PxActor** Scb::Scene::getFrozenActors(PxU32& nbActorsOut)
{
	if(!isPhysicsBuffering())
		return mScene.getFrozenActors(nbActorsOut);
	else
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::getFrozenActors() not allowed while simulation is running. Call will be ignored.");
		nbActorsOut = 0;
		return NULL;
	}
}

PX_INLINE PxClientID Scb::Scene::createClient()
{
	if(!isPhysicsBuffering())
		return mScene.createClient();
	else
		return PxClientID(mScene.getNbClients() + mBufferedData.mNumClientsCreated++);
}

PX_INLINE void Scb::Scene::setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value)
{
	if(!isPhysicsBuffering())
		mScene.setVisualizationParameter(param, value);
	else
	{
		PX_ASSERT(param < PxVisualizationParameter::eNUM_VALUES);
		mBufferedData.mVisualizationParamChanged[param] = 1;
		mBufferedData.mVisualizationParam[param] = value;
		markUpdated(BF_VISUALIZATION);
	}
}

PX_INLINE PxReal Scb::Scene::getVisualizationParameter(PxVisualizationParameter::Enum param) const
{
	PX_ASSERT(param < PxVisualizationParameter::eNUM_VALUES);

	if(isBuffered(BF_VISUALIZATION) && mBufferedData.mVisualizationParamChanged[param])
		return mBufferedData.mVisualizationParam[param];
	else
		return mScene.getVisualizationParameter(param);
}

PX_INLINE void Scb::Scene::setVisualizationCullingBox(const PxBounds3& box)
{
	if(!isPhysicsBuffering())
		mScene.setVisualizationCullingBox(box);
	else
	{
		mBufferedData.mVisualizationCullingBox = box;
		markUpdated(BF_CULLING_BOX);
	}
}

PX_INLINE const PxBounds3& Scb::Scene::getVisualizationCullingBox() const
{
	if(isBuffered(BF_CULLING_BOX))
		return mBufferedData.mVisualizationCullingBox;
	else
		return mScene.getVisualizationCullingBox();
}

}

#endif
