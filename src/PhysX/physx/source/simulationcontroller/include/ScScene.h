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


#ifndef PX_PHYSICS_SCP_SCENE
#define PX_PHYSICS_SCP_SCENE

#include "CmPhysXCommon.h"
#include "PxPhysXConfig.h"
#include "PxScene.h"
#include "PxSceneDesc.h"
#include "PxSimulationEventCallback.h"
#include "PsPool.h"
#include "PsHashSet.h"
#include "CmRenderOutput.h"
#include "CmTask.h"
#include "CmFlushPool.h"
#include "CmPreallocatingPool.h"
#include "CmBitMap.h"
#include "ScIterators.h"
#include "PxsMaterialManager.h"
#include "PxvManager.h"
#include "ScBodyCore.h"
#include "PxArticulationBase.h"

#define PX_MAX_DOMINANCE_GROUP 32

class OverlapFilterTask;

namespace physx
{

// PT: TODO: move INVALID_FILTER_PAIR_INDEX out of public API
struct PxFilterInfo
{
	PX_FORCE_INLINE	PxFilterInfo()								:	filterFlags(0), pairFlags(0), filterPairIndex(INVALID_FILTER_PAIR_INDEX)			{}
	PX_FORCE_INLINE	PxFilterInfo(PxFilterFlags filterFlags_)	:	filterFlags(filterFlags_), pairFlags(0), filterPairIndex(INVALID_FILTER_PAIR_INDEX)	{}

	PxFilterFlags	filterFlags;
	PxPairFlags		pairFlags;
	PxU32			filterPairIndex;
};

struct PxTriggerPair;

class PxsIslandManager;
class PxsSimulationController;
class PxsSimulationControllerCallback;
class PxsMemoryManager;

#if PX_SUPPORT_GPU_PHYSX
class PxsKernelWranglerManager;
class PxsHeapMemoryAllocatorManager;
#endif

namespace IG
{
	class SimpleIslandManager;
}

class PxsCCDContext;

namespace Cm
{
	class DeferredIDPool;
	class IDPool;
}

namespace Bp
{
	class AABBManager;
	class BroadPhase;
	class BoundsArray;
}

namespace Sq
{
	typedef PxU32 PrunerHandle;	// PT: we should get this from SqPruner.h but it cannot be included from here
}

namespace Dy
{
	class ArticulationV;
	class Context;
}

namespace Pt
{
	class Context;
}

namespace Sc
{
	class ActorSim;
	class ElementSim;
	class Interaction;

	class ShapeCore;
	class RigidCore;
	class StaticCore;
	class ConstraintCore;
	class MaterialCore;
	class ArticulationCore;
	class ArticulationJointCore;
	class LLArticulationPool;
	class LLArticulationRCPool;
	class BodyCore;

	class NPhaseCore;
	class LowLevelThreadingThunk;
	class Client;
	class ConstraintInteraction;

	class BodySim;
	class ShapeSim;
	class RigidSim;
	class StaticSim;
	class ConstraintSim;
	struct ConstraintGroupNode;
	class ConstraintProjectionManager;
	struct TriggerPairExtraData;
	class ObjectIDTracker;
	class ActorPairReport;
	class ContactStreamManager;
	class SqBoundsManager;
	class ShapeInteraction;
	class ElementInteractionMarker;
	class ArticulationSim;

	class SimStats;

	struct SimStateData;

	struct BatchInsertionState
	{
		BodySim*  bodySim;
		StaticSim*staticSim; 
		ShapeSim* shapeSim;
		ptrdiff_t staticActorOffset;
		ptrdiff_t staticShapeTableOffset;
		ptrdiff_t dynamicActorOffset;
		ptrdiff_t dynamicShapeTableOffset;
		ptrdiff_t shapeOffset;
	};

	struct BatchRemoveState
	{
		Ps::InlineArray<Sc::ShapeSim*, 64>				bufferedShapes;
		Ps::InlineArray<const Sc::ShapeCore*, 64>		removedShapes;
	};

	struct InteractionType
	{
		enum Enum
		{
			eOVERLAP		= 0,		// corresponds to ShapeInteraction
			eTRIGGER,					// corresponds to TriggerInteraction
			eMARKER,					// corresponds to ElementInteractionMarker
			eTRACKED_IN_SCENE_COUNT,	// not a real type, interactions above this limit are tracked in the scene
			eCONSTRAINTSHADER,			// corresponds to ConstraintInteraction
			eARTICULATION,				// corresponds to ArticulationJointSim

			eINVALID
		};
	};

	struct SceneInternalFlag
	{
		enum Enum
		{
			eSCENE_SIP_STATES_DIRTY_DOMINANCE		= (1<<1),
			eSCENE_SIP_STATES_DIRTY_VISUALIZATION	= (1<<2),
			eSCENE_DEFAULT							= 0
		};
	};

	struct SimulationStage
	{
		enum Enum
		{
			eCOMPLETE,
			eCOLLIDE,
			eFETCHCOLLIDE,
			eADVANCE,
			eFETCHRESULT
		};
	};

	// PT: TODO: revisit the need for a virtual interface
	struct SqBoundsSync
	{
		virtual void sync(const Sq::PrunerHandle* handles, const PxU32* indices, const PxBounds3* bounds, PxU32 count, const Cm::BitMap& dirtyShapeSimMap) = 0;

		virtual ~SqBoundsSync() {}
	};

	// PT: TODO: revisit the need for a virtual interface
	struct SqRefFinder
	{
		virtual Sq::PrunerHandle find(const PxRigidBody* body, const PxShape* shape) = 0;

		virtual ~SqRefFinder() {}
	};

	class Scene : public Ps::UserAllocated
	{
		struct SimpleBodyPair
		{
			BodySim* body1;
			BodySim* body2;
			PxU32 body1ID;
			PxU32 body2ID;
		};

		PX_NOCOPY(Scene)

		//---------------------------------------------------------------------------------
		// External interface
		//---------------------------------------------------------------------------------
	public:    
   
					void						release();

	PX_FORCE_INLINE	void						setGravity(const PxVec3& g)			{ mGravity = g;	mBodyGravityDirty = true;			}
	PX_FORCE_INLINE	PxVec3						getGravity()				const	{ return mGravity;									}
	PX_FORCE_INLINE void						setElapsedTime(const PxReal t)		{ mDt = t; mOneOverDt = t > 0.0f ? 1.0f/t : 0.0f;	}

					void						setBounceThresholdVelocity(const PxReal t);
					PxReal						getBounceThresholdVelocity() const;

	PX_FORCE_INLINE	void						setPublicFlags(PxSceneFlags flags)	{ mPublicFlags = flags;						}
	PX_FORCE_INLINE	PxSceneFlags				getPublicFlags()			const	{ return mPublicFlags;						}

					void						setFrictionType(PxFrictionType::Enum model);
					PxFrictionType::Enum 		getFrictionType() const;
					void						setPCM(bool enabled);
					void						setContactCache(bool enabled);

					void						addStatic(StaticCore&, void*const *shapes, PxU32 nbShapes, size_t shapePtrOffset, PxBounds3* uninflatedBounds);
					void						removeStatic(StaticCore&, Ps::InlineArray<const Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch);

					void						addBody(BodyCore&, void*const *shapes, PxU32 nbShapes, size_t shapePtrOffset, PxBounds3* uninflatedBounds, bool compound);
					void						removeBody(BodyCore&, Ps::InlineArray<const Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch);

					// Batch insertion API.
					// the bounds generated here are the uninflated bounds for the shapes, *if* they are trigger or sim shapes. 
					// It's up to the caller to ensure the bounds array is big enough.
					// Some care is required in handling these since sim and SQ tweak the bounds in slightly different ways.

					void						startBatchInsertion(BatchInsertionState&);
					void						addStatic(PxActor* actor, BatchInsertionState&, PxBounds3* outBounds);
					void						addBody(PxActor* actor, BatchInsertionState&, PxBounds3* outBounds, bool compound);
					void						finishBatchInsertion(BatchInsertionState&);

					// Batch remove helpers
	PX_FORCE_INLINE	void						setBatchRemove(BatchRemoveState* bs)	{ mBatchRemoveState = bs;	}
	PX_FORCE_INLINE	BatchRemoveState*			getBatchRemove()		const			{ return mBatchRemoveState;	}
					void						prefetchForRemove(const BodyCore& ) const;
					void						prefetchForRemove(const StaticCore& ) const;

					void						addConstraint(ConstraintCore&, RigidCore*, RigidCore*);
					void						removeConstraint(ConstraintCore&);

					void						addArticulation(ArticulationCore&, BodyCore& root);
					void						removeArticulation(ArticulationCore&);

					void						addArticulationJoint(ArticulationJointCore&, BodyCore& parent, BodyCore& child);
					void						removeArticulationJoint(ArticulationJointCore&);

					void						addArticulationSimControl(Sc::ArticulationCore& core);

					PxU32						getNbArticulations() const;
					Sc::ArticulationCore* const* getArticulations();
	
					PxU32						getNbConstraints() const;
					Sc::ConstraintCore*const *	getConstraints();

					void						initContactsIterator(ContactIterator&, PxsContactManagerOutputIterator&);

		// Simulation events
					void						setSimulationEventCallback(PxSimulationEventCallback* callback);
					PxSimulationEventCallback*	getSimulationEventCallback() const;

		// Contact modification
					void						setContactModifyCallback(PxContactModifyCallback* callback);
					PxContactModifyCallback*	getContactModifyCallback() const;
					void						setCCDContactModifyCallback(PxCCDContactModifyCallback* callback);
					PxCCDContactModifyCallback*	getCCDContactModifyCallback() const;

					void						setCCDMaxPasses(PxU32 ccdMaxPasses);
					PxU32						getCCDMaxPasses() const;

		// Broad-phase callback
					void						setBroadPhaseCallback(PxBroadPhaseCallback* callback);
					PxBroadPhaseCallback*		getBroadPhaseCallback()	const;

		// Broad-phase management
					void						finishBroadPhase(PxBaseTask* continuation);
					void						finishBroadPhaseStage2(const PxU32 ccdPass);
					void						preallocateContactManagers(PxBaseTask* continuation);

					void						islandInsertion(PxBaseTask* continuation);
					void						registerContactManagers(PxBaseTask* continuation);
					void						registerInteractions(PxBaseTask* continuation);
					void						registerSceneInteractions(PxBaseTask* continuation);

					void						secondPassNarrowPhase(PxBaseTask* continuation);

		// Groups
					void						setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance);
					PxDominanceGroupPair		getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const;

					void						setSolverBatchSize(PxU32 solverBatchSize);
					PxU32						getSolverBatchSize() const;

					void						setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value);
					PxReal						getVisualizationParameter(PxVisualizationParameter::Enum param) const;

					void						setVisualizationCullingBox(const PxBounds3& box);
					const PxBounds3&			getVisualizationCullingBox() const;

		// Run
					void						simulate(PxReal timeStep, PxBaseTask* continuation);
					void						advance(PxReal timeStep, PxBaseTask* continuation);
					void						collide(PxReal timeStep, PxBaseTask* continuation);
					void						endSimulation();
					void						flush(bool sendPendingReports);
					void						fireBrokenConstraintCallbacks();
					void						fireTriggerCallbacks();
					void						fireQueuedContactCallbacks(bool asPartOfFlush);
					void						fireOnAdvanceCallback();

					const Ps::Array<PxContactPairHeader>&
												getQueuedContactPairHeaders();

					bool						fireOutOfBoundsCallbacks();
					void						prepareOutOfBoundsCallbacks();
					void						postCallbacksPreSync();
					void						postReportsCleanup();
					void						fireCallbacksPostSync();
					void						syncSceneQueryBounds(SqBoundsSync& sync, SqRefFinder& finder);					

					PxU32						getDefaultContactReportStreamBufferSize() const;

					PxReal						getFrictionOffsetThreshold() const;

	PX_FORCE_INLINE	void						setLimits(const PxSceneLimits& limits)	{ mLimits = limits;	}
	PX_FORCE_INLINE	const PxSceneLimits&		getLimits()						const	{ return mLimits;	}

					void						visualizeStartStep();
					void						visualizeEndStep();
					Cm::RenderBuffer&			getRenderBuffer();

					void						setNbContactDataBlocks(PxU32 blockCount);
					PxU32						getNbContactDataBlocksUsed() const;
					PxU32						getMaxNbContactDataBlocksUsed() const;
					PxU32						getMaxNbConstraintDataBlocksUsed() const;

					void						setScratchBlock(void* addr, PxU32 size);

// PX_ENABLE_SIM_STATS
					void						getStats(PxSimulationStatistics& stats) const;
	PX_FORCE_INLINE	SimStats&					getStatsInternal() { return *mStats; }
// PX_ENABLE_SIM_STATS

					void						buildActiveActors();
					void						buildActiveAndFrozenActors();
					PxActor**					getActiveActors(PxU32& nbActorsOut);
					void						setActiveActors(PxActor** actors, PxU32 nbActors);

					PxActor**					getFrozenActors(PxU32& nbActorsOut);

					void						finalizeContactStreamAndCreateHeader(PxContactPairHeader& header, 
						const ActorPairReport& aPair, 
						ContactStreamManager& cs, PxU32 removedShapeTestMask);

					PxClientID					createClient();
	PX_FORCE_INLINE	PxU32						getNbClients()	const	{ return mClients.size();	}

					PxTaskManager*				getTaskManagerPtr() const { return mTaskManager; }

					void						shiftOrigin(const PxVec3& shift);

	PX_FORCE_INLINE	Ps::Pool<SimStateData>*		getSimStateDataPool() { return mSimStateDataPool; }

		//---------------------------------------------------------------------------------
		// Miscellaneous
		//---------------------------------------------------------------------------------							
		//internal public methods:
	public:
												Scene(const PxSceneDesc& desc, PxU64 contextID);
												~Scene() {}	//use release() plz.

					void						preAllocate(PxU32 nbStatics, PxU32 nbBodies, PxU32 nbStaticShapes, PxU32 nbDynamicShapes);

	PX_FORCE_INLINE	PxsContext*					getLowLevelContext() { return mLLContext; }
	PX_FORCE_INLINE const PxsContext*			getLowLevelContext() const { return mLLContext; }

	PX_FORCE_INLINE	Dy::Context*				getDynamicsContext() { return mDynamicsContext; }
	PX_FORCE_INLINE const Dy::Context*			getDynamicsContext() const { return mDynamicsContext; }

	PX_FORCE_INLINE	PxsSimulationController*	getSimulationController() { return mSimulationController; }
	PX_FORCE_INLINE	const PxsSimulationController*	getSimulationController() const { return mSimulationController; }

	PX_FORCE_INLINE	Bp::AABBManager*			getAABBManager()			{ return mAABBManager;	}
	PX_FORCE_INLINE const Bp::AABBManager*		getAABBManager()	const	{ return mAABBManager;	}
	PX_FORCE_INLINE Ps::Array<BodySim*>&		getCcdBodies()				{ return mCcdBodies;	}

	/*PX_FORCE_INLINE	PxsIslandManager*			getIslandManager()	{ return mIslandManager; }
	PX_FORCE_INLINE	const PxsIslandManager*		getIslandManager() const { return mIslandManager; }*/

	PX_FORCE_INLINE	IG::SimpleIslandManager*	getSimpleIslandManager()	{ return mSimpleIslandManager; }
	PX_FORCE_INLINE	const IG::SimpleIslandManager*		getSimpleIslandManager() const { return mSimpleIslandManager; }

	PX_FORCE_INLINE Sc::SimulationStage::Enum	getSimulationStage() const { return mSimulationStage; }
	PX_FORCE_INLINE void						setSimulationStage(Sc::SimulationStage::Enum stage) { mSimulationStage = stage; }

					void						addShape(RigidSim&, ShapeCore&, PxBounds3* uninflatedBounds);
					void						removeShape(ShapeSim&, bool wakeOnLostTouch);

					void						registerShapeInNphase(const ShapeCore& shapeCore);
					void						unregisterShapeFromNphase(const ShapeCore& shapeCore);

					void						notifyNphaseOnUpdateShapeMaterial(const ShapeCore& shapeCore);

	// Get an array of the active actors.
	PX_FORCE_INLINE	BodyCore*const*				getActiveBodiesArray() const { return mActiveBodies.begin(); }
	PX_FORCE_INLINE	PxU32						getNumActiveBodies() const { return mActiveBodies.size(); }

	PX_FORCE_INLINE	BodyCore*const*				getActiveCompoundBodiesArray() const { return mActiveCompoundBodies.begin(); }
	PX_FORCE_INLINE	PxU32						getNumActiveCompoundBodies() const { return mActiveCompoundBodies.size(); }

	PX_FORCE_INLINE	PxU32						getNbInteractions(InteractionType::Enum type)		const	{ return mInteractions[type].size();	}
	PX_FORCE_INLINE	PxU32						getNbActiveInteractions(InteractionType::Enum type)	const	{ return mActiveInteractionCount[type];	}
	// Get all interactions of a certain type
	PX_FORCE_INLINE	Interaction**				getInteractions(InteractionType::Enum type)					{ return mInteractions[type].begin();	}
	PX_FORCE_INLINE	Interaction**				getActiveInteractions(InteractionType::Enum type)			{ return mInteractions[type].begin();	}

					void						registerInteraction(Interaction* interaction, bool active);
					void						unregisterInteraction(Interaction* interaction);

					void						notifyInteractionActivated(Interaction* interaction);
					void						notifyInteractionDeactivated(Interaction* interaction);

	// for pool management of interaction arrays, a major cause of dynamic allocation
					void**						allocatePointerBlock(PxU32 size);
					void						deallocatePointerBlock(void**, PxU32 size);
	private:
	// Get the number of active one-way dominator actors
	PX_FORCE_INLINE	PxU32						getActiveKinematicBodiesCount() const { return mActiveKinematicBodyCount; }

	// Get an iterator to the active one-way dominator actors
	PX_FORCE_INLINE	BodyCore*const*				getActiveKinematicBodies() const { return mActiveBodies.begin(); }

	// Get the number of active non-kinematic actors
	PX_FORCE_INLINE	PxU32						getActiveDynamicBodiesCount() const { return mActiveBodies.size() - mActiveKinematicBodyCount; }

	// Get the active non-kinematic actors
	PX_FORCE_INLINE	BodyCore*const*				getActiveDynamicBodies() const { return mActiveBodies.begin() + mActiveKinematicBodyCount; }

					void						swapInteractionArrayIndices(PxU32 id1, PxU32 id2, InteractionType::Enum type);
	public:

		PX_FORCE_INLINE Cm::BitMap&				getDirtyShapeSimMap() { return mDirtyShapeSimMap; }

					void						addToActiveBodyList(BodySim& actor);
					void						addToActiveCompoundBodyList(BodySim& actor);
					void						removeFromActiveBodyList(BodySim& actor);
					void						removeFromActiveCompoundBodyList(BodySim& actor);
					void						swapInActiveBodyList(BodySim& body); // call when an active body gets switched from dynamic to kinematic or vice versa

					void						addToFrozenBodyList(BodySim& actor);
					void						removeFromFrozenBodyList(BodySim& actor);
					void						addBrokenConstraint(Sc::ConstraintCore*);
					void						addActiveBreakableConstraint(Sc::ConstraintSim*, ConstraintInteraction*);
					void						removeActiveBreakableConstraint(Sc::ConstraintSim*);
		//the Actor should register its top level shapes with these.
					void						removeBody(BodySim&);

					void						raiseSceneFlag(SceneInternalFlag::Enum flag) { mInternalFlags |= flag; }

					//lists of actors woken up or put to sleep last simulate
					void                        onBodyWakeUp(BodySim* body);
					void                        onBodySleep(BodySim* body);

	PX_FORCE_INLINE	bool						isValid() const	{ return (mLLContext != NULL);	}

					void						addToLostTouchList(BodySim* body1, BodySim* body2);

					void						initDominanceMatrix();
					
					void						setCreateContactReports(bool s);

					PxU32						createAggregate(void* userData, bool selfCollisions);
					void						deleteAggregate(PxU32 id);

					Dy::ArticulationV*			createLLArticulation(Sc::ArticulationSim* sim);
					void						destroyLLArticulation(Dy::ArticulationV&);


					Ps::Pool<ConstraintInteraction>*
													getConstraintInteractionPool()			const	{ return mConstraintInteractionPool;	}
	public:
						PxBroadPhaseType::Enum		getBroadPhaseType()																				const;
						bool						getBroadPhaseCaps(PxBroadPhaseCaps& caps)														const;
						PxU32						getNbBroadPhaseRegions()																		const;
						PxU32						getBroadPhaseRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;
						PxU32						addBroadPhaseRegion(const PxBroadPhaseRegion& region, bool populateRegion);
						bool						removeBroadPhaseRegion(PxU32 handle);
						void**						getOutOfBoundsAggregates();
						PxU32						getNbOutOfBoundsAggregates();
						void						clearOutOfBoundsAggregates();

		PX_FORCE_INLINE	const PxsMaterialManager&	getMaterialManager()					const	{ return mMaterialManager;				}
		PX_FORCE_INLINE	PxsMaterialManager&			getMaterialManager()							{ return mMaterialManager;				}

		//material management functions to be called from Scb::Scene
						void						registerMaterialInNP(const PxsMaterialCore& materialCore);
						void						updateMaterialInNP(const PxsMaterialCore& materialCore);
						void						unregisterMaterialInNP(const PxsMaterialCore& materialCore);

		// Collision filtering
						void						setFilterShaderData(const void* data, PxU32 dataSize);
		PX_FORCE_INLINE	const void*					getFilterShaderDataFast()				const	{ return mFilterShaderData;				}
		PX_FORCE_INLINE	PxU32						getFilterShaderDataSizeFast()			const	{ return mFilterShaderDataSize;			}
		PX_FORCE_INLINE	PxSimulationFilterShader	getFilterShaderFast()					const	{ return mFilterShader;					}
		PX_FORCE_INLINE	PxSimulationFilterCallback*	getFilterCallbackFast()					const	{ return mFilterCallback;				}
		PX_FORCE_INLINE	PxPairFilteringMode::Enum	getKineKineFilteringMode()				const	{ return mKineKineFilteringMode;		}
		PX_FORCE_INLINE	PxPairFilteringMode::Enum	getStaticKineFilteringMode()			const	{ return mStaticKineFilteringMode;		}

		PX_FORCE_INLINE	PxU32						getTimeStamp()							const	{ return mTimeStamp;					}
		PX_FORCE_INLINE	PxU32						getReportShapePairTimeStamp()			const	{ return mReportShapePairTimeStamp;		}

		PX_FORCE_INLINE	PxReal						getOneOverDt()							const	{ return mOneOverDt;					}
		PX_FORCE_INLINE	PxReal						getDt()									const	{ return mDt;							}

		PX_FORCE_INLINE	const PxVec3&				getGravityFast()						const	{ return mGravity;						}
		PX_FORCE_INLINE	bool						readFlag(SceneInternalFlag::Enum flag)	const	{ return (mInternalFlags & flag) != 0;	}
		PX_FORCE_INLINE	bool						readPublicFlag(PxSceneFlag::Enum flag)	const	{ return (mPublicFlags & flag);			}

		PX_FORCE_INLINE	NPhaseCore*					getNPhaseCore()							const	{ return mNPhaseCore;					}

						void						checkConstraintBreakage();

		PX_FORCE_INLINE	Ps::Array<TriggerPairExtraData>&		
													getTriggerBufferExtraData()						{ return *mTriggerBufferExtraData;		}
		PX_FORCE_INLINE	Ps::Array<PxTriggerPair>&	getTriggerBufferAPI()							{ return mTriggerBufferAPI;				}
						void						reserveTriggerReportBufferSpace(const PxU32 pairCount, PxTriggerPair*& triggerPairBuffer, TriggerPairExtraData*& triggerPairExtraBuffer);

		PX_FORCE_INLINE	ObjectIDTracker&			getRigidIDTracker()								{ return *mRigidIDTracker;				}
		PX_FORCE_INLINE	ObjectIDTracker&			getShapeIDTracker()								{ return *mShapeIDTracker;				}

		PX_FORCE_INLINE	void						markReleasedBodyIDForLostTouch(PxU32 id)		{ mLostTouchPairsDeletedBodyIDs.growAndSet(id); }
						void						resizeReleasedBodyIDMaps(PxU32 maxActors, PxU32 numActors);

		PX_FORCE_INLINE	StaticSim&					getStaticAnchor()								{ return *mStaticAnchor;				}

		PX_FORCE_INLINE	ConstraintProjectionManager& getProjectionManager()							{ return *mProjectionManager;			}

		PX_FORCE_INLINE Bp::BoundsArray&			getBoundsArray()						const	{ return *mBoundsArray; }
		PX_FORCE_INLINE void						updateContactDistance(PxU32 idx, PxReal distance)	{ (*mContactDistance)[idx] = distance; mHasContactDistanceChanged = true; }
		PX_FORCE_INLINE SqBoundsManager&			getSqBoundsManager()					const	{ return *mSqBoundsManager; }

		PX_FORCE_INLINE	PxReal						getVisualizationScale()					const	{ return mVisualizationScale;			}  // This is actually redundant but makes checks whether debug visualization is enabled faster

		PX_FORCE_INLINE BodyCore* const*			getSleepBodiesArray(PxU32& count)				{ count = mSleepBodies.size(); return mSleepBodies.getEntries(); }

		PX_FORCE_INLINE PxTaskManager&				getTaskManager()						const	{ PX_ASSERT(mTaskManager); return *mTaskManager; }

		Cm::FlushPool*								getFlushPool();
	
		PX_FORCE_INLINE bool						getStabilizationEnabled()				const	{ return mEnableStabilization; }

		PX_FORCE_INLINE void						setPostSolverVelocityNeeded()					{ mContactReportsNeedPostSolverVelocity = true; }

									ObjectIDTracker&	getConstraintIDTracker() { return *mConstraintIDTracker; }


									void*			allocateConstraintBlock(PxU32 size);
									void			deallocateConstraintBlock(void* addr, PxU32 size);

		PX_INLINE					 ObjectIDTracker&	getElementIDPool() { return *mElementIDPool; }

		PX_FORCE_INLINE Cm::BitMap&					getVelocityModifyMap() { return mVelocityModifyMap; }

					void							stepSetupCollide(PxBaseTask* continuation);//This is very important to guarantee thread safty in the collide
		PX_FORCE_INLINE void						addToPosePreviewList(BodySim& b)				{ PX_ASSERT(!mPosePreviewBodies.contains(&b)); mPosePreviewBodies.insert(&b); }
		PX_FORCE_INLINE void						removeFromPosePreviewList(BodySim& b)			{ PX_ASSERT(mPosePreviewBodies.contains(&b)); mPosePreviewBodies.erase(&b); }
#if PX_DEBUG
		PX_FORCE_INLINE bool						isInPosePreviewList(BodySim& b)			const	{ return mPosePreviewBodies.contains(&b); }
#endif

		PX_FORCE_INLINE	void						setSpeculativeCCDRigidBody(const PxU32 index) { mSpeculativeCCDRigidBodyBitMap.growAndSet(index); }
		PX_FORCE_INLINE void						resetSpeculativeCCDRigidBody(const PxU32 index) { mSpeculativeCCDRigidBodyBitMap.reset(index); }

		PX_FORCE_INLINE	void						setSpeculativeCCDArticulationLink(const PxU32 index) { mSpeculativeCDDArticulationBitMap.growAndSet(index); }
		PX_FORCE_INLINE void						resetSpeculativeCCDArticulationLink(const PxU32 index) { mSpeculativeCDDArticulationBitMap.reset(index); }

		PX_FORCE_INLINE	PxU64						getContextId() const { return mContextId; }
		PX_FORCE_INLINE bool						isUsingGpuRigidBodies() const { return mUseGpuRigidBodies; }

		//internal private methods:
	private:
					void						releaseConstraints(bool endOfScene);
		PX_INLINE	void						clearBrokenConstraintBuffer();

		/////////////////////////////////////////////////////////////

					void						prepareCollide();

					void						collideStep(PxBaseTask* continuation);
					void						advanceStep(PxBaseTask* continuation);

		// subroutines of collideStep/solveStep:
					void						kinematicsSetup(PxBaseTask* continuation);
					void						stepSetupSolve(PxBaseTask* continuation);
					//void						stepSetupSimulate();

					void						fetchPatchEvents(PxBaseTask*);
					void						processNarrowPhaseTouchEvents();
					void						processNarrowPhaseTouchEventsStage2(PxBaseTask*);
					void						setEdgesConnected(PxBaseTask*);
					void						processNarrowPhaseLostTouchEvents(PxBaseTask*);
					void						processNarrowPhaseLostTouchEventsIslands(PxBaseTask*);
					void						processLostTouchPairs();
					void						integrateKinematicPose();
					void						updateKinematicCached(PxBaseTask* task);

					void						beforeSolver(PxBaseTask* continuation);
					void						checkForceThresholdContactEvents(const PxU32 ccdPass);
					void						processCallbacks(bool pendingReportsOnly = false);
					void						endStep();
	private:
		PX_FORCE_INLINE void					putObjectsToSleep(PxU32 infoFlag);
		PX_FORCE_INLINE void					putInteractionsToSleep();
		PX_FORCE_INLINE void					wakeObjectsUp(PxU32 infoFlag);

					void						collectPostSolverVelocitiesBeforeCCD();
					void						updateFromVisualizationParameters();

					void						clearSleepWakeBodies(void);
		PX_INLINE	void						cleanUpSleepBodies();
		PX_INLINE	void						cleanUpWokenBodies();
		PX_INLINE	void						cleanUpSleepOrWokenBodies(Ps::CoalescedHashSet<BodyCore*>& bodyList, PxU32 removeFlag, bool& validMarker);

		//internal variables:
	private:
		// Material manager
					PX_ALIGN(16, PxsMaterialManager			mMaterialManager);

					PxU64						mContextId;

					Ps::Array<BodyCore*>		mActiveBodies;  // Sorted: kinematic before dynamic
					PxU32						mActiveKinematicBodyCount;  // Number of active kinematics. This is also the index in mActiveBodies where the active dynamic bodies start.
					Ps::Array<BodyCore*>		mActiveCompoundBodies;  

					Ps::Array<Interaction*>		mInteractions[InteractionType::eTRACKED_IN_SCENE_COUNT];
					PxU32						mActiveInteractionCount[InteractionType::eTRACKED_IN_SCENE_COUNT]; // Interactions with id < activeInteractionCount are active

					template <typename T, PxU32 size>
					struct Block
					{
						PxU8 mem[sizeof(T)*size];
						Block() {}	// get around VS warning C4345, otherwise useless
					};

					typedef Block<void*, 8>	PointerBlock8;
					typedef Block<void*, 16> PointerBlock16;
					typedef Block<void*, 32> PointerBlock32;

					Ps::Pool<PointerBlock8>		mPointerBlock8Pool;
					Ps::Pool<PointerBlock16>	mPointerBlock16Pool;
					Ps::Pool<PointerBlock32>	mPointerBlock32Pool;

					PxsContext*					mLLContext;

					Bp::AABBManager*			mAABBManager;
					Bp::BroadPhase*				mBP;
					PxsCCDContext*				mCCDContext;
					PxI32						mNumFastMovingShapes;
					PxU32						mCCDPass;

					//PxsIslandManager*			mIslandManager;

					IG::SimpleIslandManager*		mSimpleIslandManager;

					Dy::Context*				mDynamicsContext;


					PxsMemoryManager*			mMemoryManager;

#if PX_SUPPORT_GPU_PHYSX
					PxsKernelWranglerManager*				mGpuWranglerManagers;
					PxsHeapMemoryAllocatorManager*			mHeapMemoryAllocationManager;
#endif

					PxsSimulationController*	mSimulationController;

					PxsSimulationControllerCallback*	mSimulationControllerCallback;

					PxSceneLimits				mLimits;

					PxVec3						mGravity;			//!< Gravity vector
					PxU32						mBodyGravityDirty; // Set to true before body->updateForces() when the mGravity has changed					

					Ps::Array<PxContactPairHeader>
												mQueuedContactPairHeaders;
		//time:
		//constants set with setTiming():
					PxReal						mDt;						//delta time for current step.
					PxReal						mOneOverDt;					//inverse of dt.
		//stepping / counters:
					PxU32						mTimeStamp;					//Counts number of steps.
					PxU32						mReportShapePairTimeStamp;	//Timestamp for refreshing the shape pair report structure. Updated before delayed shape/actor deletion and before CCD passes.
		//containers:
		// Those ones contain shape ptrs from Actor, i.e. compound level, not subparts

					Ps::CoalescedHashSet<Sc::ConstraintCore*> 
												mConstraints;
												
					Sc::ConstraintProjectionManager*				mProjectionManager;
					Bp::BoundsArray*								mBoundsArray;
					Ps::Array<PxReal, Ps::VirtualAllocator>*		mContactDistance;
					bool											mHasContactDistanceChanged;
					SqBoundsManager*								mSqBoundsManager;

					Ps::Array<BodySim*>			mCcdBodies;
					Ps::Array<BodySim*>			mProjectedBodies;
					Ps::Array<PxTriggerPair>	mTriggerBufferAPI;
					Ps::Array<TriggerPairExtraData>*		
												mTriggerBufferExtraData;

					PxU32						mRemovedShapeCountAtSimStart;  // counter used to detect whether there have been buffered shape removals

					Ps::CoalescedHashSet<ArticulationCore*> mArticulations;

					Ps::Array<Sc::ConstraintCore*>	mBrokenConstraints;
					Ps::CoalescedHashSet<Sc::ConstraintSim*> mActiveBreakableConstraints;

					// pools for joint buffers
					// Fixed joint is 92 bytes, D6 is 364 bytes right now. So these three pools cover all the internal cases
					typedef Block<PxU8, 128> MemBlock128;
					typedef Block<PxU8, 256> MemBlock256;
					typedef Block<PxU8, 384> MemBlock384;

					Ps::Pool2<MemBlock128, 8192>	mMemBlock128Pool;
					Ps::Pool2<MemBlock256, 8192>	mMemBlock256Pool;
					Ps::Pool2<MemBlock384, 8192>	mMemBlock384Pool;


					// broad phase data:
					NPhaseCore*					mNPhaseCore;

					// Collision filtering
					void*						mFilterShaderData;
					PxU32						mFilterShaderDataSize;
					PxU32						mFilterShaderDataCapacity;
					PxSimulationFilterShader	mFilterShader;
					PxSimulationFilterCallback*	mFilterCallback;

					PxPairFilteringMode::Enum	mKineKineFilteringMode;
					PxPairFilteringMode::Enum	mStaticKineFilteringMode;

					Ps::CoalescedHashSet<BodyCore*> mSleepBodies;
					Ps::CoalescedHashSet<BodyCore*> mWokeBodies;
					bool						mWokeBodyListValid;
					bool						mSleepBodyListValid;
					bool						mEnableStabilization;
					Ps::Array<Client*>			mClients;	//an array of transform arrays, one for each client.

						Ps::Array<PxActor*>				mActiveActors;
						Ps::Array<PxActor*>				mFrozenActors;

						Ps::Array<const PxRigidBody*>	mClientPosePreviewBodies;	// buffer for bodies that requested early report of the integrated pose (eENABLE_POSE_INTEGRATION_PREVIEW).
																			// This buffer gets exposed to users. Is officially accessible from PxSimulationEventCallback::onAdvance()
																			// until the next simulate()/advance().
						Ps::Array<PxTransform>			mClientPosePreviewBuffer;	// buffer of newly integrated poses for the bodies that requested a preview. This buffer gets exposed
																			// to users.

						PxSimulationEventCallback*	mSimulationEventCallback;
						PxBroadPhaseCallback*		mBroadPhaseCallback;

					SimStats*					mStats;
					PxU32						mInternalFlags;	//!< Combination of ::SceneFlag
					PxSceneFlags				mPublicFlags;	//copy of PxSceneDesc::flags, of type PxSceneFlag

					ObjectIDTracker*			mConstraintIDTracker;
					ObjectIDTracker*			mShapeIDTracker;
					ObjectIDTracker*			mRigidIDTracker;
					ObjectIDTracker*			mElementIDPool;

					StaticSim*					mStaticAnchor;

					Cm::PreallocatingPool<ShapeSim>*	mShapeSimPool;
					Cm::PreallocatingPool<StaticSim>*	mStaticSimPool;
					Cm::PreallocatingPool<BodySim>*		mBodySimPool;
					Ps::Pool<ConstraintSim>*			mConstraintSimPool;
					LLArticulationPool*					mLLArticulationPool;
					LLArticulationRCPool*				mLLArticulationRCPool;
														
					Ps::Pool<ConstraintInteraction>*
												mConstraintInteractionPool;

					Ps::Pool<SimStateData>*		mSimStateDataPool;

					BatchRemoveState*			mBatchRemoveState;

					Ps::Array<SimpleBodyPair>	mLostTouchPairs;
					Cm::BitMap					mLostTouchPairsDeletedBodyIDs;	// Need to know which bodies have been deleted when processing the lost touch pair list.
																				// Can't use the existing rigid object ID tracker class since this map needs to be cleared at
																				// another point in time.

					Cm::BitMap					mVelocityModifyMap;

					Ps::Array<PxvContactManagerTouchEvent> mTouchFoundEvents;
					Ps::Array<PxvContactManagerTouchEvent> mTouchLostEvents;

					Ps::Array<PxsContactManager*> mFoundPatchManagers;
					Ps::Array<PxsContactManager*> mLostPatchManagers;

					Ps::Array<PxU32>			mOutOfBoundsIDs;

					Cm::BitMap					mDirtyShapeSimMap;

					PxU32						mDominanceBitMatrix[PX_MAX_DOMINANCE_GROUP];

					PxReal						mVisualizationScale;  // Redundant but makes checks whether debug visualization is enabled faster

					bool						mVisualizationParameterChanged;

					// statics:
					PxU32						mNbRigidStatics;
					PxU32						mNbRigidDynamics;
					PxU32						mNbGeometries[PxGeometryType::eGEOMETRY_COUNT];

					PxU32						mNumDeactivatingNodes[2];

					// task decomposition
					void						preBroadPhase(PxBaseTask* continuation);
					void						broadPhase(PxBaseTask* continuation);
					void						postBroadPhase(PxBaseTask* continuation);
					void						postBroadPhaseContinuation(PxBaseTask* continuation);
					void						preRigidBodyNarrowPhase(PxBaseTask* continuation);
					void						postBroadPhaseStage2(PxBaseTask* continuation);
					void						postBroadPhaseStage3(PxBaseTask* continuation);
					void						rigidBodyNarrowPhase(PxBaseTask* continuation);
					void						unblockNarrowPhase(PxBaseTask* continuation);
					void						islandGen(PxBaseTask* continuation);
					void						processLostSolverPatches(PxBaseTask* continuation);
					void						postIslandGen(PxBaseTask* continuation);
					void						processTriggerInteractions(PxBaseTask* continuation);
					void						solver(PxBaseTask* continuation);
					void						updateBodiesAndShapes(PxBaseTask* continuation);
					void						updateSimulationController(PxBaseTask* continuation);
					void						updateDynamics(PxBaseTask* continuation);
					void						processLostContacts(PxBaseTask*);
					void						processLostContacts2(PxBaseTask*);
					void						processLostContacts3(PxBaseTask*);
					void						destroyManagers(PxBaseTask*);
					void						lostTouchReports(PxBaseTask*);
					void						unregisterInteractions(PxBaseTask*);
					void						postThirdPassIslandGen(PxBaseTask*);
					void						postSolver(PxBaseTask* continuation);
					void						constraintProjection(PxBaseTask* continuation);
					void						afterIntegration(PxBaseTask* continuation);  // performs sleep check, for instance
					void						postCCDPass(PxBaseTask* continuation);
					void						ccdBroadPhaseAABB(PxBaseTask* continuation);
					void						ccdBroadPhase(PxBaseTask* continuation);
					void						updateCCDMultiPass(PxBaseTask* continuation);
					void						updateCCDSinglePass(PxBaseTask* continuation);
					void						updateCCDSinglePassStage2(PxBaseTask* continuation);
					void						updateCCDSinglePassStage3(PxBaseTask* continuation);
					void						finalizationPhase(PxBaseTask* continuation);

					void						postNarrowPhase(PxBaseTask* continuation);

					void						addShapes(void *const* shapes, PxU32 nbShapes, size_t ptrOffset, RigidSim& sim, PxBounds3* outBounds);
					void						removeShapes(RigidSim& , Ps::InlineArray<Sc::ShapeSim*, 64>& , Ps::InlineArray<const Sc::ShapeCore*, 64>&, bool wakeOnLostTouch);


	private:

					void						addShapes(void *const* shapes, PxU32 nbShapes, size_t ptrOffset, RigidSim& sim, ShapeSim*& prefetchedShapeSim, PxBounds3* outBounds);

					Cm::DelegateTask<Sc::Scene, &Sc::Scene::secondPassNarrowPhase>			mSecondPassNarrowPhase;
					Cm::DelegateFanoutTask<Sc::Scene, &Sc::Scene::postNarrowPhase>			mPostNarrowPhase;
					Cm::DelegateFanoutTask<Sc::Scene, &Sc::Scene::finalizationPhase>	mFinalizationPhase;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDMultiPass>			mUpdateCCDMultiPass;

					//multi-pass ccd stuff
					Ps::Array<Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePass> >	mUpdateCCDSinglePass;
					Ps::Array<Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePassStage2> >	mUpdateCCDSinglePass2;
					Ps::Array<Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePassStage3> >	mUpdateCCDSinglePass3;
					Ps::Array<Cm::DelegateTask<Sc::Scene, &Sc::Scene::ccdBroadPhaseAABB> >			mCCDBroadPhaseAABB;
					Ps::Array<Cm::DelegateTask<Sc::Scene, &Sc::Scene::ccdBroadPhase> >			mCCDBroadPhase;
					Ps::Array<Cm::DelegateTask<Sc::Scene, &Sc::Scene::postCCDPass> >			mPostCCDPass;
					PxU32																		mCurrentCCDTask;

					Cm::DelegateTask<Sc::Scene, &Sc::Scene::afterIntegration>				mAfterIntegration;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::constraintProjection>			mConstraintProjection;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::postSolver>						mPostSolver;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::solver>							mSolver;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateBodiesAndShapes>			mUpdateBodiesAndShapes;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateSimulationController>		mUpdateSimulationController;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateDynamics>					mUpdateDynamics;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::processLostContacts>			mProcessLostContactsTask;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::processLostContacts2>			mProcessLostContactsTask2;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::processLostContacts3>			mProcessLostContactsTask3;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::destroyManagers>				mDestroyManagersTask;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::lostTouchReports>				mLostTouchReportsTask;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::unregisterInteractions>			mUnregisterInteractionsTask;
					Cm::DelegateTask<Sc::Scene,
						&Sc::Scene::processNarrowPhaseLostTouchEventsIslands>				mProcessNarrowPhaseLostTouchTasks;
					Cm::DelegateTask<Sc::Scene,
						&Sc::Scene::processNarrowPhaseLostTouchEvents>						mProcessNPLostTouchEvents;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::postThirdPassIslandGen>			mPostThirdPassIslandGenTask;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::postIslandGen>					mPostIslandGen;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::islandGen>						mIslandGen;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::preRigidBodyNarrowPhase>		mPreRigidBodyNarrowPhase;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::setEdgesConnected>				mSetEdgesConnectedTask;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::fetchPatchEvents>				mFetchPatchEventsTask;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::processLostSolverPatches>		mProcessLostPatchesTask;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::rigidBodyNarrowPhase>			mRigidBodyNarrowPhase;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::unblockNarrowPhase>				mRigidBodyNPhaseUnlock;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::postBroadPhase>					mPostBroadPhase;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::postBroadPhaseContinuation>		mPostBroadPhaseCont;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::postBroadPhaseStage2>			mPostBroadPhase2;
					Cm::DelegateFanoutTask<Sc::Scene, &Sc::Scene::postBroadPhaseStage3>		mPostBroadPhase3;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::preallocateContactManagers>		mPreallocateContactManagers;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::islandInsertion>				mIslandInsertion;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::registerContactManagers>		mRegisterContactManagers;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::registerInteractions>			mRegisterInteractions;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::registerSceneInteractions>		mRegisterSceneInteractions;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::broadPhase>						mBroadPhase;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::advanceStep>					mAdvanceStep;
					Cm::DelegateTask<Sc::Scene, &Sc::Scene::collideStep>					mCollideStep;

					Cm::FlushPool															mTaskPool;
					PxTaskManager*															mTaskManager;

					bool																	mContactReportsNeedPostSolverVelocity;
					bool																	mUseGpuRigidBodies;

					SimulationStage::Enum													mSimulationStage;

					ConstraintGroupNode**													mTmpConstraintGroupRootBuffer;  // temporary list of constraint group roots, used for constraint projection

					Ps::CoalescedHashSet<const BodySim*>									mPosePreviewBodies;  // list of bodies that requested early report of the integrated pose (eENABLE_POSE_INTEGRATION_PREVIEW).

					Ps::Array<PxsContactManager*>											mPreallocatedContactManagers;
					Ps::Array<ShapeInteraction*>											mPreallocatedShapeInteractions;
					Ps::Array<ElementInteractionMarker*>									mPreallocatedInteractionMarkers;

					OverlapFilterTask*														mOverlapFilterTaskHead;
					Ps::Array<PxFilterInfo>													mFilterInfo;
					Cm::BitMap																mSpeculativeCCDRigidBodyBitMap;
					Cm::BitMap																mSpeculativeCDDArticulationBitMap;
	};

	bool	activateInteraction(Sc::Interaction* interaction, void* data);
	bool	deactivateInteraction(Sc::Interaction* interaction);

} // namespace Sc

}

#endif
