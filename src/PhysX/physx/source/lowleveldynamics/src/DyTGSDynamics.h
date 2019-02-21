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


#ifndef DY_TGS_DYNAMICS_H
#define DY_TGS_DYNAMICS_H

#include "PxvConfig.h"
#include "CmSpatialVector.h"
#include "CmTask.h"
#include "CmPool.h"
#include "PxcThreadCoherentCache.h"
#include "DyThreadContext.h"
#include "PxcConstraintBlockStream.h"
#include "DySolverBody.h"
#include "DyContext.h"
#include "PxsIslandManagerTypes.h"
#include "PxvNphaseImplementationContext.h"
#include "solver/PxSolverDefs.h"
#include "PxsIslandSim.h"

namespace physx
{

	namespace Cm
	{
		class FlushPool;
	}

	namespace IG
	{
		class SimpleIslandManager;
		struct Edge;
	}

	class PxsRigidBody;

	class PxsStreamedThresholdTable;

	struct PxsBodyCore;
	struct PxsIslandObjects;
	class PxsIslandIndices;
	struct PxsIndexedInteraction;
	class PxsIslandManager;
	struct PxsIndexedConstraint;
	struct PxsIndexedContactManager;
	class PxsHeapMemoryAllocator;
	class PxsMemoryManager;
	class PxsDefaultMemoryManager;
	

	namespace Cm
	{
		class Bitmap;
		class SpatialVector;
	}

	namespace Dy
	{
		class SolverCore;
		struct SolverIslandParams;
		struct ArticulationSolverDesc;
		class ArticulationV;
		class DynamicsContext;
		struct SolverContext;

		struct PxTGSSolverConstraintDesc
		{
			static const PxU16 NO_LINK = 0xffff;

			enum ConstraintType
			{
				eCONTACT_CONSTRAINT,				//!< Defines this pair is a contact constraint
				eJOINT_CONSTRAINT					//!< Defines this pair is a joint constraint
			};

			union
			{
				PxTGSSolverBodyVel*	bodyA;			//!< bodyA pointer
				Dy::ArticulationV*		articulationA;	//!< Articulation pointer for body A
			};

			union
			{
				PxTGSSolverBodyVel*	bodyB;			//!< BodyB pointer
				Dy::ArticulationV*		articulationB;	//!< Articulation pointer for body B
			};

			PxU16					linkIndexA;
			PxU16					linkIndexB;
			union
			{
				PxU16				articulationALength;
				PxU32				bodyAIdx;
			};
			union
			{
				PxU16				articulationBLength;
				PxU32				bodyBIdx;
			};

			PxU16					writeBackLengthOver4;	//!< writeBackLength/4, max writeback length is 256K, allows PxSolverConstraintDesc to fit in 32 bytes
			PxU16					constraintLengthOver16;	//!< constraintLength/16, max constraint length is 1MB, allows PxSolverConstraintDesc to fit in 32 bytes

			PxU8*					constraint;				//!< Pointer to the constraint rows to be solved
			void*					writeBack;				//!< Pointer to the writeback structure results for this given constraint are to be written to

		};

		struct SolverIslandObjectsStep
		{
			PxsRigidBody**				bodies;
			ArticulationV**				articulations;
			ArticulationV**				articulationOwners;
			PxsIndexedContactManager*	contactManagers;

			const IG::IslandId*			islandIds;
			PxU32						numIslands;
			PxU32*						bodyRemapTable;
			PxU32*						nodeIndexArray;

			PxTGSSolverConstraintDesc*	constraintDescs;
			PxTGSSolverConstraintDesc*	orderedConstraintDescs;
			PxTGSSolverConstraintDesc*	tempConstraintDescs;
			PxConstraintBatchHeader*	constraintBatchHeaders;
			Cm::SpatialVector*			motionVelocities;
			PxsBodyCore**				bodyCoreArray;

			PxU32						solverBodyOffset;

			SolverIslandObjectsStep() : bodies(NULL), articulations(NULL), articulationOwners(NULL),
				contactManagers(NULL), islandIds(NULL), numIslands(0), nodeIndexArray(NULL), constraintDescs(NULL), motionVelocities(NULL), bodyCoreArray(NULL),
				solverBodyOffset(0)
			{
			}
		};

		struct IslandContextStep
		{
			//The thread context for this island (set in in the island start task, released in the island end task)
			ThreadContext*		mThreadContext;
			PxsIslandIndices	mCounts;
			SolverIslandObjectsStep mObjects;
			PxU32				mPosIters;
			PxU32				mVelIters;
			PxU32				mArticulationOffset;
			PxReal				mStepDt;
			PxReal				mInvStepDt;
			PxI32				mSharedSolverIndex;
			PxI32				mSolvedCount;
			PxI32				mSharedRigidBodyIndex;
			PxI32				mRigidBodyIntegratedCount;
			PxI32				mSharedArticulationIndex;
			PxI32				mArticulationIntegratedCount;
		};


		

		struct PxTGSSolverConstraintPrepDescBase
		{
			PxConstraintInvMassScale mInvMassScales;	//!< In: The local mass scaling for this pair.

			PxTGSSolverConstraintDesc* desc;				//!< Output: The PxSolverConstraintDesc filled in by contact prep

			const PxTGSSolverBodyVel* body0;					//!< In: The first body. Stores velocity information. Unused unless contact involves articulations.
			const PxTGSSolverBodyVel *body1;					//!< In: The second body. Stores velocity information. Unused unless contact involves articulations.

			const PxTGSSolverBodyTxInertia* body0TxI;
			const PxTGSSolverBodyTxInertia* body1TxI;

			const PxStepSolverBodyData* bodyData0;
			const PxStepSolverBodyData* bodyData1;

			PxTransform bodyFrame0;						//!< In: The world-space transform of the first body.
			PxTransform bodyFrame1;						//!< In: The world-space transform of the second body.

			PxSolverContactDesc::BodyState bodyState0;						//!< In: Defines what kind of actor the first body is
			PxSolverContactDesc::BodyState bodyState1;						//!< In: Defines what kind of actor the second body is

		};

		struct PxTGSSolverConstraintPrepDesc : public PxTGSSolverConstraintPrepDescBase
		{
			Px1DConstraint* rows;			//!< The start of the constraint rows
			PxU32 numRows;								//!< The number of rows

			PxReal linBreakForce, angBreakForce;		//!< Break forces
			PxReal minResponseThreshold;				//!< The minimum response threshold
			void* writeback;							//!< Pointer to constraint writeback structure. Reports back joint breaking. If not required, set to NULL.
			bool disablePreprocessing;					//!< Disable joint pre-processing. Pre-processing can improve stability but under certain circumstances, e.g. when some invInertia rows are zero/almost zero, can cause instabilities.	
			bool improvedSlerp;							//!< Use improved slerp model
			bool driveLimitsAreForces;					//!< Indicates whether drive limits are forces
			bool extendedLimits;						//!< Indiciates whether extended limits are used

			PxVec3 body0WorldOffset;					//!< Body0 world offset
			PxVec3 cA2w;								//!< Location of anchor point A in world space
			PxVec3 cB2w;								//!< Location of anchor point B in world space
		};


		struct PxTGSSolverContactDesc : public PxTGSSolverConstraintPrepDescBase
		{

			Sc::ShapeInteraction* shapeInteraction; //!< Pointer to share interaction. Used for force threshold reports in solver. Set to NULL if using immediate mode.
			Gu::ContactPoint* contacts;				//!< The start of the contacts for this pair
			PxU32 numContacts;						//!< The total number of contacs this pair references.

			bool hasMaxImpulse;						//!< Defines whether this pairs has maxImpulses clamping enabled
			bool disableStrongFriction;				//!< Defines whether this pair disables strong friction (sticky friction correlation)
			bool hasForceThresholds;				//!< Defines whether this pair requires force thresholds	

			PxReal restDistance;					//!< A distance at which the solver should aim to hold the bodies separated. Default is 0
			PxReal maxCCDSeparation;				//!< A distance used to configure speculative CCD behavior. Default is PX_MAX_F32. Set internally in PhysX for bodies with eENABLE_SPECULATIVE_CCD on. Do not set directly!

			PxU8* frictionPtr;						//!< InOut: Friction patch correlation data. Set each frame by solver. Can be retained for improved behaviour or discarded each frame.
			PxU8 frictionCount;						//!< The total number of friction patches in this pair

			PxReal* contactForces;					//!< Out: A buffer for the solver to write applied contact forces to.

			PxU32 startFrictionPatchIndex;			//!< Start index of friction patch in the correlation buffer. Set by friction correlation
			PxU32 numFrictionPatches;				//!< Total number of friction patches in this pair. Set by friction correlation

			PxU32 startContactPatchIndex;			//!< The start index of this pair's contact patches in the correlation buffer. For internal use only
			PxU16 numContactPatches;				//!< Total number of contact patches.
			PxU16 axisConstraintCount;				//!< Axis constraint count. Defines how many constraint rows this pair has produced. Useful for statistical purposes.

			PxReal maxImpulse;

			PxReal torsionalPatchRadius;
			PxReal minTorsionalPatchRadius;
		};



		struct SolverIslandObjectsStep;

		class SolverBodyVelDataPool : public Ps::Array<PxTGSSolverBodyVel, Ps::AlignedAllocator<128, Ps::ReflectionAllocator<PxTGSSolverBodyVel> > >
		{
			PX_NOCOPY(SolverBodyVelDataPool)
		public:
			SolverBodyVelDataPool() {}
		};

		class SolverBodyTxInertiaPool : public Ps::Array<PxTGSSolverBodyTxInertia, Ps::AlignedAllocator<128, Ps::ReflectionAllocator<PxTGSSolverBodyTxInertia> > >
		{
			PX_NOCOPY(SolverBodyTxInertiaPool)
		public:
			SolverBodyTxInertiaPool() {}
		};

		class SolverBodyDataStepPool : public Ps::Array<PxStepSolverBodyData, Ps::AlignedAllocator<128, Ps::ReflectionAllocator<PxStepSolverBodyData> > >
		{
			PX_NOCOPY(SolverBodyDataStepPool)
		public:
			SolverBodyDataStepPool() {}
		};

		

		class SolverStepConstraintDescPool : public Ps::Array<PxTGSSolverConstraintDesc, Ps::AlignedAllocator<128, Ps::ReflectionAllocator<PxTGSSolverConstraintDesc> > >
		{
			PX_NOCOPY(SolverStepConstraintDescPool)
		public:
			SolverStepConstraintDescPool() { }
		};


#if PX_VC 
#pragma warning(push)
#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif


		class DynamicsTGSContext : public Context
		{
			PX_NOCOPY(DynamicsTGSContext)
		public:

			/**PxBaseTask* continuation
			\brief Creates a DynamicsContext associated with a PxsContext
			\return A pointer to the newly-created DynamicsContext.
			*/
			static DynamicsTGSContext*	create(PxcNpMemBlockPool* memBlockPool,
				PxcScratchAllocator& scratchAllocator,
				Cm::FlushPool& taskPool,
				PxvSimStats& simStats,
				PxTaskManager* taskManager,
				Ps::VirtualAllocatorCallback* allocator,
				PxsMaterialManager* materialManager,
				IG::IslandSim* accurateIslandSim,
				PxU64 contextID,
				const bool enableStabilization,
				const bool useEnhancedDeterminism,
				const bool useAdaptiveForce,
				const PxReal lengthScale
				);

			/**
			\brief Destroys this DynamicsContext
			*/
			void						destroy();

			/**
			\brief Returns the static world solver body
			\return The static world solver body.
			*/
			//PX_FORCE_INLINE PxSolverBody&		getWorldSolverBody()					{ return mWorldSolverBody; }

			PX_FORCE_INLINE Cm::FlushPool&			getTaskPool()						{ return mTaskPool; }

			PX_FORCE_INLINE ThresholdStream&		getThresholdStream()					{ return *mThresholdStream; }

			PX_FORCE_INLINE PxvSimStats&			getSimStats()							{ return mSimStats; }

#if PX_ENABLE_SIM_STATS
			void									addThreadStats(const ThreadContext::ThreadSimStats& stats);
#endif

			/**
			\brief The entry point for the constraint solver.
			\param[in]	dt	The simulation time-step
			\param[in]	continuation The continuation task for the solver

			This method is called after the island generation has completed. Its main responsibilities are:
			(1) Reserving the solver body pools
			(2) Initializing the static and kinematic solver bodies, which are shared resources between islands.
			(3) Construct the solver task chains for each island

			Each island is solved as an independent solver task chain in parallel.

			*/

			virtual void						update(IG::SimpleIslandManager& simpleIslandManager, PxBaseTask* continuation, PxBaseTask* lostTouchTask,
				PxsContactManager** foundPatchManagers, PxU32 nbFoundPatchManagers, PxsContactManager** lostPatchManagers, PxU32 nbLostPatchManagers,
				PxU32 maxPatchesPerCM, PxsContactManagerOutputIterator& iter, PxsContactManagerOutput* gpuOutputs, const PxReal dt, const PxVec3& gravity, const PxU32 bitMapWordCounts);

			void updatePostKinematic(IG::SimpleIslandManager& simpleIslandManager, PxBaseTask* continuation, 
				PxBaseTask* lostTouchTask);

			virtual void						processLostPatches(IG::SimpleIslandManager& /*simpleIslandManager*/, PxsContactManager** /*lostPatchManagers*/, PxU32 /*nbLostPatchManagers*/, PxsContactManagerOutputIterator& /*iterator*/){}

			virtual void						updateBodyCore(PxBaseTask* continuation);

			virtual void						setSimulationController(PxsSimulationController* simulationController){ mSimulationController = simulationController; }
			/**
			\brief This method combines the results of several islands, e.g. constructing scene-level simulation statistics and merging together threshold streams for contact notification.
			*/
			virtual void							mergeResults();

			virtual void							getDataStreamBase(void*& /*contactStreamBase*/, void*& /*patchStreamBase*/, void*& /*forceAndIndicesStreamBase*/){}

			/**
			\brief Allocates and returns a thread context object.
			\return A thread context.
			*/
			PX_FORCE_INLINE ThreadContext*					getThreadContext()
			{
				return mThreadContextPool.get();
			}

			/**
			\brief Returns a thread context to the thread context pool.
			\param[in] context The thread context to return to the thread context pool.
			*/
			void								putThreadContext(ThreadContext* context)
			{
				mThreadContextPool.put(context);
			}


			PX_FORCE_INLINE	PxU32					getKinematicCount()		const	{ return mKinematicCount; }
			PX_FORCE_INLINE	PxU64					getContextId()			const	{ return mContextID; }

		protected:

			/**
			\brief Constructor for DynamicsContext
			*/
			DynamicsTGSContext(PxcNpMemBlockPool* memBlockPool,
				PxcScratchAllocator& scratchAllocator,
				Cm::FlushPool& taskPool,
				PxvSimStats& simStats,
				PxTaskManager* taskManager,
				Ps::VirtualAllocatorCallback* allocator,
				PxsMaterialManager* materialManager,
				IG::IslandSim* accurateIslandSim,
				PxU64 contextID,
				const bool enableStabilization,
				const bool useEnhancedDeterminism,
				const bool useAdaptiveForce,
				const PxReal lengthScale
				);
			/**
			\brief Destructor for DynamicsContext
			*/
			virtual								~DynamicsTGSContext();


			// Solver helper-methods
			/**
			\brief Computes the unconstrained velocity for a given PxsRigidBody
			\param[in] atom The PxsRigidBody
			*/
			void								computeUnconstrainedVelocity(PxsRigidBody* atom)	const;

			/**
			\brief fills in a PxSolverConstraintDesc from an indexed interaction
			\param[in,out] desc The PxSolverConstraintDesc
			\param[in] constraint The PxsIndexedInteraction
			*/
			void								setDescFromIndices(PxTGSSolverConstraintDesc& desc,
				const PxsIndexedInteraction& constraint, const PxU32 solverBodyOffset, PxTGSSolverBodyVel* solverBodies);


			void								setDescFromIndices(PxTGSSolverConstraintDesc& desc, IG::EdgeIndex edgeIndex,
				const IG::SimpleIslandManager& islandManager, PxU32* bodyRemapTable, const PxU32 solverBodyOffset, PxTGSSolverBodyVel* solverBodies);


			void solveIsland(const SolverIslandObjectsStep& objects,
				const PxsIslandIndices& counts,
				const PxU32 solverBodyOffset,
				IG::SimpleIslandManager& islandManager,
				PxU32* bodyRemapTable, PxsMaterialManager* materialManager,
				PxsContactManagerOutputIterator& iterator,
				PxBaseTask* continuation);

			void prepareBodiesAndConstraints(const SolverIslandObjectsStep& objects,
				IG::SimpleIslandManager& islandManager,
				IslandContextStep& islandContext);

			void setupDescs(IslandContextStep& islandContext, const SolverIslandObjectsStep& objects, IG::SimpleIslandManager& mIslandManager, PxU32* mBodyRemapTable, PxU32 mSolverBodyOffset,
				PxsContactManagerOutputIterator& outputs);

			void preIntegrateBodies(PxsBodyCore** bodyArray, PxsRigidBody** originalBodyArray,
				PxTGSSolverBodyVel* solverBodyVelPool, PxTGSSolverBodyTxInertia* solverBodyTxInertia, PxStepSolverBodyData* solverBodyDataPool2,
				PxU32* nodeIndexArray, const PxU32 bodyCount, const PxVec3& gravity, const PxReal dt, PxU32& posIters, PxU32& velIters, PxU32 iteration);

			void setupArticulations(IslandContextStep& islandContext, const PxVec3& gravity, const PxReal dt, PxU32& posIters, PxU32& velIters, PxBaseTask* continuation);

			PxU32 setupArticulationInternalConstraints(IslandContextStep& islandContext, PxReal dt, PxReal invStepDt, PxTGSSolverConstraintDesc* constraintDescs);

			void createSolverConstraints(PxTGSSolverConstraintDesc* contactDescPtr, PxConstraintBatchHeader* headers, const PxU32 nbHeaders,
				PxsContactManagerOutputIterator& outputs, Dy::ThreadContext& islandThreadContext, Dy::ThreadContext& threadContext, PxReal stepDt, PxReal totalDt, 
				PxReal invStepDt, PxU32 nbSubsteps);

			void solveConstraintsIteration(const PxTGSSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, const PxU32 nbHeaders, bool doFriction, PxReal invStepDt,
				const PxTGSSolverBodyTxInertia* const solverTxInertia, const PxReal elapsedTime, const PxReal minPenetration, SolverContext& cache);

			void solveConcludeConstraintsIteration(const PxTGSSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, const PxU32 nbHeaders,
				PxTGSSolverBodyTxInertia* solverTxInertia, const PxReal elapsedTime, SolverContext& cache);

			void parallelSolveConstraints(const PxTGSSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, const PxU32 nbHeaders, bool doFriction, PxTGSSolverBodyTxInertia* solverTxInertia,
				const PxReal elapsedTime, const PxReal minPenetration, SolverContext& cache);

			void writebackConstraintsIteration(const PxConstraintBatchHeader* const hdrs, const PxTGSSolverConstraintDesc* const contactDescPtr, const PxU32 nbHeaders);

			void parallelWritebackConstraintsIteration(const PxTGSSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, const PxU32 nbHeaders);

			void integrateBodies(const SolverIslandObjectsStep& objects,
				const PxU32 count, PxTGSSolverBodyVel* vels,
				PxTGSSolverBodyTxInertia* txInertias, const PxStepSolverBodyData*const bodyDatas, PxReal dt);

			void parallelIntegrateBodies(PxTGSSolverBodyVel* vels, PxTGSSolverBodyTxInertia* txInertias,
				const PxStepSolverBodyData* const bodyDatas, const PxU32 count, PxReal dt);

			void copyBackBodies(const SolverIslandObjectsStep& objects,
				PxTGSSolverBodyVel* vels, PxTGSSolverBodyTxInertia* txInertias,
				PxStepSolverBodyData* solverBodyData, PxReal invDt,	IG::IslandSim& islandSim,
				PxU32 startIdx, PxU32 endIdx);

			void updateArticulations(Dy::ThreadContext& threadContext, const PxU32 startIdx, const PxU32 endIdx, PxReal dt);

			void stepArticulations(Dy::ThreadContext& threadContext, const PxsIslandIndices& counts, PxReal dt);

			void iterativeSolveIsland(const SolverIslandObjectsStep& objects, const PxsIslandIndices& counts, ThreadContext& mThreadContext,
				const PxReal stepDt, const PxReal invStepDt, const PxU32 posIters, const PxU32 velIters, SolverContext& cache);

			void iterativeSolveIslandParallel(const SolverIslandObjectsStep& objects, const PxsIslandIndices& counts, ThreadContext& mThreadContext,
				const PxReal stepDt, const PxU32 posIters, const PxU32 velIters, PxI32* solverCounts, PxI32* integrationCounts, PxI32* articulationIntegrationCounts,
				PxI32* solverProgressCount, PxI32* integrationProgressCount, PxI32* articulationProgressCount, PxU32 solverUnrollSize, PxU32 integrationUnrollSize);

			void endIsland(ThreadContext& mThreadContext);

			void finishSolveIsland(ThreadContext& mThreadContext, const SolverIslandObjectsStep& objects,
				const PxsIslandIndices& counts, IG::SimpleIslandManager& islandManager, PxBaseTask* continuation);




			/**
			\brief Resets the thread contexts
			*/
			void									resetThreadContexts();

			/**
			\brief Returns the scratch memory allocator.
			\return The scratch memory allocator.
			*/
			PX_FORCE_INLINE PxcScratchAllocator&	getScratchAllocator() { return mScratchAllocator; }

			//Data

			PxTGSSolverBodyVel						mWorldSolverBodyVel;
			PxTGSSolverBodyTxInertia				mWorldSolverBodyTxInertia;
			PxStepSolverBodyData					mWorldSolverBodyData2;

			/**
			\brief A thread context pool
			*/
			PxcThreadCoherentCache<ThreadContext, PxcNpMemBlockPool> mThreadContextPool;

			/**
			\brief Solver constraint desc array
			*/
			SolverStepConstraintDescPool	mSolverConstraintDescPool;

			SolverStepConstraintDescPool	mOrderedSolverConstraintDescPool;

			SolverStepConstraintDescPool	mTempSolverConstraintDescPool;

			Ps::Array<PxConstraintBatchHeader> mContactConstraintBatchHeaders;

			/**
			\brief Array of motion velocities for all bodies in the scene.
			*/
			Ps::Array<Cm::SpatialVector> mMotionVelocityArray;

			/**
			\brief Array of body core pointers for all bodies in the scene.
			*/
			Ps::Array<PxsBodyCore*>	mBodyCoreArray;

			/**
			\brief Array of rigid body pointers for all bodies in the scene.
			*/
			Ps::Array<PxsRigidBody*> mRigidBodyArray;

			/**
			\brief Array of articulationpointers for all articulations in the scene.
			*/
			Ps::Array<ArticulationV*> mArticulationArray;

			SolverBodyVelDataPool		mSolverBodyVelPool;

			SolverBodyTxInertiaPool		mSolverBodyTxInertiaPool;

			SolverBodyDataStepPool		mSolverBodyDataPool2;


			ThresholdStream*		mExceededForceThresholdStream[2]; //this store previous and current exceeded force thresholdStream	

			Ps::Array<PxU32>		mExceededForceThresholdStreamMask;

			Ps::Array<PxU32>		mSolverBodyRemapTable;				//Remaps from the "active island" index to the index within a solver island

			Ps::Array<PxU32>		mNodeIndexArray;					//island node index

			Ps::Array<PxsIndexedContactManager> mContactList;

			/**
			\brief The total number of kinematic bodies in the scene
			*/
			PxU32						mKinematicCount;

			/**
			\brief Atomic counter for the number of threshold stream elements.
			*/
			PxI32						mThresholdStreamOut;



			PxsMaterialManager*			mMaterialManager;

			PxsContactManagerOutputIterator mOutputIterator;

			PxReal						mLengthScale;

		private:
			//private:
			PxcScratchAllocator&						mScratchAllocator;
			Cm::FlushPool&								mTaskPool;
			PxTaskManager*								mTaskManager;
			PxU32										mCurrentIndex; // this is the index point to the current exceeded force threshold stream

			PxU64										mContextID;

			friend class SetupDescsTask;
			friend class PreIntegrateTask;
			friend class SetupArticulationTask;
			friend class SetupArticulationInternalConstraintsTask;
			friend class SetupSolverConstraintsTask;
			friend class SolveIslandTask;
			friend class EndIslandTask;
			friend class SetupSolverConstraintsSubTask;
			friend class ParallelSolveTask;
			friend class PreIntegrateParallelTask;
			friend class CopyBackTask;
			friend class UpdateArticTask;
			friend class FinishSolveIslandTask;
		};

#if PX_VC 
#pragma warning(pop)
#endif

	}
}

#endif

