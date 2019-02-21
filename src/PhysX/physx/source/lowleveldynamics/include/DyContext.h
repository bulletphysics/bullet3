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

#ifndef PXV_DYNAMICS_CONTEXT_H
#define PXV_DYNAMICS_CONTEXT_H

#include "CmPhysXCommon.h"
#include "PxSceneDesc.h"
#include "DyThresholdTable.h"
#include "PxcNpThreadContext.h"
#include "PxsSimulationController.h"
#include "DyConstraintWriteBack.h"
#include "PsAllocator.h"

#define DY_MAX_VELOCITY_COUNT 4

namespace physx
{

class PxsIslandManager;
class PxcNpMemBlockPool;

namespace Cm
{
	class EventProfiler;
	class FlushPool;
}

namespace IG
{
	class SimpleIslandManager;
	class IslandSim;
}

template<typename T, typename P> class PxcThreadCoherentCache;
class PxcScratchAllocator;
struct PxvSimStats;
class PxTaskManager;
class PxcNpMemBlockPool;
struct PxgDynamicsMemoryConfig;
class PxsContactManagerOutputIterator;
struct PxsContactManagerOutput;
class PxsKernelWranglerManager;
class PxsHeapMemoryAllocator;
class PxsMemoryManager;
class PxsContactManager;


namespace Dy
{


class Context
{
	PX_NOCOPY(Context)
public:
	/**
	\brief Returns the bounce threshold
	\return The bounce threshold.
	*/
	PX_FORCE_INLINE PxReal				getBounceThreshold()			const	{ return mBounceThreshold;	}
	/**
	\brief Returns the friction offset threshold
	\return The friction offset threshold.
	*/
	PX_FORCE_INLINE PxReal				getFrictionOffsetThreshold()	const	{ return mFrictionOffsetThreshold;	}
	/**
	\brief Returns the friction offset threshold
	\return The friction offset threshold.
	*/
	PX_FORCE_INLINE PxReal				getSolverOffsetSlop()	const	{ return mSolverOffsetSlop; }
	/**
	\brief Returns the correlation distance
	\return The correlation distance.
	*/
	PX_FORCE_INLINE PxReal				getCorrelationDistance()		const	{ return mCorrelationDistance;	}

	/**
	\brief Returns the CCD separation threshold
	\return The CCD separation threshold.
	*/
	PX_FORCE_INLINE PxReal				getCCDSeparationThreshold()		const	{ return mCCDSeparationThreshold; }

	/**
	\brief Sets the bounce threshold
	\param[in] f The bounce threshold
	*/
	PX_FORCE_INLINE void				setBounceThreshold(PxReal f)			{ mBounceThreshold = f;		}
	/**
	\brief Sets the correlation distance
	\param[in] f The correlation distance
	*/
	PX_FORCE_INLINE void				setCorrelationDistance(PxReal f)			{ mCorrelationDistance = f;		}
	/**
	\brief Sets the friction offset threshold
	\param[in] offset The friction offset threshold
	*/
	PX_FORCE_INLINE void				setFrictionOffsetThreshold(PxReal offset)		{ mFrictionOffsetThreshold = offset;				}
	/**
	\brief Sets the solver offset slop
	\param[in] offset The solver offset slop
	*/
	PX_FORCE_INLINE void				setSolverOffsetSlop(PxReal offset)		{ mSolverOffsetSlop = offset; }
	/**
	\brief Sets the friction offset threshold
	\param[in] offset The friction offset threshold
	*/
	PX_FORCE_INLINE void				setCCDSeparationThreshold(PxReal offset)		{ mCCDSeparationThreshold = offset; }


	/**
	\brief Returns the solver batch size
	\return The solver batch size.
	*/
	PX_FORCE_INLINE PxU32				getSolverBatchSize()				const	{ return mSolverBatchSize;	}
	/**
	\brief Sets the solver batch size
	\param[in] f The solver batch size
	*/
 	PX_FORCE_INLINE void				setSolverBatchSize(PxU32 f)				{ mSolverBatchSize = f;		}
	/**
	\brief Returns the maximum solver constraint size
	\return The maximum solver constraint size in this island in bytes.
	*/
	PX_FORCE_INLINE PxU32				getMaxSolverConstraintSize()	const	{ return mMaxSolverConstraintSize; }

	/**
	\brief Returns the friction model being used.
	\return The friction model being used.
	*/
	PX_FORCE_INLINE PxFrictionType::Enum getFrictionType() const				{ return mFrictionType; }

	/**
	\brief Returns the threshold stream
	\return The threshold stream
	*/
	PX_FORCE_INLINE ThresholdStream&	getThresholdStream()					{ return *mThresholdStream; }

	PX_FORCE_INLINE ThresholdStream&	getForceChangedThresholdStream()		{ return *mForceChangedThresholdStream; }

	/**
	\brief Returns the threshold table
	\return The threshold table
	*/
	PX_FORCE_INLINE ThresholdTable&		getThresholdTable()						{ return mThresholdTable; }

	/**
	\brief Sets the friction model to be used.
	\param[in] f The friction model to be used.
	*/
	PX_FORCE_INLINE void				setFrictionType(PxFrictionType::Enum f) 	{ mFrictionType = f; }

	/**
	\brief Destroys this dynamics context
	*/
	virtual void						destroy() = 0;



	PX_FORCE_INLINE PxcDataStreamPool&				getContactStreamPool()						{ return mContactStreamPool;	}

	PX_FORCE_INLINE PxcDataStreamPool&				getPatchStreamPool()						{ return mPatchStreamPool;	}

	PX_FORCE_INLINE PxcDataStreamPool&				getForceStreamPool()						{ return mForceStreamPool;	}

	PX_FORCE_INLINE Ps::Array<Dy::ConstraintWriteback, Ps::VirtualAllocator>&		getConstraintWriteBackPool()			{ return mConstraintWriteBackPool;  }

	
	/**
	\brief Returns the current frame's timestep
	\return The current frame's timestep.
	*/
	PX_FORCE_INLINE PxReal					getDt()							const	{ return mDt;				}
	/**
	\brief Returns 1/(current frame's timestep)
	\return 1/(current frame's timestep).
	*/
	PX_FORCE_INLINE PxReal					getInvDt()						const	{ return mInvDt;			}

	PX_FORCE_INLINE PxReal					getMaxBiasCoefficient()			const { return mMaxBiasCoefficient; }

	PX_FORCE_INLINE PxVec3					getGravity()					const	{ return mGravity;			}



	/**
	\brief The entry point for the constraint solver. 
	\param[in]	dt	The simulation time-step	
	\param[in]	continuation The continuation task for the solver
	\param[in] processLostTouchTask The task that processes lost touches	

	This method is called after the island generation has completed. Its main responsibilities are:
	(1) Reserving the solver body pools
	(2) Initializing the static and kinematic solver bodies, which are shared resources between islands.
	(3) Construct the solver task chains for each island

	Each island is solved as an independent solver task chain. In addition, large islands may be solved using multiple parallel tasks.
	Island solving is asynchronous. Once all islands have been solved, the continuation task will be called.

	*/
	virtual void						update(IG::SimpleIslandManager& simpleIslandManager, PxBaseTask* continuation, PxBaseTask* processLostTouchTask,
		PxsContactManager** foundPatchManagers, PxU32 nbFoundPatchManagers, PxsContactManager** lostPatchManagers, PxU32 nbLostPatchManagers, PxU32 maxPatchesPerCM, 
		PxsContactManagerOutputIterator& iterator, PxsContactManagerOutput* gpuOutputs, const PxReal dt, const PxVec3& gravity, const PxU32 bitMapWordCounts) = 0;

	virtual void						processLostPatches(IG::SimpleIslandManager& simpleIslandManager, PxsContactManager** lostPatchManagers, PxU32 nbLostPatchManagers, PxsContactManagerOutputIterator& iterator) = 0;


	/**
	\brief This method copy gpu solver body data to cpu body core
	*/
	virtual void						updateBodyCore(PxBaseTask* continuation) = 0;

	/**
	\brief Called after update's task chain has completed. This collects the results of the solver together
	*/
	virtual void						mergeResults() = 0;

	virtual void						setSimulationController(PxsSimulationController* simulationController) = 0;

	virtual void						getDataStreamBase(void*& contactStreamBase, void*& patchStreamBase, void*& forceAndIndiceStreamBase) = 0;

	void createThresholdStream(Ps::VirtualAllocatorCallback& callback) { PX_ASSERT(mThresholdStream == NULL); mThresholdStream = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(ThresholdStream), PX_DEBUG_EXP("ThresholdStream")), ThresholdStream(callback));}

	void createForceChangeThresholdStream(Ps::VirtualAllocatorCallback& callback) { PX_ASSERT(mForceChangedThresholdStream == NULL); mForceChangedThresholdStream = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(ThresholdStream), PX_DEBUG_EXP("ThresholdStream")), ThresholdStream(callback));}



protected:

	Context(IG::IslandSim*	accurateIslandSim, Ps::VirtualAllocatorCallback* allocatorCallback,
		PxvSimStats& simStats, bool enableStabilization, bool useEnhancedDeterminism, bool useAdaptiveForce,
		const PxReal maxBiasCoefficient) :
		mThresholdStream(NULL),
		mForceChangedThresholdStream(NULL),		
		mAccurateIslandSim(accurateIslandSim), 		
		mDt							(1.0f), 
		mInvDt						(1.0f),
		mMaxBiasCoefficient			(maxBiasCoefficient),
		mEnableStabilization		(enableStabilization),
		mUseEnhancedDeterminism		(useEnhancedDeterminism),
		mUseAdaptiveForce			(useAdaptiveForce),
		mBounceThreshold(-2.0f),
		mSolverBatchSize(32),
		mConstraintWriteBackPool(Ps::VirtualAllocator(allocatorCallback)),
		mSimStats(simStats)
		 {
		 }

	virtual ~Context() 
	{ 
		if(mThresholdStream)
		{
			mThresholdStream->~ThresholdStream();
			PX_FREE(mThresholdStream);
		} 
		mThresholdStream = NULL;
		if(mForceChangedThresholdStream)
		{
			mForceChangedThresholdStream->~ThresholdStream();
			PX_FREE(mForceChangedThresholdStream);
		}
		mForceChangedThresholdStream = NULL; 
	}

	ThresholdStream*						mThresholdStream;
	ThresholdStream*						mForceChangedThresholdStream;
	ThresholdTable							mThresholdTable;

	IG::IslandSim*							mAccurateIslandSim;
	PxsSimulationController*				mSimulationController;
	/**
	\brief Time-step.
	*/
	PxReal						mDt;
	/**
	\brief 1/time-step.
	*/
	PxReal						mInvDt;

	PxReal						mMaxBiasCoefficient;

	const bool					mEnableStabilization;

	const bool					mUseEnhancedDeterminism;

	const bool					mUseAdaptiveForce;

	PxVec3						mGravity;
	/**
	\brief max solver constraint size
	*/
	PxU32						mMaxSolverConstraintSize;

	/**
	\brief Threshold controlling the relative velocity at which the solver transitions between restitution and bias for solving normal contact constraint.
	*/
	PxReal						mBounceThreshold;
	/**
	\brief Threshold controlling whether friction anchors are constructed or not. If the separation is above mFrictionOffsetThreshold, the contact will not be considered to become a friction anchor
	*/
	PxReal						mFrictionOffsetThreshold;

	/**
	\brief Tolerance used to zero offsets along an axis if it is below this threshold. Used to compensate for small numerical divergence inside contact gen.
	*/
	PxReal						mSolverOffsetSlop;

	/**
	\brief Threshold controlling whether distant contacts are processed using bias, restitution or a combination of the two. This only has effect on pairs involving bodies that have enabled speculative CCD simulation mode.
	*/
	PxReal						mCCDSeparationThreshold;

	/**
	\brief Threshold for controlling friction correlation
	*/
	PxReal						mCorrelationDistance;
	/**
	\brief The minimum size of an island to generate a solver task chain.
	*/
	PxU32						mSolverBatchSize;

	/**
	\brief The current friction model being used
	*/
	PxFrictionType::Enum		mFrictionType;

	/**
	\brief Structure to encapsulate contact stream allocations. Used by GPU solver to reference pre-allocated pinned host memory
	*/
	PxcDataStreamPool		mContactStreamPool;

	/**
	\brief	Struct to encapsulate the contact patch stream allocations. Used by GPU solver to reference pre-allocated pinned host memory
	*/

	PxcDataStreamPool		mPatchStreamPool;

	/**
	\brief Structure to encapsulate force stream allocations. Used by GPU solver to reference pre-allocated pinned host memory for force reports.
	*/
	PxcDataStreamPool		mForceStreamPool;

	/**
	\brief Structure to encapsulate constraint write back allocations. Used by GPU/CPU solver to reference pre-allocated pinned host memory for breakable joint reports.
	*/
	Ps::Array<Dy::ConstraintWriteback, Ps::VirtualAllocator>	mConstraintWriteBackPool;

	PxvSimStats& mSimStats;


};

Context* createDynamicsContext(	PxcNpMemBlockPool* memBlockPool,
								PxcScratchAllocator& scratchAllocator, Cm::FlushPool& taskPool,
								PxvSimStats& simStats, PxTaskManager* taskManager, Ps::VirtualAllocatorCallback* allocatorCallback, PxsMaterialManager* materialManager,
								IG::IslandSim* accurateIslandSim, PxU64 contextID,
								const bool enableStabilization, const bool useEnhancedDeterminism, const bool useAdaptiveForce, const PxReal maxBiasCoefficient,
								const bool frictionEveryIteration
								);

Context* createTGSDynamicsContext(PxcNpMemBlockPool* memBlockPool,
	PxcScratchAllocator& scratchAllocator, Cm::FlushPool& taskPool,
	PxvSimStats& simStats, PxTaskManager* taskManager, Ps::VirtualAllocatorCallback* allocatorCallback, PxsMaterialManager* materialManager,
	IG::IslandSim* accurateIslandSim, PxU64 contextID,
	const bool enableStabilization, const bool useEnhancedDeterminism, const bool useAdaptiveForce, const PxReal lengthScale
);


}

}

#endif

