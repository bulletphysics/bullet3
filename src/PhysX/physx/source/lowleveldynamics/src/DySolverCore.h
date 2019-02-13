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


#ifndef DY_SOLVERCORE_H
#define DY_SOLVERCORE_H

#include "PxvConfig.h"
#include "PsArray.h"
#include "PsThread.h"


namespace physx
{

struct PxSolverBody;
struct PxSolverBodyData;
struct PxSolverConstraintDesc;
struct PxConstraintBatchHeader;

namespace Dy
{
struct ThresholdStreamElement;
	

struct ArticulationSolverDesc;
class Articulation;
struct SolverContext;

typedef void (*WriteBackMethod)(const PxSolverConstraintDesc& desc, SolverContext& cache, PxSolverBodyData& sbd0, PxSolverBodyData& sbd1);
typedef void (*SolveMethod)(const PxSolverConstraintDesc& desc, SolverContext& cache);
typedef void (*SolveBlockMethod)(const PxSolverConstraintDesc* desc, const PxU32 constraintCount, SolverContext& cache);
typedef void (*SolveWriteBackBlockMethod)(const PxSolverConstraintDesc* desc, const PxU32 constraintCount, SolverContext& cache);
typedef void (*WriteBackBlockMethod)(const PxSolverConstraintDesc* desc, const PxU32 constraintCount, SolverContext& cache);

#define PX_PROFILE_SOLVE_STALLS 0
#if PX_PROFILE_SOLVE_STALLS
#if PX_WINDOWS
#include <windows.h>


PX_FORCE_INLINE PxU64 readTimer()
{
	//return __rdtsc();

	LARGE_INTEGER i;
	QueryPerformanceCounter(&i);
	return i.QuadPart;
}

#endif
#endif


#define YIELD_THREADS 1

#if YIELD_THREADS

#define ATTEMPTS_BEFORE_BACKOFF 30000
#define ATTEMPTS_BEFORE_RETEST 10000

#endif

PX_INLINE void WaitForProgressCount(volatile PxI32* pGlobalIndex, const PxI32 targetIndex)
{
#if YIELD_THREADS
	if(*pGlobalIndex < targetIndex)
	{
		bool satisfied = false;
		PxU32 count = ATTEMPTS_BEFORE_BACKOFF;
		do
		{
			satisfied = true;
			while(*pGlobalIndex < targetIndex)
			{
				if(--count == 0)
				{
					satisfied = false;
					break;
				}
			}
			if(!satisfied)
				Ps::Thread::yield();
			count = ATTEMPTS_BEFORE_RETEST;
		}
		while(!satisfied);
	}
#else
	while(*pGlobalIndex < targetIndex);
#endif
}


#if PX_PROFILE_SOLVE_STALLS
PX_INLINE void WaitForProgressCount(volatile PxI32* pGlobalIndex, const PxI32 targetIndex, PxU64& stallTime)
{
	if(*pGlobalIndex < targetIndex)
	{
		bool satisfied = false;
		PxU32 count = ATTEMPTS_BEFORE_BACKOFF;
		do
		{
			satisfied = true;
			PxU64 startTime = readTimer();
			while(*pGlobalIndex < targetIndex)
			{
				if(--count == 0)
				{
					satisfied = false;
					break;
				}
			}
			PxU64 endTime = readTimer();
			stallTime += (endTime - startTime);
			if(!satisfied)
				Ps::Thread::yield();
			count = ATTEMPTS_BEFORE_BACKOFF;
		}
		while(!satisfied);
	}
}

#define WAIT_FOR_PROGRESS(pGlobalIndex, targetIndex) if(*pGlobalIndex < targetIndex) WaitForProgressCount(pGlobalIndex, targetIndex, stallCount)
#else
#define WAIT_FOR_PROGRESS(pGlobalIndex, targetIndex) if(*pGlobalIndex < targetIndex) WaitForProgressCount(pGlobalIndex, targetIndex)
#endif
#define WAIT_FOR_PROGRESS_NO_TIMER(pGlobalIndex, targetIndex) if(*pGlobalIndex < targetIndex) WaitForProgressCount(pGlobalIndex, targetIndex)


struct SolverIslandParams
{
	//Default friction model params
	PxU32 positionIterations;
	PxU32 velocityIterations;
	PxSolverBody* PX_RESTRICT bodyListStart;
	PxSolverBodyData* PX_RESTRICT bodyDataList;
	PxU32 bodyListSize;
	PxU32 solverBodyOffset;
	ArticulationSolverDesc* PX_RESTRICT articulationListStart; 
	PxU32 articulationListSize;
	PxSolverConstraintDesc* PX_RESTRICT constraintList;
	PxConstraintBatchHeader* constraintBatchHeaders;
	PxU32 numConstraintHeaders;
	PxU32* headersPerPartition;
	PxU32 nbPartitions;
	Cm::SpatialVector* PX_RESTRICT motionVelocityArray;
	PxU32 batchSize;
	PxsBodyCore*const* bodyArray;
	PxsRigidBody** PX_RESTRICT rigidBodies;

	//Shared state progress counters
	PxI32 constraintIndex;
	PxI32 constraintIndex2;
	PxI32 bodyListIndex;
	PxI32 bodyListIndex2;
	PxI32 articSolveIndex;
	PxI32 articSolveIndex2;
	PxI32 bodyIntegrationListIndex;
	PxI32 numObjectsIntegrated;

	PxReal dt;
	PxReal invDt;


	//Additional 1d/2d friction model params
	PxSolverConstraintDesc* PX_RESTRICT frictionConstraintList;
	
	PxConstraintBatchHeader* frictionConstraintBatches;
	PxU32 numFrictionConstraintHeaders;
	PxU32* frictionHeadersPerPartition;
	PxU32 nbFrictionPartitions;

	//Additional Shared state progress counters
	PxI32 frictionConstraintIndex;

	//Write-back threshold information
	ThresholdStreamElement* PX_RESTRICT thresholdStream;
	PxU32 thresholdStreamLength;

	PxI32* outThresholdPairs;

	PxU32 mMaxArticulationLinks;
	Cm::SpatialVectorF* Z;
	Cm::SpatialVectorF* deltaV;
};


/*!
Interface to constraint solver cores

*/    
class SolverCore
{
public:
	virtual void destroyV() = 0;
    virtual ~SolverCore() {}
	/*
	solves dual problem exactly by GS-iterating until convergence stops
	only uses regular velocity vector for storing results, and backs up initial state, which is restored.
	the solution forces are saved in a vector.

	state should not be stored, this function is safe to call from multiple threads.

	Returns the total number of constraints that should be solved across all threads. Used for synchronization outside of this method
	*/

	virtual PxI32 solveVParallelAndWriteBack
		(SolverIslandParams& params, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV) const = 0;


	virtual void solveV_Blocks
		(SolverIslandParams& params) const = 0;


	virtual void writeBackV
		(const PxSolverConstraintDesc* PX_RESTRICT constraintList, const PxU32 constraintListSize, PxConstraintBatchHeader* contactConstraintBatches, const PxU32 numConstraintBatches,
	 	 ThresholdStreamElement* PX_RESTRICT thresholdStream, const PxU32 thresholdStreamLength, PxU32& outThresholdPairs,
		 PxSolverBodyData* atomListData, WriteBackBlockMethod writeBackTable[]) const = 0;
};

}

}

#endif //DY_SOLVERCORE_H
