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


#ifndef DY_SOLVERCOREGENERAL_H
#define DY_SOLVERCOREGENERAL_H

#include "DySolverCore.h"
#include "DySolverConstraintDesc.h"

namespace physx
{

namespace Dy
{

struct FsData;

inline void BusyWaitState(volatile PxU32* state, const PxU32 requiredState)
{
	while(requiredState != *state );
}

inline void WaitBodyRequiredState(PxU32* state, const PxU32 requiredState)
{
	if(*state != requiredState)
	{
		BusyWaitState(state, requiredState);
	}
}

inline void BusyWaitStates(volatile PxU32* stateA, volatile PxU32* stateB, const PxU32 requiredStateA, const PxU32 requiredStateB)
{
	while(*stateA != requiredStateA);
	while(*stateB != requiredStateB);
}


class BatchIterator
{
public:
	PxConstraintBatchHeader* constraintBatchHeaders;
	PxU32 mSize;
	PxU32 mCurrentIndex;

	BatchIterator(PxConstraintBatchHeader* _constraintBatchHeaders, PxU32 size) : constraintBatchHeaders(_constraintBatchHeaders),
		mSize(size), mCurrentIndex(0)
	{
	}

	PX_FORCE_INLINE const PxConstraintBatchHeader& GetCurrentHeader(const PxU32 constraintIndex)
	{
		PxU32 currentIndex = mCurrentIndex;
		while((constraintIndex - constraintBatchHeaders[currentIndex].mStartIndex) >= constraintBatchHeaders[currentIndex].mStride)
			currentIndex = (currentIndex + 1)%mSize;
		Ps::prefetchLine(&constraintBatchHeaders[currentIndex], 128);
		mCurrentIndex = currentIndex;
		return constraintBatchHeaders[currentIndex];
	}
private:
	BatchIterator& operator=(const BatchIterator&);
};


inline void SolveBlockParallel	(PxSolverConstraintDesc* PX_RESTRICT constraintList, const PxI32 batchCount, const PxI32 index,  
						 const PxI32 headerCount, SolverContext& cache, BatchIterator& iterator,
						 SolveBlockMethod solveTable[],
						 const PxI32 iteration
						)
{
	const PxI32 indA = index - (iteration * headerCount);

	const PxConstraintBatchHeader* PX_RESTRICT headers = iterator.constraintBatchHeaders;

	const PxI32 endIndex = indA + batchCount;
	for(PxI32 i = indA; i < endIndex; ++i)
	{
		const PxConstraintBatchHeader& header = headers[i];

		const PxI32 numToGrab = header.mStride;
		PxSolverConstraintDesc* PX_RESTRICT block = &constraintList[header.mStartIndex];

		Ps::prefetch(block[0].constraint, 384);

		for(PxI32 b = 0; b < numToGrab; ++b)
		{
			Ps::prefetchLine(block[b].bodyA);
			Ps::prefetchLine(block[b].bodyB);
		}

		//OK. We have a number of constraints to run...
		solveTable[header.mConstraintType](block, PxU32(numToGrab), cache);
	}
}




class SolverCoreGeneral : public SolverCore
{
public:
	bool frictionEveryIteration;
	static SolverCoreGeneral* create(bool fricEveryIteration);

	// Implements SolverCore
	virtual void destroyV();

	virtual PxI32 solveVParallelAndWriteBack
		(SolverIslandParams& params, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV) const;

	virtual void solveV_Blocks
		(SolverIslandParams& params) const;

	virtual void writeBackV
		(const PxSolverConstraintDesc* PX_RESTRICT constraintList, const PxU32 constraintListSize, PxConstraintBatchHeader* contactConstraintBatches, const PxU32 numBatches,
		 ThresholdStreamElement* PX_RESTRICT thresholdStream, const PxU32 thresholdStreamLength, PxU32& outThresholdPairs,
		 PxSolverBodyData* atomListData, WriteBackBlockMethod writeBackTable[]) const;

private:

	//~Implements SolverCore
};

#define SOLVEV_BLOCK_METHOD_ARGS											\
	SolverCore*	solverCore,												\
	SolverIslandParams& params

void solveVBlock(SOLVEV_BLOCK_METHOD_ARGS);

SolveBlockMethod* getSolveBlockTable();

SolveBlockMethod* getSolverConcludeBlockTable();

SolveWriteBackBlockMethod* getSolveWritebackBlockTable();


}

}

#endif //DY_SOLVERCOREGENERAL_H
