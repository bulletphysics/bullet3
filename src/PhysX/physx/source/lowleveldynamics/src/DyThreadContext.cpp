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


#include "DyThreadContext.h"
#include "PsBitUtils.h"

namespace physx
{
namespace Dy
{

ThreadContext::ThreadContext(PxcNpMemBlockPool* memBlockPool):
	mFrictionPatchStreamPair(*memBlockPool),
	mConstraintBlockManager	(*memBlockPool),
	mConstraintBlockStream	(*memBlockPool),
	mNumDifferentBodyConstraints(0),
	mNumSelfConstraints(0),
	mNumSelfConstraintBlocks(0),
	mConstraintsPerPartition(PX_DEBUG_EXP("ThreadContext::mConstraintsPerPartition")),
	mFrictionConstraintsPerPartition(PX_DEBUG_EXP("ThreadContext::frictionsConstraintsPerPartition")),
	mPartitionNormalizationBitmap(PX_DEBUG_EXP("ThreadContext::mPartitionNormalizationBitmap")),
	frictionConstraintDescArray(PX_DEBUG_EXP("ThreadContext::solverFrictionConstraintArray")),
	frictionConstraintBatchHeaders(PX_DEBUG_EXP("ThreadContext::frictionConstraintBatchHeaders")),
	compoundConstraints(PX_DEBUG_EXP("ThreadContext::compoundConstraints")),
	orderedContactList(PX_DEBUG_EXP("ThreadContext::orderedContactList")),
	tempContactList(PX_DEBUG_EXP("ThreadContext::tempContactList")),
	sortIndexArray(PX_DEBUG_EXP("ThreadContext::sortIndexArray")),
	mConstraintSize			(0),
	mAxisConstraintCount(0),
	mSelfConstraintBlocks(NULL),
	mMaxPartitions(0),
	mMaxSolverPositionIterations(0),
	mMaxSolverVelocityIterations(0),
	mMaxArticulationLength(0),
	mContactDescPtr(NULL),
	mFrictionDescPtr(NULL),
	mArticulations(PX_DEBUG_EXP("ThreadContext::articulations"))
	
{
#if PX_ENABLE_SIM_STATS
	mThreadSimStats.clear();
#endif
	//Defaulted to have space for 16384 bodies
	mPartitionNormalizationBitmap.reserve(512); 
	//Defaulted to have space for 128 partitions (should be more-than-enough)
	mConstraintsPerPartition.reserve(128);
}

void ThreadContext::resizeArrays(PxU32 frictionConstraintDescCount, PxU32 articulationCount)
{
	// resize resizes smaller arrays to the exact target size, which can generate a lot of churn
	frictionConstraintDescArray.forceSize_Unsafe(0);
	frictionConstraintDescArray.reserve((frictionConstraintDescCount+63)&~63);

	mArticulations.forceSize_Unsafe(0);
	mArticulations.reserve(PxMax<PxU32>(Ps::nextPowerOfTwo(articulationCount), 16));
	mArticulations.forceSize_Unsafe(articulationCount);

	mContactDescPtr = contactConstraintDescArray;
	mFrictionDescPtr = frictionConstraintDescArray.begin();
}

void ThreadContext::reset()
{
	// TODO: move these to the PxcNpThreadContext
	mFrictionPatchStreamPair.reset();
	mConstraintBlockStream.reset();

	mContactDescPtr = contactConstraintDescArray;
	mFrictionDescPtr = frictionConstraintDescArray.begin();

	mAxisConstraintCount = 0;
	mMaxSolverPositionIterations = 0;
	mMaxSolverVelocityIterations = 0;
	mNumDifferentBodyConstraints = 0;
	mNumSelfConstraints = 0;
	mSelfConstraintBlocks = NULL;
	mNumSelfConstraintBlocks = 0;
	mConstraintSize = 0;
}

}
} 
