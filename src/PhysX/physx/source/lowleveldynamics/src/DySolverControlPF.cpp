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

#include "foundation/PxPreprocessor.h"
#include "PsAllocator.h"
#include <new>
#include <stdio.h>
#include "CmPhysXCommon.h"
#include "DySolverBody.h"
#include "DySolverConstraint1D.h"
#include "DySolverContact.h"
#include "DyThresholdTable.h"
#include "DySolverControl.h"
#include "DyArticulationHelper.h"
#include "PsAtomic.h"
#include "PsIntrinsics.h"
#include "DyArticulationPImpl.h"
#include "PsThread.h"
#include "DySolverConstraintDesc.h"
#include "DySolverContext.h"
#include "DySolverControlPF.h"

namespace physx
{

namespace Dy
{
//-----------------------------------

void solve1DBlock					(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveExt1DBlock				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solve1D4_Block					(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);


void solve1DConcludeBlock				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveExt1DConcludeBlock			(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solve1D4Block_Conclude				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);

void solve1DBlockWriteBack				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveExt1DBlockWriteBack			(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solve1D4Block_WriteBack			(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);

void writeBack1DBlock				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void ext1DBlockWriteBack			(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void writeBack1D4Block				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);


void solveFrictionBlock					(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveFriction_BStaticBlock			(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveExtFrictionBlock				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveContactCoulombBlock			(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveExtContactCoulombBlock		(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveContactCoulomb_BStaticBlock	(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);


void solveContactCoulombConcludeBlock			(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveExtContactCoulombConcludeBlock		(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveContactCoulomb_BStaticConcludeBlock	(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);

void solveContactCoulombBlockWriteBack			(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveExtContactCoulombBlockWriteBack		(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveContactCoulomb_BStaticBlockWriteBack	(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveFrictionBlockWriteBack				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveFriction_BStaticBlockWriteBack		(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveExtFrictionBlockWriteBack				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);

//Pre-block 1d/2d friction stuff...

void solveContactCoulombPreBlock				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveContactCoulombPreBlock_Static			(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveContactCoulombPreBlock_Conclude		(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveContactCoulombPreBlock_ConcludeStatic	(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveContactCoulombPreBlock_WriteBack		(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveContactCoulombPreBlock_WriteBackStatic(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveFrictionCoulombPreBlock				(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);

void solveFrictionCoulombPreBlock_Static		(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveFrictionCoulombPreBlock_Conclude		(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);
void solveFrictionCoulombPreBlock_ConcludeStatic(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);

void solveFrictionCoulombPreBlock_WriteBack		(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);

void solveFrictionCoulombPreBlock_WriteBackStatic(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache);


// could move this to PxPreprocessor.h but 
// no implementation available for MSVC
#if PX_GCC_FAMILY
#define PX_UNUSED_ATTRIBUTE __attribute__((unused))
#else
#define PX_UNUSED_ATTRIBUTE 
#endif
 
#define DYNAMIC_ARTICULATION_REGISTRATION(x) 0


static SolveBlockMethod gVTableSolveBlockCoulomb[] PX_UNUSED_ATTRIBUTE = 
{
	0,
	solveContactCoulombBlock,												// DY_SC_TYPE_RB_CONTACT
	solve1DBlock,															// DY_SC_TYPE_RB_1D
	DYNAMIC_ARTICULATION_REGISTRATION(solveExtContactCoulombBlock),			// DY_SC_TYPE_EXT_CONTACT
	DYNAMIC_ARTICULATION_REGISTRATION(solveExt1DBlock),						// DY_SC_TYPE_EXT_1D
	solveContactCoulomb_BStaticBlock,										// DY_SC_TYPE_STATIC_CONTACT
	solveContactCoulombBlock,												// DY_SC_TYPE_NOFRICTION_RB_CONTACT
	solveContactCoulombPreBlock,											// DY_SC_TYPE_BLOCK_RB_CONTACT
	solveContactCoulombPreBlock_Static,										// DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT
	solve1D4_Block,															// DY_SC_TYPE_BLOCK_1D,
	solveFrictionBlock,														// DY_SC_TYPE_FRICTION_CONSTRAINT
	solveFriction_BStaticBlock,												// DY_SC_TYPE_STATIC_FRICTION_CONSTRAINT
	DYNAMIC_ARTICULATION_REGISTRATION(solveExtFrictionBlock),				// DY_SC_TYPE_EXT_FRICTION_CONSTRAINT
	solveFrictionCoulombPreBlock,											// DY_SC_TYPE_BLOCK_FRICTION					
	solveFrictionCoulombPreBlock_Static										// DY_SC_TYPE_BLOCK_STATIC_FRICTION
};

static SolveWriteBackBlockMethod gVTableSolveWriteBackBlockCoulomb[] PX_UNUSED_ATTRIBUTE = 
{
	0,
	solveContactCoulombBlockWriteBack,												// DY_SC_TYPE_RB_CONTACT
	solve1DBlockWriteBack,															// DY_SC_TYPE_RB_1D
	DYNAMIC_ARTICULATION_REGISTRATION(solveExtContactCoulombBlockWriteBack),		// DY_SC_TYPE_EXT_CONTACT
	DYNAMIC_ARTICULATION_REGISTRATION(solveExt1DBlockWriteBack),					// DY_SC_TYPE_EXT_1D
	solveContactCoulomb_BStaticBlockWriteBack,										// DY_SC_TYPE_STATIC_CONTACT
	solveContactCoulombBlockWriteBack,												// DY_SC_TYPE_NOFRICTION_RB_CONTACT
	solveContactCoulombPreBlock_WriteBack,											// DY_SC_TYPE_BLOCK_RB_CONTACT
	solveContactCoulombPreBlock_WriteBackStatic,									// DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT
	solve1D4Block_WriteBack,														// DY_SC_TYPE_BLOCK_1D,
	solveFrictionBlockWriteBack,													// DY_SC_TYPE_FRICTION_CONSTRAINT
	solveFriction_BStaticBlockWriteBack,											// DY_SC_TYPE_STATIC_FRICTION_CONSTRAINT
	DYNAMIC_ARTICULATION_REGISTRATION(solveExtFrictionBlockWriteBack),				// DY_SC_TYPE_EXT_FRICTION_CONSTRAINT
	solveFrictionCoulombPreBlock_WriteBack,											// DY_SC_TYPE_BLOCK_FRICTION
	solveFrictionCoulombPreBlock_WriteBackStatic									// DY_SC_TYPE_BLOCK_STATIC_FRICTION
};


static SolveBlockMethod gVTableSolveConcludeBlockCoulomb[] PX_UNUSED_ATTRIBUTE = 
{
	0,
	solveContactCoulombConcludeBlock,												// DY_SC_TYPE_RB_CONTACT
	solve1DConcludeBlock,															// DY_SC_TYPE_RB_1D
	DYNAMIC_ARTICULATION_REGISTRATION(solveExtContactCoulombConcludeBlock),			// DY_SC_TYPE_EXT_CONTACT
	DYNAMIC_ARTICULATION_REGISTRATION(solveExt1DConcludeBlock),						// DY_SC_TYPE_EXT_1D
	solveContactCoulomb_BStaticConcludeBlock,										// DY_SC_TYPE_STATIC_CONTACT
	solveContactCoulombConcludeBlock,												// DY_SC_TYPE_NOFRICTION_RB_CONTACT
	solveContactCoulombPreBlock_Conclude,											// DY_SC_TYPE_BLOCK_RB_CONTACT
	solveContactCoulombPreBlock_ConcludeStatic,										// DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT
	solve1D4Block_Conclude,															// DY_SC_TYPE_BLOCK_1D,
	solveFrictionBlock,																// DY_SC_TYPE_FRICTION_CONSTRAINT
	solveFriction_BStaticBlock,														// DY_SC_TYPE_STATIC_FRICTION_CONSTRAINT
	DYNAMIC_ARTICULATION_REGISTRATION(solveExtFrictionBlock),						// DY_SC_TYPE_EXT_FRICTION_CONSTRAINT
	solveFrictionCoulombPreBlock_Conclude,											// DY_SC_TYPE_BLOCK_FRICTION
	solveFrictionCoulombPreBlock_ConcludeStatic										// DY_SC_TYPE_BLOCK_STATIC_FRICTION
};


void SolverCoreRegisterArticulationFnsCoulomb()
{
	gVTableSolveBlockCoulomb[DY_SC_TYPE_EXT_CONTACT] = solveExtContactCoulombBlock;
	gVTableSolveBlockCoulomb[DY_SC_TYPE_EXT_1D] = solveExt1DBlock;

	gVTableSolveWriteBackBlockCoulomb[DY_SC_TYPE_EXT_CONTACT] = solveExtContactCoulombBlockWriteBack;
	gVTableSolveWriteBackBlockCoulomb[DY_SC_TYPE_EXT_1D] = solveExt1DBlockWriteBack;
	gVTableSolveConcludeBlockCoulomb[DY_SC_TYPE_EXT_CONTACT] = solveExtContactCoulombConcludeBlock;
	gVTableSolveConcludeBlockCoulomb[DY_SC_TYPE_EXT_1D] = solveExt1DConcludeBlock;

	gVTableSolveBlockCoulomb[DY_SC_TYPE_EXT_FRICTION] = solveExtFrictionBlock;
	gVTableSolveWriteBackBlockCoulomb[DY_SC_TYPE_EXT_FRICTION] = solveExtFrictionBlockWriteBack;
	gVTableSolveConcludeBlockCoulomb[DY_SC_TYPE_EXT_FRICTION] = solveExtFrictionBlock;
}

SolverCoreGeneralPF* SolverCoreGeneralPF::create()
{
	SolverCoreGeneralPF* scg = reinterpret_cast<SolverCoreGeneralPF*>(
		PX_ALLOC(sizeof(SolverCoreGeneralPF), "SolverCoreGeneral"));

	if(scg)
		new (scg) SolverCoreGeneralPF;

	return scg;
}

void SolverCoreGeneralPF::destroyV()
{
	this->~SolverCoreGeneralPF();
	PX_FREE(this);
}

void SolverCoreGeneralPF::solveV_Blocks(SolverIslandParams& params) const
{
	const PxI32 TempThresholdStreamSize = 32;
	ThresholdStreamElement tempThresholdStream[TempThresholdStreamSize];

	SolverContext cache;
	cache.solverBodyArray			= params.bodyDataList;
	cache.mThresholdStream			= tempThresholdStream;
	cache.mThresholdStreamLength	= TempThresholdStreamSize;
	cache.mThresholdStreamIndex		= 0;
	cache.writeBackIteration		= false;
	cache.deltaV					= params.deltaV;
	cache.Z							= params.Z;

	PxI32 batchCount = PxI32(params.numConstraintHeaders);

	PxSolverBody* PX_RESTRICT bodyListStart = params.bodyListStart;
	const PxU32 bodyListSize = params.bodyListSize;

	Cm::SpatialVector* PX_RESTRICT motionVelocityArray = params.motionVelocityArray;

	const PxU32 velocityIterations = params.velocityIterations;
	const PxU32 positionIterations = params.positionIterations;

	const PxU32 numConstraintHeaders = params.numConstraintHeaders;
	const PxU32 articulationListSize = params.articulationListSize;

	ArticulationSolverDesc* PX_RESTRICT articulationListStart = params.articulationListStart;


	PX_ASSERT(velocityIterations >= 1);
	PX_ASSERT(positionIterations >= 1);

	if(numConstraintHeaders == 0)
	{
		for (PxU32 baIdx = 0; baIdx < bodyListSize; baIdx++)
		{
			Cm::SpatialVector& motionVel = motionVelocityArray[baIdx];
			PxSolverBody& atom = bodyListStart[baIdx];
			motionVel.linear = atom.linearVelocity;
			motionVel.angular = atom.angularState;
		}

		for (PxU32 i = 0; i < articulationListSize; i++)
			ArticulationPImpl::saveVelocity(articulationListStart[i], cache.deltaV);

		return;
	}

	BatchIterator contactIterator(params.constraintBatchHeaders, params.numConstraintHeaders);
	BatchIterator frictionIterator(params.frictionConstraintBatches, params.numFrictionConstraintHeaders);


	PxI32 frictionBatchCount = PxI32(params.numFrictionConstraintHeaders);

	PxSolverConstraintDesc* PX_RESTRICT constraintList = params.constraintList;

	PxSolverConstraintDesc* PX_RESTRICT frictionConstraintList = params.frictionConstraintList;


	//0-(n-1) iterations
	PxI32 normalIter = 0;
	PxI32 frictionIter = 0;
	for (PxU32 iteration = positionIterations; iteration > 0; iteration--)	//decreasing positive numbers == position iters
	{

		SolveBlockParallel(constraintList, batchCount, normalIter * batchCount, batchCount, 
			cache, contactIterator, iteration == 1 ? gVTableSolveConcludeBlockCoulomb : gVTableSolveBlockCoulomb, normalIter);
		++normalIter;
	
	}

	if(frictionBatchCount>0)
	{
		const PxU32 numIterations = positionIterations * 2;
		for (PxU32 iteration = numIterations; iteration > 0; iteration--)	//decreasing positive numbers == position iters
		{
			SolveBlockParallel(frictionConstraintList, frictionBatchCount, frictionIter * frictionBatchCount, frictionBatchCount, 
				cache, frictionIterator, iteration == 1 ? gVTableSolveConcludeBlockCoulomb : gVTableSolveBlockCoulomb, frictionIter);
			++frictionIter;
		}
	}

	for (PxU32 baIdx = 0; baIdx < bodyListSize; baIdx++)
	{
		const PxSolverBody& atom = bodyListStart[baIdx];
		Cm::SpatialVector& motionVel = motionVelocityArray[baIdx];
		motionVel.linear = atom.linearVelocity;
		motionVel.angular = atom.angularState;
	}
	

	for (PxU32 i = 0; i < articulationListSize; i++)
		ArticulationPImpl::saveVelocity(articulationListStart[i], cache.deltaV);


	const PxU32 velItersMinOne = velocityIterations - 1;

	PxU32 iteration = 0;

	for(; iteration < velItersMinOne; ++iteration)
	{	

		SolveBlockParallel(constraintList, batchCount, normalIter * batchCount, batchCount, 
			cache, contactIterator, gVTableSolveBlockCoulomb, normalIter);
		++normalIter;

		if(frictionBatchCount > 0)
		{
			SolveBlockParallel(frictionConstraintList, frictionBatchCount, frictionIter * frictionBatchCount, frictionBatchCount, 
				cache, frictionIterator, gVTableSolveBlockCoulomb, frictionIter);
			++frictionIter;
		}
	}

	PxI32* outThresholdPairs = params.outThresholdPairs;
	ThresholdStreamElement* PX_RESTRICT thresholdStream = params.thresholdStream;
	PxU32 thresholdStreamLength = params.thresholdStreamLength;

	cache.writeBackIteration = true;

	cache.mSharedOutThresholdPairs = outThresholdPairs;
	cache.mSharedThresholdStreamLength = thresholdStreamLength;
	cache.mSharedThresholdStream = thresholdStream;

	for(; iteration < velocityIterations; ++iteration)
	{
		SolveBlockParallel(constraintList, batchCount, normalIter * batchCount, batchCount, 
			cache, contactIterator, gVTableSolveWriteBackBlockCoulomb, normalIter);
		++normalIter;

		if(frictionBatchCount > 0)
		{
			SolveBlockParallel(frictionConstraintList, frictionBatchCount, frictionIter * frictionBatchCount, frictionBatchCount, 
				cache, frictionIterator, gVTableSolveWriteBackBlockCoulomb, frictionIter);
				++frictionIter;
		}
	}

	//Write back remaining threshold streams
	if(cache.mThresholdStreamIndex > 0)
	{
		//Write back to global buffer
		PxI32 threshIndex = physx::shdfnd::atomicAdd(reinterpret_cast<PxI32*>(&outThresholdPairs), PxI32(cache.mThresholdStreamIndex)) - PxI32(cache.mThresholdStreamIndex);
		for(PxU32 b = 0; b < cache.mThresholdStreamIndex; ++b)
		{
			thresholdStream[b + threshIndex] = cache.mThresholdStream[b];
		}
		cache.mThresholdStreamIndex = 0;
	}

}

PxI32 SolverCoreGeneralPF::solveVParallelAndWriteBack(SolverIslandParams& params,
	Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV) const
{
	SolverContext cache;
	cache.solverBodyArray = params.bodyDataList;

	const PxI32 UnrollCount = PxI32(params.batchSize);
	const PxI32 SaveUnrollCount = 64;

	const PxI32 TempThresholdStreamSize = 32;
	ThresholdStreamElement tempThresholdStream[TempThresholdStreamSize];


	const PxI32 batchCount = PxI32(params.numConstraintHeaders);
	const PxI32 frictionBatchCount = PxI32(params.numFrictionConstraintHeaders);//frictionConstraintBatches.size();
	cache.mThresholdStream = tempThresholdStream;
	cache.mThresholdStreamLength = TempThresholdStreamSize;
	cache.mThresholdStreamIndex = 0;
	cache.Z = Z;
	cache.deltaV = deltaV;

	const PxI32 positionIterations = PxI32(params.positionIterations);
	const PxU32 velocityIterations = params.velocityIterations;

	const PxI32 bodyListSize = PxI32(params.bodyListSize);
	const PxI32 articulationListSize = PxI32(params.articulationListSize);

	PX_ASSERT(velocityIterations >= 1);
	PX_ASSERT(positionIterations >= 1);

	PxI32* constraintIndex = &params.constraintIndex;
	PxI32* constraintIndex2 = &params.constraintIndex2;
	PxI32* frictionConstraintIndex = &params.frictionConstraintIndex;

	PxI32 endIndexCount = UnrollCount;
	PxI32 index = physx::shdfnd::atomicAdd(constraintIndex, UnrollCount) - UnrollCount;
	PxI32 frictionIndex = physx::shdfnd::atomicAdd(frictionConstraintIndex, UnrollCount) - UnrollCount;
	

	BatchIterator contactIter(params.constraintBatchHeaders, params.numConstraintHeaders);
	BatchIterator frictionIter(params.frictionConstraintBatches, params.numFrictionConstraintHeaders);

	PxU32* headersPerPartition = params.headersPerPartition;
	PxU32 nbPartitions = params.nbPartitions;

	PxU32* frictionHeadersPerPartition = params.frictionHeadersPerPartition;
	PxU32 nbFrictionPartitions = params.nbFrictionPartitions;

	PxSolverConstraintDesc* PX_RESTRICT constraintList = params.constraintList;
	PxSolverConstraintDesc* PX_RESTRICT frictionConstraintList = params.frictionConstraintList;


	PxI32 maxNormalIndex = 0;
	PxI32 maxProgress = 0;
	PxI32 frictionEndIndexCount = UnrollCount;
	PxI32 maxFrictionIndex = 0;

	PxI32 normalIteration = 0;
	PxI32 frictionIteration = 0;
	PxU32 a = 0;
	for(PxU32 i = 0; i < 2; ++i)
	{
		SolveBlockMethod* solveTable = i == 0 ? gVTableSolveBlockCoulomb : gVTableSolveConcludeBlockCoulomb;
		for(; a < positionIterations - 1 + i; ++a)
		{
			for(PxU32 b = 0; b < nbPartitions; ++b)
			{
				WAIT_FOR_PROGRESS(constraintIndex2, maxProgress);
				maxNormalIndex += headersPerPartition[b];
				maxProgress += headersPerPartition[b];
				PxI32 nbSolved = 0;
				while(index < maxNormalIndex)
				{
					const PxI32 remainder = PxMin(maxNormalIndex - index, endIndexCount);
					SolveBlockParallel(constraintList, remainder, index, batchCount, cache, contactIter, solveTable, 
						normalIteration);
					index += remainder;
					endIndexCount -= remainder;
					nbSolved += remainder;
					if(endIndexCount == 0)
					{
						endIndexCount = UnrollCount;
						index = physx::shdfnd::atomicAdd(constraintIndex, UnrollCount) - UnrollCount;
					}
				}
				if(nbSolved)
				{
					Ps::memoryBarrier();
					Ps::atomicAdd(constraintIndex2, nbSolved);
				}
			}
			++normalIteration;
		}

	}


	for(PxU32 i = 0; i < 2; ++i)
	{
		SolveBlockMethod* solveTable = i == 0 ? gVTableSolveBlockCoulomb : gVTableSolveConcludeBlockCoulomb;
		const PxI32 numIterations = positionIterations *2;
		for(; a <  numIterations - 1 + i; ++a)
		{
			for(PxU32 b = 0; b < nbFrictionPartitions; ++b)
			{
				WAIT_FOR_PROGRESS(constraintIndex2, maxProgress);
				maxProgress += frictionHeadersPerPartition[b];
				maxFrictionIndex += frictionHeadersPerPartition[b];
				PxI32 nbSolved = 0;
				while(frictionIndex < maxFrictionIndex)
				{
					const PxI32 remainder = PxMin(maxFrictionIndex - frictionIndex, frictionEndIndexCount);
					SolveBlockParallel(frictionConstraintList, remainder, frictionIndex, frictionBatchCount, cache, frictionIter, 
						solveTable, frictionIteration);
					frictionIndex += remainder;
					frictionEndIndexCount -= remainder;
					nbSolved += remainder;
					if(frictionEndIndexCount == 0)
					{
						frictionEndIndexCount = UnrollCount;
						frictionIndex  = physx::shdfnd::atomicAdd(frictionConstraintIndex, UnrollCount) - UnrollCount;
					}
				}
				if(nbSolved)
				{
					Ps::memoryBarrier();
					Ps::atomicAdd(constraintIndex2, nbSolved);
				}
			}
			++frictionIteration;
			
		}

	}

	WAIT_FOR_PROGRESS(constraintIndex2, maxProgress);

	
	PxI32* bodyListIndex = &params.bodyListIndex;

	ArticulationSolverDesc* PX_RESTRICT articulationListStart = params.articulationListStart;

	PxSolverBody* PX_RESTRICT bodyListStart = params.bodyListStart;

	Cm::SpatialVector* PX_RESTRICT motionVelocityArray = params.motionVelocityArray;

	PxI32* bodyListIndex2 = &params.bodyListIndex2;

	PxI32 endIndexCount2 = SaveUnrollCount;
	PxI32 index2 = physx::shdfnd::atomicAdd(bodyListIndex, SaveUnrollCount) - SaveUnrollCount;
	{
		PxI32 nbConcluded = 0;
		while(index2 < articulationListSize)
		{
			const PxI32 remainder = PxMin(SaveUnrollCount, (articulationListSize - index2));
			endIndexCount2 -= remainder;
			for(PxI32 b = 0; b < remainder; ++b, ++index2)
			{
				ArticulationPImpl::saveVelocity(articulationListStart[index2], cache.deltaV);
			}
			nbConcluded += remainder;
			if(endIndexCount2 == 0)
			{
				index2 = physx::shdfnd::atomicAdd(bodyListIndex, SaveUnrollCount) - SaveUnrollCount;
				endIndexCount2 = SaveUnrollCount;
			}
			nbConcluded += remainder;
		}

		index2 -= articulationListSize;

		//save velocity
		

		while(index2 < bodyListSize)
		{
			const PxI32 remainder = PxMin(endIndexCount2, (bodyListSize - index2));
			endIndexCount2 -= remainder;
			for(PxI32 b = 0; b < remainder; ++b, ++index2)
			{
				Ps::prefetchLine(&bodyListStart[index2 + 8]);
				Ps::prefetchLine(&motionVelocityArray[index2 + 8]);
				PxSolverBody& body = bodyListStart[index2];
				Cm::SpatialVector& motionVel = motionVelocityArray[index2];
				motionVel.linear = body.linearVelocity;
				motionVel.angular = body.angularState;
				PX_ASSERT(motionVel.linear.isFinite());
				PX_ASSERT(motionVel.angular.isFinite());
			}

			nbConcluded += remainder;
			
			//Branch not required because this is the last time we use this atomic variable
			//if(index2 < articulationListSizePlusbodyListSize)
			{
				index2 = physx::shdfnd::atomicAdd(bodyListIndex, SaveUnrollCount) - SaveUnrollCount - articulationListSize;
				endIndexCount2 = SaveUnrollCount;
			}
		}

		if(nbConcluded)
		{
			Ps::memoryBarrier();
			physx::shdfnd::atomicAdd(bodyListIndex2, nbConcluded);
		}
	}


	WAIT_FOR_PROGRESS(bodyListIndex2, (bodyListSize + articulationListSize));

	a = 0;
	for(; a < velocityIterations-1; ++a)
	{
		for(PxU32 b = 0; b < nbPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(constraintIndex2, maxProgress);
			maxNormalIndex += headersPerPartition[b];
			maxProgress += headersPerPartition[b];
			
			PxI32 nbSolved = 0;
			while(index < maxNormalIndex)
			{
				const PxI32 remainder = PxMin(maxNormalIndex - index, endIndexCount);
				SolveBlockParallel(constraintList, remainder, index, batchCount, cache, contactIter, gVTableSolveBlockCoulomb, normalIteration);
				index += remainder;
				endIndexCount -= remainder;
				nbSolved += remainder;
				if(endIndexCount == 0)
				{
					endIndexCount = UnrollCount;
					index = physx::shdfnd::atomicAdd(constraintIndex, UnrollCount) - UnrollCount;
				}
			}
			if(nbSolved)
			{
				Ps::memoryBarrier();
				Ps::atomicAdd(constraintIndex2, nbSolved);
			}
		}
		++normalIteration;

		for(PxU32 b = 0; b < nbFrictionPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(constraintIndex2, maxProgress);
			maxFrictionIndex += frictionHeadersPerPartition[b];
			maxProgress += frictionHeadersPerPartition[b];

			PxI32 nbSolved = 0;
			while(frictionIndex < maxFrictionIndex)
			{
				const PxI32 remainder = PxMin(maxFrictionIndex - frictionIndex, frictionEndIndexCount);
				SolveBlockParallel(constraintList, remainder, index, batchCount, cache, contactIter, gVTableSolveBlockCoulomb, 
					normalIteration);

				frictionIndex += remainder;
				frictionEndIndexCount -= remainder;
				nbSolved += remainder;
				if(frictionEndIndexCount == 0)
				{
					frictionEndIndexCount = UnrollCount;
					frictionIndex  = physx::shdfnd::atomicAdd(frictionConstraintIndex, UnrollCount) - UnrollCount;
				}
			}
			if(nbSolved)
			{
				Ps::memoryBarrier();
				Ps::atomicAdd(constraintIndex2, nbSolved);
			}
		}

		++frictionIteration;
	}

	ThresholdStreamElement* PX_RESTRICT thresholdStream = params.thresholdStream;
	const PxU32 thresholdStreamLength = params.thresholdStreamLength;
	PxI32* outThresholdPairs = params.outThresholdPairs;

	cache.mSharedThresholdStream = thresholdStream;
	cache.mSharedOutThresholdPairs = outThresholdPairs;
	cache.mSharedThresholdStreamLength = thresholdStreamLength;

	{
		for(PxU32 b = 0; b < nbPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(constraintIndex2, maxProgress);
			maxNormalIndex += headersPerPartition[b];
			maxProgress += headersPerPartition[b];
			
			PxI32 nbSolved = 0;
			while(index < maxNormalIndex)
			{
				const PxI32 remainder = PxMin(maxNormalIndex - index, endIndexCount);

				SolveBlockParallel(constraintList, remainder, normalIteration * batchCount, batchCount, 
					cache, contactIter, gVTableSolveWriteBackBlockCoulomb, normalIteration);

				index += remainder;
				endIndexCount -= remainder;
				nbSolved += remainder;
				if(endIndexCount == 0)
				{
					endIndexCount = UnrollCount;
					index = physx::shdfnd::atomicAdd(constraintIndex, UnrollCount) - UnrollCount;
				}
			}
			if(nbSolved)
			{
				Ps::memoryBarrier();
				Ps::atomicAdd(constraintIndex2, nbSolved);
			}
		}

		++normalIteration;

		cache.mSharedOutThresholdPairs = outThresholdPairs;
		cache.mSharedThresholdStream = thresholdStream;
		cache.mSharedThresholdStreamLength = thresholdStreamLength;

		for(PxU32 b = 0; b < nbFrictionPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(constraintIndex2, maxProgress);
			maxFrictionIndex += frictionHeadersPerPartition[b];
			maxProgress += frictionHeadersPerPartition[b];

			PxI32 nbSolved = 0;
			while(frictionIndex < maxFrictionIndex)
			{
				const PxI32 remainder = PxMin(maxFrictionIndex - frictionIndex, frictionEndIndexCount);

				SolveBlockParallel(frictionConstraintList, remainder, frictionIndex, frictionBatchCount, cache, frictionIter, 
					gVTableSolveWriteBackBlockCoulomb, frictionIteration);

				frictionIndex += remainder;
				frictionEndIndexCount -= remainder;
				nbSolved += remainder;
				if(frictionEndIndexCount == 0)
				{
					frictionEndIndexCount = UnrollCount;
					frictionIndex  = physx::shdfnd::atomicAdd(frictionConstraintIndex, UnrollCount) - UnrollCount;
				}
			}
			if(nbSolved)
			{
				Ps::memoryBarrier();
				Ps::atomicAdd(constraintIndex2, nbSolved);
			}
		}

		if(cache.mThresholdStreamIndex > 0)
		{
			//Write back to global buffer
			PxI32 threshIndex = physx::shdfnd::atomicAdd(outThresholdPairs, PxI32(cache.mThresholdStreamIndex)) - PxI32(cache.mThresholdStreamIndex);
			for(PxU32 b = 0; b < cache.mThresholdStreamIndex; ++b)
			{
				thresholdStream[b + threshIndex] = cache.mThresholdStream[b];
			}
			cache.mThresholdStreamIndex = 0;
		}

		++frictionIteration;
	}

	return normalIteration * batchCount + frictionIteration * frictionBatchCount;
}


void SolverCoreGeneralPF::writeBackV
(const PxSolverConstraintDesc* PX_RESTRICT constraintList, const PxU32 /*constraintListSize*/, PxConstraintBatchHeader* batchHeaders, const PxU32 numBatches,
 ThresholdStreamElement* PX_RESTRICT thresholdStream, const PxU32 thresholdStreamLength, PxU32& outThresholdPairs,
 PxSolverBodyData* atomListData, WriteBackBlockMethod writeBackTable[]) const
{
	SolverContext cache;
	cache.solverBodyArray			= atomListData;
	cache.mThresholdStream			= thresholdStream;
	cache.mThresholdStreamLength	= thresholdStreamLength;
	cache.mThresholdStreamIndex		= 0;

	PxI32 outThreshIndex = 0;
	for(PxU32 j = 0; j < numBatches; ++j)
	{
		PxU8 type = *constraintList[batchHeaders[j].mStartIndex].constraint;
		writeBackTable[type](constraintList + batchHeaders[j].mStartIndex,
			batchHeaders[j].mStride, cache);
	}

	outThresholdPairs = PxU32(outThreshIndex);
}

}

}


//#endif
