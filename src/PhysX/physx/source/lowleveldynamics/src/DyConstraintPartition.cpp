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

#include "DyConstraintPartition.h"
#include "DyArticulationUtils.h"

#define INTERLEAVE_SELF_CONSTRAINTS 1


namespace physx
{
namespace Dy
{

namespace
{

PX_FORCE_INLINE PxU32 getArticulationIndex(const uintptr_t eaArticulation, const uintptr_t* eaArticulations, const PxU32 numEas)
{
	PxU32 index=0xffffffff;
	for(PxU32 i=0;i<numEas;i++)
	{
		if(eaArticulations[i]== eaArticulation)
		{
			index=i;
			break;
		}
	}
	PX_ASSERT(index!=0xffffffff);
	return index;
}



#define MAX_NUM_PARTITIONS 32

static PxU32 bitTable[32] = 
{
	1u<<0, 1u<<1, 1u<<2, 1u<<3, 1u<<4, 1u<<5, 1u<<6, 1u<<7, 1u<<8, 1u<<9, 1u<<10, 1u<<11, 1u<<12, 1u<<13, 1u<<14, 1u<<15, 1u<<16, 1u<<17,
	1u<<18, 1u<<19, 1u<<20, 1u<<21, 1u<<22, 1u<<23, 1u<<24, 1u<<25, 1u<<26, 1u<<27, 1u<<28, 1u<<29, 1u<<30, 1u<<31
};

PxU32 getBit(const PxU32 index)
{
	PX_ASSERT(index < 32);
	return bitTable[index];
}


class RigidBodyClassification
{
	PxSolverBody* PX_RESTRICT mBodies;
	PxU32 mNumBodies;

public:
	RigidBodyClassification(PxSolverBody* PX_RESTRICT bodies, PxU32 numBodies) : mBodies(bodies), mNumBodies(numBodies)
	{
	}

	//Returns true if it is a dynamic-dynamic constriant; false if it is a dynamic-static or dynamic-kinematic constraint
	PX_FORCE_INLINE bool classifyConstraint(const PxSolverConstraintDesc& desc, uintptr_t& indexA, uintptr_t& indexB, 
		bool& activeA, bool& activeB, PxU32& bodyAProgress, PxU32& bodyBProgress) const
	{
		indexA=uintptr_t(desc.bodyA - mBodies);
		indexB=uintptr_t(desc.bodyB - mBodies);
		activeA = indexA < mNumBodies;
		activeB = indexB < mNumBodies;
		bodyAProgress = desc.bodyA->solverProgress;
		bodyBProgress = desc.bodyB->solverProgress;
		return activeA && activeB;
	}

	PX_FORCE_INLINE void getProgress(const PxSolverConstraintDesc& desc,
		PxU32& bodyAProgress, PxU32& bodyBProgress)
	{
		bodyAProgress = desc.bodyA->solverProgress;
		bodyBProgress = desc.bodyB->solverProgress;
	}

	PX_FORCE_INLINE void storeProgress(const PxSolverConstraintDesc& desc, const PxU32 bodyAProgress, const PxU32 bodyBProgress,
		const PxU16 availablePartition)
	{
		desc.bodyA->solverProgress = bodyAProgress;
		desc.bodyA->maxSolverNormalProgress = PxMax(desc.bodyA->maxSolverNormalProgress, availablePartition);
		desc.bodyB->solverProgress = bodyBProgress;
		desc.bodyB->maxSolverNormalProgress = PxMax(desc.bodyB->maxSolverNormalProgress, availablePartition);
	}

	PX_FORCE_INLINE void storeProgress(const PxSolverConstraintDesc& desc, const PxU32 bodyAProgress, const PxU32 bodyBProgress)
	{
		desc.bodyA->solverProgress = bodyAProgress;
		desc.bodyB->solverProgress = bodyBProgress;
	}

	PX_FORCE_INLINE PxU32 getStaticContactWriteIndex(const PxSolverConstraintDesc& desc,
		bool activeA, bool activeB)
	{
		if (activeA)
			return PxU32(desc.bodyA->maxSolverNormalProgress + desc.bodyA->maxSolverFrictionProgress++);
		else if (activeB)
			return PxU32(desc.bodyB->maxSolverNormalProgress + desc.bodyB->maxSolverFrictionProgress++);
	
		return 0xffffffff;
	
	}

	PX_FORCE_INLINE void recordStaticConstraint(const PxSolverConstraintDesc& desc, bool& activeA, bool& activeB) const
	{
		if (activeA)
		{
			desc.bodyA->maxSolverFrictionProgress++;
		}

		if (activeB)
		{
			desc.bodyB->maxSolverFrictionProgress++;
		}
	}

	PX_FORCE_INLINE void clearState()
	{
		for(PxU32 a = 0; a < mNumBodies; ++a)
			mBodies[a].solverProgress = 0;
	}

	PX_FORCE_INLINE void reserveSpaceForStaticConstraints(Ps::Array<PxU32>& numConstraintsPerPartition)
	{
		for(PxU32 a = 0; a < mNumBodies; ++a)
		{
			mBodies[a].solverProgress = 0;

			PxU32 requiredSize = PxU32(mBodies[a].maxSolverNormalProgress + mBodies[a].maxSolverFrictionProgress);
			if(requiredSize > numConstraintsPerPartition.size())
			{
				numConstraintsPerPartition.resize(requiredSize);
			}

			for(PxU32 b = 0; b < mBodies[a].maxSolverFrictionProgress; ++b)
			{
				numConstraintsPerPartition[mBodies[a].maxSolverNormalProgress + b]++;
			}
		}
	}
};

class ExtendedRigidBodyClassification
{

	PxSolverBody* PX_RESTRICT mBodies;
	PxU32 mNumBodies;
	//uintptr_t* PX_RESTRICT mFsDatas;
	uintptr_t* PX_RESTRICT mArticulations;
	PxU32 mNumArticulations;

public:

	ExtendedRigidBodyClassification(PxSolverBody* PX_RESTRICT bodies, PxU32 numBodies, uintptr_t* articulations, PxU32 numArticulations)
		: mBodies(bodies), mNumBodies(numBodies), mArticulations(articulations), mNumArticulations(numArticulations)
	{
	}

	//Returns true if it is a dynamic-dynamic constriant; false if it is a dynamic-static or dynamic-kinematic constraint
	PX_FORCE_INLINE bool classifyConstraint(const PxSolverConstraintDesc& desc, uintptr_t& indexA, uintptr_t& indexB, 
		bool& activeA, bool& activeB, PxU32& bodyAProgress, PxU32& bodyBProgress) const
	{
		bool hasStatic = false;
		if(PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
		{
			indexA=uintptr_t(desc.bodyA - mBodies);
			activeA = indexA < mNumBodies;
			hasStatic = desc.bodyADataIndex == 0;
			bodyAProgress = activeA ? desc.bodyA->solverProgress: 0;
		}
		else
		{
			
			ArticulationV* articulationA = desc.articulationA;
			indexA=mNumBodies+getArticulationIndex(uintptr_t(articulationA), mArticulations ,mNumArticulations);
			//bodyAProgress = articulationA->getFsDataPtr()->solverProgress;
			bodyAProgress = articulationA->solverProgress;
			activeA = true;
		}
		if(PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
		{
			indexB=uintptr_t(desc.bodyB - mBodies);
			activeB = indexB < mNumBodies;
			hasStatic = hasStatic || desc.bodyBDataIndex == 0;
			bodyBProgress = activeB ? desc.bodyB->solverProgress : 0;
		}
		else
		{
			Articulation* articulationB = static_cast<Articulation*>(desc.articulationB);
			indexB=mNumBodies+getArticulationIndex(uintptr_t(articulationB), mArticulations, mNumArticulations);
			activeB = true;
			bodyBProgress = articulationB->solverProgress;
		}
		return !hasStatic;
	}

	PX_FORCE_INLINE void getProgress(const PxSolverConstraintDesc& desc,
		PxU32& bodyAProgress, PxU32& bodyBProgress) const
	{
		if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
		{
			bodyAProgress = desc.bodyA->solverProgress;
		}
		else
		{
			ArticulationV* articulationA = desc.articulationA;
			bodyAProgress = articulationA->solverProgress;
		}

		if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
		{
			
			bodyBProgress = desc.bodyB->solverProgress;
		}
		else
		{
			ArticulationV* articulationB = desc.articulationB;
			bodyBProgress = articulationB->solverProgress;
		}
	}


	PX_FORCE_INLINE void storeProgress(const PxSolverConstraintDesc& desc, const PxU32 bodyAProgress, const PxU32 bodyBProgress,
		const PxU16 availablePartition)
	{
		if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
		{
			desc.bodyA->solverProgress = bodyAProgress;
			desc.bodyA->maxSolverNormalProgress = PxMax(desc.bodyA->maxSolverNormalProgress, availablePartition);
		}
		else
		{
			ArticulationV* articulationA = desc.articulationA;
			articulationA->solverProgress = bodyAProgress;
			articulationA->maxSolverNormalProgress = PxMax(articulationA->maxSolverNormalProgress, availablePartition);
		}

		if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
		{
			desc.bodyB->solverProgress = bodyBProgress;
			desc.bodyB->maxSolverNormalProgress = PxMax(desc.bodyB->maxSolverNormalProgress, availablePartition);
		}
		else
		{
			ArticulationV* articulationB = desc.articulationB;
			articulationB->solverProgress = bodyBProgress;
			articulationB->maxSolverNormalProgress = PxMax(articulationB->maxSolverNormalProgress, availablePartition);
		}
	}

	PX_FORCE_INLINE void storeProgress(const PxSolverConstraintDesc& desc, const PxU32 bodyAProgress,
		const PxU32 bodyBProgress)
	{
		if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
		{
			desc.bodyA->solverProgress = bodyAProgress;
		}
		else
		{
			ArticulationV* articulationA = desc.articulationA;
			articulationA->solverProgress = bodyAProgress;
		}

		if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
		{
			desc.bodyB->solverProgress = bodyBProgress;
		}
		else
		{
			ArticulationV* articulationB = desc.articulationB;
			articulationB->solverProgress = bodyBProgress;
		}
	}

	PX_FORCE_INLINE void recordStaticConstraint(const PxSolverConstraintDesc& desc, bool& activeA, bool& activeB)
	{
		if (activeA)
		{
			if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
				desc.bodyA->maxSolverFrictionProgress++;
			else
			{
				ArticulationV* articulationA = desc.articulationA;
				articulationA->maxSolverFrictionProgress++;
			}
		}

		if (activeB)
		{
			if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
				desc.bodyB->maxSolverFrictionProgress++;
			else
			{
				ArticulationV* articulationB = desc.articulationB;
				articulationB->maxSolverFrictionProgress++;
			}
		}
	}

	PX_FORCE_INLINE PxU32 getStaticContactWriteIndex(const PxSolverConstraintDesc& desc,
		bool activeA, bool activeB)
	{
		if (activeA)
		{
			if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
			{
				return PxU32(desc.bodyA->maxSolverNormalProgress + desc.bodyA->maxSolverFrictionProgress++);
			}
			else
			{
				ArticulationV* articulationA = desc.articulationA;
				return PxU32(articulationA->maxSolverNormalProgress + articulationA->maxSolverFrictionProgress++);
			}
		}
		else if (activeB)
		{
			if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
			{
				return PxU32(desc.bodyB->maxSolverNormalProgress + desc.bodyB->maxSolverFrictionProgress++);
			}
			else
			{
				ArticulationV* articulationB = desc.articulationB;
				return PxU32(articulationB->maxSolverNormalProgress + articulationB->maxSolverFrictionProgress++);
			}
		}

		return 0xffffffff;
	}

	PX_FORCE_INLINE void clearState()
	{
		for(PxU32 a = 0; a < mNumBodies; ++a)
			mBodies[a].solverProgress = 0;

		for(PxU32 a = 0; a < mNumArticulations; ++a)
			(reinterpret_cast<ArticulationV*>(mArticulations[a]))->solverProgress = 0;
	}

	PX_FORCE_INLINE void reserveSpaceForStaticConstraints(Ps::Array<PxU32>& numConstraintsPerPartition)
	{
		for(PxU32 a = 0; a < mNumBodies; ++a)
		{
			mBodies[a].solverProgress = 0;

			PxU32 requiredSize = PxU32(mBodies[a].maxSolverNormalProgress + mBodies[a].maxSolverFrictionProgress);
			if(requiredSize > numConstraintsPerPartition.size())
			{
				numConstraintsPerPartition.resize(requiredSize);
			}

			for(PxU32 b = 0; b < mBodies[a].maxSolverFrictionProgress; ++b)
			{
				numConstraintsPerPartition[mBodies[a].maxSolverNormalProgress + b]++;
			}
		}

		for(PxU32 a = 0; a < mNumArticulations; ++a)
		{
			ArticulationV* articulation = reinterpret_cast<ArticulationV*>(mArticulations[a]);
			articulation->solverProgress = 0;

			PxU32 requiredSize = PxU32(articulation->maxSolverNormalProgress + articulation->maxSolverFrictionProgress);
			if(requiredSize > numConstraintsPerPartition.size())
			{
				numConstraintsPerPartition.resize(requiredSize);
			}

			for(PxU32 b = 0; b < articulation->maxSolverFrictionProgress; ++b)
			{
				numConstraintsPerPartition[articulation->maxSolverNormalProgress + b]++;
			}
		}
	}

};

template <typename Classification>
void classifyConstraintDesc(const PxSolverConstraintDesc* PX_RESTRICT descs, const PxU32 numConstraints, Classification& classification, 
							Ps::Array<PxU32>& numConstraintsPerPartition, PxSolverConstraintDesc* PX_RESTRICT eaTempConstraintDescriptors)
{
	const PxSolverConstraintDesc* _desc = descs;
	const PxU32 numConstraintsMin1 = numConstraints - 1;

	PxU32 numUnpartitionedConstraints = 0;

	numConstraintsPerPartition.forceSize_Unsafe(32);

	PxMemZero(numConstraintsPerPartition.begin(), sizeof(PxU32) * 32);

	for(PxU32 i = 0; i < numConstraints; ++i, _desc++)
	{
		const PxU32 prefetchOffset = PxMin(numConstraintsMin1 - i, 4u);
		Ps::prefetchLine(_desc[prefetchOffset].constraint);
		Ps::prefetchLine(_desc[prefetchOffset].bodyA);
		Ps::prefetchLine(_desc[prefetchOffset].bodyB);
		Ps::prefetchLine(_desc + 8);

		uintptr_t indexA, indexB;
		bool activeA, activeB;

		PxU32 partitionsA, partitionsB;
		const bool notContainsStatic = classification.classifyConstraint(*_desc, indexA, indexB, activeA, activeB,
			partitionsA, partitionsB);
		
		if(notContainsStatic)
		{			
			PxU32 availablePartition;
			{
				const PxU32 combinedMask = (~partitionsA & ~partitionsB);
				availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : Ps::lowestSetBit(combinedMask);
				if(availablePartition == MAX_NUM_PARTITIONS)
				{
					eaTempConstraintDescriptors[numUnpartitionedConstraints++] = *_desc;
					continue;
				}

				const PxU32 partitionBit = getBit(availablePartition);
				if (activeA)
					partitionsA |= partitionBit;
				if(activeB)
					partitionsB |= partitionBit;
			}

			numConstraintsPerPartition[availablePartition]++;
			availablePartition++;

			classification.storeProgress(*_desc, partitionsA, partitionsB, PxU16(availablePartition));
		}
		else
		{
			classification.recordStaticConstraint(*_desc, activeA, activeB);
		}
	}

	PxU32 partitionStartIndex = 0;

	while(numUnpartitionedConstraints > 0)
	{
		classification.clearState();

		partitionStartIndex += 32;
		//Keep partitioning the un-partitioned constraints and blat the whole thing to 0!
		numConstraintsPerPartition.resize(32 + numConstraintsPerPartition.size());
		PxMemZero(numConstraintsPerPartition.begin() + partitionStartIndex, sizeof(PxU32) * 32);

		PxU32 newNumUnpartitionedConstraints = 0;
		PxU32 partitionsA, partitionsB;
		bool activeA, activeB;
		uintptr_t indexA, indexB;
		for(PxU32 i = 0; i < numUnpartitionedConstraints; ++i)
		{
			const PxSolverConstraintDesc& desc = eaTempConstraintDescriptors[i];
			
			classification.classifyConstraint(desc, indexA, indexB, activeA, activeB,
				partitionsA, partitionsB);
			
			PxU32 availablePartition;
			{
				const PxU32 combinedMask = (~partitionsA & ~partitionsB);
				availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : Ps::lowestSetBit(combinedMask);
				if(availablePartition == MAX_NUM_PARTITIONS)
				{
					//Need to shuffle around unpartitioned constraints...
					eaTempConstraintDescriptors[newNumUnpartitionedConstraints++] = desc;
					continue;
				}

				const PxU32 partitionBit = getBit(availablePartition);
				if(activeA)
					partitionsA |= partitionBit;
				if(activeB)
					partitionsB |= partitionBit;
			}

			
			/*desc.bodyA->solverProgress = partitionsA;
			desc.bodyB->solverProgress = partitionsB;*/
			availablePartition += partitionStartIndex;
			numConstraintsPerPartition[availablePartition]++;
			availablePartition++;

			classification.storeProgress(desc, partitionsA, partitionsB, PxU16(availablePartition) );

		/*	desc.bodyA->maxSolverNormalProgress = PxMax(desc.bodyA->maxSolverNormalProgress, PxU16(availablePartition));
			desc.bodyB->maxSolverNormalProgress = PxMax(desc.bodyB->maxSolverNormalProgress, PxU16(availablePartition));*/
		}

		numUnpartitionedConstraints = newNumUnpartitionedConstraints;
	}

	classification.reserveSpaceForStaticConstraints(numConstraintsPerPartition);

}

template <typename Classification>
void writeConstraintDesc(const PxSolverConstraintDesc* PX_RESTRICT descs, const PxU32 numConstraints, Classification& classification,
						 Ps::Array<PxU32>& accumulatedConstraintsPerPartition, PxSolverConstraintDesc* eaTempConstraintDescriptors,
							PxSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDesc)
{
	PX_UNUSED(eaTempConstraintDescriptors);
	const PxSolverConstraintDesc* _desc = descs;
	const PxU32 numConstraintsMin1 = numConstraints - 1;

	PxU32 numUnpartitionedConstraints = 0;

	for(PxU32 i = 0; i < numConstraints; ++i, _desc++)
	{
		const PxU32 prefetchOffset = PxMin(numConstraintsMin1 - i, 4u);
		Ps::prefetchLine(_desc[prefetchOffset].constraint);
		Ps::prefetchLine(_desc[prefetchOffset].bodyA);
		Ps::prefetchLine(_desc[prefetchOffset].bodyB);
		Ps::prefetchLine(_desc + 8);

		uintptr_t indexA, indexB;
		bool activeA, activeB;
		PxU32 partitionsA, partitionsB;
		const bool notContainsStatic = classification.classifyConstraint(*_desc, indexA, indexB, activeA, activeB, partitionsA, partitionsB);

		if(notContainsStatic)
		{
			PxU32 availablePartition;
			{
				const PxU32 combinedMask = (~partitionsA & ~partitionsB);
				availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : Ps::lowestSetBit(combinedMask);
				if(availablePartition == MAX_NUM_PARTITIONS)
				{
					eaTempConstraintDescriptors[numUnpartitionedConstraints++] = *_desc;
					continue;
				}

				const PxU32 partitionBit = getBit(availablePartition);
				if(activeA)
					partitionsA |= partitionBit;
				if(activeB)
					partitionsB |= partitionBit;
			}

			classification.storeProgress(*_desc, partitionsA, partitionsB, PxU16(availablePartition + 1));

			eaOrderedConstraintDesc[accumulatedConstraintsPerPartition[availablePartition]++] = *_desc;
		}
		else
		{
			//Just count the number of static constraints and store in maxSolverFrictionProgress...
			PxU32 index = classification.getStaticContactWriteIndex(*_desc, activeA, activeB);
			
			eaOrderedConstraintDesc[accumulatedConstraintsPerPartition[index]++] = *_desc;
		}
	}

	PxU32 partitionStartIndex = 0;

	while(numUnpartitionedConstraints > 0)
	{
		classification.clearState();

		partitionStartIndex += 32;	
		PxU32 newNumUnpartitionedConstraints = 0;

		PxU32 partitionsA, partitionsB;
		bool activeA, activeB;
		uintptr_t indexA, indexB;
		for(PxU32 i = 0; i < numUnpartitionedConstraints; ++i)
		{
			const PxSolverConstraintDesc& desc = eaTempConstraintDescriptors[i];
			
		/*	PxU32 partitionsA=desc.bodyA->solverProgress;
			PxU32 partitionsB=desc.bodyB->solverProgress;*/

			classification.classifyConstraint(desc, indexA, indexB,
				activeA, activeB, partitionsA, partitionsB);
				
			PxU32 availablePartition;
			{
				const PxU32 combinedMask = (~partitionsA & ~partitionsB);
				availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : Ps::lowestSetBit(combinedMask);
				if(availablePartition == MAX_NUM_PARTITIONS)
				{
					//Need to shuffle around unpartitioned constraints...
					eaTempConstraintDescriptors[newNumUnpartitionedConstraints++] = desc;
					continue;
				}

				const PxU32 partitionBit = getBit(availablePartition);

				if(activeA)
					partitionsA |= partitionBit;
				if(activeB)
					partitionsB |= partitionBit;
			}

			/*desc.bodyA->solverProgress = partitionsA;
			desc.bodyB->solverProgress = partitionsB;
*/
			classification.storeProgress(desc, partitionsA, partitionsB);
			availablePartition += partitionStartIndex;
			eaOrderedConstraintDesc[accumulatedConstraintsPerPartition[availablePartition]++] = desc;
		}

		numUnpartitionedConstraints = newNumUnpartitionedConstraints;
	}
}

}

#define PX_NORMALIZE_PARTITIONS 1

#if PX_NORMALIZE_PARTITIONS

template<typename Classification>
PxU32 normalizePartitions(Ps::Array<PxU32>& accumulatedConstraintsPerPartition, PxSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDescriptors, 
	const PxU32 numConstraintDescriptors, Ps::Array<PxU32>& bitField, const Classification& classification, const PxU32 numBodies, const PxU32 numArticulations)
{
	PxU32 numPartitions = 0;
	
	PxU32 prevAccumulation = 0;
	for(; numPartitions < accumulatedConstraintsPerPartition.size() && accumulatedConstraintsPerPartition[numPartitions] > prevAccumulation; 
		prevAccumulation = accumulatedConstraintsPerPartition[numPartitions++]);

	PxU32 targetSize = (numPartitions == 0 ? 0 : (numConstraintDescriptors)/numPartitions);

	bitField.reserve((numBodies + numArticulations + 31)/32);
	bitField.forceSize_Unsafe((numBodies + numArticulations + 31)/32);

	for(PxU32 i = numPartitions; i > 0; i--)
	{
		PxU32 partitionIndex = i-1;

		//Build the partition mask...

		PxU32 startIndex = partitionIndex == 0 ? 0 : accumulatedConstraintsPerPartition[partitionIndex-1];
		PxU32 endIndex = accumulatedConstraintsPerPartition[partitionIndex];

		//If its greater than target size, there's nothing that will be pulled into it from earlier partitions
		if((endIndex - startIndex) >= targetSize)
			continue;


		PxMemZero(bitField.begin(), sizeof(PxU32)*bitField.size());

		for(PxU32 a = startIndex; a < endIndex; ++a)
		{
			PxSolverConstraintDesc& desc = eaOrderedConstraintDescriptors[a];

			uintptr_t indexA, indexB;
			bool activeA, activeB;
			PxU32 partitionsA, partitionsB;

			classification.classifyConstraint(desc, indexA, indexB, activeA, activeB, partitionsA, partitionsB);

			if(activeA)
				bitField[PxU32(indexA)/32] |= getBit(indexA & 31);
			if(activeB)
				bitField[PxU32(indexB)/32] |= getBit(indexB & 31);
		}

		bool bTerm = false;
		for(PxU32 a = partitionIndex; a > 0 && !bTerm; --a)
		{
			PxU32 pInd = a-1;

			PxU32 si = pInd == 0 ? 0 : accumulatedConstraintsPerPartition[pInd-1];
			PxU32 ei = accumulatedConstraintsPerPartition[pInd];

			for(PxU32 b = ei; b > si && !bTerm; --b)
			{
				PxU32 ind = b-1;
				PxSolverConstraintDesc& desc = eaOrderedConstraintDescriptors[ind];

				uintptr_t indexA, indexB;
				bool activeA, activeB;
				PxU32 partitionsA, partitionsB;

				classification.classifyConstraint(desc, indexA, indexB, activeA, activeB, partitionsA, partitionsB);

				bool canAdd = true;

				if(activeA && (bitField[PxU32(indexA)/32] & (getBit(indexA & 31))))
					canAdd = false;
				if(activeB && (bitField[PxU32(indexB)/32] & (getBit(indexB & 31))))
					canAdd = false;

				if(canAdd)
				{
					PxSolverConstraintDesc tmp = eaOrderedConstraintDescriptors[ind];

					if(activeA)
						bitField[PxU32(indexA)/32] |= (getBit(indexA & 31));
					if(activeB)
						bitField[PxU32(indexB)/32] |= (getBit(indexB & 31));

					PxU32 index = ind;
					for(PxU32 c = pInd; c < partitionIndex; ++c)
					{
						PxU32 newIndex = --accumulatedConstraintsPerPartition[c];
						if(index != newIndex)
							eaOrderedConstraintDescriptors[index] = eaOrderedConstraintDescriptors[newIndex];	
						index = newIndex;
					}

					if(index != ind)
						eaOrderedConstraintDescriptors[index] = tmp;

					if((accumulatedConstraintsPerPartition[partitionIndex] - accumulatedConstraintsPerPartition[partitionIndex-1]) >= targetSize)
					{
						bTerm = true;
						break;
					}
				}
			}
		}
	}
		
	PxU32 partitionCount = 0;
	PxU32 lastPartitionCount = 0;
	for (PxU32 a = 0; a < numPartitions; ++a)
	{
		const PxU32 constraintCount = accumulatedConstraintsPerPartition[a];
		accumulatedConstraintsPerPartition[partitionCount] = constraintCount;
		if (constraintCount != lastPartitionCount)
		{
			lastPartitionCount = constraintCount;
			partitionCount++;
		}
	}

	accumulatedConstraintsPerPartition.forceSize_Unsafe(partitionCount);

	return partitionCount;
}

#endif

PxU32 partitionContactConstraints(ConstraintPartitionArgs& args) 
{
	PxU32 maxPartition = 0;
	//Unpack the input data.
	const PxU32 numBodies=args.mNumBodies;
	PxSolverBody* PX_RESTRICT eaAtoms=args.mBodies;
	const PxU32	numArticulations=args.mNumArticulationPtrs;
	
	const PxU32 numConstraintDescriptors=args.mNumContactConstraintDescriptors;

	PxSolverConstraintDesc* PX_RESTRICT eaConstraintDescriptors=args.mContactConstraintDescriptors;
	PxSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDescriptors=args.mOrderedContactConstraintDescriptors;
	PxSolverConstraintDesc* PX_RESTRICT eaTempConstraintDescriptors=args.mTempContactConstraintDescriptors;

	Ps::Array<PxU32>& constraintsPerPartition = *args.mConstraintsPerPartition;
	constraintsPerPartition.forceSize_Unsafe(0);

	for(PxU32 a = 0; a < numBodies; ++a)
	{
		PxSolverBody& body = args.mBodies[a];
		Ps::prefetchLine(&args.mBodies[a], 256);
		body.solverProgress = 0;
		//We re-use maxSolverFrictionProgress and maxSolverNormalProgress to record the
		//maximum partition used by dynamic constraints and the number of static constraints affecting
		//a body. We use this to make partitioning much cheaper and be able to support 
		body.maxSolverFrictionProgress = 0;
		body.maxSolverNormalProgress = 0;
	}

	PxU32 numOrderedConstraints=0;	

	PxU32 numSelfConstraintBlocks=0;

	if(numArticulations == 0)
	{
		RigidBodyClassification classification(eaAtoms, numBodies);
		classifyConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition,
			eaTempConstraintDescriptors);
		
		PxU32 accumulation = 0;
		for(PxU32 a = 0; a < constraintsPerPartition.size(); ++a)
		{
			PxU32 count = constraintsPerPartition[a];
			constraintsPerPartition[a] = accumulation;
			accumulation += count;
		}

		for(PxU32 a = 0; a < numBodies; ++a)
		{
			PxSolverBody& body = args.mBodies[a];
			Ps::prefetchLine(&args.mBodies[a], 256);
			body.solverProgress = 0;
			//Keep the dynamic constraint count but bump the static constraint count back to 0.
			//This allows us to place the static constraints in the appropriate place when we see them
			//because we know the maximum index for the dynamic constraints...
			body.maxSolverFrictionProgress = 0;
		}

		writeConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition, 
			eaTempConstraintDescriptors, eaOrderedConstraintDescriptors);

		numOrderedConstraints = numConstraintDescriptors;

		if(!args.enhancedDeterminism)
			maxPartition = normalizePartitions(constraintsPerPartition, eaOrderedConstraintDescriptors, numConstraintDescriptors, *args.mBitField,
				classification, numBodies, 0);

	}
	else
	{
		
		const ArticulationSolverDesc* articulationDescs=args.mArticulationPtrs;
		PX_ALLOCA(_eaArticulations, uintptr_t, numArticulations);
		uintptr_t* eaArticulations = _eaArticulations;
		for(PxU32 i=0;i<numArticulations;i++)
		{
			ArticulationV* articulation = articulationDescs[i].articulation;
			eaArticulations[i]=uintptr_t(articulation);
			articulation->solverProgress = 0;
			articulation->maxSolverFrictionProgress = 0;
			articulation->maxSolverNormalProgress = 0;
		}
		ExtendedRigidBodyClassification classification(eaAtoms, numBodies, eaArticulations, numArticulations);

		classifyConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, 
			constraintsPerPartition, eaTempConstraintDescriptors);

		PxU32 accumulation = 0;
		for(PxU32 a = 0; a < constraintsPerPartition.size(); ++a)
		{
			PxU32 count = constraintsPerPartition[a];
			constraintsPerPartition[a] = accumulation;
			accumulation += count;
		}

		for(PxU32 a = 0; a < numBodies; ++a)
		{
			PxSolverBody& body = args.mBodies[a];
			Ps::prefetchLine(&args.mBodies[a], 256);
			body.solverProgress = 0;
			//Keep the dynamic constraint count but bump the static constraint count back to 0.
			//This allows us to place the static constraints in the appropriate place when we see them
			//because we know the maximum index for the dynamic constraints...
			body.maxSolverFrictionProgress = 0;
		}

		for(PxU32 a = 0; a < numArticulations; ++a)
		{
			ArticulationV* articulation = reinterpret_cast<ArticulationV*>(eaArticulations[a]);
			articulation->solverProgress = 0;
			articulation->maxSolverFrictionProgress = 0;
		}

		writeConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition, 
			eaTempConstraintDescriptors, eaOrderedConstraintDescriptors);

		numOrderedConstraints = numConstraintDescriptors;

		if (!args.enhancedDeterminism)
			maxPartition = normalizePartitions(constraintsPerPartition, eaOrderedConstraintDescriptors,  
				numConstraintDescriptors, *args.mBitField, classification, numBodies, numArticulations);

	}



	const PxU32 numConstraintsDifferentBodies=numOrderedConstraints;

	PX_ASSERT(numConstraintsDifferentBodies == numConstraintDescriptors);

	//Now handle the articulated self-constraints.
	PxU32 totalConstraintCount = numConstraintsDifferentBodies;	

	args.mNumSelfConstraintBlocks=numSelfConstraintBlocks;

	args.mNumDifferentBodyConstraints=numConstraintsDifferentBodies;
	args.mNumSelfConstraints=totalConstraintCount-numConstraintsDifferentBodies;

	if (args.enhancedDeterminism)
	{
		PxU32 prevPartitionSize = 0;
		maxPartition = 0;
		for (PxU32 a = 0; a < constraintsPerPartition.size(); ++a, maxPartition++)
		{
			if (constraintsPerPartition[a] == prevPartitionSize)
				break;
			prevPartitionSize = constraintsPerPartition[a];
		}
	}

	return maxPartition;
}

}

}
