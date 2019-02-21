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

#include "DyTGSPartition.h"
#include "DyArticulationUtils.h"
#include "PsAlloca.h"
#include "foundation/PxMemory.h"

#define INTERLEAVE_SELF_CONSTRAINTS 1


namespace physx
{
	namespace Dy
	{

		namespace
		{

			PX_FORCE_INLINE PxU32 getArticulationIndex(const uintptr_t eaArticulation, const uintptr_t* eaArticulations, const PxU32 numEas)
			{
				PxU32 index = 0xffffffff;
				for (PxU32 i = 0; i<numEas; i++)
				{
					if (eaArticulations[i] == eaArticulation)
					{
						index = i;
						break;
					}
				}
				PX_ASSERT(index != 0xffffffff);
				return index;
			}


#define MAX_NUM_PARTITIONS 32

			static PxU32 bitTable[32] =
			{
				1u << 0, 1u << 1, 1u << 2, 1u << 3, 1u << 4, 1u << 5, 1u << 6, 1u << 7, 1u << 8, 1u << 9, 1u << 10, 1u << 11, 1u << 12, 1u << 13, 1u << 14, 1u << 15, 1u << 16, 1u << 17,
				1u << 18, 1u << 19, 1u << 20, 1u << 21, 1u << 22, 1u << 23, 1u << 24, 1u << 25, 1u << 26, 1u << 27, 1u << 28, 1u << 29, 1u << 30, 1u << 31
			};

			PxU32 getBit(const PxU32 index)
			{
				PX_ASSERT(index < 32);
				return bitTable[index];
			}


			class RigidBodyClassification
			{
				PxTGSSolverBodyVel* PX_RESTRICT mBodies;
				PxU32 mNumBodies;

			public:
				RigidBodyClassification(PxTGSSolverBodyVel* PX_RESTRICT bodies, PxU32 numBodies) : mBodies(bodies), mNumBodies(numBodies)
				{
				}

				//Returns true if it is a dynamic-dynamic constriant; false if it is a dynamic-static or dynamic-kinematic constraint
				PX_FORCE_INLINE bool classifyConstraint(const PxTGSSolverConstraintDesc& desc, uintptr_t& indexA, uintptr_t& indexB,
					bool& activeA, bool& activeB, PxU32& bodyAProgress, PxU32& bodyBProgress) const
				{
					indexA = uintptr_t(desc.bodyA - mBodies);
					indexB = uintptr_t(desc.bodyB - mBodies);
					activeA = indexA < mNumBodies;
					activeB = indexB < mNumBodies;
					bodyAProgress = desc.bodyA->partitionMask;
					bodyBProgress = desc.bodyB->partitionMask;
					return desc.bodyAIdx != 0 && desc.bodyBIdx != 0;
				}

				PX_FORCE_INLINE void getProgress(const PxTGSSolverConstraintDesc& desc,
					PxU32& bodyAProgress, PxU32& bodyBProgress)
				{
					bodyAProgress = desc.bodyA->partitionMask;
					bodyBProgress = desc.bodyB->partitionMask;
				}

				PX_FORCE_INLINE void storeProgress(const PxTGSSolverConstraintDesc& desc, const PxU32 bodyAProgress, const PxU32 bodyBProgress,
					const PxU16 availablePartition)
				{
					desc.bodyA->partitionMask = bodyAProgress;
					desc.bodyA->maxDynamicPartition = PxMax(desc.bodyA->maxDynamicPartition, availablePartition);
					desc.bodyB->partitionMask = bodyBProgress;
					desc.bodyB->maxDynamicPartition = PxMax(desc.bodyB->maxDynamicPartition, availablePartition);
				}

				PX_FORCE_INLINE void storeProgress(const PxTGSSolverConstraintDesc& desc, const bool activeA, const bool activeB,
					const PxU32 bodyAProgress, const PxU32 bodyBProgress)
				{
					if(activeA)
						desc.bodyA->partitionMask = bodyAProgress;
					if(activeB)
						desc.bodyB->partitionMask = bodyBProgress;
				}

				PX_FORCE_INLINE void recordStaticConstraint(const PxTGSSolverConstraintDesc& desc, bool& activeA, bool& activeB)
				{
					if (activeA)
						desc.bodyA->nbStaticInteractions++;
						
					if (activeB)
						desc.bodyB->nbStaticInteractions++;
						
				}

				PX_FORCE_INLINE PxU32 getStaticContactWriteIndex(const PxTGSSolverConstraintDesc& desc,
					bool activeA, bool activeB)
				{
					if (activeA)
					{
						return PxU32(desc.bodyA->maxDynamicPartition + desc.bodyA->nbStaticInteractions++);
						
					}
					else if (activeB)
					{
						return PxU32(desc.bodyB->maxDynamicPartition + desc.bodyB->nbStaticInteractions++);
					}

					return 0xffffffff;
				}

				PX_FORCE_INLINE void clearState()
				{
					for (PxU32 a = 0; a < mNumBodies; ++a)
						mBodies[a].partitionMask = 0;
				}

				PX_FORCE_INLINE void reserveSpaceForStaticConstraints(Ps::Array<PxU32>& numConstraintsPerPartition)
				{
					for (PxU32 a = 0; a < mNumBodies; ++a)
					{
						mBodies[a].partitionMask = 0;

						PxU32 requiredSize = PxU32(mBodies[a].maxDynamicPartition + mBodies[a].nbStaticInteractions);
						if (requiredSize > numConstraintsPerPartition.size())
						{
							numConstraintsPerPartition.resize(requiredSize);
						}

						for (PxU32 b = 0; b < mBodies[a].nbStaticInteractions; ++b)
						{
							numConstraintsPerPartition[mBodies[a].maxDynamicPartition + b]++;
						}
					}
				}
			};

			class ExtendedRigidBodyClassification
			{

				PxTGSSolverBodyVel* PX_RESTRICT mBodies;
				PxU32 mNumBodies;
				uintptr_t* PX_RESTRICT mArticulations;
				PxU32 mNumArticulations;
				

			public:

				ExtendedRigidBodyClassification(PxTGSSolverBodyVel* PX_RESTRICT bodies, PxU32 numBodies, uintptr_t* PX_RESTRICT articulations,
					PxU32 numArticulations): mBodies(bodies), mNumBodies(numBodies), 
					mArticulations(articulations), mNumArticulations(numArticulations)
				{
				}

				//Returns true if it is a dynamic-dynamic constriant; false if it is a dynamic-static or dynamic-kinematic constraint
				PX_FORCE_INLINE bool classifyConstraint(const PxTGSSolverConstraintDesc& desc, uintptr_t& indexA, uintptr_t& indexB,
					bool& activeA, bool& activeB, PxU32& bodyAProgress, PxU32& bodyBProgress) const
				{
					bool staticA = false, staticB = false;
					if (PxTGSSolverConstraintDesc::NO_LINK == desc.linkIndexA)
					{
						indexA = uintptr_t(desc.bodyA - mBodies);
						activeA = indexA < mNumBodies;
						staticA = desc.bodyAIdx == 0;
						bodyAProgress = activeA ? desc.bodyA->partitionMask : 0;
					}
					else
					{
						ArticulationV* articulationA = desc.articulationA;
						indexA = mNumBodies + getArticulationIndex(uintptr_t(articulationA), mArticulations, mNumArticulations);
						activeA = true;
						bodyAProgress = articulationA->solverProgress;
					}

					if (PxTGSSolverConstraintDesc::NO_LINK == desc.linkIndexB)
					{
						indexB = uintptr_t(desc.bodyB - mBodies);
						activeB = indexB < mNumBodies;
						staticB = desc.bodyBIdx == 0;
						bodyBProgress = activeB ? desc.bodyB->partitionMask : 0;
					}
					else
					{
						ArticulationV* articulationB = desc.articulationB;
						indexB = mNumBodies + getArticulationIndex(uintptr_t(articulationB), mArticulations, mNumArticulations);
						activeB = true;
						bodyBProgress = articulationB->solverProgress;
					}
					return !(staticA || staticB);
				}

				PX_FORCE_INLINE void getProgress(const PxTGSSolverConstraintDesc& desc,
					PxU32& bodyAProgress, PxU32& bodyBProgress)
				{
					if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
					{
						bodyAProgress = desc.bodyA->partitionMask;
					}
					else
					{
						ArticulationV* articulationA = desc.articulationA;
						bodyAProgress = articulationA->solverProgress;
					}

					if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
					{
						bodyBProgress = desc.bodyB->partitionMask;
					}
					else
					{
						ArticulationV* articulationB = desc.articulationB;
						bodyBProgress = articulationB->solverProgress;
					}
				}

				//called by classifyConstraintDesc
				PX_FORCE_INLINE void storeProgress(const PxTGSSolverConstraintDesc& desc, const PxU32 bodyAProgress, const PxU32 bodyBProgress,
					const PxU16 availablePartition)
				{
					if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
					{
						desc.bodyA->partitionMask = bodyAProgress;
						desc.bodyA->maxDynamicPartition = PxMax(desc.bodyA->maxDynamicPartition, availablePartition);
					}
					else
					{
						ArticulationV* articulationA = desc.articulationA;
						articulationA->solverProgress = bodyAProgress;
						articulationA->maxSolverFrictionProgress = PxMax(articulationA->maxSolverFrictionProgress, availablePartition);
					}

					if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
					{
						desc.bodyB->partitionMask = bodyBProgress;
						desc.bodyB->maxDynamicPartition = PxMax(desc.bodyB->maxDynamicPartition, availablePartition);
					}
					else
					{
						ArticulationV* articulationB = desc.articulationB;
						articulationB->solverProgress = bodyBProgress;
						articulationB->maxSolverFrictionProgress = PxMax(articulationB->maxSolverFrictionProgress, availablePartition);
					}
				}

				//called by writeConstraintDesc
				PX_FORCE_INLINE void storeProgress(const PxTGSSolverConstraintDesc& desc,
					const bool activeA, const bool activeB, const PxU32 bodyAProgress, const PxU32 bodyBProgress)
				{
					if (activeA)
					{
						if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
						{
							desc.bodyA->partitionMask = bodyAProgress;
						}
						else
						{
							ArticulationV* articulationA = desc.articulationA;
							articulationA->solverProgress = bodyAProgress;
							articulationA->numTotalConstraints++;
						}
					}

					if (activeB)
					{
						if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
						{
							desc.bodyB->partitionMask = bodyBProgress;
						}
						else
						{
							ArticulationV* articulationB = desc.articulationB;
							articulationB->solverProgress = bodyBProgress;
							articulationB->numTotalConstraints++;
						}
					}
				}

				//called by classifyConstraintDesc
				PX_FORCE_INLINE void recordStaticConstraint(const PxTGSSolverConstraintDesc& desc, bool& activeA, bool& activeB)
				{
					if (activeA)
					{
						if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
							desc.bodyA->nbStaticInteractions++;
						else
						{
							ArticulationV* articulationA = desc.articulationA;
							articulationA->maxSolverNormalProgress++;
						}
					}

					if (activeB)
					{
						if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
							desc.bodyB->nbStaticInteractions++;
						else
						{
							ArticulationV* articulationB = desc.articulationB;
							articulationB->maxSolverNormalProgress++;
						}
					}
				}

				//called by writeConstraintDesc
				PX_FORCE_INLINE PxU32 getStaticContactWriteIndex(const PxTGSSolverConstraintDesc& desc,
					bool activeA, bool activeB)
				{
					if (activeA)
					{
						if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexA)
						{
							return PxU32(desc.bodyA->maxDynamicPartition + desc.bodyA->nbStaticInteractions++);
						}
						else
						{
							ArticulationV* articulationA = desc.articulationA;
							articulationA->numTotalConstraints++;
							return PxU32(articulationA->maxSolverFrictionProgress + articulationA->maxSolverNormalProgress++);
						}

					}
					else if (activeB)
					{
						if (PxSolverConstraintDesc::NO_LINK == desc.linkIndexB)
						{
							return PxU32(desc.bodyB->maxDynamicPartition + desc.bodyB->nbStaticInteractions++);
						}
						else
						{
							ArticulationV* articulationB = desc.articulationB;
							articulationB->numTotalConstraints++;
							return PxU32(articulationB->maxSolverFrictionProgress + articulationB->maxSolverNormalProgress++);
						}
					}

					return 0xffffffff;
				}

				PX_FORCE_INLINE void clearState()
				{
					for (PxU32 a = 0; a < mNumBodies; ++a)
						mBodies[a].partitionMask = 0;

					for (PxU32 a = 0; a < mNumArticulations; ++a)
						(reinterpret_cast<ArticulationV*>(mArticulations[a]))->solverProgress = 0;
				}

				PX_FORCE_INLINE void reserveSpaceForStaticConstraints(Ps::Array<PxU32>& numConstraintsPerPartition)
				{
					for (PxU32 a = 0; a < mNumBodies; ++a)
					{
						mBodies[a].partitionMask = 0;

						PxU32 requiredSize = PxU32(mBodies[a].maxDynamicPartition + mBodies[a].nbStaticInteractions);
						if (requiredSize > numConstraintsPerPartition.size())
						{
							numConstraintsPerPartition.resize(requiredSize);
						}

						for (PxU32 b = 0; b < mBodies[a].nbStaticInteractions; ++b)
						{
							numConstraintsPerPartition[mBodies[a].maxDynamicPartition + b]++;
						}
					}

					for (PxU32 a = 0; a < mNumArticulations; ++a)
					{
						ArticulationV* articulation = reinterpret_cast<ArticulationV*>(mArticulations[a]);
						articulation->solverProgress = 0;

						PxU32 requiredSize = PxU32(articulation->maxSolverNormalProgress + articulation->maxSolverFrictionProgress);
						if (requiredSize > numConstraintsPerPartition.size())
						{
							numConstraintsPerPartition.resize(requiredSize);
						}

						for (PxU32 b = 0; b < articulation->maxSolverNormalProgress; ++b)
						{
							numConstraintsPerPartition[articulation->maxSolverFrictionProgress + b]++;
						}
					}
				}

			};

			template <typename Classification>
			void classifyConstraintDesc(const PxTGSSolverConstraintDesc* PX_RESTRICT descs, const PxU32 numConstraints, Classification& classification,
				Ps::Array<PxU32>& numConstraintsPerPartition, PxTGSSolverConstraintDesc* PX_RESTRICT eaTempConstraintDescriptors)
			{
				const PxTGSSolverConstraintDesc* _desc = descs;
				const PxU32 numConstraintsMin1 = numConstraints - 1;

				PxU32 numUnpartitionedConstraints = 0;
				PxU32 nbNormalConstraints = 0;
				PxU32 nbStaticConstraints = 0;

				numConstraintsPerPartition.forceSize_Unsafe(32);

				PxMemZero(numConstraintsPerPartition.begin(), sizeof(PxU32) * 32);

				for (PxU32 i = 0; i < numConstraints; ++i, _desc++)
				{
					const PxU32 prefetchOffset = PxMin(numConstraintsMin1 - i, 4u);
					Ps::prefetchLine(_desc[prefetchOffset].constraint);
					Ps::prefetchLine(_desc[prefetchOffset].bodyA);
					Ps::prefetchLine(_desc[prefetchOffset].bodyB);
					Ps::prefetchLine(_desc + 8);

					uintptr_t indexA, indexB;
					bool activeA, activeB;
					PxU32 partitionsA, partitionsB;
					const bool notContainsStatic = classification.classifyConstraint(*_desc, indexA, indexB, 
						activeA, activeB, partitionsA, partitionsB);

					if (notContainsStatic)
					{
						nbNormalConstraints++;
					/*	PxU32 partitionsA = _desc->bodyA->partitionMask;
						PxU32 partitionsB = _desc->bodyB->partitionMask;*/

						PxU32 availablePartition;
						{
							const PxU32 combinedMask = ((~partitionsA) & (~partitionsB));
							availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : Ps::lowestSetBit(combinedMask);
							if (availablePartition == MAX_NUM_PARTITIONS)
							{
								eaTempConstraintDescriptors[numUnpartitionedConstraints++] = *_desc;
								continue;
							}

							const PxU32 partitionBit = getBit(availablePartition);
							PX_ASSERT((partitionBit & partitionsA) == 0);
							PX_ASSERT((partitionBit & partitionsB) == 0);
							if(activeA)
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
						nbStaticConstraints++;

						//Just count the number of static constraints and store in maxSolverFrictionProgress...
						PX_ASSERT(activeA || activeB);
						classification.recordStaticConstraint(*_desc, activeA, activeB);
					}
				}

				PxU32 partitionStartIndex = 0;

				while (numUnpartitionedConstraints > 0)
				{
					classification.clearState();

					partitionStartIndex += 32;
					//Keep partitioning the un-partitioned constraints and blat the whole thing to 0!
					numConstraintsPerPartition.resize(32 + numConstraintsPerPartition.size());
					PxMemZero(numConstraintsPerPartition.begin() + partitionStartIndex, sizeof(PxU32) * 32);

					PxU32 newNumUnpartitionedConstraints = 0;

					uintptr_t indexA, indexB;
					bool activeA, activeB;
					PxU32 partitionsA, partitionsB;

					for (PxU32 i = 0; i < numUnpartitionedConstraints; ++i)
					{
						const PxTGSSolverConstraintDesc& desc = eaTempConstraintDescriptors[i];

						classification.classifyConstraint(desc, indexA, indexB, 
							activeA, activeB, partitionsA, partitionsB);

						/*PxU32 partitionsA = desc.bodyA->partitionMask;
						PxU32 partitionsB = desc.bodyB->partitionMask;*/

						PxU32 availablePartition;
						{
							const PxU32 combinedMask = (~partitionsA & ~partitionsB);
							availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : Ps::lowestSetBit(combinedMask);
							if (availablePartition == MAX_NUM_PARTITIONS)
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

						availablePartition += partitionStartIndex;
						numConstraintsPerPartition[availablePartition]++;
						availablePartition++;
						classification.storeProgress(desc, partitionsA, partitionsB, PxU16(availablePartition));
						
					}

					numUnpartitionedConstraints = newNumUnpartitionedConstraints;
				}

				classification.reserveSpaceForStaticConstraints(numConstraintsPerPartition);

			}

			template <typename Classification>
			void writeConstraintDesc(const PxTGSSolverConstraintDesc* PX_RESTRICT descs, const PxU32 numConstraints, Classification& classification,
				Ps::Array<PxU32>& accumulatedConstraintsPerPartition, PxTGSSolverConstraintDesc* eaTempConstraintDescriptors,
				PxTGSSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDesc)
			{
				PX_UNUSED(eaTempConstraintDescriptors);
				const PxTGSSolverConstraintDesc* _desc = descs;
				const PxU32 numConstraintsMin1 = numConstraints - 1;

				PxU32 numUnpartitionedConstraints = 0;

				for (PxU32 i = 0; i < numConstraints; ++i, _desc++)
				{
					const PxU32 prefetchOffset = PxMin(numConstraintsMin1 - i, 4u);
					Ps::prefetchLine(_desc[prefetchOffset].constraint);
					Ps::prefetchLine(_desc[prefetchOffset].bodyA);
					Ps::prefetchLine(_desc[prefetchOffset].bodyB);
					Ps::prefetchLine(_desc + 8);

					uintptr_t indexA, indexB;
					bool activeA, activeB;
					PxU32 partitionsA, partitionsB;
					const bool notContainsStatic = classification.classifyConstraint(*_desc, indexA, indexB, 
						activeA, activeB, partitionsA, partitionsB);

					if (notContainsStatic)
					{
						/*PxU32 partitionsA = _desc->bodyA->partitionMask;
						PxU32 partitionsB = _desc->bodyB->partitionMask;*/

						PxU32 availablePartition;
						{
							const PxU32 combinedMask = (~partitionsA & ~partitionsB);
							availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : Ps::lowestSetBit(combinedMask);
							if (availablePartition == MAX_NUM_PARTITIONS)
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

						/*_desc->bodyA->partitionMask = partitionsA;
						_desc->bodyB->partitionMask = partitionsB;
*/
						classification.storeProgress(*_desc, activeA, activeB, partitionsA,  partitionsB);
						

						eaOrderedConstraintDesc[accumulatedConstraintsPerPartition[availablePartition]++] = *_desc;
					}
					else
					{
						//Just count the number of static constraints and store in maxSolverFrictionProgress...
						/*PxU32 index = 0;
						if (activeA)
							index = PxU32(_desc->bodyA->maxDynamicPartition + _desc->bodyA->nbStaticInteractions++);
						else if (activeB)
							index = PxU32(_desc->bodyB->maxDynamicPartition + _desc->bodyB->nbStaticInteractions++);*/

						PxU32 index = classification.getStaticContactWriteIndex(*_desc, activeA, activeB);

						eaOrderedConstraintDesc[accumulatedConstraintsPerPartition[index]++] = *_desc;
					}
				}

				PxU32 partitionStartIndex = 0;

				while (numUnpartitionedConstraints > 0)
				{
					classification.clearState();

					partitionStartIndex += 32;
					PxU32 newNumUnpartitionedConstraints = 0;

					uintptr_t indexA, indexB;
					bool activeA, activeB;
					PxU32 partitionsA, partitionsB;

					for (PxU32 i = 0; i < numUnpartitionedConstraints; ++i)
					{
						const PxTGSSolverConstraintDesc& desc = eaTempConstraintDescriptors[i];

						classification.classifyConstraint(desc, indexA, indexB, 
							activeA, activeB, partitionsA, partitionsB);

					/*	PxU32 partitionsA = desc.bodyA->partitionMask;
						PxU32 partitionsB = desc.bodyB->partitionMask;*/

						PxU32 availablePartition;
						{
							const PxU32 combinedMask = (~partitionsA & ~partitionsB);
							availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : Ps::lowestSetBit(combinedMask);
							if (availablePartition == MAX_NUM_PARTITIONS)
							{
								//Need to shuffle around unpartitioned constraints...
								eaTempConstraintDescriptors[newNumUnpartitionedConstraints++] = desc;
								continue;
							}

							const PxU32 partitionBit = getBit(availablePartition);

							partitionsA |= partitionBit;
							partitionsB |= partitionBit;
						}

						/*if(activeA)
							desc.bodyA->partitionMask = partitionsA;
						if(activeB)
							desc.bodyB->partitionMask = partitionsB;*/
						classification.storeProgress(desc, activeA, activeB, partitionsA, partitionsB);
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
		PxU32 normalizePartitions(Ps::Array<PxU32>& accumulatedConstraintsPerPartition, PxTGSSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDescriptors,
			const PxU32 numConstraintDescriptors, Ps::Array<PxU32>& bitField, const Classification& classification, const PxU32 numBodies, const PxU32 numArticulations)
		{
			PxU32 numPartitions = 0;

			PxU32 prevAccumulation = 0;
			for (; numPartitions < accumulatedConstraintsPerPartition.size() && accumulatedConstraintsPerPartition[numPartitions] > prevAccumulation;
				prevAccumulation = accumulatedConstraintsPerPartition[numPartitions++]);

			PxU32 targetSize = (numPartitions == 0 ? 0 : (numConstraintDescriptors) / numPartitions);

			bitField.reserve((numBodies + numArticulations + 31) / 32);
			bitField.forceSize_Unsafe((numBodies + numArticulations + 31) / 32);

			for (PxU32 i = numPartitions; i > 0; i--)
			{
				PxU32 partitionIndex = i - 1;

				//Build the partition mask...

				PxU32 startIndex = partitionIndex == 0 ? 0 : accumulatedConstraintsPerPartition[partitionIndex - 1];
				PxU32 endIndex = accumulatedConstraintsPerPartition[partitionIndex];

				//If its greater than target size, there's nothing that will be pulled into it from earlier partitions
				if ((endIndex - startIndex) >= targetSize)
					continue;


				PxMemZero(bitField.begin(), sizeof(PxU32)*bitField.size());

				for (PxU32 a = startIndex; a < endIndex; ++a)
				{
					PxTGSSolverConstraintDesc& desc = eaOrderedConstraintDescriptors[a];

					uintptr_t indexA, indexB;
					bool activeA, activeB;
					PxU32 partitionsA, partitionsB;
					classification.classifyConstraint(desc, indexA, indexB, 
						activeA, activeB, partitionsA, partitionsB);

					if (activeA)
						bitField[PxU32(indexA) / 32] |= getBit(indexA & 31);
					if (activeB)
						bitField[PxU32(indexB) / 32] |= getBit(indexB & 31);
				}

				bool bTerm = false;
				for (PxU32 a = partitionIndex; a > 0 && !bTerm; --a)
				{
					PxU32 pInd = a - 1;

					PxU32 si = pInd == 0 ? 0 : accumulatedConstraintsPerPartition[pInd - 1];
					PxU32 ei = accumulatedConstraintsPerPartition[pInd];

					for (PxU32 b = ei; b > si && !bTerm; --b)
					{
						PxU32 ind = b - 1;
						PxTGSSolverConstraintDesc& desc = eaOrderedConstraintDescriptors[ind];

						uintptr_t indexA, indexB;
						bool activeA, activeB;
						PxU32 partitionsA, partitionsB;
						classification.classifyConstraint(desc, indexA, indexB, 
							activeA, activeB, partitionsA, partitionsB);

						bool canAdd = true;

						if (activeA && (bitField[PxU32(indexA) / 32] & (getBit(indexA & 31))))
							canAdd = false;
						if (activeB && (bitField[PxU32(indexB) / 32] & (getBit(indexB & 31))))
							canAdd = false;

						if (canAdd)
						{
							PxTGSSolverConstraintDesc tmp = eaOrderedConstraintDescriptors[ind];

							if (activeA)
								bitField[PxU32(indexA) / 32] |= (getBit(indexA & 31));
							if (activeB)
								bitField[PxU32(indexB) / 32] |= (getBit(indexB & 31));

							PxU32 index = ind;
							for (PxU32 c = pInd; c < partitionIndex; ++c)
							{
								PxU32 newIndex = --accumulatedConstraintsPerPartition[c];
								if (index != newIndex)
									eaOrderedConstraintDescriptors[index] = eaOrderedConstraintDescriptors[newIndex];
								index = newIndex;
							}

							if (index != ind)
								eaOrderedConstraintDescriptors[index] = tmp;

							if ((accumulatedConstraintsPerPartition[partitionIndex] - accumulatedConstraintsPerPartition[partitionIndex - 1]) >= targetSize)
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

		PxU32 partitionContactConstraintsStep(TGSConstraintPartitionArgs& args)
		{
			PxU32 maxPartition = 0;
			//Unpack the input data.
			const PxU32 numBodies = args.mNumBodies;
			PxTGSSolverBodyVel* PX_RESTRICT eaAtoms = args.mBodies;
			const PxU32	numArticulations = args.mNumArticulationPtrs;

			const PxU32 numConstraintDescriptors = args.mNumContactConstraintDescriptors;

			PxTGSSolverConstraintDesc* PX_RESTRICT eaConstraintDescriptors = args.mContactConstraintDescriptors;
			PxTGSSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDescriptors = args.mOrderedContactConstraintDescriptors;
			PxTGSSolverConstraintDesc* PX_RESTRICT eaTempConstraintDescriptors = args.mTempContactConstraintDescriptors;

			Ps::Array<PxU32>& constraintsPerPartition = *args.mConstraintsPerPartition;
			constraintsPerPartition.forceSize_Unsafe(0);

			for (PxU32 a = 0; a < numBodies; ++a)
			{
				PxTGSSolverBodyVel& body = args.mBodies[a];
				Ps::prefetchLine(&args.mBodies[a], 256);
				body.partitionMask = 0;
				//We re-use maxSolverFrictionProgress and maxSolverNormalProgress to record the
				//maximum partition used by dynamic constraints and the number of static constraints affecting
				//a body. We use this to make partitioning much cheaper and be able to support 
				body.maxDynamicPartition = 0;
				body.nbStaticInteractions = 0;
			}

			PxU32 numOrderedConstraints = 0;

			PxU32 numSelfConstraintBlocks = 0;

			if (numArticulations == 0)
			{
				RigidBodyClassification classification(eaAtoms, numBodies);
				classifyConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition,
					eaTempConstraintDescriptors);

				PxU32 accumulation = 0;
				for (PxU32 a = 0; a < constraintsPerPartition.size(); ++a)
				{
					PxU32 count = constraintsPerPartition[a];
					constraintsPerPartition[a] = accumulation;
					accumulation += count;
				}

				for (PxU32 a = 0; a < numBodies; ++a)
				{
					PxTGSSolverBodyVel& body = args.mBodies[a];
					Ps::prefetchLine(&args.mBodies[a], 256);
					body.partitionMask = 0;
					//Keep the dynamic constraint count but bump the static constraint count back to 0.
					//This allows us to place the static constraints in the appropriate place when we see them
					//because we know the maximum index for the dynamic constraints...
					body.nbStaticInteractions = 0;
				}

				writeConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition,
					eaTempConstraintDescriptors, eaOrderedConstraintDescriptors);

				numOrderedConstraints = numConstraintDescriptors;

				if (!args.enhancedDeterminism)
					maxPartition = normalizePartitions(constraintsPerPartition, eaOrderedConstraintDescriptors, numConstraintDescriptors, *args.mBitField,
						classification, numBodies, 0);

			}
			else
			{

				ArticulationSolverDesc* articulationDescs = args.mArticulationPtrs;
				PX_ALLOCA(_eaArticulations, uintptr_t, numArticulations);
				uintptr_t* eaArticulations = _eaArticulations;
				for (PxU32 i = 0; i<numArticulations; i++)
				{
					ArticulationV* articulation = articulationDescs[i].articulation;
					eaArticulations[i] = uintptr_t(articulation);
					articulation->solverProgress = 0;
					articulation->maxSolverFrictionProgress = 0;
					articulation->maxSolverNormalProgress = 0;
					articulation->numTotalConstraints = 0;
				}
				ExtendedRigidBodyClassification classification(eaAtoms, numBodies, eaArticulations, numArticulations);

				classifyConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification,
					constraintsPerPartition, eaTempConstraintDescriptors);

				PxU32 accumulation = 0;
				for (PxU32 a = 0; a < constraintsPerPartition.size(); ++a)
				{
					PxU32 count = constraintsPerPartition[a];
					constraintsPerPartition[a] = accumulation;
					accumulation += count;
				}

				for (PxU32 a = 0; a < numBodies; ++a)
				{
					PxTGSSolverBodyVel& body = args.mBodies[a];
					Ps::prefetchLine(&args.mBodies[a], 256);
					body.partitionMask = 0;
					//Keep the dynamic constraint count but bump the static constraint count back to 0.
					//This allows us to place the static constraints in the appropriate place when we see them
					//because we know the maximum index for the dynamic constraints...
					body.nbStaticInteractions = 0;
				}

				for (PxU32 a = 0; a < numArticulations; ++a)
				{
					ArticulationV* articulation = reinterpret_cast<ArticulationV*>(eaArticulations[a]);
					articulation->solverProgress = 0;
					articulation->maxSolverNormalProgress = 0;
					articulation->numTotalConstraints = 0;
				}

				writeConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition,
					eaTempConstraintDescriptors, eaOrderedConstraintDescriptors);

				numOrderedConstraints = numConstraintDescriptors;

				if (!args.enhancedDeterminism)
					maxPartition = normalizePartitions(constraintsPerPartition, eaOrderedConstraintDescriptors,
						numConstraintDescriptors, *args.mBitField, classification, numBodies, numArticulations);

			}



			const PxU32 numConstraintsDifferentBodies = numOrderedConstraints;

			PX_ASSERT(numConstraintsDifferentBodies == numConstraintDescriptors);

			//Now handle the articulated self-constraints.
			PxU32 totalConstraintCount = numConstraintsDifferentBodies;

			args.mNumSelfConstraintBlocks = numSelfConstraintBlocks;

			args.mNumDifferentBodyConstraints = numConstraintsDifferentBodies;
			args.mNumSelfConstraints = totalConstraintCount - numConstraintsDifferentBodies;

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
