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

#include "PxImmediateMode.h"
#include "CmUtils.h"
#include "PsMathUtils.h"
#include "../../lowleveldynamics/src/DyBodyCoreIntegrator.h"
#include "../../lowleveldynamics/src/DySolverBody.h"
#include "../../lowleveldynamics/src/DyContactPrep.h"
#include "../../lowleveldynamics/src/DyCorrelationBuffer.h"
#include "../../lowleveldynamics/src/DyConstraintPrep.h"
#include "../../lowleveldynamics/src/DySolverControl.h"
#include "../../lowleveldynamics/src/DySolverContext.h"
#include "PxGeometry.h"
#include "GuGeometryUnion.h"
#include "GuContactMethodImpl.h"
#include "../../lowlevel/common/include/collision/PxcContactMethodImpl.h"
#include "GuPersistentContactManifold.h"
#include "NpConstraint.h"

namespace physx
{

	namespace
	{

#define MAX_NUM_PARTITIONS 32u

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
			PxSolverBody* PX_RESTRICT mBodies;
			PxU32 mNumBodies;

		public:
			RigidBodyClassification(PxSolverBody* PX_RESTRICT bodies, PxU32 numBodies) : mBodies(bodies), mNumBodies(numBodies)
			{
			}

			//Returns true if it is a dynamic-dynamic constriant; false if it is a dynamic-static or dynamic-kinematic constraint
			PX_FORCE_INLINE bool classifyConstraint(const PxSolverConstraintDesc& desc, uintptr_t& indexA, uintptr_t& indexB, bool& activeA, bool& activeB) const
			{
				indexA = uintptr_t(desc.bodyA - mBodies);
				indexB = uintptr_t(desc.bodyB - mBodies);
				activeA = indexA < mNumBodies;
				activeB = indexB < mNumBodies;
				return activeA && activeB;
			}

			PX_FORCE_INLINE void reserveSpaceForStaticConstraints(PxU32* numConstraintsPerPartition)
			{
				for (PxU32 a = 0; a < mNumBodies; ++a)
				{
					mBodies[a].solverProgress = 0;

					for (PxU32 b = 0; b < mBodies[a].maxSolverFrictionProgress; ++b)
					{
						PxU32 partId = PxMin(PxU32(mBodies[a].maxSolverNormalProgress + b), MAX_NUM_PARTITIONS);
						numConstraintsPerPartition[partId]++;
					}
				}
			}
		};

		template <typename Classification>
		void classifyConstraintDesc(const PxSolverConstraintDesc* PX_RESTRICT descs, const PxU32 numConstraints, Classification& classification,
			PxU32* numConstraintsPerPartition)
		{
			const PxSolverConstraintDesc* _desc = descs;
			const PxU32 numConstraintsMin1 = numConstraints - 1;

			PxMemZero(numConstraintsPerPartition, sizeof(PxU32) * 33);

			for (PxU32 i = 0; i < numConstraints; ++i, _desc++)
			{
				const PxU32 prefetchOffset = PxMin(numConstraintsMin1 - i, 4u);
				Ps::prefetchLine(_desc[prefetchOffset].constraint);
				Ps::prefetchLine(_desc[prefetchOffset].bodyA);
				Ps::prefetchLine(_desc[prefetchOffset].bodyB);
				Ps::prefetchLine(_desc + 8);

				uintptr_t indexA, indexB;
				bool activeA, activeB;

				const bool notContainsStatic = classification.classifyConstraint(*_desc, indexA, indexB, activeA, activeB);

				if (notContainsStatic)
				{
					PxU32 partitionsA = _desc->bodyA->solverProgress;
					PxU32 partitionsB = _desc->bodyB->solverProgress;

					PxU32 availablePartition;
					{
						const PxU32 combinedMask = (~partitionsA & ~partitionsB);
						availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : Ps::lowestSetBit(combinedMask);
						if (availablePartition == MAX_NUM_PARTITIONS)
						{
							//Write to overflow partition...
							numConstraintsPerPartition[availablePartition]++;
							_desc->bodyA->maxSolverNormalProgress = MAX_NUM_PARTITIONS;
							_desc->bodyB->maxSolverNormalProgress = MAX_NUM_PARTITIONS;
							continue;
						}

						const PxU32 partitionBit = getBit(availablePartition);
						partitionsA |= partitionBit;
						partitionsB |= partitionBit;
					}

					_desc->bodyA->solverProgress = partitionsA;
					_desc->bodyB->solverProgress = partitionsB;
					numConstraintsPerPartition[availablePartition]++;
					availablePartition++;
					_desc->bodyA->maxSolverNormalProgress = PxMax(_desc->bodyA->maxSolverNormalProgress, PxU16(availablePartition));
					_desc->bodyB->maxSolverNormalProgress = PxMax(_desc->bodyB->maxSolverNormalProgress, PxU16(availablePartition));
				}
				else
				{
					//Just count the number of static constraints and store in maxSolverFrictionProgress...
					if (activeA)
						_desc->bodyA->maxSolverFrictionProgress++;
					else if (activeB)
						_desc->bodyB->maxSolverFrictionProgress++;
				}
			}

			classification.reserveSpaceForStaticConstraints(numConstraintsPerPartition);

		}

		template <typename Classification>
		void writeConstraintDesc(const PxSolverConstraintDesc* PX_RESTRICT descs, const PxU32 numConstraints, Classification& classification,
			PxU32* accumulatedConstraintsPerPartition, PxSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDesc)
		{
			const PxSolverConstraintDesc* _desc = descs;
			const PxU32 numConstraintsMin1 = numConstraints - 1;
			for (PxU32 i = 0; i < numConstraints; ++i, _desc++)
			{
				const PxU32 prefetchOffset = PxMin(numConstraintsMin1 - i, 4u);
				Ps::prefetchLine(_desc[prefetchOffset].constraint);
				Ps::prefetchLine(_desc[prefetchOffset].bodyA);
				Ps::prefetchLine(_desc[prefetchOffset].bodyB);
				Ps::prefetchLine(_desc + 8);

				uintptr_t indexA, indexB;
				bool activeA, activeB;
				const bool notContainsStatic = classification.classifyConstraint(*_desc, indexA, indexB, activeA, activeB);

				if (notContainsStatic)
				{
					PxU32 partitionsA = _desc->bodyA->solverProgress;
					PxU32 partitionsB = _desc->bodyB->solverProgress;

					PxU32 availablePartition;
					{
						const PxU32 combinedMask = (~partitionsA & ~partitionsB);
						availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : Ps::lowestSetBit(combinedMask);
						if (availablePartition == MAX_NUM_PARTITIONS)
						{
							eaOrderedConstraintDesc[accumulatedConstraintsPerPartition[availablePartition]++] = *_desc;
							continue;
						}

						const PxU32 partitionBit = getBit(availablePartition);

						partitionsA |= partitionBit;
						partitionsB |= partitionBit;
					}

					_desc->bodyA->solverProgress = partitionsA;
					_desc->bodyB->solverProgress = partitionsB;

					eaOrderedConstraintDesc[accumulatedConstraintsPerPartition[availablePartition]++] = *_desc;
				}
				else
				{
					//Just count the number of static constraints and store in maxSolverFrictionProgress...
					PxU32 index = 0;
					if (activeA)
						index = PxMin(PxU32(_desc->bodyA->maxSolverNormalProgress + _desc->bodyA->maxSolverFrictionProgress++), MAX_NUM_PARTITIONS);
					else if (activeB)
						index = PxMin(PxU32(_desc->bodyB->maxSolverNormalProgress + _desc->bodyB->maxSolverFrictionProgress++), MAX_NUM_PARTITIONS);

					eaOrderedConstraintDesc[accumulatedConstraintsPerPartition[index]++] = *_desc;
				}
			}
		}

	}


	void immediate::PxConstructSolverBodies(const PxRigidBodyData* inRigidData, PxSolverBodyData* outSolverBodyData, const PxU32 nbBodies, const PxVec3& gravity, const PxReal dt)
	{
		for (PxU32 a = 0; a < nbBodies; ++a)
		{
			const PxRigidBodyData& rigidData = inRigidData[a];
			PxVec3 lv = rigidData.linearVelocity, av = rigidData.angularVelocity;
			Dy::bodyCoreComputeUnconstrainedVelocity(gravity, dt, rigidData.linearDamping, rigidData.angularDamping, 1.f, rigidData.maxLinearVelocitySq, rigidData.maxAngularVelocitySq, lv, av, false);
			Dy::copyToSolverBodyData(lv, av, rigidData.invMass, rigidData.invInertia, rigidData.body2World, -rigidData.maxDepenetrationVelocity, rigidData.maxContactImpulse, IG_INVALID_NODE, PX_MAX_F32, outSolverBodyData[a], 0);
		}
	}

	void immediate::PxConstructStaticSolverBody(const PxTransform& globalPose, PxSolverBodyData& solverBodyData)
	{
		PxVec3 zero(0.f);
		Dy::copyToSolverBodyData(zero, zero, 0.f, zero, globalPose, -PX_MAX_F32, PX_MAX_F32, IG_INVALID_NODE, PX_MAX_F32, solverBodyData, 0);
	}


	void immediate::PxIntegrateSolverBodies(PxSolverBodyData* solverBodyData, PxSolverBody* solverBody, const PxVec3* linearMotionVelocity, const PxVec3* angularMotionState, const PxU32 nbBodiesToIntegrate, PxReal dt)
	{
		for (PxU32 i = 0; i < nbBodiesToIntegrate; ++i)
		{
			PxVec3 lmv = linearMotionVelocity[i];
			PxVec3 amv = angularMotionState[i];
			Dy::integrateCore(lmv, amv, solverBody[i], solverBodyData[i], dt);
		}
	}


	PxU32 immediate::PxBatchConstraints(PxSolverConstraintDesc* solverConstraintDescs, const PxU32 nbConstraints, PxSolverBody* solverBodies, PxU32 numBodies, PxConstraintBatchHeader* outBatchHeaders,
		PxSolverConstraintDesc* outOrderedConstraintDescs)
	{
		PxU32 constraintsPerPartition[MAX_NUM_PARTITIONS + 1];

		for (PxU32 a = 0; a < numBodies; ++a)
		{
			PxSolverBody& body = solverBodies[a];
			body.solverProgress = 0;
			//We re-use maxSolverFrictionProgress and maxSolverNormalProgress to record the
			//maximum partition used by dynamic constraints and the number of static constraints affecting
			//a body. We use this to make partitioning much cheaper and be able to support 
			body.maxSolverFrictionProgress = 0;
			body.maxSolverNormalProgress = 0;
		}

		{
			RigidBodyClassification classification(solverBodies, numBodies);
			classifyConstraintDesc(solverConstraintDescs, nbConstraints, classification, constraintsPerPartition);

			PxU32 accumulation = 0;
			for (PxU32 a = 0; a < MAX_NUM_PARTITIONS+1; ++a)
			{
				PxU32 count = constraintsPerPartition[a];
				constraintsPerPartition[a] = accumulation;
				accumulation += count;
			}

			for (PxU32 a = 0; a < numBodies; ++a)
			{
				PxSolverBody& body = solverBodies[a];
				body.solverProgress = 0;
				//Keep the dynamic constraint count but bump the static constraint count back to 0.
				//This allows us to place the static constraints in the appropriate place when we see them
				//because we know the maximum index for the dynamic constraints...
				body.maxSolverFrictionProgress = 0;
			}

			writeConstraintDesc(solverConstraintDescs, nbConstraints, classification, constraintsPerPartition, outOrderedConstraintDescs);

			PxU32 numHeaders = 0;
			PxU32 currentPartition = 0;
			PxU32 maxJ = nbConstraints == 0 ? 0 : constraintsPerPartition[0];

			const PxU32 maxBatchPartition = MAX_NUM_PARTITIONS;
			for (PxU32 a = 0; a < nbConstraints;)
			{
				PxConstraintBatchHeader& header = outBatchHeaders[numHeaders++];
				header.mStartIndex = a;

				PxU32 loopMax = PxMin(maxJ - a, 4u);
				PxU16 j = 0;
				if (loopMax > 0)
				{
					j = 1;
					PxSolverConstraintDesc& desc = outOrderedConstraintDescs[a];
					if (currentPartition < maxBatchPartition)
					{
						for (; j < loopMax && desc.constraintLengthOver16 == outOrderedConstraintDescs[a + j].constraintLengthOver16; ++j);
					}
					header.mStride = j;
					header.mConstraintType = desc.constraintLengthOver16;
				}
				if (maxJ == (a + j) && maxJ != nbConstraints)
				{
					currentPartition++;
					maxJ = constraintsPerPartition[currentPartition];
				}
				a += j;
			}
			return numHeaders;
		}
	}


	bool immediate::PxCreateContactConstraints(PxConstraintBatchHeader* batchHeaders, const PxU32 nbHeaders, PxSolverContactDesc* contactDescs,
		PxConstraintAllocator& allocator, PxReal invDt, PxReal bounceThreshold, PxReal frictionOffsetThreshold, PxReal correlationDistance)
	{
		PX_ASSERT(invDt > 0.f && PxIsFinite(invDt));
		PX_ASSERT(bounceThreshold < 0.f);
		PX_ASSERT(frictionOffsetThreshold > 0.f);
		PX_ASSERT(correlationDistance > 0.f);

		Dy::SolverConstraintPrepState::Enum state = Dy::SolverConstraintPrepState::eUNBATCHABLE;

		Dy::CorrelationBuffer cb;

		PxU32 currentContactDescIdx = 0;

		for (PxU32 i = 0; i < nbHeaders; ++i)
		{
			PxConstraintBatchHeader& batchHeader = batchHeaders[i];
			if (batchHeader.mStride == 4)
			{
				PxU32 totalContacts = contactDescs[currentContactDescIdx].numContacts + contactDescs[currentContactDescIdx + 1].numContacts + 
					contactDescs[currentContactDescIdx + 2].numContacts + contactDescs[currentContactDescIdx + 3].numContacts;

				if (totalContacts <= 64)
				{
					state = Dy::createFinalizeSolverContacts4(cb,
						contactDescs + currentContactDescIdx,
						invDt,
						bounceThreshold,
						frictionOffsetThreshold,
						correlationDistance,
						0.f,
						allocator);
				}
			}

			if (state == Dy::SolverConstraintPrepState::eUNBATCHABLE)
			{
				for (PxU32 a = 0; a < batchHeader.mStride; ++a)
				{
					Dy::createFinalizeSolverContacts(contactDescs[currentContactDescIdx + a], cb, invDt, bounceThreshold, 
						frictionOffsetThreshold, correlationDistance, 0.f, allocator, NULL);
				}
			}
			PxU8 type = *contactDescs[currentContactDescIdx].desc->constraint;

			if (type == DY_SC_TYPE_STATIC_CONTACT)
			{
				//Check if any block of constraints is classified as type static (single) contact constraint.
				//If they are, iterate over all constraints grouped with it and switch to "dynamic" contact constraint
				//type if there's a dynamic contact constraint in the group.
				for (PxU32 c = 1; c < batchHeader.mStride; ++c)
				{
					if (*contactDescs[currentContactDescIdx + c].desc->constraint == DY_SC_TYPE_RB_CONTACT)
					{
						type = DY_SC_TYPE_RB_CONTACT;
						break;
					}
				}
			}

			batchHeader.mConstraintType = type;

			currentContactDescIdx += batchHeader.mStride;
		}

		

		

		return true;
	}


	bool immediate::PxCreateJointConstraints(PxConstraintBatchHeader* batchHeaders, const PxU32 nbHeaders, PxSolverConstraintPrepDesc* jointDescs, PxConstraintAllocator& allocator, PxReal dt, PxReal invDt)
	{
		PX_ASSERT(dt > 0.f);
		PX_ASSERT(invDt > 0.f && PxIsFinite(invDt));

		Dy::SolverConstraintPrepState::Enum state = Dy::SolverConstraintPrepState::eUNBATCHABLE; 
		
		PxU32 currentDescIdx = 0;
		for (PxU32 i = 0; i < nbHeaders; ++i)
		{
			PxConstraintBatchHeader& batchHeader = batchHeaders[i];

			PxU8 type = DY_SC_TYPE_BLOCK_1D;
			if (batchHeader.mStride == 4)
			{
				PxU32 totalRows = 0;
				PxU32 maxRows = 0;
				bool batchable = true;
				for (PxU32 a = 0; a < batchHeader.mStride; ++a)
				{
					if (jointDescs[currentDescIdx + a].numRows == 0)
					{
						batchable = false;
						break;
					}
					totalRows += jointDescs[currentDescIdx + a].numRows;
					maxRows = PxMax(maxRows, jointDescs[currentDescIdx + a].numRows);
				}

				if (batchable)
				{
					state = Dy::setupSolverConstraint4
						(jointDescs + currentDescIdx,
						dt, invDt, totalRows,
						allocator, maxRows);
				}
			}

			if (state == Dy::SolverConstraintPrepState::eUNBATCHABLE)
			{
				type = DY_SC_TYPE_RB_1D;
				for (PxU32 a = 0; a < batchHeader.mStride; ++a)
				{
					Dy::ConstraintHelper::setupSolverConstraint(jointDescs[currentDescIdx + a], 
						allocator, dt, invDt, NULL);
				}
			}

			batchHeader.mConstraintType = type;
			currentDescIdx += batchHeader.mStride;
		}

		return true;
	}

	bool immediate::PxCreateJointConstraintsWithShaders(PxConstraintBatchHeader* batchHeaders, const PxU32 nbHeaders, PxConstraint** constraints, PxSolverConstraintPrepDesc* jointDescs, PxConstraintAllocator& allocator,
		PxReal dt, PxReal invDt)
	{
		Px1DConstraint allRows[Dy::MAX_CONSTRAINT_ROWS * 4];

		//Runs shaders to fill in rows...

		PxU32 currentDescIdx = 0;

		for (PxU32 i = 0; i < nbHeaders; ++i)
		{

			PxU32 numRows = 0;
			PxU32 preppedIndex = 0;
			PxU32 maxRows = 0;

			PxConstraintBatchHeader& batchHeader = batchHeaders[i];

			for (PxU32 a = 0; a < batchHeader.mStride; ++a)
			{
				Px1DConstraint* rows = allRows + numRows;
				PxSolverConstraintPrepDesc& desc = jointDescs[currentDescIdx + a];

				NpConstraint* npConstraint = static_cast<NpConstraint*>(constraints[currentDescIdx + a]);

				npConstraint->updateConstants();

				Sc::ConstraintCore& core = npConstraint->getScbConstraint().getScConstraint();

				PxConstraintSolverPrep prep = core.getPxConnector()->getPrep();
				const void* constantBlock = core.getPxConnector()->getConstantBlock();
				PX_ASSERT(prep);

				PxMemZero(rows + preppedIndex, sizeof(Px1DConstraint)*(Dy::MAX_CONSTRAINT_ROWS));
				for (PxU32 b = preppedIndex; b < Dy::MAX_CONSTRAINT_ROWS; ++b)
				{
					Px1DConstraint& c = rows[b];
					//Px1DConstraintInit(c);
					c.minImpulse = -PX_MAX_REAL;
					c.maxImpulse = PX_MAX_REAL;
				}

				desc.mInvMassScales.linear0 = desc.mInvMassScales.linear1 = desc.mInvMassScales.angular0 = desc.mInvMassScales.angular1 = 1.f;

				desc.body0WorldOffset = PxVec3(0.f);

				PxVec3 cA2w, cB2w;
				PxU32 constraintCount = prep(rows,
					desc.body0WorldOffset,
					Dy::MAX_CONSTRAINT_ROWS,
					desc.mInvMassScales,
					constantBlock,
					desc.bodyFrame0, desc.bodyFrame1, 
					!!(core.getFlags() & PxConstraintFlag::eENABLE_EXTENDED_LIMITS),
					cA2w, cB2w);

				preppedIndex = Dy::MAX_CONSTRAINT_ROWS - constraintCount;

				maxRows = PxMax(constraintCount, maxRows);

				desc.rows = rows;
				desc.numRows = constraintCount;
				numRows += constraintCount;
			}

			PxCreateJointConstraints(&batchHeader, 1, jointDescs + currentDescIdx, allocator, dt, invDt);

			currentDescIdx += batchHeader.mStride;
		}

		return true; //KS - TODO - do some error reporting/management...
	}

	PX_FORCE_INLINE bool PxIsZero(PxSolverBody* bodies, PxU32 nbBodies)
	{
		for (PxU32 i = 0; i < nbBodies; ++i)
		{
			if (!bodies[i].linearVelocity.isZero() ||
				!bodies[i].angularState.isZero())
				return false;
		}
		return true;
	}


	void immediate::PxSolveConstraints(PxConstraintBatchHeader* batchHeaders, const PxU32 nbBatchHeaders, PxSolverConstraintDesc* solverConstraintDescs, PxSolverBody* solverBodies,
		PxVec3* linearMotionVelocity, PxVec3* angularMotionVelocity, const PxU32 nbSolverBodies, const PxU32 nbPositionIterations, const PxU32 nbVelocityIterations)
	{
		PX_ASSERT(nbPositionIterations > 0);
		PX_ASSERT(nbVelocityIterations > 0);
		PX_ASSERT(PxIsZero(solverBodies, nbSolverBodies)); //Ensure that solver body velocities have been zeroed before solving

		//Stage 1: solve the position iterations...
		Dy::SolveBlockMethod*  solveTable = Dy::getSolveBlockTable();

		Dy::SolveBlockMethod* solveConcludeTable = Dy::getSolverConcludeBlockTable();

		Dy::SolveWriteBackBlockMethod* solveWritebackTable = Dy::getSolveWritebackBlockTable();

		Dy::SolverContext cache;
		cache.mThresholdStreamIndex = 0;
		cache.mThresholdStreamLength = 0xFFFFFFF;
		
		PX_ASSERT(nbPositionIterations > 0);
		PX_ASSERT(nbVelocityIterations > 0);

		for (PxU32 i = nbPositionIterations; i > 1; --i)
		{
			cache.doFriction = i <= 3;
			for (PxU32 a = 0; a < nbBatchHeaders; ++a)
			{
				PxConstraintBatchHeader& batch = batchHeaders[a];
				solveTable[batch.mConstraintType](solverConstraintDescs + batch.mStartIndex, batch.mStride, cache);
			}
		}

		cache.doFriction = true;
		for (PxU32 a = 0; a < nbBatchHeaders; ++a)
		{
			PxConstraintBatchHeader& batch = batchHeaders[a];
			solveConcludeTable[batch.mConstraintType](solverConstraintDescs + batch.mStartIndex, batch.mStride, cache);
		}

		//Save motion velocities...

		for (PxU32 a = 0; a < nbSolverBodies; ++a)
		{
			linearMotionVelocity[a] = solverBodies[a].linearVelocity;
			angularMotionVelocity[a] = solverBodies[a].angularState;
		}

		for (PxU32 i = nbVelocityIterations; i > 1; --i)
		{
			for (PxU32 a = 0; a < nbBatchHeaders; ++a)
			{
				PxConstraintBatchHeader& batch = batchHeaders[a];
				solveTable[batch.mConstraintType](solverConstraintDescs + batch.mStartIndex, batch.mStride, cache);
			}
		}

		for (PxU32 a = 0; a < nbBatchHeaders; ++a)
		{
			PxConstraintBatchHeader& batch = batchHeaders[a];
			solveWritebackTable[batch.mConstraintType](solverConstraintDescs + batch.mStartIndex, batch.mStride, cache);
		}

	}


	void createCache(Gu::Cache& cache, PxGeometryType::Enum geomType0, PxGeometryType::Enum geomType1, PxCacheAllocator& allocator)
	{
		
		if (gEnablePCMCaching[geomType0][geomType1])
		{
			if (geomType0 <= PxGeometryType::eCONVEXMESH &&
				geomType1 <= PxGeometryType::eCONVEXMESH)
			{
				if (geomType0 == PxGeometryType::eSPHERE || geomType1 == PxGeometryType::eSPHERE)
				{
					Gu::PersistentContactManifold* manifold = PX_PLACEMENT_NEW(allocator.allocateCacheData(sizeof(Gu::SpherePersistentContactManifold)), Gu::SpherePersistentContactManifold)();
					cache.setManifold(manifold);
				}
				else
				{
					Gu::PersistentContactManifold* manifold = PX_PLACEMENT_NEW(allocator.allocateCacheData(sizeof(Gu::LargePersistentContactManifold)), Gu::LargePersistentContactManifold)();
					cache.setManifold(manifold);

				}
				cache.getManifold().clearManifold();
			}
			else
			{
				//ML: raised 1 to indicate the manifold is multiManifold which is for contact gen in mesh/height field
				//cache.manifold = 1;
				cache.setMultiManifold(NULL);
			}
		}
		else
		{
			//cache.manifold =  0;
			cache.mCachedData = NULL;
			cache.mManifoldFlags = 0;
		}
	}



	bool immediate::PxGenerateContacts(const PxGeometry* const * geom0, const PxGeometry* const * geom1, const PxTransform* pose0, const PxTransform* pose1, PxCache* contactCache, const PxU32 nbPairs, PxContactRecorder& contactRecorder,
		const PxReal contactDistance, const PxReal meshContactMargin, const PxReal toleranceLength, PxCacheAllocator& allocator)
	{
		PX_ASSERT(meshContactMargin > 0.f);
		PX_ASSERT(toleranceLength > 0.f);
		PX_ASSERT(contactDistance > 0.f);
		Gu::ContactBuffer contactBuffer;

		for (PxU32 i = 0; i < nbPairs; ++i)
		{

			contactBuffer.count = 0;
			PxGeometryType::Enum type0 = geom0[i]->getType();
			PxGeometryType::Enum type1 = geom1[i]->getType();

			const PxGeometry* tempGeom0 = geom0[i];
			const PxGeometry* tempGeom1 = geom1[i];

			PX_ALIGN(16, PxTransform) transform0 = pose0[i];
			PX_ALIGN(16, PxTransform) transform1 = pose1[i];

			const bool bSwap = type0 > type1;
			if (bSwap)
			{
				const PxGeometry* temp = tempGeom0;
				tempGeom0 = geom1[i];
				tempGeom1 = temp;
				PxGeometryType::Enum tempType = type0;
				type0 = type1;
				type1 = tempType;
				transform1 = pose0[i];
				transform0 = pose1[i];
			}

			const Gu::GeometryUnion geomUnion0(*tempGeom0);
			const Gu::GeometryUnion geomUnion1(*tempGeom1);

			//Now work out which type of PCM we need...

			Gu::Cache& cache = static_cast<Gu::Cache&>(contactCache[i]);

			bool needsMultiManifold = type1 > PxGeometryType::eCONVEXMESH;

			Gu::NarrowPhaseParams params(contactDistance, meshContactMargin, toleranceLength);

			if (needsMultiManifold)
			{
				Gu::MultiplePersistentContactManifold multiManifold;

				if (cache.isMultiManifold())
				{
					multiManifold.fromBuffer(reinterpret_cast<PxU8*>(&cache.getMultipleManifold()));
				}
				else
				{
					multiManifold.initialize();
				}
				cache.setMultiManifold(&multiManifold);

				//Do collision detection, then write manifold out...
				g_PCMContactMethodTable[type0][type1](geomUnion0, geomUnion1, transform0, transform1, params, cache, contactBuffer, NULL);

				const PxU32 size = (sizeof(Gu::MultiPersistentManifoldHeader) +
					multiManifold.mNumManifolds * sizeof(Gu::SingleManifoldHeader) +
					multiManifold.mNumTotalContacts * sizeof(Gu::CachedMeshPersistentContact));

				PxU8* buffer = allocator.allocateCacheData(size);

				multiManifold.toBuffer(buffer);

				cache.setMultiManifold(buffer);

			}
			else
			{
				//Allocate the type of manifold we need again...
				Gu::PersistentContactManifold* oldManifold = NULL;

				if (cache.isManifold())
					oldManifold = &cache.getManifold();

				//Allocates and creates the PCM...
				createCache(cache, type0, type1, allocator);

				//Copy PCM from old to new manifold...
				if (oldManifold)
				{
					Gu::PersistentContactManifold& manifold = cache.getManifold();
					manifold.mRelativeTransform = oldManifold->mRelativeTransform;
					manifold.mNumContacts = oldManifold->mNumContacts;
					manifold.mNumWarmStartPoints = oldManifold->mNumWarmStartPoints;
					manifold.mAIndice[0] = oldManifold->mAIndice[0]; manifold.mAIndice[1] = oldManifold->mAIndice[1];
					manifold.mAIndice[2] = oldManifold->mAIndice[2]; manifold.mAIndice[3] = oldManifold->mAIndice[3];
					manifold.mBIndice[0] = oldManifold->mBIndice[0]; manifold.mBIndice[1] = oldManifold->mBIndice[1];
					manifold.mBIndice[2] = oldManifold->mBIndice[2]; manifold.mBIndice[3] = oldManifold->mBIndice[3];
					PxMemCopy(manifold.mContactPoints, oldManifold->mContactPoints, sizeof(Gu::PersistentContact)*manifold.mNumContacts);
				}

				g_PCMContactMethodTable[type0][type1](geomUnion0, geomUnion1, transform0, transform1, params, cache, contactBuffer, NULL);
			}

			if (bSwap)
			{
				for (PxU32 a = 0; a < contactBuffer.count; ++a)
				{
					contactBuffer.contacts[a].normal = -contactBuffer.contacts[a].normal;
				}
			}


			if (contactBuffer.count != 0)
			{
				//Record this contact pair...
				contactRecorder.recordContacts(contactBuffer.contacts, contactBuffer.count, i);
			}
		}
		return true;
	}

}

