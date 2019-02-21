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
#include "PxSceneDesc.h"
#include "PsVecMath.h"
#include "PsMathUtils.h"
#include "DySolverContact.h"
#include "DySolverContact4.h"
#include "DySolverConstraintTypes.h"
#include "PxcNpWorkUnit.h"
#include "DyThreadContext.h"
#include "DyContactPrep.h"
#include "PxcNpContactPrepShared.h"
#include "PxvDynamics.h"
#include "DyCorrelationBuffer.h"
#include "DyDynamics.h"
#include "DyArticulationContactPrep.h"
#include "PxsContactManager.h"
#include "PsFoundation.h"
#include "DyFeatherstoneArticulation.h"
#include "DyFeatherstoneArticulationLink.h"
#include "DyFeatherstoneArticulationJointData.h"

using namespace physx;
using namespace Gu;


#include "PsVecMath.h"
#include "PxContactModifyCallback.h"
#include "PxsMaterialManager.h"
#include "PxsMaterialCombiner.h"
#include "DyContactPrepShared.h"
#include "DySolverConstraint1D.h"
#include "DyConstraintPrep.h"
#include "DyTGSDynamics.h"

#include "CmConeLimitHelper.h"

#include "DySolverContext.h"
#include "DySolverConstraint1DStep.h"

using namespace Ps::aos;


namespace physx
{
namespace Dy
{
	PX_FORCE_INLINE PxReal safeRecip(const PxReal x)
	{
		return x > PX_EPS_F32 ? 1.f/x : 0.f;
	}

	PX_FORCE_INLINE void computeBlockStreamByteSizesStep(const bool useExtContacts, const CorrelationBuffer& c,
		PxU32& _solverConstraintByteSize, PxU32& _frictionPatchByteSize, PxU32& _numFrictionPatches,
		PxU32& _axisConstraintCount, PxReal torsionalPatchRadius)
	{
		PX_ASSERT(0 == _solverConstraintByteSize);
		PX_ASSERT(0 == _frictionPatchByteSize);
		PX_ASSERT(0 == _numFrictionPatches);
		PX_ASSERT(0 == _axisConstraintCount);

		// PT: use local vars to remove LHS
		PxU32 solverConstraintByteSize = 0;
		PxU32 numFrictionPatches = 0;
		PxU32 axisConstraintCount = 0;


		for (PxU32 i = 0; i < c.frictionPatchCount; i++)
		{
			//Friction patches.
			if (c.correlationListHeads[i] != CorrelationBuffer::LIST_END)
				numFrictionPatches++;

			const FrictionPatch& frictionPatch = c.frictionPatches[i];

			const bool haveFriction = (frictionPatch.materialFlags & PxMaterialFlag::eDISABLE_FRICTION) == 0;

			//Solver constraint data.
			if (c.frictionPatchContactCounts[i] != 0)
			{
				solverConstraintByteSize += sizeof(SolverContactHeaderStep);
				solverConstraintByteSize += useExtContacts ? c.frictionPatchContactCounts[i] * sizeof(SolverContactPointStepExt)
					: c.frictionPatchContactCounts[i] * sizeof(SolverContactPointStep);
				solverConstraintByteSize += sizeof(PxF32) * ((c.frictionPatchContactCounts[i] + 3)&(~3)); //Add on space for applied impulses

				axisConstraintCount += c.frictionPatchContactCounts[i];

				if (haveFriction)
				{
					PxU32 nbAnchors = PxU32(c.frictionPatches[i].anchorCount * 2);
					if (torsionalPatchRadius > 0.f && c.frictionPatches[i].anchorCount == 1)
						nbAnchors++;
					solverConstraintByteSize += useExtContacts ? nbAnchors * sizeof(SolverContactFrictionStepExt)
						: nbAnchors * sizeof(SolverContactFrictionStep);
					axisConstraintCount += nbAnchors;

				}
			}
		}
		PxU32 frictionPatchByteSize = numFrictionPatches*sizeof(FrictionPatch);

		_numFrictionPatches = numFrictionPatches;
		_axisConstraintCount = axisConstraintCount;

		//16-byte alignment.
		_frictionPatchByteSize = ((frictionPatchByteSize + 0x0f) & ~0x0f);
		_solverConstraintByteSize = ((solverConstraintByteSize + 0x0f) & ~0x0f);
		PX_ASSERT(0 == (_solverConstraintByteSize & 0x0f));
		PX_ASSERT(0 == (_frictionPatchByteSize & 0x0f));
	}

	static bool reserveBlockStreams(const bool useExtContacts, Dy::CorrelationBuffer& cBuffer,
		PxU8*& solverConstraint,
		FrictionPatch*& _frictionPatches,
		PxU32& numFrictionPatches, PxU32& solverConstraintByteSize,
		PxU32& axisConstraintCount, PxConstraintAllocator& constraintAllocator,
		PxReal torsionalPatchRadius)
	{
		PX_ASSERT(NULL == solverConstraint);
		PX_ASSERT(NULL == _frictionPatches);
		PX_ASSERT(0 == numFrictionPatches);
		PX_ASSERT(0 == solverConstraintByteSize);
		PX_ASSERT(0 == axisConstraintCount);

		//From frictionPatchStream we just need to reserve a single buffer.
		PxU32 frictionPatchByteSize = 0;
		//Compute the sizes of all the buffers.
		computeBlockStreamByteSizesStep(
			useExtContacts, cBuffer,
			solverConstraintByteSize, frictionPatchByteSize, numFrictionPatches,
			axisConstraintCount, torsionalPatchRadius);

		//Reserve the buffers.

		//First reserve the accumulated buffer size for the constraint block.
		PxU8* constraintBlock = NULL;
		const PxU32 constraintBlockByteSize = solverConstraintByteSize;
		if (constraintBlockByteSize > 0)
		{
			constraintBlock = constraintAllocator.reserveConstraintData(constraintBlockByteSize + 16u);

			if (0 == constraintBlock || (reinterpret_cast<PxU8*>(-1)) == constraintBlock)
			{
				if (0 == constraintBlock)
				{
					PX_WARN_ONCE(
						"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
						"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.");
				}
				else
				{
					PX_WARN_ONCE(
						"Attempting to allocate more than 16K of contact data for a single contact pair in constraint prep. "
						"Either accept dropped contacts or simplify collision geometry.");
					constraintBlock = NULL;
				}
			}
			PX_ASSERT((size_t(constraintBlock) & 0xF) == 0);
		}

		FrictionPatch* frictionPatches = NULL;
		//If the constraint block reservation didn't fail then reserve the friction buffer too.
		if (frictionPatchByteSize > 0 && (0 == constraintBlockByteSize || constraintBlock))
		{
			frictionPatches = reinterpret_cast<FrictionPatch*>(constraintAllocator.reserveFrictionData(frictionPatchByteSize));

			if (0 == frictionPatches || (reinterpret_cast<FrictionPatch*>(-1)) == frictionPatches)
			{
				if (0 == frictionPatches)
				{
					PX_WARN_ONCE(
						"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
						"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.");
				}
				else
				{
					PX_WARN_ONCE(
						"Attempting to allocate more than 16K of friction data for a single contact pair in constraint prep. "
						"Either accept dropped contacts or simplify collision geometry.");
					frictionPatches = NULL;
				}
			}
		}

		_frictionPatches = frictionPatches;

		//Patch up the individual ptrs to the buffer returned by the constraint block reservation (assuming the reservation didn't fail).
		if (0 == constraintBlockByteSize || constraintBlock)
		{
			if (solverConstraintByteSize)
			{
				solverConstraint = constraintBlock;
				PX_ASSERT(0 == (uintptr_t(solverConstraint) & 0x0f));
			}
		}

		//Return true if neither of the two block reservations failed.
		return ((0 == constraintBlockByteSize || constraintBlock) && (0 == frictionPatchByteSize || frictionPatches));
	}

	class SolverExtBodyStep
	{
	public:
		union
		{
			const ArticulationV* mArticulation;
			const PxTGSSolverBodyVel* mBody;
		};

		const PxTGSSolverBodyTxInertia* mTxI;
		const PxStepSolverBodyData* mData;

		PxU16 mLinkIndex;

		SolverExtBodyStep(const void* bodyOrArticulation, const PxTGSSolverBodyTxInertia* txI, 
			const PxStepSolverBodyData* data, PxU16 linkIndex) :
			mBody(reinterpret_cast<const PxTGSSolverBodyVel*>(bodyOrArticulation)),
			mTxI(txI),
			mData(data),
			mLinkIndex(linkIndex)
		{}

		PxReal projectVelocity(const PxVec3& linear, const PxVec3& angular) const;
		PxVec3 getLinVel() const;
		PxVec3 getAngVel() const;
		bool isKinematic() const 
		{ 
			return (mLinkIndex == PxSolverConstraintDesc::NO_LINK) && 
				mBody->isKinematic;
		}
	};

	Cm::SpatialVector createImpulseResponseVector(const PxVec3& linear, const PxVec3& angular, const SolverExtBodyStep& body)
	{
		if (body.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
		{
			return Cm::SpatialVector(linear, body.mTxI->sqrtInvInertia * angular);
		}
		return Cm::SpatialVector(linear, angular);
	}


	PxReal getImpulseResponse(const SolverExtBodyStep& b0, const Cm::SpatialVector& impulse0, Cm::SpatialVector& deltaV0, PxReal dom0, PxReal angDom0,
		const SolverExtBodyStep& b1, const Cm::SpatialVector& impulse1, Cm::SpatialVector& deltaV1, PxReal dom1, PxReal angDom1,
		bool allowSelfCollision)
	{
		PxReal response;
		if (allowSelfCollision && b0.mArticulation == b1.mArticulation)
		{
			Cm::SpatialVectorF Z[64];
			b0.mArticulation->getImpulseSelfResponse(b0.mLinkIndex, b1.mLinkIndex, Z,
				impulse0.scale(dom0, angDom0), impulse1.scale(dom1, angDom1), deltaV0, deltaV1);

			response = impulse0.dot(deltaV0) + impulse1.dot(deltaV1);
		}
		else
		{

			if (b0.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
			{
				deltaV0.linear = impulse0.linear * b0.mData->invMass * dom0;
				deltaV0.angular =/* b0.mBody->sqrtInvInertia * */impulse0.angular * angDom0;
			}
			else
			{
				//ArticulationHelper::getImpulseResponse(*b0.mFsData, b0.mLinkIndex, impulse0.scale(dom0, angDom0), deltaV0);
				const ArticulationV* articulation = b0.mArticulation;
				Cm::SpatialVectorF Z[64];
				articulation->getImpulseResponse(b0.mLinkIndex, Z, impulse0.scale(dom0, angDom0), deltaV0);
			}

			response = impulse0.dot(deltaV0);
			if (b1.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
			{
				deltaV1.linear = impulse1.linear * b1.mData->invMass * dom1;
				deltaV1.angular = /*b1.mBody->sqrtInvInertia * */impulse1.angular * angDom1;
			}
			else
			{
				const ArticulationV* articulation = b1.mArticulation;
				Cm::SpatialVectorF Z[64];
				articulation->getImpulseResponse(b1.mLinkIndex, Z, impulse1.scale(dom1, angDom1), deltaV1);
				//ArticulationHelper::getImpulseResponse(*b1.mFsData, b1.mLinkIndex, impulse1.scale(dom1, angDom1), deltaV1);

			}
			response += impulse1.dot(deltaV1);
		}

		return response;
	}


	PxReal SolverExtBodyStep::projectVelocity(const PxVec3& linear, const PxVec3& angular) const
	{
		if (mLinkIndex == PxSolverConstraintDesc::NO_LINK)
		{
			return mData->projectVelocity(linear, angular);
		}
		else
		{
			Cm::SpatialVectorV velocities = mArticulation->getLinkVelocity(mLinkIndex);
			FloatV fv = velocities.dot(Cm::SpatialVector(linear, angular));
			PxF32 f;
			FStore(fv, &f);
			/*PxF32 f;
			FStore(getVelocity(*mFsData)[mLinkIndex].dot(Cm::SpatialVector(linear, angular)), &f);
			return f;*/
			return f;
		}
	}



	static FloatV constructContactConstraintStep(const Mat33V& sqrtInvInertia0, const Mat33V& sqrtInvInertia1, const FloatVArg invMassNorLenSq0,
		const FloatVArg invMassNorLenSq1, const FloatVArg angD0, const FloatVArg angD1, const Vec3VArg bodyFrame0p, const Vec3VArg bodyFrame1p,
		const QuatV& /*bodyFrame0q*/, const QuatV& /*bodyFrame1q*/,
		const Vec3VArg normal, const FloatVArg norVel0, const FloatVArg norVel1, const VecCrossV& norCross, const Vec3VArg angVel0, const Vec3VArg angVel1,
		const FloatVArg invDt, const FloatVArg invDtp8, const FloatVArg restDistance, const FloatVArg restitution,
		const FloatVArg bounceThreshold, const Gu::ContactPoint& contact, SolverContactPointStep& solverContact,
		const FloatVArg /*ccdMaxSeparation*/, const bool isKinematic0, const bool isKinematic1)
	{
		const FloatV zero = FZero();
		const Vec3V point = V3LoadA(contact.point);
		const FloatV separation = FLoad(contact.separation);

		const FloatV cTargetVel = V3Dot(normal, V3LoadA(contact.targetVel));

		const Vec3V ra = V3Sub(point, bodyFrame0p);
		const Vec3V rb = V3Sub(point, bodyFrame1p);

		const Vec3V raXn = V3Cross(ra, norCross);
		const Vec3V rbXn = V3Cross(rb, norCross);

		const Vec3V raXnInertia = M33MulV3(sqrtInvInertia0, raXn);
		const Vec3V rbXnInertia = M33MulV3(sqrtInvInertia1, rbXn);

		const FloatV resp0 = FAdd(invMassNorLenSq0, FMul(V3Dot(raXnInertia, raXnInertia), angD0));
		const FloatV resp1 = FSub(FMul(V3Dot(rbXnInertia, rbXnInertia), angD1), invMassNorLenSq1);

		const FloatV unitResponse = FAdd(resp0, resp1);

		const FloatV vrel1 = FAdd(norVel0, V3Dot(raXn, angVel0));
		const FloatV vrel2 = FAdd(norVel1, V3Dot(rbXn, angVel1));
		const FloatV vrel = FSub(vrel1, vrel2);

		const FloatV penetration = FSub(separation, restDistance);

		const FloatV penetrationInvDt = FMul(penetration, invDt);

		FloatV scaledBias = FNeg(invDtp8);

		const BoolV isGreater2 = BAnd(BAnd(FIsGrtr(restitution, zero), FIsGrtr(bounceThreshold, vrel)), FIsGrtr(FNeg(vrel), penetrationInvDt));

		const FloatV totalError = penetration;

		const FloatV sumVRel(vrel);

		const FloatV velMultiplier = FSel(FIsGrtr(unitResponse, FZero()), FRecip(unitResponse), FZero());

		FloatV targetVelocity = FAdd(cTargetVel, FSel(isGreater2, FMul(FNeg(sumVRel), restitution), zero));

		if(isKinematic0)
			targetVelocity = FSub(targetVelocity, vrel1);
		if(isKinematic1)
			targetVelocity = FAdd(targetVelocity, vrel2);

		//KS - don't store scaled by angD0/angD1 here! It'll corrupt the velocity projection...
		V3StoreA(raXnInertia, solverContact.raXnI);
		V3StoreA(rbXnInertia, solverContact.rbXnI);

		FStore(velMultiplier, &solverContact.velMultiplier);

		FStore(totalError, &solverContact.separation);
		FStore(scaledBias, &solverContact.biasCoefficient);
		FStore(targetVelocity, &solverContact.targetVelocity);	

		return penetration;

	}


	static void setupFinalizeSolverConstraints(Sc::ShapeInteraction* shapeInteraction,
		const ContactPoint* buffer,
		const CorrelationBuffer& c,
		const PxTransform& bodyFrame0,
		const PxTransform& bodyFrame1,
		PxU8* workspace,
		const PxTGSSolverBodyVel& b0,
		const PxTGSSolverBodyVel& b1,
		const PxTGSSolverBodyTxInertia& txI0,
		const PxTGSSolverBodyTxInertia& txI1,
		const PxStepSolverBodyData& data0,
		const PxStepSolverBodyData& data1,
		const PxReal invDtF32,
		const PxReal invTotalDtF32,
		PxReal bounceThresholdF32,
		PxReal invMassScale0, PxReal invInertiaScale0,
		PxReal invMassScale1, PxReal invInertiaScale1,
		bool hasForceThreshold, bool staticOrKinematicBody,
		const PxReal restDist, PxU8* frictionDataPtr,
		const PxReal maxCCDSeparation,
		const bool disableStrongFriction,
		const PxReal torsionalPatchRadiusF32,
		const PxReal minTorsionalPatchRadiusF32)
	{
		bool hasTorsionalFriction = torsionalPatchRadiusF32 > 0.f || minTorsionalPatchRadiusF32 > 0.f;

		// NOTE II: the friction patches are sparse (some of them have no contact patches, and
		// therefore did not get written back to the cache) but the patch addresses are dense,
		// corresponding to valid patches

		const bool isKinematic0 = b0.isKinematic;
		const bool isKinematic1 = b1.isKinematic;

		const FloatV ccdMaxSeparation = FLoad(maxCCDSeparation);

		PxU8 flags = PxU8(hasForceThreshold ? SolverContactHeaderStep::eHAS_FORCE_THRESHOLDS : 0);

		PxU8* PX_RESTRICT ptr = workspace;

		PxU8 type = Ps::to8(staticOrKinematicBody ? DY_SC_TYPE_STATIC_CONTACT
			: DY_SC_TYPE_RB_CONTACT);

		const FloatV zero = FZero();

		const FloatV d0 = FLoad(invMassScale0);
		const FloatV d1 = FLoad(invMassScale1);
		const FloatV angD0 = FLoad(invInertiaScale0);
		const FloatV angD1 = FLoad(invInertiaScale1);

		const FloatV nDom1fV = FNeg(d1);

		const FloatV invMass0 = FLoad(data0.invMass);
		const FloatV invMass1 = FLoad(data1.invMass);

		const FloatV invMass0_dom0fV = FMul(d0, invMass0);
		const FloatV invMass1_dom1fV = FMul(nDom1fV, invMass1);

		Vec4V staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4Zero();
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, invMass0_dom0fV);
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, invMass1_dom1fV);

		const FloatV restDistance = FLoad(restDist);

		const PxReal maxPenBias = PxMax(data0.penBiasClamp, data1.penBiasClamp);

		const QuatV bodyFrame0q = QuatVLoadU(&bodyFrame0.q.x);
		const Vec3V bodyFrame0p = V3LoadU(bodyFrame0.p);

		const QuatV bodyFrame1q = QuatVLoadU(&bodyFrame1.q.x);
		const Vec3V bodyFrame1p = V3LoadU(bodyFrame1.p);

		

		PxU32 frictionPatchWritebackAddrIndex = 0;
		PxU32 contactWritebackCount = 0;

		Ps::prefetchLine(c.contactID);
		Ps::prefetchLine(c.contactID, 128);

		//const Vec3V linVel0 = V3LoadU_SafeReadW(b0.linearVelocity);	// PT: safe because 'invMass' follows 'initialLinVel' in PxSolverBodyData
		//const Vec3V linVel1 = V3LoadU_SafeReadW(b1.linearVelocity);	// PT: safe because 'invMass' follows 'initialLinVel' in PxSolverBodyData
		//const Vec3V angVel0 = V3LoadU_SafeReadW(b0.angularVelocity);	// PT: safe because 'reportThreshold' follows 'initialAngVel' in PxSolverBodyData
		//const Vec3V angVel1 = V3LoadU_SafeReadW(b1.angularVelocity);	// PT: safe because 'reportThreshold' follows 'initialAngVel' in PxSolverBodyData


		const Vec3V linVel0 = V3LoadU_SafeReadW(data0.originalLinearVelocity);	// PT: safe because 'invMass' follows 'initialLinVel' in PxSolverBodyData
		const Vec3V linVel1 = V3LoadU_SafeReadW(data1.originalLinearVelocity);	// PT: safe because 'invMass' follows 'initialLinVel' in PxSolverBodyData
		const Vec3V angVel0 = V3LoadU_SafeReadW(data0.originalAngularVelocity);	// PT: safe because 'reportThreshold' follows 'initialAngVel' in PxSolverBodyData
		const Vec3V angVel1 = V3LoadU_SafeReadW(data1.originalAngularVelocity);	// PT: safe because 'reportThreshold' follows 'initialAngVel' in PxSolverBodyData


		PX_ALIGN(16, const Mat33V sqrtInvInertia0)
			(
			V3LoadU_SafeReadW(txI0.sqrtInvInertia.column0),	// PT: safe because 'column1' follows 'column0' in PxMat33
			V3LoadU_SafeReadW(txI0.sqrtInvInertia.column1),	// PT: safe because 'column2' follows 'column1' in PxMat33
			V3LoadU(txI0.sqrtInvInertia.column2)
			);

		PX_ALIGN(16, const Mat33V sqrtInvInertia1)
			(
			V3LoadU_SafeReadW(txI1.sqrtInvInertia.column0),	// PT: safe because 'column1' follows 'column0' in PxMat33
			V3LoadU_SafeReadW(txI1.sqrtInvInertia.column1),	// PT: safe because 'column2' follows 'column1' in PxMat33
			V3LoadU(txI1.sqrtInvInertia.column2)
			);

		const FloatV invDt = FLoad(invDtF32);
		const FloatV invTotalDt = FLoad(invTotalDtF32);
		const FloatV p8 = FLoad(0.8f);
		const FloatV bounceThreshold = FLoad(bounceThresholdF32);

		const FloatV invDtp8 = FMul(invDt, p8);

		const PxReal frictionBiasScale = disableStrongFriction ? 0.f : invDtF32;


		for (PxU32 i = 0; i<c.frictionPatchCount; i++)
		{
			PxU32 contactCount = c.frictionPatchContactCounts[i];
			if (contactCount == 0)
				continue;

			const FrictionPatch& frictionPatch = c.frictionPatches[i];
			PX_ASSERT(frictionPatch.anchorCount <= 2);

			PxU32 firstPatch = c.correlationListHeads[i];
			const Gu::ContactPoint* contactBase0 = buffer + c.contactPatches[firstPatch].start;

			const PxReal combinedRestitution = contactBase0->restitution;

			SolverContactHeaderStep* PX_RESTRICT header = reinterpret_cast<SolverContactHeaderStep*>(ptr);
			ptr += sizeof(SolverContactHeaderStep);


			Ps::prefetchLine(ptr, 128);
			Ps::prefetchLine(ptr, 256);

			header->shapeInteraction = shapeInteraction;
			header->flags = flags;
			FStore(invMass0_dom0fV, &header->invMass0);
			FStore(FNeg(invMass1_dom1fV), &header->invMass1);
			const FloatV restitution = FLoad(combinedRestitution);

			PxU32 pointStride = sizeof(SolverContactPointStep);
			PxU32 frictionStride = sizeof(SolverContactFrictionStep);

			const Vec3V normal = V3LoadA(buffer[c.contactPatches[c.correlationListHeads[i]].start].normal);
			const FloatV normalLenSq = V3LengthSq(normal);
			const VecCrossV norCross = V3PrepareCross(normal);
			//const FloatV norVel = V3SumElems(V3NegMulSub(normal, linVel1, V3Mul(normal, linVel0)));
			const FloatV norVel0 = V3Dot(linVel0, normal);
			const FloatV norVel1 = V3Dot(linVel1, normal);

			const FloatV invMassNorLenSq0 = FMul(invMass0_dom0fV, normalLenSq);
			const FloatV invMassNorLenSq1 = FMul(invMass1_dom1fV, normalLenSq);

			V3StoreA(normal, header->normal);
			header->maxPenBias = maxPenBias;

			FloatV maxPenetration = FMax();

			for (PxU32 patch = c.correlationListHeads[i];
				patch != CorrelationBuffer::LIST_END;
				patch = c.contactPatches[patch].next)
			{
				const PxU32 count = c.contactPatches[patch].count;
				const Gu::ContactPoint* contactBase = buffer + c.contactPatches[patch].start;

				PxU8* p = ptr;

				for (PxU32 j = 0; j<count; j++)
				{
					Ps::prefetchLine(p, 256);
					const Gu::ContactPoint& contact = contactBase[j];

					SolverContactPointStep* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPointStep*>(p);
					p += pointStride;

					maxPenetration = FMin(maxPenetration, constructContactConstraintStep(sqrtInvInertia0, sqrtInvInertia1, invMassNorLenSq0,
						invMassNorLenSq1, angD0, angD1, bodyFrame0p, bodyFrame1p, bodyFrame0q, bodyFrame1q,
						normal, norVel0, norVel1, norCross, angVel0, angVel1,
						invTotalDt, invDtp8, restDistance, restitution,
						bounceThreshold, contact, *solverContact,
						ccdMaxSeparation, isKinematic0, isKinematic1));
				}

				ptr = p;
			}

			contactWritebackCount += contactCount;

			PxF32* forceBuffers = reinterpret_cast<PxF32*>(ptr);
			PxMemZero(forceBuffers, sizeof(PxF32) * contactCount);
			ptr += ((contactCount + 3) & (~3)) * sizeof(PxF32); // jump to next 16-byte boundary

			const PxReal staticFriction = contactBase0->staticFriction;
			const PxReal dynamicFriction = contactBase0->dynamicFriction;
			const bool disableFriction = !!(contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION);
			staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(staticFriction));
			staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(dynamicFriction));

			const bool haveFriction = (disableFriction == 0 && frictionPatch.anchorCount != 0);//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
			header->numNormalConstr = Ps::to8(contactCount);
			header->numFrictionConstr = Ps::to8(haveFriction ? frictionPatch.anchorCount * 2 : 0);

			header->type = type;

			header->staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;
			FStore(angD0, &header->angDom0);
			FStore(angD1, &header->angDom1);

			header->broken = 0;

			if (haveFriction)
			{
				

				const Vec3V linVrel = V3Sub(linVel0, linVel1);
				//const Vec3V normal = Vec3V_From_PxVec3_Aligned(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);

				const FloatV orthoThreshold = FLoad(0.70710678f);
				const FloatV p1 = FLoad(0.0001f);
				// fallback: normal.cross((1,0,0)) or normal.cross((0,0,1))
				const FloatV normalX = V3GetX(normal);
				const FloatV normalY = V3GetY(normal);
				const FloatV normalZ = V3GetZ(normal);

				Vec3V t0Fallback1 = V3Merge(zero, FNeg(normalZ), normalY);
				Vec3V t0Fallback2 = V3Merge(FNeg(normalY), normalX, zero);
				Vec3V t0Fallback = V3Sel(FIsGrtr(orthoThreshold, FAbs(normalX)), t0Fallback1, t0Fallback2);

				Vec3V t0 = V3Sub(linVrel, V3Scale(normal, V3Dot(normal, linVrel)));
				t0 = V3Sel(FIsGrtr(V3LengthSq(t0), p1), t0, t0Fallback);
				t0 = V3Normalize(t0);

				const VecCrossV t0Cross = V3PrepareCross(t0);

				const Vec3V t1 = V3Normalize(V3Cross(norCross, t0Cross));
				//const VecCrossV t1Cross = V3PrepareCross(t1);


				// since we don't even have the body velocities we can't compute the tangent dirs, so 
				// the only thing we can do right now is to write the geometric information (which is the
				// same for both axis constraints of an anchor) We put ra in the raXn field, rb in the rbXn
				// field, and the error in the normal field. See corresponding comments in
				// completeContactFriction()

				//We want to set the writeBack ptr to point to the broken flag of the friction patch.
				//On spu we have a slight problem here because the friction patch array is 
				//in local store rather than in main memory. The good news is that the address of the friction 
				//patch array in main memory is stored in the work unit. These two addresses will be equal 
				//except on spu where one is local store memory and the other is the effective address in main memory.
				//Using the value stored in the work unit guarantees that the main memory address is used on all platforms.
				PxU8* PX_RESTRICT writeback = frictionDataPtr + frictionPatchWritebackAddrIndex*sizeof(FrictionPatch);

				

				const FloatV norVel00 = V3Dot(linVel0, t0);
				const FloatV norVel01 = V3Dot(linVel1, t0);
				const FloatV norVel10 = V3Dot(linVel0, t1);
				const FloatV norVel11 = V3Dot(linVel1, t1);
				
				const Vec3V relTr = V3Sub(bodyFrame0p, bodyFrame1p);

				header->frictionBrokenWritebackByte = writeback;

				PxReal frictionScale = (contactBase0->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && frictionPatch.anchorCount == 2) ? 0.5f : 1.f;

				for (PxU32 j = 0; j < frictionPatch.anchorCount; j++)
				{
					Ps::prefetchLine(ptr, 256);
					Ps::prefetchLine(ptr, 384);
					SolverContactFrictionStep* PX_RESTRICT f0 = reinterpret_cast<SolverContactFrictionStep*>(ptr);
					ptr += frictionStride;
					SolverContactFrictionStep* PX_RESTRICT f1 = reinterpret_cast<SolverContactFrictionStep*>(ptr);
					ptr += frictionStride;

					Vec3V body0Anchor = V3LoadU(frictionPatch.body0Anchors[j]);
					Vec3V body1Anchor = V3LoadU(frictionPatch.body1Anchors[j]);

					Vec3V ra = QuatRotate(bodyFrame0q, body0Anchor);
					Vec3V rb = QuatRotate(bodyFrame1q, body1Anchor);

					PxU32 index = c.contactID[i][j];

					index = index == 0xFFFF ? c.contactPatches[c.correlationListHeads[i]].start : index;

					const Vec3V tvel = V3LoadA(buffer[index].targetVel);

					const Vec3V error = V3Add(V3Sub(ra, rb), relTr);

					

					{
						const Vec3V raXn = V3Cross(ra, t0);
						const Vec3V rbXn = V3Cross(rb, t0);
						const Vec3V raXnInertia = M33MulV3(sqrtInvInertia0, raXn);
						const Vec3V rbXnInertia = M33MulV3(sqrtInvInertia1, rbXn);

						const FloatV resp0 = FAdd(invMassNorLenSq0, FMul(V3Dot(raXnInertia, raXnInertia), angD0));
						const FloatV resp1 = FSub(FMul(V3Dot(rbXnInertia, rbXnInertia), angD1), invMassNorLenSq1);

						const FloatV unitResponse = FAdd(resp0, resp1);

						const FloatV velMultiplier = FSel(FIsGrtr(unitResponse, zero), FDiv(p8, unitResponse), zero);

						FloatV targetVel = V3Dot(tvel, t0);

						if(isKinematic0)
							targetVel = FSub(targetVel, FAdd(norVel00, V3Dot(raXn, angVel0)));
						if (isKinematic1)
							targetVel = FAdd(targetVel, FAdd(norVel01, V3Dot(rbXn, angVel1)));

						f0->normalXYZ_ErrorW = V4SetW(t0, V3Dot(error, t0));
						//f0->raXnXYZ_targetVelW = V4SetW(body0Anchor, targetVel);
						//f0->rbXnXYZ_biasW = V4SetW(body1Anchor, FZero());
						/*f0->raXn_biasScaleW = V4SetW(Vec4V_From_Vec3V(raXn), frictionBiasScale);
						f0->rbXn_errorW = V4SetW(rbXn, V3Dot(error, t0));*/
						f0->raXnI_targetVelW = V4SetW(raXnInertia, targetVel);
						f0->rbXnI_velMultiplierW = V4SetW(rbXnInertia, velMultiplier);
						f0->appliedForce = 0.f;
						f0->frictionScale = frictionScale;
						f0->biasScale = frictionBiasScale;
					}

					{
						FloatV targetVel = V3Dot(tvel, t1);


						const Vec3V raXn = V3Cross(ra, t1);
						const Vec3V rbXn = V3Cross(rb, t1);
						const Vec3V raXnInertia = M33MulV3(sqrtInvInertia0, raXn);
						const Vec3V rbXnInertia = M33MulV3(sqrtInvInertia1, rbXn);

						const FloatV resp0 = FAdd(invMassNorLenSq0, FMul(V3Dot(raXnInertia, raXnInertia), angD0));
						const FloatV resp1 = FSub(FMul(V3Dot(rbXnInertia, rbXnInertia), angD1), invMassNorLenSq1);

						const FloatV unitResponse = FAdd(resp0, resp1);

						const FloatV velMultiplier = FSel(FIsGrtr(unitResponse, zero), FDiv(p8, unitResponse), zero);

						if (isKinematic0)
							targetVel = FSub(targetVel, FAdd(norVel10, V3Dot(raXn, angVel0)));
						if (isKinematic1)
							targetVel = FAdd(targetVel, FAdd(norVel11, V3Dot(rbXn, angVel1)));


						f1->normalXYZ_ErrorW = V4SetW(t1, V3Dot(error, t1));
						//f1->raXnXYZ_targetVelW = V4SetW(body0Anchor, targetVel);
						//f1->rbXnXYZ_biasW = V4SetW(body1Anchor, FZero());
						//f1->raXn_biasScaleW = V4SetW(Vec4V_From_Vec3V(V3Cross(ra, t1)), frictionBiasScale);
						//f1->rbXn_errorW = V4SetW(V3Cross(rb, t1), V3Dot(error, t1));
						f1->raXnI_targetVelW = V4SetW(raXnInertia, targetVel);
						f1->rbXnI_velMultiplierW = V4SetW(rbXnInertia, velMultiplier);
						f1->appliedForce = 0.f;
						f1->frictionScale = frictionScale;
						f1->biasScale = frictionBiasScale;
					}
				}

				if (hasTorsionalFriction && frictionPatch.anchorCount == 1)
				{
					const FloatV torsionalPatchRadius = FLoad(torsionalPatchRadiusF32);
					const FloatV minTorsionalPatchRadius = FLoad(minTorsionalPatchRadiusF32);
					const FloatV torsionalFriction = FMax(minTorsionalPatchRadius, FSqrt(FMul(FMax(zero, FNeg(maxPenetration)), torsionalPatchRadius)));
					header->numFrictionConstr++;
					SolverContactFrictionStep* PX_RESTRICT f = reinterpret_cast<SolverContactFrictionStep*>(ptr);
					ptr += frictionStride;

					//Rotate the oldRelativeQuat into world space to get the new target relative quat
					const PxQuat newTargetQ1 = txI0.deltaBody2World.q * frictionPatch.relativeQuat;
					//Now we need to find the rotation around "normal" from Q1 to newTargetQ1. This is the error...
					const PxQuat deltaQ = newTargetQ1.getConjugate() * txI1.deltaBody2World.q;

					const PxVec3 nTemp = buffer[c.contactPatches[c.correlationListHeads[i]].start].normal;

					PxQuat temp(deltaQ.x*nTemp.x, deltaQ.y*nTemp.y, deltaQ.z*nTemp.z, deltaQ.w);

					const PxReal magnitude = temp.normalize();

					PxReal angle = PxAtan(physx::intrinsics::fsel(magnitude - 1e-6f, temp.dot(PxQuat(nTemp.x, nTemp.y, nTemp.z, 0.f)) / temp.w, 0.f));

					//OK. We have old relative quat and new relative quat. Now find difference


					const Vec3V raXnInertia = M33MulV3(sqrtInvInertia0, normal);
					const Vec3V rbXnInertia = M33MulV3(sqrtInvInertia1, normal);

					const FloatV resp0 = FMul(V3Dot(raXnInertia, raXnInertia), angD0);
					const FloatV resp1 = FMul(V3Dot(rbXnInertia, rbXnInertia), angD1);

					const FloatV unitResponse = FAdd(resp0, resp1);

					const FloatV velMultiplier = FSel(FIsGrtr(unitResponse, zero), FDiv(p8, unitResponse), zero);

					FloatV targetVel = zero;

					if (isKinematic0)
						targetVel = V3Dot(normal, angVel0);
					if (isKinematic1)
						targetVel = V3Dot(normal, angVel1);

					f->normalXYZ_ErrorW = V4SetW(V3Zero(), FLoad(-angle));
					f->raXnI_targetVelW = V4SetW(raXnInertia, targetVel);
					f->rbXnI_velMultiplierW = V4SetW(rbXnInertia, velMultiplier);
					f->biasScale = frictionBiasScale;
					f->appliedForce = 0.f;
					FStore(torsionalFriction, &f->frictionScale);
				}
			}

			frictionPatchWritebackAddrIndex++;
		}
	}


	static FloatV setupExtSolverContactStep(const SolverExtBodyStep& b0, const SolverExtBodyStep& b1,
		const PxF32 d0, const PxF32 d1, const PxF32 angD0, const PxF32 angD1, const PxTransform& bodyFrame0, const PxTransform& bodyFrame1,
		const Vec3VArg normal, const FloatVArg invDt, const FloatVArg invDtp8, const FloatVArg restDistance, const FloatVArg restitution,
		const FloatVArg bounceThreshold, const Gu::ContactPoint& contact, SolverContactPointStepExt& solverContact, const FloatVArg /*ccdMaxSeparation*/,
		const bool isKinematic0, const bool isKinematic1)
	{
		const FloatV zero = FZero();
		const FloatV separation = FLoad(contact.separation);

		const FloatV penetration = FSub(separation, restDistance);

		const PxVec3 ra = contact.point - bodyFrame0.p;
		const PxVec3 rb = contact.point - bodyFrame1.p;

		const PxVec3 raXn = ra.cross(contact.normal);
		const PxVec3 rbXn = rb.cross(contact.normal);

		Cm::SpatialVector deltaV0, deltaV1;

		const Cm::SpatialVector resp0 = createImpulseResponseVector(contact.normal, raXn, b0);
		const Cm::SpatialVector resp1 = createImpulseResponseVector(-contact.normal, -rbXn, b1);

		PxReal resp = getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
			b1, resp1, deltaV1, d1, angD1, false);
		
		FloatV unitResponse = FLoad(resp);

		if (b0.mLinkIndex != PxSolverConstraintDesc::NO_LINK &&
			b1.mLinkIndex != PxSolverConstraintDesc::NO_LINK)
		{
			PxReal lr0 = deltaV0.linear.dot(resp0.linear);
			PxReal ar0 = deltaV0.angular.dot(resp0.angular) * safeRecip(resp0.angular.magnitude());
			PxReal lr1 = deltaV1.linear.dot(resp1.linear);
			PxReal ar1 = deltaV1.angular.dot(resp1.angular)* safeRecip(resp1.angular.magnitude());

			Cm::SpatialVector rem0(deltaV0.linear - resp0.linear*lr0, deltaV0.angular - resp0.angular*ar0);
			Cm::SpatialVector rem1(deltaV1.linear - resp1.linear*lr1, deltaV1.angular - resp1.angular*ar1);


			PxReal rem = (rem0 - rem1).magnitude();

			unitResponse = FAdd(unitResponse, FLoad(rem));
		}

		const FloatV vel0 = FLoad(b0.projectVelocity(contact.normal, raXn));
		const FloatV vel1 = FLoad(b1.projectVelocity(contact.normal, rbXn));
		const FloatV vrel = FSub(vel0, vel1);

		


		FloatV velMultiplier = FSel(FIsGrtr(unitResponse, FEps()), FRecip(unitResponse), zero);
		FloatV scaledBias = invDtp8;
		const FloatV penetrationInvDt = FMul(penetration, invDt);

		const BoolV isGreater2 = BAnd(BAnd(FIsGrtr(restitution, zero), FIsGrtr(bounceThreshold, vrel)), FIsGrtr(FNeg(vrel), penetrationInvDt));

		//scaledBias = FSel(isGreater2, FZero(), FNeg(scaledBias));
		scaledBias = FNeg(scaledBias);

		FloatV targetVelocity = FSel(isGreater2, FMul(FNeg(vrel), restitution), zero);

		targetVelocity = FAdd(targetVelocity, V3Dot(V3LoadA(contact.targetVel), normal));

		if (isKinematic0)
			targetVelocity = FSub(targetVelocity, vel0);
		if(isKinematic1)
			targetVelocity = FAdd(targetVelocity, vel1);

		const FloatV biasedErr = FMul(FAdd(targetVelocity, FMul(penetration, scaledBias)),velMultiplier);

		const FloatV deltaF = FMax(FNegScaleSub(vrel, velMultiplier, biasedErr), zero);


		FStore(velMultiplier, &solverContact.velMultiplier);
		FStore(scaledBias, &solverContact.biasCoefficient);
		FStore(penetration, &solverContact.separation);
		FStore(targetVelocity, &solverContact.targetVelocity);

		solverContact.raXnI = resp0.angular;
		solverContact.rbXnI = -resp1.angular;
		solverContact.linDeltaVA = V3LoadA(deltaV0.linear);
		solverContact.angDeltaVA = V3LoadA(deltaV0.angular);
		solverContact.linDeltaVB = V3LoadA(deltaV1.linear);
		solverContact.angDeltaVB = V3LoadA(deltaV1.angular);

		return deltaF;
	}

	PX_INLINE void computeFrictionTangents(const PxVec3& vrel, const PxVec3& unitNormal, PxVec3& t0, PxVec3& t1)
	{
		PX_ASSERT(PxAbs(unitNormal.magnitude() - 1)<1e-3f);

		t0 = vrel - unitNormal * unitNormal.dot(vrel);
		PxReal ll = t0.magnitudeSquared();

		if (ll > 0.1f)										//can set as low as 0.
		{
			t0 *= PxRecipSqrt(ll);
			t1 = unitNormal.cross(t0);
		}
		else
			Ps::normalToTangents(unitNormal, t0, t1);		//fallback
	}

	PxVec3 SolverExtBodyStep::getLinVel() const
	{
		if (mLinkIndex == PxSolverConstraintDesc::NO_LINK)
			return mBody->linearVelocity;
		else
		{
			Cm::SpatialVectorV velocity = mArticulation->getLinkVelocity(mLinkIndex);
			PxVec3 result;
			V3StoreU(velocity.linear, result);
			return result;
		}
	}



	void setupFinalizeExtSolverContactsStep(
		const ContactPoint* buffer,
		const CorrelationBuffer& c,
		const PxTransform& bodyFrame0,
		const PxTransform& bodyFrame1,
		PxU8* workspace,
		const SolverExtBodyStep& b0,
		const SolverExtBodyStep& b1,
		const PxReal invDtF32,
		const PxReal invTotalDtF32,
		PxReal bounceThresholdF32,
		PxReal invMassScale0, PxReal invInertiaScale0,
		PxReal invMassScale1, PxReal invInertiaScale1,
		const PxReal restDist,
		PxU8* frictionDataPtr,
		PxReal ccdMaxContactDist,
		const PxReal torsionalPatchRadiusF32,
		const PxReal minTorsionalPatchRadiusF32)
	{
		// NOTE II: the friction patches are sparse (some of them have no contact patches, and
		// therefore did not get written back to the cache) but the patch addresses are dense,
		// corresponding to valid patches

		/*const bool haveFriction = PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;*/

		const bool isKinematic0 = b0.isKinematic();
		const bool isKinematic1 = b1.isKinematic();

		const FloatV ccdMaxSeparation = FLoad(ccdMaxContactDist);

		bool hasTorsionalFriction = torsionalPatchRadiusF32 > 0.f || minTorsionalPatchRadiusF32 > 0.f;

		PxU8* PX_RESTRICT ptr = workspace;

		const FloatV zero = FZero();

		//KS - TODO - this should all be done in SIMD to avoid LHS
		/*const PxF32 maxPenBias0 = b0.mLinkIndex == PxSolverConstraintDesc::NO_LINK ? b0.mData->penBiasClamp : getMaxPenBias(*b0.mFsData)[b0.mLinkIndex];
		const PxF32 maxPenBias1 = b1.mLinkIndex == PxSolverConstraintDesc::NO_LINK ? b1.mData->penBiasClamp : getMaxPenBias(*b1.mFsData)[b1.mLinkIndex];*/

		PxF32 maxPenBias0 = b0.mData->penBiasClamp;
		PxF32 maxPenBias1 = b1.mData->penBiasClamp;

		if (b0.mLinkIndex != PxSolverConstraintDesc::NO_LINK)
		{
			maxPenBias0 = b0.mArticulation->getLinkMaxPenBias(b0.mLinkIndex);
		}

		if (b1.mLinkIndex != PxSolverConstraintDesc::NO_LINK)
		{
			maxPenBias1 = b1.mArticulation->getLinkMaxPenBias(b1.mLinkIndex);
		}

		const PxReal maxPenBias = PxMax(maxPenBias0, maxPenBias1);


		const PxReal d0 = invMassScale0;
		const PxReal d1 = invMassScale1;

		const PxReal angD0 = invInertiaScale0;
		const PxReal angD1 = invInertiaScale1;

		Vec4V staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4Zero();
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(d0));
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(d1));

		const FloatV restDistance = FLoad(restDist);

		PxU32 frictionPatchWritebackAddrIndex = 0;
		PxU32 contactWritebackCount = 0;

		Ps::prefetchLine(c.contactID);
		Ps::prefetchLine(c.contactID, 128);

		const FloatV invDt = FLoad(invDtF32);
		const FloatV invTotalDt = FLoad(invTotalDtF32);
		const FloatV p8 = FLoad(0.8f);
		const FloatV bounceThreshold = FLoad(bounceThresholdF32);

		const FloatV invDtp8 = FMul(invDt, p8);

		PxU8 flags = 0;

		for (PxU32 i = 0; i<c.frictionPatchCount; i++)
		{
			PxU32 contactCount = c.frictionPatchContactCounts[i];
			if (contactCount == 0)
				continue;

			const FrictionPatch& frictionPatch = c.frictionPatches[i];
			PX_ASSERT(frictionPatch.anchorCount <= 2);  //0==anchorCount is allowed if all the contacts in the manifold have a large offset. 

			const Gu::ContactPoint* contactBase0 = buffer + c.contactPatches[c.correlationListHeads[i]].start;
			const PxReal combinedRestitution = contactBase0->restitution;

			const bool useImprovedFrictionPatch = !!(contactBase0->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION);
			PxReal coefficient = useImprovedFrictionPatch ? 1.f/frictionPatch.anchorCount : 1.f;

			const PxReal staticFriction = contactBase0->staticFriction*coefficient;
			const PxReal dynamicFriction = contactBase0->dynamicFriction*coefficient;
			const bool disableStrongFriction = !!(contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION);
			staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(staticFriction));
			staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(dynamicFriction));

			const PxReal frictionBiasScale = disableStrongFriction ? 0.f : invDtF32;

			SolverContactHeaderStep* PX_RESTRICT header = reinterpret_cast<SolverContactHeaderStep*>(ptr);
			ptr += sizeof(SolverContactHeaderStep);


			Ps::prefetchLine(ptr + 128);
			Ps::prefetchLine(ptr + 256);
			Ps::prefetchLine(ptr + 384);

			const bool haveFriction = (disableStrongFriction == 0);//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
			header->numNormalConstr = Ps::to8(contactCount);
			header->numFrictionConstr = Ps::to8(haveFriction ? frictionPatch.anchorCount * 2 : 0);

			header->type = Ps::to8(DY_SC_TYPE_EXT_CONTACT);

			header->flags = flags;

			const FloatV restitution = FLoad(combinedRestitution);

			header->staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;

			header->angDom0 = angD0;
			header->angDom1 = angD1;

			const PxU32 pointStride = sizeof(SolverContactPointStepExt);
			const PxU32 frictionStride = sizeof(SolverContactFrictionStepExt);

			const Vec3V normal = V3LoadU(buffer[c.contactPatches[c.correlationListHeads[i]].start].normal);

			V3StoreA(normal, header->normal);
			header->maxPenBias = maxPenBias;

			FloatV maxPenetration = FZero();

			FloatV accumulatedImpulse = FZero();

			for (PxU32 patch = c.correlationListHeads[i];
				patch != CorrelationBuffer::LIST_END;
				patch = c.contactPatches[patch].next)
			{
				const PxU32 count = c.contactPatches[patch].count;
				const Gu::ContactPoint* contactBase = buffer + c.contactPatches[patch].start;

				PxU8* p = ptr;
				
				for (PxU32 j = 0; j<count; j++)
				{
					const Gu::ContactPoint& contact = contactBase[j];

					SolverContactPointStepExt* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPointStepExt*>(p);
					p += pointStride;

					accumulatedImpulse = FAdd(accumulatedImpulse, setupExtSolverContactStep(b0, b1, d0, d1, angD0, angD1, bodyFrame0, bodyFrame1, normal, invTotalDt, invDtp8, restDistance, restitution,
						bounceThreshold, contact, *solverContact, ccdMaxSeparation, isKinematic0, isKinematic1));

					maxPenetration = FMin(FLoad(contact.separation), maxPenetration);

				}

				ptr = p;
			}
			contactWritebackCount += contactCount;

			accumulatedImpulse = FDiv(accumulatedImpulse, FLoad(PxF32(contactCount)));

			FStore(accumulatedImpulse, &header->minNormalForce);

			PxF32* forceBuffer = reinterpret_cast<PxF32*>(ptr);
			PxMemZero(forceBuffer, sizeof(PxF32) * contactCount);
			ptr += sizeof(PxF32) * ((contactCount + 3) & (~3));

			header->broken = 0;

			if (haveFriction)
			{
				//const Vec3V normal = Vec3V_From_PxVec3(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);
				PxVec3 normalS = buffer[c.contactPatches[c.correlationListHeads[i]].start].normal;

				PxVec3 t0, t1;
				computeFrictionTangents(b0.getLinVel() - b1.getLinVel(), normalS, t0, t1);

				Vec3V vT0 = V3LoadU(t0);
				Vec3V vT1 = V3LoadU(t1);

				//We want to set the writeBack ptr to point to the broken flag of the friction patch.
				//On spu we have a slight problem here because the friction patch array is 
				//in local store rather than in main memory. The good news is that the address of the friction 
				//patch array in main memory is stored in the work unit. These two addresses will be equal 
				//except on spu where one is local store memory and the other is the effective address in main memory.
				//Using the value stored in the work unit guarantees that the main memory address is used on all platforms.
				PxU8* PX_RESTRICT writeback = frictionDataPtr + frictionPatchWritebackAddrIndex * sizeof(FrictionPatch);

				header->frictionBrokenWritebackByte = writeback;

				for (PxU32 j = 0; j < frictionPatch.anchorCount; j++)
				{
					SolverContactFrictionStepExt* PX_RESTRICT f0 = reinterpret_cast<SolverContactFrictionStepExt*>(ptr);
					ptr += frictionStride;
					SolverContactFrictionStepExt* PX_RESTRICT f1 = reinterpret_cast<SolverContactFrictionStepExt*>(ptr);
					ptr += frictionStride;

					PxVec3 ra = bodyFrame0.q.rotate(frictionPatch.body0Anchors[j]);
					PxVec3 rb = bodyFrame1.q.rotate(frictionPatch.body1Anchors[j]);
					PxVec3 error = (ra + bodyFrame0.p) - (rb + bodyFrame1.p);

					{
						const PxVec3 raXn = ra.cross(t0);
						const PxVec3 rbXn = rb.cross(t0);

						Cm::SpatialVector deltaV0, deltaV1;

						const Cm::SpatialVector resp0 = createImpulseResponseVector(t0, raXn, b0);
						const Cm::SpatialVector resp1 = createImpulseResponseVector(-t0, -rbXn, b1);
						PxReal ur = getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
							b1, resp1, deltaV1, d1, angD1, false);
						FloatV resp = FLoad(ur);

						if (b0.mLinkIndex != PxSolverConstraintDesc::NO_LINK &&
							b1.mLinkIndex != PxSolverConstraintDesc::NO_LINK)
						{
							PxReal lr0 = deltaV0.linear.dot(resp0.linear);
							PxReal ar0 = deltaV0.angular.dot(resp0.angular) * safeRecip(resp0.angular.magnitude());
							PxReal lr1 = deltaV1.linear.dot(resp1.linear);
							PxReal ar1 = deltaV1.angular.dot(resp1.angular)* safeRecip(resp1.angular.magnitude());

							Cm::SpatialVector rem0(deltaV0.linear - resp0.linear*lr0, deltaV0.angular - resp0.angular*ar0);
							Cm::SpatialVector rem1(deltaV1.linear - resp1.linear*lr1, deltaV1.angular - resp1.angular*ar1);


							PxReal rem = (rem0 - rem1).magnitude();

							resp = FAdd(resp, FLoad(rem));
						}

						const FloatV velMultiplier = FSel(FIsGrtr(resp, FEps()), FDiv(p8, resp), zero);

						PxU32 index = c.contactPatches[c.correlationListHeads[i]].start;
						PxF32 targetVel = buffer[index].targetVel.dot(t0);

						if(isKinematic0)
							targetVel -= b0.projectVelocity(t0, raXn);
						if(isKinematic1)
							targetVel += b1.projectVelocity(t0, rbXn);

						f0->normalXYZ_ErrorW = V4SetW(vT0, FLoad(error.dot(t0)));
						f0->raXnI_targetVelW = V4SetW(V4LoadA(&resp0.angular.x), FLoad(targetVel));
						f0->rbXnI_velMultiplierW = V4SetW(V4Neg(V4LoadA(&resp1.angular.x)), velMultiplier);
						f0->appliedForce = 0.f;
						f0->biasScale = frictionBiasScale;
						f0->linDeltaVA = V3LoadA(deltaV0.linear);
						f0->linDeltaVB = V3LoadA(deltaV1.linear);
						f0->angDeltaVA = V3LoadA(deltaV0.angular);
						f0->angDeltaVB = V3LoadA(deltaV1.angular);
					}

					{

						const PxVec3 raXn = ra.cross(t1);
						const PxVec3 rbXn = rb.cross(t1);

						Cm::SpatialVector deltaV0, deltaV1;


						const Cm::SpatialVector resp0 = createImpulseResponseVector(t1, raXn, b0);
						const Cm::SpatialVector resp1 = createImpulseResponseVector(-t1, -rbXn, b1);

						PxReal ur = getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
							b1, resp1, deltaV1, d1, angD1, false);
						FloatV resp = FLoad(ur);

						if (b0.mLinkIndex != PxSolverConstraintDesc::NO_LINK &&
							b1.mLinkIndex != PxSolverConstraintDesc::NO_LINK)
						{
							PxReal lr0 = deltaV0.linear.dot(resp0.linear);
							PxReal ar0 = deltaV0.angular.dot(resp0.angular) * safeRecip(resp0.angular.magnitude());
							PxReal lr1 = deltaV1.linear.dot(resp1.linear);
							PxReal ar1 = deltaV1.angular.dot(resp1.angular)* safeRecip(resp1.angular.magnitude());

							Cm::SpatialVector rem0(deltaV0.linear - resp0.linear*lr0, deltaV0.angular - resp0.angular*ar0);
							Cm::SpatialVector rem1(deltaV1.linear - resp1.linear*lr1, deltaV1.angular - resp1.angular*ar1);


							PxReal rem = (rem0 - rem1).magnitude();

							resp = FAdd(resp, FLoad(rem));
						}

						const FloatV velMultiplier = FSel(FIsGrtr(resp, FEps()), FDiv(p8, resp), zero);

						PxU32 index = c.contactPatches[c.correlationListHeads[i]].start;
						PxF32 targetVel = buffer[index].targetVel.dot(t1);

						if (isKinematic0)
							targetVel -= b0.projectVelocity(t1, raXn);
						if (isKinematic1)
							targetVel += b1.projectVelocity(t1, rbXn);

						f1->normalXYZ_ErrorW = V4SetW(vT1, FLoad(error.dot(t1)));
						f1->raXnI_targetVelW = V4SetW(V4LoadA(&resp0.angular.x), FLoad(targetVel));
						f1->rbXnI_velMultiplierW = V4SetW(V4Neg(V4LoadA(&resp1.angular.x)), velMultiplier);
						f1->appliedForce = 0.f;
						f1->biasScale = frictionBiasScale;
						f1->linDeltaVA = V3LoadA(deltaV0.linear);
						f1->linDeltaVB = V3LoadA(deltaV1.linear);
						f1->angDeltaVA = V3LoadA(deltaV0.angular);
						f1->angDeltaVB = V3LoadA(deltaV1.angular);
					}
				}

				if (hasTorsionalFriction && frictionPatch.anchorCount == 1)
				{
					const FloatV torsionalPatchRadius = FLoad(torsionalPatchRadiusF32);
					const FloatV minTorsionalPatchRadius = FLoad(minTorsionalPatchRadiusF32);
					const FloatV torsionalFriction = FMax(minTorsionalPatchRadius, FSqrt(FMul(FMax(zero, FNeg(maxPenetration)), torsionalPatchRadius)));
					header->numFrictionConstr++;
					SolverContactFrictionStepExt* PX_RESTRICT f = reinterpret_cast<SolverContactFrictionStepExt*>(ptr);
					ptr += frictionStride;

					//Rotate the oldRelativeQuat into world space to get the new target relative quat
					const PxQuat newTargetQ1 = bodyFrame0.q * frictionPatch.relativeQuat;
					//Now we need to find the rotation around "normal" from Q1 to newTargetQ1. This is the error...
					const PxQuat deltaQ = newTargetQ1.getConjugate() * bodyFrame1.q;

					const PxVec3 nTemp = buffer[c.contactPatches[c.correlationListHeads[i]].start].normal;

					PxQuat temp(deltaQ.x*nTemp.x, deltaQ.y*nTemp.y, deltaQ.z*nTemp.z, deltaQ.w);

					const PxReal magnitude = temp.normalize();

					PxReal angle = PxAtan(physx::intrinsics::fsel(magnitude - 1e-6f, temp.dot(PxQuat(nTemp.x, nTemp.y, nTemp.z, 0.f)) / temp.w, 0.f));

					//OK. We have old relative quat and new relative quat. Now find difference

					const Cm::SpatialVector resp0 = createImpulseResponseVector(PxVec3(0.f), header->normal, b0);
					const Cm::SpatialVector resp1 = createImpulseResponseVector(PxVec3(0.f), -header->normal, b1);

					Cm::SpatialVector deltaV0, deltaV1;

					PxReal ur = getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
						b1, resp1, deltaV1, d1, angD1, false);

					FloatV resp = FLoad(ur);

					if (b0.mLinkIndex != PxSolverConstraintDesc::NO_LINK &&
						b1.mLinkIndex != PxSolverConstraintDesc::NO_LINK)
					{
						PxReal lr0 = deltaV0.linear.dot(resp0.linear);
						PxReal ar0 = deltaV0.angular.dot(resp0.angular) * safeRecip(resp0.angular.magnitude());
						PxReal lr1 = deltaV1.linear.dot(resp1.linear);
						PxReal ar1 = deltaV1.angular.dot(resp1.angular)* safeRecip(resp1.angular.magnitude());

						Cm::SpatialVector rem0(deltaV0.linear - resp0.linear*lr0, deltaV0.angular - resp0.angular*ar0);
						Cm::SpatialVector rem1(deltaV1.linear - resp1.linear*lr1, deltaV1.angular - resp1.angular*ar1);


						PxReal rem = (rem0 - rem1).magnitude();

						resp = FAdd(resp, FLoad(rem));
					}


					const FloatV velMultiplier = FSel(FIsGrtr(resp, FEps()), FDiv(p8, resp), zero);

					f->normalXYZ_ErrorW = V4SetW(V3Zero(), FLoad(-angle));
					f->raXnI_targetVelW = V4SetW(V3LoadA(resp0.angular), zero);
					f->rbXnI_velMultiplierW = V4SetW(V4Neg(Vec4V_From_Vec3V(V3LoadA(resp1.angular))), velMultiplier);
					f->biasScale = frictionBiasScale;
					f->appliedForce = 0.f;
					FStore(torsionalFriction, &f->frictionScale);
					f->linDeltaVA = V3LoadA(deltaV0.linear);
					f->linDeltaVB = V3LoadA(deltaV1.linear);
					f->angDeltaVA = V3LoadA(deltaV0.angular);
					f->angDeltaVB = V3LoadA(deltaV1.angular);

				}

			}

			frictionPatchWritebackAddrIndex++;
		}
	}




	bool createFinalizeSolverContactsStep(
		PxTGSSolverContactDesc& contactDesc,
		CorrelationBuffer& c,
		const PxReal invDtF32,
		const PxReal invTotalDtF32,
		PxReal bounceThresholdF32,
		PxReal frictionOffsetThreshold,
		PxReal correlationDistance,
		PxConstraintAllocator& constraintAllocator)
	{
		Ps::prefetchLine(contactDesc.body0);
		Ps::prefetchLine(contactDesc.body1);

		c.frictionPatchCount = 0;
		c.contactPatchCount = 0;

		const bool hasForceThreshold = contactDesc.hasForceThresholds;
		const bool staticOrKinematicBody = contactDesc.bodyState1 == PxSolverContactDesc::eKINEMATIC_BODY || contactDesc.bodyState1 == PxSolverContactDesc::eSTATIC_BODY;

		const bool disableStrongFriction = contactDesc.disableStrongFriction;
		const bool useExtContacts = ((contactDesc.bodyState0 | contactDesc.bodyState1) & PxSolverContactDesc::eARTICULATION) != 0;

		PxTGSSolverConstraintDesc& desc = *contactDesc.desc;

		desc.constraintLengthOver16 = 0;


		if (contactDesc.numContacts == 0)
		{
			contactDesc.frictionPtr = NULL;
			contactDesc.frictionCount = 0;
			desc.constraint = NULL;
			return true;
		}

		if (!disableStrongFriction)
		{
			getFrictionPatches(c, contactDesc.frictionPtr, contactDesc.frictionCount, contactDesc.bodyFrame0, contactDesc.bodyFrame1, correlationDistance);
		}

		bool overflow = !createContactPatches(c, contactDesc.contacts, contactDesc.numContacts, PXC_SAME_NORMAL);
		overflow = correlatePatches(c, contactDesc.contacts, contactDesc.bodyFrame0, contactDesc.bodyFrame1, PXC_SAME_NORMAL, 0, 0) || overflow;
		PX_UNUSED(overflow);

#if PX_CHECKED
		if (overflow)
		{
			Ps::getFoundation().error(physx::PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
				"Dropping contacts in solver because we exceeded limit of 32 friction patches.");
		}
#endif

		growPatches(c, contactDesc.contacts, contactDesc.bodyFrame0, contactDesc.bodyFrame1, correlationDistance, 0, frictionOffsetThreshold + contactDesc.restDistance);

		//PX_ASSERT(patchCount == c.frictionPatchCount);

		FrictionPatch* frictionPatches = NULL;
		PxU8* solverConstraint = NULL;
		PxU32 numFrictionPatches = 0;
		PxU32 solverConstraintByteSize = 0;
		PxU32 axisConstraintCount = 0;

		const bool successfulReserve = reserveBlockStreams(
			useExtContacts, c,
			solverConstraint, frictionPatches,
			numFrictionPatches,
			solverConstraintByteSize,
			axisConstraintCount,
			constraintAllocator,
			PxMax(contactDesc.torsionalPatchRadius, contactDesc.minTorsionalPatchRadius));
		// initialise the work unit's ptrs to the various buffers.

		contactDesc.frictionPtr = NULL;
		contactDesc.frictionCount = 0;
		desc.constraint = NULL;
		desc.constraintLengthOver16 = 0;
		// patch up the work unit with the reserved buffers and set the reserved buffer data as appropriate.

		if (successfulReserve)
		{
			PxU8* frictionDataPtr = reinterpret_cast<PxU8*>(frictionPatches);
			contactDesc.frictionPtr = frictionDataPtr;
			desc.constraint = solverConstraint;
			//output.nbContacts = Ps::to8(numContacts);
			contactDesc.frictionCount = Ps::to8(numFrictionPatches);
			PX_ASSERT((solverConstraintByteSize & 0xf) == 0);
			desc.constraintLengthOver16 = Ps::to16(solverConstraintByteSize / 16);
			desc.writeBack = contactDesc.contactForces;
			desc.writeBackLengthOver4 = PxU16(contactDesc.contactForces ? contactDesc.numContacts : 0);

			//Initialise friction buffer.
			if (frictionPatches)
			{
				// PT: TODO: revisit this... not very satisfying
				//const PxU32 maxSize = numFrictionPatches*sizeof(FrictionPatch);
				Ps::prefetchLine(frictionPatches);
				Ps::prefetchLine(frictionPatches, 128);
				Ps::prefetchLine(frictionPatches, 256);

				for (PxU32 i = 0; i<c.frictionPatchCount; i++)
				{
					//if(c.correlationListHeads[i]!=CorrelationBuffer::LIST_END)
					if (c.frictionPatchContactCounts[i])
					{
						*frictionPatches++ = c.frictionPatches[i];
						Ps::prefetchLine(frictionPatches, 256);
					}
				}
			}

			//Initialise solverConstraint buffer.
			if (solverConstraint)
			{
				
				if (useExtContacts)
				{
					const SolverExtBodyStep b0(reinterpret_cast<const void*>(contactDesc.body0), contactDesc.body0TxI, contactDesc.bodyData0, desc.linkIndexA);
					const SolverExtBodyStep b1(reinterpret_cast<const void*>(contactDesc.body1), contactDesc.body1TxI, contactDesc.bodyData1, desc.linkIndexB);

					setupFinalizeExtSolverContactsStep(contactDesc.contacts, c, contactDesc.bodyFrame0, contactDesc.bodyFrame1, solverConstraint,
						b0, b1, invDtF32, invTotalDtF32, bounceThresholdF32,
						contactDesc.mInvMassScales.linear0, contactDesc.mInvMassScales.angular0, contactDesc.mInvMassScales.linear1, contactDesc.mInvMassScales.angular1,
						contactDesc.restDistance, frictionDataPtr, contactDesc.maxCCDSeparation, contactDesc.torsionalPatchRadius, contactDesc.minTorsionalPatchRadius);
				}
				else
				{

					const PxTGSSolverBodyVel& b0 = *contactDesc.body0;
					const PxTGSSolverBodyVel& b1 = *contactDesc.body1;

					setupFinalizeSolverConstraints(contactDesc.shapeInteraction, contactDesc.contacts, c, contactDesc.bodyFrame0, contactDesc.bodyFrame1, solverConstraint,
						b0, b1, *contactDesc.body0TxI, *contactDesc.body1TxI, *contactDesc.bodyData0, *contactDesc.bodyData1, invDtF32, invTotalDtF32, bounceThresholdF32,
						contactDesc.mInvMassScales.linear0, contactDesc.mInvMassScales.angular0, contactDesc.mInvMassScales.linear1, contactDesc.mInvMassScales.angular1,
						hasForceThreshold, staticOrKinematicBody, contactDesc.restDistance, frictionDataPtr, contactDesc.maxCCDSeparation, disableStrongFriction,
						contactDesc.torsionalPatchRadius, contactDesc.minTorsionalPatchRadius);
				}
				//KS - set to 0 so we have a counter for the number of times we solved the constraint
				//only going to be used on SPU but might as well set on all platforms because this code is shared
				*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize)) = 0;
			}
		}

		return successfulReserve;
	}




	bool createFinalizeSolverContactsStep(PxTGSSolverContactDesc& contactDesc,
		PxsContactManagerOutput& output,
		ThreadContext& threadContext,
		const PxReal invDtF32,
		const PxReal invTotalDt,
		PxReal bounceThresholdF32,
		PxReal frictionOffsetThreshold,
		PxReal correlationDistance,
		PxConstraintAllocator& constraintAllocator)
	{
		using namespace Gu;
		ContactBuffer& buffer = threadContext.mContactBuffer;



		buffer.count = 0;

		// We pull the friction patches out of the cache to remove the dependency on how
		// the cache is organized. Remember original addrs so we can write them back 
		// efficiently.

		PxU32 numContacts = 0;
		{
			PxReal invMassScale0 = 1.f;
			PxReal invMassScale1 = 1.f;
			PxReal invInertiaScale0 = 1.f;
			PxReal invInertiaScale1 = 1.f;
			contactDesc.mInvMassScales.angular0 = (contactDesc.bodyState0 != PxSolverContactDesc::eARTICULATION && contactDesc.body0->isKinematic) ? 0.f : contactDesc.mInvMassScales.angular0;
			contactDesc.mInvMassScales.angular1 = (contactDesc.bodyState1 != PxSolverContactDesc::eARTICULATION && contactDesc.body1->isKinematic) ? 0.f : contactDesc.mInvMassScales.angular1;

			bool hasMaxImpulse = false, hasTargetVelocity = false;

			numContacts = extractContacts(buffer, output, hasMaxImpulse, hasTargetVelocity, invMassScale0, invMassScale1,
				invInertiaScale0, invInertiaScale1, contactDesc.maxImpulse);

			contactDesc.contacts = buffer.contacts;
			contactDesc.numContacts = numContacts;
			contactDesc.disableStrongFriction = contactDesc.disableStrongFriction || hasTargetVelocity;
			contactDesc.hasMaxImpulse = hasMaxImpulse;
			contactDesc.mInvMassScales.linear0 *= invMassScale0;
			contactDesc.mInvMassScales.linear1 *= invMassScale1;
			contactDesc.mInvMassScales.angular0 *= invInertiaScale0;
			contactDesc.mInvMassScales.angular1 *= invInertiaScale1;
		}

		CorrelationBuffer& c = threadContext.mCorrelationBuffer;

		return createFinalizeSolverContactsStep(contactDesc, c, invDtF32, invTotalDt, bounceThresholdF32, frictionOffsetThreshold, correlationDistance, constraintAllocator);
	}

	PX_FORCE_INLINE PxU32 getConstraintLength(const PxTGSSolverConstraintDesc& desc)
	{
		return PxU32(desc.constraintLengthOver16 << 4);
	}

	PX_FORCE_INLINE PxU32 getWritebackLength(const PxTGSSolverConstraintDesc& desc)
	{
		return PxU32(desc.writeBackLengthOver4 << 2);
	}



	static FloatV solveDynamicContactsStep(SolverContactPointStep* contacts, const PxU32 nbContactPoints, const Vec3VArg contactNormal,
		const FloatVArg invMassA, const FloatVArg invMassB, Vec3V& linVel0_, Vec3V& angState0_,
		Vec3V& linVel1_, Vec3V& angState1_, PxF32* PX_RESTRICT forceBuffer,
		const Vec3V& angMotion0, const Vec3V& angMotion1,
		const Vec3V& linRelMotion, const FloatVArg maxPenBias,
		const FloatVArg angD0, const FloatVArg angD1, const FloatVArg minPen,
		const FloatVArg elapsedTime)
	{
		Vec3V linVel0 = linVel0_;
		Vec3V angState0 = angState0_;
		Vec3V linVel1 = linVel1_;
		Vec3V angState1 = angState1_;
		FloatV accumulatedNormalImpulse = FZero();

		const Vec3V delLinVel0 = V3Scale(contactNormal, invMassA);
		const Vec3V delLinVel1 = V3Scale(contactNormal, invMassB);

		const FloatV deltaV = V3Dot(linRelMotion, contactNormal);

		for (PxU32 i = 0; i<nbContactPoints; i++)
		{
			SolverContactPointStep& c = contacts[i];
			Ps::prefetchLine(&contacts[i], 128);


			const Vec3V raXnI = V3LoadA(c.raXnI);
			const Vec3V rbXnI = V3LoadA(c.rbXnI);

			const FloatV angDelta0 = V3Dot(angMotion0, raXnI);
			const FloatV angDelta1 = V3Dot(angMotion1, rbXnI);

			const FloatV deltaAng = FSub(angDelta0, angDelta1);

			const FloatV targetVel = FLoad(c.targetVelocity);

			const FloatV deltaBias = FSub(FAdd(deltaV, deltaAng), FMul(targetVel, elapsedTime));

			const FloatV biasCoefficient = FLoad(c.biasCoefficient);

			FloatV sep = FMax(minPen, FAdd(FLoad(c.separation), deltaBias));

			const FloatV bias = FMin(FNeg(maxPenBias), FMul(biasCoefficient, sep));

			

			const FloatV velMultiplier = FLoad(c.velMultiplier);

			const FloatV tVel = FAdd(bias, targetVel);

			const FloatV appliedForce = FLoad(forceBuffer[i]);

			const FloatV maxImpulse = FMax(); //KS - todo - hook up!

			//Compute the normal velocity of the constraint.
			const Vec3V v0 = V3MulAdd(linVel0, contactNormal, V3Mul(angState0, raXnI));
			const Vec3V v1 = V3MulAdd(linVel1, contactNormal, V3Mul(angState1, rbXnI));
			const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

			const FloatV _deltaF = FMax(FMul(FSub(tVel, normalVel), velMultiplier), FNeg(appliedForce));
			const FloatV _newForce = FAdd(appliedForce, _deltaF);
			const FloatV newForce = FMin(_newForce, maxImpulse);
			const FloatV deltaF = FSub(newForce, appliedForce);

			linVel0 = V3ScaleAdd(delLinVel0, deltaF, linVel0);
			linVel1 = V3NegScaleSub(delLinVel1, deltaF, linVel1);
			angState0 = V3ScaleAdd(raXnI, FMul(deltaF, angD0), angState0);
			angState1 = V3NegScaleSub(rbXnI, FMul(deltaF, angD1), angState1);

			FStore(newForce, &forceBuffer[i]);

			accumulatedNormalImpulse = FAdd(accumulatedNormalImpulse, newForce);
		}

		linVel0_ = linVel0;
		angState0_ = angState0;
		linVel1_ = linVel1;
		angState1_ = angState1;
		return accumulatedNormalImpulse;
	}


	void solveContact(const PxTGSSolverConstraintDesc& desc, bool doFriction, const PxReal minPenetration,
		const PxReal elapsedTimeF32)
	{
		PxTGSSolverBodyVel& b0 = *desc.bodyA;
		PxTGSSolverBodyVel& b1 = *desc.bodyB;

		const FloatV minPen = FLoad(minPenetration);

		Vec3V linVel0 = V3LoadA(b0.linearVelocity);
		Vec3V linVel1 = V3LoadA(b1.linearVelocity);
		Vec3V angState0 = V3LoadA(b0.angularVelocity);
		Vec3V angState1 = V3LoadA(b1.angularVelocity);

		const Vec3V angMotion0 = V3LoadA(b0.deltaAngDt);
		const Vec3V angMotion1 = V3LoadA(b1.deltaAngDt);

		const Vec3V linMotion0 = V3LoadA(b0.deltaLinDt);
		const Vec3V linMotion1 = V3LoadA(b1.deltaLinDt);

		const Vec3V relMotion = V3Sub(linMotion0, linMotion1);

		const FloatV elapsedTime = FLoad(elapsedTimeF32);

		const PxU8* PX_RESTRICT last = desc.constraint + getConstraintLength(desc);

		//hopefully pointer aliasing doesn't bite.
		PxU8* PX_RESTRICT currPtr = desc.constraint;

		while (currPtr < last)
		{
			SolverContactHeaderStep* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStep*>(currPtr);
			currPtr += sizeof(SolverContactHeaderStep);

			const PxU32 numNormalConstr = hdr->numNormalConstr;
			const PxU32	numFrictionConstr = hdr->numFrictionConstr;

			SolverContactPointStep* PX_RESTRICT contacts = reinterpret_cast<SolverContactPointStep*>(currPtr);
			Ps::prefetchLine(contacts);
			currPtr += numNormalConstr * sizeof(SolverContactPointStep);

			PxF32* forceBuffer = reinterpret_cast<PxF32*>(currPtr);
			currPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

			SolverContactFrictionStep* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionStep*>(currPtr);
			currPtr += numFrictionConstr * sizeof(SolverContactFrictionStep);

			const FloatV invMassA = FLoad(hdr->invMass0);
			const FloatV invMassB = FLoad(hdr->invMass1);

			const FloatV angDom0 = FLoad(hdr->angDom0);
			const FloatV angDom1 = FLoad(hdr->angDom1);

			//const FloatV sumMass = FAdd(invMassA, invMassB);

			const Vec3V contactNormal = V3LoadA(hdr->normal);

			const FloatV maxPenBias = FLoad(hdr->maxPenBias);

			const FloatV accumulatedNormalImpulse = solveDynamicContactsStep(contacts, numNormalConstr, contactNormal, invMassA, invMassB,
				linVel0, angState0, linVel1, angState1, forceBuffer,angMotion0, angMotion1, relMotion, maxPenBias, angDom0, angDom1, minPen,
				elapsedTime);

			if (numFrictionConstr && doFriction)
			{
				const FloatV staticFrictionCof = hdr->getStaticFriction();
				const FloatV dynamicFrictionCof = hdr->getDynamicFriction();
				const FloatV maxFrictionImpulse = FMul(staticFrictionCof, accumulatedNormalImpulse);
				const FloatV maxDynFrictionImpulse = FMul(dynamicFrictionCof, accumulatedNormalImpulse);
				const FloatV negMaxDynFrictionImpulse = FNeg(maxDynFrictionImpulse);

				BoolV broken = BFFFF();

				for (PxU32 i = 0; i<numFrictionConstr; i++)
				{
					SolverContactFrictionStep& f = frictions[i];
					Ps::prefetchLine(&frictions[i], 128);

					const FloatV frictionScale = FLoad(f.frictionScale);


					const Vec4V normalXYZ_ErrorW = f.normalXYZ_ErrorW;
					const Vec4V raXnI_targetVelW = f.raXnI_targetVelW;
					const Vec4V rbXnI_velMultiplierW = f.rbXnI_velMultiplierW;

					const Vec3V normal = Vec3V_From_Vec4V(normalXYZ_ErrorW);

					//const Vec4V raXn_biasScaleW = f.raXn_biasScaleW;
					//const Vec3V raXn = Vec3V_From_Vec4V(raXn_biasScaleW);
					//const Vec3V rbXn = Vec3V_From_Vec4V(f.rbXn_errorW);

					const FloatV initialError = V4GetW(normalXYZ_ErrorW);

					const FloatV biasScale = FLoad(f.biasScale);

					const Vec3V raXnI = Vec3V_From_Vec4V(raXnI_targetVelW);
					const Vec3V rbXnI = Vec3V_From_Vec4V(rbXnI_velMultiplierW);

					const FloatV appliedForce = FLoad(f.appliedForce);

					const FloatV targetVel = V4GetW(raXnI_targetVelW);

					FloatV deltaV = FAdd(FSub(V3Dot(raXnI, angMotion0), V3Dot(rbXnI, angMotion1)), V3Dot(normal, relMotion));
					
					deltaV = FSub(deltaV, FMul(targetVel, elapsedTime));

					const FloatV error = FAdd(initialError, deltaV);

					const FloatV bias = FMul(error, biasScale);
					
					const FloatV velMultiplier = V4GetW(rbXnI_velMultiplierW);

					const Vec3V delLinVel0 = V3Scale(normal, invMassA);
					const Vec3V delLinVel1 = V3Scale(normal, invMassB);

					const Vec3V v0 = V3MulAdd(linVel0, normal, V3Mul(angState0, raXnI));
					const Vec3V v1 = V3MulAdd(linVel1, normal, V3Mul(angState1, rbXnI));
					const FloatV normalVel = V3SumElems(V3Sub(v0, v1));



					// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
					const FloatV tmp1 = FNegScaleSub(FSub(bias, targetVel), velMultiplier, appliedForce);

					// Algorithm:
					// if abs(appliedForce + deltaF) > maxFrictionImpulse
					//    clamp newAppliedForce + deltaF to [-maxDynFrictionImpulse, maxDynFrictionImpulse]
					//      (i.e. clamp deltaF to [-maxDynFrictionImpulse-appliedForce, maxDynFrictionImpulse-appliedForce]
					//    set broken flag to true || broken flag

					// FloatV deltaF = FMul(FAdd(bias, normalVel), minusVelMultiplier);
					// FloatV potentialSumF = FAdd(appliedForce, deltaF);

					const FloatV totalImpulse = FNegScaleSub(normalVel, velMultiplier, tmp1);

					// On XBox this clamping code uses the vector simple pipe rather than vector float,
					// which eliminates a lot of stall cycles

					const BoolV clamp = FIsGrtr(FAbs(totalImpulse), FMul(frictionScale, maxFrictionImpulse));

					const FloatV totalClamped = FMin(FMul(frictionScale, maxDynFrictionImpulse), FMax(FMul(frictionScale, negMaxDynFrictionImpulse), totalImpulse));

					const FloatV newAppliedForce = FSel(clamp, totalClamped, totalImpulse);

					broken = BOr(broken, clamp);

					FloatV deltaF = FSub(newAppliedForce, appliedForce);

					// we could get rid of the stall here by calculating and clamping delta separately, but
					// the complexity isn't really worth it.

					linVel0 = V3ScaleAdd(delLinVel0, deltaF, linVel0);
					linVel1 = V3NegScaleSub(delLinVel1, deltaF, linVel1);
					angState0 = V3ScaleAdd(raXnI, FMul(deltaF, angDom0), angState0);
					angState1 = V3NegScaleSub(rbXnI, FMul(deltaF, angDom1), angState1);

					f.setAppliedForce(newAppliedForce);


				}
				Store_From_BoolV(broken, &hdr->broken);
			}

		}

		PX_ASSERT(b0.linearVelocity.isFinite());
		PX_ASSERT(b0.angularVelocity.isFinite());
		PX_ASSERT(b1.linearVelocity.isFinite());
		PX_ASSERT(b1.angularVelocity.isFinite());

		// Write back
		V3StoreA(linVel0, b0.linearVelocity);
		V3StoreA(linVel1, b1.linearVelocity);
		V3StoreA(angState0, b0.angularVelocity);
		V3StoreA(angState1, b1.angularVelocity);

		PX_ASSERT(b0.linearVelocity.isFinite());
		PX_ASSERT(b0.angularVelocity.isFinite());
		PX_ASSERT(b1.linearVelocity.isFinite());
		PX_ASSERT(b1.angularVelocity.isFinite());

		PX_ASSERT(currPtr == last);
	}

	void writeBackContact(const PxTGSSolverConstraintDesc& desc, SolverContext* cache)
	{
		PX_UNUSED(cache);
		PxReal normalForce = 0;

		PxU8* PX_RESTRICT cPtr = desc.constraint;
		PxReal* PX_RESTRICT vForceWriteback = reinterpret_cast<PxReal*>(desc.writeBack);
		PxU8* PX_RESTRICT last = desc.constraint + desc.constraintLengthOver16 * 16;

		bool forceThreshold = false;

		while (cPtr < last)
		{
			const SolverContactHeaderStep* PX_RESTRICT hdr = reinterpret_cast<const SolverContactHeaderStep*>(cPtr);
			cPtr += sizeof(SolverContactHeaderStep);

			forceThreshold = hdr->flags & SolverContactHeaderStep::eHAS_FORCE_THRESHOLDS;
			const PxU32 numNormalConstr = hdr->numNormalConstr;
			const PxU32	numFrictionConstr = hdr->numFrictionConstr;

			//if(cPtr < last)
			Ps::prefetchLine(cPtr, 256);
			Ps::prefetchLine(cPtr, 384);

			const PxU32 pointStride = hdr->type == DY_SC_TYPE_EXT_CONTACT ? sizeof(SolverContactPointStepExt)
				: sizeof(SolverContactPointStep);

			cPtr += pointStride * numNormalConstr;
			PxF32* forceBuffer = reinterpret_cast<PxF32*>(cPtr);
			cPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

			if (vForceWriteback != NULL)
			{
				for (PxU32 i = 0; i<numNormalConstr; i++)
				{
					PxReal appliedForce = forceBuffer[i];
					*vForceWriteback++ = appliedForce;
					normalForce += appliedForce;
				}
			}

			const PxU32 frictionStride = hdr->type == DY_SC_TYPE_EXT_CONTACT ? sizeof(SolverContactFrictionStepExt)
				: sizeof(SolverContactFrictionStep);

			if (hdr->broken && hdr->frictionBrokenWritebackByte != NULL)
			{
				*hdr->frictionBrokenWritebackByte = 1;
			}

			cPtr += frictionStride * numFrictionConstr;

		}
		PX_ASSERT(cPtr == last);

		PX_UNUSED(forceThreshold);

#if 0
		if (cache && forceThreshold && desc.linkIndexA == PxSolverConstraintDesc::NO_LINK && desc.linkIndexB == PxSolverConstraintDesc::NO_LINK &&
		normalForce != 0 && (desc.bodyA->reportThreshold < PX_MAX_REAL || desc.bodyB->reportThreshold < PX_MAX_REAL))
		{
			ThresholdStreamElement elt;
			elt.normalForce = normalForce;
			elt.threshold = PxMin<float>(desc.bodyA->reportThreshold, desc.bodyB->reportThreshold);
			elt.nodeIndexA = desc.bodyA->nodeIndex;
			elt.nodeIndexB = desc.bodyB->nodeIndex;
			elt.shapeInteraction = reinterpret_cast<const SolverContactHeader*>(desc.constraint)->shapeInteraction;
			Ps::order(elt.nodeIndexA, elt.nodeIndexB);
			PX_ASSERT(elt.nodeIndexA < elt.nodeIndexB);
			PX_ASSERT(cache->mThresholdStreamIndex<cache->mThresholdStreamLength);
			cache->mThresholdStream[cache->mThresholdStreamIndex++] = elt;
		}
#endif
	}





	PX_FORCE_INLINE Vec3V V3FromV4(Vec4V x)			{ return Vec3V_From_Vec4V(x); }
	PX_FORCE_INLINE Vec3V V3FromV4Unsafe(Vec4V x)	{ return Vec3V_From_Vec4V_WUndefined(x); }
	PX_FORCE_INLINE Vec4V V4FromV3(Vec3V x)			{ return Vec4V_From_Vec3V(x); }
	//PX_FORCE_INLINE Vec4V V4ClearW(Vec4V x)			{ return V4SetW(x, FZero()); }


	void preprocessRows(Px1DConstraint** sorted,
		Px1DConstraint* rows,
		PxVec4* angSqrtInvInertia0,
		PxVec4* angSqrtInvInertia1,
		PxU32 rowCount,
		const PxMat33& sqrtInvInertia0F32,
		const PxMat33& sqrtInvInertia1F32,
		const PxReal invMass0,
		const PxReal invMass1,
		const PxConstraintInvMassScale& ims,
		bool disablePreprocessing,
		bool diagonalizeDrive,
		bool preprocessLinear = true);



void setSolverConstantsStep(PxReal& error,
	PxReal& biasScale,
	PxReal& targetVel,
	PxReal& maxBias,
	PxReal& velMultiplier,
	PxReal& impulseMultiplier,
	PxReal& rcpResponse,
	const Px1DConstraint& c,
	PxReal normalVel,
	PxReal unitResponse,
	PxReal minRowResponse,
	PxReal erp,
	PxReal dt,
	PxReal totalDt,
	PxReal biasClamp,
	PxReal recipdt,
	PxReal recipTotalDt)
{
	PX_UNUSED(dt);
	PX_UNUSED(totalDt);
	PX_ASSERT(PxIsFinite(unitResponse));
	PxReal recipResponse = unitResponse <= minRowResponse ? 0 : 1.0f / unitResponse;
	PX_ASSERT(recipResponse < 1e5f);
	PxReal geomError = c.geometricError;

	rcpResponse = recipResponse;

	

	if (c.flags & Px1DConstraintFlag::eSPRING)
	{
		error = geomError;

		PxReal a = dt *  (dt*c.mods.spring.stiffness + c.mods.spring.damping);
		PxReal b = dt*(c.mods.spring.damping * c.velocityTarget /*- c.mods.spring.stiffness * geomError*/);
		maxBias = biasClamp;

		if (c.flags & Px1DConstraintFlag::eACCELERATION_SPRING)
		{
			PxReal x = 1.0f / (1.0f + a);
			targetVel = x * b;
			velMultiplier = -x * a;
			biasScale = -x * c.mods.spring.stiffness*dt;
			//KS - impulse multiplier for TGS solver is 1 because we consider each pass to be a separate step, with the solver computing the impulse proportional to the step dt.
			//Alternatively, we could consider the spring over totalDt and then use an impulseMultiplier of 1-x, but this would require the bias to be constant to allow the constraint
			//to produce stiff results if the number of iterations was large
			impulseMultiplier = 1.0f; 
		}
		else
		{
			PxReal x = 1.0f / (1.0f + a*unitResponse);
			targetVel = x * b*unitResponse;
			velMultiplier = -x*a*unitResponse;
			//KS - impulse multiplier for TGS solver is 1 because we consider each pass to be a separate step, with the solver computing the impulse proportional to the step dt.
			//Alternatively, we could consider the spring over totalDt and then use an impulseMultiplier of 1-x, but this would require the bias to be constant to allow the constraint
			//to produce stiff results if the number of iterations was large
			biasScale = -x * c.mods.spring.stiffness*unitResponse*dt;
			impulseMultiplier = 1.0f;
		}

	}
	else
	{
		velMultiplier = -1.f;// *recipResponse;
		impulseMultiplier = 1.0f;

		if (c.flags & Px1DConstraintFlag::eRESTITUTION && -normalVel>c.mods.bounce.velocityThreshold)
		{
			error = 0.f;
			biasScale = 0.f;

			targetVel = c.mods.bounce.restitution*-normalVel;
			maxBias = 0.f;
		}
		else
		{
			
			biasScale = -recipdt*erp;// *recipResponse;
			if (c.flags & Px1DConstraintFlag::eDRIVE_ROW)
			{
				error = 0.f;
				targetVel = c.velocityTarget - geomError *recipTotalDt;
			}
			else
			{
				error = geomError;
				//KS - if there is a velocity target, then we cannot also have bias otherwise the two compete against each-other.
				//Therefore, we set the velocity target 
				targetVel = c.velocityTarget;// *recipResponse;
			}

			maxBias = biasClamp;// *recipResponse;

			/*PxReal errorBias = PxClamp(geomError*erp*recipdt, -biasClamp, biasClamp);

			constant = (c.velocityTarget - errorBias) * recipResponse;*/

		}
	}
}





PxU32 setupSolverConstraintStep(
	const PxTGSSolverConstraintPrepDesc& prepDesc,
	PxConstraintAllocator& allocator,
	const PxReal dt, const PxReal totalDt, const PxReal invdt, const PxReal invTotalDt,
	const PxU32 nbSubsteps,
	const PxReal lengthScale)
{
	PX_UNUSED(nbSubsteps);
	//static const PxU32 MAX_CONSTRAINT_ROWS = 12;

	if (prepDesc.numRows == 0)
	{
		prepDesc.desc->constraint = NULL;
		prepDesc.desc->writeBack = NULL;
		prepDesc.desc->constraintLengthOver16 = 0;
		prepDesc.desc->writeBackLengthOver4 = 0;
		return 0;
	}

	PxTGSSolverConstraintDesc& desc = *prepDesc.desc;

	const bool isExtended = desc.linkIndexA != PxTGSSolverConstraintDesc::NO_LINK
		|| desc.linkIndexB != PxTGSSolverConstraintDesc::NO_LINK;

	const bool isKinematic0 = desc.linkIndexA == PxTGSSolverConstraintDesc::NO_LINK && 
		desc.bodyA->isKinematic;

	const bool isKinematic1 = desc.linkIndexB == PxTGSSolverConstraintDesc::NO_LINK &&
		desc.bodyB->isKinematic;

	PxU32 stride = isExtended ? sizeof(SolverConstraint1DExtStep) : sizeof(SolverConstraint1DStep);
	const PxU32 constraintLength = sizeof(SolverConstraint1DHeaderStep) + stride * prepDesc.numRows;

	//KS - +16 is for the constraint progress counter, which needs to be the last element in the constraint (so that we
	//know SPU DMAs have completed)
	PxU8* ptr = allocator.reserveConstraintData(constraintLength + 16u);
	if (NULL == ptr || (reinterpret_cast<PxU8*>(-1)) == ptr)
	{
		if (NULL == ptr)
		{
			PX_WARN_ONCE(
				"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
				"Either accept joints detaching/exploding or increase buffer size allocated for constraint prep by increasing PxSceneDesc::maxNbContactDataBlocks.");
			return 0;
		}
		else
		{
			PX_WARN_ONCE(
				"Attempting to allocate more than 16K of constraint data. "
				"Either accept joints detaching/exploding or simplify constraints.");
			ptr = NULL;
			return 0;
		}
	}
	desc.constraint = ptr;

	//setConstraintLength(desc, constraintLength);
	PX_ASSERT((constraintLength & 0xf) == 0);
	desc.constraintLengthOver16 = Ps::to16(constraintLength / 16);

	desc.writeBack = prepDesc.writeback;
	desc.writeBackLengthOver4 = Ps::to16(sizeof(ConstraintWriteback)/4);

	memset(desc.constraint, 0, constraintLength);

	SolverConstraint1DHeaderStep* header = reinterpret_cast<SolverConstraint1DHeaderStep*>(desc.constraint);
	PxU8* constraints = desc.constraint + sizeof(SolverConstraint1DHeaderStep);
	init(*header, Ps::to8(prepDesc.numRows), isExtended, prepDesc.mInvMassScales);
	header->body0WorldOffset = prepDesc.body0WorldOffset;
	header->linBreakImpulse = prepDesc.linBreakForce * totalDt;
	header->angBreakImpulse = prepDesc.angBreakForce * totalDt;
	header->breakable = PxU8((prepDesc.linBreakForce != PX_MAX_F32) || (prepDesc.angBreakForce != PX_MAX_F32));
	header->invMass0D0 = prepDesc.bodyData0->invMass * prepDesc.mInvMassScales.linear0;
	header->invMass1D1 = prepDesc.bodyData1->invMass * prepDesc.mInvMassScales.linear1;

	header->rAWorld = prepDesc.cA2w - prepDesc.bodyFrame0.p;
	header->rBWorld = prepDesc.cB2w - prepDesc.bodyFrame1.p;

	Px1DConstraint* sorted[MAX_CONSTRAINT_ROWS];

	PX_ALIGN(16, PxVec4) angSqrtInvInertia0[MAX_CONSTRAINT_ROWS];
	PX_ALIGN(16, PxVec4) angSqrtInvInertia1[MAX_CONSTRAINT_ROWS];

	for (PxU32 i = 0; i < prepDesc.numRows; ++i)
	{
		if (prepDesc.rows[i].flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT)
		{
			if (prepDesc.rows[i].solveHint == PxConstraintSolveHint::eEQUALITY)
				prepDesc.rows[i].solveHint = PxConstraintSolveHint::eROTATIONAL_EQUALITY;
			else if (prepDesc.rows[i].solveHint == PxConstraintSolveHint::eINEQUALITY)
				prepDesc.rows[i].solveHint = PxConstraintSolveHint::eROTATIONAL_INEQUALITY;
		}
	}

	preprocessRows(sorted, prepDesc.rows, angSqrtInvInertia0, angSqrtInvInertia1, prepDesc.numRows,
		prepDesc.body0TxI->sqrtInvInertia, prepDesc.body1TxI->sqrtInvInertia, prepDesc.bodyData0->invMass, prepDesc.bodyData1->invMass,
		prepDesc.mInvMassScales, isExtended || prepDesc.disablePreprocessing, prepDesc.improvedSlerp, false);

	PxReal erp = 1.f;
	PxReal linearErp = 1.f;

	const PxReal recipDt = invdt;

	PxU32 orthoCount = 0;
	for (PxU32 i = 0; i<prepDesc.numRows; i++)
	{
		Ps::prefetchLine(constraints, 128);
		SolverConstraint1DStep &s = *reinterpret_cast<SolverConstraint1DStep *>(constraints);
		Px1DConstraint& c = *sorted[i];

		PxReal driveScale = c.flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && prepDesc.driveLimitsAreForces ? PxMin(totalDt, 1.0f) : 1.0f;

		PxReal unitResponse;
		PxReal normalVel = 0.0f;

		PxReal angSpeedLimit = 100.f;
		PxReal linSpeedLimit = 1000.f;

		PxReal vel0, vel1;

		if (!isExtended)
		{		
			const PxVec3 angSqrtInvInertia0V3(angSqrtInvInertia0[i].x, angSqrtInvInertia0[i].y, angSqrtInvInertia0[i].z);
			const PxVec3 angSqrtInvInertia1V3(angSqrtInvInertia1[i].x, angSqrtInvInertia1[i].y, angSqrtInvInertia1[i].z);
			init(s, c.linear0, c.linear1, c.angular0, c.angular1, c.minImpulse * driveScale, c.maxImpulse * driveScale);

			/*unitResponse = prepDesc.body0->getResponse(c.linear0, c.angular0, s.ang0, prepDesc.mInvMassScales.linear0, prepDesc.mInvMassScales.angular0)
				+ prepDesc.body1->getResponse(-c.linear1, -c.angular1, s.ang1, prepDesc.mInvMassScales.linear1, prepDesc.mInvMassScales.angular1);*/

			const PxReal linSumMass = s.lin0.magnitudeSquared() * prepDesc.bodyData0->invMass * prepDesc.mInvMassScales.linear0 + s.lin1.magnitudeSquared() * prepDesc.bodyData1->invMass * prepDesc.mInvMassScales.linear1;

			PxReal resp0 = angSqrtInvInertia0V3.magnitudeSquared() * prepDesc.mInvMassScales.angular0;
			PxReal resp1 = angSqrtInvInertia1V3.magnitudeSquared() * prepDesc.mInvMassScales.angular1;
			unitResponse = resp0 + resp1 + linSumMass;

			//s.recipResponseOrLinearSumMass = linSumMass;

			vel0 = prepDesc.bodyData0->projectVelocity(s.lin0, s.ang0);
			vel1 = prepDesc.bodyData1->projectVelocity(s.lin1, s.ang1);

			normalVel = vel0 - vel1;

			//if (c.solveHint & PxConstraintSolveHint::eEQUALITY)
			if(!(c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT))
			{
				s.ang0 = PxVec3(0.f);
				s.ang1 = PxVec3(0.f);
				s.angularErrorScale = 0.f;
			}
			
		}
		else
		{
			linearErp = 0.7f;
			erp = 0.7f;
			//angSpeedLimit = 25.f;
			linSpeedLimit = 200.f;
			const SolverExtBodyStep eb0(reinterpret_cast<const void*>(prepDesc.body0), prepDesc.body0TxI, prepDesc.bodyData0, desc.linkIndexA);
			const SolverExtBodyStep eb1(reinterpret_cast<const void*>(prepDesc.body1), prepDesc.body1TxI, prepDesc.bodyData1, desc.linkIndexB);
			const Cm::SpatialVector resp0 = createImpulseResponseVector(c.linear0, c.angular0, eb0);
			const Cm::SpatialVector resp1 = createImpulseResponseVector(-c.linear1, -c.angular1, eb1);

			init(s, resp0.linear, -resp1.linear, resp0.angular, -resp1.angular, c.minImpulse * driveScale, c.maxImpulse * driveScale);
			SolverConstraint1DExtStep& e = static_cast<SolverConstraint1DExtStep&>(s);

			Cm::SpatialVector& delta0 = unsimdRef(e.deltaVA);
			Cm::SpatialVector& delta1 = unsimdRef(e.deltaVB);

			unitResponse = getImpulseResponse(eb0, resp0, delta0, prepDesc.mInvMassScales.linear0, prepDesc.mInvMassScales.angular0,
				eb1, resp1, delta1, prepDesc.mInvMassScales.linear1, prepDesc.mInvMassScales.angular1, false);

			if (eb0.mLinkIndex != PxSolverConstraintDesc::NO_LINK &&
				eb1.mLinkIndex != PxSolverConstraintDesc::NO_LINK)
			{
				PxReal lr0 = delta0.linear.dot(resp0.linear);
				PxReal ar0 = delta0.angular.dot(resp0.angular) * safeRecip(resp0.angular.magnitude());
				PxReal lr1 = delta1.linear.dot(resp1.linear);
				PxReal ar1 = delta1.angular.dot(resp1.angular)* safeRecip(resp1.angular.magnitude());

				Cm::SpatialVector rem0(delta0.linear - resp0.linear*lr0, delta0.angular - resp0.angular*ar0);
				Cm::SpatialVector rem1(delta1.linear - resp1.linear*lr1, delta1.angular - resp1.angular*ar1);
				

				PxReal rem = (rem0 - rem1).magnitude();

				unitResponse += rem;
			}


		
			{
				vel0 = eb0.projectVelocity(s.lin0, s.ang0);
				vel1 = eb1.projectVelocity(s.lin1, s.ang1);

				normalVel = vel0 - vel1;
			}

			if (!(c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT))
			{
				s.angularErrorScale = 0.f;
			}
		}

		PxReal recipResponse = 0.f;

		setSolverConstantsStep(s.error, s.biasScale, s.velTarget, s.maxBias, s.velMultiplier, s.impulseMultiplier, recipResponse, c,
			normalVel, unitResponse, isExtended ? 1e-5f : prepDesc.minResponseThreshold, (c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT ? erp : linearErp), dt, totalDt,
			c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT ? angSpeedLimit : linSpeedLimit*lengthScale, recipDt, invTotalDt);

		s.recipResponse = recipResponse;

		if(isKinematic0)
			s.velTarget -= vel0;
		if(isKinematic1)
			s.velMultiplier += vel1;

		

		if (c.flags & Px1DConstraintFlag::eOUTPUT_FORCE)
			s.flags |= DY_SC_FLAG_OUTPUT_FORCE;

		if ((c.flags & Px1DConstraintFlag::eKEEPBIAS))
			s.flags |= DY_SC_FLAG_KEEP_BIAS;
		if (c.solveHint & 1)
			s.flags |= DY_SC_FLAG_INEQUALITY;

		if (!(isExtended || prepDesc.disablePreprocessing))
		{
			//KS - the code that orthogonalizes constraints on-the-fly only works if the linear and angular constraints have already been pre-orthogonalized
			if (c.solveHint == PxConstraintSolveHint::eROTATIONAL_EQUALITY)
			{
				s.flags |= DY_SC_FLAG_ROT_EQ;

				PX_ASSERT(orthoCount < 3);

				/*angOrtho0[orthoCount] = PxVec3(angSqrtInvInertia0[i].x, angSqrtInvInertia0[i].y, angSqrtInvInertia0[i].z);
				angOrtho1[orthoCount] = PxVec3(angSqrtInvInertia1[i].x, angSqrtInvInertia1[i].y, angSqrtInvInertia1[i].z);
				recipResponses[orthoCount] = recipResponse;*/

				header->angOrthoAxis0_recipResponseW[orthoCount] =
					PxVec4(angSqrtInvInertia0[i].x*prepDesc.mInvMassScales.angular0, angSqrtInvInertia0[i].y*prepDesc.mInvMassScales.angular0, angSqrtInvInertia0[i].z*prepDesc.mInvMassScales.angular0, recipResponse);
				header->angOrthoAxis1_Error[orthoCount].x = angSqrtInvInertia1[i].x*prepDesc.mInvMassScales.angular1;
				header->angOrthoAxis1_Error[orthoCount].y = angSqrtInvInertia1[i].y*prepDesc.mInvMassScales.angular1;
				header->angOrthoAxis1_Error[orthoCount].z = angSqrtInvInertia1[i].z*prepDesc.mInvMassScales.angular1;
				header->angOrthoAxis1_Error[orthoCount].w = c.geometricError;

				orthoCount++;
			}

			else if (c.solveHint & PxConstraintSolveHint::eEQUALITY)
				s.flags |= DY_SC_FLAG_ORTHO_TARGET;
		}
		



		constraints += stride;
	}


	//KS - Set the solve count at the end to 0 
	*(reinterpret_cast<PxU32*>(constraints)) = 0;
	*(reinterpret_cast<PxU32*>(constraints + 4)) = 0;
	PX_ASSERT(desc.constraint + getConstraintLength(desc) == constraints);
	return prepDesc.numRows;
}

PxU32 SetupSolverConstraintStep(SolverConstraintShaderPrepDesc& shaderDesc,
	PxTGSSolverConstraintPrepDesc& prepDesc,
	PxConstraintAllocator& allocator,
	const PxReal dt, const PxReal totalDt, const PxReal invdt, const PxReal invTotalDt, const PxU32 nbSubsteps,
	const PxReal lengthScale)
{
	// LL shouldn't see broken constraints

	PX_ASSERT(!(reinterpret_cast<ConstraintWriteback*>(prepDesc.writeback)->broken));

	prepDesc.desc->constraintLengthOver16 = 0;
	//setConstraintLength(*prepDesc.desc, 0);

	if (!shaderDesc.solverPrep)
		return 0;

	//PxU32 numAxisConstraints = 0;

	Px1DConstraint rows[MAX_CONSTRAINT_ROWS];

	// This is necessary so that there will be sensible defaults and shaders will
	// continue to work (albeit with a recompile) if the row format changes.
	// It's a bit inefficient because it fills in all constraint rows even if there
	// is only going to be one generated. A way around this would be for the shader to
	// specify the maximum number of rows it needs, or it could call a subroutine to
	// prep the row before it starts filling it it.

	PxMemZero(rows, sizeof(Px1DConstraint)*MAX_CONSTRAINT_ROWS);

	for (PxU32 i = 0; i<MAX_CONSTRAINT_ROWS; i++)
	{
		Px1DConstraint& c = rows[i];
		//Px1DConstraintInit(c);
		c.minImpulse = -PX_MAX_REAL;
		c.maxImpulse = PX_MAX_REAL;
	}

	prepDesc.mInvMassScales.linear0 = prepDesc.mInvMassScales.linear1 = prepDesc.mInvMassScales.angular0 = prepDesc.mInvMassScales.angular1 = 1.f;

	PxVec3 body0WorldOffset(0.f);
	PxU32 constraintCount = (*shaderDesc.solverPrep)(rows,
		body0WorldOffset,
		MAX_CONSTRAINT_ROWS,
		prepDesc.mInvMassScales,
		shaderDesc.constantBlock,
		prepDesc.bodyFrame0, prepDesc.bodyFrame1,
		prepDesc.extendedLimits, prepDesc.cA2w, prepDesc.cB2w);

	prepDesc.rows = rows;
	prepDesc.numRows = constraintCount;

	prepDesc.body0WorldOffset = body0WorldOffset;

	if (prepDesc.bodyState0 != PxSolverContactDesc::eARTICULATION && prepDesc.body0->isKinematic)
		prepDesc.mInvMassScales.angular0 = 0.f;
	if (prepDesc.bodyState1 != PxSolverContactDesc::eARTICULATION && prepDesc.body1->isKinematic)
		prepDesc.mInvMassScales.angular1 = 0.f;


	return setupSolverConstraintStep(prepDesc, allocator, dt, totalDt, invdt, invTotalDt, nbSubsteps, lengthScale);
}

//Port of scalar implementation to SIMD maths with some interleaving of instructions
void solveExt1DStep(const PxTGSSolverConstraintDesc& desc, const PxReal elapsedTimeF32, SolverContext& cache,
	const PxTGSSolverBodyTxInertia* const txInertias)
{
	PxU8* PX_RESTRICT bPtr = desc.constraint;
	//PxU32 length = desc.constraintLength;

	const SolverConstraint1DHeaderStep* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeaderStep*>(bPtr);
	SolverConstraint1DExtStep* PX_RESTRICT base = reinterpret_cast<SolverConstraint1DExtStep*>(bPtr + sizeof(SolverConstraint1DHeaderStep));

	const FloatV elapsedTime = FLoad(elapsedTimeF32);

	Vec3V linVel0, angVel0, linVel1, angVel1;
	Vec3V linMotion0, angMotion0, linMotion1, angMotion1;

	QuatV rotA, rotB;

	if (desc.articulationA == desc.articulationB)
	{
		Cm::SpatialVectorV v0, v1;
		desc.articulationA->pxcFsGetVelocities(desc.linkIndexA, desc.linkIndexB, v0, v1);
		linVel0 = v0.linear;
		angVel0 = v0.angular;
		linVel1 = v1.linear;
		angVel1 = v1.angular;

		Cm::SpatialVectorV motionV0 = PxcFsGetMotionVector(*desc.articulationA, desc.linkIndexA);
		Cm::SpatialVectorV motionV1 = PxcFsGetMotionVector(*desc.articulationB, desc.linkIndexB);

		linMotion0 = motionV0.linear;
		angMotion0 = motionV0.angular;
		linMotion1 = motionV1.linear;
		angMotion1 = motionV1.angular;

		rotA = Ps::aos::QuatVLoadU(&desc.articulationA->getDeltaQ(desc.linkIndexA).x);
		rotB = Ps::aos::QuatVLoadU(&desc.articulationB->getDeltaQ(desc.linkIndexB).x);
	}
	else
	{

		if (desc.linkIndexA == PxSolverConstraintDesc::NO_LINK)
		{
			linVel0 = V3LoadA(desc.bodyA->linearVelocity);
			angVel0 = V3LoadA(desc.bodyA->angularVelocity);
			linMotion0 = V3LoadA(desc.bodyA->deltaLinDt);
			angMotion0 = V3LoadA(desc.bodyA->deltaAngDt);
			rotA = Ps::aos::QuatVLoadA(&txInertias[desc.bodyAIdx].deltaBody2World.q.x);
		}
		else
		{
			Cm::SpatialVectorV v = desc.articulationA->pxcFsGetVelocity(desc.linkIndexA);
			rotA = Ps::aos::QuatVLoadU(&desc.articulationA->getDeltaQ(desc.linkIndexA).x);
			Cm::SpatialVectorV motionV = PxcFsGetMotionVector(*desc.articulationA, desc.linkIndexA);
			linVel0 = v.linear;
			angVel0 = v.angular;

			linMotion0 = motionV.linear;
			angMotion0 = motionV.angular;
		}

		if (desc.linkIndexB == PxSolverConstraintDesc::NO_LINK)
		{
			linVel1 = V3LoadA(desc.bodyB->linearVelocity);
			angVel1 = V3LoadA(desc.bodyB->angularVelocity);
			linMotion1 = V3LoadA(desc.bodyB->deltaLinDt);
			angMotion1 = V3LoadA(desc.bodyB->deltaAngDt);
			rotB = Ps::aos::QuatVLoadA(&txInertias[desc.bodyBIdx].deltaBody2World.q.x);
		}
		else
		{
			Cm::SpatialVectorV v = desc.articulationB->pxcFsGetVelocity(desc.linkIndexB);
			rotB = Ps::aos::QuatVLoadU(&desc.articulationB->getDeltaQ(desc.linkIndexB).x);
			Cm::SpatialVectorV motionV = PxcFsGetMotionVector(*desc.articulationB, desc.linkIndexB);
			linVel1 = v.linear;
			angVel1 = v.angular;

			linMotion1 = motionV.linear;
			angMotion1 = motionV.angular;
		}
	}

	/*PX_ASSERT(FAllGrtr(FLoad(40.f * 40.f), V3Dot(linVel0, linVel0)));
	PX_ASSERT(FAllGrtr(FLoad(40.f * 40.f), V3Dot(linVel1, linVel1)));
	PX_ASSERT(FAllGrtr(FLoad(20.f * 20.f), V3Dot(angVel0, angVel0)));
	PX_ASSERT(FAllGrtr(FLoad(20.f * 20.f), V3Dot(angVel1, angVel1)));*/

	const Vec3V raPrev = V3LoadA(header->rAWorld);
	const Vec3V rbPrev = V3LoadA(header->rBWorld);

	const Vec3V ra = QuatRotate(rotA, V3LoadA(header->rAWorld));
	const Vec3V rb = QuatRotate(rotB, V3LoadA(header->rBWorld));

	const Vec3V raMotion = V3Sub(V3Add(ra, linMotion0), raPrev);
	const Vec3V rbMotion = V3Sub(V3Add(rb, linMotion1), rbPrev);


	Vec3V li0 = V3Zero(), li1 = V3Zero(), ai0 = V3Zero(), ai1 = V3Zero();

	for (PxU32 i = 0; i<header->count; ++i, base++)
	{
		Ps::prefetchLine(base + 1);

		SolverConstraint1DExtStep& c = *base;

		const Vec3V clinVel0 = V3LoadA(c.lin0);
		const Vec3V clinVel1 = V3LoadA(c.lin1);

		const Vec3V cangVel0 = V3LoadA(c.ang0);
		const Vec3V cangVel1 = V3LoadA(c.ang1);

		const FloatV recipResponse = FLoad(c.recipResponse);

		const FloatV targetVel = FLoad(c.velTarget);

		const FloatV deltaAng = FMul(FSub(V3Dot(cangVel0, angMotion0), V3Dot(cangVel1, angMotion1)), FLoad(c.angularErrorScale));
		const FloatV error = FNegScaleSub(targetVel, elapsedTime, FAdd(FAdd(FLoad(c.error), FSub(V3Dot(raMotion, clinVel0), V3Dot(rbMotion, clinVel1))), deltaAng));

		const FloatV biasScale = FLoad(c.biasScale);
		const FloatV maxBias = FLoad(c.maxBias);
		
		const FloatV vMul = FMul(recipResponse, FLoad(c.velMultiplier));
		const FloatV iMul = FLoad(c.impulseMultiplier);
		const FloatV appliedForce = FLoad(c.appliedForce);

		const FloatV unclampedBias = FMul(error, biasScale);
		const FloatV minBias = c.flags & DY_SC_FLAG_INEQUALITY ? FNeg(FMax()) : FNeg(maxBias);
		const FloatV bias = FClamp(unclampedBias, minBias, maxBias);

		const FloatV constant = FMul(recipResponse, FAdd(bias, targetVel));

		const FloatV maxImpulse = FLoad(c.maxImpulse);
		const FloatV minImpulse = FLoad(c.minImpulse);

		const Vec3V v0 = V3MulAdd(linVel0, clinVel0, V3Mul(angVel0, cangVel0));
		const Vec3V v1 = V3MulAdd(linVel1, clinVel1, V3Mul(angVel1, cangVel1));
		const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

		const FloatV iM = FMul(iMul, appliedForce);

		const FloatV unclampedForce = FAdd(iM, FScaleAdd(vMul, normalVel, constant));
		const FloatV clampedForce = FMin(maxImpulse, (FMax(minImpulse, unclampedForce)));
		const FloatV deltaF = FSub(clampedForce, appliedForce);

		FStore(clampedForce, &c.appliedForce);

		//PX_ASSERT(FAllGrtr(FLoad(1000.f), FAbs(deltaF)));


		FStore(clampedForce, &base->appliedForce);
		li0 = V3ScaleAdd(clinVel0, deltaF, li0);	ai0 = V3ScaleAdd(cangVel0, deltaF, ai0);
		li1 = V3ScaleAdd(clinVel1, deltaF, li1);	ai1 = V3ScaleAdd(cangVel1, deltaF, ai1);

		linVel0 = V3ScaleAdd(base->deltaVA.linear, deltaF, linVel0); 		angVel0 = V3ScaleAdd(base->deltaVA.angular, deltaF, angVel0);
		linVel1 = V3ScaleAdd(base->deltaVB.linear, deltaF, linVel1); 		angVel1 = V3ScaleAdd(base->deltaVB.angular, deltaF, angVel1);

		/*PX_ASSERT(FAllGrtr(FLoad(40.f * 40.f), V3Dot(linVel0, linVel0)));
		PX_ASSERT(FAllGrtr(FLoad(40.f * 40.f), V3Dot(linVel1, linVel1)));
		PX_ASSERT(FAllGrtr(FLoad(20.f * 20.f), V3Dot(angVel0, angVel0)));
		PX_ASSERT(FAllGrtr(FLoad(20.f * 20.f), V3Dot(angVel1, angVel1)));*/
	}

	if (desc.articulationA == desc.articulationB)
	{
		desc.articulationA->pxcFsApplyImpulses(desc.linkIndexA, V3Scale(li0, FLoad(header->linearInvMassScale0)),
			V3Scale(ai0, FLoad(header->angularInvMassScale0)), desc.linkIndexB, V3Scale(li1, FLoad(header->linearInvMassScale1)),
			V3Scale(ai1, FLoad(header->angularInvMassScale1)), cache.Z, cache.deltaV);
	}
	else
	{

		if (desc.linkIndexA == PxSolverConstraintDesc::NO_LINK)
		{
			V3StoreA(linVel0, desc.bodyA->linearVelocity);
			V3StoreA(angVel0, desc.bodyA->angularVelocity);
		}
		else
		{
			desc.articulationA->pxcFsApplyImpulse(desc.linkIndexA, V3Scale(li0, FLoad(header->linearInvMassScale0)),
				V3Scale(ai0, FLoad(header->angularInvMassScale0)), cache.Z, cache.deltaV);
		}

		if (desc.linkIndexB == PxSolverConstraintDesc::NO_LINK)
		{
			V3StoreA(linVel1, desc.bodyB->linearVelocity);
			V3StoreA(angVel1, desc.bodyB->angularVelocity);
		}
		else
		{
			desc.articulationB->pxcFsApplyImpulse(desc.linkIndexB, V3Scale(li1, FLoad(header->linearInvMassScale1)),
				V3Scale(ai1, FLoad(header->angularInvMassScale1)), cache.Z, cache.deltaV);
		}
	}
}




//Port of scalar implementation to SIMD maths with some interleaving of instructions
void solve1DStep(const PxTGSSolverConstraintDesc& desc, const PxTGSSolverBodyTxInertia* const txInertias, const PxReal elapsedTime)
{
	PxU8* PX_RESTRICT bPtr = desc.constraint;
	if (bPtr == NULL)
		return;

	PxTGSSolverBodyVel& b0 = *desc.bodyA;
	PxTGSSolverBodyVel& b1 = *desc.bodyB;

	const FloatV elapsed = FLoad(elapsedTime);


	const PxTGSSolverBodyTxInertia& txI0 = txInertias[desc.bodyAIdx];
	const PxTGSSolverBodyTxInertia& txI1 = txInertias[desc.bodyBIdx];	

	const SolverConstraint1DHeaderStep* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeaderStep*>(bPtr);
	SolverConstraint1DStep* PX_RESTRICT base = reinterpret_cast<SolverConstraint1DStep*>(bPtr + sizeof(SolverConstraint1DHeaderStep));

	Vec3V linVel0 = V3LoadA(b0.linearVelocity);
	Vec3V linVel1 = V3LoadA(b1.linearVelocity);
	Vec3V angState0 = V3LoadA(b0.angularVelocity);
	Vec3V angState1 = V3LoadA(b1.angularVelocity);

	Mat33V sqrtInvInertia0 = Mat33V(V3LoadU(txI0.sqrtInvInertia.column0), V3LoadU(txI0.sqrtInvInertia.column1), V3LoadU(txI0.sqrtInvInertia.column2));
	Mat33V sqrtInvInertia1 = Mat33V(V3LoadU(txI1.sqrtInvInertia.column0), V3LoadU(txI1.sqrtInvInertia.column1), V3LoadU(txI1.sqrtInvInertia.column2));


	const FloatV invMass0 = FLoad(header->invMass0D0);
	const FloatV invMass1 = FLoad(header->invMass1D1);
	const FloatV invInertiaScale0 = FLoad(header->angularInvMassScale0);
	const FloatV invInertiaScale1 = FLoad(header->angularInvMassScale1);

	
	const QuatV deltaRotA = Ps::aos::QuatVLoadA(&txI0.deltaBody2World.q.x);
	const QuatV deltaRotB = Ps::aos::QuatVLoadA(&txI1.deltaBody2World.q.x);

	const Vec3V raPrev = V3LoadA(header->rAWorld);
	const Vec3V rbPrev = V3LoadA(header->rBWorld);

	const Vec3V ra = QuatRotate(deltaRotA, raPrev);
	const Vec3V rb = QuatRotate(deltaRotB, rbPrev);

	
	const Vec3V ang0 = V3LoadA(b0.deltaAngDt);
	const Vec3V ang1 = V3LoadA(b1.deltaAngDt);

	const Vec3V lin0 = V3LoadA(b0.deltaLinDt);
	const Vec3V lin1 = V3LoadA(b1.deltaLinDt);

	const Vec3V raMotion = V3Sub(V3Add(ra, lin0), raPrev);
	const Vec3V rbMotion = V3Sub(V3Add(rb, lin1), rbPrev);

	const VecCrossV raCross = V3PrepareCross(ra);
	const VecCrossV rbCross = V3PrepareCross(rb);

	const Vec4V ang0Ortho0_recipResponseW = V4LoadA(&header->angOrthoAxis0_recipResponseW[0].x);
	const Vec4V ang0Ortho1_recipResponseW = V4LoadA(&header->angOrthoAxis0_recipResponseW[1].x);
	const Vec4V ang0Ortho2_recipResponseW = V4LoadA(&header->angOrthoAxis0_recipResponseW[2].x);

	const Vec4V ang1Ortho0_Error0 = V4LoadA(&header->angOrthoAxis1_Error[0].x);
	const Vec4V ang1Ortho1_Error1 = V4LoadA(&header->angOrthoAxis1_Error[1].x);
	const Vec4V ang1Ortho2_Error2 = V4LoadA(&header->angOrthoAxis1_Error[2].x);

	const FloatV recipResponse0 = V4GetW(ang0Ortho0_recipResponseW);
	const FloatV recipResponse1 = V4GetW(ang0Ortho1_recipResponseW);
	const FloatV recipResponse2 = V4GetW(ang0Ortho2_recipResponseW);

	const Vec3V ang0Ortho0 = Vec3V_From_Vec4V(ang0Ortho0_recipResponseW);
	const Vec3V ang0Ortho1 = Vec3V_From_Vec4V(ang0Ortho1_recipResponseW);
	const Vec3V ang0Ortho2 = Vec3V_From_Vec4V(ang0Ortho2_recipResponseW);

	const Vec3V ang1Ortho0 = Vec3V_From_Vec4V(ang1Ortho0_Error0);
	const Vec3V ang1Ortho1 = Vec3V_From_Vec4V(ang1Ortho1_Error1);
	const Vec3V ang1Ortho2 = Vec3V_From_Vec4V(ang1Ortho2_Error2);

	FloatV error0 = FAdd(V4GetW(ang1Ortho0_Error0), FSub(V3Dot(ang0Ortho0, ang0), V3Dot(ang1Ortho0, ang1)));
	FloatV error1 = FAdd(V4GetW(ang1Ortho1_Error1), FSub(V3Dot(ang0Ortho1, ang0), V3Dot(ang1Ortho1, ang1)));
	FloatV error2 = FAdd(V4GetW(ang1Ortho2_Error2), FSub(V3Dot(ang0Ortho2, ang0), V3Dot(ang1Ortho2, ang1)));

	for (PxU32 i = 0; i<header->count; ++i, base++)
	{
		Ps::prefetchLine(base + 1);
		SolverConstraint1DStep& c = *base;

		const Vec3V clinVel0 = V3LoadA(c.lin0);
		const Vec3V clinVel1 = V3LoadA(c.lin1);

		const Vec3V cangVel0_ = V3LoadA(c.ang0);
		const Vec3V cangVel1_ = V3LoadA(c.ang1);

		const FloatV angularErrorScale = FLoad(c.angularErrorScale);

		const FloatV biasScale = FLoad(c.biasScale);
		const FloatV maxBias = FLoad(c.maxBias);
		const FloatV targetVel = FLoad(c.velTarget);
		const FloatV iMul = FLoad(c.impulseMultiplier);
		const FloatV appliedForce = FLoad(c.appliedForce);
		const FloatV velMultiplier = FLoad(c.velMultiplier);

		const FloatV maxImpulse = FLoad(c.maxImpulse);
		const FloatV minImpulse = FLoad(c.minImpulse);

		Vec3V cangVel0 = V3Add(cangVel0_, V3Cross(raCross, clinVel0));
		Vec3V cangVel1 = V3Add(cangVel1_, V3Cross(rbCross, clinVel1));
		

		FloatV error = FLoad(c.error);

		const FloatV minBias = (c.flags & DY_SC_FLAG_INEQUALITY) ? FNeg(FMax()) : FNeg(maxBias);

		
		Vec3V raXnI = M33MulV3(sqrtInvInertia0, cangVel0);
		Vec3V rbXnI = M33MulV3(sqrtInvInertia1, cangVel1);

		if (c.flags & DY_SC_FLAG_ORTHO_TARGET)
		{
			//Re-orthogonalize the constraints before velocity projection and impulse response calculation
			//Can be done in using instruction parallelism because angular locked axes are orthogonal to linear axes!

			const FloatV proj0 = FMul(V3SumElems(V3MulAdd(raXnI, ang0Ortho0,
				V3Mul(rbXnI, ang1Ortho0))), recipResponse0);

			const FloatV proj1 = FMul(V3SumElems(V3MulAdd(raXnI, ang0Ortho1, 
				V3Mul(rbXnI, ang1Ortho1))), recipResponse1);
			const FloatV proj2 = FMul(V3SumElems(V3MulAdd(raXnI, ang0Ortho2,
				V3Mul(rbXnI, ang1Ortho2))), recipResponse2);

			const Vec3V delta0 = V3ScaleAdd(ang0Ortho0, proj0, V3ScaleAdd(ang0Ortho1, proj1, V3Scale(ang0Ortho2, proj2)));
			const Vec3V delta1 = V3ScaleAdd(ang1Ortho0, proj0, V3ScaleAdd(ang1Ortho1, proj1, V3Scale(ang1Ortho2, proj2)));

			raXnI = V3Sub(raXnI, delta0);
			rbXnI = V3Sub(rbXnI, delta1);

			error = FSub(error, FScaleAdd(error0, proj0, FScaleAdd(error1, proj1, FMul(error2, proj2))));
		}

		const FloatV deltaAng = FMul(angularErrorScale, FSub(V3Dot(raXnI, ang0), V3Dot(rbXnI, ang1)));

		error = FNegScaleSub(targetVel, elapsed, FAdd(FAdd(error, FSub(V3Dot(raMotion, clinVel0), V3Dot(rbMotion, clinVel1))), deltaAng));

		const FloatV resp0 = FScaleAdd(invMass0, V3Dot(clinVel0, clinVel0), V3SumElems(V3Mul(V3Scale(raXnI, invInertiaScale0), raXnI)));
		const FloatV resp1 = FSub(FMul(invMass1, V3Dot(clinVel1, clinVel1)), V3SumElems(V3Mul(V3Scale(rbXnI, invInertiaScale1), rbXnI)));
		
		const FloatV response = FAdd(resp0, resp1);
		const FloatV recipResponse = FSel(FIsGrtr(response, FZero()), FRecip(response), FZero());
	
		const FloatV vMul = FMul(recipResponse, velMultiplier);
		
		const FloatV unclampedBias = FMul(error, biasScale);
		const FloatV bias = FClamp(unclampedBias, minBias, maxBias);

		const FloatV constant = FMul(recipResponse, FAdd(bias, targetVel));

		const Vec3V v0 = V3MulAdd(linVel0, clinVel0, V3Mul(angState0, raXnI));
		const Vec3V v1 = V3MulAdd(linVel1, clinVel1, V3Mul(angState1, rbXnI));
		const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

		const FloatV af = FMul(iMul, appliedForce);

		const FloatV unclampedForce = FAdd(af, FScaleAdd(vMul, normalVel, constant));
		const FloatV clampedForce = FClamp(unclampedForce, minImpulse, maxImpulse);
		const FloatV deltaF = FSub(clampedForce, appliedForce);

		FStore(clampedForce, &c.appliedForce);
		linVel0 = V3ScaleAdd(clinVel0, FMul(deltaF, invMass0), linVel0);
		linVel1 = V3NegScaleSub(clinVel1, FMul(deltaF, invMass1), linVel1);
		angState0 = V3ScaleAdd(raXnI, FMul(deltaF, invInertiaScale0), angState0);
		angState1 = V3ScaleAdd(rbXnI, FMul(deltaF, invInertiaScale1), angState1);

	}

	V3StoreA(linVel0, b0.linearVelocity);
	V3StoreA(angState0, b0.angularVelocity);
	V3StoreA(linVel1, b1.linearVelocity);
	V3StoreA(angState1, b1.angularVelocity);

	PX_ASSERT(b0.linearVelocity.isFinite());
	PX_ASSERT(b0.angularVelocity.isFinite());
	PX_ASSERT(b1.linearVelocity.isFinite());
	PX_ASSERT(b1.angularVelocity.isFinite());
}


//Port of scalar implementation to SIMD maths with some interleaving of instructions
void conclude1DStep(const PxTGSSolverConstraintDesc& desc)
{
	PxU8* PX_RESTRICT bPtr = desc.constraint;
	if (bPtr == NULL)
		return;
	
	const SolverConstraint1DHeaderStep* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeaderStep*>(bPtr);
	PxU8* PX_RESTRICT base = bPtr + sizeof(SolverConstraint1DHeaderStep);
	const PxU32 stride = header->type == DY_SC_TYPE_RB_1D ? sizeof(SolverConstraint1DStep) : sizeof(SolverConstraint1DExtStep);

	for (PxU32 i = 0; i<header->count; ++i, base+=stride)
	{
		SolverConstraint1DStep& c = *reinterpret_cast<SolverConstraint1DStep*>(base);
		Ps::prefetchLine(&c + 1);
		if (!(c.flags & DY_SC_FLAG_KEEP_BIAS))
			c.biasScale = 0.f;
	}
}

void concludeContact(const PxTGSSolverConstraintDesc& desc)
{
	PX_UNUSED(desc);
	//const PxU8* PX_RESTRICT last = desc.constraint + getConstraintLength(desc);

	////hopefully pointer aliasing doesn't bite.
	//PxU8* PX_RESTRICT currPtr = desc.constraint;

	//SolverContactHeaderStep* PX_RESTRICT firstHdr = reinterpret_cast<SolverContactHeaderStep*>(currPtr);

	//bool isExtended = firstHdr->type == DY_SC_TYPE_EXT_CONTACT;

	//const PxU32 contactStride = isExtended ? sizeof(SolverContactPointStepExt) : sizeof(SolverContactPointStep);
	//const PxU32 frictionStride = isExtended ? sizeof(SolverContactFrictionStepExt) : sizeof(SolverContactFrictionStep);


	//while (currPtr < last)
	//{
	//	SolverContactHeaderStep* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStep*>(currPtr);
	//	currPtr += sizeof(SolverContactHeaderStep);

	//	const PxU32 numNormalConstr = hdr->numNormalConstr;
	//	const PxU32	numFrictionConstr = hdr->numFrictionConstr;

	//	/*PxU8* PX_RESTRICT contacts = currPtr;
	//	Ps::prefetchLine(contacts);*/
	//	currPtr += numNormalConstr * contactStride;

	//	//PxF32* forceBuffer = reinterpret_cast<PxF32*>(currPtr);
	//	currPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

	//	PxU8* PX_RESTRICT frictions = currPtr;
	//	currPtr += numFrictionConstr * frictionStride;

	//	/*for (PxU32 i = 0; i < numNormalConstr; ++i)
	//	{
	//		SolverContactPointStep& c = *reinterpret_cast<SolverContactPointStep*>(contacts);
	//		contacts += contactStride;
	//		if(c.separation <= 0.f)
	//			c.biasCoefficient = 0.f;
	//	}*/

	//	for (PxU32 i = 0; i < numFrictionConstr; ++i)
	//	{
	//		SolverContactFrictionStep& f = *reinterpret_cast<SolverContactFrictionStep*>(frictions);
	//		frictions += frictionStride;
	//		f.biasScale = 0.f;
	//	}
	//}

	//PX_ASSERT(currPtr == last);
}




void writeBack1D(const PxTGSSolverConstraintDesc& desc)
{
	ConstraintWriteback* writeback = reinterpret_cast<ConstraintWriteback*>(desc.writeBack);
	if (writeback)
	{
		SolverConstraint1DHeaderStep* header = reinterpret_cast<SolverConstraint1DHeaderStep*>(desc.constraint);
		PxU8* base = desc.constraint + sizeof(SolverConstraint1DHeaderStep);
		PxU32 stride = header->type == DY_SC_TYPE_EXT_1D ? sizeof(SolverConstraint1DExtStep) : sizeof(SolverConstraint1DStep);

		PxVec3 lin(0), ang(0);
		for (PxU32 i = 0; i<header->count; i++)
		{
			const SolverConstraint1DStep* c = reinterpret_cast<SolverConstraint1DStep*>(base);
			if (c->flags & DY_SC_FLAG_OUTPUT_FORCE)
			{
				lin += c->lin0 * c->appliedForce;
				ang += (c->ang0 + c->lin0.cross(header->rAWorld)) * c->appliedForce;
			}
			base += stride;
		}

		ang -= header->body0WorldOffset.cross(lin);
		writeback->linearImpulse = lin;
		writeback->angularImpulse = ang;
		writeback->broken = header->breakable ? PxU32(lin.magnitude()>header->linBreakImpulse || ang.magnitude()>header->angBreakImpulse) : 0;

		PX_ASSERT(desc.constraint + (desc.constraintLengthOver16 * 16) == base);
	}
}


PxU32 Articulation::setupSolverConstraintsTGS(const ArticulationSolverDesc& articDesc,
	PxcConstraintBlockStream& stream,
	PxTGSSolverConstraintDesc* constraintDesc,
	PxReal dt,
	PxReal invDt,
	PxReal,
	PxU32& acCount,
	PxsConstraintBlockManager& constraintBlockManager,
	Cm::SpatialVectorF* /*Z*/)
{
	Articulation* articulation = static_cast<Articulation*>(articDesc.articulation);
	FsData& fsData = *articulation->getFsDataPtr();
	const PxU32 solverDataSize = articDesc.solverDataSize;
	const ArticulationLink* links = articDesc.links;

	PxcFsScratchAllocator allocator(articDesc.scratchMemory, articDesc.scratchMemorySize);
	FsInertia*						PX_RESTRICT baseInertia = allocator.alloc<FsInertia>(articDesc.linkCount);
	PX_UNUSED(baseInertia);
	ArticulationJointTransforms*	PX_RESTRICT jointTransforms = allocator.alloc<ArticulationJointTransforms>(articDesc.linkCount);

	PX_UNUSED(dt);
	acCount = 0;

	const PxU16 linkCount = fsData.linkCount;
	PxU32 descCount = 0;
	const PxReal recipDt = invDt;

	const PxConstraintInvMassScale ims(1.0f, 1.0f, 1.0f, 1.0f);

	for (PxU16 i = 1; i<linkCount; i++)
	{
		const ArticulationJointCore& j = static_cast<const ArticulationJointCore&>(*links[i].inboundJoint);

		if (i + 1<linkCount)
		{
			Ps::prefetch(links[i + 1].inboundJoint, sizeof(ArticulationJointCore));
			Ps::prefetch(&jointTransforms[i + 1], sizeof(ArticulationJointTransforms));
		}

		if (!(j.twistLimited || j.swingLimited))
			continue;

		PxQuat swing, twist;
		Ps::separateSwingTwist(jointTransforms[i].cB2cA.q, swing, twist);

		Cm::ConeLimitHelper eh(j.tanQSwingY, j.tanQSwingZ, j.tanQSwingPad);
		PxVec3 swingLimitAxis;
		PxReal swingLimitError = 0.0f;

		const bool swingLimited = j.swingLimited && eh.getLimit(swing, swingLimitAxis, swingLimitError);
		const bool tangentialStiffness = swingLimited && (j.tangentialStiffness>0 || j.tangentialDamping>0);

		const PxVec3 twistAxis = jointTransforms[i].cB2w.rotate(PxVec3(1.0f, 0, 0));
		const PxReal tqTwistAngle = Ps::tanHalf(twist.x, twist.w);

		const bool twistLowerLimited = j.twistLimited && tqTwistAngle < Cm::tanAdd(j.tanQTwistLow, j.tanQTwistPad);
		const bool twistUpperLimited = j.twistLimited && tqTwistAngle > Cm::tanAdd(j.tanQTwistHigh, -j.tanQTwistPad);

		const PxU8 constraintCount = PxU8(swingLimited + tangentialStiffness + twistUpperLimited + twistLowerLimited);
		if (!constraintCount)
			continue;

		PxTGSSolverConstraintDesc& desc = constraintDesc[descCount++];

		desc.articulationA = articulation;
		desc.linkIndexA = Ps::to16(links[i].parent);
		desc.articulationALength = Ps::to16(solverDataSize);

		desc.articulationB = articulation;
		desc.linkIndexB = i;
		desc.articulationBLength = Ps::to16(solverDataSize);

		const PxU32 constraintLength = sizeof(SolverConstraint1DHeaderStep) +
			sizeof(SolverConstraint1DExtStep) * constraintCount;

		PX_ASSERT(0 == (constraintLength & 0x0f));
		desc.constraintLengthOver16 = Ps::to16(constraintLength / 16);

		desc.constraint = stream.reserve(constraintLength + 16u, constraintBlockManager);

		desc.writeBack = NULL;

		SolverConstraint1DHeaderStep* header = reinterpret_cast<SolverConstraint1DHeaderStep*>(desc.constraint);
		SolverConstraint1DExtStep* constraints = reinterpret_cast<SolverConstraint1DExtStep*>(desc.constraint + sizeof(SolverConstraint1DHeaderStep));

		init(*header, constraintCount, true, ims);

		header->rAWorld = PxVec3(0.f);
		header->rBWorld = PxVec3(0.f);

		PxU32 cIndex = 0;

		if (swingLimited)
		{
			const PxVec3 normal = jointTransforms[i].cA2w.rotate(swingLimitAxis);
			ArticulationHelper::createHardLimitTGS(fsData, links, i, constraints[cIndex++], normal, swingLimitError, recipDt);
			if (tangentialStiffness)
			{
				const PxVec3 tangent = twistAxis.cross(normal).getNormalized();
				ArticulationHelper::createTangentialSpringTGS(fsData, links, i, constraints[cIndex++], tangent, j.tangentialStiffness, j.tangentialDamping, 1.f/recipDt);
			}
		}

		if (twistUpperLimited)
			ArticulationHelper::createHardLimitTGS(fsData, links, i, constraints[cIndex++], twistAxis, (j.tanQTwistHigh - tqTwistAngle) * 4, recipDt);

		if (twistLowerLimited)
			ArticulationHelper::createHardLimitTGS(fsData, links, i, constraints[cIndex++], -twistAxis, -(j.tanQTwistLow - tqTwistAngle) * 4, recipDt);

		*(desc.constraint + getConstraintLength(desc)) = 0;

		PX_ASSERT(cIndex == constraintCount);
		acCount += constraintCount;
	}

	return descCount;

}


void ArticulationHelper::createHardLimitTGS(const FsData& fsData,
	const ArticulationLink* links,
	PxU32 linkIndex,
	SolverConstraint1DExtStep& s,
	const PxVec3& axis,
	PxReal err,
	PxReal recipDt)
{
	PxReal error = err;
	init(s, PxVec3(0), PxVec3(0), axis, axis, 0, PX_MAX_F32);

	ArticulationHelper::getImpulseSelfResponse(fsData,
		links[linkIndex].parent, Cm::SpatialVector(PxVec3(0), axis), s.deltaVA,
		linkIndex, Cm::SpatialVector(PxVec3(0), -axis), s.deltaVB);

	const PxReal unitResponse = axis.dot(reinterpret_cast<PxVec3&>(s.deltaVA.angular)) - axis.dot(reinterpret_cast<PxVec3&>(s.deltaVB.angular));
	if (unitResponse<0.0f)
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Warning: articulation ill-conditioned or under severe stress, joint limit ignored");

	const PxReal recipResponse = unitResponse>0.0f ? 1.0f / unitResponse : 0.0f;

	s.error = error;
	s.biasScale = -recipDt*0.7f;
	s.maxBias = PX_MAX_F32;
	s.velMultiplier = -1.f;
	s.recipResponse = recipResponse;
	s.impulseMultiplier = 1.0f;
	s.velTarget = 0.f;
}

void ArticulationHelper::createTangentialSpringTGS(const FsData& fsData,
	const ArticulationLink* links,
	PxU32 linkIndex,
	SolverConstraint1DExtStep& s,
	const PxVec3& axis,
	PxReal stiffness,
	PxReal damping,
	PxReal dt)
{
	init(s, PxVec3(0), PxVec3(0), axis, axis, -PX_MAX_F32, PX_MAX_F32);

	Cm::SpatialVector axis6(PxVec3(0), axis);
	PxU32 parent = links[linkIndex].parent;
	getImpulseSelfResponse(fsData, parent, axis6, s.deltaVA, linkIndex, -axis6, s.deltaVB);

	const PxReal unitResponse = axis.dot(reinterpret_cast<PxVec3&>(s.deltaVA.angular)) - axis.dot(reinterpret_cast<PxVec3&>(s.deltaVB.angular));
	if (unitResponse<0.0f)
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Warning: articulation ill-conditioned or under severe stress, tangential spring ignored");
	const PxReal recipResponse = unitResponse>0.0F ? 1.0f / unitResponse : 0.0f;

	// this is a specialization of the spring code in setSolverConstants() for acceleration springs.
	// general case is  b = dt * (c.mods.spring.damping * c.velocityTarget - c.mods.spring.stiffness * geomError);
	// but geomError and velocityTarget are both zero

	const PxReal a = dt * dt * stiffness + dt * damping;
	const PxReal x = 1.0f / (1.0f + a);
	s.error = 0.f;
	s.biasScale = 0.f;
	s.maxBias = 0.f;
	s.velMultiplier = -x * a;
	s.impulseMultiplier = 1.0f - x;
	s.velTarget = 0.f;
	s.recipResponse = recipResponse;
}

static FloatV solveExtContactsStep(SolverContactPointStepExt* contacts, const PxU32 nbContactPoints, const Vec3VArg contactNormal,
	Vec3V& linVel0, Vec3V& angVel0,
	Vec3V& linVel1, Vec3V& angVel1,
	Vec3V& li0, Vec3V& ai0,
	Vec3V& li1, Vec3V& ai1,
	const Vec3V& linDeltaA, const Vec3V& linDeltaB, const Vec3V& angDeltaA, const Vec3V angDeltaB, const FloatV& maxPenBias,
	PxF32* PX_RESTRICT appliedForceBuffer,
	const FloatV& minPen,
	const FloatV& elapsedTime)
{

	const FloatV deltaV = V3Dot(contactNormal, V3Sub(linDeltaA, linDeltaB));

	FloatV accumulatedNormalImpulse = FZero();
	for (PxU32 i = 0; i<nbContactPoints; i++)
	{
		SolverContactPointStepExt& c = contacts[i];
		Ps::prefetchLine(&contacts[i + 1]);

		const Vec3V raXn = V3LoadA(c.raXnI);
		const Vec3V rbXn = V3LoadA(c.rbXnI);

		const FloatV appliedForce = FLoad(appliedForceBuffer[i]);
		const FloatV velMultiplier = FLoad(c.velMultiplier);

		Vec3V v = V3MulAdd(linVel0, contactNormal, V3Mul(angVel0, raXn));
		v = V3Sub(v, V3MulAdd(linVel1, contactNormal, V3Mul(angVel1, rbXn)));
		const FloatV normalVel = V3SumElems(v);


		const FloatV angDelta0 = V3Dot(angDeltaA, raXn);
		const FloatV angDelta1 = V3Dot(angDeltaB, rbXn);

		const FloatV deltaAng = FSub(angDelta0, angDelta1);

		const FloatV targetVel = FLoad(c.targetVelocity);

		const FloatV deltaBias = FSub(FAdd(deltaV, deltaAng), FMul(targetVel, elapsedTime));

		const FloatV biasCoefficient = FLoad(c.biasCoefficient);

		FloatV sep = FMax(minPen, FAdd(FLoad(c.separation), deltaBias));

		const FloatV bias = FMin(FNeg(maxPenBias), FMul(biasCoefficient, sep));

		

		const FloatV tVel = FAdd(bias, targetVel);

		const FloatV deltaF = FMax(FMul(FSub(tVel, normalVel), velMultiplier), FNeg(appliedForce));

		const Vec3V raXnI = c.angDeltaVA;
		const Vec3V rbXnI = c.angDeltaVB;


		linVel0 = V3ScaleAdd(c.linDeltaVA, deltaF, linVel0);
		angVel0 = V3ScaleAdd(raXnI, deltaF, angVel0);
		linVel1 = V3ScaleAdd(c.linDeltaVB, deltaF, linVel1);
		angVel1 = V3ScaleAdd(rbXnI, deltaF, angVel1);

		li0 = V3ScaleAdd(contactNormal, deltaF, li0);	ai0 = V3ScaleAdd(raXn, deltaF, ai0);
		li1 = V3ScaleAdd(contactNormal, deltaF, li1);	ai1 = V3ScaleAdd(rbXn, deltaF, ai1);

		const FloatV newAppliedForce = FAdd(appliedForce, deltaF);

		FStore(newAppliedForce, &appliedForceBuffer[i]);

		accumulatedNormalImpulse = FAdd(accumulatedNormalImpulse, newAppliedForce);
	}
	return accumulatedNormalImpulse;
}

void solveExtContactStep(const PxTGSSolverConstraintDesc& desc, bool doFriction, const PxReal minPenetration,
	const PxReal elapsedTimeF32, SolverContext& cache)
{
	const FloatV elapsedTime = FLoad(elapsedTimeF32);
	Vec3V linVel0, angVel0, linVel1, angVel1;
	Vec3V linDelta0, angDelta0, linDelta1, angDelta1;

	const FloatV minPen = FLoad(minPenetration);

	if (desc.linkIndexA == PxSolverConstraintDesc::NO_LINK)
	{
		linVel0 = V3LoadA(desc.bodyA->linearVelocity);
		angVel0 = V3LoadA(desc.bodyA->angularVelocity);
		linDelta0 = V3LoadA(desc.bodyA->deltaLinDt);
		angDelta0 = V3LoadA(desc.bodyA->deltaAngDt);
	}
	else
	{
		Cm::SpatialVectorV v = desc.articulationA->pxcFsGetVelocity(desc.linkIndexA);
		Cm::SpatialVectorV deltaV = PxcFsGetMotionVector(*desc.articulationA, desc.linkIndexA);
		linVel0 = v.linear;
		angVel0 = v.angular;
		linDelta0 = deltaV.linear;
		angDelta0 = deltaV.angular;
	}

	if (desc.linkIndexB == PxSolverConstraintDesc::NO_LINK)
	{
		linVel1 = V3LoadA(desc.bodyB->linearVelocity);
		angVel1 = V3LoadA(desc.bodyB->angularVelocity);
		linDelta1 = V3LoadA(desc.bodyB->deltaLinDt);
		angDelta1 = V3LoadA(desc.bodyB->deltaAngDt);
	}
	else
	{
		Cm::SpatialVectorV v = desc.articulationB->pxcFsGetVelocity(desc.linkIndexB);
		Cm::SpatialVectorV deltaV = PxcFsGetMotionVector(*desc.articulationB, desc.linkIndexB);
		linVel1 = v.linear;
		angVel1 = v.angular;
		linDelta1 = deltaV.linear;
		angDelta1 = deltaV.angular;
	}

	/*PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(linVel0, linVel0)));
	PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(linVel1, linVel1)));
	PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(angVel0, angVel0)));
	PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(angVel1, angVel1)));*/


	const PxU8* PX_RESTRICT last = desc.constraint + desc.constraintLengthOver16 * 16;

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc.constraint;

	Vec3V linImpulse0 = V3Zero(), linImpulse1 = V3Zero(), angImpulse0 = V3Zero(), angImpulse1 = V3Zero();

	const Vec3V relMotion = V3Sub(linDelta0, linDelta1);

	while (currPtr < last)
	{
		SolverContactHeaderStep* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStep*>(currPtr);
		currPtr += sizeof(SolverContactHeaderStep);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		SolverContactPointStepExt* PX_RESTRICT contacts = reinterpret_cast<SolverContactPointStepExt*>(currPtr);
		Ps::prefetchLine(contacts);
		currPtr += numNormalConstr * sizeof(SolverContactPointStepExt);

		PxF32* appliedForceBuffer = reinterpret_cast<PxF32*>(currPtr);
		currPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

		SolverContactFrictionStepExt* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionStepExt*>(currPtr);
		currPtr += numFrictionConstr * sizeof(SolverContactFrictionStepExt);



		Vec3V li0 = V3Zero(), li1 = V3Zero(), ai0 = V3Zero(), ai1 = V3Zero();

		const Vec3V contactNormal = V3LoadA(hdr->normal);

		const FloatV accumulatedNormalImpulse = FMax(solveExtContactsStep(contacts, numNormalConstr, contactNormal, linVel0, angVel0, linVel1,
			angVel1, li0, ai0, li1, ai1, linDelta0, linDelta1, angDelta0, angDelta1, FLoad(hdr->maxPenBias), appliedForceBuffer, minPen, elapsedTime),
			FLoad(hdr->minNormalForce));

		if(doFriction && numFrictionConstr)
		{
			Ps::prefetchLine(frictions);
			const FloatV maxFrictionImpulse = FMul(hdr->getStaticFriction(), accumulatedNormalImpulse);
			const FloatV maxDynFrictionImpulse = FMul(hdr->getDynamicFriction(), accumulatedNormalImpulse);

			BoolV broken = BFFFF();

			for (PxU32 i = 0; i<numFrictionConstr; i++)
			{
				SolverContactFrictionStepExt& f = frictions[i];
				Ps::prefetchLine(&frictions[i + 1]);

				const Vec4V normalXYZ_ErrorW = f.normalXYZ_ErrorW;
				const Vec4V raXn_targetVelW = f.raXnI_targetVelW;
				const Vec4V rbXn_velMultiplierW = f.rbXnI_velMultiplierW;

				const Vec3V normal = Vec3V_From_Vec4V(normalXYZ_ErrorW);
				const Vec3V raXn = Vec3V_From_Vec4V(raXn_targetVelW);
				const Vec3V rbXn = Vec3V_From_Vec4V(rbXn_velMultiplierW);
				const Vec3V raXnI = f.angDeltaVA;
				const Vec3V rbXnI = f.angDeltaVB;

				const FloatV appliedForce = FLoad(f.appliedForce);
				const FloatV biasScale = FLoad(f.biasScale);
				const FloatV velMultiplier = V4GetW(rbXn_velMultiplierW);
				const FloatV targetVel = V4GetW(raXn_targetVelW);
				const FloatV initialError = V4GetW(normalXYZ_ErrorW);

				const FloatV negMaxDynFrictionImpulse = FNeg(maxDynFrictionImpulse);
				//const FloatV negMaxFrictionImpulse = FNeg(maxFrictionImpulse);

				const FloatV error = FAdd(initialError, FScaleAdd(targetVel, elapsedTime, FAdd(FSub(V3Dot(raXn, angDelta0), V3Dot(rbXn, angDelta1)), V3Dot(normal, relMotion))));

				const FloatV bias = FMul(error, biasScale);

				const Vec3V v0 = V3MulAdd(linVel0, normal, V3Mul(angVel0, raXn));
				const Vec3V v1 = V3MulAdd(linVel1, normal, V3Mul(angVel1, rbXn));
				const FloatV normalVel = V3SumElems(V3Sub(v0, v1));



				// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
				const FloatV tmp1 = FNegScaleSub(FSub(bias, targetVel), velMultiplier, appliedForce);

				// Algorithm:
				// if abs(appliedForce + deltaF) > maxFrictionImpulse
				//    clamp newAppliedForce + deltaF to [-maxDynFrictionImpulse, maxDynFrictionImpulse]
				//      (i.e. clamp deltaF to [-maxDynFrictionImpulse-appliedForce, maxDynFrictionImpulse-appliedForce]
				//    set broken flag to true || broken flag

				// FloatV deltaF = FMul(FAdd(bias, normalVel), minusVelMultiplier);
				// FloatV potentialSumF = FAdd(appliedForce, deltaF);

				const FloatV totalImpulse = FNegScaleSub(normalVel, velMultiplier, tmp1);

				// On XBox this clamping code uses the vector simple pipe rather than vector float,
				// which eliminates a lot of stall cycles

				const BoolV clamp = FIsGrtr(FAbs(totalImpulse), maxFrictionImpulse);

				const FloatV totalClamped = FMin(maxDynFrictionImpulse, FMax(negMaxDynFrictionImpulse, totalImpulse));

				const FloatV newAppliedForce = FSel(clamp, totalClamped, totalImpulse);

				broken = BOr(broken, clamp);

				FloatV deltaF = FSub(newAppliedForce, appliedForce);

				linVel0 = V3ScaleAdd(f.linDeltaVA, deltaF, linVel0);
				angVel0 = V3ScaleAdd(raXnI, deltaF, angVel0);
				linVel1 = V3ScaleAdd(f.linDeltaVB, deltaF, linVel1);
				angVel1 = V3ScaleAdd(rbXnI, deltaF, angVel1);

				/*PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(linVel0, linVel0)));
				PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(linVel1, linVel1)));
				PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(angVel0, angVel0)));
				PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(angVel1, angVel1)));*/

				li0 = V3ScaleAdd(normal, deltaF, li0);	ai0 = V3ScaleAdd(raXn, deltaF, ai0);
				li1 = V3ScaleAdd(normal, deltaF, li1);	ai1 = V3ScaleAdd(rbXn, deltaF, ai1);

				f.setAppliedForce(newAppliedForce);
			}
			Store_From_BoolV(broken, &hdr->broken);
		}

		linImpulse0 = V3ScaleAdd(li0, hdr->getDominance0(), linImpulse0);
		angImpulse0 = V3ScaleAdd(ai0, FLoad(hdr->angDom0), angImpulse0);
		linImpulse1 = V3NegScaleSub(li1, hdr->getDominance1(), linImpulse1);
		angImpulse1 = V3NegScaleSub(ai1, FLoad(hdr->angDom1), angImpulse1);
	}

	if (desc.linkIndexA == PxSolverConstraintDesc::NO_LINK)
	{
		V3StoreA(linVel0, desc.bodyA->linearVelocity);
		V3StoreA(angVel0, desc.bodyA->angularVelocity);
	}
	else
	{
		desc.articulationA->pxcFsApplyImpulse(desc.linkIndexA, 
			linImpulse0, angImpulse0, cache.Z, cache.deltaV);
	}

	if (desc.linkIndexB == PxSolverConstraintDesc::NO_LINK)
	{
		V3StoreA(linVel1, desc.bodyB->linearVelocity);
		V3StoreA(angVel1, desc.bodyB->angularVelocity);
	}
	else
	{
		desc.articulationB->pxcFsApplyImpulse(desc.linkIndexB, linImpulse1, 
			angImpulse1, cache.Z, cache.deltaV);
	}

	PX_ASSERT(currPtr == last);
}

void solveContactBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const bool doFriction,
	const PxReal minPenetration, const PxReal elapsedTime)
{
	for (PxU32 i = hdr.mStartIndex, endIdx = hdr.mStartIndex + hdr.mStride; i < endIdx; ++i)
	{
		solveContact(desc[i], doFriction, minPenetration, elapsedTime);
	}
}

void solve1DBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const PxTGSSolverBodyTxInertia* const txInertias, const PxReal elapsedTime)
{
	for (PxU32 i = hdr.mStartIndex, endIdx = hdr.mStartIndex + hdr.mStride; i < endIdx; ++i)
	{
		solve1DStep(desc[i], txInertias, elapsedTime);
	}
}

void solveExtContactBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const bool doFriction,
	const PxReal minPenetration, const PxReal elapsedTime, SolverContext& cache)
{
	for (PxU32 i = hdr.mStartIndex, endIdx = hdr.mStartIndex + hdr.mStride; i < endIdx; ++i)
	{
		solveExtContactStep(desc[i], doFriction, minPenetration, elapsedTime, cache);
	}
}

void solveExt1DBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const PxReal elapsedTime, SolverContext& cache,
	const PxTGSSolverBodyTxInertia* const txInertias)
{
	for (PxU32 i = hdr.mStartIndex, endIdx = hdr.mStartIndex + hdr.mStride; i < endIdx; ++i)
	{
		solveExt1DStep(desc[i], elapsedTime, cache, txInertias);
	}
}

void writeBackContact(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, SolverContext* cache)
{
	for (PxU32 i = hdr.mStartIndex, endIdx = hdr.mStartIndex + hdr.mStride; i < endIdx; ++i)
	{
		writeBackContact(desc[i], cache);
	}
}

void writeBack1D(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc)
{
	for (PxU32 i = hdr.mStartIndex, endIdx = hdr.mStartIndex + hdr.mStride; i < endIdx; ++i)
	{
		writeBack1D(desc[i]);
	}
}

void solveConclude1DBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc,
	const PxTGSSolverBodyTxInertia* const txInertias, const PxReal elapsedTime)
{
	for (PxU32 i = hdr.mStartIndex, endIdx = hdr.mStartIndex + hdr.mStride; i < endIdx; ++i)
	{
		solve1DStep(desc[i], txInertias, elapsedTime);
		conclude1DStep(desc[i]);
	}
}

void solveConclude1DBlockExt(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc,
	const PxReal elapsedTime, SolverContext& cache, const PxTGSSolverBodyTxInertia* const txInertias)
{
	for (PxU32 i = hdr.mStartIndex, endIdx = hdr.mStartIndex + hdr.mStride; i < endIdx; ++i)
	{
		solveExt1DStep(desc[i], elapsedTime, cache, txInertias);
		conclude1DStep(desc[i]);
	}
}


void solveConcludeContactBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const PxReal elapsedTime)
{
	for (PxU32 i = hdr.mStartIndex, endIdx = hdr.mStartIndex + hdr.mStride; i < endIdx; ++i)
	{
		solveContact(desc[i], true, -PX_MAX_F32, elapsedTime);
		concludeContact(desc[i]);
	}
}

void solveConcludeContactExtBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const PxReal elapsedTime, SolverContext& cache)
{
	for (PxU32 i = hdr.mStartIndex, endIdx = hdr.mStartIndex + hdr.mStride; i < endIdx; ++i)
	{
		solveExtContactStep(desc[i], true, -PX_MAX_F32, elapsedTime, cache);
		concludeContact(desc[i]);
	}
}


}
}
