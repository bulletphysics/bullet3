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

#include "foundation/PxVec3.h"
#include "foundation/PxMath.h"
#include "foundation/PxMemory.h"
#include "common/PxProfileZone.h"

#include "PsUtilities.h"
#include "CmSpatialVector.h"
#include "DyArticulationHelper.h"
#include "DyArticulationReference.h"
#include "DyArticulationFnsSimd.h"
#include "DyArticulationFnsScalar.h"
#include "DyArticulationFnsDebug.h"
#include "DySolverConstraintDesc.h"
#include "PxvDynamics.h"
#include "DyArticulation.h"
#include "PxcRigidBody.h"
#include "CmConeLimitHelper.h"
#include "DySolverConstraint1D.h"
#include "PxcConstraintBlockStream.h"
#include "DySolverConstraint1D.h"
#include "DyArticulationPImpl.h"
#include "PsFoundation.h"

namespace physx
{

namespace Dy
{


void getImpulseResponseSlow(const FsData& matrix, 
							PxU32 linkID0, 
							const Cm::SpatialVectorV& impulse0,
							Cm::SpatialVectorV& deltaV0,
							PxU32 linkID1,
							const Cm::SpatialVectorV& impulse1,
							Cm::SpatialVectorV& deltaV1)
{
	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	const FsRow* rows = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	PX_ASSERT(matrix.linkCount<=DY_ARTICULATION_MAX_SIZE);
	PxU32 stack[DY_ARTICULATION_MAX_SIZE];
	Vec3V SZ[DY_ARTICULATION_MAX_SIZE];

	PxU32 i0, i1, ic;
	
	for(i0 = linkID0, i1 = linkID1; i0!=i1;)	// find common path
	{
		if(i0<i1)
			i1 = matrix.parent[i1];
		else
			i0 = matrix.parent[i0];
	}

	PxU32 common = i0;

	Cm::SpatialVectorV Z0 = -impulse0, Z1 = -impulse1;
	for(i0 = 0; linkID0!=common; linkID0 = matrix.parent[linkID0])
	{
		Z0 = Fns::propagateImpulse(rows[linkID0], jointVectors[linkID0], SZ[linkID0], Z0, aux[linkID0]);
		stack[i0++] = linkID0;
	}

	for(i1 = i0; linkID1!=common; linkID1 = matrix.parent[linkID1])
	{
		Z1 = Fns::propagateImpulse(rows[linkID1], jointVectors[linkID1], SZ[linkID1], Z1, aux[linkID1]);
		stack[i1++] = linkID1;
	}

	Cm::SpatialVectorV Z = Z0 + Z1;
	for(ic = i1; common; common = matrix.parent[common])
	{
		Z = Fns::propagateImpulse(rows[common], jointVectors[common], SZ[common], Z, aux[common]);
		stack[ic++] = common;
	}

	Cm::SpatialVectorV v = Fns::multiply(getRootInverseInertia(matrix), -Z);

	for(PxU32 index = ic; index-->i1 ;)
		v = Fns::propagateVelocity(rows[stack[index]], jointVectors[stack[index]], SZ[stack[index]], v, aux[stack[index]]);

	deltaV1 = v;
	for(PxU32 index = i1; index-->i0 ;)
		deltaV1 = Fns::propagateVelocity(rows[stack[index]], jointVectors[stack[index]], SZ[stack[index]], deltaV1, aux[stack[index]]);

	deltaV0 = v;
	for(PxU32 index = i0; index-->0;)
		deltaV0 = Fns::propagateVelocity(rows[stack[index]], jointVectors[stack[index]], SZ[stack[index]], deltaV0, aux[stack[index]]);
}

void PxcFsGetImpulseResponse(const FsData& matrix,
							 PxU32 linkID,
							 const Cm::SpatialVectorV& impulse,
							 Cm::SpatialVectorV& deltaV)
{
	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	PX_ASSERT(matrix.linkCount<=DY_ARTICULATION_MAX_SIZE);
	Vec3V SZ[DY_ARTICULATION_MAX_SIZE];

	const FsRow* rows = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	Cm::SpatialVectorV Z = -impulse;
	
	for(PxU32 i = linkID; i; i = matrix.parent[i])
		Z = Fns::propagateImpulse(rows[i], jointVectors[i], SZ[i], Z, aux[i]);

	deltaV = Fns::multiply(getRootInverseInertia(matrix), -Z);

	PX_ASSERT(rows[linkID].pathToRoot&1);

	for(ArticulationBitField i=rows[linkID].pathToRoot-1; i; i &= (i-1))
	{
		const PxU32 index = ArticulationLowestSetBit(i);
		deltaV = Fns::propagateVelocity(rows[index], jointVectors[index], SZ[index], deltaV, aux[index]);
	}
}

void PxcFsGetImpulseSelfResponse(const FsData& matrix, 
								 PxU32 linkID0, 
								 const Cm::SpatialVectorV& impulse0,
								 Cm::SpatialVectorV& deltaV0,
								 PxU32 linkID1,
								 const Cm::SpatialVectorV& impulse1,
								 Cm::SpatialVectorV& deltaV1)
{
	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	PX_ASSERT(linkID0 != linkID1);

	const FsRow* rows = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	// standard case: parent-child limit
	if(matrix.parent[linkID1] == linkID0)
	{
		Vec3V SZ;
		const Cm::SpatialVectorV Z = impulse0 - Fns::propagateImpulse(rows[linkID1], jointVectors[linkID1], SZ, -impulse1, aux[linkID1]);
		PxcFsGetImpulseResponse(matrix, linkID0, Z, deltaV0);
		deltaV1 = Fns::propagateVelocity(rows[linkID1], jointVectors[linkID1], SZ, deltaV0, aux[linkID1]);
	}
	else
		getImpulseResponseSlow(matrix, linkID0, impulse0, deltaV0, linkID1, impulse1, deltaV1);

#if DY_ARTICULATION_DEBUG_VERIFY
	Cm::SpatialVector V[DY_ARTICULATION_MAX_SIZE];
	for(PxU32 i=0;i<matrix.linkCount;i++) V[i] = Cm::SpatialVector::zero();
	ArticulationRef::applyImpulse(matrix,V,linkID0, reinterpret_cast<const Cm::SpatialVector&>(impulse0));
	ArticulationRef::applyImpulse(matrix,V,linkID1, reinterpret_cast<const Cm::SpatialVector&>(impulse1));

	Cm::SpatialVector refV0 = V[linkID0];
	Cm::SpatialVector refV1 = V[linkID1];
#endif
}

namespace
{

	PX_FORCE_INLINE Cm::SpatialVectorV getImpulseResponseSimd(const FsData& matrix, PxU32 linkID, Vec3V lZ, Vec3V aZ)
	{
		PX_ASSERT(matrix.linkCount<=DY_ARTICULATION_MAX_SIZE);
		Vec3V SZ[DY_ARTICULATION_MAX_SIZE];
		PxU32 indices[DY_ARTICULATION_MAX_SIZE], iCount = 0;

		const FsRow*PX_RESTRICT rows = getFsRows(matrix);
		const FsRowAux*PX_RESTRICT aux = getAux(matrix);
		const FsJointVectors* jointVectors = getJointVectors(matrix);

		PX_UNUSED(aux);
		PX_ASSERT(rows[linkID].pathToRoot&1);

		lZ = V3Neg(lZ);
		aZ = V3Neg(aZ);

		for(PxU32 i = linkID; i; i = matrix.parent[i])
		{
			const FsRow& r = rows[i];
			const FsJointVectors& j = jointVectors[i];

			Vec3V sz = V3Add(aZ, V3Cross(lZ, j.jointOffset));
			SZ[iCount] = sz;
			
			lZ = V3NegScaleSub(r.DSI[0].linear, V3GetX(sz), V3NegScaleSub(r.DSI[1].linear, V3GetY(sz), V3NegScaleSub(r.DSI[2].linear, V3GetZ(sz), lZ)));
			aZ = V3NegScaleSub(r.DSI[0].angular, V3GetX(sz), V3NegScaleSub(r.DSI[1].angular, V3GetY(sz), V3NegScaleSub(r.DSI[2].angular, V3GetZ(sz), aZ)));

			aZ = V3Add(aZ, V3Cross(j.parentOffset, lZ));
			indices[iCount++] = i;
		}

		const FsInertia& I = getRootInverseInertia(matrix);

		Vec3V lV = V3Neg(V3Add(M33MulV3(I.ll, lZ), M33MulV3(I.la, aZ)));
		Vec3V aV = V3Neg(V3Add(M33TrnspsMulV3(I.la, lZ), M33MulV3(I.aa, aZ)));

		while(iCount)
		{
			PxU32 i = indices[--iCount];
			const FsRow& r = rows[i];
			const FsJointVectors& j = jointVectors[i];

			lV = V3Sub(lV, V3Cross(j.parentOffset, aV));

			Vec3V n = V3Add(V3Merge(V3Dot(r.DSI[0].linear, lV),  V3Dot(r.DSI[1].linear, lV),  V3Dot(r.DSI[2].linear, lV)),
							V3Merge(V3Dot(r.DSI[0].angular, aV), V3Dot(r.DSI[1].angular, aV), V3Dot(r.DSI[2].angular, aV)));

			n = V3Add(n, M33MulV3(r.D, SZ[iCount]));
			lV = V3Sub(lV, V3Cross(j.jointOffset, n));
			aV = V3Sub(aV, n);
		}

		return Cm::SpatialVectorV(lV, aV);
	}
}
					
void ArticulationHelper::getImpulseResponse(const FsData& matrix,
											PxU32 linkID,
											const Cm::SpatialVectorV& impulse,
											Cm::SpatialVectorV& deltaV)
{
	PX_ASSERT(matrix.linkCount<=DY_ARTICULATION_MAX_SIZE);

	deltaV = getImpulseResponseSimd(matrix, linkID, impulse.linear, impulse.angular);

#if DY_ARTICULATION_DEBUG_VERIFY
	Cm::SpatialVectorV deltaV_;
	PxcFsGetImpulseResponse(matrix, linkID, impulse, deltaV_);
	PX_ASSERT(almostEqual(deltaV_, deltaV,1e-3f));
#endif
}

void ArticulationHelper::getImpulseSelfResponse(const FsData& matrix,
												PxU32 linkID0,
												const Cm::SpatialVectorV& impulse0,
												Cm::SpatialVectorV& deltaV0,
												PxU32 linkID1,
												const Cm::SpatialVectorV& impulse1,
												Cm::SpatialVectorV& deltaV1)
{
	PX_ASSERT(linkID0 != linkID1);

	const FsRow* rows = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	PX_UNUSED(aux);

	Cm::SpatialVectorV& dV0 = deltaV0, 
				  & dV1 = deltaV1;

	// standard case: parent-child limit
	if(matrix.parent[linkID1] == linkID0)
	{
		const FsRow& r = rows[linkID1];
		const FsJointVectors& j = jointVectors[linkID1];

		Vec3V lZ = V3Neg(impulse1.linear),
			  aZ = V3Neg(impulse1.angular);

		Vec3V sz = V3Add(aZ, V3Cross(lZ, j.jointOffset));
		
		lZ = V3Sub(lZ, V3ScaleAdd(r.DSI[0].linear, V3GetX(sz), V3ScaleAdd(r.DSI[1].linear, V3GetY(sz), V3Scale(r.DSI[2].linear, V3GetZ(sz)))));
		aZ = V3Sub(aZ, V3ScaleAdd(r.DSI[0].angular, V3GetX(sz), V3ScaleAdd(r.DSI[1].angular, V3GetY(sz), V3Scale(r.DSI[2].angular, V3GetZ(sz)))));

		aZ = V3Add(aZ, V3Cross(j.parentOffset, lZ));

		lZ = V3Sub(impulse0.linear, lZ);
		aZ = V3Sub(impulse0.angular, aZ);

		dV0 = getImpulseResponseSimd(matrix, linkID0, lZ, aZ);

		Vec3V aV = dV0.angular;
		Vec3V lV = V3Sub(dV0.linear, V3Cross(j.parentOffset, aV));

		Vec3V n = V3Add(V3Merge(V3Dot(r.DSI[0].linear, lV),  V3Dot(r.DSI[1].linear, lV),  V3Dot(r.DSI[2].linear, lV)),
						V3Merge(V3Dot(r.DSI[0].angular, aV), V3Dot(r.DSI[1].angular, aV), V3Dot(r.DSI[2].angular, aV)));

		n = V3Add(n, M33MulV3(r.D, sz));
		lV = V3Sub(lV, V3Cross(j.jointOffset, n));
		aV = V3Sub(aV, n);

		dV1 = Cm::SpatialVectorV(lV, aV);
	}
	else
	{
#if 0
		getImpulseResponseSlow(matrix, linkID0, impulse0, deltaV0, linkID1, impulse1, deltaV1);
#else
		getImpulseResponse(matrix, linkID0, impulse0, deltaV0);
		getImpulseResponse(matrix, linkID1, impulse1, deltaV1);
#endif
	}

#if DY_ARTICULATION_DEBUG_VERIFY
	Cm::SpatialVectorV dV0_, dV1_;
	PxcFsGetImpulseSelfResponse(matrix, linkID0, impulse0, dV0_, linkID1, impulse1, dV1_);

	PX_ASSERT(almostEqual(dV0_, dV0, 1e-3f));
	PX_ASSERT(almostEqual(dV1_, dV1, 1e-3f));
#endif
}

PxU32 ArticulationHelper::getFsDataSize(PxU32 linkCount)
{
	return sizeof(FsInertia) + sizeof(FsRow) * linkCount;
}

PxU32 ArticulationHelper::getLtbDataSize(PxU32 linkCount)
{
	return sizeof(LtbRow) * linkCount;
}


void ArticulationHelper::createHardLimit(	const FsData& fsData,
											const ArticulationLink* links,
											PxU32 linkIndex,
											SolverConstraint1DExt& s, 
											const PxVec3& axis, 
											PxReal err,
											PxReal recipDt)
{
	init(s, PxVec3(0), PxVec3(0), axis, axis, 0, PX_MAX_F32);

	ArticulationHelper::getImpulseSelfResponse(fsData, 
												  links[linkIndex].parent,Cm::SpatialVector(PxVec3(0), axis), s.deltaVA,
												  linkIndex, Cm::SpatialVector(PxVec3(0), -axis), s.deltaVB);

	const PxReal unitResponse = axis.dot(reinterpret_cast<PxVec3&>(s.deltaVA.angular)) - axis.dot(reinterpret_cast<PxVec3&>(s.deltaVB.angular));
	if(unitResponse<0.0f)
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Warning: articulation ill-conditioned or under severe stress, joint limit ignored");

	const PxReal recipResponse = unitResponse>0.0f ? 1.0f/unitResponse : 0.0f;

	s.constant = recipResponse * -err * recipDt;
	s.unbiasedConstant = err>0.0f ? s.constant : 0.0f;
	s.velMultiplier = -recipResponse;
	s.impulseMultiplier = 1.0f;
}

void ArticulationHelper::createTangentialSpring(const FsData& fsData,
												const ArticulationLink* links,
												PxU32 linkIndex,
												SolverConstraint1DExt& s, 
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
	if(unitResponse<0.0f)
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Warning: articulation ill-conditioned or under severe stress, tangential spring ignored");
	const PxReal recipResponse = unitResponse>0.0F ? 1.0f/unitResponse : 0.0f;

	// this is a specialization of the spring code in setSolverConstants() for acceleration springs.
	// general case is  b = dt * (c.mods.spring.damping * c.velocityTarget - c.mods.spring.stiffness * geomError);
    // but geomError and velocityTarget are both zero

	const PxReal a = dt * dt * stiffness + dt * damping;
    const PxReal x = 1.0f/(1.0f+a);
    s.constant = s.unbiasedConstant = 0.0f;
    s.velMultiplier = -x * recipResponse * a;
    s.impulseMultiplier = 1.0f - x;
}

PxU32 ArticulationHelper::setupSolverConstraints(	Articulation& articulation, PxU32 solverDataSize,
													PxConstraintAllocator& allocator,
													PxSolverConstraintDesc* constraintDesc,
													const ArticulationLink* links,
													const ArticulationJointTransforms* jointTransforms,
													PxReal dt,
													PxU32& acCount)
{
	acCount = 0;

	FsData& fsData = *articulation.getFsDataPtr();
	const PxU16 linkCount = fsData.linkCount;
	PxU32 descCount = 0;
	const PxReal recipDt = 1.0f/dt;

	const PxConstraintInvMassScale ims(1.0f, 1.0f, 1.0f, 1.0f);

	for(PxU16 i=1;i<linkCount;i++)
	{
		const ArticulationJointCore& j = static_cast<const ArticulationJointCore&>(*links[i].inboundJoint);

		if(i+1<linkCount)
		{
			Ps::prefetch(links[i+1].inboundJoint, sizeof (ArticulationJointCore));
			Ps::prefetch(&jointTransforms[i+1], sizeof(ArticulationJointTransforms));
		}
		
		if(!(j.twistLimited || j.swingLimited))
			continue;

		PxQuat swing, twist;
		Ps::separateSwingTwist(jointTransforms[i].cB2cA.q, swing, twist);
	
		Cm::ConeLimitHelper eh(j.tanQSwingY, j.tanQSwingZ, j.tanQSwingPad);
		PxVec3 swingLimitAxis;
		PxReal swingLimitError = 0.0f;

		const bool swingLimited = j.swingLimited && eh.getLimit(swing, swingLimitAxis, swingLimitError);
		const bool tangentialStiffness = swingLimited && (j.tangentialStiffness>0 || j.tangentialDamping>0);

		const PxVec3 twistAxis = jointTransforms[i].cB2w.rotate(PxVec3(1.0f,0,0));
		const PxReal tqTwistAngle = Ps::tanHalf(twist.x, twist.w);

		const bool twistLowerLimited = j.twistLimited && tqTwistAngle < Cm::tanAdd(j.tanQTwistLow, j.tanQTwistPad);
		const bool twistUpperLimited = j.twistLimited && tqTwistAngle > Cm::tanAdd(j.tanQTwistHigh, -j.tanQTwistPad);
	
		const PxU8 constraintCount = PxU8(swingLimited + tangentialStiffness + twistUpperLimited + twistLowerLimited);
		if(!constraintCount)
			continue;

		PxSolverConstraintDesc& desc = constraintDesc[descCount++];

		desc.articulationA = &articulation;
		desc.linkIndexA = Ps::to16(links[i].parent);
		desc.articulationALength = Ps::to16(solverDataSize);

		desc.articulationB = &articulation;
		desc.linkIndexB = i;
		desc.articulationBLength = Ps::to16(solverDataSize);

		const PxU32 constraintLength = sizeof(SolverConstraint1DHeader) + 
								 sizeof(SolverConstraint1DExt) * constraintCount;

		PX_ASSERT(0==(constraintLength & 0x0f));
		desc.constraintLengthOver16 = Ps::to16(constraintLength/16);
		
		//desc.constraint = stream.reserve(constraintLength + 16u, constraintBlockManager);
		desc.constraint = allocator.reserveConstraintData(constraintLength + 16u);

		desc.writeBack = NULL;
		
		SolverConstraint1DHeader* header = reinterpret_cast<SolverConstraint1DHeader*>(desc.constraint);
		SolverConstraint1DExt* constraints = reinterpret_cast<SolverConstraint1DExt*>(desc.constraint + sizeof(SolverConstraint1DHeader));

		init(*header, constraintCount, true, ims);

		PxU32 cIndex = 0;

		if(swingLimited)
		{
			const PxVec3 normal = jointTransforms[i].cA2w.rotate(swingLimitAxis);
			createHardLimit(fsData, links, i, constraints[cIndex++], normal, swingLimitError, recipDt);
			if(tangentialStiffness)
			{
				const PxVec3 tangent = twistAxis.cross(normal).getNormalized();
				createTangentialSpring(fsData, links, i, constraints[cIndex++], tangent, j.tangentialStiffness, j.tangentialDamping, dt);
			}
		}

		if(twistUpperLimited)
			createHardLimit(fsData, links, i, constraints[cIndex++], twistAxis, (j.tanQTwistHigh - tqTwistAngle)*4, recipDt);

		if(twistLowerLimited)
			createHardLimit(fsData, links, i, constraints[cIndex++], -twistAxis, -(j.tanQTwistLow - tqTwistAngle)*4, recipDt);

		*(desc.constraint + getConstraintLength(desc)) = 0;

		PX_ASSERT(cIndex == constraintCount);
		acCount += constraintCount;
	}

	return descCount;
}


ArticulationPImpl::ComputeUnconstrainedVelocitiesFn ArticulationPImpl::sComputeUnconstrainedVelocities[2] = { NULL, NULL };
ArticulationPImpl::UpdateBodiesFn ArticulationPImpl::sUpdateBodies[2] = { NULL, NULL };
ArticulationPImpl::UpdateBodiesFn ArticulationPImpl::sUpdateBodiesTGS[2] = { NULL, NULL };
ArticulationPImpl::SaveVelocityFn ArticulationPImpl::sSaveVelocity[2] = { NULL, NULL };
ArticulationPImpl::SaveVelocityTGSFn ArticulationPImpl::sSaveVelocityTGS[2] = { NULL, NULL };

ArticulationPImpl::UpdateDeltaMotionFn ArticulationPImpl::sUpdateDeltaMotion[2] = { NULL, NULL };
ArticulationPImpl::DeltaMotionToMotionVelFn ArticulationPImpl::sDeltaMotionToMotionVel[2] = { NULL, NULL };
ArticulationPImpl::ComputeUnconstrainedVelocitiesTGSFn ArticulationPImpl::sComputeUnconstrainedVelocitiesTGS[2] = { NULL, NULL };

ArticulationPImpl::SetupInternalConstraintsTGSFn ArticulationPImpl::sSetupInternalConstraintsTGS[2] = { NULL, NULL };

}
}
