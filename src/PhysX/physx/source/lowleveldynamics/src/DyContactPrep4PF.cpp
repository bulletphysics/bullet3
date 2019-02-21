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
#include "PsVecMath.h"
#include "PsMathUtils.h"
#include "DySolverContact.h"
#include "DySolverContactPF.h"
#include "DySolverConstraintTypes.h"
#include "PxcNpWorkUnit.h"
#include "DyThreadContext.h"
#include "DyContactPrep.h"
#include "PxcNpContactPrepShared.h"
//#include "PxvGeometry.h"
#include "PxvDynamics.h"
#include "DyCorrelationBuffer.h"
#include "DySolverConstraintDesc.h"
#include "DySolverBody.h"
#include "DySolverContact4.h"
#include "DySolverContactPF4.h"


#include "PsVecMath.h"
#include "PxContactModifyCallback.h"
#include "PxsMaterialManager.h"
#include "PxsMaterialCombiner.h"
#include "DySolverExt.h"
#include "DyArticulationContactPrep.h"
#include "DyContactPrepShared.h"
#include "PsFoundation.h"

using namespace physx::Gu;
using namespace physx::shdfnd::aos;

namespace physx
{
namespace Dy
{

SolverConstraintPrepState::Enum createFinalizeSolverContacts4Coulomb(
		PxsContactManagerOutput** outputs,
		ThreadContext& threadContext,
		PxSolverContactDesc* blockDescs,
		const PxReal invDtF32,
		PxReal bounceThresholdF32,
		PxReal frictionOffsetThreshold,
		PxReal correlationDistance,
		PxReal solverOffsetSlop,
		PxConstraintAllocator& constraintAllocator,
		PxFrictionType::Enum frictionType);

static bool setupFinalizeSolverConstraintsCoulomb4(PxSolverContactDesc* PX_RESTRICT descs, PxU8* PX_RESTRICT workspace, 
											const PxReal invDtF32, PxReal bounceThresholdF32, PxReal solverOffsetSlopF32, CorrelationBuffer& c, const PxU32 numFrictionPerPoint,
											const PxU32 numContactPoints4, const PxU32 /*solverConstraintByteSize*/,
											const Ps::aos::Vec4VArg invMassScale0, const Ps::aos::Vec4VArg invInertiaScale0, 
											const Ps::aos::Vec4VArg invMassScale1, const Ps::aos::Vec4VArg invInertiaScale1)
{
	//KS - final step. Create the constraints in the place we pre-allocated...

	const Vec4V ccdMaxSeparation = Ps::aos::V4LoadXYZW(descs[0].maxCCDSeparation, descs[1].maxCCDSeparation, descs[2].maxCCDSeparation, descs[3].maxCCDSeparation);
	const Vec4V solverOffsetSlop = V4Load(solverOffsetSlopF32);

	const Vec4V zero = V4Zero();

	PxU8 flags[4] = {	PxU8(descs[0].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
						PxU8(descs[1].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
						PxU8(descs[2].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
						PxU8(descs[3].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0) };


	//The block is dynamic if **any** of the constraints have a non-static body B. This allows us to batch static and non-static constraints but we only get a memory/perf
	//saving if all 4 are static. This simplifies the constraint partitioning such that it only needs to care about separating contacts and 1D constraints (which it already does)
	const bool isDynamic = ((descs[0].bodyState1 | descs[1].bodyState1 | descs[2].bodyState1 | descs[3].bodyState1) & PxSolverContactDesc::eDYNAMIC_BODY) != 0;

	const PxU32 constraintSize = isDynamic ? sizeof(SolverContact4Dynamic) : sizeof(SolverContact4Base);
	const PxU32 frictionSize = isDynamic ? sizeof(SolverFriction4Dynamic) : sizeof(SolverFriction4Base);

	PxU8* PX_RESTRICT ptr = workspace;

	const Vec4V dom0 = invMassScale0;
	const Vec4V dom1 = invMassScale1;
	const Vec4V angDom0 = invInertiaScale0;
	const Vec4V angDom1 = invInertiaScale1;

	const Vec4V maxPenBias = V4Max(V4Merge(FLoad(descs[0].data0->penBiasClamp), FLoad(descs[1].data0->penBiasClamp), 
		FLoad(descs[2].data0->penBiasClamp), FLoad(descs[3].data0->penBiasClamp)), 
		V4Merge(FLoad(descs[0].data1->penBiasClamp), FLoad(descs[1].data1->penBiasClamp), 
		FLoad(descs[2].data1->penBiasClamp), FLoad(descs[3].data1->penBiasClamp)));

	const Vec4V restDistance = V4Merge(FLoad(descs[0].restDistance), FLoad(descs[1].restDistance), FLoad(descs[2].restDistance),
		FLoad(descs[3].restDistance)); 

	//load up velocities
	Vec4V linVel00 = V4LoadA(&descs[0].data0->linearVelocity.x);
	Vec4V linVel10 = V4LoadA(&descs[1].data0->linearVelocity.x);
	Vec4V linVel20 = V4LoadA(&descs[2].data0->linearVelocity.x);
	Vec4V linVel30 = V4LoadA(&descs[3].data0->linearVelocity.x);

	Vec4V linVel01 = V4LoadA(&descs[0].data1->linearVelocity.x);
	Vec4V linVel11 = V4LoadA(&descs[1].data1->linearVelocity.x);
	Vec4V linVel21 = V4LoadA(&descs[2].data1->linearVelocity.x);
	Vec4V linVel31 = V4LoadA(&descs[3].data1->linearVelocity.x);

	Vec4V angVel00 = V4LoadA(&descs[0].data0->angularVelocity.x);
	Vec4V angVel10 = V4LoadA(&descs[1].data0->angularVelocity.x);
	Vec4V angVel20 = V4LoadA(&descs[2].data0->angularVelocity.x);
	Vec4V angVel30 = V4LoadA(&descs[3].data0->angularVelocity.x);

	Vec4V angVel01 = V4LoadA(&descs[0].data1->angularVelocity.x);
	Vec4V angVel11 = V4LoadA(&descs[1].data1->angularVelocity.x);
	Vec4V angVel21 = V4LoadA(&descs[2].data1->angularVelocity.x);
	Vec4V angVel31 = V4LoadA(&descs[3].data1->angularVelocity.x);

	Vec4V linVelT00, linVelT10, linVelT20;
	Vec4V linVelT01, linVelT11, linVelT21;
	Vec4V angVelT00, angVelT10, angVelT20;
	Vec4V angVelT01, angVelT11, angVelT21;

	PX_TRANSPOSE_44_34(linVel00, linVel10, linVel20, linVel30, linVelT00, linVelT10, linVelT20);
	PX_TRANSPOSE_44_34(linVel01, linVel11, linVel21, linVel31, linVelT01, linVelT11, linVelT21);
	PX_TRANSPOSE_44_34(angVel00, angVel10, angVel20, angVel30, angVelT00, angVelT10, angVelT20);
	PX_TRANSPOSE_44_34(angVel01, angVel11, angVel21, angVel31, angVelT01, angVelT11, angVelT21);

	const Vec4V vrelX = V4Sub(linVelT00, linVelT01);
	const Vec4V vrelY = V4Sub(linVelT10, linVelT11);
	const Vec4V vrelZ = V4Sub(linVelT20, linVelT21);



	//Load up masses and invInertia

	const Vec4V invMass0 = V4Merge(FLoad(descs[0].data0->invMass), FLoad(descs[1].data0->invMass), FLoad(descs[2].data0->invMass),
		FLoad(descs[3].data0->invMass));

	const Vec4V invMass1 = V4Merge(FLoad(descs[0].data1->invMass), FLoad(descs[1].data1->invMass), FLoad(descs[2].data1->invMass),
		FLoad(descs[3].data1->invMass));

	const Vec4V invMass0_dom0fV = V4Mul(dom0, invMass0);
	const Vec4V invMass1_dom1fV = V4Mul(dom1, invMass1);

	Vec4V invInertia00X = Vec4V_From_Vec3V(V3LoadU(descs[0].data0->sqrtInvInertia.column0));
	Vec4V invInertia00Y = Vec4V_From_Vec3V(V3LoadU(descs[0].data0->sqrtInvInertia.column1));
	Vec4V invInertia00Z = Vec4V_From_Vec3V(V3LoadU(descs[0].data0->sqrtInvInertia.column2));

	Vec4V invInertia10X = Vec4V_From_Vec3V(V3LoadU(descs[1].data0->sqrtInvInertia.column0));
	Vec4V invInertia10Y = Vec4V_From_Vec3V(V3LoadU(descs[1].data0->sqrtInvInertia.column1));
	Vec4V invInertia10Z = Vec4V_From_Vec3V(V3LoadU(descs[1].data0->sqrtInvInertia.column2));

	Vec4V invInertia20X = Vec4V_From_Vec3V(V3LoadU(descs[2].data0->sqrtInvInertia.column0));
	Vec4V invInertia20Y = Vec4V_From_Vec3V(V3LoadU(descs[2].data0->sqrtInvInertia.column1));
	Vec4V invInertia20Z = Vec4V_From_Vec3V(V3LoadU(descs[2].data0->sqrtInvInertia.column2));

	Vec4V invInertia30X = Vec4V_From_Vec3V(V3LoadU(descs[3].data0->sqrtInvInertia.column0));
	Vec4V invInertia30Y = Vec4V_From_Vec3V(V3LoadU(descs[3].data0->sqrtInvInertia.column1));
	Vec4V invInertia30Z = Vec4V_From_Vec3V(V3LoadU(descs[3].data0->sqrtInvInertia.column2));

	Vec4V invInertia01X = Vec4V_From_Vec3V(V3LoadU(descs[0].data1->sqrtInvInertia.column0));
	Vec4V invInertia01Y = Vec4V_From_Vec3V(V3LoadU(descs[0].data1->sqrtInvInertia.column1));
	Vec4V invInertia01Z = Vec4V_From_Vec3V(V3LoadU(descs[0].data1->sqrtInvInertia.column2));

	Vec4V invInertia11X = Vec4V_From_Vec3V(V3LoadU(descs[1].data1->sqrtInvInertia.column0));
	Vec4V invInertia11Y = Vec4V_From_Vec3V(V3LoadU(descs[1].data1->sqrtInvInertia.column1));
	Vec4V invInertia11Z = Vec4V_From_Vec3V(V3LoadU(descs[1].data1->sqrtInvInertia.column2));

	Vec4V invInertia21X = Vec4V_From_Vec3V(V3LoadU(descs[2].data1->sqrtInvInertia.column0));
	Vec4V invInertia21Y = Vec4V_From_Vec3V(V3LoadU(descs[2].data1->sqrtInvInertia.column1));
	Vec4V invInertia21Z = Vec4V_From_Vec3V(V3LoadU(descs[2].data1->sqrtInvInertia.column2));

	Vec4V invInertia31X = Vec4V_From_Vec3V(V3LoadU(descs[3].data1->sqrtInvInertia.column0));
	Vec4V invInertia31Y = Vec4V_From_Vec3V(V3LoadU(descs[3].data1->sqrtInvInertia.column1));
	Vec4V invInertia31Z = Vec4V_From_Vec3V(V3LoadU(descs[3].data1->sqrtInvInertia.column2));

	Vec4V invInertia0X0, invInertia0X1, invInertia0X2;
	Vec4V invInertia0Y0, invInertia0Y1, invInertia0Y2;
	Vec4V invInertia0Z0, invInertia0Z1, invInertia0Z2;

	Vec4V invInertia1X0, invInertia1X1, invInertia1X2;
	Vec4V invInertia1Y0, invInertia1Y1, invInertia1Y2;
	Vec4V invInertia1Z0, invInertia1Z1, invInertia1Z2;

	PX_TRANSPOSE_44_34(invInertia00X, invInertia10X, invInertia20X, invInertia30X, invInertia0X0, invInertia0Y0, invInertia0Z0);
	PX_TRANSPOSE_44_34(invInertia00Y, invInertia10Y, invInertia20Y, invInertia30Y, invInertia0X1, invInertia0Y1, invInertia0Z1);
	PX_TRANSPOSE_44_34(invInertia00Z, invInertia10Z, invInertia20Z, invInertia30Z, invInertia0X2, invInertia0Y2, invInertia0Z2);

	PX_TRANSPOSE_44_34(invInertia01X, invInertia11X, invInertia21X, invInertia31X, invInertia1X0, invInertia1Y0, invInertia1Z0);
	PX_TRANSPOSE_44_34(invInertia01Y, invInertia11Y, invInertia21Y, invInertia31Y, invInertia1X1, invInertia1Y1, invInertia1Z1);
	PX_TRANSPOSE_44_34(invInertia01Z, invInertia11Z, invInertia21Z, invInertia31Z, invInertia1X2, invInertia1Y2, invInertia1Z2);

	const FloatV invDt = FLoad(invDtF32);
	const FloatV p8 = FLoad(0.8f);
	//const Vec4V p84 = V4Splat(p8);
	const Vec4V p1 = V4Splat(FLoad(0.1f));
	const Vec4V bounceThreshold = V4Splat(FLoad(bounceThresholdF32));
	const Vec4V orthoThreshold = V4Splat(FLoad(0.70710678f));

	const FloatV invDtp8 = FMul(invDt, p8);

	const Vec3V bodyFrame00p = V3LoadU(descs[0].bodyFrame0.p);
	const Vec3V bodyFrame01p = V3LoadU(descs[1].bodyFrame0.p);
	const Vec3V bodyFrame02p = V3LoadU(descs[2].bodyFrame0.p);
	const Vec3V bodyFrame03p = V3LoadU(descs[3].bodyFrame0.p);

	Vec4V bodyFrame00p4 = Vec4V_From_Vec3V(bodyFrame00p);
	Vec4V bodyFrame01p4 = Vec4V_From_Vec3V(bodyFrame01p);
	Vec4V bodyFrame02p4 = Vec4V_From_Vec3V(bodyFrame02p);
	Vec4V bodyFrame03p4 = Vec4V_From_Vec3V(bodyFrame03p);

	Vec4V bodyFrame0pX, bodyFrame0pY, bodyFrame0pZ;
	PX_TRANSPOSE_44_34(bodyFrame00p4, bodyFrame01p4, bodyFrame02p4, bodyFrame03p4, bodyFrame0pX, bodyFrame0pY, bodyFrame0pZ);

	
	const Vec3V bodyFrame10p = V3LoadU(descs[0].bodyFrame1.p);
	const Vec3V bodyFrame11p = V3LoadU(descs[1].bodyFrame1.p);
	const Vec3V bodyFrame12p = V3LoadU(descs[2].bodyFrame1.p);
	const Vec3V bodyFrame13p = V3LoadU(descs[3].bodyFrame1.p);

	Vec4V bodyFrame10p4 = Vec4V_From_Vec3V(bodyFrame10p);
	Vec4V bodyFrame11p4 = Vec4V_From_Vec3V(bodyFrame11p);
	Vec4V bodyFrame12p4 = Vec4V_From_Vec3V(bodyFrame12p);
	Vec4V bodyFrame13p4 = Vec4V_From_Vec3V(bodyFrame13p);

	Vec4V bodyFrame1pX, bodyFrame1pY, bodyFrame1pZ;
	PX_TRANSPOSE_44_34(bodyFrame10p4, bodyFrame11p4, bodyFrame12p4, bodyFrame13p4, bodyFrame1pX, bodyFrame1pY, bodyFrame1pZ);

	
	Ps::prefetchLine(c.contactID);
	Ps::prefetchLine(c.contactID, 128);

	PxU32 frictionIndex0 = 0, frictionIndex1 = 0, frictionIndex2 = 0, frictionIndex3 = 0;


	PxU32 maxPatches = PxMax(descs[0].numFrictionPatches, PxMax(descs[1].numFrictionPatches, PxMax(descs[2].numFrictionPatches, descs[3].numFrictionPatches)));
	PxU32 maxContacts = numContactPoints4;

	//This is the address at which the first friction patch exists
	PxU8* ptr2 = ptr + ((sizeof(SolverContactCoulombHeader4) * maxPatches) + constraintSize * maxContacts);

	//PxU32 contactId = 0;

	for(PxU32 i=0;i<maxPatches;i++)
	{
		const bool hasFinished0 = i >= descs[0].numFrictionPatches;
		const bool hasFinished1 = i >= descs[1].numFrictionPatches;
		const bool hasFinished2 = i >= descs[2].numFrictionPatches;
		const bool hasFinished3 = i >= descs[3].numFrictionPatches;


		frictionIndex0 = hasFinished0 ? frictionIndex0 : descs[0].startFrictionPatchIndex + i;
		frictionIndex1 = hasFinished1 ? frictionIndex1 : descs[1].startFrictionPatchIndex + i;
		frictionIndex2 = hasFinished2 ? frictionIndex2 : descs[2].startFrictionPatchIndex + i;
		frictionIndex3 = hasFinished3 ? frictionIndex3 : descs[3].startFrictionPatchIndex + i;

		PxU32 clampedContacts0 = hasFinished0 ? 0 : c.frictionPatchContactCounts[frictionIndex0];
		PxU32 clampedContacts1 = hasFinished1 ? 0 : c.frictionPatchContactCounts[frictionIndex1];
		PxU32 clampedContacts2 = hasFinished2 ? 0 : c.frictionPatchContactCounts[frictionIndex2];
		PxU32 clampedContacts3 = hasFinished3 ? 0 : c.frictionPatchContactCounts[frictionIndex3];

		PxU32 clampedFric0 = clampedContacts0 * numFrictionPerPoint;
		PxU32 clampedFric1 = clampedContacts1 * numFrictionPerPoint;
		PxU32 clampedFric2 = clampedContacts2 * numFrictionPerPoint;
		PxU32 clampedFric3 = clampedContacts3 * numFrictionPerPoint;


		const PxU32 numContacts = PxMax(clampedContacts0, PxMax(clampedContacts1, PxMax(clampedContacts2, clampedContacts3)));
		const PxU32 numFrictions = PxMax(clampedFric0, PxMax(clampedFric1, PxMax(clampedFric2, clampedFric3)));

		PxU32 firstPatch0 = c.correlationListHeads[frictionIndex0];
		PxU32 firstPatch1 = c.correlationListHeads[frictionIndex1];
		PxU32 firstPatch2 = c.correlationListHeads[frictionIndex2];
		PxU32 firstPatch3 = c.correlationListHeads[frictionIndex3];

		const Gu::ContactPoint* contactBase0 = descs[0].contacts + c.contactPatches[firstPatch0].start;
		const Gu::ContactPoint* contactBase1 = descs[1].contacts + c.contactPatches[firstPatch1].start;
		const Gu::ContactPoint* contactBase2 = descs[2].contacts + c.contactPatches[firstPatch2].start;
		const Gu::ContactPoint* contactBase3 = descs[3].contacts + c.contactPatches[firstPatch3].start;

		const Vec4V restitution = V4Merge(FLoad(contactBase0->restitution), FLoad(contactBase1->restitution), FLoad(contactBase2->restitution),
			FLoad(contactBase3->restitution));

		const Vec4V staticFriction = V4Merge(FLoad(contactBase0->staticFriction), FLoad(contactBase1->staticFriction), FLoad(contactBase2->staticFriction),
			FLoad(contactBase3->staticFriction));

		SolverContactCoulombHeader4* PX_RESTRICT header = reinterpret_cast<SolverContactCoulombHeader4*>(ptr);

		header->frictionOffset = PxU16(ptr2 - ptr);

		ptr += sizeof(SolverContactCoulombHeader4);	

		SolverFrictionHeader4* PX_RESTRICT fricHeader = reinterpret_cast<SolverFrictionHeader4*>(ptr2);
		ptr2 += sizeof(SolverFrictionHeader4) + sizeof(Vec4V) * numContacts;


		header->numNormalConstr0 = Ps::to8(clampedContacts0);
		header->numNormalConstr1 = Ps::to8(clampedContacts1);
		header->numNormalConstr2 = Ps::to8(clampedContacts2);
		header->numNormalConstr3 = Ps::to8(clampedContacts3);
		header->numNormalConstr = Ps::to8(numContacts);
		header->invMassADom = invMass0_dom0fV;
		header->invMassBDom = invMass1_dom1fV;
		header->angD0 = angDom0;
		header->angD1 = angDom1;
		header->restitution = restitution;

		header->flags[0] = flags[0]; header->flags[1] = flags[1]; header->flags[2] = flags[2]; header->flags[3] = flags[3];

		header->type = Ps::to8(isDynamic ? DY_SC_TYPE_BLOCK_RB_CONTACT : DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT);
		header->shapeInteraction[0] = descs[0].shapeInteraction; header->shapeInteraction[1] = descs[1].shapeInteraction;
		header->shapeInteraction[2] = descs[2].shapeInteraction; header->shapeInteraction[3] = descs[3].shapeInteraction;


		fricHeader->invMassADom = invMass0_dom0fV;
		fricHeader->invMassBDom = invMass1_dom1fV;
		fricHeader->angD0 = angDom0;
		fricHeader->angD1 = angDom1;
		fricHeader->numFrictionConstr0 = Ps::to8(clampedFric0);
		fricHeader->numFrictionConstr1 = Ps::to8(clampedFric1);
		fricHeader->numFrictionConstr2 = Ps::to8(clampedFric2);
		fricHeader->numFrictionConstr3 = Ps::to8(clampedFric3);
		fricHeader->numNormalConstr = Ps::to8(numContacts);
		fricHeader->numNormalConstr0 = Ps::to8(clampedContacts0);
		fricHeader->numNormalConstr1 = Ps::to8(clampedContacts1);
		fricHeader->numNormalConstr2 = Ps::to8(clampedContacts2);
		fricHeader->numNormalConstr3 = Ps::to8(clampedContacts3);
		fricHeader->type = Ps::to8(isDynamic ? DY_SC_TYPE_BLOCK_FRICTION : DY_SC_TYPE_BLOCK_STATIC_FRICTION);
		fricHeader->staticFriction = staticFriction;
		fricHeader->frictionPerContact = PxU32(numFrictionPerPoint == 2 ? 1 : 0);

		fricHeader->numFrictionConstr = Ps::to8(numFrictions);
		
		Vec4V normal0 = V4LoadA(&contactBase0->normal.x);
		Vec4V normal1 = V4LoadA(&contactBase1->normal.x);
		Vec4V normal2 = V4LoadA(&contactBase2->normal.x);
		Vec4V normal3 = V4LoadA(&contactBase3->normal.x);

		Vec4V normalX, normalY, normalZ;
		PX_TRANSPOSE_44_34(normal0, normal1, normal2, normal3, normalX, normalY, normalZ);
		header->normalX = normalX;
		header->normalY = normalY;
		header->normalZ = normalZ;

		const Vec4V normalLenSq = V4MulAdd(normalZ, normalZ, V4MulAdd(normalY, normalY, V4Mul(normalX, normalX)));

		const Vec4V linNorVel0 = V4MulAdd(normalZ, linVelT20, V4MulAdd(normalY, linVelT10, V4Mul(normalX, linVelT00)));
		const Vec4V linNorVel1 = V4MulAdd(normalZ, linVelT21, V4MulAdd(normalY, linVelT11, V4Mul(normalX, linVelT01)));

		const Vec4V invMassNorLenSq0 = V4Mul(invMass0_dom0fV, normalLenSq);
		const Vec4V invMassNorLenSq1 = V4Mul(invMass1_dom1fV, normalLenSq);		


		//Calculate friction directions
		const BoolV cond =V4IsGrtr(orthoThreshold, V4Abs(normalX));

		const Vec4V t0FallbackX = V4Sel(cond, zero, V4Neg(normalY));
		const Vec4V t0FallbackY = V4Sel(cond, V4Neg(normalZ), normalX);
		const Vec4V t0FallbackZ = V4Sel(cond, normalY, zero);

		const Vec4V dotNormalVrel = V4MulAdd(normalZ, vrelZ, V4MulAdd(normalY, vrelY, V4Mul(normalX, vrelX)));
		const Vec4V vrelSubNorVelX = V4NegMulSub(normalX, dotNormalVrel, vrelX);
		const Vec4V vrelSubNorVelY = V4NegMulSub(normalY, dotNormalVrel, vrelY);
		const Vec4V vrelSubNorVelZ = V4NegMulSub(normalZ, dotNormalVrel, vrelZ);

		const Vec4V lenSqvrelSubNorVelZ = V4MulAdd(vrelSubNorVelX, vrelSubNorVelX, V4MulAdd(vrelSubNorVelY, vrelSubNorVelY, V4Mul(vrelSubNorVelZ, vrelSubNorVelZ)));

		const BoolV bcon2 = V4IsGrtr(lenSqvrelSubNorVelZ, p1);

		Vec4V t0X = V4Sel(bcon2, vrelSubNorVelX, t0FallbackX);
		Vec4V t0Y = V4Sel(bcon2, vrelSubNorVelY, t0FallbackY);
		Vec4V t0Z = V4Sel(bcon2, vrelSubNorVelZ, t0FallbackZ);

		//Now normalize this...
		const Vec4V recipLen = V4Rsqrt(V4MulAdd(t0X, t0X, V4MulAdd(t0Y, t0Y, V4Mul(t0Z, t0Z))));

		t0X = V4Mul(t0X, recipLen);
		t0Y = V4Mul(t0Y, recipLen);
		t0Z = V4Mul(t0Z, recipLen);

		const Vec4V t1X = V4NegMulSub(normalZ, t0Y, V4Mul(normalY, t0Z));
		const Vec4V t1Y = V4NegMulSub(normalX, t0Z, V4Mul(normalZ, t0X));
		const Vec4V t1Z = V4NegMulSub(normalY, t0X, V4Mul(normalX, t0Y));

		const Vec4V tFallbackX[2] = {t0X, t1X};
		const Vec4V tFallbackY[2] = {t0Y, t1Y};
		const Vec4V tFallbackZ[2] = {t0Z, t1Z};


		//For all correlation heads - need to pull this out I think

		//OK, we have a counter for all our patches...
		PxU32 finished = (PxU32(hasFinished0)) | 
						 ((PxU32(hasFinished1)) << 1) | 
						 ((PxU32(hasFinished2)) << 2) | 
						 ((PxU32(hasFinished3)) << 3);

		CorrelationListIterator iter0(c, firstPatch0);
		CorrelationListIterator iter1(c, firstPatch1);
		CorrelationListIterator iter2(c, firstPatch2);
		CorrelationListIterator iter3(c, firstPatch3);

		PxU32 contact0, contact1, contact2, contact3;
		PxU32 patch0, patch1, patch2, patch3;

		iter0.nextContact(patch0, contact0);
		iter1.nextContact(patch1, contact1);
		iter2.nextContact(patch2, contact2);
		iter3.nextContact(patch3, contact3);

		PxU8* p = ptr;

		PxU32 contactCount = 0;
		PxU32 newFinished = 
			(PxU32(hasFinished0 || !iter0.hasNextContact()))		| 
			((PxU32(hasFinished1 || !iter1.hasNextContact())) << 1) | 
			((PxU32(hasFinished2 || !iter2.hasNextContact())) << 2) | 
			((PxU32(hasFinished3 || !iter3.hasNextContact())) << 3);

		PxU32 fricIndex = 0;

		while(finished != 0xf)
		{
			finished = newFinished;
			++contactCount;
			Ps::prefetchLine(p, 384);
			Ps::prefetchLine(p, 512);
			Ps::prefetchLine(p, 640);	

			SolverContact4Base* PX_RESTRICT solverContact = reinterpret_cast<SolverContact4Base*>(p);
			p += constraintSize;

			const Gu::ContactPoint& con0 = descs[0].contacts[c.contactPatches[patch0].start + contact0];
			const Gu::ContactPoint& con1 = descs[1].contacts[c.contactPatches[patch1].start + contact1];
			const Gu::ContactPoint& con2 = descs[2].contacts[c.contactPatches[patch2].start + contact2];
			const Gu::ContactPoint& con3 = descs[3].contacts[c.contactPatches[patch3].start + contact3];

			//Now we need to splice these 4 contacts into a single structure

			{
				Vec4V point0 = V4LoadA(&con0.point.x);
				Vec4V point1 = V4LoadA(&con1.point.x);
				Vec4V point2 = V4LoadA(&con2.point.x);
				Vec4V point3 = V4LoadA(&con3.point.x);

				Vec4V pointX, pointY, pointZ;
				PX_TRANSPOSE_44_34(point0, point1, point2, point3, pointX, pointY, pointZ);

				Vec4V targetVel0 = V4LoadA(&con0.targetVel.x);
				Vec4V targetVel1 = V4LoadA(&con1.targetVel.x);
				Vec4V targetVel2 = V4LoadA(&con2.targetVel.x);
				Vec4V targetVel3 = V4LoadA(&con3.targetVel.x);

				Vec4V targetVelX, targetVelY, targetVelZ;
				PX_TRANSPOSE_44_34(targetVel0, targetVel1, targetVel2, targetVel3, targetVelX, targetVelY, targetVelZ);

				Vec4V raX = V4Sub(pointX, bodyFrame0pX);
				Vec4V raY = V4Sub(pointY, bodyFrame0pY);
				Vec4V raZ = V4Sub(pointZ, bodyFrame0pZ);

				Vec4V rbX = V4Sub(pointX, bodyFrame1pX);
				Vec4V rbY = V4Sub(pointY, bodyFrame1pY);
				Vec4V rbZ = V4Sub(pointZ, bodyFrame1pZ);

				raX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raX)), zero, raX);
				raY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raY)), zero, raY);
				raZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raZ)), zero, raZ);

				rbX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbX)), zero, rbX);
				rbY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbY)), zero, rbY);
				rbZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbZ)), zero, rbZ);

				{
					const Vec4V separation = V4Merge(FLoad(con0.separation), FLoad(con1.separation), FLoad(con2.separation),
						FLoad(con3.separation));
					const Vec4V maxImpulse = V4Merge(FLoad(con0.maxImpulse), FLoad(con1.maxImpulse), FLoad(con2.maxImpulse),
						FLoad(con3.maxImpulse));

					const Vec4V cTargetVel = V4MulAdd(normalX, targetVelX, V4MulAdd(normalY, targetVelY, V4Mul(normalZ, targetVelZ)));

					//raXn = cross(ra, normal) which = Vec3V( a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
					const Vec4V raXnX = V4NegMulSub(raZ, normalY, V4Mul(raY, normalZ));
					const Vec4V raXnY = V4NegMulSub(raX, normalZ, V4Mul(raZ, normalX));
					const Vec4V raXnZ = V4NegMulSub(raY, normalX, V4Mul(raX, normalY));

					const Vec4V v0a0 = V4Mul(invInertia0X0, raXnX);
					const Vec4V v0a1 = V4Mul(invInertia0X1, raXnX);
					const Vec4V v0a2 = V4Mul(invInertia0X2, raXnX);

					const Vec4V v0PlusV1a0 = V4MulAdd(invInertia0Y0, raXnY, v0a0);
					const Vec4V v0PlusV1a1 = V4MulAdd(invInertia0Y1, raXnY, v0a1);
					const Vec4V v0PlusV1a2 = V4MulAdd(invInertia0Y2, raXnY, v0a2);

					const Vec4V delAngVel0X = V4MulAdd(invInertia0Z0, raXnZ, v0PlusV1a0);
					const Vec4V delAngVel0Y = V4MulAdd(invInertia0Z1, raXnZ, v0PlusV1a1);
					const Vec4V delAngVel0Z = V4MulAdd(invInertia0Z2, raXnZ, v0PlusV1a2);

					const Vec4V dotDelAngVel0 = V4MulAdd(delAngVel0Z, delAngVel0Z, V4MulAdd(delAngVel0Y, delAngVel0Y, V4Mul(delAngVel0X, delAngVel0X)));
					const Vec4V dotRaXnAngVel0 = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4Mul(raXnX, angVelT00)));

					Vec4V unitResponse = V4Add(invMassNorLenSq0, dotDelAngVel0);
					Vec4V vrel = V4Add(linNorVel0, dotRaXnAngVel0);


					//The dynamic-only parts - need to if-statement these up. A branch here shouldn't cost us too much
					if(isDynamic)
					{
						SolverContact4Dynamic* PX_RESTRICT dynamicContact = static_cast<SolverContact4Dynamic*>(solverContact);
						const Vec4V rbXnX = V4NegMulSub(rbZ, normalY, V4Mul(rbY, normalZ));
						const Vec4V rbXnY = V4NegMulSub(rbX, normalZ, V4Mul(rbZ, normalX));
						const Vec4V rbXnZ = V4NegMulSub(rbY, normalX, V4Mul(rbX, normalY));

						const Vec4V v0b0 = V4Mul(invInertia1X0, rbXnX);
						const Vec4V v0b1 = V4Mul(invInertia1X1, rbXnX);
						const Vec4V v0b2 = V4Mul(invInertia1X2, rbXnX);

						const Vec4V v0PlusV1b0 = V4MulAdd(invInertia1Y0, rbXnY, v0b0);
						const Vec4V v0PlusV1b1 = V4MulAdd(invInertia1Y1, rbXnY, v0b1);
						const Vec4V v0PlusV1b2 = V4MulAdd(invInertia1Y2, rbXnY, v0b2);

						const Vec4V delAngVel1X = V4MulAdd(invInertia1Z0, rbXnZ, v0PlusV1b0);
						const Vec4V delAngVel1Y = V4MulAdd(invInertia1Z1, rbXnZ, v0PlusV1b1);
						const Vec4V delAngVel1Z = V4MulAdd(invInertia1Z2, rbXnZ, v0PlusV1b2);


						//V3Dot(raXn, delAngVel0)
						
						const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1Z, delAngVel1Z, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1X, delAngVel1X)));
						
						const Vec4V dotRbXnAngVel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4Mul(rbXnX, angVelT01)));

						const Vec4V resp1 = V4Add(dotDelAngVel1, invMassNorLenSq1);

						unitResponse = V4Add(unitResponse, resp1);

						const Vec4V vrel2 = V4Add(linNorVel1, dotRbXnAngVel1);
						vrel = V4Sub(vrel, vrel2);

						//These are for dynamic-only contacts.
						dynamicContact->rbXnX = delAngVel1X;
						dynamicContact->rbXnY = delAngVel1Y;
						dynamicContact->rbXnZ = delAngVel1Z;

					}

					const Vec4V velMultiplier = V4Sel(V4IsGrtr(unitResponse, zero), V4Recip(unitResponse), zero);

					const Vec4V penetration = V4Sub(separation, restDistance);

					const Vec4V penInvDtp8 = V4Max(maxPenBias, V4Scale(penetration, invDtp8));

					Vec4V scaledBias = V4Mul(velMultiplier, penInvDtp8);

					const Vec4V penetrationInvDt = V4Scale(penetration, invDt);

					const BoolV isGreater2 = BAnd(BAnd(V4IsGrtr(restitution, zero), V4IsGrtr(bounceThreshold, vrel)), 
						V4IsGrtr(V4Neg(vrel), penetrationInvDt));

					const BoolV ccdSeparationCondition = V4IsGrtrOrEq(ccdMaxSeparation, penetration);

					scaledBias = V4Sel(BAnd(ccdSeparationCondition, isGreater2), zero, scaledBias);

					const Vec4V sumVRel(vrel);

					const Vec4V targetVelocity = V4Sub(V4Add(V4Sel(isGreater2, V4Mul(V4Neg(sumVRel), restitution), zero), cTargetVel), vrel);

					//These values are present for static and dynamic contacts			
					solverContact->raXnX = delAngVel0X;
					solverContact->raXnY = delAngVel0Y;
					solverContact->raXnZ = delAngVel0Z;
					solverContact->velMultiplier = velMultiplier;
					solverContact->appliedForce = zero;
					solverContact->scaledBias = scaledBias;
					solverContact->targetVelocity = targetVelocity;
					solverContact->maxImpulse = maxImpulse;	
				}

				//PxU32 conId = contactId++;

				/*Vec4V targetVel0 = V4LoadA(&con0.targetVel.x);
				Vec4V targetVel1 = V4LoadA(&con1.targetVel.x);
				Vec4V targetVel2 = V4LoadA(&con2.targetVel.x);
				Vec4V targetVel3 = V4LoadA(&con3.targetVel.x);

				Vec4V targetVelX, targetVelY, targetVelZ;
				PX_TRANSPOSE_44_34(targetVel0, targetVel1, targetVel2, targetVel3, targetVelX, targetVelY, targetVelZ);*/

				for(PxU32 a = 0; a < numFrictionPerPoint; ++a)
				{
					SolverFriction4Base* PX_RESTRICT friction = reinterpret_cast<SolverFriction4Base*>(ptr2);

					ptr2 += frictionSize;

					const Vec4V tX = tFallbackX[fricIndex];
					const Vec4V tY = tFallbackY[fricIndex];
					const Vec4V tZ = tFallbackZ[fricIndex];

					fricIndex = 1 - fricIndex;

					const Vec4V raXnX = V4NegMulSub(raZ, tY, V4Mul(raY, tZ));
					const Vec4V raXnY = V4NegMulSub(raX, tZ, V4Mul(raZ, tX));
					const Vec4V raXnZ = V4NegMulSub(raY, tX, V4Mul(raX, tY));

					const Vec4V v0a0 = V4Mul(invInertia0X0, raXnX);
					const Vec4V v0a1 = V4Mul(invInertia0X1, raXnX);
					const Vec4V v0a2 = V4Mul(invInertia0X2, raXnX);

					const Vec4V v0PlusV1a0 = V4MulAdd(invInertia0Y0, raXnY, v0a0);
					const Vec4V v0PlusV1a1 = V4MulAdd(invInertia0Y1, raXnY, v0a1);
					const Vec4V v0PlusV1a2 = V4MulAdd(invInertia0Y2, raXnY, v0a2);

					const Vec4V delAngVel0X = V4MulAdd(invInertia0Z0, raXnZ, v0PlusV1a0);
					const Vec4V delAngVel0Y = V4MulAdd(invInertia0Z1, raXnZ, v0PlusV1a1);
					const Vec4V delAngVel0Z = V4MulAdd(invInertia0Z2, raXnZ, v0PlusV1a2);

					const Vec4V dotDelAngVel0 = V4MulAdd(delAngVel0Z, delAngVel0Z, V4MulAdd(delAngVel0Y, delAngVel0Y, V4Mul(delAngVel0X, delAngVel0X)));

					const Vec4V norVel0 = V4MulAdd(tX, linVelT00, V4MulAdd(tY, linVelT10, V4Mul(tZ, linVelT20)));
					const Vec4V dotRaXnAngVel0 = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4Mul(raXnX, angVelT00)));
					Vec4V vrel = V4Add(norVel0, dotRaXnAngVel0);
					
					Vec4V unitResponse = V4Add(invMass0_dom0fV, dotDelAngVel0);
					
					if(isDynamic)
					{
						SolverFriction4Dynamic* PX_RESTRICT dFric = static_cast<SolverFriction4Dynamic*>(friction);

						const Vec4V rbXnX = V4NegMulSub(rbZ, tY, V4Mul(rbY, tZ));
						const Vec4V rbXnY = V4NegMulSub(rbX, tZ, V4Mul(rbZ, tX));
						const Vec4V rbXnZ = V4NegMulSub(rbY, tX, V4Mul(rbX, tY));
						
						const Vec4V v0b0 = V4Mul(invInertia1X0, rbXnX);
						const Vec4V v0b1 = V4Mul(invInertia1X1, rbXnX);
						const Vec4V v0b2 = V4Mul(invInertia1X2, rbXnX);

						const Vec4V v0PlusV1b0 = V4MulAdd(invInertia1Y0, rbXnY, v0b0);
						const Vec4V v0PlusV1b1 = V4MulAdd(invInertia1Y1, rbXnY, v0b1);
						const Vec4V v0PlusV1b2 = V4MulAdd(invInertia1Y2, rbXnY, v0b2);

						const Vec4V delAngVel1X = V4MulAdd(invInertia1Z0, rbXnZ, v0PlusV1b0);
						const Vec4V delAngVel1Y = V4MulAdd(invInertia1Z1, rbXnZ, v0PlusV1b1);
						const Vec4V delAngVel1Z = V4MulAdd(invInertia1Z2, rbXnZ, v0PlusV1b2);

						const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1Z, delAngVel1Z, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1X, delAngVel1X)));
						
						const Vec4V norVel1 = V4MulAdd(tX, linVelT01, V4MulAdd(tY, linVelT11, V4Mul(tZ, linVelT21)));
						const Vec4V dotRbXnAngVel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4Mul(rbXnX, angVelT01)));
						vrel = V4Sub(vrel, V4Add(norVel1, dotRbXnAngVel1));
				
						const Vec4V resp1 = V4Add(dotDelAngVel1, invMassNorLenSq1);

						unitResponse = V4Add(unitResponse, resp1);

						dFric->rbXnX = delAngVel1X;
						dFric->rbXnY = delAngVel1Y;
						dFric->rbXnZ = delAngVel1Z;
					}

					const Vec4V velMultiplier = V4Neg(V4Sel(V4IsGrtr(unitResponse, zero), V4Recip(unitResponse), zero));

					friction->appliedForce = zero;
					friction->raXnX = delAngVel0X;
					friction->raXnY = delAngVel0Y;
					friction->raXnZ = delAngVel0Z;
					friction->velMultiplier = velMultiplier;
					friction->targetVelocity = V4Sub(V4MulAdd(targetVelZ, tZ, V4MulAdd(targetVelY, tY, V4Mul(targetVelX, tX))), vrel);
					friction->normalX = tX;
					friction->normalY = tY;
					friction->normalZ = tZ;
				}
			}
			if(!(finished & 0x1))
			{
				iter0.nextContact(patch0, contact0);
				newFinished |= PxU32(!iter0.hasNextContact());
			}

			if(!(finished & 0x2))
			{
				iter1.nextContact(patch1, contact1);
				newFinished |= (PxU32(!iter1.hasNextContact()) << 1);
			}

			if(!(finished & 0x4))
			{
				iter2.nextContact(patch2, contact2);
				newFinished |= (PxU32(!iter2.hasNextContact()) << 2);
			}

			if(!(finished & 0x8))
			{
				iter3.nextContact(patch3, contact3);
				newFinished |= (PxU32(!iter3.hasNextContact()) << 3);
			}
		}
		ptr = p;
	}
	return true;
}



//The persistent friction patch correlation/allocation will already have happenned as this is per-pair.
//This function just computes the size of the combined solve data.
void computeBlockStreamByteSizesCoulomb4(PxSolverContactDesc* descs,
								  ThreadContext& threadContext, const CorrelationBuffer& c,
								  const PxU32 numFrictionPerPoint, 
								PxU32& _solverConstraintByteSize, PxU32* _axisConstraintCount, PxU32& _numContactPoints4)
{
	PX_ASSERT(0 == _solverConstraintByteSize);
	PX_UNUSED(threadContext);

	PxU32 maxPatches = 0;
	PxU32 maxContactCount[CorrelationBuffer::MAX_FRICTION_PATCHES];
	PxU32 maxFrictionCount[CorrelationBuffer::MAX_FRICTION_PATCHES];
	PxMemZero(maxContactCount, sizeof(maxContactCount));
	PxMemZero(maxFrictionCount, sizeof(maxFrictionCount));
	for(PxU32 a = 0; a < 4; ++a)
	{
		PxU32 axisConstraintCount = 0;

		for(PxU32 i = 0; i < descs[a].numFrictionPatches; i++)
		{
			PxU32 ind = i + descs[a].startFrictionPatchIndex;

			const FrictionPatch& frictionPatch = c.frictionPatches[ind];

			const bool haveFriction = (frictionPatch.materialFlags & PxMaterialFlag::eDISABLE_FRICTION) == 0;
			//Solver constraint data.
			if(c.frictionPatchContactCounts[ind]!=0)
			{
				maxContactCount[i] = PxMax(c.frictionPatchContactCounts[ind], maxContactCount[i]);
				axisConstraintCount += c.frictionPatchContactCounts[ind];

				if(haveFriction)
				{
					//const PxU32 fricCount = c.frictionPatches[ind].numConstraints;
					const PxU32 fricCount = c.frictionPatchContactCounts[ind] * numFrictionPerPoint;
					maxFrictionCount[i] = PxMax(fricCount, maxFrictionCount[i]);
					axisConstraintCount += fricCount;
				}
			}
		}
		maxPatches = PxMax(descs[a].numFrictionPatches, maxPatches);
		_axisConstraintCount[a] = axisConstraintCount;
	}

	PxU32 totalContacts = 0, totalFriction = 0;
	for(PxU32 a = 0; a < maxPatches; ++a)
	{
		totalContacts += maxContactCount[a];
		totalFriction += maxFrictionCount[a];
	}

	_numContactPoints4 = totalContacts;


	//OK, we have a given number of friction patches, contact points and friction constraints so we can calculate how much memory we need

	const bool isStatic = (((descs[0].bodyState1 | descs[1].bodyState1 | descs[2].bodyState1 | descs[3].bodyState1) & PxSolverContactDesc::eDYNAMIC_BODY) == 0);

	const PxU32 headerSize = (sizeof(SolverContactCoulombHeader4) + sizeof(SolverFrictionHeader4)) * maxPatches;
	//Add on 1 Vec4V per contact for the applied force buffer
	const PxU32 constraintSize = isStatic ? ((sizeof(SolverContact4Base) + sizeof(Vec4V)) * totalContacts) + ( sizeof(SolverFriction4Base) * totalFriction) : 
		((sizeof(SolverContact4Dynamic) + sizeof(Vec4V)) * totalContacts) + (sizeof(SolverFriction4Dynamic) * totalFriction);

	_solverConstraintByteSize =  ((constraintSize + headerSize + 0x0f) & ~0x0f);
	PX_ASSERT(0 == (_solverConstraintByteSize & 0x0f));
}


static SolverConstraintPrepState::Enum reserveBlockStreamsCoulomb4(PxSolverContactDesc* descs, ThreadContext& threadContext, const CorrelationBuffer& c,
						PxU8*& solverConstraint, const PxU32 numFrictionPerContactPoint,
						PxU32& solverConstraintByteSize,
						PxU32* axisConstraintCount, PxU32& numContactPoints4, PxConstraintAllocator& constraintAllocator)
{
	PX_ASSERT(NULL == solverConstraint);
	PX_ASSERT(0 == solverConstraintByteSize);

	//From constraintBlockStream we need to reserve contact points, contact forces, and a char buffer for the solver constraint data (already have a variable for this).
	//From frictionPatchStream we just need to reserve a single buffer.

	//Compute the sizes of all the buffers.
	computeBlockStreamByteSizesCoulomb4(
		descs, threadContext, c, numFrictionPerContactPoint, solverConstraintByteSize,
		axisConstraintCount, numContactPoints4);

	//Reserve the buffers.

	//First reserve the accumulated buffer size for the constraint block.
	PxU8* constraintBlock = NULL;
	const PxU32 constraintBlockByteSize = solverConstraintByteSize;
	if(constraintBlockByteSize > 0)
	{
		if((constraintBlockByteSize + 16u) > 16384)
			return SolverConstraintPrepState::eUNBATCHABLE;

		constraintBlock = constraintAllocator.reserveConstraintData(constraintBlockByteSize + 16u);

		if(0==constraintBlock || (reinterpret_cast<PxU8*>(-1))==constraintBlock)
		{
			if(0==constraintBlock)
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
				constraintBlock=NULL;
			}
		}
	}

	//Patch up the individual ptrs to the buffer returned by the constraint block reservation (assuming the reservation didn't fail).
	if(0==constraintBlockByteSize || constraintBlock)
	{
		if(solverConstraintByteSize)
		{
			solverConstraint = constraintBlock;
			PX_ASSERT(0==(uintptr_t(solverConstraint) & 0x0f));
		}
	}

	//Return true if neither of the two block reservations failed.
	return ((0==constraintBlockByteSize || constraintBlock)) ? SolverConstraintPrepState::eSUCCESS : SolverConstraintPrepState::eOUT_OF_MEMORY;
}

SolverConstraintPrepState::Enum createFinalizeSolverContacts4Coulomb1D(
	PxsContactManagerOutput** outputs,
	ThreadContext& threadContext,
	PxSolverContactDesc* blockDescs,
	const PxReal invDtF32,
	PxReal bounceThresholdF32,
	PxReal frictionOffsetThreshold,
	PxReal correlationDistance,
	PxReal solverOffsetSlop,
	PxConstraintAllocator& constraintAllocator)
{
	return createFinalizeSolverContacts4Coulomb(outputs, threadContext, blockDescs, invDtF32, bounceThresholdF32, 
		frictionOffsetThreshold, correlationDistance, solverOffsetSlop, constraintAllocator, PxFrictionType::eONE_DIRECTIONAL);
}

SolverConstraintPrepState::Enum createFinalizeSolverContacts4Coulomb2D(
	PxsContactManagerOutput** outputs,
	ThreadContext& threadContext,
	PxSolverContactDesc* blockDescs,
	const PxReal invDtF32,
	PxReal bounceThresholdF32,
	PxReal frictionOffsetThreshold,
	PxReal correlationDistance,
	PxReal solverOffsetSlop,
	PxConstraintAllocator& constraintAllocator)
{
	return createFinalizeSolverContacts4Coulomb(outputs, threadContext, blockDescs, invDtF32, bounceThresholdF32,
		frictionOffsetThreshold, correlationDistance, solverOffsetSlop, constraintAllocator, PxFrictionType::eTWO_DIRECTIONAL);
}


SolverConstraintPrepState::Enum createFinalizeSolverContacts4Coulomb(
								PxsContactManagerOutput** outputs,
								 ThreadContext& threadContext,
								 PxSolverContactDesc* blockDescs,
								 const PxReal invDtF32,
								 PxReal bounceThresholdF32,
								 PxReal frictionOffsetThreshold,
								 PxReal correlationDistance,
								 PxReal solverOffsetSlop,
								 PxConstraintAllocator& constraintAllocator,
								 PxFrictionType::Enum frictionType)
{
	PX_UNUSED(frictionOffsetThreshold);
	PX_UNUSED(correlationDistance);

	for(PxU32 i = 0; i < 4; ++i)
	{
		blockDescs[i].desc->constraintLengthOver16 = 0;
	}

	PX_ASSERT(outputs[0]->nbContacts && outputs[1]->nbContacts && outputs[2]->nbContacts && outputs[3]->nbContacts);

	Gu::ContactBuffer& buffer = threadContext.mContactBuffer;

	buffer.count = 0;

	PxU32 numContacts = 0;

	CorrelationBuffer& c = threadContext.mCorrelationBuffer;

	c.frictionPatchCount = 0;
	c.contactPatchCount = 0;

	PxU32 numFrictionPerPoint = PxU32(frictionType == PxFrictionType::eONE_DIRECTIONAL ? 1 : 2);

	PX_ALIGN(16, PxReal invMassScale0[4]);
	PX_ALIGN(16, PxReal invMassScale1[4]);
	PX_ALIGN(16, PxReal invInertiaScale0[4]);
	PX_ALIGN(16, PxReal invInertiaScale1[4]);
	
	for(PxU32 a = 0; a < 4; ++a)
	{
		PxSolverContactDesc& blockDesc = blockDescs[a];
		PxSolverConstraintDesc& desc = *blockDesc.desc;
		
		//blockDesc.startContactIndex = numContacts;
		blockDesc.contacts = &buffer.contacts[numContacts];

		Ps::prefetchLine(desc.bodyA);
		Ps::prefetchLine(desc.bodyB);

		if((numContacts + outputs[a]->nbContacts) > 64)
		{
			return SolverConstraintPrepState::eUNBATCHABLE;
		}
		bool hasMaxImpulse, hasTargetVelocity;

		const PxReal defaultMaxImpulse = PxMin(blockDesc.data0->maxContactImpulse, blockDesc.data1->maxContactImpulse);

		PxU32 contactCount = extractContacts(buffer, *outputs[a], hasMaxImpulse, hasTargetVelocity, invMassScale0[a], invMassScale1[a], 
			invInertiaScale0[a], invInertiaScale1[a], defaultMaxImpulse);

		if(contactCount == 0)
			return SolverConstraintPrepState::eUNBATCHABLE;

		numContacts+=contactCount;

		blockDesc.numContacts = contactCount;
		blockDesc.hasMaxImpulse = hasMaxImpulse;
	
		blockDesc.startFrictionPatchIndex = c.frictionPatchCount;
		blockDesc.startContactPatchIndex = c.contactPatchCount;

		createContactPatches(c, blockDesc.contacts, contactCount, PXC_SAME_NORMAL);
		
		bool overflow = correlatePatches(c, blockDesc.contacts, blockDesc.bodyFrame0, blockDesc.bodyFrame1, PXC_SAME_NORMAL, blockDesc.startContactPatchIndex,
			blockDesc.startFrictionPatchIndex);
		if(overflow)
			return SolverConstraintPrepState::eUNBATCHABLE;

		blockDesc.numContactPatches = PxU16(c.contactPatchCount - blockDesc.startContactPatchIndex);
		blockDesc.numFrictionPatches = c.frictionPatchCount - blockDesc.startFrictionPatchIndex;

		invMassScale0[a] *= blockDesc.mInvMassScales.linear0;
		invMassScale1[a] *= blockDesc.mInvMassScales.linear1;
		invInertiaScale0[a] *= blockDesc.mInvMassScales.angular0;
		invInertiaScale1[a] *= blockDesc.mInvMassScales.angular1;

	}

	//OK, now we need to work out how much memory to allocate, allocate it and then block-create the constraints...

	PxU8* solverConstraint = NULL;
	PxU32 solverConstraintByteSize = 0;
	PxU32 axisConstraintCount[4];
	PxU32 numContactPoints4 = 0;

	SolverConstraintPrepState::Enum state = reserveBlockStreamsCoulomb4(blockDescs, threadContext, c,
												solverConstraint, numFrictionPerPoint, 
												solverConstraintByteSize,
												axisConstraintCount, numContactPoints4, constraintAllocator);

	if(state != SolverConstraintPrepState::eSUCCESS)
		return state;

	//OK, we allocated the memory, now let's create the constraints

	for(PxU32 a = 0; a < 4; ++a)
	{
		PxSolverConstraintDesc& desc = *blockDescs[a].desc;
		//n[a]->solverConstraintPointer = solverConstraint;
		desc.constraint = solverConstraint;

		//KS - TODO - add back in counters for axisConstraintCount somewhere...
		blockDescs[a].axisConstraintCount += Ps::to16(axisConstraintCount[a]);

		desc.constraintLengthOver16 = Ps::to16(solverConstraintByteSize/16);

		PxU32 writeBackLength = outputs[a]->nbContacts * sizeof(PxReal);
		void* writeBack = outputs[a]->contactForces;
		desc.writeBack = writeBack;
		setWritebackLength(desc, writeBackLength);
	}

	const Vec4V iMassScale0 = V4LoadA(invMassScale0); 
	const Vec4V iInertiaScale0 = V4LoadA(invInertiaScale0);
	const Vec4V iMassScale1 = V4LoadA(invMassScale1); 
	const Vec4V iInertiaScale1 = V4LoadA(invInertiaScale1);


	bool hasFriction = setupFinalizeSolverConstraintsCoulomb4(blockDescs, solverConstraint, 
											invDtF32, bounceThresholdF32, solverOffsetSlop, c, numFrictionPerPoint, numContactPoints4, solverConstraintByteSize,
											iMassScale0, iInertiaScale0, iMassScale1, iInertiaScale1);

	*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize)) = 0;
	*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize + 4)) = hasFriction ? 0xFFFFFFFF : 0;


	return SolverConstraintPrepState::eSUCCESS;
}

}

}

