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

using namespace physx;
using namespace Gu;


#include "PsVecMath.h"
#include "PxContactModifyCallback.h"
#include "PxsMaterialManager.h"
#include "PxsMaterialCombiner.h"
#include "DyContactPrepShared.h"

using namespace Ps::aos;

namespace physx
{
namespace Dy
{

PxcCreateFinalizeSolverContactMethod4 createFinalizeMethods4[3] = 
{
	createFinalizeSolverContacts4,
	createFinalizeSolverContacts4Coulomb1D,
	createFinalizeSolverContacts4Coulomb2D
};

inline bool ValidateVec4(const Vec4V v)
{
	PX_ALIGN(16, PxVec4 vF);
	Ps::aos::V4StoreA(v, &vF.x);
	return vF.isFinite();
}

static void setupFinalizeSolverConstraints4(PxSolverContactDesc* PX_RESTRICT descs, CorrelationBuffer& c, PxU8* PX_RESTRICT workspace,
											const PxReal invDtF32, PxReal bounceThresholdF32, const PxReal solverOffsetSlopF32,
											const Ps::aos::Vec4VArg invMassScale0, const Ps::aos::Vec4VArg invInertiaScale0, 
											const Ps::aos::Vec4VArg invMassScale1, const Ps::aos::Vec4VArg invInertiaScale1)
{

	//OK, we have a workspace of pre-allocated space to store all 4 descs in. We now need to create the constraints in it

	const Vec4V ccdMaxSeparation = Ps::aos::V4LoadXYZW(descs[0].maxCCDSeparation, descs[1].maxCCDSeparation, descs[2].maxCCDSeparation, descs[3].maxCCDSeparation);
	const Vec4V solverOffsetSlop = V4Load(solverOffsetSlopF32);

	const Vec4V zero = V4Zero();
	const BoolV bFalse = BFFFF();
	const FloatV fZero = FZero();

	PxU8 flags[4] = {	PxU8(descs[0].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
						PxU8(descs[1].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
						PxU8(descs[2].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
						PxU8(descs[3].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0) };

	bool hasMaxImpulse = descs[0].hasMaxImpulse || descs[1].hasMaxImpulse || descs[2].hasMaxImpulse || descs[3].hasMaxImpulse;

	//The block is dynamic if **any** of the constraints have a non-static body B. This allows us to batch static and non-static constraints but we only get a memory/perf
	//saving if all 4 are static. This simplifies the constraint partitioning such that it only needs to care about separating contacts and 1D constraints (which it already does)
	bool isDynamic = false;
	bool hasKinematic = false;
	for(PxU32 a = 0; a < 4; ++a)
	{
		isDynamic = isDynamic || (descs[a].bodyState1 == PxSolverContactDesc::eDYNAMIC_BODY);
		hasKinematic = hasKinematic || descs[a].bodyState1 == PxSolverContactDesc::eKINEMATIC_BODY;
	}
	
	const PxU32 constraintSize = isDynamic ? sizeof(SolverContactBatchPointDynamic4) : sizeof(SolverContactBatchPointBase4);
	const PxU32 frictionSize = isDynamic ? sizeof(SolverContactFrictionDynamic4) : sizeof(SolverContactFrictionBase4);

	PxU8* PX_RESTRICT ptr = workspace;

	const Vec4V dom0 = invMassScale0;
	const Vec4V dom1 = invMassScale1;
	const Vec4V angDom0 = invInertiaScale0;
	const Vec4V angDom1 = invInertiaScale1;

	const Vec4V maxPenBias = V4Max(V4LoadXYZW(descs[0].data0->penBiasClamp, descs[1].data0->penBiasClamp, 
		descs[2].data0->penBiasClamp, descs[3].data0->penBiasClamp), 
		V4LoadXYZW(descs[0].data1->penBiasClamp, descs[1].data1->penBiasClamp, 
		descs[2].data1->penBiasClamp, descs[3].data1->penBiasClamp));

	const Vec4V restDistance = V4LoadXYZW(descs[0].restDistance, descs[1].restDistance, descs[2].restDistance,
		descs[3].restDistance); 


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

	/*const Vec4V sqrtInvMass0 = V4Merge(FLoad(descs[0].data0->sqrtInvMass), FLoad(descs[1].data0->sqrtInvMass), FLoad(descs[2].data0->sqrtInvMass),
		FLoad(descs[3].data0->sqrtInvMass));

	const Vec4V sqrtInvMass1 = V4Merge(FLoad(descs[0].data1->sqrtInvMass), FLoad(descs[1].data1->sqrtInvMass), FLoad(descs[2].data1->sqrtInvMass),
		FLoad(descs[3].data1->sqrtInvMass));*/

	const Vec4V invMass0 = V4LoadXYZW(descs[0].data0->invMass, descs[1].data0->invMass, descs[2].data0->invMass, descs[3].data0->invMass);
	const Vec4V invMass1 = V4LoadXYZW(descs[0].data1->invMass, descs[1].data1->invMass, descs[2].data1->invMass, descs[3].data1->invMass);

	const Vec4V invMass0D0 = V4Mul(dom0, invMass0);
	const Vec4V invMass1D1 = V4Mul(dom1, invMass1);

	Vec4V invInertia00X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[0].data0->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia00Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[0].data0->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia00Z = Vec4V_From_Vec3V(V3LoadU(descs[0].data0->sqrtInvInertia.column2));

	Vec4V invInertia10X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[1].data0->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia10Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[1].data0->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia10Z = Vec4V_From_Vec3V(V3LoadU(descs[1].data0->sqrtInvInertia.column2));

	Vec4V invInertia20X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[2].data0->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia20Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[2].data0->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia20Z = Vec4V_From_Vec3V(V3LoadU(descs[2].data0->sqrtInvInertia.column2));

	Vec4V invInertia30X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[3].data0->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia30Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[3].data0->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia30Z = Vec4V_From_Vec3V(V3LoadU(descs[3].data0->sqrtInvInertia.column2));

	Vec4V invInertia01X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[0].data1->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia01Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[0].data1->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia01Z = Vec4V_From_Vec3V(V3LoadU(descs[0].data1->sqrtInvInertia.column2));

	Vec4V invInertia11X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[1].data1->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia11Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[1].data1->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia11Z = Vec4V_From_Vec3V(V3LoadU(descs[1].data1->sqrtInvInertia.column2));

	Vec4V invInertia21X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[2].data1->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia21Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[2].data1->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia21Z = Vec4V_From_Vec3V(V3LoadU(descs[2].data1->sqrtInvInertia.column2));

	Vec4V invInertia31X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[3].data1->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia31Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[3].data1->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
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
	const Vec4V p84 = V4Splat(p8);
	const Vec4V bounceThreshold = V4Splat(FLoad(bounceThresholdF32));

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


	const QuatV bodyFrame00q = QuatVLoadU(&descs[0].bodyFrame0.q.x);	
	const QuatV bodyFrame01q = QuatVLoadU(&descs[1].bodyFrame0.q.x);
	const QuatV bodyFrame02q = QuatVLoadU(&descs[2].bodyFrame0.q.x);
	const QuatV bodyFrame03q = QuatVLoadU(&descs[3].bodyFrame0.q.x);

	const QuatV bodyFrame10q = QuatVLoadU(&descs[0].bodyFrame1.q.x);	
	const QuatV bodyFrame11q = QuatVLoadU(&descs[1].bodyFrame1.q.x);
	const QuatV bodyFrame12q = QuatVLoadU(&descs[2].bodyFrame1.q.x);	
	const QuatV bodyFrame13q = QuatVLoadU(&descs[3].bodyFrame1.q.x);

	PxU32 frictionPatchWritebackAddrIndex0 = 0;
	PxU32 frictionPatchWritebackAddrIndex1 = 0;
	PxU32 frictionPatchWritebackAddrIndex2 = 0;
	PxU32 frictionPatchWritebackAddrIndex3 = 0;

	Ps::prefetchLine(c.contactID);
	Ps::prefetchLine(c.contactID, 128);

	PxU32 frictionIndex0 = 0, frictionIndex1 = 0, frictionIndex2 = 0, frictionIndex3 = 0;
	//PxU32 contactIndex0 = 0, contactIndex1 = 0, contactIndex2 = 0, contactIndex3 = 0;


	//OK, we iterate through all friction patch counts in the constraint patch, building up the constraint list etc.

	PxU32 maxPatches = PxMax(descs[0].numFrictionPatches, PxMax(descs[1].numFrictionPatches, PxMax(descs[2].numFrictionPatches, descs[3].numFrictionPatches)));

	const Vec4V p1 = V4Splat(FLoad(0.1f));
	const Vec4V orthoThreshold = V4Splat(FLoad(0.70710678f));

	
	PxU32 contact0 = 0, contact1 = 0, contact2 = 0, contact3 = 0;
	PxU32 patch0 = 0, patch1 = 0, patch2 = 0, patch3 = 0;

	PxU8 flag = 0;
	if(hasMaxImpulse)
		flag |= SolverContactHeader4::eHAS_MAX_IMPULSE;

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

		PxU32 firstPatch0 = c.correlationListHeads[frictionIndex0];
		PxU32 firstPatch1 = c.correlationListHeads[frictionIndex1];
		PxU32 firstPatch2 = c.correlationListHeads[frictionIndex2];
		PxU32 firstPatch3 = c.correlationListHeads[frictionIndex3];

		const Gu::ContactPoint* contactBase0 = descs[0].contacts + c.contactPatches[firstPatch0].start;
		const Gu::ContactPoint* contactBase1 = descs[1].contacts + c.contactPatches[firstPatch1].start;
		const Gu::ContactPoint* contactBase2 = descs[2].contacts + c.contactPatches[firstPatch2].start;
		const Gu::ContactPoint* contactBase3 = descs[3].contacts + c.contactPatches[firstPatch3].start;

		const Vec4V restitution = V4Neg(V4LoadXYZW(contactBase0->restitution, contactBase1->restitution, contactBase2->restitution,
			contactBase3->restitution));

		SolverContactHeader4* PX_RESTRICT header = reinterpret_cast<SolverContactHeader4*>(ptr);
		ptr += sizeof(SolverContactHeader4);	

		
		header->flags[0] = flags[0];
		header->flags[1] = flags[1];
		header->flags[2] = flags[2];
		header->flags[3] = flags[3];

		header->flag = flag;

		PxU32 totalContacts = PxMax(clampedContacts0, PxMax(clampedContacts1, PxMax(clampedContacts2, clampedContacts3)));

		Vec4V* PX_RESTRICT appliedNormalForces = reinterpret_cast<Vec4V*>(ptr);
		ptr += sizeof(Vec4V)*totalContacts;

		PxMemZero(appliedNormalForces, sizeof(Vec4V) * totalContacts);

		header->numNormalConstr		= Ps::to8(totalContacts);
		header->numNormalConstr0 = Ps::to8(clampedContacts0);
		header->numNormalConstr1 = Ps::to8(clampedContacts1);
		header->numNormalConstr2 = Ps::to8(clampedContacts2);
		header->numNormalConstr3 = Ps::to8(clampedContacts3);
		//header->sqrtInvMassA = sqrtInvMass0;
		//header->sqrtInvMassB = sqrtInvMass1;
		header->invMass0D0 = invMass0D0;
		header->invMass1D1 = invMass1D1;
		header->angDom0 = angDom0;
		header->angDom1 = angDom1;
		header->shapeInteraction[0] = descs[0].shapeInteraction; header->shapeInteraction[1] = descs[1].shapeInteraction; 
		header->shapeInteraction[2] = descs[2].shapeInteraction; header->shapeInteraction[3] = descs[3].shapeInteraction;

		Vec4V* maxImpulse = reinterpret_cast<Vec4V*>(ptr + constraintSize * totalContacts);

		header->restitution = restitution;

		Vec4V normal0 = V4LoadA(&contactBase0->normal.x);
		Vec4V normal1 = V4LoadA(&contactBase1->normal.x);
		Vec4V normal2 = V4LoadA(&contactBase2->normal.x);
		Vec4V normal3 = V4LoadA(&contactBase3->normal.x);

		Vec4V normalX, normalY, normalZ;
		PX_TRANSPOSE_44_34(normal0, normal1, normal2, normal3, normalX, normalY, normalZ);

		PX_ASSERT(ValidateVec4(normalX));
		PX_ASSERT(ValidateVec4(normalY));
		PX_ASSERT(ValidateVec4(normalZ));

		header->normalX = normalX;
		header->normalY = normalY;
		header->normalZ = normalZ;

		const Vec4V norVel0 = V4MulAdd(normalZ, linVelT20, V4MulAdd(normalY, linVelT10, V4Mul(normalX, linVelT00)));
		const Vec4V norVel1 = V4MulAdd(normalZ, linVelT21, V4MulAdd(normalY, linVelT11, V4Mul(normalX, linVelT01)));
		const Vec4V relNorVel  = V4Sub(norVel0, norVel1);

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

		//PxU32 contact0, contact1, contact2, contact3;
		//PxU32 patch0, patch1, patch2, patch3;

		if(!hasFinished0)
			iter0.nextContact(patch0, contact0);
		if(!hasFinished1)
			iter1.nextContact(patch1, contact1);
		if(!hasFinished2)
			iter2.nextContact(patch2, contact2);
		if(!hasFinished3)
			iter3.nextContact(patch3, contact3);

		PxU8* p = ptr;

		PxU32 contactCount = 0;
		PxU32 newFinished = 
			(PxU32(hasFinished0 || !iter0.hasNextContact()))		| 
			((PxU32(hasFinished1 || !iter1.hasNextContact())) << 1) | 
			((PxU32(hasFinished2 || !iter2.hasNextContact())) << 2) | 
			((PxU32(hasFinished3 || !iter3.hasNextContact())) << 3);

		while(finished != 0xf)
		{
			finished = newFinished;
			++contactCount;
			Ps::prefetchLine(p, 384);
			Ps::prefetchLine(p, 512);
			Ps::prefetchLine(p, 640);	

			SolverContactBatchPointBase4* PX_RESTRICT solverContact = reinterpret_cast<SolverContactBatchPointBase4*>(p);
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

				PX_ASSERT(ValidateVec4(pointX));
				PX_ASSERT(ValidateVec4(pointY));
				PX_ASSERT(ValidateVec4(pointZ));

				Vec4V cTargetVel0 = V4LoadA(&con0.targetVel.x);
				Vec4V cTargetVel1 = V4LoadA(&con1.targetVel.x);
				Vec4V cTargetVel2 = V4LoadA(&con2.targetVel.x);
				Vec4V cTargetVel3 = V4LoadA(&con3.targetVel.x);

				Vec4V cTargetVelX, cTargetVelY, cTargetVelZ;
				PX_TRANSPOSE_44_34(cTargetVel0, cTargetVel1, cTargetVel2, cTargetVel3, cTargetVelX, cTargetVelY, cTargetVelZ);

				const Vec4V separation = V4LoadXYZW(con0.separation, con1.separation, con2.separation, con3.separation);

				const Vec4V cTargetNorVel = V4MulAdd(cTargetVelX, normalX, V4MulAdd(cTargetVelY, normalY, V4Mul(cTargetVelZ, normalZ)));

				const Vec4V raX = V4Sub(pointX, bodyFrame0pX);
				const Vec4V raY = V4Sub(pointY, bodyFrame0pY);
				const Vec4V raZ = V4Sub(pointZ, bodyFrame0pZ);
				
				const Vec4V rbX = V4Sub(pointX, bodyFrame1pX);
				const Vec4V rbY = V4Sub(pointY, bodyFrame1pY);
				const Vec4V rbZ = V4Sub(pointZ, bodyFrame1pZ);

				/*raX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raX)), zero, raX);
				raY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raY)), zero, raY);
				raZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raZ)), zero, raZ);

				rbX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbX)), zero, rbX);
				rbY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbY)), zero, rbY);
				rbZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbZ)), zero, rbZ);*/

				PX_ASSERT(ValidateVec4(raX));
				PX_ASSERT(ValidateVec4(raY));
				PX_ASSERT(ValidateVec4(raZ));

				PX_ASSERT(ValidateVec4(rbX));
				PX_ASSERT(ValidateVec4(rbY));
				PX_ASSERT(ValidateVec4(rbZ));


				//raXn = cross(ra, normal) which = Vec3V( a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);

				Vec4V raXnX = V4NegMulSub(raZ, normalY, V4Mul(raY, normalZ));
				Vec4V raXnY = V4NegMulSub(raX, normalZ, V4Mul(raZ, normalX));
				Vec4V raXnZ = V4NegMulSub(raY, normalX, V4Mul(raX, normalY));

				raXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnX)), zero, raXnX);
				raXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnY)), zero, raXnY);
				raXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnZ)), zero, raXnZ);

				Vec4V delAngVel0X = V4Mul(invInertia0X0, raXnX);
				Vec4V delAngVel0Y = V4Mul(invInertia0X1, raXnX);
				Vec4V delAngVel0Z = V4Mul(invInertia0X2, raXnX);

				delAngVel0X = V4MulAdd(invInertia0Y0, raXnY, delAngVel0X);
				delAngVel0Y = V4MulAdd(invInertia0Y1, raXnY, delAngVel0Y);
				delAngVel0Z = V4MulAdd(invInertia0Y2, raXnY, delAngVel0Z);

				delAngVel0X = V4MulAdd(invInertia0Z0, raXnZ, delAngVel0X);
				delAngVel0Y = V4MulAdd(invInertia0Z1, raXnZ, delAngVel0Y);
				delAngVel0Z = V4MulAdd(invInertia0Z2, raXnZ, delAngVel0Z);


				PX_ASSERT(ValidateVec4(delAngVel0X));
				PX_ASSERT(ValidateVec4(delAngVel0Y));
				PX_ASSERT(ValidateVec4(delAngVel0Z));

				const Vec4V dotDelAngVel0 = V4MulAdd(delAngVel0X, delAngVel0X, V4MulAdd(delAngVel0Y, delAngVel0Y, V4Mul(delAngVel0Z, delAngVel0Z)));
				const Vec4V dotRaXnAngVel0 = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4Mul(raXnX, angVelT00)));

				Vec4V unitResponse = V4MulAdd(invMass0D0, angDom0, dotDelAngVel0);
				Vec4V vrel = V4Add(relNorVel, dotRaXnAngVel0);


				//The dynamic-only parts - need to if-statement these up. A branch here shouldn't cost us too much
				if(isDynamic)
				{
					SolverContactBatchPointDynamic4* PX_RESTRICT dynamicContact = static_cast<SolverContactBatchPointDynamic4*>(solverContact);
					Vec4V rbXnX = V4NegMulSub(rbZ, normalY, V4Mul(rbY, normalZ));
					Vec4V rbXnY = V4NegMulSub(rbX, normalZ, V4Mul(rbZ, normalX));
					Vec4V rbXnZ = V4NegMulSub(rbY, normalX, V4Mul(rbX, normalY));

					rbXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnX)), zero, rbXnX);
					rbXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnY)), zero, rbXnY);
					rbXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnZ)), zero, rbXnZ);

					Vec4V delAngVel1X = V4Mul(invInertia1X0, rbXnX);
					Vec4V delAngVel1Y = V4Mul(invInertia1X1, rbXnX);
					Vec4V delAngVel1Z = V4Mul(invInertia1X2, rbXnX);

					delAngVel1X = V4MulAdd(invInertia1Y0, rbXnY, delAngVel1X);
					delAngVel1Y = V4MulAdd(invInertia1Y1, rbXnY, delAngVel1Y);
					delAngVel1Z = V4MulAdd(invInertia1Y2, rbXnY, delAngVel1Z);

					delAngVel1X = V4MulAdd(invInertia1Z0, rbXnZ, delAngVel1X);
					delAngVel1Y = V4MulAdd(invInertia1Z1, rbXnZ, delAngVel1Y);
					delAngVel1Z = V4MulAdd(invInertia1Z2, rbXnZ, delAngVel1Z);

					PX_ASSERT(ValidateVec4(delAngVel1X));
					PX_ASSERT(ValidateVec4(delAngVel1Y));
					PX_ASSERT(ValidateVec4(delAngVel1Z));

					const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1X, delAngVel1X, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1Z, delAngVel1Z)));
					const Vec4V dotRbXnAngVel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4Mul(rbXnX, angVelT01)));
					
					const Vec4V resp1 = V4MulAdd(dotDelAngVel1, angDom1, invMass1D1);

					unitResponse = V4Add(unitResponse, resp1);

					vrel = V4Sub(vrel, dotRbXnAngVel1);

					//These are for dynamic-only contacts.
					dynamicContact->rbXnX = delAngVel1X;
					dynamicContact->rbXnY = delAngVel1Y;
					dynamicContact->rbXnZ = delAngVel1Z;

				}
				else if(hasKinematic)
				{
					const Vec4V rbXnX = V4NegMulSub(rbZ, normalY, V4Mul(rbY, normalZ));
					const Vec4V rbXnY = V4NegMulSub(rbX, normalZ, V4Mul(rbZ, normalX));
					const Vec4V rbXnZ = V4NegMulSub(rbY, normalX, V4Mul(rbX, normalY));

					const Vec4V dotRbXnAngVel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4Mul(rbXnX, angVelT01)));

					vrel = V4Sub(vrel, dotRbXnAngVel1);
				}

				const Vec4V velMultiplier = V4Sel(V4IsGrtr(unitResponse, zero), V4Recip(unitResponse), zero);

				const Vec4V penetration = V4Sub(separation, restDistance);
				const Vec4V penInvDtPt8 = V4Max(maxPenBias, V4Scale(penetration, invDtp8));
				Vec4V scaledBias = V4Mul(penInvDtPt8, velMultiplier);

				const Vec4V penetrationInvDt = V4Scale(penetration, invDt);

				const BoolV isGreater2 = BAnd(BAnd(V4IsGrtr(zero, restitution), V4IsGrtr(bounceThreshold, vrel)),
					V4IsGrtr(V4Neg(vrel), penetrationInvDt));

				const BoolV ccdSeparationCondition = V4IsGrtrOrEq(ccdMaxSeparation, penetration);

				scaledBias = V4Sel(BAnd(ccdSeparationCondition, isGreater2), zero, V4Neg(scaledBias));

				const Vec4V targetVelocity = V4Sel(isGreater2, V4Mul(velMultiplier, V4Mul(vrel, restitution)), zero);

				//Vec4V biasedErr = V4Sel(isGreater2, targetVelocity, scaledBias);
				Vec4V biasedErr = V4Add(targetVelocity, scaledBias);

				biasedErr = V4NegMulSub(V4Sub(vrel, cTargetNorVel), velMultiplier, biasedErr);

				//These values are present for static and dynamic contacts			
				solverContact->raXnX = delAngVel0X;
				solverContact->raXnY = delAngVel0Y;
				solverContact->raXnZ = delAngVel0Z;
				solverContact->velMultiplier = velMultiplier;
				solverContact->biasedErr = biasedErr;

				//solverContact->scaledBias = V4Max(zero, scaledBias);
				solverContact->scaledBias = V4Sel(isGreater2, scaledBias, V4Max(zero, scaledBias));

				if(hasMaxImpulse)
				{
					maxImpulse[contactCount-1] = V4Merge(FLoad(con0.maxImpulse), FLoad(con1.maxImpulse), FLoad(con2.maxImpulse),
						FLoad(con3.maxImpulse));					
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
		if(hasMaxImpulse)
		{
			ptr += sizeof(Vec4V) * totalContacts;
		}

		//OK...friction time :-)

		Vec4V maxImpulseScale = V4One();
		{
			const Vec4V staticFriction = V4LoadXYZW(contactBase0->staticFriction, contactBase1->staticFriction,
				contactBase2->staticFriction, contactBase3->staticFriction);

			const Vec4V dynamicFriction = V4LoadXYZW(contactBase0->dynamicFriction, contactBase1->dynamicFriction,
				contactBase2->dynamicFriction, contactBase3->dynamicFriction);			

			PX_ASSERT(totalContacts == contactCount);
			header->dynamicFriction = dynamicFriction;
			header->staticFriction = staticFriction;

			const FrictionPatch& frictionPatch0 = c.frictionPatches[frictionIndex0];
			const FrictionPatch& frictionPatch1 = c.frictionPatches[frictionIndex1];
			const FrictionPatch& frictionPatch2 = c.frictionPatches[frictionIndex2];
			const FrictionPatch& frictionPatch3 = c.frictionPatches[frictionIndex3];

			PxU32 anchorCount0 = frictionPatch0.anchorCount;
			PxU32 anchorCount1 = frictionPatch1.anchorCount;
			PxU32 anchorCount2 = frictionPatch2.anchorCount;
			PxU32 anchorCount3 = frictionPatch3.anchorCount;

			PxU32 clampedAnchorCount0 = hasFinished0 || (contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION) ? 0 : anchorCount0;
			PxU32 clampedAnchorCount1 = hasFinished1 || (contactBase1->materialFlags & PxMaterialFlag::eDISABLE_FRICTION) ? 0 : anchorCount1;
			PxU32 clampedAnchorCount2 = hasFinished2 || (contactBase2->materialFlags & PxMaterialFlag::eDISABLE_FRICTION) ? 0 : anchorCount2;
			PxU32 clampedAnchorCount3 = hasFinished3 || (contactBase3->materialFlags & PxMaterialFlag::eDISABLE_FRICTION) ? 0 : anchorCount3;
			
			const PxU32 maxAnchorCount = PxMax(clampedAnchorCount0, PxMax(clampedAnchorCount1, PxMax(clampedAnchorCount2, clampedAnchorCount3)));

			//if(clampedAnchorCount0 != clampedAnchorCount1 || clampedAnchorCount0 != clampedAnchorCount2 || clampedAnchorCount0 != clampedAnchorCount3)
			//	Ps::debugBreak();


			//const bool haveFriction = maxAnchorCount != 0;
			header->numFrictionConstr	= Ps::to8(maxAnchorCount*2);
			header->numFrictionConstr0 = Ps::to8(clampedAnchorCount0*2);
			header->numFrictionConstr1 = Ps::to8(clampedAnchorCount1*2);
			header->numFrictionConstr2 = Ps::to8(clampedAnchorCount2*2);
			header->numFrictionConstr3 = Ps::to8(clampedAnchorCount3*2);
		
			//KS - TODO - extend this if needed
			header->type = Ps::to8(isDynamic ? DY_SC_TYPE_BLOCK_RB_CONTACT : DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT);

			if(maxAnchorCount)
			{

				//Allocate the shared friction data...

				SolverFrictionSharedData4* PX_RESTRICT fd = reinterpret_cast<SolverFrictionSharedData4*>(ptr);
				ptr += sizeof(SolverFrictionSharedData4);
				PX_UNUSED(fd);

				const BoolV cond =V4IsGrtr(orthoThreshold, V4Abs(normalX));

				const Vec4V t0FallbackX = V4Sel(cond, zero, V4Neg(normalY));
				const Vec4V t0FallbackY = V4Sel(cond, V4Neg(normalZ), normalX);
				const Vec4V t0FallbackZ = V4Sel(cond, normalY, zero);

				//const Vec4V dotNormalVrel = V4MulAdd(normalZ, vrelZ, V4MulAdd(normalY, vrelY, V4Mul(normalX, vrelX)));
				const Vec4V vrelSubNorVelX = V4NegMulSub(normalX, relNorVel, vrelX);
				const Vec4V vrelSubNorVelY = V4NegMulSub(normalY, relNorVel, vrelY);
				const Vec4V vrelSubNorVelZ = V4NegMulSub(normalZ, relNorVel, vrelZ);

				const Vec4V lenSqvrelSubNorVelZ = V4MulAdd(vrelSubNorVelX, vrelSubNorVelX, V4MulAdd(vrelSubNorVelY, vrelSubNorVelY, V4Mul(vrelSubNorVelZ, vrelSubNorVelZ)));

				const BoolV bcon2 = V4IsGrtr(lenSqvrelSubNorVelZ, p1);

				Vec4V t0X = V4Sel(bcon2, vrelSubNorVelX, t0FallbackX);
				Vec4V t0Y = V4Sel(bcon2, vrelSubNorVelY, t0FallbackY);
				Vec4V t0Z = V4Sel(bcon2, vrelSubNorVelZ, t0FallbackZ);


				//Now normalize this...
				const Vec4V recipLen = V4Rsqrt(V4MulAdd(t0Z, t0Z, V4MulAdd(t0Y, t0Y, V4Mul(t0X, t0X))));

				t0X = V4Mul(t0X, recipLen);
				t0Y = V4Mul(t0Y, recipLen);
				t0Z = V4Mul(t0Z, recipLen);

				Vec4V t1X = V4NegMulSub(normalZ, t0Y, V4Mul(normalY, t0Z));
				Vec4V t1Y = V4NegMulSub(normalX, t0Z, V4Mul(normalZ, t0X));
				Vec4V t1Z = V4NegMulSub(normalY, t0X, V4Mul(normalX, t0Y));

				PX_ASSERT((uintptr_t(descs[0].frictionPtr) & 0xF) == 0);
				PX_ASSERT((uintptr_t(descs[1].frictionPtr) & 0xF) == 0);
				PX_ASSERT((uintptr_t(descs[2].frictionPtr) & 0xF) == 0);
				PX_ASSERT((uintptr_t(descs[3].frictionPtr) & 0xF) == 0);


				PxU8* PX_RESTRICT writeback0 = descs[0].frictionPtr + frictionPatchWritebackAddrIndex0*sizeof(FrictionPatch);
				PxU8* PX_RESTRICT writeback1 = descs[1].frictionPtr + frictionPatchWritebackAddrIndex1*sizeof(FrictionPatch);
				PxU8* PX_RESTRICT writeback2 = descs[2].frictionPtr + frictionPatchWritebackAddrIndex2*sizeof(FrictionPatch);
				PxU8* PX_RESTRICT writeback3 = descs[3].frictionPtr + frictionPatchWritebackAddrIndex3*sizeof(FrictionPatch);

				PxU32 index0 = 0, index1 = 0, index2 = 0, index3 = 0;

				fd->broken = bFalse;
				fd->frictionBrokenWritebackByte[0] = writeback0;
				fd->frictionBrokenWritebackByte[1] = writeback1;
				fd->frictionBrokenWritebackByte[2] = writeback2;
				fd->frictionBrokenWritebackByte[3] = writeback3;


				fd->normalX[0] = t0X;
				fd->normalY[0] = t0Y;
				fd->normalZ[0] = t0Z;

				fd->normalX[1] = t1X;
				fd->normalY[1] = t1Y;
				fd->normalZ[1] = t1Z;

				Vec4V* PX_RESTRICT appliedForces = reinterpret_cast<Vec4V*>(ptr);
				ptr += sizeof(Vec4V)*header->numFrictionConstr;

				PxMemZero(appliedForces, sizeof(Vec4V) * header->numFrictionConstr);

				for(PxU32 j = 0; j < maxAnchorCount; j++)
				{
					Ps::prefetchLine(ptr, 384);
					Ps::prefetchLine(ptr, 512);
					Ps::prefetchLine(ptr, 640);
					SolverContactFrictionBase4* PX_RESTRICT f0 = reinterpret_cast<SolverContactFrictionBase4*>(ptr);
					ptr += frictionSize;
					SolverContactFrictionBase4* PX_RESTRICT f1 = reinterpret_cast<SolverContactFrictionBase4*>(ptr);
					ptr += frictionSize;

					index0 = j < clampedAnchorCount0 ? j : index0;
					index1 = j < clampedAnchorCount1 ? j : index1;
					index2 = j < clampedAnchorCount2 ? j : index2;
					index3 = j < clampedAnchorCount3 ? j : index3;

					if(j >= clampedAnchorCount0)
						maxImpulseScale = V4SetX(maxImpulseScale, fZero);
					if(j >= clampedAnchorCount1)
						maxImpulseScale = V4SetY(maxImpulseScale, fZero);
					if(j >= clampedAnchorCount2)
						maxImpulseScale = V4SetZ(maxImpulseScale, fZero);
					if(j >= clampedAnchorCount3)
						maxImpulseScale = V4SetW(maxImpulseScale, fZero);

					t0X = V4Mul(maxImpulseScale, t0X);
					t0Y = V4Mul(maxImpulseScale, t0Y);
					t0Z = V4Mul(maxImpulseScale, t0Z);

					t1X = V4Mul(maxImpulseScale, t1X);
					t1Y = V4Mul(maxImpulseScale, t1Y);
					t1Z = V4Mul(maxImpulseScale, t1Z);


					Vec3V body0Anchor0 = V3LoadU(frictionPatch0.body0Anchors[index0]);
					Vec3V body0Anchor1 = V3LoadU(frictionPatch1.body0Anchors[index1]);
					Vec3V body0Anchor2 = V3LoadU(frictionPatch2.body0Anchors[index2]);
					Vec3V body0Anchor3 = V3LoadU(frictionPatch3.body0Anchors[index3]);

					Vec4V ra0 = Vec4V_From_Vec3V(QuatRotate(bodyFrame00q, body0Anchor0));
					Vec4V ra1 = Vec4V_From_Vec3V(QuatRotate(bodyFrame01q, body0Anchor1));
					Vec4V ra2 = Vec4V_From_Vec3V(QuatRotate(bodyFrame02q, body0Anchor2));
					Vec4V ra3 = Vec4V_From_Vec3V(QuatRotate(bodyFrame03q, body0Anchor3));

					Vec4V raX, raY, raZ;
					PX_TRANSPOSE_44_34(ra0, ra1, ra2, ra3, raX, raY, raZ);

					/*raX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raX)), zero, raX);
					raY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raY)), zero, raY);
					raZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raZ)), zero, raZ);*/

					const Vec4V raWorldX = V4Add(raX, bodyFrame0pX);
					const Vec4V raWorldY = V4Add(raY, bodyFrame0pY);
					const Vec4V raWorldZ = V4Add(raZ, bodyFrame0pZ);

					Vec3V body1Anchor0 = V3LoadU(frictionPatch0.body1Anchors[index0]);	
					Vec3V body1Anchor1 = V3LoadU(frictionPatch1.body1Anchors[index1]);
					Vec3V body1Anchor2 = V3LoadU(frictionPatch2.body1Anchors[index2]);
					Vec3V body1Anchor3 = V3LoadU(frictionPatch3.body1Anchors[index3]);
				
					Vec4V rb0 = Vec4V_From_Vec3V(QuatRotate(bodyFrame10q, body1Anchor0));
					Vec4V rb1 = Vec4V_From_Vec3V(QuatRotate(bodyFrame11q, body1Anchor1));
					Vec4V rb2 = Vec4V_From_Vec3V(QuatRotate(bodyFrame12q, body1Anchor2));
					Vec4V rb3 = Vec4V_From_Vec3V(QuatRotate(bodyFrame13q, body1Anchor3));

					Vec4V rbX, rbY, rbZ;
					PX_TRANSPOSE_44_34(rb0, rb1, rb2, rb3, rbX, rbY, rbZ);

					/*rbX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbX)), zero, rbX);
					rbY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbY)), zero, rbY);
					rbZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbZ)), zero, rbZ);*/

					const Vec4V rbWorldX = V4Add(rbX, bodyFrame1pX);
					const Vec4V rbWorldY = V4Add(rbY, bodyFrame1pY);
					const Vec4V rbWorldZ = V4Add(rbZ, bodyFrame1pZ);

					Vec4V errorX = V4Sub(raWorldX, rbWorldX);
					Vec4V errorY = V4Sub(raWorldY, rbWorldY);
					Vec4V errorZ = V4Sub(raWorldZ, rbWorldZ);

					/*errorX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(errorX)), zero, errorX);
					errorY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(errorY)), zero, errorY);
					errorZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(errorZ)), zero, errorZ);*/

					//KS - todo - get this working with per-point friction
					PxU32 contactIndex0 = c.contactID[frictionIndex0][index0];
					PxU32 contactIndex1 = c.contactID[frictionIndex1][index1];
					PxU32 contactIndex2 = c.contactID[frictionIndex2][index2];
					PxU32 contactIndex3 = c.contactID[frictionIndex3][index3];

					//Ensure that the ocntact indices are valid
					PX_ASSERT(contactIndex0 == 0xffff || contactIndex0 < descs[0].numContacts);
					PX_ASSERT(contactIndex1 == 0xffff || contactIndex1 < descs[1].numContacts);
					PX_ASSERT(contactIndex2 == 0xffff || contactIndex2 < descs[2].numContacts);
					PX_ASSERT(contactIndex3 == 0xffff || contactIndex3 < descs[3].numContacts);

					Vec4V targetVel0 = V4LoadA(contactIndex0 == 0xFFFF ? &contactBase0->targetVel.x : &descs[0].contacts[contactIndex0].targetVel.x);
					Vec4V targetVel1 = V4LoadA(contactIndex1 == 0xFFFF ? &contactBase0->targetVel.x : &descs[1].contacts[contactIndex1].targetVel.x);
					Vec4V targetVel2 = V4LoadA(contactIndex2 == 0xFFFF ? &contactBase0->targetVel.x : &descs[2].contacts[contactIndex2].targetVel.x);
					Vec4V targetVel3 = V4LoadA(contactIndex3 == 0xFFFF ? &contactBase0->targetVel.x : &descs[3].contacts[contactIndex3].targetVel.x);

					Vec4V targetVelX, targetVelY, targetVelZ;
					PX_TRANSPOSE_44_34(targetVel0, targetVel1, targetVel2, targetVel3, targetVelX, targetVelY, targetVelZ);

					
					{
						Vec4V raXnX = V4NegMulSub(raZ, t0Y, V4Mul(raY, t0Z));
						Vec4V raXnY = V4NegMulSub(raX, t0Z, V4Mul(raZ, t0X));
						Vec4V raXnZ = V4NegMulSub(raY, t0X, V4Mul(raX, t0Y));

						raXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnX)), zero, raXnX);
						raXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnY)), zero, raXnY);
						raXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnZ)), zero, raXnZ);

						Vec4V delAngVel0X = V4Mul(invInertia0X0, raXnX);
						Vec4V delAngVel0Y = V4Mul(invInertia0X1, raXnX);
						Vec4V delAngVel0Z = V4Mul(invInertia0X2, raXnX);

						delAngVel0X = V4MulAdd(invInertia0Y0, raXnY, delAngVel0X);
						delAngVel0Y = V4MulAdd(invInertia0Y1, raXnY, delAngVel0Y);
						delAngVel0Z = V4MulAdd(invInertia0Y2, raXnY, delAngVel0Z);

						delAngVel0X = V4MulAdd(invInertia0Z0, raXnZ, delAngVel0X);
						delAngVel0Y = V4MulAdd(invInertia0Z1, raXnZ, delAngVel0Y);
						delAngVel0Z = V4MulAdd(invInertia0Z2, raXnZ, delAngVel0Z);

						const Vec4V dotDelAngVel0 = V4MulAdd(delAngVel0Z, delAngVel0Z, V4MulAdd(delAngVel0Y, delAngVel0Y, V4Mul(delAngVel0X, delAngVel0X)));
					
						Vec4V resp = V4MulAdd(dotDelAngVel0, angDom0, invMass0D0);

						const Vec4V tVel0 = V4MulAdd(t0Z, linVelT20, V4MulAdd(t0Y, linVelT10, V4Mul(t0X, linVelT00)));
						Vec4V vrel = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4MulAdd(raXnX, angVelT00, tVel0)));

						if(isDynamic)
						{
							SolverContactFrictionDynamic4* PX_RESTRICT dynamicF0 = static_cast<SolverContactFrictionDynamic4*>(f0);

							Vec4V rbXnX = V4NegMulSub(rbZ, t0Y, V4Mul(rbY, t0Z));
							Vec4V rbXnY = V4NegMulSub(rbX, t0Z, V4Mul(rbZ, t0X));
							Vec4V rbXnZ = V4NegMulSub(rbY, t0X, V4Mul(rbX, t0Y));

							rbXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnX)), zero, rbXnX);
							rbXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnY)), zero, rbXnY);
							rbXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnZ)), zero, rbXnZ);

							Vec4V delAngVel1X = V4Mul(invInertia1X0, rbXnX);
							Vec4V delAngVel1Y = V4Mul(invInertia1X1, rbXnX);
							Vec4V delAngVel1Z = V4Mul(invInertia1X2, rbXnX);

							delAngVel1X = V4MulAdd(invInertia1Y0, rbXnY, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Y1, rbXnY, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Y2, rbXnY, delAngVel1Z);

							delAngVel1X = V4MulAdd(invInertia1Z0, rbXnZ, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Z1, rbXnZ, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Z2, rbXnZ, delAngVel1Z);					
						
							const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1Z, delAngVel1Z, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1X, delAngVel1X)));
							
							const Vec4V resp1 = V4MulAdd(dotDelAngVel1, angDom1, invMass1D1);

							resp = V4Add(resp, resp1);
							
							dynamicF0->rbXnX = delAngVel1X;
							dynamicF0->rbXnY = delAngVel1Y;
							dynamicF0->rbXnZ = delAngVel1Z;

							const Vec4V tVel1 = V4MulAdd(t0Z, linVelT21, V4MulAdd(t0Y, linVelT11, V4Mul(t0X, linVelT01)));
							const Vec4V vel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));

							vrel = V4Sub(vrel, vel1);
						}
						else if(hasKinematic)
						{
							const Vec4V rbXnX = V4NegMulSub(rbZ, t0Y, V4Mul(rbY, t0Z));
							const Vec4V rbXnY = V4NegMulSub(rbX, t0Z, V4Mul(rbZ, t0X));
							const Vec4V rbXnZ = V4NegMulSub(rbY, t0X, V4Mul(rbX, t0Y));

							const Vec4V tVel1 = V4MulAdd(t0Z, linVelT21, V4MulAdd(t0Y, linVelT11, V4Mul(t0X, linVelT01)));
							const Vec4V dotRbXnAngVel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));

							vrel = V4Sub(vrel, dotRbXnAngVel1);
						}


						const Vec4V velMultiplier = V4Mul(maxImpulseScale, V4Sel(V4IsGrtr(resp, zero), V4Div(p84, resp), zero));

						Vec4V bias = V4Scale(V4MulAdd(t0Z, errorZ, V4MulAdd(t0Y, errorY, V4Mul(t0X, errorX))), invDt);

						Vec4V targetVel = V4MulAdd(t0Z, targetVelZ,V4MulAdd(t0Y, targetVelY, V4Mul(t0X, targetVelX)));
						targetVel = V4Sub(targetVel, vrel);
						f0->targetVelocity = V4Neg(V4Mul(targetVel, velMultiplier));
						bias = V4Sub(bias, targetVel);

						f0->raXnX = delAngVel0X;
						f0->raXnY = delAngVel0Y;
						f0->raXnZ = delAngVel0Z;
						f0->scaledBias = V4Mul(bias, velMultiplier);
						f0->velMultiplier = velMultiplier;								
					}

					{
						Vec4V raXnX = V4NegMulSub(raZ, t1Y, V4Mul(raY, t1Z));
						Vec4V raXnY = V4NegMulSub(raX, t1Z, V4Mul(raZ, t1X));
						Vec4V raXnZ = V4NegMulSub(raY, t1X, V4Mul(raX, t1Y));

						raXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnX)), zero, raXnX);
						raXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnY)), zero, raXnY);
						raXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnZ)), zero, raXnZ);

						Vec4V delAngVel0X = V4Mul(invInertia0X0, raXnX);
						Vec4V delAngVel0Y = V4Mul(invInertia0X1, raXnX);
						Vec4V delAngVel0Z = V4Mul(invInertia0X2, raXnX);

						delAngVel0X = V4MulAdd(invInertia0Y0, raXnY, delAngVel0X);
						delAngVel0Y = V4MulAdd(invInertia0Y1, raXnY, delAngVel0Y);
						delAngVel0Z = V4MulAdd(invInertia0Y2, raXnY, delAngVel0Z);

						delAngVel0X = V4MulAdd(invInertia0Z0, raXnZ, delAngVel0X);
						delAngVel0Y = V4MulAdd(invInertia0Z1, raXnZ, delAngVel0Y);
						delAngVel0Z = V4MulAdd(invInertia0Z2, raXnZ, delAngVel0Z);

						const Vec4V dotDelAngVel0 = V4MulAdd(delAngVel0Z, delAngVel0Z, V4MulAdd(delAngVel0Y, delAngVel0Y, V4Mul(delAngVel0X, delAngVel0X)));
					
						Vec4V resp = V4MulAdd(dotDelAngVel0, angDom0, invMass0D0);

						const Vec4V tVel0 = V4MulAdd(t1Z, linVelT20, V4MulAdd(t1Y, linVelT10, V4Mul(t1X, linVelT00)));
						Vec4V vrel = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4MulAdd(raXnX, angVelT00, tVel0)));

						if(isDynamic)
						{
							SolverContactFrictionDynamic4* PX_RESTRICT dynamicF1 = static_cast<SolverContactFrictionDynamic4*>(f1);

							Vec4V rbXnX = V4NegMulSub(rbZ, t1Y, V4Mul(rbY, t1Z));
							Vec4V rbXnY = V4NegMulSub(rbX, t1Z, V4Mul(rbZ, t1X));
							Vec4V rbXnZ = V4NegMulSub(rbY, t1X, V4Mul(rbX, t1Y));

							rbXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnX)), zero, rbXnX);
							rbXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnY)), zero, rbXnY);
							rbXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnZ)), zero, rbXnZ);

							Vec4V delAngVel1X = V4Mul(invInertia1X0, rbXnX);
							Vec4V delAngVel1Y = V4Mul(invInertia1X1, rbXnX);
							Vec4V delAngVel1Z = V4Mul(invInertia1X2, rbXnX);

							delAngVel1X = V4MulAdd(invInertia1Y0, rbXnY, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Y1, rbXnY, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Y2, rbXnY, delAngVel1Z);

							delAngVel1X = V4MulAdd(invInertia1Z0, rbXnZ, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Z1, rbXnZ, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Z2, rbXnZ, delAngVel1Z);					
						
							const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1Z, delAngVel1Z, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1X, delAngVel1X)));
							
							const Vec4V resp1 = V4MulAdd(dotDelAngVel1, angDom1, invMass1D1);

							resp = V4Add(resp, resp1);
							
							dynamicF1->rbXnX = delAngVel1X;
							dynamicF1->rbXnY = delAngVel1Y;
							dynamicF1->rbXnZ = delAngVel1Z;

							const Vec4V tVel1 = V4MulAdd(t1Z, linVelT21, V4MulAdd(t1Y, linVelT11, V4Mul(t1X, linVelT01)));
							const Vec4V vel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));

							vrel = V4Sub(vrel, vel1);

						}
						else if(hasKinematic)
						{
							const Vec4V rbXnX = V4NegMulSub(rbZ, t1Y, V4Mul(rbY, t1Z));
							const Vec4V rbXnY = V4NegMulSub(rbX, t1Z, V4Mul(rbZ, t1X));
							const Vec4V rbXnZ = V4NegMulSub(rbY, t1X, V4Mul(rbX, t1Y));

							const Vec4V tVel1 = V4MulAdd(t1Z, linVelT21, V4MulAdd(t1Y, linVelT11, V4Mul(t1X, linVelT01)));
							const Vec4V dotRbXnAngVel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));

							vrel = V4Sub(vrel, dotRbXnAngVel1);
						}


						const Vec4V velMultiplier = V4Mul(maxImpulseScale, V4Sel(V4IsGrtr(resp, zero), V4Div(p84, resp), zero));

						Vec4V bias = V4Scale(V4MulAdd(t1Z, errorZ, V4MulAdd(t1Y, errorY, V4Mul(t1X, errorX))), invDt);

						Vec4V targetVel = V4MulAdd(t1Z, targetVelZ,V4MulAdd(t1Y, targetVelY, V4Mul(t1X, targetVelX)));
						targetVel = V4Sub(targetVel, vrel);
						f1->targetVelocity = V4Neg(V4Mul(targetVel, velMultiplier));
						bias = V4Sub(bias, targetVel);
						f1->raXnX = delAngVel0X;
						f1->raXnY = delAngVel0Y;
						f1->raXnZ = delAngVel0Z;
						f1->scaledBias = V4Mul(bias, velMultiplier);
						f1->velMultiplier = velMultiplier;
					}				
				}

				frictionPatchWritebackAddrIndex0++;
				frictionPatchWritebackAddrIndex1++;
				frictionPatchWritebackAddrIndex2++;
				frictionPatchWritebackAddrIndex3++;
			}
		}
	}
}



PX_FORCE_INLINE void computeBlockStreamFrictionByteSizes(const CorrelationBuffer& c,
														 PxU32& _frictionPatchByteSize, PxU32& _numFrictionPatches,
														 PxU32 frictionPatchStartIndex, PxU32 frictionPatchEndIndex)
{
	// PT: use local vars to remove LHS
	PxU32 numFrictionPatches = 0;

	for(PxU32 i = frictionPatchStartIndex; i < frictionPatchEndIndex; i++)
	{
		//Friction patches.
		if(c.correlationListHeads[i] != CorrelationBuffer::LIST_END)
			numFrictionPatches++;
	}
	PxU32 frictionPatchByteSize = numFrictionPatches*sizeof(FrictionPatch);

	_numFrictionPatches = numFrictionPatches;

	//16-byte alignment.
	_frictionPatchByteSize = ((frictionPatchByteSize + 0x0f) & ~0x0f);
	PX_ASSERT(0 == (_frictionPatchByteSize & 0x0f));
}

static bool reserveFrictionBlockStreams(const CorrelationBuffer& c, PxConstraintAllocator& constraintAllocator, PxU32 frictionPatchStartIndex, PxU32 frictionPatchEndIndex,
						FrictionPatch*& _frictionPatches,
						PxU32& numFrictionPatches)
{

	//From frictionPatchStream we just need to reserve a single buffer.
	PxU32 frictionPatchByteSize = 0;
	//Compute the sizes of all the buffers.

	computeBlockStreamFrictionByteSizes(c, frictionPatchByteSize, numFrictionPatches, frictionPatchStartIndex, frictionPatchEndIndex);

	FrictionPatch* frictionPatches = NULL;
	//If the constraint block reservation didn't fail then reserve the friction buffer too.
	if(frictionPatchByteSize > 0)
	{
		frictionPatches = reinterpret_cast<FrictionPatch*>(constraintAllocator.reserveFrictionData(frictionPatchByteSize));

		if(0==frictionPatches || (reinterpret_cast<FrictionPatch*>(-1))==frictionPatches)
		{
			if(0==frictionPatches)
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
				frictionPatches=NULL;
			}
		}
	}

	_frictionPatches = frictionPatches;

	//Return true if neither of the two block reservations failed.
	return (0==frictionPatchByteSize || frictionPatches);
}

//The persistent friction patch correlation/allocation will already have happenned as this is per-pair.
//This function just computes the size of the combined solve data.
void computeBlockStreamByteSizes4(PxSolverContactDesc* descs,
								PxU32& _solverConstraintByteSize, PxU32* _axisConstraintCount,
								const CorrelationBuffer& c)
{
	PX_ASSERT(0 == _solverConstraintByteSize);

	PxU32 maxPatches = 0;
	PxU32 maxFrictionPatches = 0;
	PxU32 maxContactCount[CorrelationBuffer::MAX_FRICTION_PATCHES];
	PxU32 maxFrictionCount[CorrelationBuffer::MAX_FRICTION_PATCHES];
	PxMemZero(maxContactCount, sizeof(maxContactCount));
	PxMemZero(maxFrictionCount, sizeof(maxFrictionCount));
	bool hasMaxImpulse = false;

	for(PxU32 a = 0; a < 4; ++a)
	{
		PxU32 axisConstraintCount = 0;
		hasMaxImpulse = hasMaxImpulse || descs[a].hasMaxImpulse;
		for(PxU32 i = 0; i < descs[a].numFrictionPatches; i++)
		{
			PxU32 ind = i + descs[a].startFrictionPatchIndex;

			const FrictionPatch& frictionPatch = c.frictionPatches[ind];

			const bool haveFriction = (frictionPatch.materialFlags & PxMaterialFlag::eDISABLE_FRICTION) == 0
				&& frictionPatch.anchorCount != 0;
			//Solver constraint data.
			if(c.frictionPatchContactCounts[ind]!=0)
			{
				maxContactCount[i] = PxMax(c.frictionPatchContactCounts[ind], maxContactCount[i]);
				axisConstraintCount += c.frictionPatchContactCounts[ind];

				if(haveFriction)
				{
					const PxU32 fricCount = PxU32(c.frictionPatches[ind].anchorCount) * 2;
					maxFrictionCount[i] = PxMax(fricCount, maxFrictionCount[i]);
					axisConstraintCount += fricCount;
				}
			}
		}
		maxPatches = PxMax(descs[a].numFrictionPatches, maxPatches);
		_axisConstraintCount[a] = axisConstraintCount;
	}

	for(PxU32 a = 0; a < maxPatches; ++a)
	{
		if(maxFrictionCount[a] > 0)
			maxFrictionPatches++;
	}


	PxU32 totalContacts = 0, totalFriction = 0;
	for(PxU32 a = 0; a < maxPatches; ++a)
	{
		totalContacts += maxContactCount[a];
		totalFriction += maxFrictionCount[a];
	}

	//OK, we have a given number of friction patches, contact points and friction constraints so we can calculate how much memory we need

	//Body 2 is considered static if it is either *not dynamic* or *kinematic*

	bool hasDynamicBody = false;
	for(PxU32 a = 0; a < 4; ++a)
	{
		hasDynamicBody = hasDynamicBody || ((descs[a].bodyState1 == PxSolverContactDesc::eDYNAMIC_BODY));
	}
	

	const bool isStatic = !hasDynamicBody;

	const PxU32 headerSize = sizeof(SolverContactHeader4) * maxPatches + sizeof(SolverFrictionSharedData4) * maxFrictionPatches;
	PxU32 constraintSize = isStatic ? (sizeof(SolverContactBatchPointBase4) * totalContacts) + ( sizeof(SolverContactFrictionBase4) * totalFriction) : 
		(sizeof(SolverContactBatchPointDynamic4) * totalContacts) + (sizeof(SolverContactFrictionDynamic4) * totalFriction);

	//Space for the appliedForce buffer
	constraintSize += sizeof(Vec4V)*(totalContacts+totalFriction);

	//If we have max impulse, reserve a buffer for it
	if(hasMaxImpulse)
		constraintSize += sizeof(Ps::aos::Vec4V) * totalContacts;

	_solverConstraintByteSize =  ((constraintSize + headerSize + 0x0f) & ~0x0f);
	PX_ASSERT(0 == (_solverConstraintByteSize & 0x0f));
}

static SolverConstraintPrepState::Enum reserveBlockStreams4(PxSolverContactDesc* descs, Dy::CorrelationBuffer& c,
						PxU8*& solverConstraint, PxU32* axisConstraintCount,
						PxU32& solverConstraintByteSize, 
						PxConstraintAllocator& constraintAllocator)
{
	PX_ASSERT(NULL == solverConstraint);
	PX_ASSERT(0 == solverConstraintByteSize);

	//Compute the sizes of all the buffers.
	computeBlockStreamByteSizes4(descs, 
		solverConstraintByteSize, axisConstraintCount,
		c);

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

	return ((0==constraintBlockByteSize || constraintBlock)) ? SolverConstraintPrepState::eSUCCESS : SolverConstraintPrepState::eOUT_OF_MEMORY;
}

SolverConstraintPrepState::Enum createFinalizeSolverContacts4(
	Dy::CorrelationBuffer& c,
	PxSolverContactDesc* blockDescs,
	const PxReal invDtF32,
	PxReal bounceThresholdF32,
	PxReal	frictionOffsetThreshold,
	PxReal correlationDistance,
	PxReal solverOffsetSlop,
	PxConstraintAllocator& constraintAllocator)
{
	PX_ALIGN(16, PxReal invMassScale0[4]);
	PX_ALIGN(16, PxReal invMassScale1[4]);
	PX_ALIGN(16, PxReal invInertiaScale0[4]);
	PX_ALIGN(16, PxReal invInertiaScale1[4]);

	c.frictionPatchCount = 0;
	c.contactPatchCount = 0;

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxSolverContactDesc& blockDesc = blockDescs[a];

		invMassScale0[a] = blockDesc.mInvMassScales.linear0;
		invMassScale1[a] = blockDesc.mInvMassScales.linear1;
		invInertiaScale0[a] = blockDesc.mInvMassScales.angular0;
		invInertiaScale1[a] = blockDesc.mInvMassScales.angular1;

		blockDesc.startFrictionPatchIndex = c.frictionPatchCount;
		if (!(blockDesc.disableStrongFriction))
		{
			bool valid = getFrictionPatches(c, blockDesc.frictionPtr, blockDesc.frictionCount,
				blockDesc.bodyFrame0, blockDesc.bodyFrame1, correlationDistance);
			if (!valid)
				return SolverConstraintPrepState::eUNBATCHABLE;
		}
		//Create the contact patches
		blockDesc.startContactPatchIndex = c.contactPatchCount;
		if (!createContactPatches(c, blockDesc.contacts, blockDesc.numContacts, PXC_SAME_NORMAL))
			return SolverConstraintPrepState::eUNBATCHABLE;
		blockDesc.numContactPatches = PxU16(c.contactPatchCount - blockDesc.startContactPatchIndex);

		bool overflow = correlatePatches(c, blockDesc.contacts, blockDesc.bodyFrame0, blockDesc.bodyFrame1, PXC_SAME_NORMAL,
			blockDesc.startContactPatchIndex, blockDesc.startFrictionPatchIndex);

		if (overflow)
			return SolverConstraintPrepState::eUNBATCHABLE;

		growPatches(c, blockDesc.contacts, blockDesc.bodyFrame0, blockDesc.bodyFrame1, correlationDistance, blockDesc.startFrictionPatchIndex,
			frictionOffsetThreshold + blockDescs[a].restDistance);

		//Remove the empty friction patches - do we actually need to do this?
		for (PxU32 p = c.frictionPatchCount; p > blockDesc.startFrictionPatchIndex; --p)
		{
			if (c.correlationListHeads[p - 1] == 0xffff)
			{
				//We have an empty patch...need to bin this one...
				for (PxU32 p2 = p; p2 < c.frictionPatchCount; ++p2)
				{
					c.correlationListHeads[p2 - 1] = c.correlationListHeads[p2];
					c.frictionPatchContactCounts[p2 - 1] = c.frictionPatchContactCounts[p2];
				}
				c.frictionPatchCount--;
			}
		}

		PxU32 numFricPatches = c.frictionPatchCount - blockDesc.startFrictionPatchIndex;
		blockDesc.numFrictionPatches = numFricPatches;
	}

	FrictionPatch* frictionPatchArray[4];
	PxU32 frictionPatchCounts[4];

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxSolverContactDesc& blockDesc = blockDescs[a];

		const bool successfulReserve = reserveFrictionBlockStreams(c, constraintAllocator, blockDesc.startFrictionPatchIndex, blockDesc.numFrictionPatches + blockDesc.startFrictionPatchIndex,
			frictionPatchArray[a],
			frictionPatchCounts[a]);

		//KS - TODO - how can we recover if we failed to allocate this memory?
		if (!successfulReserve)
		{
			return SolverConstraintPrepState::eOUT_OF_MEMORY;
		}
	}
	//At this point, all the friction data has been calculated, the correlation has been done. Provided this was all successful, 
	//we are ready to create the batched constraints

	PxU8* solverConstraint = NULL;
	PxU32 solverConstraintByteSize = 0;



	{
		PxU32 axisConstraintCount[4];
		SolverConstraintPrepState::Enum state = reserveBlockStreams4(blockDescs, c,
			solverConstraint, axisConstraintCount,
			solverConstraintByteSize,
			constraintAllocator);

		if (state != SolverConstraintPrepState::eSUCCESS)
			return state;


		for (PxU32 a = 0; a < 4; ++a)
		{

			FrictionPatch* frictionPatches = frictionPatchArray[a];

			PxSolverContactDesc& blockDesc = blockDescs[a];
			PxSolverConstraintDesc& desc = *blockDesc.desc;
			blockDesc.frictionPtr = reinterpret_cast<PxU8*>(frictionPatches);
			blockDesc.frictionCount = Ps::to8(frictionPatchCounts[a]);

			//Initialise friction buffer.
			if (frictionPatches)
			{
				// PT: TODO: revisit this... not very satisfying
				//const PxU32 maxSize = numFrictionPatches*sizeof(FrictionPatch);
				Ps::prefetchLine(frictionPatches);
				Ps::prefetchLine(frictionPatches, 128);
				Ps::prefetchLine(frictionPatches, 256);

				for (PxU32 i = 0; i<blockDesc.numFrictionPatches; i++)
				{
					if (c.correlationListHeads[blockDesc.startFrictionPatchIndex + i] != CorrelationBuffer::LIST_END)
					{
						//*frictionPatches++ = c.frictionPatches[blockDesc.startFrictionPatchIndex + i];
						PxMemCopy(frictionPatches++, &c.frictionPatches[blockDesc.startFrictionPatchIndex + i], sizeof(FrictionPatch));
						//Ps::prefetchLine(frictionPatches, 256);
					}
				}
			}


			blockDesc.axisConstraintCount += Ps::to16(axisConstraintCount[a]);

			desc.constraint = solverConstraint;
			desc.constraintLengthOver16 = Ps::to16(solverConstraintByteSize / 16);
			desc.writeBackLengthOver4 = PxU16(blockDesc.numContacts);
			desc.writeBack = blockDesc.contactForces;
		}

		const Vec4V iMassScale0 = V4LoadA(invMassScale0);
		const Vec4V iInertiaScale0 = V4LoadA(invInertiaScale0);
		const Vec4V iMassScale1 = V4LoadA(invMassScale1);
		const Vec4V iInertiaScale1 = V4LoadA(invInertiaScale1);

		setupFinalizeSolverConstraints4(blockDescs, c, solverConstraint, invDtF32, bounceThresholdF32, solverOffsetSlop,
			iMassScale0, iInertiaScale0, iMassScale1, iInertiaScale1);

		PX_ASSERT((*solverConstraint == DY_SC_TYPE_BLOCK_RB_CONTACT) || (*solverConstraint == DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT));

		*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize)) = 0;
	}
	return SolverConstraintPrepState::eSUCCESS;
}


//This returns 1 of 3 states: success, unbatchable or out-of-memory. If the constraint is unbatchable, we must fall back on 4 separate constraint
//prep calls
SolverConstraintPrepState::Enum createFinalizeSolverContacts4(
	PxsContactManagerOutput** cmOutputs,
	ThreadContext& threadContext,
	PxSolverContactDesc* blockDescs,
	const PxReal invDtF32,
	PxReal bounceThresholdF32,
	PxReal	frictionOffsetThreshold,
	PxReal correlationDistance,
	PxReal solverOffsetSlop,
	PxConstraintAllocator& constraintAllocator)
{

	for (PxU32 a = 0; a < 4; ++a)
	{
		blockDescs[a].desc->constraintLengthOver16 = 0;
	}

	PX_ASSERT(cmOutputs[0]->nbContacts && cmOutputs[1]->nbContacts && cmOutputs[2]->nbContacts && cmOutputs[3]->nbContacts);


	Gu::ContactBuffer& buffer = threadContext.mContactBuffer;

	buffer.count = 0;

	//PxTransform idt = PxTransform(PxIdentity);

	CorrelationBuffer& c = threadContext.mCorrelationBuffer;

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxSolverContactDesc& blockDesc = blockDescs[a];
		PxSolverConstraintDesc& desc = *blockDesc.desc;

		//blockDesc.startContactIndex = buffer.count;
		blockDesc.contacts = buffer.contacts + buffer.count;

		Ps::prefetchLine(desc.bodyA);
		Ps::prefetchLine(desc.bodyB);


		if ((buffer.count + cmOutputs[a]->nbContacts) > 64)
		{
			return SolverConstraintPrepState::eUNBATCHABLE;
		}

		bool hasMaxImpulse = false;
		bool hasTargetVelocity = false;

		//OK...do the correlation here as well...
		Ps::prefetchLine(blockDescs[a].frictionPtr);
		Ps::prefetchLine(blockDescs[a].frictionPtr, 64);
		Ps::prefetchLine(blockDescs[a].frictionPtr, 128);

		if (a < 3)
		{
			Ps::prefetchLine(cmOutputs[a]->contactPatches);
			Ps::prefetchLine(cmOutputs[a]->contactPoints);
		}

		PxReal invMassScale0, invMassScale1, invInertiaScale0, invInertiaScale1;

		const PxReal defaultMaxImpulse = PxMin(blockDesc.data0->maxContactImpulse, blockDesc.data1->maxContactImpulse);

		PxU32 contactCount = extractContacts(buffer, *cmOutputs[a], hasMaxImpulse, hasTargetVelocity, invMassScale0, invMassScale1,
			invInertiaScale0, invInertiaScale1, defaultMaxImpulse);

		if (contactCount == 0)
			return SolverConstraintPrepState::eUNBATCHABLE;

		blockDesc.numContacts = contactCount;
		blockDesc.hasMaxImpulse = hasMaxImpulse;
		blockDesc.disableStrongFriction = blockDesc.disableStrongFriction || hasTargetVelocity;

		blockDesc.mInvMassScales.linear0 *= invMassScale0;
		blockDesc.mInvMassScales.linear1 *= invMassScale1;
		blockDesc.mInvMassScales.angular0 *= invInertiaScale0;
		blockDesc.mInvMassScales.angular1 *= invInertiaScale1;

		//blockDesc.frictionPtr = &blockDescs[a].frictionPtr;
		//blockDesc.frictionCount = blockDescs[a].frictionCount;

	}
	return createFinalizeSolverContacts4(c, blockDescs,
		invDtF32, bounceThresholdF32,	frictionOffsetThreshold,
		correlationDistance, solverOffsetSlop, constraintAllocator);
}




}

}


