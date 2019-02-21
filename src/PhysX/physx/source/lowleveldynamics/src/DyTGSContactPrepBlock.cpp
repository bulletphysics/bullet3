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
#include "DyTGSDynamics.h"

using namespace physx;
using namespace Gu;


#include "PsVecMath.h"
#include "PxContactModifyCallback.h"
#include "PxsMaterialManager.h"
#include "PxsMaterialCombiner.h"
#include "DyContactPrepShared.h"
#include "DySolverConstraint1D.h"
#include "DyConstraintPrep.h"

#include "CmConeLimitHelper.h"

#include "DySolverContext.h"

namespace physx
{
namespace Dy
{

	inline bool ValidateVec4(const Vec4V v)
	{
		PX_ALIGN(16, PxVec4 vF);
		Ps::aos::V4StoreA(v, &vF.x);
		return vF.isFinite();
	}

	PX_FORCE_INLINE void QuatRotate4(const Vec4VArg qx, const Vec4VArg qy, const Vec4VArg qz, const Vec4VArg qw, const Vec4VArg vx, const Vec4VArg vy, const Vec4VArg vz,
		Vec4V& rX, Vec4V& rY, Vec4V& rZ)
	{
		/*
		const PxVec3 qv(x,y,z);
		return (v*(w*w-0.5f) + (qv.cross(v))*w + qv*(qv.dot(v)))*2;
		*/

		const Vec4V two = V4Splat(FLoad(2.f));
		const Vec4V nhalf = V4Splat(FLoad(-0.5f));
		const Vec4V w2 = V4MulAdd(qw, qw, nhalf);
		const Vec4V ax = V4Mul(vx, w2);
		const Vec4V ay = V4Mul(vy, w2);
		const Vec4V az = V4Mul(vz, w2);

		const Vec4V crX = V4NegMulSub(qz, vy, V4Mul(qy, vz));
		const Vec4V crY = V4NegMulSub(qx, vz, V4Mul(qz, vx));
		const Vec4V crZ = V4NegMulSub(qy, vx, V4Mul(qx, vy));

		const Vec4V tempX = V4MulAdd(crX, qw, ax);
		const Vec4V tempY = V4MulAdd(crY, qw, ay);
		const Vec4V tempZ = V4MulAdd(crZ, qw, az);

		Vec4V dotuv = V4Mul(qx, vx);
		dotuv = V4MulAdd(qy, vy, dotuv);
		dotuv = V4MulAdd(qz, vz, dotuv);

		rX = V4Mul(V4MulAdd(qx, dotuv, tempX), two);
		rY = V4Mul(V4MulAdd(qy, dotuv, tempY), two);
		rZ = V4Mul(V4MulAdd(qz, dotuv, tempZ), two);
	}


struct SolverContactHeaderStepBlock
{
	enum
	{
		eHAS_MAX_IMPULSE = 1 << 0,
		eHAS_TARGET_VELOCITY = 1 << 1
	};

	PxU8	type;					//Note: mType should be first as the solver expects a type in the first byte.
	PxU8	numNormalConstr;
	PxU8	numFrictionConstr;
	PxU8	flag;

	PxU8	flags[4];

	//KS - used for write-back only
	PxU8	numNormalConstrs[4];
	PxU8	numFrictionConstrs[4];

	Vec4V	restitution;
	Vec4V   staticFriction;
	Vec4V	dynamicFriction;
	//Technically, these mass properties could be pulled out into a new structure and shared. For multi-manifold contacts,
	//this would save 64 bytes per-manifold after the cost of the first manifold
	Vec4V	invMass0D0;
	Vec4V	invMass1D1;
	Vec4V	angDom0;
	Vec4V	angDom1;
	//Normal is shared between all contacts in the batch. This will save some memory!
	Vec4V normalX;
	Vec4V normalY;
	Vec4V normalZ;

	Vec4V maxPenBias;

	Sc::ShapeInteraction* shapeInteraction[4];		//192 or 208

	BoolV broken;
	PxU8* frictionBrokenWritebackByte[4];
};

struct SolverContactPointStepBlock
{
	Vec4V raXnI[3];
	Vec4V rbXnI[3];
	Vec4V separation;
	Vec4V velMultiplier;
	Vec4V targetVelocity;
	Vec4V biasCoefficient;
};

//KS - technically, this friction constraint has identical data to the above contact constraint.
//We make them separate structs for clarity
struct SolverContactFrictionStepBlock
{
	Vec4V normal[3];
	Vec4V raXnI[3];
	Vec4V rbXnI[3];
	Vec4V error;
	Vec4V velMultiplier;
	Vec4V targetVel;
	Vec4V biasCoefficient;
};

struct SolverConstraint1DHeaderStep4
{
	PxU8	type;			// enum SolverConstraintType - must be first byte
	PxU8	pad0[3];
	//These counts are the max of the 4 sets of data.
	//When certain pairs have fewer constraints than others, they are padded with 0s so that no work is performed but 
	//calculations are still shared (afterall, they're computationally free because we're doing 4 things at a time in SIMD)
	PxU32	count;
	PxU8	counts[4];
	PxU8	breakable[4];

	Vec4V	linBreakImpulse;
	Vec4V	angBreakImpulse;
	Vec4V	invMass0D0;
	Vec4V	invMass1D1;
	Vec4V	angD0;
	Vec4V	angD1;

	Vec4V	body0WorkOffset[3];
	Vec4V	rAWorld[3];
	Vec4V	rBWorld[3];

	Vec4V	angOrthoAxis0X[3];
	Vec4V	angOrthoAxis0Y[3];
	Vec4V	angOrthoAxis0Z[3];
	Vec4V	angOrthoAxis1X[3];
	Vec4V	angOrthoAxis1Y[3];
	Vec4V	angOrthoAxis1Z[3];
	Vec4V	angOrthoRecipResponse[3];
	Vec4V	angOrthoError[3];	
};


PX_ALIGN_PREFIX(16)
struct SolverConstraint1DStep4
{
public:
	Vec4V		lin0[3];					//!< linear velocity projection (body 0)	
	Vec4V		error;					//!< constraint error term - must be scaled by biasScale. Can be adjusted at run-time

	Vec4V		lin1[3];					//!< linear velocity projection (body 1)
	Vec4V		biasScale;				//!< constraint constant bias scale. Constant

	Vec4V		ang0[3];					//!< angular velocity projection (body 0)
	Vec4V		velMultiplier;			//!< constraint velocity multiplier

	Vec4V		ang1[3];					//!< angular velocity projection (body 1)
	Vec4V		impulseMultiplier;		//!< constraint impulse multiplier

	Vec4V		velTarget;				//!< Scaled target velocity of the constraint drive

	Vec4V		minImpulse;				//!< Lower bound on impulse magnitude	 
	Vec4V		maxImpulse;				//!< Upper bound on impulse magnitude
	Vec4V		appliedForce;			//!< applied force to correct velocity+bias

	Vec4V		maxBias;
	Vec4V		angularErrorScale;			//Constant
	PxU32		flags[4];
} PX_ALIGN_SUFFIX(16);




static void setupFinalizeSolverConstraints4Step(PxTGSSolverContactDesc* PX_RESTRICT descs, CorrelationBuffer& c,
	PxU8* PX_RESTRICT workspace, const PxReal invDtF32, const PxReal invTotalDtF32, PxReal bounceThresholdF32, const PxReal solverOffsetSlopF32,
	const Ps::aos::Vec4VArg invMassScale0, const Ps::aos::Vec4VArg invInertiaScale0,
	const Ps::aos::Vec4VArg invMassScale1, const Ps::aos::Vec4VArg invInertiaScale1)
{

	//OK, we have a workspace of pre-allocated space to store all 4 descs in. We now need to create the constraints in it

	//const Vec4V ccdMaxSeparation = Ps::aos::V4LoadXYZW(descs[0].maxCCDSeparation, descs[1].maxCCDSeparation, descs[2].maxCCDSeparation, descs[3].maxCCDSeparation);
	const Vec4V solverOffsetSlop = V4Load(solverOffsetSlopF32);

	const Vec4V zero = V4Zero();
	const BoolV bFalse = BFFFF();
	const FloatV fZero = FZero();

	PxU8 flags[4] = { PxU8(descs[0].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
		PxU8(descs[1].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
		PxU8(descs[2].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
		PxU8(descs[3].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0) };

	bool hasMaxImpulse = descs[0].hasMaxImpulse || descs[1].hasMaxImpulse || descs[2].hasMaxImpulse || descs[3].hasMaxImpulse;

	//The block is dynamic if **any** of the constraints have a non-static body B. This allows us to batch static and non-static constraints but we only get a memory/perf
	//saving if all 4 are static. This simplifies the constraint partitioning such that it only needs to care about separating contacts and 1D constraints (which it already does)
	bool isDynamic = false;
	bool hasKinematic = false;
	
	PxReal kinematicScale0F32[4];
	PxReal kinematicScale1F32[4];

	for (PxU32 a = 0; a < 4; ++a)
	{
		isDynamic = isDynamic || (descs[a].bodyState1 == PxSolverContactDesc::eDYNAMIC_BODY);
		hasKinematic = hasKinematic || descs[a].bodyState1 == PxSolverContactDesc::eKINEMATIC_BODY;
		kinematicScale0F32[a] = descs[a].body0->isKinematic ? 1.f : 0.f;
		kinematicScale1F32[a] = descs[a].body1->isKinematic ? 1.f : 0.f;
	}

	/*BoolV kinematic0 = BLoad(isKinematic0);
	BoolV kinematic1 = BLoad(isKinematic1);*/

	const Vec4V kinematicScale0 = V4LoadU(kinematicScale0F32);
	const Vec4V kinematicScale1 = V4LoadU(kinematicScale1F32);

	const PxU32 constraintSize = sizeof(SolverContactPointStepBlock);
	const PxU32 frictionSize = sizeof(SolverContactFrictionStepBlock);

	PxU8* PX_RESTRICT ptr = workspace;

	const Vec4V dom0 = invMassScale0;
	const Vec4V dom1 = invMassScale1;
	const Vec4V angDom0 = invInertiaScale0;
	const Vec4V angDom1 = invInertiaScale1;

	const Vec4V maxPenBias = V4Max(V4LoadXYZW(descs[0].bodyData0->penBiasClamp, descs[1].bodyData0->penBiasClamp,
		descs[2].bodyData0->penBiasClamp, descs[3].bodyData0->penBiasClamp),
		V4LoadXYZW(descs[0].bodyData1->penBiasClamp, descs[1].bodyData1->penBiasClamp,
			descs[2].bodyData1->penBiasClamp, descs[3].bodyData1->penBiasClamp));

	const Vec4V restDistance = V4LoadXYZW(descs[0].restDistance, descs[1].restDistance, descs[2].restDistance,
		descs[3].restDistance);


	//load up velocities
	Vec4V linVel00 = V4LoadA(&descs[0].bodyData0->originalLinearVelocity.x);
	Vec4V linVel10 = V4LoadA(&descs[1].bodyData0->originalLinearVelocity.x);
	Vec4V linVel20 = V4LoadA(&descs[2].bodyData0->originalLinearVelocity.x);
	Vec4V linVel30 = V4LoadA(&descs[3].bodyData0->originalLinearVelocity.x);

	Vec4V linVel01 = V4LoadA(&descs[0].bodyData1->originalLinearVelocity.x);
	Vec4V linVel11 = V4LoadA(&descs[1].bodyData1->originalLinearVelocity.x);
	Vec4V linVel21 = V4LoadA(&descs[2].bodyData1->originalLinearVelocity.x);
	Vec4V linVel31 = V4LoadA(&descs[3].bodyData1->originalLinearVelocity.x);

	Vec4V angVel00 = V4LoadA(&descs[0].bodyData0->originalAngularVelocity.x);
	Vec4V angVel10 = V4LoadA(&descs[1].bodyData0->originalAngularVelocity.x);
	Vec4V angVel20 = V4LoadA(&descs[2].bodyData0->originalAngularVelocity.x);
	Vec4V angVel30 = V4LoadA(&descs[3].bodyData0->originalAngularVelocity.x);

	Vec4V angVel01 = V4LoadA(&descs[0].bodyData1->originalAngularVelocity.x);
	Vec4V angVel11 = V4LoadA(&descs[1].bodyData1->originalAngularVelocity.x);
	Vec4V angVel21 = V4LoadA(&descs[2].bodyData1->originalAngularVelocity.x);
	Vec4V angVel31 = V4LoadA(&descs[3].bodyData1->originalAngularVelocity.x);

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

	const Vec4V invMass0 = V4LoadXYZW(descs[0].bodyData0->invMass, descs[1].bodyData0->invMass, descs[2].bodyData0->invMass, descs[3].bodyData0->invMass);
	const Vec4V invMass1 = V4LoadXYZW(descs[0].bodyData1->invMass, descs[1].bodyData1->invMass, descs[2].bodyData1->invMass, descs[3].bodyData1->invMass);

	const Vec4V invMass0D0 = V4Mul(dom0, invMass0);
	const Vec4V invMass1D1 = V4Mul(dom1, invMass1);

	Vec4V invInertia00X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[0].body0TxI->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia00Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[0].body0TxI->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia00Z = Vec4V_From_Vec3V(V3LoadU(descs[0].body0TxI->sqrtInvInertia.column2));

	Vec4V invInertia10X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[1].body0TxI->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia10Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[1].body0TxI->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia10Z = Vec4V_From_Vec3V(V3LoadU(descs[1].body0TxI->sqrtInvInertia.column2));

	Vec4V invInertia20X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[2].body0TxI->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia20Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[2].body0TxI->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia20Z = Vec4V_From_Vec3V(V3LoadU(descs[2].body0TxI->sqrtInvInertia.column2));

	Vec4V invInertia30X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[3].body0TxI->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia30Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[3].body0TxI->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia30Z = Vec4V_From_Vec3V(V3LoadU(descs[3].body0TxI->sqrtInvInertia.column2));

	Vec4V invInertia01X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[0].body1TxI->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia01Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[0].body1TxI->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia01Z = Vec4V_From_Vec3V(V3LoadU(descs[0].body1TxI->sqrtInvInertia.column2));

	Vec4V invInertia11X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[1].body1TxI->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia11Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[1].body1TxI->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia11Z = Vec4V_From_Vec3V(V3LoadU(descs[1].body1TxI->sqrtInvInertia.column2));

	Vec4V invInertia21X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[2].body1TxI->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia21Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[2].body1TxI->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia21Z = Vec4V_From_Vec3V(V3LoadU(descs[2].body1TxI->sqrtInvInertia.column2));

	Vec4V invInertia31X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[3].body1TxI->sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia31Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(descs[3].body1TxI->sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia31Z = Vec4V_From_Vec3V(V3LoadU(descs[3].body1TxI->sqrtInvInertia.column2));

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
	const FloatV invTotalDt = FLoad(invTotalDtF32);
	const FloatV p8 = FLoad(0.8f);
	const Vec4V p84 = V4Splat(p8);
	const Vec4V bounceThreshold = V4Splat(FLoad(bounceThresholdF32));

	const Vec4V invDtp8 = V4Splat(FMul(invDt, p8));

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

	const Vec4V p1 = V4Splat(FLoad(0.0001f));
	const Vec4V orthoThreshold = V4Splat(FLoad(0.70710678f));


	PxU32 contact0 = 0, contact1 = 0, contact2 = 0, contact3 = 0;
	PxU32 patch0 = 0, patch1 = 0, patch2 = 0, patch3 = 0;

	PxU8 flag = 0;
	if (hasMaxImpulse)
		flag |= SolverContactHeader4::eHAS_MAX_IMPULSE;

	for (PxU32 i = 0; i<maxPatches; i++)
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

		SolverContactHeaderStepBlock* PX_RESTRICT header = reinterpret_cast<SolverContactHeaderStepBlock*>(ptr);
		ptr += sizeof(SolverContactHeaderStepBlock);


		header->flags[0] = flags[0];
		header->flags[1] = flags[1];
		header->flags[2] = flags[2];
		header->flags[3] = flags[3];

		header->flag = flag;

		PxU32 totalContacts = PxMax(clampedContacts0, PxMax(clampedContacts1, PxMax(clampedContacts2, clampedContacts3)));

		Vec4V* PX_RESTRICT appliedNormalForces = reinterpret_cast<Vec4V*>(ptr);
		ptr += sizeof(Vec4V)*totalContacts;

		PxMemZero(appliedNormalForces, sizeof(Vec4V) * totalContacts);

		header->numNormalConstr = Ps::to8(totalContacts);
		header->numNormalConstrs[0] = Ps::to8(clampedContacts0);
		header->numNormalConstrs[1] = Ps::to8(clampedContacts1);
		header->numNormalConstrs[2] = Ps::to8(clampedContacts2);
		header->numNormalConstrs[3] = Ps::to8(clampedContacts3);
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

		header->maxPenBias = maxPenBias;

		const Vec4V norVel0 = V4MulAdd(normalZ, linVelT20, V4MulAdd(normalY, linVelT10, V4Mul(normalX, linVelT00)));
		const Vec4V norVel1 = V4MulAdd(normalZ, linVelT21, V4MulAdd(normalY, linVelT11, V4Mul(normalX, linVelT01)));
		const Vec4V relNorVel = V4Sub(norVel0, norVel1);

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

		if (!hasFinished0)
			iter0.nextContact(patch0, contact0);
		if (!hasFinished1)
			iter1.nextContact(patch1, contact1);
		if (!hasFinished2)
			iter2.nextContact(patch2, contact2);
		if (!hasFinished3)
			iter3.nextContact(patch3, contact3);

		PxU8* p = ptr;

		PxU32 contactCount = 0;
		PxU32 newFinished =
			(PxU32(hasFinished0 || !iter0.hasNextContact())) |
			((PxU32(hasFinished1 || !iter1.hasNextContact())) << 1) |
			((PxU32(hasFinished2 || !iter2.hasNextContact())) << 2) |
			((PxU32(hasFinished3 || !iter3.hasNextContact())) << 3);

		while (finished != 0xf)
		{
			finished = newFinished;
			++contactCount;
			Ps::prefetchLine(p, 384);
			Ps::prefetchLine(p, 512);
			Ps::prefetchLine(p, 640);

			SolverContactPointStepBlock* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPointStepBlock*>(p);
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
				const Vec4V relAngVel0 = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4Mul(raXnX, angVelT00)));

				Vec4V unitResponse = V4MulAdd(invMass0D0, angDom0, dotDelAngVel0);
				Vec4V vrel0 = V4Add(norVel0, relAngVel0);
				Vec4V vrel1 = norVel1;

				Vec4V delAngVel1X = zero;
				Vec4V delAngVel1Y = zero;
				Vec4V delAngVel1Z = zero;

				//The dynamic-only parts - need to if-statement these up. A branch here shouldn't cost us too much
				if (isDynamic)
				{
					Vec4V rbXnX = V4NegMulSub(rbZ, normalY, V4Mul(rbY, normalZ));
					Vec4V rbXnY = V4NegMulSub(rbX, normalZ, V4Mul(rbZ, normalX));
					Vec4V rbXnZ = V4NegMulSub(rbY, normalX, V4Mul(rbX, normalY));

					rbXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnX)), zero, rbXnX);
					rbXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnY)), zero, rbXnY);
					rbXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnZ)), zero, rbXnZ);

					delAngVel1X = V4Mul(invInertia1X0, rbXnX);
					delAngVel1Y = V4Mul(invInertia1X1, rbXnX);
					delAngVel1Z = V4Mul(invInertia1X2, rbXnX);

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
					const Vec4V relAngVel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4Mul(rbXnX, angVelT01)));

					const Vec4V resp1 = V4MulAdd(dotDelAngVel1, angDom1, invMass1D1);

					unitResponse = V4Add(unitResponse, resp1);

					vrel1 = V4Add(vrel1, relAngVel1);

					//These are for dynamic-only contacts.
					

				}
				else if (hasKinematic)
				{
					const Vec4V rbXnX = V4NegMulSub(rbZ, normalY, V4Mul(rbY, normalZ));
					const Vec4V rbXnY = V4NegMulSub(rbX, normalZ, V4Mul(rbZ, normalX));
					const Vec4V rbXnZ = V4NegMulSub(rbY, normalX, V4Mul(rbX, normalY));

					const Vec4V relAngVel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4Mul(rbXnX, angVelT01)));

					vrel1 = V4Add(vrel1, relAngVel1);
				}

				Vec4V vrel = V4Sub(vrel0, vrel1);

				solverContact->rbXnI[0] = delAngVel1X;
				solverContact->rbXnI[1] = delAngVel1Y;
				solverContact->rbXnI[2] = delAngVel1Z;
				
				const Vec4V velMultiplier = V4Sel(V4IsGrtr(unitResponse, zero), V4Recip(unitResponse), zero);

				const Vec4V penetration = V4Sub(separation, restDistance);
				//const Vec4V penInvDtPt8 = V4Max(maxPenBias, V4Scale(penetration, invDtp8));
				//Vec4V scaledBias = invDtPt8;

				const Vec4V penetrationInvDt = V4Scale(penetration, invTotalDt);

				const BoolV isGreater2 = BAnd(BAnd(V4IsGrtr(zero, restitution), V4IsGrtr(bounceThreshold, vrel)),
					V4IsGrtr(V4Neg(vrel), penetrationInvDt));


				const Vec4V scaledBias = V4Neg(invDtp8);

				Vec4V targetVelocity = V4NegMulSub(vrel0, kinematicScale0, V4MulAdd(vrel1, kinematicScale1, V4Sel(isGreater2, V4Mul(vrel, restitution), zero)));



				//Vec4V biasedErr = V4Sel(isGreater2, targetVelocity, scaledBias);
				//Vec4V biasedErr = V4Add(targetVelocity, scaledBias);

				//biasedErr = V4NegMulSub(V4Sub(vrel, cTargetNorVel), velMultiplier, biasedErr);

				//These values are present for static and dynamic contacts			
				solverContact->raXnI[0] = delAngVel0X;
				solverContact->raXnI[1] = delAngVel0Y;
				solverContact->raXnI[2] = delAngVel0Z;
				solverContact->velMultiplier = velMultiplier;
				solverContact->targetVelocity = V4Add(cTargetNorVel, targetVelocity);
				solverContact->separation = penetration;
				solverContact->biasCoefficient = scaledBias;

				if (hasMaxImpulse)
				{
					maxImpulse[contactCount - 1] = V4Merge(FLoad(con0.maxImpulse), FLoad(con1.maxImpulse), FLoad(con2.maxImpulse),
						FLoad(con3.maxImpulse));
				}
			}
			if (!(finished & 0x1))
			{
				iter0.nextContact(patch0, contact0);
				newFinished |= PxU32(!iter0.hasNextContact());
			}

			if (!(finished & 0x2))
			{
				iter1.nextContact(patch1, contact1);
				newFinished |= (PxU32(!iter1.hasNextContact()) << 1);
			}

			if (!(finished & 0x4))
			{
				iter2.nextContact(patch2, contact2);
				newFinished |= (PxU32(!iter2.hasNextContact()) << 2);
			}

			if (!(finished & 0x8))
			{
				iter3.nextContact(patch3, contact3);
				newFinished |= (PxU32(!iter3.hasNextContact()) << 3);
			}
		}
		ptr = p;
		if (hasMaxImpulse)
		{
			ptr += sizeof(Vec4V) * totalContacts;
		}

		//OK...friction time :-)

		Vec4V maxImpulseScale = V4One();
		{

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

			PX_ALIGN(16, PxReal staticFriction[4]);
			PX_ALIGN(16, PxReal dynamicFriction[4]);

			//for (PxU32 f = 0; f < 4; ++f)
			{
				PxReal coeff0 = (contactBase0->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && clampedAnchorCount0 == 2) ? 0.5f : 1.f;
				PxReal coeff1 = (contactBase1->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && clampedAnchorCount1 == 2) ? 0.5f : 1.f;
				PxReal coeff2 = (contactBase2->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && clampedAnchorCount2 == 2) ? 0.5f : 1.f;
				PxReal coeff3 = (contactBase3->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && clampedAnchorCount3 == 2) ? 0.5f : 1.f;

				staticFriction[0] = contactBase0->staticFriction * coeff0;
				dynamicFriction[0] = contactBase0->dynamicFriction * coeff0;
				staticFriction[1] = contactBase1->staticFriction * coeff1;
				dynamicFriction[1] = contactBase1->dynamicFriction * coeff1;
				staticFriction[2] = contactBase2->staticFriction * coeff2;
				dynamicFriction[2] = contactBase2->dynamicFriction * coeff2;
				staticFriction[3] = contactBase3->staticFriction * coeff3;
				dynamicFriction[3] = contactBase3->dynamicFriction * coeff3;
			}

			PX_ASSERT(totalContacts == contactCount);
			
			header->numFrictionConstr = Ps::to8(maxAnchorCount * 2);
			header->numFrictionConstrs[0] = Ps::to8(clampedAnchorCount0 * 2);
			header->numFrictionConstrs[1] = Ps::to8(clampedAnchorCount1 * 2);
			header->numFrictionConstrs[2] = Ps::to8(clampedAnchorCount2 * 2);
			header->numFrictionConstrs[3] = Ps::to8(clampedAnchorCount3 * 2);

			//KS - TODO - extend this if needed
			header->type = Ps::to8(DY_SC_TYPE_BLOCK_RB_CONTACT);

			if (maxAnchorCount)
			{
				const BoolV cond = V4IsGrtr(orthoThreshold, V4Abs(normalX));

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


				PxU8* PX_RESTRICT writeback0 = descs[0].frictionPtr + frictionPatchWritebackAddrIndex0 * sizeof(FrictionPatch);
				PxU8* PX_RESTRICT writeback1 = descs[1].frictionPtr + frictionPatchWritebackAddrIndex1 * sizeof(FrictionPatch);
				PxU8* PX_RESTRICT writeback2 = descs[2].frictionPtr + frictionPatchWritebackAddrIndex2 * sizeof(FrictionPatch);
				PxU8* PX_RESTRICT writeback3 = descs[3].frictionPtr + frictionPatchWritebackAddrIndex3 * sizeof(FrictionPatch);

				PxU32 index0 = 0, index1 = 0, index2 = 0, index3 = 0;

				header->broken = bFalse;
				header->frictionBrokenWritebackByte[0] = writeback0;
				header->frictionBrokenWritebackByte[1] = writeback1;
				header->frictionBrokenWritebackByte[2] = writeback2;
				header->frictionBrokenWritebackByte[3] = writeback3;


				/*header->frictionNormal[0][0] = t0X;
				header->frictionNormal[0][1] = t0Y;
				header->frictionNormal[0][2] = t0Z;

				header->frictionNormal[1][0] = t1X;
				header->frictionNormal[1][1] = t1Y;
				header->frictionNormal[1][2] = t1Z;*/

				Vec4V* PX_RESTRICT appliedForces = reinterpret_cast<Vec4V*>(ptr);
				ptr += sizeof(Vec4V)*header->numFrictionConstr;

				PxMemZero(appliedForces, sizeof(Vec4V) * header->numFrictionConstr);

				for (PxU32 j = 0; j < maxAnchorCount; j++)
				{
					Ps::prefetchLine(ptr, 384);
					Ps::prefetchLine(ptr, 512);
					Ps::prefetchLine(ptr, 640);
					SolverContactFrictionStepBlock* PX_RESTRICT f0 = reinterpret_cast<SolverContactFrictionStepBlock*>(ptr);
					ptr += frictionSize;
					SolverContactFrictionStepBlock* PX_RESTRICT f1 = reinterpret_cast<SolverContactFrictionStepBlock*>(ptr);
					ptr += frictionSize;

					index0 = j < clampedAnchorCount0 ? j : index0;
					index1 = j < clampedAnchorCount1 ? j : index1;
					index2 = j < clampedAnchorCount2 ? j : index2;
					index3 = j < clampedAnchorCount3 ? j : index3;

					if (j >= clampedAnchorCount0)
						maxImpulseScale = V4SetX(maxImpulseScale, fZero);
					if (j >= clampedAnchorCount1)
						maxImpulseScale = V4SetY(maxImpulseScale, fZero);
					if (j >= clampedAnchorCount2)
						maxImpulseScale = V4SetZ(maxImpulseScale, fZero);
					if (j >= clampedAnchorCount3)
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
						Vec4V vrel0 = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4MulAdd(raXnX, angVelT00, tVel0)));

						Vec4V delAngVel1X = zero;
						Vec4V delAngVel1Y = zero;
						Vec4V delAngVel1Z = zero;

						Vec4V vrel1 = zero;

						if (isDynamic)
						{
							Vec4V rbXnX = V4NegMulSub(rbZ, t0Y, V4Mul(rbY, t0Z));
							Vec4V rbXnY = V4NegMulSub(rbX, t0Z, V4Mul(rbZ, t0X));
							Vec4V rbXnZ = V4NegMulSub(rbY, t0X, V4Mul(rbX, t0Y));

							rbXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnX)), zero, rbXnX);
							rbXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnY)), zero, rbXnY);
							rbXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnZ)), zero, rbXnZ);

							delAngVel1X = V4Mul(invInertia1X0, rbXnX);
							delAngVel1Y = V4Mul(invInertia1X1, rbXnX);
							delAngVel1Z = V4Mul(invInertia1X2, rbXnX);

							delAngVel1X = V4MulAdd(invInertia1Y0, rbXnY, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Y1, rbXnY, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Y2, rbXnY, delAngVel1Z);

							delAngVel1X = V4MulAdd(invInertia1Z0, rbXnZ, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Z1, rbXnZ, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Z2, rbXnZ, delAngVel1Z);

							const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1Z, delAngVel1Z, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1X, delAngVel1X)));

							const Vec4V resp1 = V4MulAdd(dotDelAngVel1, angDom1, invMass1D1);

							resp = V4Add(resp, resp1);

							const Vec4V tVel1 = V4MulAdd(t0Z, linVelT21, V4MulAdd(t0Y, linVelT11, V4Mul(t0X, linVelT01)));
							vrel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));
						}
						else if (hasKinematic)
						{
							const Vec4V rbXnX = V4NegMulSub(rbZ, t0Y, V4Mul(rbY, t0Z));
							const Vec4V rbXnY = V4NegMulSub(rbX, t0Z, V4Mul(rbZ, t0X));
							const Vec4V rbXnZ = V4NegMulSub(rbY, t0X, V4Mul(rbX, t0Y));

							const Vec4V tVel1 = V4MulAdd(t0Z, linVelT21, V4MulAdd(t0Y, linVelT11, V4Mul(t0X, linVelT01)));
							vrel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));
						}

						f0->rbXnI[0] = delAngVel1X;
						f0->rbXnI[1] = delAngVel1Y;
						f0->rbXnI[2] = delAngVel1Z;

						const Vec4V velMultiplier = V4Mul(maxImpulseScale, V4Sel(V4IsGrtr(resp, zero), V4Div(p84, resp), zero));

						Vec4V error = V4MulAdd(t0Z, errorZ, V4MulAdd(t0Y, errorY, V4Mul(t0X, errorX)));

						Vec4V targetVel = V4NegMulSub(vrel0, kinematicScale0, V4MulAdd(vrel1, kinematicScale1, V4MulAdd(t0Z, targetVelZ, V4MulAdd(t0Y, targetVelY, V4Mul(t0X, targetVelX)))));
						
						f0->normal[0] = t0X;
						f0->normal[1] = t0Y;
						f0->normal[2] = t0Z;
						f0->raXnI[0] = delAngVel0X;
						f0->raXnI[1] = delAngVel0Y;
						f0->raXnI[2] = delAngVel0Z;
						f0->error = error;
						f0->velMultiplier = velMultiplier;
						f0->biasCoefficient = V4Splat(invDt);
						f0->targetVel = targetVel;
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
						Vec4V vrel0 = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4MulAdd(raXnX, angVelT00, tVel0)));

						Vec4V delAngVel1X = zero;
						Vec4V delAngVel1Y = zero;
						Vec4V delAngVel1Z = zero;

						Vec4V vrel1 = zero;

						if (isDynamic)
						{
							
							Vec4V rbXnX = V4NegMulSub(rbZ, t1Y, V4Mul(rbY, t1Z));
							Vec4V rbXnY = V4NegMulSub(rbX, t1Z, V4Mul(rbZ, t1X));
							Vec4V rbXnZ = V4NegMulSub(rbY, t1X, V4Mul(rbX, t1Y));

							rbXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnX)), zero, rbXnX);
							rbXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnY)), zero, rbXnY);
							rbXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnZ)), zero, rbXnZ);

							delAngVel1X = V4Mul(invInertia1X0, rbXnX);
							delAngVel1Y = V4Mul(invInertia1X1, rbXnX);
							delAngVel1Z = V4Mul(invInertia1X2, rbXnX);

							delAngVel1X = V4MulAdd(invInertia1Y0, rbXnY, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Y1, rbXnY, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Y2, rbXnY, delAngVel1Z);

							delAngVel1X = V4MulAdd(invInertia1Z0, rbXnZ, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Z1, rbXnZ, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Z2, rbXnZ, delAngVel1Z);

							const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1Z, delAngVel1Z, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1X, delAngVel1X)));

							const Vec4V resp1 = V4MulAdd(dotDelAngVel1, angDom1, invMass1D1);

							resp = V4Add(resp, resp1);

							

							const Vec4V tVel1 = V4MulAdd(t1Z, linVelT21, V4MulAdd(t1Y, linVelT11, V4Mul(t1X, linVelT01)));
							vrel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));
						}
						else if (hasKinematic)
						{
							const Vec4V rbXnX = V4NegMulSub(rbZ, t1Y, V4Mul(rbY, t1Z));
							const Vec4V rbXnY = V4NegMulSub(rbX, t1Z, V4Mul(rbZ, t1X));
							const Vec4V rbXnZ = V4NegMulSub(rbY, t1X, V4Mul(rbX, t1Y));

							const Vec4V tVel1 = V4MulAdd(t1Z, linVelT21, V4MulAdd(t1Y, linVelT11, V4Mul(t1X, linVelT01)));
							vrel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));
						}

						f1->rbXnI[0] = delAngVel1X;
						f1->rbXnI[1] = delAngVel1Y;
						f1->rbXnI[2] = delAngVel1Z;


						const Vec4V velMultiplier = V4Mul(maxImpulseScale, V4Sel(V4IsGrtr(resp, zero), V4Div(p84, resp), zero));

						Vec4V error = V4MulAdd(t1Z, errorZ, V4MulAdd(t1Y, errorY, V4Mul(t1X, errorX)));

						Vec4V targetVel = V4NegMulSub(vrel0, kinematicScale0, V4MulAdd(vrel1, kinematicScale1, V4MulAdd(t1Z, targetVelZ, V4MulAdd(t1Y, targetVelY, V4Mul(t1X, targetVelX)))));
						
						f1->normal[0] = t1X;
						f1->normal[1] = t1Y;
						f1->normal[2] = t1Z;
						f1->raXnI[0] = delAngVel0X;
						f1->raXnI[1] = delAngVel0Y;
						f1->raXnI[2] = delAngVel0Z;
						f1->error = error;
						f1->velMultiplier = velMultiplier;
						f1->targetVel = targetVel;
						f1->biasCoefficient = V4Splat(invDt);
					}
				}

				header->dynamicFriction = V4LoadA(dynamicFriction);
				header->staticFriction = V4LoadA(staticFriction);

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

	for (PxU32 i = frictionPatchStartIndex; i < frictionPatchEndIndex; i++)
	{
		//Friction patches.
		if (c.correlationListHeads[i] != CorrelationBuffer::LIST_END)
			numFrictionPatches++;
	}
	PxU32 frictionPatchByteSize = numFrictionPatches * sizeof(FrictionPatch);

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
	if (frictionPatchByteSize > 0)
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

	//Return true if neither of the two block reservations failed.
	return (0 == frictionPatchByteSize || frictionPatches);
}

//The persistent friction patch correlation/allocation will already have happenned as this is per-pair.
//This function just computes the size of the combined solve data.
void computeBlockStreamByteSizes4(PxTGSSolverContactDesc* descs,
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

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxU32 axisConstraintCount = 0;
		hasMaxImpulse = hasMaxImpulse || descs[a].hasMaxImpulse;
		for (PxU32 i = 0; i < descs[a].numFrictionPatches; i++)
		{
			PxU32 ind = i + descs[a].startFrictionPatchIndex;

			const FrictionPatch& frictionPatch = c.frictionPatches[ind];

			const bool haveFriction = (frictionPatch.materialFlags & PxMaterialFlag::eDISABLE_FRICTION) == 0
				&& frictionPatch.anchorCount != 0;
			//Solver constraint data.
			if (c.frictionPatchContactCounts[ind] != 0)
			{
				maxContactCount[i] = PxMax(c.frictionPatchContactCounts[ind], maxContactCount[i]);
				axisConstraintCount += c.frictionPatchContactCounts[ind];

				if (haveFriction)
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

	for (PxU32 a = 0; a < maxPatches; ++a)
	{
		if (maxFrictionCount[a] > 0)
			maxFrictionPatches++;
	}


	PxU32 totalContacts = 0, totalFriction = 0;
	for (PxU32 a = 0; a < maxPatches; ++a)
	{
		totalContacts += maxContactCount[a];
		totalFriction += maxFrictionCount[a];
	}

	//OK, we have a given number of friction patches, contact points and friction constraints so we can calculate how much memory we need

	//Body 2 is considered static if it is either *not dynamic* or *kinematic*

	/*bool hasDynamicBody = false;
	for (PxU32 a = 0; a < 4; ++a)
	{
		hasDynamicBody = hasDynamicBody || ((descs[a].bodyState1 == PxSolverContactDesc::eDYNAMIC_BODY));
	}


	const bool isStatic = !hasDynamicBody;*/

	const PxU32 headerSize = sizeof(SolverContactHeaderStepBlock) * maxPatches;
	//PxU32 constraintSize = isStatic ? (sizeof(SolverContactBatchPointBase4) * totalContacts) + (sizeof(SolverContactFrictionBase4) * totalFriction) :
	//	(sizeof(SolverContactBatchPointDynamic4) * totalContacts) + (sizeof(SolverContactFrictionDynamic4) * totalFriction);

	PxU32 constraintSize = (sizeof(SolverContactPointStepBlock) * totalContacts) + (sizeof(SolverContactFrictionStepBlock) * totalFriction);

	//Space for the appliedForce buffer
	constraintSize += sizeof(Vec4V)*(totalContacts + totalFriction);

	//If we have max impulse, reserve a buffer for it
	if (hasMaxImpulse)
		constraintSize += sizeof(Ps::aos::Vec4V) * totalContacts;

	_solverConstraintByteSize = ((constraintSize + headerSize + 0x0f) & ~0x0f);
	PX_ASSERT(0 == (_solverConstraintByteSize & 0x0f));
}

static SolverConstraintPrepState::Enum reserveBlockStreams4(PxTGSSolverContactDesc* descs, Dy::CorrelationBuffer& c,
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
	if (constraintBlockByteSize > 0)
	{
		if ((constraintBlockByteSize + 16u) > 16384)
			return SolverConstraintPrepState::eUNBATCHABLE;

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
	}

	//Patch up the individual ptrs to the buffer returned by the constraint block reservation (assuming the reservation didn't fail).
	if (0 == constraintBlockByteSize || constraintBlock)
	{
		if (solverConstraintByteSize)
		{
			solverConstraint = constraintBlock;
			PX_ASSERT(0 == (uintptr_t(solverConstraint) & 0x0f));
		}
	}

	return ((0 == constraintBlockByteSize || constraintBlock)) ? SolverConstraintPrepState::eSUCCESS : SolverConstraintPrepState::eOUT_OF_MEMORY;
}



SolverConstraintPrepState::Enum createFinalizeSolverContacts4Step(
	Dy::CorrelationBuffer& c,
	PxTGSSolverContactDesc* blockDescs,
	const PxReal invDtF32,
	const PxReal invTotalDtF32,
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
		PxTGSSolverContactDesc& blockDesc = blockDescs[a];

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
		PxTGSSolverContactDesc& blockDesc = blockDescs[a];

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

			PxTGSSolverContactDesc& blockDesc = blockDescs[a];
			PxTGSSolverConstraintDesc& desc = *blockDesc.desc;
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

		setupFinalizeSolverConstraints4Step(blockDescs, c, solverConstraint, invDtF32, invTotalDtF32, bounceThresholdF32, solverOffsetSlop,
			iMassScale0, iInertiaScale0, iMassScale1, iInertiaScale1);

		PX_ASSERT((*solverConstraint == DY_SC_TYPE_BLOCK_RB_CONTACT) || (*solverConstraint == DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT));

		*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize)) = 0;
	}
	return SolverConstraintPrepState::eSUCCESS;
}


SolverConstraintPrepState::Enum createFinalizeSolverContacts4Step(
	PxsContactManagerOutput** cmOutputs,
	ThreadContext& threadContext,
	PxTGSSolverContactDesc* blockDescs,
	const PxReal invDtF32,
	const PxReal invTotalDtF32,
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

	//PX_ASSERT(cmOutputs[0]->nbContacts && cmOutputs[1]->nbContacts && cmOutputs[2]->nbContacts && cmOutputs[3]->nbContacts);


	Gu::ContactBuffer& buffer = threadContext.mContactBuffer;

	buffer.count = 0;

	//PxTransform idt = PxTransform(PxIdentity);

	CorrelationBuffer& c = threadContext.mCorrelationBuffer;

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxTGSSolverContactDesc& blockDesc = blockDescs[a];
		PxTGSSolverConstraintDesc& desc = *blockDesc.desc;

		//blockDesc.startContactIndex = buffer.count;
		blockDesc.contacts = buffer.contacts + buffer.count;

		Ps::prefetchLine(desc.bodyA);
		Ps::prefetchLine(desc.bodyB);


		//Unbatchable if we have (a) too many contacts or (b) torsional friction enabled - it just seems easier to handle this on an individual contact basis because it is expected to 
		//be used relatively rarely
		if ((buffer.count + cmOutputs[a]->nbContacts) > 64 || (blockDesc.torsionalPatchRadius != 0.f || blockDesc.minTorsionalPatchRadius != 0.f) )
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

		const PxReal defaultMaxImpulse = PxMin(blockDesc.bodyData0->maxContactImpulse, blockDesc.bodyData1->maxContactImpulse);

		PxU32 contactCount = extractContacts(buffer, *cmOutputs[a], hasMaxImpulse, hasTargetVelocity, invMassScale0, invMassScale1,
			invInertiaScale0, invInertiaScale1, defaultMaxImpulse);

		if (contactCount == 0 || hasTargetVelocity)
			return SolverConstraintPrepState::eUNBATCHABLE;

		blockDesc.numContacts = contactCount;
		blockDesc.hasMaxImpulse = hasMaxImpulse;
		blockDesc.disableStrongFriction = blockDesc.disableStrongFriction || hasTargetVelocity;

		blockDesc.mInvMassScales.linear0 *= invMassScale0;
		blockDesc.mInvMassScales.linear1 *= invMassScale1;
		blockDesc.mInvMassScales.angular0 *= blockDesc.body0->isKinematic ? 0.f : invInertiaScale0;
		blockDesc.mInvMassScales.angular1 *= blockDesc.body1->isKinematic ? 0.f : invInertiaScale1;
	}

	return createFinalizeSolverContacts4Step(c, blockDescs,
		invDtF32, invTotalDtF32, bounceThresholdF32, frictionOffsetThreshold,
		correlationDistance, solverOffsetSlop, constraintAllocator);
}

PX_FORCE_INLINE PxU32 getConstraintLength(const PxTGSSolverConstraintDesc& desc)
{
	return PxU32(desc.constraintLengthOver16 << 4);
}

PX_FORCE_INLINE PxU32 getWritebackLength(const PxTGSSolverConstraintDesc& desc)
{
	return PxU32(desc.writeBackLengthOver4 << 2);
}



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
	PxReal recipTotalDt);


namespace
{
	void setConstants(PxReal& error, PxReal& biasScale, PxReal& targetVel, PxReal& maxBias, PxReal& velMultiplier, PxReal& impulseMultiplier,
		PxReal& rcpResponse, const Px1DConstraint& c, PxReal unitResponse, PxReal minRowResponse, PxReal dt, PxReal totalDt,
		PxReal recipdt, PxReal recipTotalDt, const bool finished,
		const PxReal lengthScale, const PxReal nv, const PxReal nv0, const PxReal nv1, const bool isKinematic0, const bool isKinematic1)
	{
		PX_UNUSED(dt);
		if (finished)
		{
			error = 0.f;
			biasScale = 0.f;
			maxBias = 0.f;
			velMultiplier = 0.f;
			impulseMultiplier = 0.f;
			rcpResponse = 0.f;
			targetVel = 0.f;
			return;
		}

		PxReal erp = 1.f;
		PxReal linearErp = 1.f;

		//PxReal biasClamp = c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT ? 50.f : 200.f*lengthScale;
		PxReal biasClamp = c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT ? 100.f : 1000.f*lengthScale;

		setSolverConstantsStep(error, biasScale, targetVel, maxBias, velMultiplier, impulseMultiplier, rcpResponse, c, nv, unitResponse,
			minRowResponse, c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT ? erp : linearErp, dt, totalDt, biasClamp, recipdt, recipTotalDt);

		if(isKinematic0)
			targetVel -= nv0;
		if(isKinematic1)
			targetVel += nv1;
	}

	void setOrthoData(const PxReal& ang0X, const PxReal& ang0Y, const PxReal& ang0Z, const PxReal& ang1X, const PxReal& ang1Y, const PxReal& ang1Z,
		const PxReal& recipResponse, const PxReal& error, PxReal& orthoAng0X, PxReal& orthoAng0Y, PxReal& orthoAng0Z, PxReal& orthoAng1X, PxReal& orthoAng1Y, PxReal& orthoAng1Z,
		PxReal& orthoRecipResponse, PxReal& orthoError, const bool disableProcessing, const PxU32 solveHint, PxU32& flags, PxU32& orthoCount, const bool finished)
	{
		if (!finished && !disableProcessing)
		{
			if (solveHint == PxConstraintSolveHint::eROTATIONAL_EQUALITY)
			{
				flags |= DY_SC_FLAG_ROT_EQ;
				orthoAng0X = ang0X; orthoAng0Y = ang0Y; orthoAng0Z = ang0Z;
				orthoAng1X = ang1X; orthoAng1Y = ang1Y; orthoAng1Z = ang1Z;
				orthoRecipResponse = recipResponse;
				orthoError = error;
				orthoCount++;
			}
			else if (solveHint & PxConstraintSolveHint::eEQUALITY)
				flags |= DY_SC_FLAG_ORTHO_TARGET;
		}
	}
}

SolverConstraintPrepState::Enum setupSolverConstraintStep4
(PxTGSSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
	const PxReal dt, const PxReal totalDt, const PxReal recipdt, const PxReal recipTotalDt, PxU32& totalRows,
	PxConstraintAllocator& allocator, PxU32 maxRows, const PxReal lengthScale);

SolverConstraintPrepState::Enum setupSolverConstraintStep4
(SolverConstraintShaderPrepDesc* PX_RESTRICT constraintShaderDescs,
	PxTGSSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
	const PxReal dt, const PxReal totalDt, const PxReal recipdt, const PxReal recipTotalDt, PxU32& totalRows,
	PxConstraintAllocator& allocator, const PxReal lengthScale)

{
	//KS - we will never get here with constraints involving articulations so we don't need to stress about those in here

	totalRows = 0;

	Px1DConstraint allRows[MAX_CONSTRAINT_ROWS * 4];

	PxU32 numRows = 0;

	PxU32 maxRows = 0;
	PxU32 preppedIndex = 0;

	for (PxU32 a = 0; a < 4; ++a)
	{
		Px1DConstraint* rows = allRows + numRows;
		SolverConstraintShaderPrepDesc& shaderDesc = constraintShaderDescs[a];
		PxTGSSolverConstraintPrepDesc& desc = constraintDescs[a];

		if (!shaderDesc.solverPrep)
			return SolverConstraintPrepState::eUNBATCHABLE;

		PxMemZero(rows + preppedIndex, sizeof(Px1DConstraint)*(MAX_CONSTRAINT_ROWS));
		for (PxU32 b = preppedIndex; b < MAX_CONSTRAINT_ROWS; ++b)
		{
			Px1DConstraint& c = rows[b];
			//Px1DConstraintInit(c);
			c.minImpulse = -PX_MAX_REAL;
			c.maxImpulse = PX_MAX_REAL;
		}

		desc.mInvMassScales.linear0 = desc.mInvMassScales.linear1 = desc.mInvMassScales.angular0 = desc.mInvMassScales.angular1 = 1.f;

		desc.body0WorldOffset = PxVec3(0.f);

		PxU32 constraintCount = (*shaderDesc.solverPrep)(rows,
			desc.body0WorldOffset,
			MAX_CONSTRAINT_ROWS,
			desc.mInvMassScales,
			shaderDesc.constantBlock,
			desc.bodyFrame0, desc.bodyFrame1, desc.extendedLimits, desc.cA2w, desc.cB2w);

		preppedIndex = MAX_CONSTRAINT_ROWS - constraintCount;

		maxRows = PxMax(constraintCount, maxRows);

		if (constraintCount == 0)
			return SolverConstraintPrepState::eUNBATCHABLE;

		desc.rows = rows;
		desc.numRows = constraintCount;
		numRows += constraintCount;

		if (desc.body0->isKinematic)
			desc.mInvMassScales.angular0 = 0.f;
		if (desc.body1->isKinematic)
			desc.mInvMassScales.angular1 = 0.f;

	}

	return setupSolverConstraintStep4(constraintDescs, dt, totalDt, recipdt, recipTotalDt, totalRows, allocator, maxRows, lengthScale);
}

SolverConstraintPrepState::Enum setupSolverConstraintStep4
(PxTGSSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
	const PxReal dt, const PxReal totalDt, const PxReal recipdt, const PxReal recipTotalDt, PxU32& totalRows,
	PxConstraintAllocator& allocator, PxU32 maxRows,
	const PxReal lengthScale)
{
	const Vec4V zero = V4Zero();
	Px1DConstraint* allSorted[MAX_CONSTRAINT_ROWS * 4];
	PxU32 startIndex[4];
	PX_ALIGN(16, PxVec4) angSqrtInvInertia0[MAX_CONSTRAINT_ROWS * 4];
	PX_ALIGN(16, PxVec4) angSqrtInvInertia1[MAX_CONSTRAINT_ROWS * 4];

	PxU32 numRows = 0;

	for (PxU32 a = 0; a < 4; ++a)
	{
		startIndex[a] = numRows;
		PxTGSSolverConstraintPrepDesc& desc = constraintDescs[a];
		Px1DConstraint** sorted = allSorted + numRows;

		for (PxU32 i = 0; i < desc.numRows; ++i)
		{
			if (desc.rows[i].flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT)
			{
				if (desc.rows[i].solveHint == PxConstraintSolveHint::eEQUALITY)
					desc.rows[i].solveHint = PxConstraintSolveHint::eROTATIONAL_EQUALITY;
				else if (desc.rows[i].solveHint == PxConstraintSolveHint::eINEQUALITY)
					desc.rows[i].solveHint = PxConstraintSolveHint::eROTATIONAL_INEQUALITY;
			}
		}


		preprocessRows(sorted, desc.rows, angSqrtInvInertia0 + numRows, angSqrtInvInertia1 + numRows, desc.numRows,
			desc.body0TxI->sqrtInvInertia, desc.body1TxI->sqrtInvInertia, desc.bodyData0->invMass, desc.bodyData1->invMass,
			desc.mInvMassScales, desc.disablePreprocessing, desc.improvedSlerp, false);

		numRows += desc.numRows;
	}


	PxU32 stride = sizeof(SolverConstraint1DStep4);


	const PxU32 constraintLength = sizeof(SolverConstraint1DHeaderStep4) + stride * maxRows;

	//KS - +16 is for the constraint progress counter, which needs to be the last element in the constraint (so that we
	//know SPU DMAs have completed)
	PxU8* ptr = allocator.reserveConstraintData(constraintLength + 16u);
	if (NULL == ptr || (reinterpret_cast<PxU8*>(-1)) == ptr)
	{
		for (PxU32 a = 0; a < 4; ++a)
		{
			PxTGSSolverConstraintPrepDesc& desc = constraintDescs[a];
			desc.desc->constraint = NULL;
			desc.desc->constraintLengthOver16 = 0;
			desc.desc->writeBack = desc.writeback;
		}

		if (NULL == ptr)
		{
			PX_WARN_ONCE(
				"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
				"Either accept joints detaching/exploding or increase buffer size allocated for constraint prep by increasing PxSceneDesc::maxNbContactDataBlocks.");
			return SolverConstraintPrepState::eOUT_OF_MEMORY;
		}
		else
		{
			PX_WARN_ONCE(
				"Attempting to allocate more than 16K of constraint data. "
				"Either accept joints detaching/exploding or simplify constraints.");
			ptr = NULL;
			return SolverConstraintPrepState::eOUT_OF_MEMORY;
		}
	}
	//desc.constraint = ptr;

	totalRows = numRows;

	const bool isKinematic00 = constraintDescs[0].body0->isKinematic;
	const bool isKinematic01 = constraintDescs[0].body1->isKinematic;
	const bool isKinematic10 = constraintDescs[1].body0->isKinematic;
	const bool isKinematic11 = constraintDescs[1].body1->isKinematic;
	const bool isKinematic20 = constraintDescs[2].body0->isKinematic;
	const bool isKinematic21 = constraintDescs[2].body1->isKinematic;
	const bool isKinematic30 = constraintDescs[3].body0->isKinematic;
	const bool isKinematic31 = constraintDescs[3].body1->isKinematic;

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxTGSSolverConstraintPrepDesc& desc = constraintDescs[a];
		desc.desc->constraint = ptr;
		desc.desc->constraintLengthOver16 = PxU16(constraintLength/16);
		desc.desc->writeBack = desc.writeback;
	}

	{
		PxU8* currPtr = ptr;
		SolverConstraint1DHeaderStep4* header = reinterpret_cast<SolverConstraint1DHeaderStep4*>(currPtr);
		currPtr += sizeof(SolverConstraint1DHeaderStep4);

		const PxStepSolverBodyData& bd00 = *constraintDescs[0].bodyData0;
		const PxStepSolverBodyData& bd01 = *constraintDescs[1].bodyData0;
		const PxStepSolverBodyData& bd02 = *constraintDescs[2].bodyData0;
		const PxStepSolverBodyData& bd03 = *constraintDescs[3].bodyData0;

		const PxStepSolverBodyData& bd10 = *constraintDescs[0].bodyData1;
		const PxStepSolverBodyData& bd11 = *constraintDescs[1].bodyData1;
		const PxStepSolverBodyData& bd12 = *constraintDescs[2].bodyData1;
		const PxStepSolverBodyData& bd13 = *constraintDescs[3].bodyData1;

		//Load up masses, invInertia, velocity etc.

		const Vec4V invMassScale0 = V4LoadXYZW(constraintDescs[0].mInvMassScales.linear0, constraintDescs[1].mInvMassScales.linear0,
			constraintDescs[2].mInvMassScales.linear0, constraintDescs[3].mInvMassScales.linear0);
		const Vec4V invMassScale1 = V4LoadXYZW(constraintDescs[0].mInvMassScales.linear1, constraintDescs[1].mInvMassScales.linear1,
			constraintDescs[2].mInvMassScales.linear1, constraintDescs[3].mInvMassScales.linear1);


		const Vec4V iMass0 = V4LoadXYZW(bd00.invMass, bd01.invMass, bd02.invMass, bd03.invMass);

		const Vec4V iMass1 = V4LoadXYZW(bd10.invMass, bd11.invMass, bd12.invMass, bd13.invMass);

		const Vec4V invMass0 = V4Mul(iMass0, invMassScale0);
		const Vec4V invMass1 = V4Mul(iMass1, invMassScale1);


		const Vec4V invInertiaScale0 = V4LoadXYZW(constraintDescs[0].mInvMassScales.angular0, constraintDescs[1].mInvMassScales.angular0,
			constraintDescs[2].mInvMassScales.angular0, constraintDescs[3].mInvMassScales.angular0);
		const Vec4V invInertiaScale1 = V4LoadXYZW(constraintDescs[0].mInvMassScales.angular1, constraintDescs[1].mInvMassScales.angular1,
			constraintDescs[2].mInvMassScales.angular1, constraintDescs[3].mInvMassScales.angular1);


		//body world offsets
		Vec4V workOffset0 = Vec4V_From_Vec3V(V3LoadU(constraintDescs[0].body0WorldOffset));
		Vec4V workOffset1 = Vec4V_From_Vec3V(V3LoadU(constraintDescs[1].body0WorldOffset));
		Vec4V workOffset2 = Vec4V_From_Vec3V(V3LoadU(constraintDescs[2].body0WorldOffset));
		Vec4V workOffset3 = Vec4V_From_Vec3V(V3LoadU(constraintDescs[3].body0WorldOffset));

		Vec4V workOffsetX, workOffsetY, workOffsetZ;

		PX_TRANSPOSE_44_34(workOffset0, workOffset1, workOffset2, workOffset3, workOffsetX, workOffsetY, workOffsetZ);

		const FloatV dtV = FLoad(totalDt);
		Vec4V linBreakForce = V4LoadXYZW(constraintDescs[0].linBreakForce, constraintDescs[1].linBreakForce,
			constraintDescs[2].linBreakForce, constraintDescs[3].linBreakForce);
		Vec4V angBreakForce = V4LoadXYZW(constraintDescs[0].angBreakForce, constraintDescs[1].angBreakForce,
			constraintDescs[2].angBreakForce, constraintDescs[3].angBreakForce);


		header->breakable[0] = PxU8((constraintDescs[0].linBreakForce != PX_MAX_F32) || (constraintDescs[0].angBreakForce != PX_MAX_F32));
		header->breakable[1] = PxU8((constraintDescs[1].linBreakForce != PX_MAX_F32) || (constraintDescs[1].angBreakForce != PX_MAX_F32));
		header->breakable[2] = PxU8((constraintDescs[2].linBreakForce != PX_MAX_F32) || (constraintDescs[2].angBreakForce != PX_MAX_F32));
		header->breakable[3] = PxU8((constraintDescs[3].linBreakForce != PX_MAX_F32) || (constraintDescs[3].angBreakForce != PX_MAX_F32));
		
		//OK, I think that's everything loaded in

		header->invMass0D0 = invMass0;
		header->invMass1D1 = invMass1;
		header->angD0 = invInertiaScale0;
		header->angD1 = invInertiaScale1;
		header->body0WorkOffset[0] = workOffsetX;
		header->body0WorkOffset[1] = workOffsetY;
		header->body0WorkOffset[2] = workOffsetZ;

		header->count = maxRows;
		header->type = DY_SC_TYPE_BLOCK_1D;
		header->linBreakImpulse = V4Scale(linBreakForce, dtV);
		header->angBreakImpulse = V4Scale(angBreakForce, dtV);
		header->counts[0] = Ps::to8(constraintDescs[0].numRows);
		header->counts[1] = Ps::to8(constraintDescs[1].numRows);
		header->counts[2] = Ps::to8(constraintDescs[2].numRows);
		header->counts[3] = Ps::to8(constraintDescs[3].numRows);


		Vec4V ca2WX, ca2WY, ca2WZ;
		Vec4V cb2WX, cb2WY, cb2WZ;

		Vec4V ca2W0 = V4LoadU(&constraintDescs[0].cA2w.x);
		Vec4V ca2W1 = V4LoadU(&constraintDescs[1].cA2w.x);
		Vec4V ca2W2 = V4LoadU(&constraintDescs[2].cA2w.x);
		Vec4V ca2W3 = V4LoadU(&constraintDescs[3].cA2w.x);

		Vec4V cb2W0 = V4LoadU(&constraintDescs[0].cB2w.x);
		Vec4V cb2W1 = V4LoadU(&constraintDescs[1].cB2w.x);
		Vec4V cb2W2 = V4LoadU(&constraintDescs[2].cB2w.x);
		Vec4V cb2W3 = V4LoadU(&constraintDescs[3].cB2w.x);

		PX_TRANSPOSE_44_34(ca2W0, ca2W1, ca2W2, ca2W3, ca2WX, ca2WY, ca2WZ);
		PX_TRANSPOSE_44_34(cb2W0, cb2W1, cb2W2, cb2W3, cb2WX, cb2WY, cb2WZ);

		Vec4V pos00 = V4LoadA(&constraintDescs[0].body0TxI->deltaBody2World.p.x);
		Vec4V pos01 = V4LoadA(&constraintDescs[0].body1TxI->deltaBody2World.p.x);
		Vec4V pos10 = V4LoadA(&constraintDescs[1].body0TxI->deltaBody2World.p.x);
		Vec4V pos11 = V4LoadA(&constraintDescs[1].body1TxI->deltaBody2World.p.x);
		Vec4V pos20 = V4LoadA(&constraintDescs[2].body0TxI->deltaBody2World.p.x);
		Vec4V pos21 = V4LoadA(&constraintDescs[2].body1TxI->deltaBody2World.p.x);
		Vec4V pos30 = V4LoadA(&constraintDescs[3].body0TxI->deltaBody2World.p.x);
		Vec4V pos31 = V4LoadA(&constraintDescs[3].body1TxI->deltaBody2World.p.x);

		Vec4V pos0X, pos0Y, pos0Z;
		Vec4V pos1X, pos1Y, pos1Z;

		PX_TRANSPOSE_44_34(pos00, pos10, pos20, pos30, pos0X, pos0Y, pos0Z);
		PX_TRANSPOSE_44_34(pos01, pos11, pos21, pos31, pos1X, pos1Y, pos1Z);

		Vec4V linVel00 = V4LoadA(&constraintDescs[0].bodyData0->originalLinearVelocity.x);
		Vec4V linVel01 = V4LoadA(&constraintDescs[0].bodyData1->originalLinearVelocity.x);
		Vec4V angState00 = V4LoadA(&constraintDescs[0].bodyData0->originalAngularVelocity.x);
		Vec4V angState01 = V4LoadA(&constraintDescs[0].bodyData1->originalAngularVelocity.x);

		Vec4V linVel10 = V4LoadA(&constraintDescs[1].bodyData0->originalLinearVelocity.x);
		Vec4V linVel11 = V4LoadA(&constraintDescs[1].bodyData1->originalLinearVelocity.x);
		Vec4V angState10 = V4LoadA(&constraintDescs[1].bodyData0->originalAngularVelocity.x);
		Vec4V angState11 = V4LoadA(&constraintDescs[1].bodyData1->originalAngularVelocity.x);

		Vec4V linVel20 = V4LoadA(&constraintDescs[2].bodyData0->originalLinearVelocity.x);
		Vec4V linVel21 = V4LoadA(&constraintDescs[2].bodyData1->originalLinearVelocity.x);
		Vec4V angState20 = V4LoadA(&constraintDescs[2].bodyData0->originalAngularVelocity.x);
		Vec4V angState21 = V4LoadA(&constraintDescs[2].bodyData1->originalAngularVelocity.x);

		Vec4V linVel30 = V4LoadA(&constraintDescs[3].bodyData0->originalLinearVelocity.x);
		Vec4V linVel31 = V4LoadA(&constraintDescs[3].bodyData1->originalLinearVelocity.x);
		Vec4V angState30 = V4LoadA(&constraintDescs[3].bodyData0->originalAngularVelocity.x);
		Vec4V angState31 = V4LoadA(&constraintDescs[3].bodyData1->originalAngularVelocity.x);


		Vec4V linVel0T0, linVel0T1, linVel0T2;
		Vec4V linVel1T0, linVel1T1, linVel1T2;
		Vec4V angState0T0, angState0T1, angState0T2;
		Vec4V angState1T0, angState1T1, angState1T2;


		PX_TRANSPOSE_44_34(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2);
		PX_TRANSPOSE_44_34(linVel01, linVel11, linVel21, linVel31, linVel1T0, linVel1T1, linVel1T2);
		PX_TRANSPOSE_44_34(angState00, angState10, angState20, angState30, angState0T0, angState0T1, angState0T2);
		PX_TRANSPOSE_44_34(angState01, angState11, angState21, angState31, angState1T0, angState1T1, angState1T2);



		const Vec4V raWorldX = V4Sub(ca2WX, pos0X);
		const Vec4V raWorldY = V4Sub(ca2WY, pos0Y);
		const Vec4V raWorldZ = V4Sub(ca2WZ, pos0Z);

		const Vec4V rbWorldX = V4Sub(cb2WX, pos1X);
		const Vec4V rbWorldY = V4Sub(cb2WY, pos1Y);
		const Vec4V rbWorldZ = V4Sub(cb2WZ, pos1Z);
		
		header->rAWorld[0] = raWorldX;
		header->rAWorld[1] = raWorldY;
		header->rAWorld[2] = raWorldZ;

		header->rBWorld[0] = rbWorldX;
		header->rBWorld[1] = rbWorldY;
		header->rBWorld[2] = rbWorldZ;

		//Now we loop over the constraints and build the results...

		PxU32 index0 = 0;
		PxU32 endIndex0 = constraintDescs[0].numRows - 1;
		PxU32 index1 = startIndex[1];
		PxU32 endIndex1 = index1 + constraintDescs[1].numRows - 1;
		PxU32 index2 = startIndex[2];
		PxU32 endIndex2 = index2 + constraintDescs[2].numRows - 1;
		PxU32 index3 = startIndex[3];
		PxU32 endIndex3 = index3 + constraintDescs[3].numRows - 1;

		const Vec4V one = V4One();
		const FloatV fOne = FOne();

		PxU32 orthoCount0 = 0, orthoCount1 = 0, orthoCount2 = 0, orthoCount3 = 0;

		for (PxU32 a = 0; a < 3; ++a)
		{
			header->angOrthoAxis0X[a] = V4Zero();
			header->angOrthoAxis0Y[a] = V4Zero();
			header->angOrthoAxis0Z[a] = V4Zero();

			header->angOrthoAxis1X[a] = V4Zero();
			header->angOrthoAxis1Y[a] = V4Zero();
			header->angOrthoAxis1Z[a] = V4Zero();

			header->angOrthoRecipResponse[a] = V4Zero();
			header->angOrthoError[a] = V4Zero();
		}

		for (PxU32 a = 0; a < maxRows; ++a)
		{
			bool finished[] = { a >= constraintDescs[0].numRows, a >= constraintDescs[1].numRows, a >= constraintDescs[2].numRows, a >= constraintDescs[3].numRows };
			BoolV bFinished = BLoad(finished);
			SolverConstraint1DStep4* c = reinterpret_cast<SolverConstraint1DStep4*>(currPtr);
			currPtr += stride;

			Px1DConstraint* con0 = allSorted[index0];
			Px1DConstraint* con1 = allSorted[index1];
			Px1DConstraint* con2 = allSorted[index2];
			Px1DConstraint* con3 = allSorted[index3];

			bool angularConstraint[4] =
			{
				!!(con0->flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT),
				!!(con1->flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT),
				!!(con2->flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT),
				!!(con3->flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT),
			};

			BoolV bAngularConstraint = BLoad(angularConstraint);

			Vec4V cangDelta00 = V4LoadA(&angSqrtInvInertia0[index0].x);
			Vec4V cangDelta01 = V4LoadA(&angSqrtInvInertia0[index1].x);
			Vec4V cangDelta02 = V4LoadA(&angSqrtInvInertia0[index2].x);
			Vec4V cangDelta03 = V4LoadA(&angSqrtInvInertia0[index3].x);

			Vec4V cangDelta10 = V4LoadA(&angSqrtInvInertia1[index0].x);
			Vec4V cangDelta11 = V4LoadA(&angSqrtInvInertia1[index1].x);
			Vec4V cangDelta12 = V4LoadA(&angSqrtInvInertia1[index2].x);
			Vec4V cangDelta13 = V4LoadA(&angSqrtInvInertia1[index3].x);

			index0 = index0 == endIndex0 ? index0 : index0 + 1;
			index1 = index1 == endIndex1 ? index1 : index1 + 1;
			index2 = index2 == endIndex2 ? index2 : index2 + 1;
			index3 = index3 == endIndex3 ? index3 : index3 + 1;

			Vec4V driveScale = one;
			if (con0->flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && constraintDescs[0].driveLimitsAreForces)
				driveScale = V4SetX(driveScale, FMin(fOne, dtV));
			if (con1->flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && constraintDescs[1].driveLimitsAreForces)
				driveScale = V4SetY(driveScale, FMin(fOne, dtV));
			if (con2->flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && constraintDescs[2].driveLimitsAreForces)
				driveScale = V4SetZ(driveScale, FMin(fOne, dtV));
			if (con3->flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && constraintDescs[3].driveLimitsAreForces)
				driveScale = V4SetW(driveScale, FMin(fOne, dtV));


			Vec4V clin00 = V4LoadA(&con0->linear0.x);
			Vec4V clin01 = V4LoadA(&con1->linear0.x);
			Vec4V clin02 = V4LoadA(&con2->linear0.x);
			Vec4V clin03 = V4LoadA(&con3->linear0.x);

			Vec4V clin0X, clin0Y, clin0Z;

			PX_TRANSPOSE_44_34(clin00, clin01, clin02, clin03, clin0X, clin0Y, clin0Z);

			Vec4V cang00 = V4LoadA(&con0->angular0.x);
			Vec4V cang01 = V4LoadA(&con1->angular0.x);
			Vec4V cang02 = V4LoadA(&con2->angular0.x);
			Vec4V cang03 = V4LoadA(&con3->angular0.x);

			Vec4V cang0X, cang0Y, cang0Z;

			PX_TRANSPOSE_44_34(cang00, cang01, cang02, cang03, cang0X, cang0Y, cang0Z);

			Vec4V cang10 = V4LoadA(&con0->angular1.x);
			Vec4V cang11 = V4LoadA(&con1->angular1.x);
			Vec4V cang12 = V4LoadA(&con2->angular1.x);
			Vec4V cang13 = V4LoadA(&con3->angular1.x);

			Vec4V cang1X, cang1Y, cang1Z;

			PX_TRANSPOSE_44_34(cang10, cang11, cang12, cang13, cang1X, cang1Y, cang1Z);

			const Vec4V maxImpulse = V4LoadXYZW(con0->maxImpulse, con1->maxImpulse, con2->maxImpulse, con3->maxImpulse);
			const Vec4V minImpulse = V4LoadXYZW(con0->minImpulse, con1->minImpulse, con2->minImpulse, con3->minImpulse);

			Vec4V angDelta0X, angDelta0Y, angDelta0Z;

			PX_TRANSPOSE_44_34(cangDelta00, cangDelta01, cangDelta02, cangDelta03, angDelta0X, angDelta0Y, angDelta0Z);

			c->flags[0] = 0;
			c->flags[1] = 0;
			c->flags[2] = 0;
			c->flags[3] = 0;

			c->lin0[0] = V4Sel(bFinished, zero, clin0X);
			c->lin0[1] = V4Sel(bFinished, zero, clin0Y);
			c->lin0[2] = V4Sel(bFinished, zero, clin0Z);
			c->ang0[0] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang0X, zero);
			c->ang0[1] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang0Y, zero);
			c->ang0[2] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang0Z, zero);
			c->angularErrorScale = V4Sel(bAngularConstraint, one, zero);

			c->minImpulse = V4Mul(minImpulse, driveScale);
			c->maxImpulse = V4Mul(maxImpulse, driveScale);
			c->appliedForce = zero;

			const Vec4V lin0MagSq = V4MulAdd(clin0Z, clin0Z, V4MulAdd(clin0Y, clin0Y, V4Mul(clin0X, clin0X)));
			const Vec4V cang0DotAngDelta = V4MulAdd(angDelta0Z, angDelta0Z, V4MulAdd(angDelta0Y, angDelta0Y, V4Mul(angDelta0X, angDelta0X)));

			Vec4V unitResponse = V4MulAdd(lin0MagSq, invMass0, V4Mul(cang0DotAngDelta, invInertiaScale0));

			Vec4V clin10 = V4LoadA(&con0->linear1.x);
			Vec4V clin11 = V4LoadA(&con1->linear1.x);
			Vec4V clin12 = V4LoadA(&con2->linear1.x);
			Vec4V clin13 = V4LoadA(&con3->linear1.x);

			Vec4V clin1X, clin1Y, clin1Z;
			PX_TRANSPOSE_44_34(clin10, clin11, clin12, clin13, clin1X, clin1Y, clin1Z);

			Vec4V angDelta1X, angDelta1Y, angDelta1Z;

			PX_TRANSPOSE_44_34(cangDelta10, cangDelta11, cangDelta12, cangDelta13, angDelta1X, angDelta1Y, angDelta1Z);

			const Vec4V lin1MagSq = V4MulAdd(clin1Z, clin1Z, V4MulAdd(clin1Y, clin1Y, V4Mul(clin1X, clin1X)));
			const Vec4V cang1DotAngDelta = V4MulAdd(angDelta1Z, angDelta1Z, V4MulAdd(angDelta1Y, angDelta1Y, V4Mul(angDelta1X, angDelta1X)));

			c->lin1[0] = V4Sel(bFinished, zero, clin1X);
			c->lin1[1] = V4Sel(bFinished, zero, clin1Y);
			c->lin1[2] = V4Sel(bFinished, zero, clin1Z);

			c->ang1[0] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang1X, zero);
			c->ang1[1] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang1Y, zero);
			c->ang1[2] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang1Z, zero);

			unitResponse = V4Add(unitResponse, V4MulAdd(lin1MagSq, invMass1, V4Mul(cang1DotAngDelta, invInertiaScale1)));

			const Vec4V lnormalVel0 = V4MulAdd(clin0X, linVel0T0, V4MulAdd(clin0Y, linVel0T1, V4Mul(clin0Z, linVel0T2)));
			const Vec4V lnormalVel1 = V4MulAdd(clin1X, linVel1T0, V4MulAdd(clin1Y, linVel1T1, V4Mul(clin1Z, linVel1T2)));

			const Vec4V angVel0 = V4MulAdd(cang0X, angState0T0, V4MulAdd(cang0Y, angState0T1, V4Mul(cang0Z, angState0T2)));
			const Vec4V angVel1 = V4MulAdd(angDelta1X, angState1T0, V4MulAdd(angDelta1Y, angState1T1, V4Mul(angDelta1Z, angState1T2)));

			const Vec4V normalVel0 = V4Add(lnormalVel0, angVel0);
			const Vec4V normalVel1 = V4Add(lnormalVel1, angVel1);

			const Vec4V normalVel = V4Sub(normalVel0, normalVel1);

			angDelta0X = V4Mul(angDelta0X, invInertiaScale0);
			angDelta0Y = V4Mul(angDelta0Y, invInertiaScale0);
			angDelta0Z = V4Mul(angDelta0Z, invInertiaScale0);

			angDelta1X = V4Mul(angDelta1X, invInertiaScale1);
			angDelta1Y = V4Mul(angDelta1Y, invInertiaScale1);
			angDelta1Z = V4Mul(angDelta1Z, invInertiaScale1);


			
			{
				PxReal recipResponse[4];
				const PxVec4& ur = reinterpret_cast<const PxVec4&>(unitResponse);
				PxVec4& cBiasScale = reinterpret_cast<PxVec4&>(c->biasScale);
				PxVec4& cError = reinterpret_cast<PxVec4&>(c->error);
				PxVec4& cMaxBias = reinterpret_cast<PxVec4&>(c->maxBias);
				PxVec4& cTargetVel = reinterpret_cast<PxVec4&>(c->velTarget);
				PxVec4& cVelMultiplier = reinterpret_cast<PxVec4&>(c->velMultiplier);
				PxVec4& cImpulseMultiplier = reinterpret_cast<PxVec4&>(c->impulseMultiplier);
				const PxVec4& nVel = reinterpret_cast<const PxVec4&>(normalVel);
				const PxVec4& nVel0 = reinterpret_cast<const PxVec4&>(normalVel0);
				const PxVec4& nVel1 = reinterpret_cast<const PxVec4&>(normalVel1);

				setConstants(cError.x, cBiasScale.x, cTargetVel.x, cMaxBias.x, cVelMultiplier.x, cImpulseMultiplier.x,
					recipResponse[0], *con0, ur.x, constraintDescs[0].minResponseThreshold, dt, totalDt, recipdt, recipTotalDt,
					a >= constraintDescs[0].numRows, lengthScale, nVel.x, nVel0.x, nVel1.x, isKinematic00, isKinematic01);

				setConstants(cError.y, cBiasScale.y, cTargetVel.y, cMaxBias.y, cVelMultiplier.y, cImpulseMultiplier.y,
					recipResponse[1], *con1, ur.y, constraintDescs[1].minResponseThreshold, dt, totalDt, recipdt, recipTotalDt,
					a >= constraintDescs[1].numRows, lengthScale, nVel.y, nVel0.y, nVel1.y, isKinematic10, isKinematic11);

				setConstants(cError.z, cBiasScale.z, cTargetVel.z, cMaxBias.z, cVelMultiplier.z, cImpulseMultiplier.z,
					recipResponse[2], *con2, ur.z, constraintDescs[2].minResponseThreshold, dt, totalDt, recipdt, recipTotalDt,
					a >= constraintDescs[2].numRows, lengthScale, nVel.z, nVel0.z, nVel1.z, isKinematic20, isKinematic21);

				setConstants(cError.w, cBiasScale.w, cTargetVel.w, cMaxBias.w, cVelMultiplier.w, cImpulseMultiplier.w,
					recipResponse[3], *con3, ur.w, constraintDescs[3].minResponseThreshold, dt, totalDt, recipdt, recipTotalDt,
					a >= constraintDescs[3].numRows, lengthScale, nVel.w, nVel0.w, nVel1.w, isKinematic30, isKinematic31);

				PxVec4* angOrthoAxes0X = reinterpret_cast<PxVec4*>(header->angOrthoAxis0X);
				PxVec4* angOrthoAxes0Y = reinterpret_cast<PxVec4*>(header->angOrthoAxis0Y);
				PxVec4* angOrthoAxes0Z = reinterpret_cast<PxVec4*>(header->angOrthoAxis0Z);
				PxVec4* angOrthoAxes1X = reinterpret_cast<PxVec4*>(header->angOrthoAxis1X);
				PxVec4* angOrthoAxes1Y = reinterpret_cast<PxVec4*>(header->angOrthoAxis1Y);
				PxVec4* angOrthoAxes1Z = reinterpret_cast<PxVec4*>(header->angOrthoAxis1Z);
				PxVec4* orthoRecipResponse = reinterpret_cast<PxVec4*>(header->angOrthoRecipResponse);
				PxVec4* orthoError = reinterpret_cast<PxVec4*>(header->angOrthoError);

				const PxVec4& ang0X = reinterpret_cast<const PxVec4&>(angDelta0X);
				const PxVec4& ang0Y = reinterpret_cast<const PxVec4&>(angDelta0Y);
				const PxVec4& ang0Z = reinterpret_cast<const PxVec4&>(angDelta0Z);

				const PxVec4& ang1X = reinterpret_cast<const PxVec4&>(angDelta1X);
				const PxVec4& ang1Y = reinterpret_cast<const PxVec4&>(angDelta1Y);
				const PxVec4& ang1Z = reinterpret_cast<const PxVec4&>(angDelta1Z);

				setOrthoData(ang0X.x, ang0Y.x, ang0Z.x, ang1X.x, ang1Y.x, ang1Z.x, recipResponse[0], cError.x, angOrthoAxes0X[orthoCount0].x,
					angOrthoAxes0Y[orthoCount0].x, angOrthoAxes0Z[orthoCount0].x, angOrthoAxes1X[orthoCount0].x, angOrthoAxes1Y[orthoCount0].x,
					angOrthoAxes1Z[orthoCount0].x, orthoRecipResponse[orthoCount0].x, orthoError[orthoCount0].x, constraintDescs[0].disablePreprocessing, con0->solveHint,
					c->flags[0], orthoCount0, a >= constraintDescs[0].numRows);

				setOrthoData(ang0X.y, ang0Y.y, ang0Z.y, ang1X.y, ang1Y.y, ang1Z.y, recipResponse[1], cError.y, angOrthoAxes0X[orthoCount1].y,
					angOrthoAxes0Y[orthoCount1].y, angOrthoAxes0Z[orthoCount1].y, angOrthoAxes1X[orthoCount1].y, angOrthoAxes1Y[orthoCount1].y,
					angOrthoAxes1Z[orthoCount1].y, orthoRecipResponse[orthoCount1].y, orthoError[orthoCount1].y, constraintDescs[1].disablePreprocessing, con1->solveHint,
					c->flags[1], orthoCount1, a >= constraintDescs[1].numRows);

				setOrthoData(ang0X.z, ang0Y.z, ang0Z.z, ang1X.z, ang1Y.z, ang1Z.z, recipResponse[2], cError.z, angOrthoAxes0X[orthoCount2].z,
					angOrthoAxes0Y[orthoCount2].z, angOrthoAxes0Z[orthoCount2].z, angOrthoAxes1X[orthoCount2].z, angOrthoAxes1Y[orthoCount2].z,
					angOrthoAxes1Z[orthoCount2].z, orthoRecipResponse[orthoCount2].z, orthoError[orthoCount2].z, constraintDescs[2].disablePreprocessing, con2->solveHint,
					c->flags[2], orthoCount2, a >= constraintDescs[2].numRows);

				setOrthoData(ang0X.w, ang0Y.w, ang0Z.w, ang1X.w, ang1Y.w, ang1Z.w, recipResponse[3], cError.w, angOrthoAxes0X[orthoCount3].w,
					angOrthoAxes0Y[orthoCount3].w, angOrthoAxes0Z[orthoCount3].w, angOrthoAxes1X[orthoCount3].w, angOrthoAxes1Y[orthoCount3].w,
					angOrthoAxes1Z[orthoCount3].w, orthoRecipResponse[orthoCount3].w, orthoError[orthoCount3].w, constraintDescs[3].disablePreprocessing, con3->solveHint,
					c->flags[3], orthoCount3, a >= constraintDescs[3].numRows);

			}

			if (con0->flags & Px1DConstraintFlag::eOUTPUT_FORCE)
				c->flags[0] |= DY_SC_FLAG_OUTPUT_FORCE;
			if (con1->flags & Px1DConstraintFlag::eOUTPUT_FORCE)
				c->flags[1] |= DY_SC_FLAG_OUTPUT_FORCE;
			if (con2->flags & Px1DConstraintFlag::eOUTPUT_FORCE)
				c->flags[2] |= DY_SC_FLAG_OUTPUT_FORCE;
			if (con3->flags & Px1DConstraintFlag::eOUTPUT_FORCE)
				c->flags[3] |= DY_SC_FLAG_OUTPUT_FORCE;

			if ((con0->flags & Px1DConstraintFlag::eKEEPBIAS))
				c->flags[0] |= DY_SC_FLAG_KEEP_BIAS;
			if ((con1->flags & Px1DConstraintFlag::eKEEPBIAS))
				c->flags[1] |= DY_SC_FLAG_KEEP_BIAS;
			if ((con2->flags & Px1DConstraintFlag::eKEEPBIAS))
				c->flags[2] |= DY_SC_FLAG_KEEP_BIAS;
			if ((con3->flags & Px1DConstraintFlag::eKEEPBIAS))
				c->flags[3] |= DY_SC_FLAG_KEEP_BIAS;

			//Flag inequality constraints...
			if (con0->solveHint & 1)
				c->flags[0] |= DY_SC_FLAG_INEQUALITY;
			if (con1->solveHint & 1)
				c->flags[1] |= DY_SC_FLAG_INEQUALITY;
			if (con2->solveHint & 1)
				c->flags[2] |= DY_SC_FLAG_INEQUALITY;
			if (con3->solveHint & 1)
				c->flags[3] |= DY_SC_FLAG_INEQUALITY;

		}
		*(reinterpret_cast<PxU32*>(currPtr)) = 0;
		*(reinterpret_cast<PxU32*>(currPtr + 4)) = 0;
	}

	//OK, we're ready to allocate and solve prep these constraints now :-)
	return SolverConstraintPrepState::eSUCCESS;
}





void solveContact4_Block(const PxTGSSolverConstraintDesc* PX_RESTRICT desc, const bool doFriction, const PxReal minPenetration,
	const PxReal elapsedTimeF32)
{
	PxTGSSolverBodyVel& b00 = *desc[0].bodyA;
	PxTGSSolverBodyVel& b01 = *desc[0].bodyB;
	PxTGSSolverBodyVel& b10 = *desc[1].bodyA;
	PxTGSSolverBodyVel& b11 = *desc[1].bodyB;
	PxTGSSolverBodyVel& b20 = *desc[2].bodyA;
	PxTGSSolverBodyVel& b21 = *desc[2].bodyB;
	PxTGSSolverBodyVel& b30 = *desc[3].bodyA;
	PxTGSSolverBodyVel& b31 = *desc[3].bodyB;

	const Vec4V minPen = V4Load(minPenetration);

	const Vec4V elapsedTime = V4Load(elapsedTimeF32);

	//We'll need this.
	const Vec4V vZero = V4Zero();

	Vec4V linVel00 = V4LoadA(&b00.linearVelocity.x);
	Vec4V linVel01 = V4LoadA(&b01.linearVelocity.x);
	Vec4V angState00 = V4LoadA(&b00.angularVelocity.x);
	Vec4V angState01 = V4LoadA(&b01.angularVelocity.x);

	Vec4V linVel10 = V4LoadA(&b10.linearVelocity.x);
	Vec4V linVel11 = V4LoadA(&b11.linearVelocity.x);
	Vec4V angState10 = V4LoadA(&b10.angularVelocity.x);
	Vec4V angState11 = V4LoadA(&b11.angularVelocity.x);

	Vec4V linVel20 = V4LoadA(&b20.linearVelocity.x);
	Vec4V linVel21 = V4LoadA(&b21.linearVelocity.x);
	Vec4V angState20 = V4LoadA(&b20.angularVelocity.x);
	Vec4V angState21 = V4LoadA(&b21.angularVelocity.x);

	Vec4V linVel30 = V4LoadA(&b30.linearVelocity.x);
	Vec4V linVel31 = V4LoadA(&b31.linearVelocity.x);
	Vec4V angState30 = V4LoadA(&b30.angularVelocity.x);
	Vec4V angState31 = V4LoadA(&b31.angularVelocity.x);


	Vec4V linVel0T0, linVel0T1, linVel0T2;
	Vec4V linVel1T0, linVel1T1, linVel1T2;
	Vec4V angState0T0, angState0T1, angState0T2;
	Vec4V angState1T0, angState1T1, angState1T2;


	PX_TRANSPOSE_44_34(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2);
	PX_TRANSPOSE_44_34(linVel01, linVel11, linVel21, linVel31, linVel1T0, linVel1T1, linVel1T2);
	PX_TRANSPOSE_44_34(angState00, angState10, angState20, angState30, angState0T0, angState0T1, angState0T2);
	PX_TRANSPOSE_44_34(angState01, angState11, angState21, angState31, angState1T0, angState1T1, angState1T2);


	Vec4V linDelta00 = V4LoadA(&b00.deltaLinDt.x);
	Vec4V linDelta01 = V4LoadA(&b01.deltaLinDt.x);
	Vec4V angDelta00 = V4LoadA(&b00.deltaAngDt.x);
	Vec4V angDelta01 = V4LoadA(&b01.deltaAngDt.x);

	Vec4V linDelta10 = V4LoadA(&b10.deltaLinDt.x);
	Vec4V linDelta11 = V4LoadA(&b11.deltaLinDt.x);
	Vec4V angDelta10 = V4LoadA(&b10.deltaAngDt.x);
	Vec4V angDelta11 = V4LoadA(&b11.deltaAngDt.x);

	Vec4V linDelta20 = V4LoadA(&b20.deltaLinDt.x);
	Vec4V linDelta21 = V4LoadA(&b21.deltaLinDt.x);
	Vec4V angDelta20 = V4LoadA(&b20.deltaAngDt.x);
	Vec4V angDelta21 = V4LoadA(&b21.deltaAngDt.x);

	Vec4V linDelta30 = V4LoadA(&b30.deltaLinDt.x);
	Vec4V linDelta31 = V4LoadA(&b31.deltaLinDt.x);
	Vec4V angDelta30 = V4LoadA(&b30.deltaAngDt.x);
	Vec4V angDelta31 = V4LoadA(&b31.deltaAngDt.x);

	Vec4V linDelta0T0, linDelta0T1, linDelta0T2;
	Vec4V linDelta1T0, linDelta1T1, linDelta1T2;
	Vec4V angDelta0T0, angDelta0T1, angDelta0T2;
	Vec4V angDelta1T0, angDelta1T1, angDelta1T2;


	PX_TRANSPOSE_44_34(linDelta00, linDelta10, linDelta20, linDelta30, linDelta0T0, linDelta0T1, linDelta0T2);
	PX_TRANSPOSE_44_34(linDelta01, linDelta11, linDelta21, linDelta31, linDelta1T0, linDelta1T1, linDelta1T2);
	PX_TRANSPOSE_44_34(angDelta00, angDelta10, angDelta20, angDelta30, angDelta0T0, angDelta0T1, angDelta0T2);
	PX_TRANSPOSE_44_34(angDelta01, angDelta11, angDelta21, angDelta31, angDelta1T0, angDelta1T1, angDelta1T2);


	const PxU8* PX_RESTRICT last = desc[0].constraint + getConstraintLength(desc[0]);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc[0].constraint;

	Vec4V vMax = V4Splat(FMax());

	const PxU8* PX_RESTRICT prefetchAddress = currPtr + sizeof(SolverContactHeaderStepBlock) + sizeof(SolverContactPointStepBlock);

	SolverContactHeaderStepBlock* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStepBlock*>(currPtr);

	const Vec4V invMassA = hdr->invMass0D0;
	const Vec4V invMassB = hdr->invMass1D1;

	const Vec4V sumInvMass = V4Add(invMassA, invMassB);

	Vec4V linDeltaX = V4Sub(linDelta0T0, linDelta1T0);
	Vec4V linDeltaY = V4Sub(linDelta0T1, linDelta1T1);
	Vec4V linDeltaZ = V4Sub(linDelta0T2, linDelta1T2);


	while (currPtr < last)
	{

		hdr = reinterpret_cast<SolverContactHeaderStepBlock*>(currPtr);

		PX_ASSERT(hdr->type == DY_SC_TYPE_BLOCK_RB_CONTACT);

		currPtr = reinterpret_cast<PxU8*>(const_cast<SolverContactHeaderStepBlock*>(hdr) + 1);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		bool hasMaxImpulse = (hdr->flag & SolverContactHeaderStepBlock::eHAS_MAX_IMPULSE) != 0;

		Vec4V* appliedForces = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numNormalConstr;

		SolverContactPointStepBlock* PX_RESTRICT contacts = reinterpret_cast<SolverContactPointStepBlock*>(currPtr);

		Vec4V* maxImpulses;
		currPtr = reinterpret_cast<PxU8*>(contacts + numNormalConstr);
		PxU32 maxImpulseMask = 0;
		if (hasMaxImpulse)
		{
			maxImpulseMask = 0xFFFFFFFF;
			maxImpulses = reinterpret_cast<Vec4V*>(currPtr);
			currPtr += sizeof(Vec4V) * numNormalConstr;
		}
		else
		{
			maxImpulses = &vMax;
		}


		/*SolverFrictionSharedData4* PX_RESTRICT fd = reinterpret_cast<SolverFrictionSharedData4*>(currPtr);
		if (numFrictionConstr)
			currPtr += sizeof(SolverFrictionSharedData4);*/

		Vec4V* frictionAppliedForce = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numFrictionConstr;

		const SolverContactFrictionStepBlock* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionStepBlock*>(currPtr);
		currPtr += numFrictionConstr * sizeof(SolverContactFrictionStepBlock);

		Vec4V accumulatedNormalImpulse = vZero;

		const Vec4V angD0 = hdr->angDom0;
		const Vec4V angD1 = hdr->angDom1;

		const Vec4V _normalT0 = hdr->normalX;
		const Vec4V _normalT1 = hdr->normalY;
		const Vec4V _normalT2 = hdr->normalZ;

		Vec4V contactNormalVel1 = V4Mul(linVel0T0, _normalT0);
		Vec4V contactNormalVel3 = V4Mul(linVel1T0, _normalT0);
		contactNormalVel1 = V4MulAdd(linVel0T1, _normalT1, contactNormalVel1);
		contactNormalVel3 = V4MulAdd(linVel1T1, _normalT1, contactNormalVel3);
		contactNormalVel1 = V4MulAdd(linVel0T2, _normalT2, contactNormalVel1);
		contactNormalVel3 = V4MulAdd(linVel1T2, _normalT2, contactNormalVel3);

		const Vec4V maxPenBias = hdr->maxPenBias;

		Vec4V relVel1 = V4Sub(contactNormalVel1, contactNormalVel3);

		Vec4V deltaNormalV = V4Mul(linDeltaX, _normalT0);
		deltaNormalV = V4MulAdd(linDeltaY, _normalT1, deltaNormalV);
		deltaNormalV = V4MulAdd(linDeltaZ, _normalT2, deltaNormalV);

		Vec4V accumDeltaF = vZero;

		for (PxU32 i = 0; i<numNormalConstr; i++)
		{
			const SolverContactPointStepBlock& c = contacts[i];

			PxU32 offset = 0;
			Ps::prefetchLine(prefetchAddress, offset += 64);
			Ps::prefetchLine(prefetchAddress, offset += 64);
			Ps::prefetchLine(prefetchAddress, offset += 64);
			prefetchAddress += offset;

			const Vec4V appliedForce = appliedForces[i];
			const Vec4V maxImpulse = maxImpulses[i & maxImpulseMask];

			Vec4V contactNormalVel2 = V4Mul(c.raXnI[0], angState0T0);
			Vec4V contactNormalVel4 = V4Mul(c.rbXnI[0], angState1T0);

			contactNormalVel2 = V4MulAdd(c.raXnI[1], angState0T1, contactNormalVel2);
			contactNormalVel4 = V4MulAdd(c.rbXnI[1], angState1T1, contactNormalVel4);

			contactNormalVel2 = V4MulAdd(c.raXnI[2], angState0T2, contactNormalVel2);
			contactNormalVel4 = V4MulAdd(c.rbXnI[2], angState1T2, contactNormalVel4);

			const Vec4V normalVel = V4Add(relVel1, V4Sub(contactNormalVel2, contactNormalVel4));

			Vec4V angDelta0 = V4Mul(angDelta0T0, c.raXnI[0]);
			Vec4V angDelta1 = V4Mul(angDelta1T0, c.rbXnI[0]);
			angDelta0 = V4MulAdd(angDelta0T1, c.raXnI[1], angDelta0);
			angDelta1 = V4MulAdd(angDelta1T1, c.rbXnI[1], angDelta1);
			angDelta0 = V4MulAdd(angDelta0T2, c.raXnI[2], angDelta0);
			angDelta1 = V4MulAdd(angDelta1T2, c.rbXnI[2], angDelta1);

			const Vec4V deltaAng = V4Sub(angDelta0, angDelta1);

			const Vec4V targetVel = c.targetVelocity;

			const Vec4V deltaBias = V4Sub(V4Add(deltaNormalV, deltaAng), V4Mul(targetVel, elapsedTime));

			const Vec4V biasCoefficient = c.biasCoefficient;

			const Vec4V sep = V4Max(minPen, V4Add(c.separation, deltaBias));

			const Vec4V bias = V4Min(V4Neg(maxPenBias), V4Mul(biasCoefficient, sep));

			

			const Vec4V velMultiplier = c.velMultiplier;

			const Vec4V tVel = V4Add(bias, targetVel);

			const Vec4V _deltaF = V4Max(V4Mul(V4Sub(tVel, normalVel), velMultiplier), V4Neg(appliedForce));
			//Vec4V deltaF = V4NegMulSub(normalVel, c.velMultiplier, c.biasedErr);

			const Vec4V newAppliedForce = V4Min(V4Add(appliedForce, _deltaF), maxImpulse);
			const Vec4V deltaF = V4Sub(newAppliedForce, appliedForce);

			accumDeltaF = V4Add(accumDeltaF, deltaF);

			const Vec4V angDetaF0 = V4Mul(deltaF, angD0);
			const Vec4V angDetaF1 = V4Mul(deltaF, angD1);

			relVel1 = V4MulAdd(sumInvMass, deltaF, relVel1);

			angState0T0 = V4MulAdd(c.raXnI[0], angDetaF0, angState0T0);
			angState1T0 = V4NegMulSub(c.rbXnI[0], angDetaF1, angState1T0);

			angState0T1 = V4MulAdd(c.raXnI[1], angDetaF0, angState0T1);
			angState1T1 = V4NegMulSub(c.rbXnI[1], angDetaF1, angState1T1);

			angState0T2 = V4MulAdd(c.raXnI[2], angDetaF0, angState0T2);
			angState1T2 = V4NegMulSub(c.rbXnI[2], angDetaF1, angState1T2);

			appliedForces[i] = newAppliedForce;

			accumulatedNormalImpulse = V4Add(accumulatedNormalImpulse, newAppliedForce);
		}

		const Vec4V accumDeltaF_IM0 = V4Mul(accumDeltaF, invMassA);
		const Vec4V accumDeltaF_IM1 = V4Mul(accumDeltaF, invMassB);

		linVel0T0 = V4MulAdd(_normalT0, accumDeltaF_IM0, linVel0T0);
		linVel1T0 = V4NegMulSub(_normalT0, accumDeltaF_IM1, linVel1T0);
		linVel0T1 = V4MulAdd(_normalT1, accumDeltaF_IM0, linVel0T1);
		linVel1T1 = V4NegMulSub(_normalT1, accumDeltaF_IM1, linVel1T1);
		linVel0T2 = V4MulAdd(_normalT2, accumDeltaF_IM0, linVel0T2);
		linVel1T2 = V4NegMulSub(_normalT2, accumDeltaF_IM1, linVel1T2);


		if (doFriction && numFrictionConstr)
		{
			const Vec4V staticFric = hdr->staticFriction;
			const Vec4V dynamicFric = hdr->dynamicFriction;

			const Vec4V maxFrictionImpulse = V4Mul(staticFric, accumulatedNormalImpulse);
			const Vec4V maxDynFrictionImpulse = V4Mul(dynamicFric, accumulatedNormalImpulse);
			const Vec4V negMaxDynFrictionImpulse = V4Neg(maxDynFrictionImpulse);
			//const Vec4V negMaxFrictionImpulse = V4Neg(maxFrictionImpulse);
			BoolV broken = BFFFF();


			for (PxU32 i = 0; i<numFrictionConstr; i++)
			{
				const SolverContactFrictionStepBlock& f = frictions[i];

				PxU32 offset = 0;
				Ps::prefetchLine(prefetchAddress, offset += 64);
				Ps::prefetchLine(prefetchAddress, offset += 64);
				Ps::prefetchLine(prefetchAddress, offset += 64);
				Ps::prefetchLine(prefetchAddress, offset += 64);
				prefetchAddress += offset;

				const Vec4V appliedForce = frictionAppliedForce[i];

				const Vec4V normalT0 = f.normal[0];
				const Vec4V normalT1 = f.normal[1];
				const Vec4V normalT2 = f.normal[2];

				Vec4V normalVel1 = V4Mul(linVel0T0, normalT0);
				Vec4V normalVel2 = V4Mul(f.raXnI[0], angState0T0);
				Vec4V normalVel3 = V4Mul(linVel1T0, normalT0);
				Vec4V normalVel4 = V4Mul(f.rbXnI[0], angState1T0);

				normalVel1 = V4MulAdd(linVel0T1, normalT1, normalVel1);
				normalVel2 = V4MulAdd(f.raXnI[1], angState0T1, normalVel2);
				normalVel3 = V4MulAdd(linVel1T1, normalT1, normalVel3);
				normalVel4 = V4MulAdd(f.rbXnI[1], angState1T1, normalVel4);

				normalVel1 = V4MulAdd(linVel0T2, normalT2, normalVel1);
				normalVel2 = V4MulAdd(f.raXnI[2], angState0T2, normalVel2);
				normalVel3 = V4MulAdd(linVel1T2, normalT2, normalVel3);
				normalVel4 = V4MulAdd(f.rbXnI[2], angState1T2, normalVel4);

				const Vec4V _normalVel = V4Add(normalVel1, normalVel2);
				const Vec4V __normalVel = V4Add(normalVel3, normalVel4);

				// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation

				const Vec4V normalVel = V4Sub(_normalVel, __normalVel);

				Vec4V deltaV = V4Mul(linDeltaX, normalT0);
				deltaV = V4MulAdd(linDeltaY, normalT1, deltaV);
				deltaV = V4MulAdd(linDeltaZ, normalT2, deltaV);

				Vec4V angDelta0 = V4Mul(angDelta0T0, f.raXnI[0]);
				Vec4V angDelta1 = V4Mul(angDelta1T0, f.rbXnI[0]);
				angDelta0 = V4MulAdd(angDelta0T1, f.raXnI[1], angDelta0);
				angDelta1 = V4MulAdd(angDelta1T1, f.rbXnI[1], angDelta1);
				angDelta0 = V4MulAdd(angDelta0T2, f.raXnI[2], angDelta0);
				angDelta1 = V4MulAdd(angDelta1T2, f.rbXnI[2], angDelta1);

				const Vec4V deltaAng = V4Sub(angDelta0, angDelta1);

				const Vec4V deltaBias = V4Sub(V4Add(deltaV, deltaAng), V4Mul(f.targetVel, elapsedTime));

				const Vec4V error = V4Add(f.error, deltaBias);

				const Vec4V bias = V4Mul(error, f.biasCoefficient);

				const Vec4V tmp1 = V4NegMulSub(V4Sub(bias, f.targetVel), f.velMultiplier, appliedForce);

				const Vec4V totalImpulse = V4NegMulSub(normalVel, f.velMultiplier, tmp1);

				const BoolV clamped = V4IsGrtr(V4Abs(totalImpulse), maxFrictionImpulse);

				broken = BOr(broken, clamped);

				const Vec4V newAppliedForce = V4Min(maxDynFrictionImpulse, V4Max(negMaxDynFrictionImpulse, totalImpulse));

				const Vec4V deltaF = V4Sub(newAppliedForce, appliedForce);

				frictionAppliedForce[i] = newAppliedForce;

				const Vec4V deltaFIM0 = V4Mul(deltaF, invMassA);
				const Vec4V deltaFIM1 = V4Mul(deltaF, invMassB);

				const Vec4V angDetaF0 = V4Mul(deltaF, angD0);
				const Vec4V angDetaF1 = V4Mul(deltaF, angD1);

				linVel0T0 = V4MulAdd(normalT0, deltaFIM0, linVel0T0);
				linVel1T0 = V4NegMulSub(normalT0, deltaFIM1, linVel1T0);
				angState0T0 = V4MulAdd(f.raXnI[0], angDetaF0, angState0T0);
				angState1T0 = V4NegMulSub(f.rbXnI[0], angDetaF1, angState1T0);

				linVel0T1 = V4MulAdd(normalT1, deltaFIM0, linVel0T1);
				linVel1T1 = V4NegMulSub(normalT1, deltaFIM1, linVel1T1);
				angState0T1 = V4MulAdd(f.raXnI[1], angDetaF0, angState0T1);
				angState1T1 = V4NegMulSub(f.rbXnI[1], angDetaF1, angState1T1);

				linVel0T2 = V4MulAdd(normalT2, deltaFIM0, linVel0T2);
				linVel1T2 = V4NegMulSub(normalT2, deltaFIM1, linVel1T2);
				angState0T2 = V4MulAdd(f.raXnI[2], angDetaF0, angState0T2);
				angState1T2 = V4NegMulSub(f.rbXnI[2], angDetaF1, angState1T2);
			}
			hdr->broken = broken;
		}
	}

	PX_TRANSPOSE_34_44(linVel0T0, linVel0T1, linVel0T2, linVel00, linVel10, linVel20, linVel30);
	PX_TRANSPOSE_34_44(linVel1T0, linVel1T1, linVel1T2, linVel01, linVel11, linVel21, linVel31);
	PX_TRANSPOSE_34_44(angState0T0, angState0T1, angState0T2, angState00, angState10, angState20, angState30);
	PX_TRANSPOSE_34_44(angState1T0, angState1T1, angState1T2, angState01, angState11, angState21, angState31);

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularVelocity.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularVelocity.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularVelocity.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularVelocity.isFinite());

	PX_ASSERT(b01.linearVelocity.isFinite());
	PX_ASSERT(b01.angularVelocity.isFinite());
	PX_ASSERT(b11.linearVelocity.isFinite());
	PX_ASSERT(b11.angularVelocity.isFinite());
	PX_ASSERT(b21.linearVelocity.isFinite());
	PX_ASSERT(b21.angularVelocity.isFinite());
	PX_ASSERT(b31.linearVelocity.isFinite());
	PX_ASSERT(b31.angularVelocity.isFinite());

	// Write back
	V4StoreA(linVel00, &b00.linearVelocity.x);
	V4StoreA(angState00, &b00.angularVelocity.x);
	V4StoreA(linVel10, &b10.linearVelocity.x);
	V4StoreA(angState10, &b10.angularVelocity.x);
	V4StoreA(linVel20, &b20.linearVelocity.x);
	V4StoreA(angState20, &b20.angularVelocity.x);
	V4StoreA(linVel30, &b30.linearVelocity.x);
	V4StoreA(angState30, &b30.angularVelocity.x);

	if (desc[0].bodyBIdx != 0)
	{
		V4StoreA(linVel01, &b01.linearVelocity.x);
		V4StoreA(angState01, &b01.angularVelocity.x);
	}
	if (desc[1].bodyBIdx != 0)
	{
		V4StoreA(linVel11, &b11.linearVelocity.x);
		V4StoreA(angState11, &b11.angularVelocity.x);
	}
	if (desc[2].bodyBIdx != 0)
	{
		V4StoreA(linVel21, &b21.linearVelocity.x);
		V4StoreA(angState21, &b21.angularVelocity.x);
	}
	if (desc[3].bodyBIdx != 0)
	{
		V4StoreA(linVel31, &b31.linearVelocity.x);
		V4StoreA(angState31, &b31.angularVelocity.x);
	}

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularVelocity.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularVelocity.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularVelocity.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularVelocity.isFinite());

	PX_ASSERT(b01.linearVelocity.isFinite());
	PX_ASSERT(b01.angularVelocity.isFinite());
	PX_ASSERT(b11.linearVelocity.isFinite());
	PX_ASSERT(b11.angularVelocity.isFinite());
	PX_ASSERT(b21.linearVelocity.isFinite());
	PX_ASSERT(b21.angularVelocity.isFinite());
	PX_ASSERT(b31.linearVelocity.isFinite());
	PX_ASSERT(b31.angularVelocity.isFinite());
}


void writeBackContact4_Block(const PxTGSSolverConstraintDesc* PX_RESTRICT desc, SolverContext* cache)
{
	PX_UNUSED(cache);
	const PxU8* PX_RESTRICT last = desc[0].constraint + getConstraintLength(desc[0]);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc[0].constraint;
	PxReal* PX_RESTRICT vForceWriteback0 = reinterpret_cast<PxReal*>(desc[0].writeBack);
	PxReal* PX_RESTRICT vForceWriteback1 = reinterpret_cast<PxReal*>(desc[1].writeBack);
	PxReal* PX_RESTRICT vForceWriteback2 = reinterpret_cast<PxReal*>(desc[2].writeBack);
	PxReal* PX_RESTRICT vForceWriteback3 = reinterpret_cast<PxReal*>(desc[3].writeBack);

	//const PxU8 type = *desc[0].constraint;
	const PxU32 contactSize = sizeof(SolverContactPointStepBlock);
	const PxU32 frictionSize = sizeof(SolverContactFrictionStepBlock);


	Vec4V normalForce = V4Zero();


	//We'll need this.
	//const Vec4V vZero	= V4Zero();

	bool writeBackThresholds[4] = { false, false, false, false };

	while ((currPtr < last))
	{
		SolverContactHeaderStepBlock* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStepBlock*>(currPtr);

		currPtr = reinterpret_cast<PxU8*>(hdr + 1);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		Vec4V* PX_RESTRICT appliedForces = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numNormalConstr;

		//SolverContactBatchPointBase4* PX_RESTRICT contacts = (SolverContactBatchPointBase4*)currPtr;
		currPtr += (numNormalConstr * contactSize);

		bool hasMaxImpulse = (hdr->flag & SolverContactHeader4::eHAS_MAX_IMPULSE) != 0;

		if (hasMaxImpulse)
			currPtr += sizeof(Vec4V) * numNormalConstr;

		currPtr += sizeof(Vec4V)*numFrictionConstr;

		//SolverContactFrictionBase4* PX_RESTRICT frictions = (SolverContactFrictionBase4*)currPtr;
		currPtr += (numFrictionConstr * frictionSize);

		writeBackThresholds[0] = hdr->flags[0] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[1] = hdr->flags[1] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[2] = hdr->flags[2] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[3] = hdr->flags[3] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;


		for (PxU32 i = 0; i<numNormalConstr; i++)
		{
			//contacts = (SolverContactBatchPointBase4*)(((PxU8*)contacts) + contactSize);
			const FloatV appliedForce0 = V4GetX(appliedForces[i]);
			const FloatV appliedForce1 = V4GetY(appliedForces[i]);
			const FloatV appliedForce2 = V4GetZ(appliedForces[i]);
			const FloatV appliedForce3 = V4GetW(appliedForces[i]);

			normalForce = V4Add(normalForce, appliedForces[i]);

			if (vForceWriteback0 && i < hdr->numNormalConstrs[0])
				FStore(appliedForce0, vForceWriteback0++);
			if (vForceWriteback1 && i < hdr->numNormalConstrs[1])
				FStore(appliedForce1, vForceWriteback1++);
			if (vForceWriteback2 && i < hdr->numNormalConstrs[2])
				FStore(appliedForce2, vForceWriteback2++);
			if (vForceWriteback3 && i < hdr->numNormalConstrs[3])
				FStore(appliedForce3, vForceWriteback3++);
		}

		if (numFrictionConstr)
		{
			PX_ALIGN(16, PxU32 broken[4]);
			BStoreA(hdr->broken, broken);

			PxU8* frictionCounts = hdr->numNormalConstrs;

			for (PxU32 a = 0; a < 4; ++a)
			{
				if (frictionCounts[a] && broken[a])
					*hdr->frictionBrokenWritebackByte[a] = 1;	// PT: bad L2 miss here
			}
		}
	}

	PX_UNUSED(writeBackThresholds);

#if 0
	if (cache)
	{

		PX_ALIGN(16, PxReal nf[4]);
		V4StoreA(normalForce, nf);

		Sc::ShapeInteraction** shapeInteractions = reinterpret_cast<SolverContactHeader4*>(desc[0].constraint)->shapeInteraction;

		for (PxU32 a = 0; a < 4; ++a)
		{
			if (writeBackThresholds[a] && desc[a].linkIndexA == PxSolverConstraintDesc::NO_LINK && desc[a].linkIndexB == PxSolverConstraintDesc::NO_LINK &&
				nf[a] != 0.f && (bd0[a]->reportThreshold < PX_MAX_REAL || bd1[a]->reportThreshold < PX_MAX_REAL))
			{
				ThresholdStreamElement elt;
				elt.normalForce = nf[a];
				elt.threshold = PxMin<float>(bd0[a]->reportThreshold, bd1[a]->reportThreshold);
				elt.nodeIndexA = bd0[a]->nodeIndex;
				elt.nodeIndexB = bd1[a]->nodeIndex;
				elt.shapeInteraction = shapeInteractions[a];
				Ps::order(elt.nodeIndexA, elt.nodeIndexB);
				PX_ASSERT(elt.nodeIndexA < elt.nodeIndexB);
				PX_ASSERT(cache.mThresholdStreamIndex < cache.mThresholdStreamLength);
				cache.mThresholdStream[cache.mThresholdStreamIndex++] = elt;
			}
		}
	}
#endif
}

void solveContact4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc, const bool doFriction,
	const PxReal minPenetration, const PxReal elapsedTime)
{
	solveContact4_Block(desc + hdr.mStartIndex, doFriction, minPenetration, elapsedTime);
}

void writeBackContact4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc, SolverContext* cache)
{
	writeBackContact4_Block(desc + hdr.mStartIndex, cache);
}

PX_FORCE_INLINE Vec4V V4Dot3(const Vec4V& x0, const Vec4V& y0, const Vec4V& z0, const Vec4V& x1, const Vec4V& y1, const Vec4V& z1)
{
	return V4MulAdd(x0, x1, V4MulAdd(y0, y1, V4Mul(z0, z1)));
}


void solve1DStep4(const PxTGSSolverConstraintDesc* PX_RESTRICT desc, const PxTGSSolverBodyTxInertia* const txInertias,
	const PxReal elapsedTimeF32)
{
	PxU8* PX_RESTRICT bPtr = desc->constraint;
	if (bPtr == NULL)
		return;

	const FloatV elapsedTime = FLoad(elapsedTimeF32);

	PxTGSSolverBodyVel& b00 = *desc[0].bodyA;
	PxTGSSolverBodyVel& b01 = *desc[0].bodyB;
	PxTGSSolverBodyVel& b10 = *desc[1].bodyA;
	PxTGSSolverBodyVel& b11 = *desc[1].bodyB;
	PxTGSSolverBodyVel& b20 = *desc[2].bodyA;
	PxTGSSolverBodyVel& b21 = *desc[2].bodyB;
	PxTGSSolverBodyVel& b30 = *desc[3].bodyA;
	PxTGSSolverBodyVel& b31 = *desc[3].bodyB;

	const PxTGSSolverBodyTxInertia& txI00 = txInertias[desc[0].bodyAIdx];
	const PxTGSSolverBodyTxInertia& txI01 = txInertias[desc[0].bodyBIdx];
	const PxTGSSolverBodyTxInertia& txI10 = txInertias[desc[1].bodyAIdx];
	const PxTGSSolverBodyTxInertia& txI11 = txInertias[desc[1].bodyBIdx];
	const PxTGSSolverBodyTxInertia& txI20 = txInertias[desc[2].bodyAIdx];
	const PxTGSSolverBodyTxInertia& txI21 = txInertias[desc[2].bodyBIdx];
	const PxTGSSolverBodyTxInertia& txI30 = txInertias[desc[3].bodyAIdx];
	const PxTGSSolverBodyTxInertia& txI31 = txInertias[desc[3].bodyBIdx];

	Vec4V linVel00 = V4LoadA(&b00.linearVelocity.x);
	Vec4V linVel01 = V4LoadA(&b01.linearVelocity.x);
	Vec4V angState00 = V4LoadA(&b00.angularVelocity.x);
	Vec4V angState01 = V4LoadA(&b01.angularVelocity.x);

	Vec4V linVel10 = V4LoadA(&b10.linearVelocity.x);
	Vec4V linVel11 = V4LoadA(&b11.linearVelocity.x);
	Vec4V angState10 = V4LoadA(&b10.angularVelocity.x);
	Vec4V angState11 = V4LoadA(&b11.angularVelocity.x);

	Vec4V linVel20 = V4LoadA(&b20.linearVelocity.x);
	Vec4V linVel21 = V4LoadA(&b21.linearVelocity.x);
	Vec4V angState20 = V4LoadA(&b20.angularVelocity.x);
	Vec4V angState21 = V4LoadA(&b21.angularVelocity.x);

	Vec4V linVel30 = V4LoadA(&b30.linearVelocity.x);
	Vec4V linVel31 = V4LoadA(&b31.linearVelocity.x);
	Vec4V angState30 = V4LoadA(&b30.angularVelocity.x);
	Vec4V angState31 = V4LoadA(&b31.angularVelocity.x);


	Vec4V linVel0T0, linVel0T1, linVel0T2;
	Vec4V linVel1T0, linVel1T1, linVel1T2;
	Vec4V angState0T0, angState0T1, angState0T2;
	Vec4V angState1T0, angState1T1, angState1T2;


	PX_TRANSPOSE_44_34(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2);
	PX_TRANSPOSE_44_34(linVel01, linVel11, linVel21, linVel31, linVel1T0, linVel1T1, linVel1T2);
	PX_TRANSPOSE_44_34(angState00, angState10, angState20, angState30, angState0T0, angState0T1, angState0T2);
	PX_TRANSPOSE_44_34(angState01, angState11, angState21, angState31, angState1T0, angState1T1, angState1T2);


	Vec4V linDelta00 = V4LoadA(&b00.deltaLinDt.x);
	Vec4V linDelta01 = V4LoadA(&b01.deltaLinDt.x);
	Vec4V angDelta00 = V4LoadA(&b00.deltaAngDt.x);
	Vec4V angDelta01 = V4LoadA(&b01.deltaAngDt.x);

	Vec4V linDelta10 = V4LoadA(&b10.deltaLinDt.x);
	Vec4V linDelta11 = V4LoadA(&b11.deltaLinDt.x);
	Vec4V angDelta10 = V4LoadA(&b10.deltaAngDt.x);
	Vec4V angDelta11 = V4LoadA(&b11.deltaAngDt.x);

	Vec4V linDelta20 = V4LoadA(&b20.deltaLinDt.x);
	Vec4V linDelta21 = V4LoadA(&b21.deltaLinDt.x);
	Vec4V angDelta20 = V4LoadA(&b20.deltaAngDt.x);
	Vec4V angDelta21 = V4LoadA(&b21.deltaAngDt.x);

	Vec4V linDelta30 = V4LoadA(&b30.deltaLinDt.x);
	Vec4V linDelta31 = V4LoadA(&b31.deltaLinDt.x);
	Vec4V angDelta30 = V4LoadA(&b30.deltaAngDt.x);
	Vec4V angDelta31 = V4LoadA(&b31.deltaAngDt.x);

	Vec4V linDelta0T0, linDelta0T1, linDelta0T2;
	Vec4V linDelta1T0, linDelta1T1, linDelta1T2;
	Vec4V angDelta0T0, angDelta0T1, angDelta0T2;
	Vec4V angDelta1T0, angDelta1T1, angDelta1T2;


	PX_TRANSPOSE_44_34(linDelta00, linDelta10, linDelta20, linDelta30, linDelta0T0, linDelta0T1, linDelta0T2);
	PX_TRANSPOSE_44_34(linDelta01, linDelta11, linDelta21, linDelta31, linDelta1T0, linDelta1T1, linDelta1T2);
	PX_TRANSPOSE_44_34(angDelta00, angDelta10, angDelta20, angDelta30, angDelta0T0, angDelta0T1, angDelta0T2);
	PX_TRANSPOSE_44_34(angDelta01, angDelta11, angDelta21, angDelta31, angDelta1T0, angDelta1T1, angDelta1T2);


	

	const SolverConstraint1DHeaderStep4* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeaderStep4*>(bPtr);
	SolverConstraint1DStep4* PX_RESTRICT base = reinterpret_cast<SolverConstraint1DStep4*>(bPtr + sizeof(SolverConstraint1DHeaderStep4));


	Vec4V invInertia00X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI00.sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia00Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI00.sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia00Z = Vec4V_From_Vec3V(V3LoadU(txI00.sqrtInvInertia.column2));

	Vec4V invInertia10X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI10.sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia10Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI10.sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia10Z = Vec4V_From_Vec3V(V3LoadU(txI10.sqrtInvInertia.column2));

	Vec4V invInertia20X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI20.sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia20Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI20.sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia20Z = Vec4V_From_Vec3V(V3LoadU(txI20.sqrtInvInertia.column2));

	Vec4V invInertia30X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI30.sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia30Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI30.sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia30Z = Vec4V_From_Vec3V(V3LoadU(txI30.sqrtInvInertia.column2));

	Vec4V invInertia01X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI01.sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia01Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI01.sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia01Z = Vec4V_From_Vec3V(V3LoadU(txI01.sqrtInvInertia.column2));

	Vec4V invInertia11X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI11.sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia11Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI11.sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia11Z = Vec4V_From_Vec3V(V3LoadU(txI11.sqrtInvInertia.column2));

	Vec4V invInertia21X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI21.sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia21Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI21.sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia21Z = Vec4V_From_Vec3V(V3LoadU(txI21.sqrtInvInertia.column2));

	Vec4V invInertia31X = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI31.sqrtInvInertia.column0));	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia31Y = Vec4V_From_Vec3V(V3LoadU_SafeReadW(txI31.sqrtInvInertia.column1));	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia31Z = Vec4V_From_Vec3V(V3LoadU(txI31.sqrtInvInertia.column2));

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

	const Vec4V invInertiaScale0 = header->angD0;
	const Vec4V invInertiaScale1 = header->angD1;

	//KS - todo - load this a bit quicker...
	Vec4V rot00 = V4LoadA(&txI00.deltaBody2World.q.x);
	Vec4V rot01 = V4LoadA(&txI01.deltaBody2World.q.x);
	Vec4V rot10 = V4LoadA(&txI10.deltaBody2World.q.x);
	Vec4V rot11 = V4LoadA(&txI11.deltaBody2World.q.x);
	Vec4V rot20 = V4LoadA(&txI20.deltaBody2World.q.x);
	Vec4V rot21 = V4LoadA(&txI21.deltaBody2World.q.x);
	Vec4V rot30 = V4LoadA(&txI30.deltaBody2World.q.x);
	Vec4V rot31 = V4LoadA(&txI31.deltaBody2World.q.x);

	Vec4V rot0X, rot0Y, rot0Z, rot0W;
	Vec4V rot1X, rot1Y, rot1Z, rot1W;

	PX_TRANSPOSE_44(rot00, rot10, rot20, rot30, rot0X, rot0Y, rot0Z, rot0W);
	PX_TRANSPOSE_44(rot01, rot11, rot21, rot31, rot1X, rot1Y, rot1Z, rot1W);

	Vec4V raX, raY, raZ;
	Vec4V rbX, rbY, rbZ;

	QuatRotate4(rot0X, rot0Y, rot0Z, rot0W, header->rAWorld[0], header->rAWorld[1], header->rAWorld[2], raX, raY, raZ);
	QuatRotate4(rot1X, rot1Y, rot1Z, rot1W, header->rBWorld[0], header->rBWorld[1], header->rBWorld[2], rbX, rbY, rbZ);


	const Vec4V raMotionX = V4Sub(V4Add(raX, linDelta0T0), header->rAWorld[0]);
	const Vec4V raMotionY = V4Sub(V4Add(raY, linDelta0T1), header->rAWorld[1]);
	const Vec4V raMotionZ = V4Sub(V4Add(raZ, linDelta0T2), header->rAWorld[2]);
	const Vec4V rbMotionX = V4Sub(V4Add(rbX, linDelta1T0), header->rBWorld[0]);
	const Vec4V rbMotionY = V4Sub(V4Add(rbY, linDelta1T1), header->rBWorld[1]);
	const Vec4V rbMotionZ = V4Sub(V4Add(rbZ, linDelta1T2), header->rBWorld[2]);

	const Vec4V mass0 = header->invMass0D0;
	const Vec4V mass1 = header->invMass1D1;

	const VecI32V orthoMask = I4Load(DY_SC_FLAG_ORTHO_TARGET);
	const VecI32V limitMask = I4Load(DY_SC_FLAG_INEQUALITY);
	const Vec4V zero = V4Zero();
	const Vec4V one = V4One();

	Vec4V error0 = V4Add(header->angOrthoError[0], 
		V4Sub(V4Dot3(header->angOrthoAxis0X[0], header->angOrthoAxis0Y[0], header->angOrthoAxis0Z[0], angDelta0T0, angDelta0T1, angDelta0T2),
			V4Dot3(header->angOrthoAxis1X[0], header->angOrthoAxis1Y[0], header->angOrthoAxis1Z[0], angDelta1T0, angDelta1T1, angDelta1T2)));

	Vec4V error1 = V4Add(header->angOrthoError[1],
		V4Sub(V4Dot3(header->angOrthoAxis0X[1], header->angOrthoAxis0Y[1], header->angOrthoAxis0Z[1], angDelta0T0, angDelta0T1, angDelta0T2),
			V4Dot3(header->angOrthoAxis1X[1], header->angOrthoAxis1Y[1], header->angOrthoAxis1Z[1], angDelta1T0, angDelta1T1, angDelta1T2)));

	Vec4V error2 = V4Add(header->angOrthoError[2],
		V4Sub(V4Dot3(header->angOrthoAxis0X[2], header->angOrthoAxis0Y[2], header->angOrthoAxis0Z[2], angDelta0T0, angDelta0T1, angDelta0T2),
			V4Dot3(header->angOrthoAxis1X[2], header->angOrthoAxis1Y[2], header->angOrthoAxis1Z[2], angDelta1T0, angDelta1T1, angDelta1T2)));

	for (PxU32 i = 0; i<header->count; ++i, base++)
	{
		Ps::prefetchLine(base + 1);
		SolverConstraint1DStep4& c = *base;

		const Vec4V cangVel0X = V4Add(c.ang0[0], V4NegMulSub(raZ, c.lin0[1], V4Mul(raY, c.lin0[2])));
		const Vec4V cangVel0Y = V4Add(c.ang0[1], V4NegMulSub(raX, c.lin0[2], V4Mul(raZ, c.lin0[0])));
		const Vec4V cangVel0Z = V4Add(c.ang0[2], V4NegMulSub(raY, c.lin0[0], V4Mul(raX, c.lin0[1])));

		const Vec4V cangVel1X = V4Add(c.ang1[0], V4NegMulSub(rbZ, c.lin1[1], V4Mul(rbY, c.lin1[2])));
		const Vec4V cangVel1Y = V4Add(c.ang1[1], V4NegMulSub(rbX, c.lin1[2], V4Mul(rbZ, c.lin1[0])));
		const Vec4V cangVel1Z = V4Add(c.ang1[2], V4NegMulSub(rbY, c.lin1[0], V4Mul(rbX, c.lin1[1])));

		const VecI32V flags = I4LoadA(reinterpret_cast<PxI32*>(c.flags));

		const BoolV useOrtho = VecI32V_IsEq(VecI32V_And(flags, orthoMask), orthoMask);


		const Vec4V angOrthoCoefficient = V4Sel(useOrtho, one, zero);


		Vec4V delAngVel0X = V4Mul(invInertia0X0, cangVel0X);
		Vec4V delAngVel0Y = V4Mul(invInertia0X1, cangVel0X);
		Vec4V delAngVel0Z = V4Mul(invInertia0X2, cangVel0X);

		delAngVel0X = V4MulAdd(invInertia0Y0, cangVel0Y, delAngVel0X);
		delAngVel0Y = V4MulAdd(invInertia0Y1, cangVel0Y, delAngVel0Y);
		delAngVel0Z = V4MulAdd(invInertia0Y2, cangVel0Y, delAngVel0Z);

		delAngVel0X = V4MulAdd(invInertia0Z0, cangVel0Z, delAngVel0X);
		delAngVel0Y = V4MulAdd(invInertia0Z1, cangVel0Z, delAngVel0Y);
		delAngVel0Z = V4MulAdd(invInertia0Z2, cangVel0Z, delAngVel0Z);

		Vec4V delAngVel1X = V4Mul(invInertia1X0, cangVel1X);
		Vec4V delAngVel1Y = V4Mul(invInertia1X1, cangVel1X);
		Vec4V delAngVel1Z = V4Mul(invInertia1X2, cangVel1X);

		delAngVel1X = V4MulAdd(invInertia1Y0, cangVel1Y, delAngVel1X);
		delAngVel1Y = V4MulAdd(invInertia1Y1, cangVel1Y, delAngVel1Y);
		delAngVel1Z = V4MulAdd(invInertia1Y2, cangVel1Y, delAngVel1Z);

		delAngVel1X = V4MulAdd(invInertia1Z0, cangVel1Z, delAngVel1X);
		delAngVel1Y = V4MulAdd(invInertia1Z1, cangVel1Z, delAngVel1Y);
		delAngVel1Z = V4MulAdd(invInertia1Z2, cangVel1Z, delAngVel1Z);

		Vec4V err = c.error;
		{
			const Vec4V proj0 = V4Mul(V4MulAdd(header->angOrthoAxis0X[0], delAngVel0X, V4MulAdd(header->angOrthoAxis0Y[0], delAngVel0Y,
				V4MulAdd(header->angOrthoAxis0Z[0], delAngVel0Z, V4MulAdd(header->angOrthoAxis1X[0], delAngVel1X,
					V4MulAdd(header->angOrthoAxis1Y[0], delAngVel1Y, V4Mul(header->angOrthoAxis1Z[0], delAngVel1Z)))))), header->angOrthoRecipResponse[0]);

			const Vec4V proj1 = V4Mul(V4MulAdd(header->angOrthoAxis0X[1], delAngVel0X, V4MulAdd(header->angOrthoAxis0Y[1], delAngVel0Y,
				V4MulAdd(header->angOrthoAxis0Z[1], delAngVel0Z, V4MulAdd(header->angOrthoAxis1X[1], delAngVel1X,
					V4MulAdd(header->angOrthoAxis1Y[1], delAngVel1Y, V4Mul(header->angOrthoAxis1Z[1], delAngVel1Z)))))), header->angOrthoRecipResponse[1]);

			const Vec4V proj2 = V4Mul(V4MulAdd(header->angOrthoAxis0X[2], delAngVel0X, V4MulAdd(header->angOrthoAxis0Y[2], delAngVel0Y,
				V4MulAdd(header->angOrthoAxis0Z[2], delAngVel0Z, V4MulAdd(header->angOrthoAxis1X[2], delAngVel1X,
					V4MulAdd(header->angOrthoAxis1Y[2], delAngVel1Y, V4Mul(header->angOrthoAxis1Z[2], delAngVel1Z)))))), header->angOrthoRecipResponse[2]);

			const Vec4V delta0X = V4MulAdd(header->angOrthoAxis0X[0], proj0, V4MulAdd(header->angOrthoAxis0X[1], proj1, V4Mul(header->angOrthoAxis0X[2], proj2)));
			const Vec4V delta0Y = V4MulAdd(header->angOrthoAxis0Y[0], proj0, V4MulAdd(header->angOrthoAxis0Y[1], proj1, V4Mul(header->angOrthoAxis0Y[2], proj2)));
			const Vec4V delta0Z = V4MulAdd(header->angOrthoAxis0Z[0], proj0, V4MulAdd(header->angOrthoAxis0Z[1], proj1, V4Mul(header->angOrthoAxis0Z[2], proj2)));

			const Vec4V delta1X = V4MulAdd(header->angOrthoAxis1X[0], proj0, V4MulAdd(header->angOrthoAxis1X[1], proj1, V4Mul(header->angOrthoAxis1X[2], proj2)));
			const Vec4V delta1Y = V4MulAdd(header->angOrthoAxis1Y[0], proj0, V4MulAdd(header->angOrthoAxis1Y[1], proj1, V4Mul(header->angOrthoAxis1Y[2], proj2)));
			const Vec4V delta1Z = V4MulAdd(header->angOrthoAxis1Z[0], proj0, V4MulAdd(header->angOrthoAxis1Z[1], proj1, V4Mul(header->angOrthoAxis1Z[2], proj2)));


			delAngVel0X = V4NegMulSub(delta0X, angOrthoCoefficient, delAngVel0X);
			delAngVel0Y = V4NegMulSub(delta0Y, angOrthoCoefficient, delAngVel0Y);
			delAngVel0Z = V4NegMulSub(delta0Z, angOrthoCoefficient, delAngVel0Z);

			delAngVel1X = V4NegMulSub(delta1X, angOrthoCoefficient, delAngVel1X);
			delAngVel1Y = V4NegMulSub(delta1Y, angOrthoCoefficient, delAngVel1Y);
			delAngVel1Z = V4NegMulSub(delta1Z, angOrthoCoefficient, delAngVel1Z);

			err = V4Sub(err, V4Mul(V4MulAdd(error0, proj0, V4MulAdd(error1, proj1, V4Mul(error2, proj2))), angOrthoCoefficient));
		}

		Vec4V ang0IX = V4Mul(invInertia0X0, delAngVel0X);
		Vec4V ang0IY = V4Mul(invInertia0X1, delAngVel0X);
		Vec4V ang0IZ = V4Mul(invInertia0X2, delAngVel0X);

		ang0IX = V4MulAdd(invInertia0Y0, delAngVel0Y, ang0IX);
		ang0IY = V4MulAdd(invInertia0Y1, delAngVel0Y, ang0IY);
		ang0IZ = V4MulAdd(invInertia0Y2, delAngVel0Y, ang0IZ);

		ang0IX = V4MulAdd(invInertia0Z0, delAngVel0Z, ang0IX);
		ang0IY = V4MulAdd(invInertia0Z1, delAngVel0Z, ang0IY);
		ang0IZ = V4MulAdd(invInertia0Z2, delAngVel0Z, ang0IZ);

		Vec4V ang1IX = V4Mul(invInertia1X0, delAngVel1X);
		Vec4V ang1IY = V4Mul(invInertia1X1, delAngVel1X);
		Vec4V ang1IZ = V4Mul(invInertia1X2, delAngVel1X);

		ang1IX = V4MulAdd(invInertia1Y0, delAngVel1Y, ang1IX);
		ang1IY = V4MulAdd(invInertia1Y1, delAngVel1Y, ang1IY);
		ang1IZ = V4MulAdd(invInertia1Y2, delAngVel1Y, ang1IZ);

		ang1IX = V4MulAdd(invInertia1Z0, delAngVel1Z, ang1IX);
		ang1IY = V4MulAdd(invInertia1Z1, delAngVel1Z, ang1IY);
		ang1IZ = V4MulAdd(invInertia1Z2, delAngVel1Z, ang1IZ);


		const Vec4V clinVel0X = c.lin0[0];
		const Vec4V clinVel0Y = c.lin0[1];
		const Vec4V clinVel0Z = c.lin0[2];

		const Vec4V clinVel1X = c.lin1[0];
		const Vec4V clinVel1Y = c.lin1[1];
		const Vec4V clinVel1Z = c.lin1[2];


		const Vec4V clinVel0X_ = c.lin0[0];
		const Vec4V clinVel0Y_ = c.lin0[1];
		const Vec4V clinVel0Z_ = c.lin0[2];

		const Vec4V clinVel1X_ = c.lin1[0];
		const Vec4V clinVel1Y_ = c.lin1[1];
		const Vec4V clinVel1Z_ = c.lin1[2];


		//KS - compute raXnI and effective mass. Unfortunately, the joints are noticeably less stable if we don't do this each 
		//iteration. It's skippable. If we do that, there's no need for the invInertiaTensors

		

		const Vec4V dotDelAngVel0 = V4MulAdd(delAngVel0X, delAngVel0X, V4MulAdd(delAngVel0Y, delAngVel0Y, V4Mul(delAngVel0Z, delAngVel0Z)));
		const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1X, delAngVel1X, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1Z, delAngVel1Z)));
		
		const Vec4V dotRbXnAngDelta0 = V4MulAdd(delAngVel0X, angDelta0T0, V4MulAdd(delAngVel0Y, angDelta0T1, V4Mul(delAngVel0Z, angDelta0T2)));
		const Vec4V dotRbXnAngDelta1 = V4MulAdd(delAngVel1X, angDelta1T0, V4MulAdd(delAngVel1Y, angDelta1T1, V4Mul(delAngVel1Z, angDelta1T2)));

		const Vec4V dotRaMotion = V4MulAdd(clinVel0X_, raMotionX, V4MulAdd(clinVel0Y_, raMotionY, V4Mul(clinVel0Z_, raMotionZ)));
		const Vec4V dotRbMotion = V4MulAdd(clinVel1X_, rbMotionX, V4MulAdd(clinVel1Y_, rbMotionY, V4Mul(clinVel1Z_, rbMotionZ)));

		const Vec4V deltaAng = V4Mul(c.angularErrorScale, V4Sub(dotRbXnAngDelta0, dotRbXnAngDelta1));
		const Vec4V error = V4NegScaleSub(c.velTarget, elapsedTime, V4Add(V4Add(err, V4Sub(dotRaMotion, dotRbMotion)), deltaAng));

		const Vec4V dotClinVel0 = V4MulAdd(clinVel0X, clinVel0X, V4MulAdd(clinVel0Y, clinVel0Y, V4Mul(clinVel0Z, clinVel0Z)));
		const Vec4V dotClinVel1 = V4MulAdd(clinVel1X, clinVel1X, V4MulAdd(clinVel1Y, clinVel1Y, V4Mul(clinVel1Z, clinVel1Z)));

		const Vec4V resp0 = V4MulAdd(mass0, dotClinVel0, V4Mul(invInertiaScale0, dotDelAngVel0));
		const Vec4V resp1 = V4MulAdd(mass1, dotClinVel1, V4Mul(invInertiaScale1, dotDelAngVel1));
		const Vec4V response = V4Add(resp0, resp1);
		const Vec4V recipResponse = V4Sel(V4IsGrtr(response, V4Zero()), V4Recip(response), V4Zero());
		

		const Vec4V vMul = V4Mul(recipResponse, c.velMultiplier);

		const BoolV isLimitConstraint = VecI32V_IsEq(VecI32V_And(flags, limitMask), limitMask);

		const Vec4V minBias = V4Sel(isLimitConstraint, V4Neg(Vec4V_From_FloatV(FMax())), V4Neg(c.maxBias));
		const Vec4V unclampedBias = V4Mul(error, c.biasScale);
		const Vec4V bias = V4Clamp(unclampedBias, minBias, c.maxBias);

		const Vec4V constant = V4Mul(recipResponse, V4Add(bias, c.velTarget));

		const Vec4V normalVel0 = V4MulAdd(clinVel0X_, linVel0T0, V4MulAdd(clinVel0Y_, linVel0T1, V4Mul(clinVel0Z_, linVel0T2)));
		const Vec4V normalVel1 = V4MulAdd(clinVel1X_, linVel1T0, V4MulAdd(clinVel1Y_, linVel1T1, V4Mul(clinVel1Z_, linVel1T2)));

		const Vec4V angVel0 = V4MulAdd(delAngVel0X, angState0T0, V4MulAdd(delAngVel0Y, angState0T1, V4Mul(delAngVel0Z, angState0T2)));
		const Vec4V angVel1 = V4MulAdd(delAngVel1X, angState1T0, V4MulAdd(delAngVel1Y, angState1T1, V4Mul(delAngVel1Z, angState1T2)));

		const Vec4V normalVel = V4Add(V4Sub(normalVel0, normalVel1), V4Sub(angVel0, angVel1));


		const Vec4V unclampedForce = V4MulAdd(c.impulseMultiplier, c.appliedForce, V4MulAdd(vMul, normalVel, constant));
		const Vec4V clampedForce = V4Clamp(unclampedForce, c.minImpulse, c.maxImpulse);
		const Vec4V deltaF = V4Sub(clampedForce, c.appliedForce);

		c.appliedForce = clampedForce;

		const Vec4V deltaFIM0 = V4Mul(deltaF, mass0);
		const Vec4V deltaFIM1 = V4Mul(deltaF, mass1);
		const Vec4V angDetaF0 = V4Mul(deltaF, invInertiaScale0);
		const Vec4V angDetaF1 = V4Mul(deltaF, invInertiaScale1);


		linVel0T0 = V4MulAdd(clinVel0X_, deltaFIM0, linVel0T0);
		linVel1T0 = V4NegMulSub(clinVel1X_, deltaFIM1, linVel1T0);
		angState0T0 = V4MulAdd(delAngVel0X, angDetaF0, angState0T0);
		angState1T0 = V4NegMulSub(delAngVel1X, angDetaF1, angState1T0);

		linVel0T1 = V4MulAdd(clinVel0Y_, deltaFIM0, linVel0T1);
		linVel1T1 = V4NegMulSub(clinVel1Y_, deltaFIM1, linVel1T1);
		angState0T1 = V4MulAdd(delAngVel0Y, angDetaF0, angState0T1);
		angState1T1 = V4NegMulSub(delAngVel1Y, angDetaF1, angState1T1);

		linVel0T2 = V4MulAdd(clinVel0Z_, deltaFIM0, linVel0T2);
		linVel1T2 = V4NegMulSub(clinVel1Z_, deltaFIM1, linVel1T2);
		angState0T2 = V4MulAdd(delAngVel0Z, angDetaF0, angState0T2);
		angState1T2 = V4NegMulSub(delAngVel1Z, angDetaF1, angState1T2);

	}

	PX_TRANSPOSE_34_44(linVel0T0, linVel0T1, linVel0T2, linVel00, linVel10, linVel20, linVel30);
	PX_TRANSPOSE_34_44(linVel1T0, linVel1T1, linVel1T2, linVel01, linVel11, linVel21, linVel31);
	PX_TRANSPOSE_34_44(angState0T0, angState0T1, angState0T2, angState00, angState10, angState20, angState30);
	PX_TRANSPOSE_34_44(angState1T0, angState1T1, angState1T2, angState01, angState11, angState21, angState31);

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularVelocity.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularVelocity.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularVelocity.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularVelocity.isFinite());

	PX_ASSERT(b01.linearVelocity.isFinite());
	PX_ASSERT(b01.angularVelocity.isFinite());
	PX_ASSERT(b11.linearVelocity.isFinite());
	PX_ASSERT(b11.angularVelocity.isFinite());
	PX_ASSERT(b21.linearVelocity.isFinite());
	PX_ASSERT(b21.angularVelocity.isFinite());
	PX_ASSERT(b31.linearVelocity.isFinite());
	PX_ASSERT(b31.angularVelocity.isFinite());

	// Write back
	V4StoreA(linVel00, &b00.linearVelocity.x);
	V4StoreA(angState00, &b00.angularVelocity.x);
	V4StoreA(linVel10, &b10.linearVelocity.x);
	V4StoreA(angState10, &b10.angularVelocity.x);
	V4StoreA(linVel20, &b20.linearVelocity.x);
	V4StoreA(angState20, &b20.angularVelocity.x);
	V4StoreA(linVel30, &b30.linearVelocity.x);
	V4StoreA(angState30, &b30.angularVelocity.x);

	V4StoreA(linVel01, &b01.linearVelocity.x);
	V4StoreA(angState01, &b01.angularVelocity.x);
	V4StoreA(linVel11, &b11.linearVelocity.x);
	V4StoreA(angState11, &b11.angularVelocity.x);
	V4StoreA(linVel21, &b21.linearVelocity.x);
	V4StoreA(angState21, &b21.angularVelocity.x);
	V4StoreA(linVel31, &b31.linearVelocity.x);
	V4StoreA(angState31, &b31.angularVelocity.x);

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularVelocity.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularVelocity.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularVelocity.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularVelocity.isFinite());

	PX_ASSERT(b01.linearVelocity.isFinite());
	PX_ASSERT(b01.angularVelocity.isFinite());
	PX_ASSERT(b11.linearVelocity.isFinite());
	PX_ASSERT(b11.angularVelocity.isFinite());
	PX_ASSERT(b21.linearVelocity.isFinite());
	PX_ASSERT(b21.angularVelocity.isFinite());
	PX_ASSERT(b31.linearVelocity.isFinite());
	PX_ASSERT(b31.angularVelocity.isFinite());
}


void solve1D4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc, const PxTGSSolverBodyTxInertia* const txInertias,
	const PxReal elapsedTime)
{
	solve1DStep4(desc + hdr.mStartIndex, txInertias, elapsedTime);
}

void writeBack1D4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc)
{
	PX_UNUSED(hdr);
	ConstraintWriteback* writeback0 = reinterpret_cast<ConstraintWriteback*>(desc[hdr.mStartIndex].writeBack);
	ConstraintWriteback* writeback1 = reinterpret_cast<ConstraintWriteback*>(desc[hdr.mStartIndex + 1].writeBack);
	ConstraintWriteback* writeback2 = reinterpret_cast<ConstraintWriteback*>(desc[hdr.mStartIndex + 2].writeBack);
	ConstraintWriteback* writeback3 = reinterpret_cast<ConstraintWriteback*>(desc[hdr.mStartIndex + 3].writeBack);

	if (writeback0 || writeback1 || writeback2 || writeback3)
	{
		SolverConstraint1DHeaderStep4* header = reinterpret_cast<SolverConstraint1DHeaderStep4*>(desc[hdr.mStartIndex].constraint);
		SolverConstraint1DStep4* base = reinterpret_cast<SolverConstraint1DStep4*>(desc[hdr.mStartIndex].constraint + sizeof(SolverConstraint1DHeaderStep4));

		const Vec4V zero = V4Zero();
		Vec4V linX(zero), linY(zero), linZ(zero);
		Vec4V angX(zero), angY(zero), angZ(zero);

		for (PxU32 i = 0; i<header->count; i++)
		{
			const SolverConstraint1DStep4* c = base;

			//Load in flags
			const VecI32V flags = I4LoadU(reinterpret_cast<const PxI32*>(&c->flags[0]));
			//Work out masks
			const VecI32V mask = I4Load(DY_SC_FLAG_OUTPUT_FORCE);

			const VecI32V masked = VecI32V_And(flags, mask);
			const BoolV isEq = VecI32V_IsEq(masked, mask);

			const Vec4V appliedForce = V4Sel(isEq, c->appliedForce, zero);

			linX = V4MulAdd(c->lin0[0], appliedForce, linX);
			linY = V4MulAdd(c->lin0[1], appliedForce, linY);
			linZ = V4MulAdd(c->lin0[2], appliedForce, linZ);

			angX = V4MulAdd(c->ang0[0], appliedForce, angX);
			angY = V4MulAdd(c->ang0[1], appliedForce, angY);
			angZ = V4MulAdd(c->ang0[2], appliedForce, angZ);

			base++;
		}

		//We need to do the cross product now

		angX = V4Sub(angX, V4NegMulSub(header->body0WorkOffset[0], linY, V4Mul(header->body0WorkOffset[1], linZ)));
		angY = V4Sub(angY, V4NegMulSub(header->body0WorkOffset[1], linZ, V4Mul(header->body0WorkOffset[2], linX)));
		angZ = V4Sub(angZ, V4NegMulSub(header->body0WorkOffset[2], linX, V4Mul(header->body0WorkOffset[0], linY)));

		const Vec4V linLenSq = V4MulAdd(linZ, linZ, V4MulAdd(linY, linY, V4Mul(linX, linX)));
		const Vec4V angLenSq = V4MulAdd(angZ, angZ, V4MulAdd(angY, angY, V4Mul(angX, angX)));

		const Vec4V linLen = V4Sqrt(linLenSq);
		const Vec4V angLen = V4Sqrt(angLenSq);

		const BoolV broken = BOr(V4IsGrtr(linLen, header->linBreakImpulse), V4IsGrtr(angLen, header->angBreakImpulse));

		PX_ALIGN(16, PxU32 iBroken[4]);
		BStoreA(broken, iBroken);

		Vec4V lin0, lin1, lin2, lin3;
		Vec4V ang0, ang1, ang2, ang3;

		PX_TRANSPOSE_34_44(linX, linY, linZ, lin0, lin1, lin2, lin3);
		PX_TRANSPOSE_34_44(angX, angY, angZ, ang0, ang1, ang2, ang3);

		if (writeback0)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin0), writeback0->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang0), writeback0->angularImpulse);
			writeback0->broken = header->breakable[0] ? PxU32(iBroken[0] != 0) : 0;
		}
		if (writeback1)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin1), writeback1->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang1), writeback1->angularImpulse);
			writeback1->broken = header->breakable[1] ? PxU32(iBroken[1] != 0) : 0;
		}
		if (writeback2)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin2), writeback2->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang2), writeback2->angularImpulse);
			writeback2->broken = header->breakable[2] ? PxU32(iBroken[2] != 0) : 0;
		}
		if (writeback3)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin3), writeback3->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang3), writeback3->angularImpulse);
			writeback3->broken = header->breakable[3] ? PxU32(iBroken[3] != 0) : 0;
		}
	}
}


void concludeContact4_Block(const PxTGSSolverConstraintDesc* PX_RESTRICT desc)
{
	PX_UNUSED(desc);
	//const PxU8* PX_RESTRICT last = desc[0].constraint + getConstraintLength(desc[0]);

	////hopefully pointer aliasing doesn't bite.
	//PxU8* PX_RESTRICT currPtr = desc[0].constraint;

	//const Vec4V zero = V4Zero();

	////const PxU8 type = *desc[0].constraint;
	//const PxU32 contactSize = sizeof(SolverContactPointStepBlock);
	//const PxU32 frictionSize = sizeof(SolverContactFrictionStepBlock);

	//while ((currPtr < last))
	//{
	//	SolverContactHeaderStepBlock* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStepBlock*>(currPtr);

	//	currPtr = reinterpret_cast<PxU8*>(hdr + 1);

	//	const PxU32 numNormalConstr = hdr->numNormalConstr;
	//	const PxU32	numFrictionConstr = hdr->numFrictionConstr;

	//	//Applied forces
	//	currPtr += sizeof(Vec4V)*numNormalConstr;

	//	//SolverContactPointStepBlock* PX_RESTRICT contacts = reinterpret_cast<SolverContactPointStepBlock*>(currPtr);
	//	currPtr += (numNormalConstr * contactSize);

	//	bool hasMaxImpulse = (hdr->flag & SolverContactHeader4::eHAS_MAX_IMPULSE) != 0;

	//	if (hasMaxImpulse)
	//		currPtr += sizeof(Vec4V) * numNormalConstr;

	//	currPtr += sizeof(Vec4V)*numFrictionConstr;

	//	SolverContactFrictionStepBlock* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionStepBlock*>(currPtr);
	//	currPtr += (numFrictionConstr * frictionSize);

	//	/*for (PxU32 i = 0; i<numNormalConstr; i++)
	//	{
	//		contacts[i].biasCoefficient = V4Sel(V4IsGrtr(contacts[i].separation, zero), contacts[i].biasCoefficient, zero);
	//	}*/

	//	for (PxU32 i = 0; i<numFrictionConstr; i++)
	//	{
	//		frictions[i].biasCoefficient = zero;
	//	}
	//}
}

void conclude1DStep4(const PxTGSSolverConstraintDesc* PX_RESTRICT desc)
{
	PxU8* PX_RESTRICT bPtr = desc->constraint;
	if (bPtr == NULL)
		return;


	const SolverConstraint1DHeaderStep4* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeaderStep4*>(bPtr);
	SolverConstraint1DStep4* PX_RESTRICT base = reinterpret_cast<SolverConstraint1DStep4*>(bPtr + sizeof(SolverConstraint1DHeaderStep4));

	const VecI32V mask = I4Load(DY_SC_FLAG_KEEP_BIAS);
	const Vec4V zero = V4Zero();

	for (PxU32 i = 0; i<header->count; ++i, base++)
	{
		Ps::prefetchLine(base + 1);
		SolverConstraint1DStep4& c = *base;

		const VecI32V flags = I4LoadA(reinterpret_cast<PxI32*>(c.flags));

		const BoolV keepBias = VecI32V_IsEq(VecI32V_And(flags, mask), mask);

		c.biasScale = V4Sel(keepBias, c.biasScale, zero);
	}
}


void solveConcludeContact4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc,
	const PxReal elapsedTime)
{
	solveContact4_Block(desc + hdr.mStartIndex, true, -PX_MAX_F32, elapsedTime);
	concludeContact4_Block(desc + hdr.mStartIndex);
}

void solveConclude1D4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc, const PxTGSSolverBodyTxInertia* const txInertias,
	const PxReal elapsedTime)
{
	solve1DStep4(desc + hdr.mStartIndex, txInertias, elapsedTime);
	conclude1DStep4(desc + hdr.mStartIndex);
}






}
}

