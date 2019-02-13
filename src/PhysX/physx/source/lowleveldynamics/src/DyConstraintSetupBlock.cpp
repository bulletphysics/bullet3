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


#include "foundation/PxMemory.h"
#include "DyConstraintPrep.h"
#include "PxsRigidBody.h"
#include "DySolverConstraint1D.h"
#include "DySolverConstraint1D4.h"
#include "PsSort.h"
#include "PxcConstraintBlockStream.h"
#include "DyArticulationContactPrep.h"
#include "PsFoundation.h"
namespace physx
{

namespace Dy
{

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


namespace
{
void setConstants(PxReal& constant, PxReal& unbiasedConstant, PxReal& velMultiplier, PxReal& impulseMultiplier,
				  const Px1DConstraint& c, PxReal unitResponse, PxReal minRowResponse, PxReal erp, PxReal dt, PxReal recipdt,
				  const PxSolverBodyData& b0, const PxSolverBodyData& b1, const bool finished)
{
	if(finished)
	{
		constant = 0.f;
		unbiasedConstant = 0.f;
		velMultiplier = 0.f;
		impulseMultiplier = 0.f;
		return;
	}
	PxReal nv = needsNormalVel(c) ? b0.projectVelocity(c.linear0, c.angular0) - b1.projectVelocity(c.linear1, c.angular1)
								  : 0;
	
	setSolverConstants(constant, unbiasedConstant, velMultiplier, impulseMultiplier, 
					   c, nv, unitResponse, minRowResponse, erp, dt, recipdt);
}
}

SolverConstraintPrepState::Enum setupSolverConstraint4
		(PxSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
		const PxReal dt, const PxReal recipdt, PxU32& totalRows,
		PxConstraintAllocator& allocator, PxU32 maxRows);

SolverConstraintPrepState::Enum setupSolverConstraint4
(SolverConstraintShaderPrepDesc* PX_RESTRICT constraintShaderDescs,
PxSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
const PxReal dt, const PxReal recipdt, PxU32& totalRows,
PxConstraintAllocator& allocator)

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
		PxSolverConstraintPrepDesc& desc = constraintDescs[a];

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

		PxVec3 ra, rb;

		PxU32 constraintCount = (*shaderDesc.solverPrep)(rows,
			desc.body0WorldOffset,
			MAX_CONSTRAINT_ROWS,
			desc.mInvMassScales,
			shaderDesc.constantBlock,
			desc.bodyFrame0, desc.bodyFrame1, desc.extendedLimits, ra, rb);

		preppedIndex = MAX_CONSTRAINT_ROWS - constraintCount;

		maxRows = PxMax(constraintCount, maxRows);

		if (constraintCount == 0)
			return SolverConstraintPrepState::eUNBATCHABLE;

		desc.rows = rows;
		desc.numRows = constraintCount;
		numRows += constraintCount;
	}

	return setupSolverConstraint4(constraintDescs, dt, recipdt, totalRows, allocator, maxRows);
}

SolverConstraintPrepState::Enum setupSolverConstraint4
(PxSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
const PxReal dt, const PxReal recipdt, PxU32& totalRows,
PxConstraintAllocator& allocator, PxU32 maxRows)
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
		PxSolverConstraintPrepDesc& desc = constraintDescs[a];
		Px1DConstraint** sorted = allSorted + numRows;

		preprocessRows(sorted, desc.rows, angSqrtInvInertia0 + numRows, angSqrtInvInertia1 + numRows, desc.numRows, 
			desc.data0->sqrtInvInertia, desc.data1->sqrtInvInertia, desc.data0->invMass, desc.data1->invMass, 
			desc.mInvMassScales, desc.disablePreprocessing, desc.improvedSlerp);

		numRows += desc.numRows;
	}


	PxU32 stride = sizeof(SolverConstraint1DDynamic4);

	
	const PxU32 constraintLength = sizeof(SolverConstraint1DHeader4) + stride * maxRows;

	//KS - +16 is for the constraint progress counter, which needs to be the last element in the constraint (so that we
	//know SPU DMAs have completed)
	PxU8* ptr = allocator.reserveConstraintData(constraintLength + 16u);
	if(NULL == ptr || (reinterpret_cast<PxU8*>(-1))==ptr)
	{
		for(PxU32 a = 0; a < 4; ++a)
		{
			PxSolverConstraintPrepDesc& desc = constraintDescs[a];
			desc.desc->constraint = NULL;
			setConstraintLength(*desc.desc, 0);
			desc.desc->writeBack = desc.writeback;
		}

		if(NULL==ptr)
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
			ptr=NULL;
			return SolverConstraintPrepState::eOUT_OF_MEMORY;
		}
	}
	//desc.constraint = ptr;

	totalRows = numRows;

	for(PxU32 a = 0; a < 4; ++a)
	{
		PxSolverConstraintPrepDesc& desc = constraintDescs[a];
		desc.desc->constraint = ptr;
		setConstraintLength(*desc.desc, constraintLength);
		desc.desc->writeBack = desc.writeback;
	}

	const PxReal erp[4] = { 1.0f, 1.0f, 1.0f, 1.0f};
	//OK, now we build all 4 constraints into a single set of rows

	{
		PxU8* currPtr = ptr;
		SolverConstraint1DHeader4* header = reinterpret_cast<SolverConstraint1DHeader4*>(currPtr);
		currPtr += sizeof(SolverConstraint1DHeader4);

		const PxSolverBodyData& bd00 = *constraintDescs[0].data0;
		const PxSolverBodyData& bd01 = *constraintDescs[1].data0;
		const PxSolverBodyData& bd02 = *constraintDescs[2].data0;
		const PxSolverBodyData& bd03 = *constraintDescs[3].data0;

		const PxSolverBodyData& bd10 = *constraintDescs[0].data1;
		const PxSolverBodyData& bd11 = *constraintDescs[1].data1;
		const PxSolverBodyData& bd12 = *constraintDescs[2].data1;
		const PxSolverBodyData& bd13 = *constraintDescs[3].data1;

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

		//Velocities
		Vec4V linVel00 = V4LoadA(&bd00.linearVelocity.x);
		Vec4V linVel01 = V4LoadA(&bd10.linearVelocity.x);
		Vec4V angVel00 = V4LoadA(&bd00.angularVelocity.x);
		Vec4V angVel01 = V4LoadA(&bd10.angularVelocity.x);

		Vec4V linVel10 = V4LoadA(&bd01.linearVelocity.x);
		Vec4V linVel11 = V4LoadA(&bd11.linearVelocity.x);
		Vec4V angVel10 = V4LoadA(&bd01.angularVelocity.x);
		Vec4V angVel11 = V4LoadA(&bd11.angularVelocity.x);

		Vec4V linVel20 = V4LoadA(&bd02.linearVelocity.x);
		Vec4V linVel21 = V4LoadA(&bd12.linearVelocity.x);
		Vec4V angVel20 = V4LoadA(&bd02.angularVelocity.x);
		Vec4V angVel21 = V4LoadA(&bd12.angularVelocity.x);

		Vec4V linVel30 = V4LoadA(&bd03.linearVelocity.x);
		Vec4V linVel31 = V4LoadA(&bd13.linearVelocity.x);
		Vec4V angVel30 = V4LoadA(&bd03.angularVelocity.x);
		Vec4V angVel31 = V4LoadA(&bd13.angularVelocity.x);


		Vec4V linVel0T0, linVel0T1, linVel0T2;
		Vec4V linVel1T0, linVel1T1, linVel1T2;
		Vec4V angVel0T0, angVel0T1, angVel0T2;
		Vec4V angVel1T0, angVel1T1, angVel1T2;


		PX_TRANSPOSE_44_34(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2);
		PX_TRANSPOSE_44_34(linVel01, linVel11, linVel21, linVel31, linVel1T0, linVel1T1, linVel1T2);
		PX_TRANSPOSE_44_34(angVel00, angVel10, angVel20, angVel30, angVel0T0, angVel0T1, angVel0T2);
		PX_TRANSPOSE_44_34(angVel01, angVel11, angVel21, angVel31, angVel1T0, angVel1T1, angVel1T2);



		//body world offsets
		Vec4V workOffset0 = Vec4V_From_Vec3V(V3LoadU(constraintDescs[0].body0WorldOffset));
		Vec4V workOffset1 = Vec4V_From_Vec3V(V3LoadU(constraintDescs[1].body0WorldOffset));
		Vec4V workOffset2 = Vec4V_From_Vec3V(V3LoadU(constraintDescs[2].body0WorldOffset));
		Vec4V workOffset3 = Vec4V_From_Vec3V(V3LoadU(constraintDescs[3].body0WorldOffset));

		Vec4V workOffsetX, workOffsetY, workOffsetZ;

		PX_TRANSPOSE_44_34(workOffset0, workOffset1, workOffset2, workOffset3, workOffsetX, workOffsetY, workOffsetZ);

		const FloatV dtV = FLoad(dt);
		Vec4V linBreakForce = V4LoadXYZW(constraintDescs[0].linBreakForce, constraintDescs[1].linBreakForce,
			constraintDescs[2].linBreakForce, constraintDescs[3].linBreakForce);
		Vec4V angBreakForce = V4LoadXYZW(constraintDescs[0].angBreakForce, constraintDescs[1].angBreakForce,
			constraintDescs[2].angBreakForce, constraintDescs[3].angBreakForce);

		
		header->break0 = PxU8((constraintDescs[0].linBreakForce != PX_MAX_F32) || (constraintDescs[0].angBreakForce != PX_MAX_F32));
		header->break1 = PxU8((constraintDescs[1].linBreakForce != PX_MAX_F32) || (constraintDescs[1].angBreakForce != PX_MAX_F32));
		header->break2 = PxU8((constraintDescs[2].linBreakForce != PX_MAX_F32) || (constraintDescs[2].angBreakForce != PX_MAX_F32));
		header->break3 = PxU8((constraintDescs[3].linBreakForce != PX_MAX_F32) || (constraintDescs[3].angBreakForce != PX_MAX_F32));


		//OK, I think that's everything loaded in

		header->invMass0D0 = invMass0;
		header->invMass1D1 = invMass1;
		header->angD0 = invInertiaScale0;
		header->angD1 = invInertiaScale1;
		header->body0WorkOffsetX = workOffsetX;
		header->body0WorkOffsetY = workOffsetY;
		header->body0WorkOffsetZ = workOffsetZ;

		header->count = maxRows;
		header->type = DY_SC_TYPE_BLOCK_1D;
		header->linBreakImpulse = V4Scale(linBreakForce, dtV);
		header->angBreakImpulse = V4Scale(angBreakForce, dtV);
		header->count0 = Ps::to8(constraintDescs[0].numRows);
		header->count1 = Ps::to8(constraintDescs[1].numRows);
		header->count2 = Ps::to8(constraintDescs[2].numRows);
		header->count3 = Ps::to8(constraintDescs[3].numRows);

		//Now we loop over the constraints and build the results...

		PxU32 index0 = 0;
		PxU32 endIndex0 = constraintDescs[0].numRows - 1;
		PxU32 index1 = startIndex[1];
		PxU32 endIndex1 = index1 + constraintDescs[1].numRows - 1;
		PxU32 index2 = startIndex[2];
		PxU32 endIndex2 = index2 + constraintDescs[2].numRows - 1;
		PxU32 index3 = startIndex[3];
		PxU32 endIndex3 = index3 + constraintDescs[3].numRows - 1;

		const FloatV one = FOne();

		for(PxU32 a = 0; a < maxRows; ++a)
		{	
			SolverConstraint1DDynamic4* c = reinterpret_cast<SolverConstraint1DDynamic4*>(currPtr);
			currPtr += stride;

			Px1DConstraint* con0 = allSorted[index0];
			Px1DConstraint* con1 = allSorted[index1];
			Px1DConstraint* con2 = allSorted[index2];
			Px1DConstraint* con3 = allSorted[index3];

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

			Vec4V driveScale = V4Splat(one);
			if (con0->flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && constraintDescs[0].driveLimitsAreForces)
				driveScale = V4SetX(driveScale, FMin(one, dtV));
			if (con1->flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && constraintDescs[1].driveLimitsAreForces)
				driveScale = V4SetY(driveScale, FMin(one, dtV));
			if (con2->flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && constraintDescs[2].driveLimitsAreForces)
				driveScale = V4SetZ(driveScale, FMin(one, dtV));
			if (con3->flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && constraintDescs[3].driveLimitsAreForces)
				driveScale = V4SetW(driveScale, FMin(one, dtV));


			Vec4V clin00 = V4LoadA(&con0->linear0.x);
			Vec4V clin01 = V4LoadA(&con1->linear0.x);
			Vec4V clin02 = V4LoadA(&con2->linear0.x);
			Vec4V clin03 = V4LoadA(&con3->linear0.x);

			Vec4V cang00 = V4LoadA(&con0->angular0.x);
			Vec4V cang01 = V4LoadA(&con1->angular0.x);
			Vec4V cang02 = V4LoadA(&con2->angular0.x);
			Vec4V cang03 = V4LoadA(&con3->angular0.x);

			Vec4V clin0X, clin0Y, clin0Z;
			Vec4V cang0X, cang0Y, cang0Z;
			
			PX_TRANSPOSE_44_34(clin00, clin01, clin02, clin03, clin0X, clin0Y, clin0Z);
			PX_TRANSPOSE_44_34(cang00, cang01, cang02, cang03, cang0X, cang0Y, cang0Z);
			
			const Vec4V maxImpulse = V4LoadXYZW(con0->maxImpulse, con1->maxImpulse, con2->maxImpulse, con3->maxImpulse);
			const Vec4V minImpulse = V4LoadXYZW(con0->minImpulse, con1->minImpulse, con2->minImpulse, con3->minImpulse);

			Vec4V angDelta0X, angDelta0Y, angDelta0Z;

			PX_TRANSPOSE_44_34(cangDelta00, cangDelta01, cangDelta02, cangDelta03, angDelta0X, angDelta0Y, angDelta0Z);

			c->flags[0] = 0;
			c->flags[1] = 0;
			c->flags[2] = 0;
			c->flags[3] = 0;

			c->lin0X = clin0X;
			c->lin0Y = clin0Y;
			c->lin0Z = clin0Z;
			c->ang0X = angDelta0X;
			c->ang0Y = angDelta0Y;
			c->ang0Z = angDelta0Z;
			c->ang0WritebackX = cang0X;
			c->ang0WritebackY = cang0Y;
			c->ang0WritebackZ = cang0Z;

			c->minImpulse = V4Mul(minImpulse, driveScale);
			c->maxImpulse = V4Mul(maxImpulse, driveScale);
			c->appliedForce = zero;

			const Vec4V lin0MagSq = V4MulAdd(clin0Z, clin0Z, V4MulAdd(clin0Y, clin0Y, V4Mul(clin0X, clin0X)));
			const Vec4V cang0DotAngDelta = V4MulAdd(angDelta0Z, angDelta0Z, V4MulAdd(angDelta0Y, angDelta0Y, V4Mul(angDelta0X, angDelta0X)));
			c->flags[0] = 0;
			c->flags[1] = 0;
			c->flags[2] = 0;
			c->flags[3] = 0;

			Vec4V unitResponse = V4MulAdd(lin0MagSq, invMass0, V4Mul(cang0DotAngDelta, invInertiaScale0));

			Vec4V clin10 = V4LoadA(&con0->linear1.x);
			Vec4V clin11 = V4LoadA(&con1->linear1.x);
			Vec4V clin12 = V4LoadA(&con2->linear1.x);
			Vec4V clin13 = V4LoadA(&con3->linear1.x);

			Vec4V cang10 = V4LoadA(&con0->angular1.x);
			Vec4V cang11 = V4LoadA(&con1->angular1.x);
			Vec4V cang12 = V4LoadA(&con2->angular1.x);
			Vec4V cang13 = V4LoadA(&con3->angular1.x);

			Vec4V clin1X, clin1Y, clin1Z;
			Vec4V cang1X, cang1Y, cang1Z;
			PX_TRANSPOSE_44_34(clin10, clin11, clin12, clin13, clin1X, clin1Y, clin1Z);
			PX_TRANSPOSE_44_34(cang10, cang11, cang12, cang13, cang1X, cang1Y, cang1Z);

			Vec4V angDelta1X, angDelta1Y, angDelta1Z;

			PX_TRANSPOSE_44_34(cangDelta10, cangDelta11, cangDelta12, cangDelta13, angDelta1X, angDelta1Y, angDelta1Z);

			const Vec4V lin1MagSq = V4MulAdd(clin1Z, clin1Z, V4MulAdd(clin1Y, clin1Y, V4Mul(clin1X, clin1X)));
			const Vec4V cang1DotAngDelta = V4MulAdd(angDelta1Z, angDelta1Z, V4MulAdd(angDelta1Y, angDelta1Y, V4Mul(angDelta1X, angDelta1X)));

			c->lin1X = clin1X;
			c->lin1Y = clin1Y;
			c->lin1Z = clin1Z;

			c->ang1X = angDelta1X;
			c->ang1Y = angDelta1Y;
			c->ang1Z = angDelta1Z;

			unitResponse = V4Add(unitResponse, V4MulAdd(lin1MagSq, invMass1, V4Mul(cang1DotAngDelta, invInertiaScale1)));

			Vec4V linProj0(V4Mul(clin0X, linVel0T0));
			Vec4V linProj1(V4Mul(clin1X, linVel1T0));
			Vec4V angProj0(V4Mul(cang0X, angVel0T0));
			Vec4V angProj1(V4Mul(cang1X, angVel1T0));

			linProj0 = V4MulAdd(clin0Y, linVel0T1, linProj0);
			linProj1 = V4MulAdd(clin1Y, linVel1T1, linProj1);
			angProj0 = V4MulAdd(cang0Y, angVel0T1, angProj0);
			angProj1 = V4MulAdd(cang1Y, angVel1T1, angProj1);
			
			linProj0 = V4MulAdd(clin0Z, linVel0T2, linProj0);
			linProj1 = V4MulAdd(clin1Z, linVel1T2, linProj1);
			angProj0 = V4MulAdd(cang0Z, angVel0T2, angProj0);
			angProj1 = V4MulAdd(cang1Z, angVel1T2, angProj1);

			const Vec4V projectVel0 = V4Add(linProj0, angProj0);
			const Vec4V projectVel1 = V4Add(linProj1, angProj1);
			
			const Vec4V normalVel = V4Sub(projectVel0, projectVel1);


			{
				const PxVec4& ur				= reinterpret_cast<const PxVec4&>(unitResponse);
				PxVec4& cConstant				= reinterpret_cast<PxVec4&>(c->constant);
				PxVec4& cUnbiasedConstant		= reinterpret_cast<PxVec4&>(c->unbiasedConstant);
				PxVec4& cVelMultiplier			= reinterpret_cast<PxVec4&>(c->velMultiplier);
				PxVec4& cImpulseMultiplier		= reinterpret_cast<PxVec4&>(c->impulseMultiplier);

				setConstants(cConstant.x, cUnbiasedConstant.x, cVelMultiplier.x, cImpulseMultiplier.x, 
							 *con0, ur.x, constraintDescs[0].minResponseThreshold, erp[0], dt, recipdt, 
							 *constraintDescs[0].data0, *constraintDescs[0].data1, a >= constraintDescs[0].numRows);

				setConstants(cConstant.y, cUnbiasedConstant.y, cVelMultiplier.y, cImpulseMultiplier.y, 
							 *con1, ur.y, constraintDescs[1].minResponseThreshold, erp[1], dt, recipdt, 
							 *constraintDescs[1].data0, *constraintDescs[1].data1, a >= constraintDescs[1].numRows);
				
				setConstants(cConstant.z, cUnbiasedConstant.z, cVelMultiplier.z, cImpulseMultiplier.z, 
							 *con2, ur.z, constraintDescs[2].minResponseThreshold, erp[2], dt, recipdt, 
							 *constraintDescs[2].data0, *constraintDescs[2].data1, a >= constraintDescs[2].numRows);

				setConstants(cConstant.w, cUnbiasedConstant.w, cVelMultiplier.w, cImpulseMultiplier.w, 
							 *con3, ur.w, constraintDescs[3].minResponseThreshold, erp[3], dt, recipdt, 
							 *constraintDescs[3].data0, *constraintDescs[3].data1, a >= constraintDescs[3].numRows);
			}

			const Vec4V velBias = V4Mul(c->velMultiplier, normalVel);
			c->constant = V4Add(c->constant, velBias);
			c->unbiasedConstant = V4Add(c->unbiasedConstant, velBias);

			if(con0->flags & Px1DConstraintFlag::eOUTPUT_FORCE)
				c->flags[0] |= DY_SC_FLAG_OUTPUT_FORCE;
			if(con1->flags & Px1DConstraintFlag::eOUTPUT_FORCE)
				c->flags[1] |= DY_SC_FLAG_OUTPUT_FORCE;
			if(con2->flags & Px1DConstraintFlag::eOUTPUT_FORCE)
				c->flags[2] |= DY_SC_FLAG_OUTPUT_FORCE;
			if(con3->flags & Px1DConstraintFlag::eOUTPUT_FORCE)
				c->flags[3] |= DY_SC_FLAG_OUTPUT_FORCE;
		}
		*(reinterpret_cast<PxU32*>(currPtr)) = 0;
		*(reinterpret_cast<PxU32*>(currPtr + 4)) = 0;
	}
	
	//OK, we're ready to allocate and solve prep these constraints now :-)
	return SolverConstraintPrepState::eSUCCESS;
}

}

}
