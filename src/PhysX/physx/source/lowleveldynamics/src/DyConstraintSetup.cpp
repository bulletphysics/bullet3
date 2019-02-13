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
#include "PsSort.h"
#include "DySolverConstraintDesc.h"
#include "PxcConstraintBlockStream.h"
#include "DyArticulationContactPrep.h"
#include "PsFoundation.h"

namespace physx
{
namespace Dy
{
	// dsequeira:
	//
	// we can choose any linear combination of equality constraints and get the same solution
	// Hence we can orthogonalize the constraints using the inner product given by the
	// inverse mass matrix, so that when we use PGS, solving a constraint row for a joint 
	// don't disturb the solution of prior rows.
	//
	// We also eliminate the equality constraints from the hard inequality constraints - 
	// (essentially projecting the direction corresponding to the lagrange multiplier 
	// onto the equality constraint subspace) but 'til I've verified this generates 
	// exactly the same KKT/complementarity conditions, status is 'experimental'. 	
	//
	// since for equality constraints the resulting rows have the property that applying
	// an impulse along one row doesn't alter the projected velocity along another row, 
	// all equality constraints (plus one inequality constraint) can be processed in parallel
	// using SIMD
	//
	// Eliminating the inequality constraints from each other would require a solver change
	// and not give us any more parallelism, although we might get better convergence.

namespace
{
	PX_FORCE_INLINE Vec3V V3FromV4(Vec4V x)			{ return Vec3V_From_Vec4V(x); }
	PX_FORCE_INLINE Vec3V V3FromV4Unsafe(Vec4V x)	{ return Vec3V_From_Vec4V_WUndefined(x); }
	PX_FORCE_INLINE Vec4V V4FromV3(Vec3V x)			{ return Vec4V_From_Vec3V(x); }
	//PX_FORCE_INLINE Vec4V V4ClearW(Vec4V x)			{ return V4SetW(x, FZero()); }

struct MassProps
{
	FloatV invMass0;
	FloatV invMass1;
	FloatV invInertiaScale0;
	FloatV invInertiaScale1;

	PX_FORCE_INLINE MassProps(const PxReal imass0, const PxReal imass1, const PxConstraintInvMassScale& ims)
	:	
		invMass0(FLoad(imass0 * ims.linear0))
	,	invMass1(FLoad(imass1 * ims.linear1))
	,	invInertiaScale0(FLoad(ims.angular0))
	,	invInertiaScale1(FLoad(ims.angular1))
	{}
};


PX_FORCE_INLINE PxReal innerProduct(const Px1DConstraint& row0, Px1DConstraint& row1, 
								 PxVec4& row0AngSqrtInvInertia0, PxVec4& row0AngSqrtInvInertia1, 
								 PxVec4& row1AngSqrtInvInertia0, PxVec4& row1AngSqrtInvInertia1, const MassProps& m)
{
	const Vec3V l0 = V3Mul(V3Scale(V3LoadA(row0.linear0), m.invMass0), V3LoadA(row1.linear0));
	const Vec3V l1 = V3Mul(V3Scale(V3LoadA(row0.linear1), m.invMass1), V3LoadA(row1.linear1));
	Vec4V r0ang0 = V4LoadA(&row0AngSqrtInvInertia0.x);
	Vec4V r1ang0 = V4LoadA(&row1AngSqrtInvInertia0.x);
	Vec4V r0ang1 = V4LoadA(&row0AngSqrtInvInertia1.x);
	Vec4V r1ang1 = V4LoadA(&row1AngSqrtInvInertia1.x);

	const Vec3V i0 = V3ScaleAdd(V3Mul(Vec3V_From_Vec4V(r0ang0), Vec3V_From_Vec4V(r1ang0)), m.invInertiaScale0, l0);
	const Vec3V i1 = V3ScaleAdd(V3MulAdd(Vec3V_From_Vec4V(r0ang1), Vec3V_From_Vec4V(r1ang1), i0), m.invInertiaScale1, l1);
	PxF32 f;
	FStore(V3SumElems(i1), &f);
	return f;
}


// indexed rotation around axis, with sine and cosine of half-angle
PX_FORCE_INLINE PxQuat indexedRotation(PxU32 axis, PxReal s, PxReal c)
{
	PxQuat q(0,0,0,c);
	reinterpret_cast<PxReal*>(&q)[axis] = s;
	return q;
}

PxQuat diagonalize(const PxMat33& m)	// jacobi rotation using quaternions 
{
	const PxU32 MAX_ITERS = 5;

	PxQuat q = PxQuat(PxIdentity);

	PxMat33 d;
	for(PxU32 i=0; i < MAX_ITERS;i++)
	{
		const PxMat33 axes(q);
		d = axes.getTranspose() * m * axes;

		const PxReal d0 = PxAbs(d[1][2]), d1 = PxAbs(d[0][2]), d2 = PxAbs(d[0][1]);
		const PxU32 a = PxU32(d0 > d1 && d0 > d2 ? 0 : d1 > d2 ? 1 : 2);						// rotation axis index, from largest off-diagonal element

		const PxU32 a1 = Ps::getNextIndex3(a), a2 = Ps::getNextIndex3(a1);											
		if(d[a1][a2] == 0.0f || PxAbs(d[a1][a1]-d[a2][a2]) > 2e6f*PxAbs(2.0f*d[a1][a2]))
			break;

		const PxReal w = (d[a1][a1]-d[a2][a2]) / (2.0f*d[a1][a2]);					// cot(2 * phi), where phi is the rotation angle
		const PxReal absw = PxAbs(w);

		PxQuat r;
		if(absw>1000)
			r = indexedRotation(a, 1.0f/(4.0f*w), 1.f);									// h will be very close to 1, so use small angle approx instead
		else
		{
  			const PxReal t = 1 / (absw + PxSqrt(w*w+1));								// absolute value of tan phi
			const PxReal h = 1 / PxSqrt(t*t+1);										// absolute value of cos phi

			PX_ASSERT(h!=1);													// |w|<1000 guarantees this with typical IEEE754 machine eps (approx 6e-8)
			r = indexedRotation(a, PxSqrt((1-h)/2) * PxSign(w), PxSqrt((1+h)/2));
		}
	
		q = (q*r).getNormalized();
	}

	return q;
}


PX_FORCE_INLINE void rescale(const Mat33V& m, PxVec3& a0, PxVec3& a1, PxVec3& a2)
{
	const Vec3V va0 = V3LoadU(a0);
	const Vec3V va1 = V3LoadU(a1);
	const Vec3V va2 = V3LoadU(a2);

	const Vec3V b0 = V3ScaleAdd(va0, V3GetX(m.col0), V3ScaleAdd(va1, V3GetY(m.col0), V3Scale(va2, V3GetZ(m.col0))));
	const Vec3V b1 = V3ScaleAdd(va0, V3GetX(m.col1), V3ScaleAdd(va1, V3GetY(m.col1), V3Scale(va2, V3GetZ(m.col1))));
	const Vec3V b2 = V3ScaleAdd(va0, V3GetX(m.col2), V3ScaleAdd(va1, V3GetY(m.col2), V3Scale(va2, V3GetZ(m.col2))));

	V3StoreU(b0, a0);
	V3StoreU(b1, a1);
	V3StoreU(b2, a2);
}

PX_FORCE_INLINE void rescale4(const Mat33V& m, PxReal* a0, PxReal* a1, PxReal* a2)
{
	const Vec4V va0 = V4LoadA(a0);
	const Vec4V va1 = V4LoadA(a1);
	const Vec4V va2 = V4LoadA(a2);

	const Vec4V b0 = V4ScaleAdd(va0, V3GetX(m.col0), V4ScaleAdd(va1, V3GetY(m.col0), V4Scale(va2, V3GetZ(m.col0))));
	const Vec4V b1 = V4ScaleAdd(va0, V3GetX(m.col1), V4ScaleAdd(va1, V3GetY(m.col1), V4Scale(va2, V3GetZ(m.col1))));
	const Vec4V b2 = V4ScaleAdd(va0, V3GetX(m.col2), V4ScaleAdd(va1, V3GetY(m.col2), V4Scale(va2, V3GetZ(m.col2))));

	V4StoreA(b0, a0);
	V4StoreA(b1, a1);
	V4StoreA(b2, a2);
}

void diagonalize(Px1DConstraint** row,
				 PxVec4* angSqrtInvInertia0,
				 PxVec4* angSqrtInvInertia1,
				 const MassProps &m)
{
	PxReal a00 = innerProduct(*row[0], *row[0], angSqrtInvInertia0[0], angSqrtInvInertia1[0], angSqrtInvInertia0[0], angSqrtInvInertia1[0], m);
	PxReal a01 = innerProduct(*row[0], *row[1], angSqrtInvInertia0[0], angSqrtInvInertia1[0], angSqrtInvInertia0[1], angSqrtInvInertia1[1], m);
	PxReal a02 = innerProduct(*row[0], *row[2], angSqrtInvInertia0[0], angSqrtInvInertia1[0], angSqrtInvInertia0[2], angSqrtInvInertia1[2], m);
	PxReal a11 = innerProduct(*row[1], *row[1], angSqrtInvInertia0[1], angSqrtInvInertia1[1], angSqrtInvInertia0[1], angSqrtInvInertia1[1], m);
	PxReal a12 = innerProduct(*row[1], *row[2], angSqrtInvInertia0[1], angSqrtInvInertia1[1], angSqrtInvInertia0[2], angSqrtInvInertia1[2], m);
	PxReal a22 = innerProduct(*row[2], *row[2], angSqrtInvInertia0[2], angSqrtInvInertia1[2], angSqrtInvInertia0[2], angSqrtInvInertia1[2], m);

	PxMat33 a(PxVec3(a00, a01, a02),
			  PxVec3(a01, a11, a12),
			  PxVec3(a02, a12, a22));

	PxQuat q = diagonalize(a);

	PxMat33 n(-q);

	Mat33V mn(V3LoadU(n.column0), V3LoadU(n.column1), V3LoadU(n.column2));

	//KS - We treat as a Vec4V so that we get geometricError rescaled for free along with linear0
	rescale4(mn, &row[0]->linear0.x, &row[1]->linear0.x, &row[2]->linear0.x);
	rescale(mn, row[0]->linear1, row[1]->linear1, row[2]->linear1);
	//KS - We treat as a PxVec4 so that we get velocityTarget rescaled for free 
	rescale4(mn, &row[0]->angular0.x, &row[1]->angular0.x, &row[2]->angular0.x);
	rescale(mn, row[0]->angular1, row[1]->angular1, row[2]->angular1);
	rescale4(mn, &angSqrtInvInertia0[0].x, &angSqrtInvInertia0[1].x, &angSqrtInvInertia0[2].x);
	rescale4(mn, &angSqrtInvInertia1[0].x, &angSqrtInvInertia1[1].x, &angSqrtInvInertia1[2].x);
	
}

void orthogonalize(Px1DConstraint** row,
				   PxVec4* angSqrtInvInertia0,
				   PxVec4* angSqrtInvInertia1,
				   PxU32 rowCount,
				   PxU32 eqRowCount,
				   const MassProps &m)
{
	PX_ASSERT(eqRowCount<=6);

	const FloatV zero = FZero();

	Vec3V lin1m[6], ang1m[6], lin1[6], ang1[6];	
	Vec4V lin0m[6], ang0m[6];			// must have 0 in the W-field
	Vec4V lin0AndG[6], ang0AndT[6];

	for(PxU32 i=0;i<rowCount;i++)
	{
		Vec4V l0AndG = V4LoadA(&row[i]->linear0.x);		// linear0 and geometric error
		Vec4V a0AndT = V4LoadA(&row[i]->angular0.x);	// angular0 and velocity target

		Vec3V l1 = V3FromV4(V4LoadA(&row[i]->linear1.x));
		Vec3V a1 = V3FromV4(V4LoadA(&row[i]->angular1.x));

		Vec4V angSqrtL0 = V4LoadA(&angSqrtInvInertia0[i].x);
		Vec4V angSqrtL1 = V4LoadA(&angSqrtInvInertia1[i].x);

		PxU32 eliminationRows = PxMin<PxU32>(i, eqRowCount);
		for(PxU32 j=0;j<eliminationRows;j++)
		{
			const Vec3V s0 = V3MulAdd(l1, lin1m[j], V3FromV4Unsafe(V4Mul(l0AndG, lin0m[j])));
			const Vec3V s1 = V3MulAdd(V3FromV4Unsafe(angSqrtL1), ang1m[j], V3FromV4Unsafe(V4Mul(angSqrtL0, ang0m[j])));
			FloatV t = V3SumElems(V3Add(s0, s1));

			l0AndG = V4NegScaleSub(lin0AndG[j], t, l0AndG);
			a0AndT = V4NegScaleSub(ang0AndT[j], t, a0AndT);
			l1 = V3NegScaleSub(lin1[j], t, l1);
			a1 = V3NegScaleSub(ang1[j], t, a1);
			angSqrtL0 = V4NegScaleSub(V4LoadA(&angSqrtInvInertia0[j].x), t, angSqrtL0);
			angSqrtL1 = V4NegScaleSub(V4LoadA(&angSqrtInvInertia1[j].x), t, angSqrtL1);
		}

		V4StoreA(l0AndG, &row[i]->linear0.x);
		V4StoreA(a0AndT, &row[i]->angular0.x);
		V3StoreA(l1, row[i]->linear1);
		V3StoreA(a1, row[i]->angular1);
		V4StoreA(angSqrtL0, &angSqrtInvInertia0[i].x);
		V4StoreA(angSqrtL1, &angSqrtInvInertia1[i].x);

		if(i<eqRowCount)
		{
			lin0AndG[i] = l0AndG;	
			ang0AndT[i] = a0AndT;
			lin1[i] = l1;	
			ang1[i] = a1;	
			
			const Vec3V l0 = V3FromV4(l0AndG);

			const Vec3V l0m = V3Scale(l0, m.invMass0);
			const Vec3V l1m = V3Scale(l1, m.invMass1);
			const Vec4V a0m = V4Scale(angSqrtL0, m.invInertiaScale0);
			const Vec4V a1m = V4Scale(angSqrtL1, m.invInertiaScale1);

			const Vec3V s0 = V3MulAdd(l0, l0m, V3Mul(l1, l1m));
			const Vec4V s1 = V4MulAdd(a0m, angSqrtL0, V4Mul(a1m, angSqrtL1));
			const FloatV s = V3SumElems(V3Add(s0, V3FromV4Unsafe(s1)));
			const FloatV a = FSel(FIsGrtr(s, zero), FRecip(s), zero);	// with mass scaling, it's possible for the inner product of a row to be zero

			lin0m[i] = V4Scale(V4ClearW(V4FromV3(l0m)), a);	
			ang0m[i] = V4Scale(V4ClearW(a0m), a);
			lin1m[i] = V3Scale(l1m, a);
			ang1m[i] = V3Scale(V3FromV4Unsafe(a1m), a);
		}
	}
}
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
					bool preprocessLinear)
{
	// j is maxed at 12, typically around 7, so insertion sort is fine
	for(PxU32 i=0; i<rowCount; i++)
	{
		Px1DConstraint* r = rows+i;
		
		PxU32 j = i;
		for(;j>0 && r->solveHint < sorted[j-1]->solveHint; j--)
			sorted[j] = sorted[j-1];

		sorted[j] = r;
	}

	for(PxU32 i=0;i<rowCount-1;i++)
		PX_ASSERT(sorted[i]->solveHint <= sorted[i+1]->solveHint);

	for (PxU32 i = 0; i<rowCount; i++)
		rows[i].forInternalUse = rows[i].flags & Px1DConstraintFlag::eKEEPBIAS ? rows[i].geometricError : 0;


	const Mat33V sqrtInvInertia0 = Mat33V(V3LoadU(sqrtInvInertia0F32.column0), V3LoadU(sqrtInvInertia0F32.column1),
		V3LoadU(sqrtInvInertia0F32.column2));

	const Mat33V sqrtInvInertia1 = Mat33V(V3LoadU(sqrtInvInertia1F32.column0), V3LoadU(sqrtInvInertia1F32.column1),
		V3LoadU(sqrtInvInertia1F32.column2));

	PX_ASSERT(((uintptr_t(angSqrtInvInertia0)) & 0xF) == 0);
	PX_ASSERT(((uintptr_t(angSqrtInvInertia1)) & 0xF) == 0);

	for(PxU32 i = 0; i < rowCount; ++i)
	{
		const Vec3V angDelta0 = M33MulV3(sqrtInvInertia0, V3LoadU(sorted[i]->angular0));
		const Vec3V angDelta1 = M33MulV3(sqrtInvInertia1, V3LoadU(sorted[i]->angular1));
		V4StoreA(Vec4V_From_Vec3V(angDelta0), &angSqrtInvInertia0[i].x);
		V4StoreA(Vec4V_From_Vec3V(angDelta1), &angSqrtInvInertia1[i].x);
	}

	if(disablePreprocessing)
		return;

	MassProps m(invMass0, invMass1, ims);
	for(PxU32 i=0;i<rowCount;)
	{
		const PxU32 groupMajorId = PxU32(sorted[i]->solveHint>>8), start = i++;
		while(i<rowCount && PxU32(sorted[i]->solveHint>>8) == groupMajorId)
			i++;

		if(groupMajorId == 4 || (groupMajorId == 8 && preprocessLinear))
		{
			PxU32 bCount = start;		// count of bilateral constraints 
			for(; bCount<i && (sorted[bCount]->solveHint&255)==0; bCount++)
				;
			orthogonalize(sorted+start, angSqrtInvInertia0+start, angSqrtInvInertia1+start, i-start, bCount-start, m);
		}

		if(groupMajorId == 1 && diagonalizeDrive)
		{			
			PxU32 slerp = start;		// count of bilateral constraints 
			for(; slerp<i && (sorted[slerp]->solveHint&255)!=2; slerp++)
				;
			if(slerp+3 == i)
				diagonalize(sorted+slerp, angSqrtInvInertia0+slerp, angSqrtInvInertia1+slerp, m);

			PX_ASSERT(i-start==3);
			diagonalize(sorted+start, angSqrtInvInertia0+start, angSqrtInvInertia1+start, m);
		}
	}
}





PxU32 ConstraintHelper::setupSolverConstraint(
PxSolverConstraintPrepDesc& prepDesc,
PxConstraintAllocator& allocator,
PxReal dt, PxReal invdt,
Cm::SpatialVectorF* Z)
{
	if (prepDesc.numRows == 0)
	{
		prepDesc.desc->constraint = NULL;
		prepDesc.desc->writeBack = NULL;
		prepDesc.desc->constraintLengthOver16 = 0;
		prepDesc.desc->writeBackLengthOver4 = 0;
		return 0;
	}

	PxSolverConstraintDesc& desc = *prepDesc.desc;

	bool isExtended = desc.linkIndexA != PxSolverConstraintDesc::NO_LINK
		|| desc.linkIndexB != PxSolverConstraintDesc::NO_LINK;

	PxU32 stride = isExtended ? sizeof(SolverConstraint1DExt) : sizeof(SolverConstraint1D);
	const PxU32 constraintLength = sizeof(SolverConstraint1DHeader) + stride * prepDesc.numRows;
	
	//KS - +16 is for the constraint progress counter, which needs to be the last element in the constraint (so that we
	//know SPU DMAs have completed)
	PxU8* ptr = allocator.reserveConstraintData(constraintLength + 16u);
	if(NULL == ptr || (reinterpret_cast<PxU8*>(-1))==ptr)
	{
		if(NULL==ptr)
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
			ptr=NULL;
			return 0;
		}
	}
	desc.constraint = ptr;

	setConstraintLength(desc,constraintLength);

	desc.writeBack = prepDesc.writeback;
	setWritebackLength(desc, sizeof(ConstraintWriteback));

	memset(desc.constraint, 0, constraintLength);

	SolverConstraint1DHeader* header = reinterpret_cast<SolverConstraint1DHeader*>(desc.constraint);
	PxU8* constraints = desc.constraint + sizeof(SolverConstraint1DHeader);
	init(*header, Ps::to8(prepDesc.numRows), isExtended, prepDesc.mInvMassScales);
	header->body0WorldOffset = prepDesc.body0WorldOffset;
	header->linBreakImpulse = prepDesc.linBreakForce * dt;
	header->angBreakImpulse = prepDesc.angBreakForce * dt;
	header->breakable = PxU8((prepDesc.linBreakForce != PX_MAX_F32) || (prepDesc.angBreakForce != PX_MAX_F32));
	header->invMass0D0 = prepDesc.data0->invMass * prepDesc.mInvMassScales.linear0;
	header->invMass1D1 = prepDesc.data1->invMass * prepDesc.mInvMassScales.linear1;


	PX_ALIGN(16, PxVec4) angSqrtInvInertia0[MAX_CONSTRAINT_ROWS];
	PX_ALIGN(16, PxVec4) angSqrtInvInertia1[MAX_CONSTRAINT_ROWS];
	
	Px1DConstraint* sorted[MAX_CONSTRAINT_ROWS];

	preprocessRows(sorted, prepDesc.rows, angSqrtInvInertia0, angSqrtInvInertia1, prepDesc.numRows, 
		prepDesc.data0->sqrtInvInertia, prepDesc.data1->sqrtInvInertia, prepDesc.data0->invMass, prepDesc.data1->invMass, 
		prepDesc.mInvMassScales, isExtended || prepDesc.disablePreprocessing, prepDesc.improvedSlerp, true);

	const PxReal erp = 1.0f;
	for (PxU32 i = 0; i<prepDesc.numRows; i++)
	{
		Ps::prefetchLine(constraints, 128);
		SolverConstraint1D &s = *reinterpret_cast<SolverConstraint1D *>(constraints);
		Px1DConstraint& c = *sorted[i];

		PxReal driveScale = c.flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && prepDesc.driveLimitsAreForces ? PxMin(dt, 1.0f) : 1.0f;

		PxReal unitResponse;
		PxReal normalVel = 0.0f;
		PxReal initVel = 0.f;

		PxReal minResponseThreshold = prepDesc.minResponseThreshold;

		if(!isExtended)
		{
			init(s, c.linear0, c.linear1, PxVec3(angSqrtInvInertia0[i].x, angSqrtInvInertia0[i].y, angSqrtInvInertia0[i].z),
				PxVec3(angSqrtInvInertia1[i].x, angSqrtInvInertia1[i].y, angSqrtInvInertia1[i].z), c.minImpulse * driveScale, c.maxImpulse * driveScale);
			s.ang0Writeback = c.angular0;
			PxReal resp0 = s.lin0.magnitudeSquared() * prepDesc.data0->invMass * prepDesc.mInvMassScales.linear0 + s.ang0.magnitudeSquared() * prepDesc.mInvMassScales.angular0;
			PxReal resp1 = s.lin1.magnitudeSquared() * prepDesc.data1->invMass * prepDesc.mInvMassScales.linear1 + s.ang1.magnitudeSquared() * prepDesc.mInvMassScales.angular1;
			unitResponse = resp0 + resp1;
			initVel = normalVel = prepDesc.data0->projectVelocity(c.linear0, c.angular0) - prepDesc.data1->projectVelocity(c.linear1, c.angular1);
		}
		else
		{
			init(s, c.linear0, c.linear1, c.angular0, c.angular1, c.minImpulse * driveScale, c.maxImpulse * driveScale);
			SolverConstraint1DExt& e = static_cast<SolverConstraint1DExt&>(s);

			const SolverExtBody eb0(reinterpret_cast<const void*>(prepDesc.body0), prepDesc.data0, desc.linkIndexA);
			const SolverExtBody eb1(reinterpret_cast<const void*>(prepDesc.body1), prepDesc.data1, desc.linkIndexB);

			const Cm::SpatialVector resp0 = createImpulseResponseVector(e.lin0, e.ang0, eb0);
			const Cm::SpatialVector resp1 = createImpulseResponseVector(-e.lin1, -e.ang1, eb1);
			unitResponse = getImpulseResponse(eb0, resp0, unsimdRef(e.deltaVA), prepDesc.mInvMassScales.linear0, prepDesc.mInvMassScales.angular0,
				eb1, resp1, unsimdRef(e.deltaVB), prepDesc.mInvMassScales.linear1, prepDesc.mInvMassScales.angular1, Z, false);

			if(!(c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT))
			{
				PxReal totalMag = (unsimdRef(e.deltaVA) - unsimdRef(e.deltaVB)).magnitude();

				PxReal ratio = unitResponse / totalMag;
				if (ratio < 0.05f)
					unitResponse = 0.f;
			}


			s.ang0Writeback = c.angular0;
			s.lin0 = resp0.linear;
			s.ang0 = resp0.angular;
			s.lin1 = -resp1.linear;
			s.ang1 = -resp1.angular;
			PxReal vel0, vel1;
			if(needsNormalVel(c) || eb0.mLinkIndex == PxSolverConstraintDesc::NO_LINK || eb1.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
			{
				vel0 = eb0.projectVelocity(c.linear0, c.angular0);
				vel1 = eb1.projectVelocity(c.linear1, c.angular1);

				normalVel = vel0 - vel1;

				//normalVel = eb0.projectVelocity(s.lin0, s.ang0) - eb1.projectVelocity(s.lin1, s.ang1);
				if(eb0.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
					initVel = vel0;
				else if(eb1.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
					initVel = -vel1;

			}

			minResponseThreshold = PxMax(minResponseThreshold, DY_ARTICULATION_MIN_RESPONSE);
		}

		setSolverConstants(s.constant, s.unbiasedConstant, s.velMultiplier, s.impulseMultiplier, 
			c, normalVel, unitResponse, minResponseThreshold, erp, dt, invdt);

		//s.targetVelocity = initVel;
		const PxReal velBias = initVel * s.velMultiplier;
		s.constant += velBias;
		s.unbiasedConstant += velBias;

		if(c.flags & Px1DConstraintFlag::eOUTPUT_FORCE)
			s.flags |= DY_SC_FLAG_OUTPUT_FORCE;

		constraints += stride;
	}

	//KS - Set the solve count at the end to 0 
	*(reinterpret_cast<PxU32*>(constraints)) = 0;
	*(reinterpret_cast<PxU32*>(constraints + 4)) = 0;
	PX_ASSERT(desc.constraint + getConstraintLength(desc) == constraints);
	return prepDesc.numRows;
}

PxU32 SetupSolverConstraint(SolverConstraintShaderPrepDesc& shaderDesc,
	PxSolverConstraintPrepDesc& prepDesc,
	PxConstraintAllocator& allocator,
	PxReal dt, PxReal invdt, Cm::SpatialVectorF* Z)
{
	// LL shouldn't see broken constraints
	
	PX_ASSERT(!(reinterpret_cast<ConstraintWriteback*>(prepDesc.writeback)->broken));

	setConstraintLength(*prepDesc.desc, 0);

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
	PxVec3 ra, rb;
	PxU32 constraintCount = (*shaderDesc.solverPrep)(rows,
		body0WorldOffset,
		MAX_CONSTRAINT_ROWS,
		prepDesc.mInvMassScales,
		shaderDesc.constantBlock,
		prepDesc.bodyFrame0, prepDesc.bodyFrame1, prepDesc.extendedLimits, ra, rb);

	prepDesc.rows = rows;
	prepDesc.numRows = constraintCount;

	prepDesc.body0WorldOffset = body0WorldOffset;

	return ConstraintHelper::setupSolverConstraint(prepDesc, allocator, dt, invdt, Z);
}

}

}
