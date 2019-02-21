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

#include "GuVecCapsule.h"
#include "GuGeometryUnion.h"
#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuDistanceSegmentSegmentSIMD.h"

using namespace physx;
using namespace Gu;
using namespace Ps;
using namespace aos;

static Vec4V pcmDistancePointSegmentTValue22(	const Vec3VArg a0, const Vec3VArg b0, 
												const Vec3VArg a1, const Vec3VArg b1,
												const Vec3VArg p0, const Vec3VArg p1,
												const Vec3VArg p2, const Vec3VArg p3)
{
	const Vec4V zero = V4Zero();
	const Vec3V ap00 = V3Sub(p0, a0);
	const Vec3V ap10 = V3Sub(p1, a0);
	const Vec3V ap01 = V3Sub(p2, a1);
	const Vec3V ap11 = V3Sub(p3, a1);

	const Vec3V ab0 = V3Sub(b0, a0);
	const Vec3V ab1 = V3Sub(b1, a1);

/*	const FloatV nom00 = V3Dot(ap00, ab0);
	const FloatV nom10 = V3Dot(ap10, ab0);
	const FloatV nom01 = V3Dot(ap01, ab1);
	const FloatV nom11 = V3Dot(ap11, ab1);*/

	const Vec4V combinedDot = V3Dot4(ap00, ab0, ap10, ab0, ap01, ab1, ap11, ab1);
	const FloatV nom00 = V4GetX(combinedDot);
	const FloatV nom10 = V4GetY(combinedDot);
	const FloatV nom01 = V4GetZ(combinedDot);
	const FloatV nom11 = V4GetW(combinedDot);
	
	const FloatV denom0 = V3Dot(ab0, ab0);
	const FloatV denom1 = V3Dot(ab1, ab1);

	const Vec4V nom = V4Merge(nom00, nom10, nom01, nom11);
	const Vec4V denom = V4Merge(denom0, denom0, denom1, denom1);

	const Vec4V tValue = V4Div(nom, denom);
	return V4Sel(V4IsEq(denom, zero), zero, tValue);
}

namespace physx
{
namespace Gu
{

	static void storeContact(const Vec3VArg contact, const Vec3VArg normal, const FloatVArg separation, Gu::ContactBuffer& buffer)
	{
		Gu::ContactPoint& point = buffer.contacts[buffer.count++];

		const Vec4V normalSep = Ps::aos::V4SetW(Vec4V_From_Vec3V(normal), separation);

		V4StoreA(normalSep, &point.normal.x);
		V3StoreA(contact, point.point);
		point.internalFaceIndex1 = PXC_CONTACT_NO_FACE_INDEX;

	}

bool pcmContactCapsuleCapsule(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	// Get actual shape data
	const PxCapsuleGeometry& shapeCapsule0 = shape0.get<const PxCapsuleGeometry>();
	const PxCapsuleGeometry& shapeCapsule1 = shape1.get<const PxCapsuleGeometry>();

	PX_ASSERT(transform1.q.isSane());
	PX_ASSERT(transform0.q.isSane());

	const Vec3V _p0 = V3LoadA(&transform0.p.x);
	const QuatV q0 = QuatVLoadA(&transform0.q.x);

	const Vec3V _p1 = V3LoadA(&transform1.p.x);
	const QuatV q1 = QuatVLoadA(&transform1.q.x);

	/*PsTransformV transf0(p0, q0);
	PsTransformV transf1(p1, q1);*/

	
	const FloatV r0 = FLoad(shapeCapsule0.radius);
	const FloatV halfHeight0 = FLoad(shapeCapsule0.halfHeight);

	const FloatV r1 = FLoad(shapeCapsule1.radius);
	const FloatV halfHeight1 = FLoad(shapeCapsule1.halfHeight);

	const FloatV cDist = FLoad(params.mContactDistance);

	const Vec3V positionOffset = V3Scale(V3Add(_p0, _p1),  FHalf());
	const Vec3V p0 = V3Sub(_p0, positionOffset);
	const Vec3V p1 = V3Sub(_p1, positionOffset);

	const FloatV zero = FZero();
	//const FloatV one = FOne();
	const Vec3V zeroV = V3Zero();
	

	/*const Vec3V positionOffset = V3Scale(V3Add(transf0.p, transf1.p), FloatV_From_F32(0.5f));
	transf0.p = V3Sub(transf0.p, positionOffset);
	transf1.p = V3Sub(transf1.p, positionOffset);*/

	const Vec3V basisVector0 = QuatGetBasisVector0(q0);
	const Vec3V tmp0 = V3Scale(basisVector0, halfHeight0);
	const Vec3V s0 = V3Add(p0, tmp0);
	const Vec3V e0 = V3Sub(p0, tmp0);
	const Vec3V d0 = V3Sub(e0, s0);

	const Vec3V basisVector1 = QuatGetBasisVector0(q1);
	const Vec3V tmp1 = V3Scale(basisVector1, halfHeight1);
	const Vec3V s1 = V3Add(p1, tmp1);
	const Vec3V e1 = V3Sub(p1, tmp1); 

	const Vec3V d1 = V3Sub(e1, s1);

	const FloatV sumRadius = FAdd(r0, r1);
	const FloatV inflatedSum = FAdd(sumRadius, cDist);
	const FloatV inflatedSumSquared = FMul(inflatedSum, inflatedSum);
	const FloatV a = V3Dot(d0, d0);//squared length of segment1
	const FloatV e = V3Dot(d1, d1);//squared length of segment2
	const FloatV eps = FLoad(1e-6f);//FEps();

	FloatV t0, t1;
	const FloatV sqDist0 = distanceSegmentSegmentSquared(s0, d0, s1, d1, t0, t1);

	if(FAllGrtrOrEq(inflatedSumSquared, sqDist0))
	{
		const Vec4V zeroV4 = V4Zero();
		const Vec4V oneV4 = V4One();
		//check to see whether these two capsule are paralle
		const FloatV parallelTolerance  = FLoad(0.9998f);
		
		
		const BoolV con0 = FIsGrtr(eps, a);
		const BoolV con1 = FIsGrtr(eps, e);
		const Vec3V dir0 = V3Sel(con0, zeroV, V3ScaleInv(d0, FSqrt(a)));
		const Vec3V dir1 = V3Sel(con1, zeroV, V3ScaleInv(d1, FSqrt(e)));

		const FloatV cos = FAbs(V3Dot(dir0, dir1));
		if(FAllGrtr(cos, parallelTolerance))//paralle
		{
			//project s, e into s1e1
			const Vec4V t= pcmDistancePointSegmentTValue22(s0, e0, s1, e1,
															s1, e1, s0, e0);

			const BoolV con = BAnd(V4IsGrtrOrEq(t, zeroV4), V4IsGrtrOrEq(oneV4, t));
			const BoolV con00 = BGetX(con);
			const BoolV con01 = BGetY(con);
			const BoolV con10 = BGetZ(con);
			const BoolV con11 = BGetW(con);

		/*	PX_ALIGN(16, PxU32 conditions[4]);
			F32Array_Aligned_From_Vec4V(con, (PxF32*)conditions);*/

			
			PxU32 numContact=0;

			if(BAllEqTTTT(con00))
			{
				const Vec3V projS1 = V3ScaleAdd(d0, V4GetX(t), s0);
				const Vec3V v = V3Sub(projS1, s1);
				const FloatV sqDist = V3Dot(v, v);
				const BoolV bCon = BAnd(FIsGrtr(sqDist, eps), FIsGrtr(inflatedSumSquared, sqDist));
				
				if(BAllEqTTTT(bCon))
				{
					const FloatV dist = FSqrt(sqDist);
					const FloatV pen =  FSub(dist, sumRadius);
					const Vec3V normal = V3ScaleInv(v, dist);
					PX_ASSERT(isFiniteVec3V(normal));
					const Vec3V _p = V3NegScaleSub(normal, r0, projS1);
					const Vec3V p = V3Add(_p, positionOffset);
					
					storeContact(p, normal, pen, contactBuffer);
					numContact++;
				}
			}
			if(BAllEqTTTT(con01))
			{
				const Vec3V projE1 = V3ScaleAdd(d0, V4GetY(t), s0);
				const Vec3V v = V3Sub(projE1, e1);
				const FloatV sqDist = V3Dot(v, v);
				const BoolV bCon = BAnd(FIsGrtr(sqDist, eps), FIsGrtr(inflatedSumSquared, sqDist));
				
				if(BAllEqTTTT(bCon))
				{
					const FloatV dist = FSqrt(sqDist);
					const FloatV pen =  FSub(dist, sumRadius);
					const Vec3V normal =  V3ScaleInv(v, dist);
					PX_ASSERT(isFiniteVec3V(normal));
					const Vec3V _p = V3NegScaleSub(normal, r0, projE1);
					const Vec3V p = V3Add(_p, positionOffset);
					storeContact(p, normal, pen, contactBuffer);
					numContact++;
				}
			}

			if(BAllEqTTTT(con10))
			{
				const Vec3V projS0 = V3ScaleAdd(d1, V4GetZ(t), s1);
				const Vec3V v = V3Sub(s0, projS0);
				const FloatV sqDist = V3Dot(v, v);
				const BoolV bCon = BAnd(FIsGrtr(sqDist, eps), FIsGrtr(inflatedSumSquared, sqDist));
				
				if(BAllEqTTTT(bCon))
				{
					const FloatV dist = FSqrt(sqDist);
					const FloatV pen =  FSub(dist, sumRadius);
					const Vec3V normal = V3ScaleInv(v, dist);
				 	PX_ASSERT(isFiniteVec3V(normal));
					const Vec3V _p = V3NegScaleSub(normal, r0, s0);
					const Vec3V p = V3Add(_p, positionOffset);
					//const Vec3V p = V3ScaleAdd(normal, r0, s0);
					storeContact(p, normal, pen, contactBuffer);
					numContact++;
				}
			}

			if(BAllEqTTTT(con11))
			{
				const Vec3V projE0 = V3ScaleAdd(d1, V4GetW(t), s1);
				const Vec3V v = V3Sub(e0, projE0);
				const FloatV sqDist = V3Dot(v, v);
				const BoolV bCon = BAnd(FIsGrtr(sqDist, eps), FIsGrtr(inflatedSumSquared, sqDist));
				
				if(BAllEqTTTT(bCon))
				{
					const FloatV dist = FSqrt(sqDist);
					const FloatV pen =  FSub(dist, sumRadius);
					const Vec3V normal = V3ScaleInv(v, dist);
					PX_ASSERT(isFiniteVec3V(normal));
					const Vec3V _p = V3NegScaleSub(normal, r0, e0);
					const Vec3V p = V3Add(_p, positionOffset);
					//const Vec3V p = V3ScaleAdd(normal, r0, e0);
					storeContact(p, normal, pen, contactBuffer);
					numContact++;
				}
			}

			if(numContact)
				return true;

		}  

		const Vec3V closestA = V3ScaleAdd(d0, t0, s0);
		const Vec3V closestB = V3ScaleAdd(d1, t1, s1);
		
		const BoolV con = FIsGrtr(eps, sqDist0);
		//const Vec3V normal = V3Sel(FIsEq(dist, zero), V3Sel(FIsGrtr(a, eps), V3Normalise(d0), V3Scale(V3Sub(closestA, closestB), FRecip(dist)));
		const Vec3V _normal = V3Sel(con, V3Sel(FIsGrtr(a, eps), d0, V3UnitX()), V3Sub(closestA, closestB));
		const Vec3V normal = V3Normalize(_normal);
		PX_ASSERT(isFiniteVec3V(normal));
		const Vec3V _point = V3NegScaleSub(normal, r0, closestA);
		const Vec3V p = V3Add(_point, positionOffset);
		const FloatV dist = FSel(con, zero, FSqrt(sqDist0));
		const FloatV pen = FSub(dist, sumRadius);
		//PX_ASSERT(FAllGrtrOrEq(zero, pen));
		storeContact(p, normal, pen, contactBuffer);
		return true;
	}
	return false;
}
}//Gu
}//physx
