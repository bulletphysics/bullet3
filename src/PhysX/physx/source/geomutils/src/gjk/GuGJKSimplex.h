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

#ifndef GU_GJKSIMPLEX_H
#define GU_GJKSIMPLEX_H

#include "CmPhysXCommon.h"
#include "PsVecMath.h"
#include "GuBarycentricCoordinates.h"

#if (defined __GNUC__ && defined _DEBUG)
#define PX_GJK_INLINE PX_INLINE
#define PX_GJK_FORCE_INLINE PX_INLINE
#else
#define PX_GJK_INLINE PX_INLINE
#define PX_GJK_FORCE_INLINE PX_FORCE_INLINE
#endif


namespace physx
{
namespace Gu
{


	PX_NOALIAS Ps::aos::Vec3V closestPtPointTetrahedron(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxU32& size);

	PX_NOALIAS Ps::aos::Vec3V closestPtPointTetrahedron(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxI32* PX_RESTRICT aInd, PxI32* PX_RESTRICT bInd, 
		PxU32& size);

	PX_NOALIAS PX_FORCE_INLINE Ps::aos::BoolV PointOutsideOfPlane4(const Ps::aos::Vec3VArg _a, const Ps::aos::Vec3VArg _b, const Ps::aos::Vec3VArg _c, const Ps::aos::Vec3VArg _d)
	{
		using namespace Ps::aos;
		
		// this is not 0 because of the following scenario:
		// All the points lie on the same plane and the plane goes through the origin (0,0,0).
		// On the Wii U, the math below has the problem that when point A gets projected on the
		// plane cumputed by A, B, C, the distance to the plane might not be 0 for the mentioned
		// scenario but a small positive or negative value. This can lead to the wrong boolean
		// results. Using a small negative value as threshold is more conservative but safer.
		const Vec4V zero = V4Load(-1e-6f);

		const Vec3V ab = V3Sub(_b, _a);
		const Vec3V ac = V3Sub(_c, _a);
		const Vec3V ad = V3Sub(_d, _a);
		const Vec3V bd = V3Sub(_d, _b);
		const Vec3V bc = V3Sub(_c, _b);

		const Vec3V v0 = V3Cross(ab, ac);
		const Vec3V v1 = V3Cross(ac, ad);
		const Vec3V v2 = V3Cross(ad, ab);
		const Vec3V v3 = V3Cross(bd, bc);

		const FloatV signa0 = V3Dot(v0, _a);
		const FloatV signa1 = V3Dot(v1, _a);
		const FloatV signa2 = V3Dot(v2, _a);
		const FloatV signd3 = V3Dot(v3, _a);

		const FloatV signd0 = V3Dot(v0, _d);
		const FloatV signd1 = V3Dot(v1, _b);
		const FloatV signd2 = V3Dot(v2, _c);
		const FloatV signa3 = V3Dot(v3, _b);

		const Vec4V signa = V4Merge(signa0, signa1, signa2, signa3);
		const Vec4V signd = V4Merge(signd0, signd1, signd2, signd3);
		return V4IsGrtrOrEq(V4Mul(signa, signd), zero);//same side, outside of the plane

		
	}


	PX_NOALIAS PX_FORCE_INLINE Ps::aos::Vec3V closestPtPointSegment(Ps::aos::Vec3V* PX_RESTRICT Q, PxU32& size)
	{
		using namespace Ps::aos;
		const Vec3V a = Q[0];
		const Vec3V b = Q[1];

		//const Vec3V origin = V3Zero();
		const FloatV zero = FZero();
		const FloatV one = FOne();

		//Test degenerated case
		const Vec3V ab = V3Sub(b, a);
		const FloatV denom = V3Dot(ab, ab);
		const Vec3V ap = V3Neg(a);//V3Sub(origin, a);
		const FloatV nom = V3Dot(ap, ab);
		const BoolV con = FIsGrtrOrEq(FEps(), denom);//FIsEq(denom, zero);
		//TODO - can we get rid of this branch? The problem is size, which isn't a vector!
		if(BAllEqTTTT(con))
		{
			size = 1;
			return Q[0];
		}

	/*	const PxU32 count = BAllEq(con, bTrue);
		size = 2 - count;*/
		
		const FloatV tValue = FClamp(FDiv(nom, denom), zero, one);
		return V3ScaleAdd(ab, tValue, a);
	}


	PX_FORCE_INLINE void getClosestPoint(const Ps::aos::Vec3V* PX_RESTRICT Q, const Ps::aos::Vec3V* PX_RESTRICT A, const Ps::aos::Vec3V* PX_RESTRICT B, const Ps::aos::Vec3VArg closest, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB, const PxU32 size)
	{
		using namespace Ps::aos;

		switch(size)
		{
		case 1:
			{
				closestA = A[0];
				closestB = B[0];
				break;
			}
		case 2:
			{
				FloatV v;
				barycentricCoordinates(closest, Q[0], Q[1], v);
				const Vec3V av = V3Sub(A[1], A[0]);
				const Vec3V bv = V3Sub(B[1], B[0]);
				closestA = V3ScaleAdd(av, v, A[0]);
				closestB = V3ScaleAdd(bv, v, B[0]);
				
				break;
			}
		case 3:
			{
				//calculate the Barycentric of closest point p in the mincowsky sum
				FloatV v, w;
				barycentricCoordinates(closest, Q[0], Q[1], Q[2], v, w);

				const Vec3V av0 = V3Sub(A[1], A[0]);
				const Vec3V av1 = V3Sub(A[2], A[0]);
				const Vec3V bv0 = V3Sub(B[1], B[0]);
				const Vec3V bv1 = V3Sub(B[2], B[0]);

				closestA = V3Add(A[0], V3Add(V3Scale(av0, v), V3Scale(av1, w)));
				closestB = V3Add(B[0], V3Add(V3Scale(bv0, v), V3Scale(bv1, w)));
			}
		};
	}

	PX_NOALIAS PX_GJK_FORCE_INLINE Ps::aos::FloatV closestPtPointTriangleBaryCentric(const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, const Ps::aos::Vec3VArg c, 
		PxU32* PX_RESTRICT indices, PxU32& size, Ps::aos::Vec3V& closestPt)
	{
		using namespace Ps::aos;

		size = 3;
		const FloatV zero = FZero();
		const FloatV eps = FEps();
		
		const Vec3V ab = V3Sub(b, a);
		const Vec3V ac = V3Sub(c, a);

		const Vec3V n = V3Cross(ab, ac);
		//ML: if the shape is oblong, the degeneracy test sometime can't catch the degeneracy in the tetraheron. Therefore, we need to make sure we still can ternimate with the previous
		//triangle by returning the maxinum distance. 
		const FloatV nn = V3Dot(n, n);
		if (FAllEq(nn, zero))
			return FMax();

		//const FloatV va = FNegScaleSub(d5, d4, FMul(d3, d6));//edge region of BC
		//const FloatV vb = FNegScaleSub(d1, d6, FMul(d5, d2));//edge region of AC
		//const FloatV vc = FNegScaleSub(d3, d2, FMul(d1, d4));//edge region of AB

		//const FloatV va = V3Dot(n, V3Cross(b, c));//edge region of BC, signed area rbc, u = S(rbc)/S(abc) for a
		//const FloatV vb = V3Dot(n, V3Cross(c, a));//edge region of AC, signed area rac, v = S(rca)/S(abc) for b
		//const FloatV vc = V3Dot(n, V3Cross(a, b));//edge region of AB, signed area rab, w = S(rab)/S(abc) for c

		const VecCrossV crossA = V3PrepareCross(a);
		const VecCrossV crossB = V3PrepareCross(b);
		const VecCrossV crossC = V3PrepareCross(c);
		const Vec3V bCrossC = V3Cross(crossB, crossC);
		const Vec3V cCrossA = V3Cross(crossC, crossA);
		const Vec3V aCrossB = V3Cross(crossA, crossB);

		const FloatV va = V3Dot(n, bCrossC);//edge region of BC, signed area rbc, u = S(rbc)/S(abc) for a
		const FloatV vb = V3Dot(n, cCrossA);//edge region of AC, signed area rac, v = S(rca)/S(abc) for b
		const FloatV vc = V3Dot(n, aCrossB);//edge region of AB, signed area rab, w = S(rab)/S(abc) for c

		const BoolV isFacePoints = BAnd(FIsGrtrOrEq(va, zero), BAnd(FIsGrtrOrEq(vb, zero), FIsGrtrOrEq(vc, zero)));


		//face region
		if(BAllEqTTTT(isFacePoints))
		{	
			const FloatV t = FDiv(V3Dot(n, a), nn);
			const Vec3V q = V3Scale(n, t);
			closestPt = q;
			return V3Dot(q, q);
		}

		const Vec3V ap = V3Neg(a);
		const Vec3V bp = V3Neg(b);
		const Vec3V cp = V3Neg(c);

		const FloatV d1 = V3Dot(ab, ap); //  snom
		const FloatV d2 = V3Dot(ac, ap); //  tnom
		const FloatV d3 = V3Dot(ab, bp); // -sdenom
		const FloatV d4 = V3Dot(ac, bp); //  unom = d4 - d3
		const FloatV d5 = V3Dot(ab, cp); //  udenom = d5 - d6
		const FloatV d6 = V3Dot(ac, cp); // -tdenom


		const FloatV unom = FSub(d4, d3);
		const FloatV udenom = FSub(d5, d6);

		size = 2;
		//check if p in edge region of AB
		const BoolV con30 = FIsGrtrOrEq(zero, vc);
		const BoolV con31 = FIsGrtrOrEq(d1, zero);
		const BoolV con32 = FIsGrtrOrEq(zero, d3);
		const BoolV con3 = BAnd(con30, BAnd(con31, con32));//edge AB region
		if(BAllEqTTTT(con3))
		{
			const FloatV toRecipAB = FSub(d1, d3);
			const FloatV recipAB = FSel(FIsGrtr(FAbs(toRecipAB), eps), FRecip(toRecipAB), zero);
			const FloatV t = FMul(d1, recipAB);
			const Vec3V q = V3ScaleAdd(ab, t, a);
			closestPt = q;
			return V3Dot(q, q);
		}
	
		//check if p in edge region of BC
		const BoolV con40 = FIsGrtrOrEq(zero, va);
		const BoolV con41 = FIsGrtrOrEq(d4, d3);
		const BoolV con42 = FIsGrtrOrEq(d5, d6);
		const BoolV con4 = BAnd(con40, BAnd(con41, con42)); //edge BC region
		if(BAllEqTTTT(con4))
		{
			const Vec3V bc = V3Sub(c, b);
			const FloatV toRecipBC = FAdd(unom, udenom);
			const FloatV recipBC = FSel(FIsGrtr(FAbs(toRecipBC), eps), FRecip(toRecipBC), zero);
			const FloatV t = FMul(unom, recipBC);
			indices[0] = indices[1];
			indices[1] = indices[2];
			const Vec3V q = V3ScaleAdd(bc, t, b);
			closestPt = q;
			return V3Dot(q, q);
		}
		
		//check if p in edge region of AC
		const BoolV con50 = FIsGrtrOrEq(zero, vb);
		const BoolV con51 = FIsGrtrOrEq(d2, zero);
		const BoolV con52 = FIsGrtrOrEq(zero, d6);
	
		const BoolV con5 = BAnd(con50, BAnd(con51, con52));//edge AC region
		if(BAllEqTTTT(con5))
		{
			const FloatV toRecipAC = FSub(d2, d6);
			const FloatV recipAC = FSel(FIsGrtr(FAbs(toRecipAC), eps), FRecip(toRecipAC), zero);
			const FloatV t = FMul(d2, recipAC);
			indices[1]=indices[2];
			const Vec3V q = V3ScaleAdd(ac, t, a);
			closestPt = q;
			return V3Dot(q, q);
		}

		size = 1;
		//check if p in vertex region outside a
		const BoolV con00 = FIsGrtrOrEq(zero, d1); // snom <= 0
		const BoolV con01 = FIsGrtrOrEq(zero, d2); // tnom <= 0
		const BoolV con0 = BAnd(con00, con01); // vertex region a
		if(BAllEqTTTT(con0))
		{
			closestPt = a;
			return V3Dot(a, a);
		}

		//check if p in vertex region outside b
		const BoolV con10 = FIsGrtrOrEq(d3, zero);
		const BoolV con11 = FIsGrtrOrEq(d3, d4);
		const BoolV con1 = BAnd(con10, con11); // vertex region b
		if(BAllEqTTTT(con1))
		{
			indices[0] = indices[1];
			closestPt = b;
			return V3Dot(b, b);
		}
		
		//p is in vertex region outside c
		indices[0] = indices[2];
		closestPt = c;
		return V3Dot(c, c);

	}

	PX_NOALIAS PX_GJK_FORCE_INLINE Ps::aos::Vec3V closestPtPointTriangle(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* A, Ps::aos::Vec3V* B, PxU32& size)
	{
		
		using namespace Ps::aos;

		size = 3;
	
		const FloatV eps = FEps();
		const Vec3V a = Q[0];
		const Vec3V b = Q[1];
		const Vec3V c = Q[2];
		const Vec3V ab = V3Sub(b, a);
		const Vec3V ac = V3Sub(c, a);
		const Vec3V signArea = V3Cross(ab, ac);//0.5*(abXac)
		const FloatV area = V3Dot(signArea, signArea);
		if(FAllGrtrOrEq(eps, area))
		{
			//degenerate
			size = 2;
			return closestPtPointSegment(Q, size);
		}

		PxU32 _size;
		PxU32 indices[3]={0, 1, 2};
		Vec3V closestPt;
		closestPtPointTriangleBaryCentric(a, b, c, indices, _size, closestPt);

		if(_size != 3)
		{
		
			const Vec3V q0 = Q[indices[0]]; const Vec3V q1 = Q[indices[1]];
			const Vec3V a0 = A[indices[0]]; const Vec3V a1 = A[indices[1]];
			const Vec3V b0 = B[indices[0]]; const Vec3V b1 = B[indices[1]];

			Q[0] = q0; Q[1] = q1;
			A[0] = a0; A[1] = a1;
			B[0] = b0; B[1] = b1;

			size = _size;
		}

		return closestPt;
	}

	PX_NOALIAS PX_GJK_FORCE_INLINE Ps::aos::Vec3V closestPtPointTriangle(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* A, Ps::aos::Vec3V* B, PxI32* PX_RESTRICT aInd, PxI32* PX_RESTRICT bInd, 
		PxU32& size)
	{
		
		using namespace Ps::aos;

		size = 3;
	
		const FloatV eps = FEps();

		const Vec3V a = Q[0];
		const Vec3V b = Q[1];
		const Vec3V c = Q[2];
		const Vec3V ab = V3Sub(b, a);
		const Vec3V ac = V3Sub(c, a);
		const Vec3V signArea = V3Cross(ab, ac);//0.5*(abXac)
		const FloatV area = V3Dot(signArea, signArea);
		if(FAllGrtrOrEq(eps, area))
		{
			//degenerate
			size = 2;
			return closestPtPointSegment(Q, size);
		}

		PxU32 _size;
		PxU32 indices[3]={0, 1, 2};
		Vec3V closestPt;
		closestPtPointTriangleBaryCentric(a, b, c, indices, _size, closestPt);

		if(_size != 3)
		{
		
			const Vec3V q0 = Q[indices[0]]; const Vec3V q1 = Q[indices[1]];
			const Vec3V a0 = A[indices[0]]; const Vec3V a1 = A[indices[1]];
			const Vec3V b0 = B[indices[0]]; const Vec3V b1 = B[indices[1]];
			const PxI32 aInd0 = aInd[indices[0]]; const PxI32 aInd1 = aInd[indices[1]];
			const PxI32 bInd0 = bInd[indices[0]]; const PxI32 bInd1 = bInd[indices[1]];

			Q[0] = q0; Q[1] = q1;
			A[0] = a0; A[1] = a1;
			B[0] = b0; B[1] = b1;
			aInd[0] = aInd0; aInd[1] = aInd1;
			bInd[0] = bInd0; bInd[1] = bInd1;

			size = _size;
		}

		return closestPt;
	}

	
	

	PX_NOALIAS PX_FORCE_INLINE Ps::aos::Vec3V GJKCPairDoSimplex(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, const Ps::aos::Vec3VArg support, 
		PxU32& size)
	{
		using namespace Ps::aos;

		//const PxU32 tempSize = size;
		//calculate a closest from origin to the simplex
		switch(size)
		{
		case 1:
			{
				return support;
			}
		case 2:
			{
			return closestPtPointSegment(Q, size);
			}
		case 3:
			{
			return closestPtPointTriangle(Q, A, B, size);
			}
		case 4:
			return closestPtPointTetrahedron(Q, A, B, size);
		default:
			PX_ASSERT(0);
		}
		return support;
	}


	PX_NOALIAS PX_FORCE_INLINE Ps::aos::Vec3V GJKCPairDoSimplex(Ps::aos::Vec3V* PX_RESTRICT Q, Ps::aos::Vec3V* PX_RESTRICT A, Ps::aos::Vec3V* PX_RESTRICT B, PxI32* PX_RESTRICT aInd, PxI32* PX_RESTRICT bInd, 
		const Ps::aos::Vec3VArg support, PxU32& size)
	{
		using namespace Ps::aos;

		//const PxU32 tempSize = size;
		//calculate a closest from origin to the simplex
		switch(size)
		{
		case 1:
			{
				return support;
			}
		case 2:
			{
			return closestPtPointSegment(Q, size);
			}
		case 3:
			{
			return closestPtPointTriangle(Q, A, B, aInd, bInd, size);
			}
		case 4:
			return closestPtPointTetrahedron(Q, A, B, aInd, bInd, size);
		default:
			PX_ASSERT(0);
		}
		return support;
	}
}

}

#endif
