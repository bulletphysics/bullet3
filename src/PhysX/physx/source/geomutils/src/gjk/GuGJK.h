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

#ifndef GU_GJK_H
#define GU_GJK_H


#include "GuGJKType.h"
#include "GuGJKUtil.h"
#include "GuConvexSupportTable.h"
#include "GuGJKSimplex.h"
#include "PsFPU.h"

#define	GJK_SEPERATING_AXIS_VALIDATE 0

namespace physx
{
namespace Gu
{

	class ConvexV;

#if	GJK_SEPERATING_AXIS_VALIDATE
	template<typename ConvexA, typename ConvexB>
	static void validateSeparatingAxis(const ConvexA& a, const ConvexB& b, const Ps::aos::Vec3VArg separatingAxis)
	{
		using namespace Ps::aos;
		const Vec3V minV0 = a.ConvexA::support(V3Neg(separatingAxis));
		const Vec3V maxV0 = a.ConvexA::support(separatingAxis); 

		const Vec3V minV1 = b.ConvexB::support(V3Neg(separatingAxis));
		const Vec3V maxV1 = b.ConvexB::support(separatingAxis);

		const FloatV min0 = V3Dot(minV0, separatingAxis);
		const FloatV max0 = V3Dot(maxV0, separatingAxis);

		const FloatV min1 = V3Dot(minV1, separatingAxis);
		const FloatV max1 = V3Dot(maxV1, separatingAxis);
				
		PX_ASSERT(FAllGrtr(min1, max0) || FAllGrtr(min0, max1));
	}
#endif


	/*
		
		initialSearchDir	:initial search direction in the mincowsky sum, it should be in the local space of ConvexB
		closestA			:it is the closest point in ConvexA in the local space of ConvexB if acceptance threshold is sufficent large. Otherwise, it will be garbage 
		closestB			:it is the closest point in ConvexB in the local space of ConvexB if acceptance threshold is sufficent large. Otherwise, it will be garbage
		normal				:normal pointing from ConvexA to ConvexB in the local space of ConvexB if acceptance threshold is sufficent large. Otherwise, it will be garbage
		distance			:the distance of the closest points between ConvexA and ConvexB if acceptance threshold is sufficent large. Otherwise, it will be garbage
		contactDist			:the distance which we will generate contact information if ConvexA and ConvexB are both separated within contactDist
	*/

	//*Each convex has
	//*         a support function
	//*         a margin	- the amount by which we shrunk the shape for a convex or box. If the shape are sphere/capsule, margin is the radius
	//*         a minMargin - some percentage of margin, which is used to determine the termination condition for gjk
	
	//*We'll report:
	//*         GJK_NON_INTERSECT if the sign distance between the shapes is greater than the sum of the margins and the the contactDistance
	//*         GJK_CLOSE if the minimum distance between the shapes is less than the sum of the margins and the the contactDistance
	//*         GJK_CONTACT if the two shapes are overlapped with each other
	template<typename ConvexA, typename ConvexB>
	GjkStatus gjk(const ConvexA& a, const ConvexB& b, const Ps::aos::Vec3V& initialSearchDir, const Ps::aos::FloatV& contactDist, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB, Ps::aos::Vec3V& normal, 
		Ps::aos::FloatV& distance)
	{
		using namespace Ps::aos;
		Vec3V Q[4];
		Vec3V A[4];
		Vec3V B[4];

		const FloatV zero = FZero();
		PxU32 size=0;

		//const Vec3V _initialSearchDir	 = aToB.p;
		Vec3V closest = V3Sel(FIsGrtr(V3Dot(initialSearchDir, initialSearchDir), zero), initialSearchDir, V3UnitX());
		Vec3V v = V3Normalize(closest);

		// ML: eps2 is the square value of an epsilon value which applied in the termination condition for two shapes overlap.
		// GJK will terminate based on sq(v) < eps and indicate that two shapes are overlapping.
		// we calculate the eps based on 10% of the minimum margin of two shapes
		const FloatV tenPerc = FLoad(0.1f);
		const FloatV minMargin = FMin(a.getMinMargin(), b.getMinMargin());
		const FloatV eps = FMul(minMargin, tenPerc);
		
		// ML:epsRel is square value of 1.5% which applied to the distance of a closest point(v) to the origin.
		// If |v|- v/|v|.dot(w) < epsRel*|v|,
		// two shapes are clearly separated, GJK terminate and return non intersect.
		// This adjusts the termination condition based on the length of v
		// which avoids ill-conditioned terminations. 
		const FloatV epsRel = FLoad(0.000225f);//1.5%.

		FloatV dist = FMax();
		FloatV prevDist;
		Vec3V prevClos, prevDir;
		
		const BoolV bTrue = BTTTT();
		BoolV bNotTerminated = bTrue;
		BoolV bNotDegenerated = bTrue;
		
		//ML: we treate sphere as a point and capsule as segment in the support function, so that we need to add on radius
		const BoolV aQuadratic = a.isMarginEqRadius();
		const BoolV bQuadratic = b.isMarginEqRadius();

		const FloatV sumMargin = FAdd(FSel(aQuadratic, a.getMargin(), zero), FSel(bQuadratic, b.getMargin(), zero));
		const FloatV separatingDist = FAdd(sumMargin, contactDist);
		const FloatV relDif = FSub(FOne(), epsRel);

		do
		{
			prevDist = dist;
			prevClos = closest;
			prevDir = v;

			//de-virtualize, we don't need to use a normalize direction to get the support point
			//this will allow the cpu better pipeline the normalize calculation while it does the
			//support map
			const Vec3V supportA=a.ConvexA::support(V3Neg(closest));
			const Vec3V supportB=b.ConvexB::support(closest);
			
			//calculate the support point
			const Vec3V support = V3Sub(supportA, supportB);

			const FloatV signDist = V3Dot(v, support);
			
			if(FAllGrtr(signDist, separatingDist))
			{
				//ML:gjk found a separating axis for these two objects and the distance is large than the seperating distance, gjk might not converage so that
				//we won't generate contact information
#if	GJK_SEPERATING_AXIS_VALIDATE
					validateSeparatingAxis(a, b, v);
#endif
				return GJK_NON_INTERSECT;
			}

			const BoolV con = BAnd(FIsGrtr(signDist, sumMargin), FIsGrtr(signDist, FMul(relDif, dist)));

			if(BAllEqTTTT(con))
			{
				//ML:: gjk converage and we get the closest point information
				Vec3V closA, closB;
				//normal point from A to B
				const Vec3V n = V3Neg(v);
				getClosestPoint(Q, A, B, closest, closA, closB, size);
				closestA = V3Sel(aQuadratic, V3ScaleAdd(n, a.getMargin(), closA), closA);
				closestB = V3Sel(bQuadratic, V3NegScaleSub(n, b.getMargin(), closB), closB);
				distance = FMax(zero, FSub(dist, sumMargin));
				normal = n;//V3Normalize(V3Neg(closest));
				return GJK_CLOSE;
			}

			PX_ASSERT(size < 4);
			A[size]=supportA;
			B[size]=supportB;
			Q[size++]=support;  

			//calculate the closest point between two convex hull
			closest = GJKCPairDoSimplex(Q, A, B, support, size);
			
			dist = V3Length(closest);
			v = V3ScaleInv(closest, dist);
			bNotDegenerated = FIsGrtr(prevDist, dist);
			bNotTerminated = BAnd(FIsGrtr(dist, eps), bNotDegenerated);
		}while(BAllEqTTTT(bNotTerminated));
		
		if(BAllEqTTTT(bNotDegenerated))
		{
			//GJK_CONTACT
			distance = zero;
			return GJK_CONTACT;
		}

		//GJK degenerated, use the previous closest point
		const FloatV acceptancePerc = FLoad(0.2f);
		const FloatV acceptanceMargin = FMul(acceptancePerc, FMin(a.getMargin(), b.getMargin()));
		const FloatV acceptanceDist = FSel(FIsGrtr(sumMargin, zero), sumMargin, acceptanceMargin);
		Vec3V closA, closB;
		const Vec3V n = V3Neg(prevDir);//V3Normalize(V3Neg(prevClos));
		getClosestPoint(Q, A, B, prevClos, closA, closB, size);
		closestA = V3Sel(aQuadratic, V3ScaleAdd(n, a.getMargin(), closA), closA);
		closestB = V3Sel(bQuadratic, V3NegScaleSub(n, b.getMargin(), closB), closB);
		normal =  n;
		dist = FMax(zero, FSub(prevDist, sumMargin));
		distance = dist;
	
		return FAllGrtr(dist, acceptanceDist) ? GJK_CLOSE: GJK_CONTACT;
	}
}

}

#endif
