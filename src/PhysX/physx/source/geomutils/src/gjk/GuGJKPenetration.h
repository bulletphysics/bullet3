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

#ifndef GU_GJK_PENETRATION_H
#define GU_GJK_PENETRATION_H


#include "GuConvexSupportTable.h"
#include "GuGJKSimplex.h"
#include "GuVecConvexHullNoScale.h"
#include "GuGJKUtil.h"
#include "PsUtilities.h"
#include "GuGJKType.h"

#define	GJK_VALIDATE 0


namespace physx
{
namespace Gu
{

	class ConvexV;


	PX_FORCE_INLINE void assignWarmStartValue(PxU8* PX_RESTRICT aIndices, PxU8* PX_RESTRICT bIndices, PxU8& size_, PxI32* PX_RESTRICT aInd, PxI32* PX_RESTRICT bInd,  PxU32 size )
	{
		if(aIndices)
		{
			PX_ASSERT(bIndices);
			size_ = Ps::to8(size);
			for(PxU32 i=0; i<size; ++i)
			{
				aIndices[i] = Ps::to8(aInd[i]);
				bIndices[i] = Ps::to8(bInd[i]);
			}
		}
	}


	PX_FORCE_INLINE void validateDuplicateVertex(const Ps::aos::Vec3V* Q, const Ps::aos::Vec3VArg support, const PxU32 size)
	{
		using namespace Ps::aos;

		const FloatV eps = FEps();
		//Get rid of the duplicate point
		BoolV match = BFFFF();
		for(PxU32 na = 0; na < size; ++na)
		{
			Vec3V dif = V3Sub(Q[na], support);
			match = BOr(match, FIsGrtr(eps, V3Dot(dif, dif)));	
		}

		//we have duplicate code
		if(BAllEqTTTT(match))
		{
			PX_ASSERT(0);
		}
	}


	//*Each convex has
	//*         a support function
	//*         a margin	- if the shape is sphere/capsule, margin is the radius
	//*         a minMargin - some percentage of margin, which is used to determine the termination condition for gjk
	
	//*We'll report:
	//*         GJK_NON_INTERSECT if the minimum distance between the shapes is greater than the sum of the margins and the the contactDistance
	//*         EPA_CONTACT if shapes overlap. We treat sphere/capsule as a point/a segment and we shrunk other shapes by 10% of margin
	//*         GJK_CONTACT if the algorithm converges, and the distance between the shapes is less than the sum of the margins plus the contactDistance. In this case we return the closest points found
	//*         GJK_DEGENERATE if the algorithm doesn't converge, we return this flag to indicate the normal and closest point we return might not be accurated
	template<typename ConvexA, typename ConvexB >
	PX_NOINLINE GjkStatus gjkPenetration(const ConvexA& a, const ConvexB& b, const Ps::aos::Vec3VArg initialSearchDir, const Ps::aos::FloatVArg contactDist, const bool takeCoreShape, 
		PxU8* PX_RESTRICT aIndices, PxU8* PX_RESTRICT bIndices, PxU8& warmStartSize,
		GjkOutput& output)
	{
		using namespace Ps::aos;

		//ML: eps is the threshold that uses to determine whether two (shrunk) shapes overlap. We calculate eps2 based on 10% of the minimum margin of two shapes
		const FloatV minMargin = FMin(a.ConvexA::getMinMargin(), b.ConvexB::getMinMargin());
		const FloatV eps = FMul(minMargin, FLoad(0.1f));

		//const FloatV eps2 = FMul(_eps2, _eps2);
		//ML: epsRel2 is the square of 0.01. This is used to scale the square distance of a closest point to origin to determine whether two shrunk shapes overlap in the margin, but
		//they don't overlap.
		//const FloatV epsRel2 = FMax(FLoad(0.0001), eps2); 

		// ML:epsRel is square value of 1.5% which applied to the distance of a closest point(v) to the origin.
		// If |v|- v/|v|.dot(w) < epsRel*|v|=>(|v|*(1-epsRel) < v/|v|.dot(w)),
		// two shapes are clearly separated, GJK terminate and return non intersect.
		// This adjusts the termination condition based on the length of v
		// which avoids ill-conditioned terminations. 
		const FloatV epsRel = FLoad(0.000225f);//1.5%.
		const FloatV relDif = FSub(FOne(), epsRel);

		const FloatV zero = FZero();
		
		//capsule/sphere will have margin which is its radius
		const FloatV marginA = a.getMargin();
		const FloatV marginB = b.getMargin();

		const BoolV aQuadratic = a.isMarginEqRadius();
		const BoolV bQuadratic = b.isMarginEqRadius();
		const FloatV tMarginA = FSel(aQuadratic, marginA, zero);
		const FloatV tMarginB = FSel(bQuadratic, marginB, zero);

		const FloatV sumMargin = FAdd(tMarginA, tMarginB);
		const FloatV sumExpandedMargin = FAdd(sumMargin, contactDist);
		
		FloatV dist = FMax();
		FloatV prevDist = dist;
		const Vec3V zeroV = V3Zero();
		Vec3V prevClos = zeroV;

		const BoolV bTrue = BTTTT();
		BoolV bNotTerminated = bTrue;
		BoolV bNotDegenerated = bTrue;
		Vec3V closest;

		Vec3V Q[4];
		Vec3V A[4];
		Vec3V B[4];
		PxI32 aInd[4];
		PxI32 bInd[4];
		Vec3V supportA = zeroV, supportB = zeroV, support=zeroV;
		Vec3V v;

		PxU32 size = 0;//_size;
		

		//ML: if _size!=0, which means we pass in the previous frame simplex so that we can warm-start the simplex. 
		//In this case, GJK will normally terminate in one iteration
		if(warmStartSize != 0)
		{
			for(PxU32 i=0; i<warmStartSize; ++i)
			{
				aInd[i] = aIndices[i];
				bInd[i] = bIndices[i];

				//de-virtualize
				supportA = a.ConvexA::supportPoint(aIndices[i]);
				supportB = b.ConvexB::supportPoint(bIndices[i]);

				support = V3Sub(supportA, supportB);

#if	GJK_VALIDATE
				//ML: this is used to varify whether we will have duplicate vertices in the warm-start value. If this function get triggered,
				//this means something isn't right and we need to investigate
				validateDuplicateVertex(Q, support, size);
#endif
				A[size] =  supportA;
				B[size] =  supportB;
				Q[size++] =	support;
			}

			//run simplex solver to determine whether the point is closest enough so that gjk can terminate
			closest = GJKCPairDoSimplex(Q, A, B, aInd, bInd, support, size);
			dist = V3Length(closest);
			//sDist = V3Dot(closest, closest);
			v = V3ScaleInv(closest, dist);
			prevDist = dist;
			prevClos = closest;
			
			bNotTerminated = FIsGrtr(dist, eps);
		}
		else
		{
			//const Vec3V _initialSearchDir = V3Sub(a.getCenter(), b.getCenter());
			closest = V3Sel(FIsGrtr(V3Dot(initialSearchDir, initialSearchDir), zero), initialSearchDir, V3UnitX());
			v = V3Normalize(closest);
		}
		
		
		// ML : termination condition
		//(1)two shapes overlap. GJK will terminate based on sq(v) < eps2 and indicate that two shapes are overlapping.
		//(2)two shapes are separated. If sq(vw) > sqMargin * sq(v), which means the original objects do not intesect, GJK terminate with GJK_NON_INTERSECT. 
		//(3)two shapes don't overlap. However, they interect within margin distance. if sq(v)- vw < epsRel2*sq(v), this means the shrunk shapes interect in the margin, 
		//   GJK terminate with GJK_CONTACT.
		while(BAllEqTTTT(bNotTerminated))
		{
			//prevDist, prevClos are used to store the previous iteration's closest point and the square distance from the closest point
			//to origin in Mincowski space
			prevDist = dist;
			prevClos = closest;

			//de-virtualize
			supportA = a.ConvexA::support(V3Neg(closest), aInd[size]);
			supportB = b.ConvexB::support(closest, bInd[size]);

			//calculate the support point
			support = V3Sub(supportA, supportB);

			const FloatV vw = V3Dot(v, support);
			if(FAllGrtr(vw, sumExpandedMargin))
			{
				assignWarmStartValue(aIndices, bIndices, warmStartSize, aInd, bInd, size);
				return GJK_NON_INTERSECT;
			}
			
			//if(FAllGrtr(FMul(epsRel, dist), FSub(dist, vw)))
			if(FAllGrtr(vw, FMul(dist, relDif)))
			{
				assignWarmStartValue(aIndices, bIndices, warmStartSize, aInd, bInd, size);
				PX_ASSERT(FAllGrtr(dist, FEps()));
				//const Vec3V n = V3ScaleInv(closest, dist);//normalise
				output.normal = v; 
				Vec3V closA, closB;
				getClosestPoint(Q, A, B, closest, closA, closB, size);
				//ML: if one of the shape is sphere/capsule and the takeCoreShape flag is true, the contact point for sphere/capsule will be the sphere center or a point in the
				//capsule segment. This will increase the stability for the manifold recycling code. Otherwise, we will return a contact point on the surface for sphere/capsule
				//while the takeCoreShape flag is set to be false
				if(takeCoreShape)
				{
					output.closestA= closA;
					output.closestB = closB;
					output.penDep = dist;
					
				}
				else
				{
					//This is for capsule/sphere want to take the surface point. For box/convex,
					//we need to get rid of margin 
					output.closestA = V3NegScaleSub(v, tMarginA, closA);
					output.closestB = V3ScaleAdd(v, tMarginB, closB);
					output.penDep = FSub(dist, sumMargin);
				}

				return GJK_CONTACT;
			}

			A[size] = supportA;
			B[size] = supportB;
			Q[size++]=support;
			PX_ASSERT(size <= 4);

			//calculate the closest point between two convex hull
			closest = GJKCPairDoSimplex(Q, A, B, aInd, bInd, support, size);

			dist = V3Length(closest);
			v = V3ScaleInv(closest, dist);

			bNotDegenerated = FIsGrtr(prevDist, dist);
			bNotTerminated = BAnd(FIsGrtr(dist, eps), bNotDegenerated);
		}

		if(BAllEqFFFF(bNotDegenerated))
		{
			assignWarmStartValue(aIndices, bIndices, warmStartSize, aInd, bInd, size-1);
			
			//Reset back to older closest point
			dist = prevDist;
			closest = prevClos;//V3Sub(closA, closB);
			Vec3V closA, closB;
			getClosestPoint(Q, A, B, closest, closA, closB, size);

			//PX_ASSERT(FAllGrtr(dist, FEps()));
			const Vec3V n = V3ScaleInv(prevClos, prevDist);//normalise
			output.normal = n;

			output.searchDir = v;

			if(takeCoreShape)
			{
				output.closestA = closA;
				output.closestB = closB;
				output.penDep = dist;
			}
			else
			{
				//This is for capsule/sphere want to take the surface point. For box/convex,
				//we need to get rid of margin
				output.closestA = V3NegScaleSub(n, tMarginA, closA);
				output.closestB = V3ScaleAdd(n, tMarginB, closB);
				output.penDep = FSub(dist, sumMargin);
				if (FAllGrtrOrEq(sumMargin, dist))
					return GJK_CONTACT;
			}
			
			return GJK_DEGENERATE;  
		}
		else 
		{
			//this two shapes are deeply intersected with each other, we need to use EPA algorithm to calculate MTD
			assignWarmStartValue(aIndices, bIndices, warmStartSize, aInd, bInd, size);
			return EPA_CONTACT;
			
		}
	}

}//Gu

}//physx

#endif
