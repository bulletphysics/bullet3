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

#include "GuGJKPenetration.h"
#include "GuEPA.h"
#include "GuVecBox.h"
#include "GuGeometryUnion.h"

#include "GuConvexHelper.h"
#include "GuPCMShapeConvex.h"
#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuPCMContactGenUtil.h"


namespace physx
{

namespace Gu
{

	static void getIncidentPolygon(Ps::aos::Vec3V* pts, Ps::aos::Vec3V& faceNormal, const Ps::aos::Vec3VArg axis, const Ps::aos::PsMatTransformV& transf1To0, 
							   const Ps::aos::Vec3VArg extents)
	{

		using namespace Ps::aos;

		const FloatV zero = FZero();

		FloatV ex = V3GetX(extents);
		FloatV ey = V3GetY(extents);
		FloatV ez = V3GetZ(extents);

		const Vec3V u0 = transf1To0.getCol0();
		const Vec3V u1 = transf1To0.getCol1();
		const Vec3V u2 = transf1To0.getCol2();

		//calcaulte the insident face for b
		const FloatV d0 = V3Dot(u0, axis);
		const FloatV d1 = V3Dot(u1, axis);
		const FloatV d2 = V3Dot(u2, axis);

		const FloatV absd0 = FAbs(d0);
		const FloatV absd1 = FAbs(d1);
		const FloatV absd2 = FAbs(d2);

		Vec3V r0, r1, r2;


		if(FAllGrtrOrEq(absd0, absd1) && FAllGrtrOrEq(absd0, absd2))
		{
			//the incident face is on u0
			const BoolV con = FIsGrtr(d0, zero);
			faceNormal = V3Sel(con, V3Neg(u0), u0);
			ex = FSel(con, FNeg(ex), ex);
			r0 = V3Scale(u0, ex);
			r1 = V3Scale(u1, ey);
			r2 = V3Scale(u2, ez);

			const Vec3V temp0 = V3Add(transf1To0.p, r0);
			const Vec3V temp1 = V3Add(r1, r2);
			const Vec3V temp2 = V3Sub(r1, r2);
			

			pts[0] = V3Add(temp0, temp1);	// (-x/x,  y,  z)
			pts[1] = V3Add(temp0, temp2);	// (-x/x,  y, -z)
			pts[2] = V3Sub(temp0, temp1);	// (-x/x, -y, -z)
			pts[3] = V3Sub(temp0, temp2);	// (-x/x, -y,  z)

		}
		else if(FAllGrtrOrEq(absd1, absd2))
		{
			//the incident face is on u1
			const BoolV con = FIsGrtr(d1, zero);
			faceNormal = V3Sel(con, V3Neg(u1), u1);
			ey = FSel(con, FNeg(ey), ey);
			r0 = V3Scale(u0, ex);
			r1 = V3Scale(u1, ey);
			r2 = V3Scale(u2, ez);

			const Vec3V temp0 = V3Add(transf1To0.p, r1);
			const Vec3V temp1 = V3Add(r0, r2);
			const Vec3V temp2 = V3Sub(r0, r2);
			
			pts[0] = V3Add(temp0, temp1);	// (x,  -y/y,  z)
			pts[1] = V3Add(temp0, temp2);	// (x,  -y/y, -z)
			pts[2] = V3Sub(temp0, temp1);	// (-x, -y/y, -z)
			pts[3] = V3Sub(temp0, temp2);	// (-x, -y/y,  z)

		}
		else
		{
			//the incident face is on u2
			const BoolV con = FIsGrtr(d2, zero);
			faceNormal = V3Sel(con, V3Neg(u2), u2);
			ez = FSel(con, FNeg(ez), ez);
			r0 = V3Scale(u0, ex);
			r1 = V3Scale(u1, ey);
			r2 = V3Scale(u2, ez);

			const Vec3V temp0 = V3Add(transf1To0.p, r2);
			const Vec3V temp1 = V3Add(r0, r1);
			const Vec3V temp2 = V3Sub(r0, r1);
			
			pts[0] = V3Add(temp0, temp1);	// ( x,   y,  z)
			pts[1] = V3Add(temp0, temp2);	// ( x,  -y,  z)
			pts[2] = V3Sub(temp0, temp1);	// (-x,  -y,  z)
			pts[3] = V3Sub(temp0, temp2);	// (-x,   y,  z)
		}
	}


	//p0 and p1 is in the local space of AABB
	static bool intersectSegmentAABB(const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg d, const Ps::aos::Vec3VArg max, const Ps::aos::Vec3VArg min, Ps::aos::FloatV& tmin, Ps::aos::FloatV& tmax)
	{
		using namespace Ps::aos;
		
		const Vec3V eps = V3Load(1e-6f);
		const Vec3V absV = V3Abs(d);
		const FloatV one = FOne();
		const Vec3V zero = V3Zero();
		const Vec3V fMax = Vec3V_From_FloatV(FMax());

		FloatV tminf = FZero();
		FloatV tmaxf = one;
		const BoolV isParallel = V3IsGrtr(eps, absV);
		const BoolV isOutsideOfRange = BOr(V3IsGrtr(p0, max), V3IsGrtr(min, p0));
		//const BoolV isParallelAndOutOfRange = BAnd(isParallel, isOutsideOfRange);

		if(!BAllEqFFFF(BAnd(isParallel, isOutsideOfRange)))
		{
			return false;
		}

		const Vec3V odd = V3RecipFast(d);
		const Vec3V t1 = V3Sel(isParallel, zero, V3Mul(V3Sub(min, p0), odd));
		const Vec3V t2 = V3Sel(isParallel, fMax, V3Mul(V3Sub(max, p0), odd));
		
		const Vec3V tt1 = V3Min(t1, t2);
		const Vec3V tt2 = V3Max(t1, t2);

		const FloatV ft1 = V3ExtractMax(tt1);
		const FloatV ft2 = V3ExtractMin(tt2);

		tminf = FMax(ft1, tminf);
		tmaxf = FMin(tmaxf, ft2);

		tmin = tminf;
		tmax = tmaxf;

		const BoolV con0 = FIsGrtr(tminf, tmaxf);
		const BoolV con1 = FIsGrtr(tminf, one);
		const BoolV isNotIntersect = BOr(con0, con1);
		return BAllEqFFFF(isNotIntersect) == 1;
	}


	//pts, faceNormal and contact normal are in the local space of new space
	static void calculateContacts( const Ps::aos::FloatVArg extentX, const Ps::aos::FloatVArg extentY, Ps::aos::Vec3V* pts, const Ps::aos::Vec3VArg incidentFaceNormalInNew, const Ps::aos::Vec3VArg localNormal, Gu::PersistentContact* manifoldContacts, PxU32& numContacts, const Ps::aos::FloatVArg contactDist)
	{
		using namespace Ps::aos;

		const FloatV zero = FZero();
		const FloatV max = FMax();

		const FloatV nExtentX = FNeg(extentX);
		const FloatV nExtentY = FNeg(extentY);
		
		bool pPenetration[4];
		bool pArea[4];

		Vec3V bmin = V3Splat(max);
		Vec3V bmax = V3Neg(bmin);

		const Vec3V bound = V3Merge(extentX, extentY, max);


		//get the projection point of pts
		for(PxU32 i=0; i< 4; ++i)
		{
			bmin = V3Min(bmin, pts[i]);
			bmax = V3Max(bmax, pts[i]);
			const FloatV z = FNeg(V3GetZ(pts[i]));
			if(FAllGrtr(contactDist, z))
			{
	
				pPenetration[i] = true;

				const Vec3V absPt= V3Abs(pts[i]);
				const BoolV con = V3IsGrtrOrEq(bound, absPt);
				if(BAllEqTTTT(con))
				{
					pArea[i] = true;
				
					//Add the point to the manifold
					manifoldContacts[numContacts].mLocalPointA = V3SetZ(pts[i], zero); //transformNewTo0.transform(localPointA);
					manifoldContacts[numContacts].mLocalPointB = pts[i];//transform1ToNew.transformInv(pts[i]);
					manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), z);
				}
				else
				{
					pArea[i] = false;
				}
			}
			else
			{
				pPenetration[i] = false;
				pArea[i] = false;
			}

		}

		if(numContacts == 4)
			return;

		//if(pPenetration[0] && pPenetration[1] && pPenetration[2] && pPenetration[3])
		{
			//if(!pArea[0] || !pArea[1] || !pArea[2] || !pArea[3])
			{
				const FloatV denom = V3GetZ(incidentFaceNormalInNew);
				{
					const Vec3V q0 = V3Merge(extentX, extentY, zero);

					if(contains(pts, 4, q0, bmin, bmax))
					{
						const FloatV nom = V3Dot(incidentFaceNormalInNew, V3Sub(pts[0], q0));
						const FloatV t = FDiv(nom, denom);
						const FloatV pen = FNeg(t);
						if(FAllGrtr(contactDist, pen))
						{
							manifoldContacts[numContacts].mLocalPointA = q0;
							manifoldContacts[numContacts].mLocalPointB = V3SetZ(q0, t); 
							manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
						}
					}
				}
			
				{
					const Vec3V q0 = V3Merge(extentX, nExtentY, zero);
					if(contains(pts, 4, q0, bmin, bmax))
					{
						const FloatV nom = V3Dot(incidentFaceNormalInNew, V3Sub(pts[0], q0));
						const FloatV t = FDiv(nom, denom);
						const FloatV pen = FNeg(t);
						if(FAllGrtr(contactDist, pen))
						{
							manifoldContacts[numContacts].mLocalPointA = q0;
							manifoldContacts[numContacts].mLocalPointB = V3SetZ(q0, t);
							manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
						}
					}
				}
				
				{
					const Vec3V q0 = V3Merge( nExtentX, extentY, zero);
					if(contains(pts, 4, q0, bmin, bmax))
					{
						const FloatV nom = V3Dot(incidentFaceNormalInNew, V3Sub(pts[0], q0));
						const FloatV t = FDiv(nom, denom);
						const FloatV pen = FNeg(t);
						if(FAllGrtr(contactDist, pen))
						{
							manifoldContacts[numContacts].mLocalPointA = q0;
							manifoldContacts[numContacts].mLocalPointB = V3SetZ(q0, t);
							manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
						}
					}
				}
			
				{
					const Vec3V q0 = V3Merge(nExtentX, nExtentY, zero);
					
					if(contains(pts, 4, q0, bmin, bmax))
					{
						const FloatV nom = V3Dot(incidentFaceNormalInNew, V3Sub(pts[0], q0));
						const FloatV t = FDiv(nom, denom);
						const FloatV pen = FNeg(t);
						if(FAllGrtr(contactDist, pen))
						{
							manifoldContacts[numContacts].mLocalPointA = q0;
							manifoldContacts[numContacts].mLocalPointB = V3SetZ(q0, t);
							manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
						}
					}
				}
			}
		}


		   
		const Vec3V ext = V3Merge(extentX, extentY, max);
		const Vec3V negExt = V3Merge(nExtentX, nExtentY, FNeg(FAdd(contactDist, FEps())));

		for (PxU32 rStart = 0, rEnd = 3; rStart < 4; rEnd = rStart++)
		{
			const Vec3V p0 = pts[rStart];	
			const Vec3V p1 = pts[rEnd];

			if(!pPenetration[rStart] &&  !pPenetration[rEnd])
				continue;

			const bool con0 = pPenetration[rStart] && pArea[rStart];
			const bool con1 = pPenetration[rEnd] && pArea[rEnd];
			if(con0 && con1)
				continue;

			//intersect t value with x plane
			const Vec3V p0p1 = V3Sub(p1, p0);

			FloatV tmin, tmax;
			if(Gu::intersectSegmentAABB(p0, p0p1, ext, negExt, tmin, tmax))
			{
				if(!con0)
				{
					const Vec3V intersectP = V3ScaleAdd(p0p1, tmin, p0);
					manifoldContacts[numContacts].mLocalPointA = V3SetZ(intersectP, zero);
					manifoldContacts[numContacts].mLocalPointB = intersectP;
					manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), FNeg(V3GetZ(intersectP)));
				}
				if(!con1)
				{
					const Vec3V intersectP = V3ScaleAdd(p0p1, tmax, p0);
					manifoldContacts[numContacts].mLocalPointA = V3SetZ(intersectP, zero);
					manifoldContacts[numContacts].mLocalPointB =  intersectP;
					manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), FNeg(V3GetZ(intersectP)));
				}
			}
		}
}


static PxU32 doBoxBoxGenerateContacts(const Ps::aos::Vec3VArg box0Extent, const Ps::aos::Vec3VArg box1Extent, const Ps::aos::PsMatTransformV& transform0, const Ps::aos::PsMatTransformV& transform1, const Ps::aos::FloatVArg contactDist, Gu::PersistentContact* manifoldContacts, PxU32& numContacts)
{
	using namespace Ps::aos;

	const FloatV ea0 = V3GetX(box0Extent);
	const FloatV ea1 = V3GetY(box0Extent);
	const FloatV ea2 = V3GetZ(box0Extent);

	const FloatV eb0 = V3GetX(box1Extent);
	const FloatV eb1 = V3GetY(box1Extent);
	const FloatV eb2 = V3GetZ(box1Extent);


	const PsMatTransformV transform1To0 = transform0.transformInv(transform1);
	const Mat33V rot0To1 =M33Trnsps(transform1To0.rot);

	const Vec3V uEps = V3Load(1e-6f); 

	const FloatV zero = FZero();

	const FloatV tx = V3GetX(transform1To0.p);
	const FloatV ty = V3GetY(transform1To0.p);
	const FloatV tz = V3GetZ(transform1To0.p);
	const Vec3V col0 = transform1To0.getCol0();
	const Vec3V col1 = transform1To0.getCol1();
	const Vec3V col2 = transform1To0.getCol2();

	const Vec3V abs1To0Col0 = V3Add(V3Abs(col0), uEps);
	const Vec3V abs1To0Col1 = V3Add(V3Abs(col1), uEps);
	const Vec3V abs1To0Col2 = V3Add(V3Abs(col2), uEps);

	const Vec3V abs0To1Col0 = V3Add(V3Abs(rot0To1.col0), uEps);
	const Vec3V abs0To1Col1 = V3Add(V3Abs(rot0To1.col1), uEps);
	const Vec3V abs0To1Col2 = V3Add(V3Abs(rot0To1.col2), uEps);

	
	FloatV sign[6];
	FloatV overlap[6];

	FloatV ra, rb, radiusSum;
	//ua0
	{
		
		sign[0] = tx;

		const Vec3V vtemp3 = V3Mul(abs0To1Col0, box1Extent);
		rb = FAdd(V3GetX(vtemp3), FAdd(V3GetY(vtemp3), V3GetZ(vtemp3)));

		radiusSum = FAdd(ea0, rb);
		overlap[0] =  FAdd(FSub(radiusSum, FAbs(sign[0])), contactDist);
		if(FAllGrtr(zero, overlap[0]))
			return false;
	}

	//ua1
	{
		sign[1] = ty;

		const Vec3V vtemp3 = V3Mul(abs0To1Col1, box1Extent);
		rb = FAdd(V3GetX(vtemp3), FAdd(V3GetY(vtemp3), V3GetZ(vtemp3)));

		radiusSum = FAdd(ea1, rb);
		overlap[1] =  FAdd(FSub(radiusSum, FAbs(sign[1])), contactDist);
		if(FAllGrtr(zero, overlap[1]))
			return false;

	}


	//ua2	
	{
		sign[2] = tz;
		ra = ea2;

		const Vec3V vtemp3 = V3Mul(abs0To1Col2, box1Extent);
		rb = FAdd(V3GetX(vtemp3), FAdd(V3GetY(vtemp3), V3GetZ(vtemp3)));

		radiusSum = FAdd(ea2, rb);
		overlap[2] =  FAdd(FSub(radiusSum, FAbs(sign[2])), contactDist);
		if(FAllGrtr(zero, overlap[2]))
			return false;
		
	}
   
	//ub0 
	{
		sign[3] = V3Dot(transform1To0.p, col0);

		const Vec3V vtemp3 = V3Mul(abs1To0Col0, box0Extent);
		ra =  FAdd(V3GetX(vtemp3), FAdd(V3GetY(vtemp3), V3GetZ(vtemp3)));

		radiusSum = FAdd(ra, eb0);
		overlap[3] =  FAdd(FSub(radiusSum, FAbs(sign[3])), contactDist);
		if(FAllGrtr(zero, overlap[3]))
			return false;
	
	}

	//ub1
	{
		sign[4] = V3Dot(transform1To0.p, col1);

		const Vec3V vtemp3 = V3Mul(abs1To0Col1, box0Extent);
		ra =  FAdd(V3GetX(vtemp3), FAdd(V3GetY(vtemp3), V3GetZ(vtemp3)));

		radiusSum = FAdd(ra, eb1);
		overlap[4] =  FAdd(FSub(radiusSum, FAbs(sign[4])), contactDist);
		if(FAllGrtr(zero, overlap[4]))
			return false;
	}

	//ub2
	{
		sign[5] = V3Dot(transform1To0.p, col2);

		const Vec3V vtemp3 = V3Mul(abs1To0Col2, box0Extent);
		ra =  FAdd(V3GetX(vtemp3), FAdd(V3GetY(vtemp3), V3GetZ(vtemp3)));
		
		radiusSum = FAdd(ra, eb2);
		overlap[5] =  FAdd(FSub(radiusSum, FAbs(sign[5])), contactDist);
		if(FAllGrtr(zero, overlap[5]))
			return false;
	}


	//ua0 X ub0
	{
		//B into A's space, ua0Xub0[0,-b3, b2]
		const FloatV absSign = FAbs(FSub(FMul(V3GetY(col0), tz), FMul(V3GetZ(col0), ty)));

		//B into A's space, ua0Xub0[0,-b3, b2]
		const FloatV vtemp0 = FMul(V3GetZ(abs1To0Col0), ea1);
		const FloatV vtemp1 = FMul(V3GetY(abs1To0Col0), ea2);
		ra = FAdd(vtemp0, vtemp1);

		//A into B's space, ua0Xub0[0, a3, -a2]
		const FloatV vtemp01 = FMul(V3GetZ(abs0To1Col0), eb1);
		const FloatV vtemp02 = FMul(V3GetY(abs0To1Col0), eb2);
		rb = FAdd(vtemp01, vtemp02);
		
		radiusSum = FAdd(FAdd(ra, rb), contactDist);
		if(FAllGrtr(absSign, radiusSum)) return false;
	}

	//ua0 X ub1
	{
		//B into A's space, ua0Xub0[0, -b3, b2]
		const FloatV absSign = FAbs(FSub(FMul(V3GetY(col1), tz), FMul(V3GetZ(col1), ty)));

		//B into A's space, ua0Xub0[0, -b3, b2]
		const FloatV vtemp0 = FMul(V3GetZ(abs1To0Col1), ea1);
		const FloatV vtemp1 = FMul(V3GetY(abs1To0Col1), ea2);
		ra = FAdd(vtemp0, vtemp1);

		//A into B's space, ua0Xub1[-a3, 0, a1]
		const FloatV vtemp01 = FMul(V3GetZ(abs0To1Col0), eb0);
		const FloatV vtemp02 = FMul(V3GetX(abs0To1Col0), eb2);
		rb = FAdd(vtemp01, vtemp02);
		
		radiusSum = FAdd(FAdd(ra, rb), contactDist);

		if(FAllGrtr(absSign, radiusSum)) return false;

	}

	//ua0 X ub2
	{
		//B into A's space, ua0Xub0[0, -b3, b2]
		const FloatV absSign = FAbs(FSub(FMul(V3GetY(col2), tz), FMul(V3GetZ(col2), ty)));


		//B into A's space, ua0Xub0[0, -b3, b2]
		const FloatV vtemp0 = FMul(V3GetZ(abs1To0Col2), ea1);
		const FloatV vtemp1 = FMul(V3GetY(abs1To0Col2), ea2);
		ra = FAdd(vtemp0, vtemp1);

		//A into B's space, ua0Xub1[a2, -a1, 0]
		const FloatV vtemp01 = FMul(V3GetY(abs0To1Col0), eb0);
		const FloatV vtemp02 = FMul(V3GetX(abs0To1Col0), eb1);
		rb = FAdd(vtemp01, vtemp02);
		
		radiusSum = FAdd(FAdd(ra, rb), contactDist);
		if(FAllGrtr(absSign, radiusSum)) return false;

	}

	//ua1 X ub0
	{
		//B into A's space, ua0Xub0[b3, 0, -b1]
		const FloatV absSign = FAbs(FSub(FMul(V3GetZ(col0), tx), FMul(V3GetX(col0), tz)));

		//B into A's space, ua0Xub0[b3, 0, -b1]
		const FloatV vtemp0 = FMul(V3GetZ(abs1To0Col0), ea0);
		const FloatV vtemp1 = FMul(V3GetX(abs1To0Col0), ea2);
		ra = FAdd(vtemp0, vtemp1);

		//A into B's space, ua0Xub1[0, a3, -a2]
		const FloatV vtemp01 = FMul(V3GetZ(abs0To1Col1), eb1);
		const FloatV vtemp02 = FMul(V3GetY(abs0To1Col1), eb2);
		rb = FAdd(vtemp01, vtemp02);
		
		radiusSum = FAdd(FAdd(ra, rb), contactDist);
		if(FAllGrtr(absSign, radiusSum))return false;

	}

	//ua1 X ub1
	{
		//B into A's space, ua0Xub0[b3, 0, -b1]
		const FloatV absSign = FAbs(FSub(FMul(V3GetZ(col1), tx), FMul(V3GetX(col1), tz)));

		//B into A's space, ua0Xub0[b3, 0, -b1]
		const FloatV vtemp0 = FMul(V3GetZ(abs1To0Col1), ea0);
		const FloatV vtemp1 = FMul(V3GetX(abs1To0Col1), ea2);
		ra = FAdd(vtemp0, vtemp1);

		//A into B's space, ua0Xub1[-a3, 0, -a1]
		const FloatV vtemp01 = FMul(V3GetZ(abs0To1Col1), eb0);
		const FloatV vtemp02 = FMul(V3GetX(abs0To1Col1), eb2);
		rb = FAdd(vtemp01, vtemp02);
		
		radiusSum = FAdd(FAdd(ra, rb), contactDist);
		if(FAllGrtr(absSign, radiusSum))return false;
	}

	//ua1 X ub2
	{
		//B into A's space, ua0Xub0[b3, 0, -b1]
		const FloatV absSign=FAbs(FSub(FMul(V3GetZ(col2), tx), FMul(V3GetX(col2), tz)));

		//B into A's space, ua0Xub0[b3, 0, -b1]
		const FloatV vtemp0 = FMul(V3GetZ(abs1To0Col2), ea0);
		const FloatV vtemp1 = FMul(V3GetX(abs1To0Col2), ea2);
		ra = FAdd(vtemp0, vtemp1);

		//A into B's space, ua0Xub1[a2, -a1, 0]
		const FloatV vtemp01 = FMul(V3GetY(abs0To1Col1), eb0);
		const FloatV vtemp02 = FMul(V3GetX(abs0To1Col1), eb1);
		rb = FAdd(vtemp01, vtemp02);
		
		radiusSum = FAdd(FAdd(ra, rb), contactDist);
		if(FAllGrtr(absSign, radiusSum))return false;
	}

	//ua2 X ub0
	{
		//B into A's space, ua2Xub0[-b2, b1, 0]
		const FloatV absSign = FAbs(FSub(FMul(V3GetX(col0), ty), FMul(V3GetY(col0), tx)));


		//B into A's space, ua2Xub0[-b2, b1, 0]
		const FloatV vtemp0 = FMul(V3GetY(abs1To0Col0), ea0);
		const FloatV vtemp1 = FMul(V3GetX(abs1To0Col0), ea1);
		ra = FAdd(vtemp0, vtemp1);

		//A into B's space, ua0Xub1[0, a3, -a2]
		const FloatV vtemp01 = FMul(V3GetZ(abs0To1Col2), eb1);
		const FloatV vtemp02 = FMul(V3GetY(abs0To1Col2), eb2);
		rb = FAdd(vtemp01, vtemp02);
		
		radiusSum = FAdd(FAdd(ra, rb), contactDist);
		if(FAllGrtr(absSign, radiusSum))return false;
	}

	//ua2 X ub1
	{
		//B into A's space, ua2Xub0[-b2, b1, 0]
		const FloatV absSign = FAbs(FSub(FMul(V3GetX(col1), ty), FMul(V3GetY(col1), tx)));

		//B into A's space, ua2Xub0[-b2, b1, 0]
		const FloatV vtemp0 = FMul(V3GetY(abs1To0Col1), ea0);
		const FloatV vtemp1 = FMul(V3GetX(abs1To0Col1), ea1);
		ra = FAdd(vtemp0, vtemp1);

		//A into B's space, ua0Xub1[-a3, 0, a1]
		const FloatV vtemp01 = FMul(V3GetZ(abs0To1Col2), eb0);
		const FloatV vtemp02 = FMul(V3GetX(abs0To1Col2), eb2);
		rb = FAdd(vtemp01, vtemp02);
		
		radiusSum = FAdd(FAdd(ra, rb), contactDist);
		if(FAllGrtr(absSign, radiusSum))return false;
	}

	//ua2 X ub2
	{
		//B into A's space, ua2Xub0[-b2, b1, 0]
		const FloatV absSign=FAbs(FSub(FMul(V3GetX(col2), ty), FMul(V3GetY(col2), tx)));

		//B into A's space, ua2Xub0[-b2, b1, 0]
		const FloatV vtemp0 = FMul(V3GetY(abs1To0Col2), ea0);
		const FloatV vtemp1 = FMul(V3GetX(abs1To0Col2), ea1);
		ra = FAdd(vtemp0, vtemp1);

		//A into B's space, ua0Xub1[a2, -a1, 0]
		const FloatV vtemp01 = FMul(V3GetY(abs0To1Col2), eb0);
		const FloatV vtemp02 = FMul(V3GetX(abs0To1Col2), eb1);
		rb = FAdd(vtemp01, vtemp02);
		
		radiusSum = FAdd(FAdd(ra, rb), contactDist);
		if(FAllGrtr(absSign, radiusSum))return false;
	}

	Vec3V mtd;

	PxU32 feature = 0;
	FloatV minOverlap = overlap[0];

	for(PxU32 i=1; i<6; ++i)
	{
		if(FAllGrtr(minOverlap, overlap[i]))
		{
			minOverlap = overlap[i];
			feature = i;
		}
	}


	PsMatTransformV newTransformV;
	const Vec3V axis00 = transform0.getCol0();
	const Vec3V axis01 = transform0.getCol1();
	const Vec3V axis02 = transform0.getCol2();
	const Vec3V axis10 = transform1.getCol0();
	const Vec3V axis11 = transform1.getCol1();
	const Vec3V axis12 = transform1.getCol2();

	Vec3V incidentFaceNormalInNew;
	Vec3V pts[4];
	bool flip = false;
	switch(feature)
	{
	case 0: //ua0
		{

			
			if(FAllGrtrOrEq(zero, sign[0]))	
			{
				mtd = axis00;
				newTransformV.rot.col0 = V3Neg(axis02);
				newTransformV.rot.col1 = axis01;
				newTransformV.rot.col2 = axis00;
				newTransformV.p = V3NegScaleSub(axis00, ea0, transform0.p);
			}
			else		
			{

				const Vec3V nAxis00 = V3Neg(axis00);
				mtd = nAxis00;
				newTransformV.rot.col0 = axis02;
				newTransformV.rot.col1 = axis01;
				newTransformV.rot.col2 = nAxis00;
				newTransformV.p = V3ScaleAdd(axis00, ea0, transform0.p);
				
			}
	
			const Ps::aos::PsMatTransformV transform1ToNew = newTransformV.transformInv(transform1);
			const Vec3V localNormal =newTransformV.rotateInv(mtd);
			getIncidentPolygon(pts, incidentFaceNormalInNew, V3Neg(localNormal), transform1ToNew, box1Extent);
		
			calculateContacts(ea2, ea1, pts, incidentFaceNormalInNew, localNormal, manifoldContacts, numContacts, contactDist);
			

			break;
		};
	case 1: //ua1
		{
			
			if(FAllGrtrOrEq(zero, sign[1]))	
			{
				mtd = axis01;
				newTransformV.rot.col0 = axis00;
				newTransformV.rot.col1 = V3Neg(axis02);
				newTransformV.rot.col2 = axis01;	
				newTransformV.p = V3NegScaleSub(axis01, ea1, transform0.p); 

			}
			else		
			{


				const Vec3V nAxis01 = V3Neg(axis01);
				mtd = nAxis01;
				newTransformV.rot.col0 = axis00;
				newTransformV.rot.col1 = axis02;
				newTransformV.rot.col2 = nAxis01;	
				newTransformV.p = V3ScaleAdd(axis01, ea1, transform0.p);
			}

			const Ps::aos::PsMatTransformV transform1ToNew = newTransformV.transformInv(transform1);
			const Vec3V localNormal =newTransformV.rotateInv(mtd);
			getIncidentPolygon(pts, incidentFaceNormalInNew, V3Neg(localNormal), transform1ToNew, box1Extent);

			calculateContacts(ea0, ea2, pts, incidentFaceNormalInNew, localNormal, manifoldContacts, numContacts, contactDist);

			break;

		};
	case 2: //ua2
		{

			if(FAllGrtrOrEq(zero, sign[2]))	
			{
				mtd = axis02;
				newTransformV.rot.col0 = axis00;
				newTransformV.rot.col1 = axis01;
				newTransformV.rot.col2 = axis02;	
				
				newTransformV.p = V3NegScaleSub(axis02, ea2, transform0.p); 
			}
			else		
			{
				const Vec3V nAxis02 = V3Neg(axis02);
				mtd = nAxis02;
				newTransformV.rot.col0 = axis00;
				newTransformV.rot.col1 = V3Neg(axis01);
				newTransformV.rot.col2 = nAxis02;
				newTransformV.p = V3ScaleAdd(axis02, ea2, transform0.p);
				
			}

			const Ps::aos::PsMatTransformV transform1ToNew = newTransformV.transformInv(transform1);
			const Vec3V localNormal =newTransformV.rotateInv(mtd);
			getIncidentPolygon(pts, incidentFaceNormalInNew, V3Neg(localNormal), transform1ToNew, box1Extent);

			calculateContacts(ea0, ea1, pts, incidentFaceNormalInNew, localNormal, manifoldContacts, numContacts, contactDist);

			
			break;
		};
	case 3: //ub0
		{
		
			flip = true;
			if(FAllGrtrOrEq(zero, sign[3]))	
			{
				mtd = axis10;
				newTransformV.rot.col0 = axis12;
				newTransformV.rot.col1 = axis11;
				newTransformV.rot.col2 = V3Neg(axis10);
				newTransformV.p = V3ScaleAdd(axis10, eb0, transform1.p); //transform0.p - extents0.x*axis00;
			}
			else		
			{
				mtd = V3Neg(axis10);
				newTransformV.rot.col0 = V3Neg(axis12);
				newTransformV.rot.col1 = axis11;
				newTransformV.rot.col2 = axis10;
				newTransformV.p =V3NegScaleSub(axis10, eb0, transform1.p);//transform0.p + extents0.x*axis00;
			}

			const Ps::aos::PsMatTransformV transform1ToNew = newTransformV.transformInv(transform0);
			const Vec3V localNormal =newTransformV.rotateInv(mtd);
			getIncidentPolygon(pts, incidentFaceNormalInNew, localNormal, transform1ToNew, box0Extent);

			calculateContacts(eb2, eb1, pts, incidentFaceNormalInNew, localNormal, manifoldContacts, numContacts, contactDist);
			
			break;
		};
	case 4: //ub1;
		{
			flip = true;
			if(FAllGrtrOrEq(zero, sign[4]))	
			{
				mtd = axis11;
				newTransformV.rot.col0 = axis10;
				newTransformV.rot.col1 = axis12;
				newTransformV.rot.col2 = V3Neg(axis11);
			
				newTransformV.p = V3ScaleAdd(axis11, eb1, transform1.p);
			
			}
			else		
			{
				mtd = V3Neg(axis11);

				newTransformV.rot.col0 = axis10;
				newTransformV.rot.col1 = V3Neg(axis12);
				newTransformV.rot.col2 = axis11;
				newTransformV.p = V3NegScaleSub(axis11, eb1, transform1.p); //transform0.p + extents0.x*axis00;
				
			}

			const Ps::aos::PsMatTransformV transform1ToNew = newTransformV.transformInv(transform0);
			const Vec3V localNormal =newTransformV.rotateInv(mtd);
			getIncidentPolygon(pts, incidentFaceNormalInNew, localNormal, transform1ToNew, box0Extent);
			calculateContacts(eb0, eb2, pts, incidentFaceNormalInNew, localNormal, manifoldContacts, numContacts, contactDist);
			break;
		}
	case 5: //ub2;
		{

			flip = true;
			if(FAllGrtrOrEq(zero, sign[5]))	
			{
				mtd = axis12;
				newTransformV.rot.col0 = axis10;
				newTransformV.rot.col1 = V3Neg(axis11);
				newTransformV.rot.col2 = V3Neg(axis12);
				newTransformV.p = V3ScaleAdd(axis12, eb2, transform1.p);
			}
			else		
			{
				mtd = V3Neg(axis12);
				
				newTransformV.rot.col0 = axis10;
				newTransformV.rot.col1 = axis11;
				newTransformV.rot.col2 = axis12;
				newTransformV.p = V3NegScaleSub(axis12, eb2, transform1.p);
			}

			const Ps::aos::PsMatTransformV transform1ToNew = newTransformV.transformInv(transform0);
			const Vec3V localNormal =newTransformV.rotateInv(mtd);
			getIncidentPolygon(pts, incidentFaceNormalInNew, localNormal, transform1ToNew, box0Extent);
		
			calculateContacts(eb0, eb1, pts,  incidentFaceNormalInNew, localNormal, manifoldContacts, numContacts, contactDist);
			break;
		};
	default:
		return false;
	}

	if(flip)
	{
		for(PxU32 i=0; i<numContacts; ++i)
		{
			const Vec3V localB = manifoldContacts[i].mLocalPointB;
			manifoldContacts[i].mLocalPointB = manifoldContacts[i].mLocalPointA;
			manifoldContacts[i].mLocalPointA = localB;
		}
	}
	const Ps::aos::PsMatTransformV transformNewTo1 = transform1.transformInv(newTransformV);
	const Ps::aos::PsMatTransformV transformNewTo0 = transform0.transformInv(newTransformV);
	//transform points to the local space of transform0 and transform1
	const Ps::aos::Vec3V localNormalInB = transformNewTo1.rotate(Vec3V_From_Vec4V(manifoldContacts[0].mLocalNormalPen));

	for(PxU32 i=0; i<numContacts; ++i)
	{
		manifoldContacts[i].mLocalPointA = transformNewTo0.transform(manifoldContacts[i].mLocalPointA);
		manifoldContacts[i].mLocalPointB = transformNewTo1.transform(manifoldContacts[i].mLocalPointB);
		manifoldContacts[i].mLocalNormalPen = V4SetW(localNormalInB, V4GetW(manifoldContacts[i].mLocalNormalPen));
	}

	return true;
}  



bool pcmContactBoxBox(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	using namespace Ps::aos;
	// Get actual shape data
	Gu::PersistentContactManifold& manifold = cache.getManifold();
	Ps::prefetchLine(&manifold, 256);
	const PxBoxGeometry& shapeBox0 = shape0.get<const PxBoxGeometry>();
	const PxBoxGeometry& shapeBox1 = shape1.get<const PxBoxGeometry>();

	PX_ASSERT(transform1.q.isSane());
	PX_ASSERT(transform0.q.isSane());


	const FloatV contactDist = FLoad(params.mContactDistance);
	const Vec3V boxExtents0 = V3LoadU(shapeBox0.halfExtents);
	const Vec3V boxExtents1 = V3LoadU(shapeBox1.halfExtents);
	
	//Transfer A into the local space of B
	const PsTransformV transf0 = loadTransformA(transform0);
	const PsTransformV transf1 = loadTransformA(transform1);
	const PsTransformV curRTrans(transf1.transformInv(transf0));
	const PsMatTransformV aToB(curRTrans);

	const PxReal toleranceLength = params.mToleranceLength;
	const FloatV boxMargin0 = Gu::CalculatePCMBoxMargin(boxExtents0, toleranceLength);
	const FloatV boxMargin1 = Gu::CalculatePCMBoxMargin(boxExtents1, toleranceLength);
	const FloatV minMargin = FMin(boxMargin0, boxMargin1);

	const PxU32 initialContacts = manifold.mNumContacts;
	const FloatV projectBreakingThreshold = FMul(minMargin, FLoad(0.8f));
	
	manifold.refreshContactPoints(aToB, projectBreakingThreshold, contactDist);

	const PxU32 newContacts = manifold.mNumContacts;

	const bool bLostContacts = (newContacts != initialContacts);
  
	PX_UNUSED(bLostContacts);

	if(bLostContacts || manifold.invalidate_BoxConvex(curRTrans, minMargin))	
	{
		
		manifold.setRelativeTransform(curRTrans);
		
		const PsMatTransformV transfV0(transf0);
		const PsMatTransformV transfV1(transf1);

		Gu::PersistentContact* manifoldContacts = PX_CP_TO_PCP(contactBuffer.contacts);
		PxU32 numContacts = 0;
	
		if(doBoxBoxGenerateContacts(boxExtents0, boxExtents1, transfV0, transfV1, contactDist, manifoldContacts, numContacts)) 
		{
			if(numContacts > 0)
			{
			
				manifold.addBatchManifoldContacts(manifoldContacts, numContacts, toleranceLength);
				const Vec3V worldNormal = V3Normalize(transfV1.rotate(Vec3V_From_Vec4V(manifold.mContactPoints[0].mLocalNormalPen)));
				manifold.addManifoldContactsToContactBuffer(contactBuffer, worldNormal, transfV1);
#if	PCM_LOW_LEVEL_DEBUG
				manifold.drawManifold(*renderOutput, transf0, transf1);
#endif
				return true;
			}
			else
			{
				const Vec3V zeroV = V3Zero();
				BoxV box0(zeroV, boxExtents0);
				BoxV box1(zeroV, boxExtents1);

				manifold.mNumWarmStartPoints = 0;
				RelativeConvex<BoxV> convexA(box0, aToB);
				LocalConvex<BoxV> convexB(box1);
				GjkOutput output;

				GjkStatus status = gjkPenetration<RelativeConvex<BoxV>, LocalConvex<BoxV> >(convexA, convexB, aToB.p, contactDist, true,
					manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);

				if(status == EPA_CONTACT)
				{
				
					RelativeConvex<BoxV> convexA1(box0, aToB);
					LocalConvex<BoxV> convexB1(box1);

					status=  epaPenetration(convexA1, convexB1, manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, 
						true, FLoad(toleranceLength), output);
				}

				if(status == GJK_CONTACT || status == EPA_CONTACT)
				{
					const FloatV replaceBreakingThreshold = FMul(minMargin, FLoad(0.05f));
					const Vec3V localPointA = aToB.transformInv(output.closestA);//curRTrans.transformInv(closestA);
					const Vec3V localPointB = output.closestB;
					const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(output.normal), output.penDep);
					numContacts += manifold.addManifoldPoint(localPointA, localPointB, localNormalPen, replaceBreakingThreshold);

					//transform the normal back to world space
					const Vec3V worldNormal = V3Normalize(transf1.rotate(output.normal));

					manifold.addManifoldContactsToContactBuffer(contactBuffer, worldNormal, transf1, contactDist);

#if	PCM_LOW_LEVEL_DEBUG
					manifold.drawManifold(*renderOutput, transf0, transf1);
#endif

					return true;
				}

			}
		}
	}
	else if(manifold.getNumContacts() > 0)
	{
		const Vec3V worldNormal =  manifold.getWorldNormal(transf1);
		manifold.addManifoldContactsToContactBuffer(contactBuffer, worldNormal, transf1, contactDist);
#if	PCM_LOW_LEVEL_DEBUG
		manifold.drawManifold(*renderOutput, transf0, transf1);
#endif
		return true;
	}
	
	return false;
}

}
}
