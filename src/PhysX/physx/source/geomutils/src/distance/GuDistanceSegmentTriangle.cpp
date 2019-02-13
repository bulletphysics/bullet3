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

#include "PsIntrinsics.h"
#include "GuDistanceSegmentTriangle.h"
#include "GuDistanceSegmentTriangleSIMD.h"
#include "GuDistancePointTriangle.h"
#include "GuDistancePointTriangleSIMD.h"
#include "GuDistanceSegmentSegment.h"
#include "GuDistanceSegmentSegmentSIMD.h"
#include "GuBarycentricCoordinates.h"

using namespace physx;
using namespace Gu;

// ptchernev: 
// The Magic Software code uses a relative error test for parallel case.
// The Novodex code does not presumably as an optimization. 
// Since the Novodex code is working in the trunk I see no reason 
// to reintroduce the relative error test here.

// PT: this might just be because the relative error test has been added
// after we grabbed the code. I don't remember making this change. A good
// idea would be NOT to refactor Magic's code, to easily grab updated
// versions from the website.............................................

// ptchernev:
// The code has been modified to use a relative error test since the absolute
// test would break down for small geometries. (TTP 4021)

static PX_FORCE_INLINE void updateClosestHit(	PxReal fSqrDist0, PxReal fR0, PxReal fS0, PxReal fT0,
												PxReal& fSqrDist, PxReal& fR, PxReal& fS, PxReal& fT)
{
	if(fSqrDist0 < fSqrDist)
	{
		fSqrDist = fSqrDist0;
		fR = fR0;
		fS = fS0;
		fT = fT0;
	}
}

PxReal Gu::distanceSegmentTriangleSquared(	const PxVec3& origin, const PxVec3& dir,
											const PxVec3& p0, const PxVec3& triEdge0, const PxVec3& triEdge1,
											PxReal* t, PxReal* u, PxReal* v)
{
	const PxReal fA00 = dir.magnitudeSquared();
	if(fA00 < 1e-6f*1e-6f)
	{
		if(t)
			*t = 0.0f;
		return distancePointTriangleSquared(origin, p0, triEdge0, triEdge1, u, v);
	}
	const PxVec3 kDiff = p0 - origin;
	const PxReal fA01 = -(dir.dot(triEdge0));
	const PxReal fA02 = -(dir.dot(triEdge1));
	const PxReal fA11 = triEdge0.magnitudeSquared();
	const PxReal fA12 = triEdge0.dot(triEdge1);
	const PxReal fA22 = triEdge1.dot(triEdge1);
	const PxReal fB0  = -(kDiff.dot(dir));
	const PxReal fB1  = kDiff.dot(triEdge0);
	const PxReal fB2  = kDiff.dot(triEdge1);
	const PxReal fCof00 = fA11*fA22-fA12*fA12;
	const PxReal fCof01 = fA02*fA12-fA01*fA22;
	const PxReal fCof02 = fA01*fA12-fA02*fA11;
	const PxReal fDet = fA00*fCof00+fA01*fCof01+fA02*fCof02;

	PxReal fSqrDist, fSqrDist0, fR, fS, fT, fR0, fS0, fT0;

	// Set up for a relative error test on the angle between ray direction
	// and triangle normal to determine parallel/nonparallel status.
	const PxVec3 kNormal = triEdge0.cross(triEdge1);
	const PxReal fDot = kNormal.dot(dir);
	if(fDot*fDot >= 1e-6f*dir.magnitudeSquared()*kNormal.magnitudeSquared())
	{
		const PxReal fCof11 = fA00*fA22-fA02*fA02;
		const PxReal fCof12 = fA02*fA01-fA00*fA12;
		const PxReal fCof22 = fA00*fA11-fA01*fA01;
		const PxReal fInvDet = fDet == 0.0f ? 0.0f : 1.0f/fDet;
		const PxReal fRhs0 = -fB0*fInvDet;
		const PxReal fRhs1 = -fB1*fInvDet;
		const PxReal fRhs2 = -fB2*fInvDet;

		fR = fCof00*fRhs0+fCof01*fRhs1+fCof02*fRhs2;
		fS = fCof01*fRhs0+fCof11*fRhs1+fCof12*fRhs2;
		fT = fCof02*fRhs0+fCof12*fRhs1+fCof22*fRhs2;

		if(fR < 0.0f)
		{
			if(fS+fT <= 1.0f)
			{
				if(fS < 0.0f)
				{
					if(fT < 0.0f)  // region 4m
					{
						// minimum on face s=0 or t=0 or r=0
						fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge1, &fR, &fT);
						fS = 0.0f;
						fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, p0, triEdge0, &fR0, &fS0);
						fT0 = 0.0f;
						updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
					}
					else  // region 3m
					{
						// minimum on face s=0 or r=0
						fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge1, &fR, &fT);
						fS = 0.0f;
					}
					fSqrDist0 = distancePointTriangleSquared(origin, p0, triEdge0, triEdge1, &fS0, &fT0);
					fR0 = 0.0f;
					updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
				}
				else if(fT < 0.0f)  // region 5m
				{
					// minimum on face t=0 or r=0
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge0, &fR, &fS);
					fT = 0.0f;
					fSqrDist0 = distancePointTriangleSquared(origin, p0, triEdge0, triEdge1, &fS0, &fT0);
					fR0 = 0.0f;
					updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
				}
				else  // region 0m
				{
					// minimum on face r=0
					fSqrDist = distancePointTriangleSquared(origin, p0, triEdge0, triEdge1, &fS, &fT);
					fR = 0.0f;
				}
			}
			else
			{
				if(fS < 0.0f)  // region 2m
				{
					// minimum on face s=0 or s+t=1 or r=0
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge1, &fR, &fT);
					fS = 0.0f;
					const PxVec3 kTriSegOrig = p0+triEdge0;
					const PxVec3 kTriSegDir = triEdge1-triEdge0;
					fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, kTriSegOrig, kTriSegDir, &fR0, &fT0);
					fS0 = 1.0f-fT0;
					updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
				}
				else if(fT < 0.0f)  // region 6m
				{
					// minimum on face t=0 or s+t=1 or r=0
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge0, &fR, &fS);
					fT = 0.0f;
					const PxVec3 kTriSegOrig = p0+triEdge0;
					const PxVec3 kTriSegDir = triEdge1-triEdge0;
					fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, kTriSegOrig, kTriSegDir, &fR0, &fT0);
					fS0 = 1.0f-fT0;
					updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
				}
				else  // region 1m
				{
					// minimum on face s+t=1 or r=0
					const PxVec3 kTriSegOrig = p0+triEdge0;
					const PxVec3 kTriSegDir = triEdge1-triEdge0;
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, kTriSegOrig, kTriSegDir, &fR, &fT);
					fS = 1.0f-fT;
				}
				fSqrDist0 = distancePointTriangleSquared(origin, p0, triEdge0, triEdge1, &fS0, &fT0);
				fR0 = 0.0f;
				updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
			}
		}
		else if(fR <= 1.0f)
		{
			if(fS+fT <= 1.0f)
			{
				if(fS < 0.0f)
				{
					if(fT < 0.0f)  // region 4
					{
						// minimum on face s=0 or t=0
						fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge1, &fR, &fT);
						fS = 0.0f;
						fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, p0, triEdge0, &fR0, &fS0);
						fT0 = 0.0f;
						updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
					}
					else  // region 3
					{
						// minimum on face s=0
						fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge1, &fR, &fT);
						fS = 0.0f;
					}
				}
				else if(fT < 0.0f)  // region 5
				{
					// minimum on face t=0
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge0, &fR, &fS);
					fT = 0.0f;
				}
				else  // region 0
				{
					// global minimum is interior, done
					fSqrDist = fR*(fA00*fR+fA01*fS+fA02*fT+2.0f*fB0)
						+fS*(fA01*fR+fA11*fS+fA12*fT+2.0f*fB1)
						+fT*(fA02*fR+fA12*fS+fA22*fT+2.0f*fB2)
						+kDiff.magnitudeSquared();
				}
			}
			else
			{
				if(fS < 0.0f)  // region 2
				{
					// minimum on face s=0 or s+t=1
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge1, &fR, &fT);
					fS = 0.0f;
					const PxVec3 kTriSegOrig = p0+triEdge0;
					const PxVec3 kTriSegDir = triEdge1-triEdge0;
					fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, kTriSegOrig, kTriSegDir, &fR0, &fT0);
					fS0 = 1.0f-fT0;
					updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
				}
				else if(fT < 0.0f)  // region 6
				{
					// minimum on face t=0 or s+t=1
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge0, &fR, &fS);
					fT = 0.0f;
					const PxVec3 kTriSegOrig = p0+triEdge0;
					const PxVec3 kTriSegDir = triEdge1-triEdge0;
					fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, kTriSegOrig, kTriSegDir, &fR0, &fT0);
					fS0 = 1.0f-fT0;
					updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
				}
				else  // region 1
				{
					// minimum on face s+t=1
					const PxVec3 kTriSegOrig = p0+triEdge0;
					const PxVec3 kTriSegDir = triEdge1-triEdge0;
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, kTriSegOrig, kTriSegDir, &fR, &fT);
					fS = 1.0f-fT;
				}
			}
		}
		else  // fR > 1
		{
			if(fS+fT <= 1.0f)
			{
				if(fS < 0.0f)
				{
					if(fT < 0.0f)  // region 4p
					{
						// minimum on face s=0 or t=0 or r=1
						fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge1, &fR, &fT);
						fS = 0.0f;
						fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, p0, triEdge0, &fR0, &fS0);
						fT0 = 0.0f;
						updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
					}
					else  // region 3p
					{
						// minimum on face s=0 or r=1
						fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge1, &fR, &fT);
						fS = 0.0f;
					}
					const PxVec3 kPt = origin+dir;
					fSqrDist0 = distancePointTriangleSquared(kPt, p0, triEdge0, triEdge1, &fS0, &fT0);
					fR0 = 1.0f;
					updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
				}
				else if(fT < 0.0f)  // region 5p
				{
					// minimum on face t=0 or r=1
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge0, &fR, &fS);
					fT = 0.0f;

					const PxVec3 kPt = origin+dir;
					fSqrDist0 = distancePointTriangleSquared(kPt, p0, triEdge0, triEdge1, &fS0, &fT0);
					fR0 = 1.0f;
					updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
				}
				else  // region 0p
				{
					// minimum face on r=1
					const PxVec3 kPt = origin+dir;
					fSqrDist = distancePointTriangleSquared(kPt, p0, triEdge0, triEdge1, &fS, &fT);
					fR = 1.0f;
				}
			}
			else
			{
				if(fS < 0.0f)  // region 2p
				{
					// minimum on face s=0 or s+t=1 or r=1
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge1, &fR, &fT);
					fS = 0.0f;
					const PxVec3 kTriSegOrig = p0+triEdge0;
					const PxVec3 kTriSegDir = triEdge1-triEdge0;
					fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, kTriSegOrig, kTriSegDir, &fR0, &fT0);
					fS0 = 1.0f-fT0;
					updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
				}
				else if(fT < 0.0f)  // region 6p
				{
					// minimum on face t=0 or s+t=1 or r=1
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge0, &fR, &fS);
					fT = 0.0f;
					const PxVec3 kTriSegOrig = p0+triEdge0;
					const PxVec3 kTriSegDir = triEdge1-triEdge0;
					fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, kTriSegOrig, kTriSegDir, &fR0, &fT0);
					fS0 = 1.0f-fT0;
					updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
				}
				else  // region 1p
				{
					// minimum on face s+t=1 or r=1
					const PxVec3 kTriSegOrig = p0+triEdge0;
					const PxVec3 kTriSegDir = triEdge1-triEdge0;
					fSqrDist = distanceSegmentSegmentSquared(origin, dir, kTriSegOrig, kTriSegDir, &fR, &fT);
					fS = 1.0f-fT;
				}
				const PxVec3 kPt = origin+dir;
				fSqrDist0 = distancePointTriangleSquared(kPt, p0, triEdge0, triEdge1, &fS0, &fT0);
				fR0 = 1.0f;
				updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
			}
		}
	}
	else
	{
		// segment and triangle are parallel
		fSqrDist = distanceSegmentSegmentSquared(origin, dir, p0, triEdge0, &fR, &fS);
		fT = 0.0f;

		fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, p0, triEdge1, &fR0, &fT0);
		fS0 = 0.0f;
		updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);

		const PxVec3 kTriSegOrig = p0+triEdge0;
		const PxVec3 kTriSegDir = triEdge1 - triEdge0;
		fSqrDist0 = distanceSegmentSegmentSquared(origin, dir, kTriSegOrig, kTriSegDir, &fR0, &fT0);
		fS0 = 1.0f-fT0;
		updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);

		fSqrDist0 = distancePointTriangleSquared(origin, p0, triEdge0, triEdge1, &fS0, &fT0);
		fR0 = 0.0f;
		updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);

		const PxVec3 kPt = origin+dir;
		fSqrDist0 = distancePointTriangleSquared(kPt, p0, triEdge0, triEdge1, &fS0, &fT0);
		fR0 = 1.0f;
		updateClosestHit(fSqrDist0, fR0, fS0, fT0, fSqrDist, fR, fS, fT);
	}

	if(t)	*t = fR;
	if(u)	*u = fS;
	if(v)	*v = fT;

	// account for numerical round-off error
	return physx::intrinsics::selectMax(0.0f, fSqrDist);
}


/*
	closest0 is the closest point on segment pq
	closest1 is the closest point on triangle abc
*/
Ps::aos::FloatV Gu::distanceSegmentTriangleSquared(	const Ps::aos::Vec3VArg p, const Ps::aos::Vec3VArg q,
													const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, const Ps::aos::Vec3VArg c,
													Ps::aos::Vec3V& closest0, Ps::aos::Vec3V& closest1)
{
	using namespace Ps::aos;
	const FloatV zero = FZero();
	//const FloatV one = FOne();
	//const FloatV parallelTolerance  = FloatV_From_F32(PX_PARALLEL_TOLERANCE);

	const Vec3V pq = V3Sub(q, p);
	const Vec3V ab = V3Sub(b, a);
	const Vec3V ac = V3Sub(c, a);
	const Vec3V bc = V3Sub(c, b);
	const Vec3V ap = V3Sub(p, a);
	const Vec3V aq = V3Sub(q, a);

	//This is used to calculate the barycentric coordinate
	const FloatV d00 = V3Dot(ab,ab);
	const FloatV d01 = V3Dot(ab, ac);
	const FloatV d11 = V3Dot(ac, ac);
	const FloatV tDenom = FSub(FMul(d00, d11), FMul(d01, d01));
	
	const FloatV bdenom = FSel(FIsGrtr(tDenom, zero), FRecip(tDenom), zero);

	const Vec3V n =V3Normalize(V3Cross(ab, ac)); // normalize vector

	//compute the closest point of p and triangle plane abc
	const FloatV dist3 = V3Dot(ap, n);
	const FloatV sqDist3 = FMul(dist3, dist3);


	//compute the closest point of q and triangle plane abc
	const FloatV dist4 = V3Dot(aq, n);
	const FloatV sqDist4 = FMul(dist4, dist4);
	const FloatV dMul = FMul(dist3, dist4);
	const BoolV con = FIsGrtr(zero, dMul);


	// intersect with the plane
	if(BAllEqTTTT(con))
	{
		//compute the intersect point
		const FloatV nom = FNeg(V3Dot(n, ap));
		const FloatV denom = FRecip(V3Dot(n, pq));
		const FloatV t = FMul(nom, denom);
		const Vec3V ip = V3ScaleAdd(pq, t, p);//V3Add(p, V3Scale(pq, t));
		const Vec3V v2 = V3Sub(ip, a);
		const FloatV d20 = V3Dot(v2, ab);
		const FloatV d21 = V3Dot(v2, ac);
		const FloatV v0 = FMul(FSub(FMul(d11, d20), FMul(d01, d21)), bdenom);
		const FloatV w0 = FMul(FSub(FMul(d00, d21), FMul(d01, d20)), bdenom);
		const BoolV con0 = isValidTriangleBarycentricCoord(v0, w0);
		if(BAllEqTTTT(con0))
		{
			closest0 = closest1 = ip;
			return zero;
		}
	}
	

	Vec4V t40, t41;
	const Vec4V sqDist44 = distanceSegmentSegmentSquared4(p,pq,a,ab, b,bc, a,ac, a,ab, t40, t41);  

	const FloatV t00 = V4GetX(t40);
	const FloatV t10 = V4GetY(t40);
	const FloatV t20 = V4GetZ(t40);

	const FloatV t01 = V4GetX(t41);
	const FloatV t11 = V4GetY(t41);
	const FloatV t21 = V4GetZ(t41);

	const FloatV sqDist0(V4GetX(sqDist44));
	const FloatV sqDist1(V4GetY(sqDist44));
	const FloatV sqDist2(V4GetZ(sqDist44));

	const Vec3V closestP00 = V3ScaleAdd(pq, t00, p);
	const Vec3V closestP01 = V3ScaleAdd(ab, t01, a);

	const Vec3V closestP10 = V3ScaleAdd(pq, t10, p);
	const Vec3V closestP11 = V3ScaleAdd(bc, t11, b);

	const Vec3V closestP20 = V3ScaleAdd(pq, t20, p);
	const Vec3V closestP21 = V3ScaleAdd(ac, t21, a);


	//Get the closest point of all edges
	const BoolV con20 = FIsGrtr(sqDist1, sqDist0);
	const BoolV con21 = FIsGrtr(sqDist2, sqDist0);
	const BoolV con2 = BAnd(con20,con21);
	const BoolV con30 = FIsGrtrOrEq(sqDist0, sqDist1);
	const BoolV con31 = FIsGrtr(sqDist2, sqDist1);
	const BoolV con3 = BAnd(con30, con31);
	const FloatV sqDistPE = FSel(con2, sqDist0, FSel(con3, sqDist1, sqDist2));
	//const FloatV tValue = FSel(con2, t00, FSel(con3, t10, t20));
	const Vec3V closestPE0 =  V3Sel(con2, closestP00, V3Sel(con3, closestP10, closestP20)); // closestP on segment
	const Vec3V closestPE1 =  V3Sel(con2, closestP01, V3Sel(con3, closestP11, closestP21)); // closestP on triangle


	const Vec3V closestP31 = V3NegScaleSub(n, dist3, p);//V3Sub(p, V3Scale(n, dist3));
	const Vec3V closestP30 = p;

	//Compute the barycentric coordinate for project point of q
	const Vec3V pV20 = V3Sub(closestP31, a);
	const FloatV pD20 = V3Dot(pV20, ab);
	const FloatV pD21 = V3Dot(pV20, ac);
	const FloatV v0 = FMul(FSub(FMul(d11, pD20), FMul(d01, pD21)), bdenom);
	const FloatV w0 = FMul(FSub(FMul(d00, pD21), FMul(d01, pD20)), bdenom);

	//check closestP3 is inside the triangle
	const BoolV con0 = isValidTriangleBarycentricCoord(v0, w0);


	
	const Vec3V closestP41 = V3NegScaleSub(n, dist4, q);// V3Sub(q, V3Scale(n, dist4));
	const Vec3V closestP40 = q;

	//Compute the barycentric coordinate for project point of q
	const Vec3V qV20 = V3Sub(closestP41, a);
	const FloatV qD20 = V3Dot(qV20, ab);
	const FloatV qD21 = V3Dot(qV20, ac);
	const FloatV v1 = FMul(FSub(FMul(d11, qD20), FMul(d01, qD21)), bdenom);
	const FloatV w1 = FMul(FSub(FMul(d00, qD21), FMul(d01, qD20)), bdenom);

	const BoolV con1 = isValidTriangleBarycentricCoord(v1, w1);

	/*
		p is interior point but not q
	*/
	const BoolV d0 = FIsGrtr(sqDistPE, sqDist3);
	const Vec3V c00 = V3Sel(d0, closestP30, closestPE0);
	const Vec3V c01 = V3Sel(d0, closestP31, closestPE1);

	/*
		q is interior point but not p
	*/
	const BoolV d1 = FIsGrtr(sqDistPE, sqDist4);
	const Vec3V c10 = V3Sel(d1, closestP40, closestPE0);
	const Vec3V c11 = V3Sel(d1, closestP41, closestPE1);

	/*
		p and q are interior point 
	*/
	const BoolV d2 = FIsGrtr(sqDist4, sqDist3);
	const Vec3V c20 = V3Sel(d2, closestP30, closestP40);
	const Vec3V c21 = V3Sel(d2, closestP31, closestP41);

	const BoolV cond2 = BAnd(con0, con1);

	const Vec3V closestP0 = V3Sel(cond2, c20, V3Sel(con0, c00, V3Sel(con1, c10, closestPE0)));
	const Vec3V closestP1 = V3Sel(cond2, c21, V3Sel(con0, c01, V3Sel(con1, c11, closestPE1)));

	const Vec3V vv = V3Sub(closestP1, closestP0);
	closest0 = closestP0;
	closest1 = closestP1;
	return V3Dot(vv, vv);
}
