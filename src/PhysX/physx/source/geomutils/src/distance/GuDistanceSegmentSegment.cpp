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

#include "GuDistanceSegmentSegment.h"
#include "GuDistanceSegmentSegmentSIMD.h"

using namespace physx;
using namespace Ps;
using namespace aos;

static const float ZERO_TOLERANCE = 1e-06f;

// S0 = origin + extent * dir;
// S1 = origin - extent * dir;
PxReal Gu::distanceSegmentSegmentSquared(	const PxVec3& origin0, const PxVec3& dir0, PxReal extent0,
											const PxVec3& origin1, const PxVec3& dir1, PxReal extent1,
											PxReal* param0,  PxReal* param1)
{
    const PxVec3 kDiff	= origin0 - origin1;
    const PxReal fA01	= -dir0.dot(dir1);
    const PxReal fB0	= kDiff.dot(dir0);
    const PxReal fB1	= -kDiff.dot(dir1);
	const PxReal fC		= kDiff.magnitudeSquared();
	const PxReal fDet	= PxAbs(1.0f - fA01*fA01);
    PxReal fS0, fS1, fSqrDist, fExtDet0, fExtDet1, fTmpS0, fTmpS1;

    if (fDet >= ZERO_TOLERANCE)
    {
        // segments are not parallel
        fS0 = fA01*fB1-fB0;
        fS1 = fA01*fB0-fB1;
        fExtDet0 = extent0*fDet;
        fExtDet1 = extent1*fDet;

        if (fS0 >= -fExtDet0)
        {
            if (fS0 <= fExtDet0)
            {
                if (fS1 >= -fExtDet1)
                {
                    if (fS1 <= fExtDet1)  // region 0 (interior)
                    {
                        // minimum at two interior points of 3D lines
                        PxReal fInvDet = 1.0f/fDet;
                        fS0 *= fInvDet;
                        fS1 *= fInvDet;
                        fSqrDist = fS0*(fS0+fA01*fS1+2.0f*fB0) + fS1*(fA01*fS0+fS1+2.0f*fB1)+fC;
                    }
                    else  // region 3 (side)
                    {
                        fS1 = extent1;
                        fTmpS0 = -(fA01*fS1+fB0);
                        if (fTmpS0 < -extent0)
                        {
                            fS0 = -extent0;
                            fSqrDist = fS0*(fS0-2.0f*fTmpS0) + fS1*(fS1+2.0f*fB1)+fC;
                        }
                        else if (fTmpS0 <= extent0)
                        {
                            fS0 = fTmpS0;
                            fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                        }
                        else
                        {
                            fS0 = extent0;
                            fSqrDist = fS0*(fS0-2.0f*fTmpS0) + fS1*(fS1+2.0f*fB1)+fC;
                        }
                    }
                }
                else  // region 7 (side)
                {
                    fS1 = -extent1;
                    fTmpS0 = -(fA01*fS1+fB0);
                    if (fTmpS0 < -extent0)
                    {
                        fS0 = -extent0;
                        fSqrDist = fS0*(fS0-2.0f*fTmpS0) + fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else if (fTmpS0 <= extent0)
                    {
                        fS0 = fTmpS0;
                        fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else
                    {
                        fS0 = extent0;
                        fSqrDist = fS0*(fS0-2.0f*fTmpS0) + fS1*(fS1+2.0f*fB1)+fC;
                    }
                }
            }
            else
            {
                if (fS1 >= -fExtDet1)
                {
                    if (fS1 <= fExtDet1)  // region 1 (side)
                    {
                        fS0 = extent0;
                        fTmpS1 = -(fA01*fS0+fB1);
                        if (fTmpS1 < -extent1)
                        {
                            fS1 = -extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                        }
                        else if (fTmpS1 <= extent1)
                        {
                            fS1 = fTmpS1;
                            fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0)+fC;
                        }
                        else
                        {
                            fS1 = extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                        }
                    }
                    else  // region 2 (corner)
                    {
                        fS1 = extent1;
                        fTmpS0 = -(fA01*fS1+fB0);
                        if (fTmpS0 < -extent0)
                        {
                            fS0 = -extent0;
                            fSqrDist = fS0*(fS0-2.0f*fTmpS0) + fS1*(fS1+2.0f*fB1)+fC;
                        }
                        else if (fTmpS0 <= extent0)
                        {
                            fS0 = fTmpS0;
                            fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                        }
                        else
                        {
                            fS0 = extent0;
                            fTmpS1 = -(fA01*fS0+fB1);
                            if (fTmpS1 < -extent1)
                            {
                                fS1 = -extent1;
                                fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                            }
                            else if (fTmpS1 <= extent1)
                            {
                                fS1 = fTmpS1;
                                fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0) + fC;
                            }
                            else
                            {
                                fS1 = extent1;
                                fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                            }
                        }
                    }
                }
                else  // region 8 (corner)
                {
                    fS1 = -extent1;
                    fTmpS0 = -(fA01*fS1+fB0);
                    if (fTmpS0 < -extent0)
                    {
                        fS0 = -extent0;
                        fSqrDist = fS0*(fS0-2.0f*fTmpS0) + fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else if (fTmpS0 <= extent0)
                    {
                        fS0 = fTmpS0;
                        fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else
                    {
                        fS0 = extent0;
                        fTmpS1 = -(fA01*fS0+fB1);
                        if (fTmpS1 > extent1)
                        {
                            fS1 = extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                        }
                        else if (fTmpS1 >= -extent1)
                        {
                            fS1 = fTmpS1;
                            fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0) + fC;
                        }
                        else
                        {
                            fS1 = -extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                        }
                    }
                }
            }
        }
        else 
        {
            if (fS1 >= -fExtDet1)
            {
                if (fS1 <= fExtDet1)  // region 5 (side)
                {
                    fS0 = -extent0;
                    fTmpS1 = -(fA01*fS0+fB1);
                    if (fTmpS1 < -extent1)
                    {
                        fS1 = -extent1;
                        fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                    }
                    else if (fTmpS1 <= extent1)
                    {
                        fS1 = fTmpS1;
                        fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0)+fC;
                    }
                    else
                    {
                        fS1 = extent1;
                        fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                    }
                }
                else  // region 4 (corner)
                {
                    fS1 = extent1;
                    fTmpS0 = -(fA01*fS1+fB0);
                    if (fTmpS0 > extent0)
                    {
                        fS0 = extent0;
                        fSqrDist = fS0*(fS0-2.0f*fTmpS0) + fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else if (fTmpS0 >= -extent0)
                    {
                        fS0 = fTmpS0;
                        fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                    }
                    else
                    {
                        fS0 = -extent0;
                        fTmpS1 = -(fA01*fS0+fB1);
                        if (fTmpS1 < -extent1)
                        {
                            fS1 = -extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                        }
                        else if (fTmpS1 <= extent1)
                        {
                            fS1 = fTmpS1;
                            fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0) + fC;
                        }
                        else
                        {
                            fS1 = extent1;
                            fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                        }
                    }
                }
            }
            else   // region 6 (corner)
            {
                fS1 = -extent1;
                fTmpS0 = -(fA01*fS1+fB0);
                if (fTmpS0 > extent0)
                {
                    fS0 = extent0;
                    fSqrDist = fS0*(fS0-2.0f*fTmpS0) + fS1*(fS1+2.0f*fB1)+fC;
                }
                else if (fTmpS0 >= -extent0)
                {
                    fS0 = fTmpS0;
                    fSqrDist = -fS0*fS0+fS1*(fS1+2.0f*fB1)+fC;
                }
                else
                {
                    fS0 = -extent0;
                    fTmpS1 = -(fA01*fS0+fB1);
                    if (fTmpS1 < -extent1)
                    {
                        fS1 = -extent1;
                        fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                    }
                    else if (fTmpS1 <= extent1)
                    {
                        fS1 = fTmpS1;
                        fSqrDist = -fS1*fS1+fS0*(fS0+2.0f*fB0) + fC;
                    }
                    else
                    {
                        fS1 = extent1;
                        fSqrDist = fS1*(fS1-2.0f*fTmpS1) + fS0*(fS0+2.0f*fB0)+fC;
                    }
                }
            }
        }
    }
    else
    {
		// The segments are parallel.
		PxReal fE0pE1 = extent0 + extent1;
		PxReal fSign = (fA01 > 0.0f ? -1.0f : 1.0f);
		PxReal b0Avr = 0.5f*(fB0 - fSign*fB1);
		PxReal fLambda = -b0Avr;
		if(fLambda < -fE0pE1)
		{
			fLambda = -fE0pE1;
		}
		else if(fLambda > fE0pE1)
		{
			fLambda = fE0pE1;
		}

		fS1 = -fSign*fLambda*extent1/fE0pE1;
		fS0 = fLambda + fSign*fS1;
		fSqrDist = fLambda*(fLambda + 2.0f*b0Avr) + fC;
	}

	if(param0)
		*param0 = fS0;
	if(param1)
		*param1 = fS1;

	// account for numerical round-off error
	return physx::intrinsics::selectMax(0.0f, fSqrDist);
}

PxReal Gu::distanceSegmentSegmentSquared(	const PxVec3& origin0, const PxVec3& extent0,
											const PxVec3& origin1, const PxVec3& extent1,
											PxReal* param0, 
											PxReal* param1)
{
	// Some conversion is needed between the old & new code
	// Old:
	// segment (s0, s1)
	// origin = s0
	// extent = s1 - s0
	//
	// New:
	// s0 = origin + extent * dir;
	// s1 = origin - extent * dir;

	// dsequeira: is this really sensible? We use a highly optimized Wild Magic routine, 
	// then use a segment representation that requires an expensive conversion to/from...

	PxVec3 dir0 = extent0;
	const PxVec3 center0 = origin0 + extent0*0.5f;
	PxReal length0 = extent0.magnitude();	//AM: change to make it work for degenerate (zero length) segments.
	const bool b0 = length0 != 0.0f;
	PxReal oneOverLength0 = 0.0f;
	if(b0)
	{
		oneOverLength0 = 1.0f / length0;
		dir0 *= oneOverLength0;
		length0 *= 0.5f;
	}

	PxVec3 dir1 = extent1;
	const PxVec3 center1 = origin1 + extent1*0.5f;
	PxReal length1 = extent1.magnitude();
	const bool b1 = length1 != 0.0f;
	PxReal oneOverLength1 = 0.0f;
	if(b1)
	{
		oneOverLength1 = 1.0f / length1;
		dir1 *= oneOverLength1;
		length1 *= 0.5f;
	}

	// the return param vals have -extent = s0, extent = s1

	const PxReal d2 = distanceSegmentSegmentSquared(center0, dir0, length0,
													center1, dir1, length1,
													param0, param1);

	//ML : This is wrong for some reason, I guess it has precision issue
	//// renormalize into the 0 = s0, 1 = s1 range
	//if (param0)
	//	*param0 = b0 ? ((*param0) * oneOverLength0 * 0.5f + 0.5f) : 0.0f;
	//if (param1)
	//	*param1 = b1 ? ((*param1) * oneOverLength1 * 0.5f + 0.5f) : 0.0f;

	if(param0)
		*param0 = b0 ? ((length0 + (*param0))*oneOverLength0) : 0.0f;
	if(param1)
		*param1 = b1 ? ((length1 + (*param1))*oneOverLength1) : 0.0f;

	return d2;
}

/*
	S0 = origin + extent * dir;
	S1 = origin + extent * dir;
	dir is the vector from start to end point
	p1 is the start point of segment1
	d1 is the direction vector(q1 - p1)
	p2 is the start point of segment2
	d2 is the direction vector(q2 - p2) 
*/

FloatV Gu::distanceSegmentSegmentSquared(	const Vec3VArg p1, 
											const Vec3VArg d1,
											const Vec3VArg p2, 
											const Vec3VArg d2,
											FloatV& s, 
											FloatV& t)
{
	const FloatV zero = FZero();
	const FloatV one = FOne();
	const FloatV eps = FEps();

	const Vec3V r = V3Sub(p1, p2);
	const Vec4V combinedDot = V3Dot4(d1, d1, d2, d2, d1, d2, d1, r);
	const Vec4V combinedRecip = V4Sel(V4IsGrtr(combinedDot, V4Splat(eps)), V4Recip(combinedDot), V4Splat(zero));
	const FloatV a = V4GetX(combinedDot);
	const FloatV e = V4GetY(combinedDot);
	const FloatV b = V4GetZ(combinedDot);
	const FloatV c = V4GetW(combinedDot);
	const FloatV aRecip = V4GetX(combinedRecip);//FSel(FIsGrtr(a, eps), FRecip(a), zero);
	const FloatV eRecip = V4GetY(combinedRecip);//FSel(FIsGrtr(e, eps), FRecip(e), zero);

	const FloatV f = V3Dot(d2, r);

	/*
		s = (b*f - c*e)/(a*e - b*b);
		t = (a*f - b*c)/(a*e - b*b);

		s = (b*t - c)/a;
		t = (b*s + f)/e;
	*/
	
	//if segments not parallel, the general non-degenerated case, compute closest point on two segments and clamp to segment1
	const FloatV denom = FSub(FMul(a, e), FMul(b, b));
	const FloatV temp = FSub(FMul(b, f), FMul(c, e));
	const FloatV s0 = FClamp(FDiv(temp, denom), zero, one);
	
	//if segment is parallel, demon < eps
	const BoolV con2 = FIsGrtr(eps, denom);//FIsEq(denom, zero);
	const FloatV sTmp = FSel(con2, FHalf(), s0);
	
	//compute point on segment2 closest to segment1
	//const FloatV tTmp = FMul(FAdd(FMul(b, sTmp), f), eRecip);
	const FloatV tTmp = FMul(FScaleAdd(b, sTmp, f), eRecip);

	//if t is in [zero, one], done. otherwise clamp t
	const FloatV t2 = FClamp(tTmp, zero, one);

	//recompute s for the new value
	const FloatV comp = FMul(FSub(FMul(b,t2), c), aRecip);
	const FloatV s2 = FClamp(comp, zero, one);

	s = s2;
	t = t2;

	const Vec3V closest1 = V3ScaleAdd(d1, s2, p1);//V3Add(p1, V3Scale(d1, tempS));
	const Vec3V closest2 = V3ScaleAdd(d2, t2, p2);//V3Add(p2, V3Scale(d2, tempT));
	const Vec3V vv = V3Sub(closest1, closest2);
	return V3Dot(vv, vv);
}




/*
	segment (p, d) and segment (p02, d02)
	segment (p, d) and segment (p12, d12)
	segment (p, d) and segment (p22, d22)
	segment (p, d) and segment (p32, d32)
*/
Vec4V Gu::distanceSegmentSegmentSquared4(	const Vec3VArg p, const Vec3VArg d0, 
											const Vec3VArg p02, const Vec3VArg d02, 
											const Vec3VArg p12, const Vec3VArg d12, 
											const Vec3VArg p22, const Vec3VArg d22,
											const Vec3VArg p32, const Vec3VArg d32,
											Vec4V& s, Vec4V& t)
{
	const Vec4V zero = V4Zero();
	const Vec4V one = V4One();
	const Vec4V eps = V4Eps();
	const Vec4V half = V4Splat(FHalf());

	const Vec4V d0X = V4Splat(V3GetX(d0));
	const Vec4V d0Y = V4Splat(V3GetY(d0));
	const Vec4V d0Z = V4Splat(V3GetZ(d0));
	const Vec4V pX  = V4Splat(V3GetX(p));
	const Vec4V pY  = V4Splat(V3GetY(p));
	const Vec4V pZ  = V4Splat(V3GetZ(p));

	Vec4V d024 = Vec4V_From_Vec3V(d02);
	Vec4V d124 = Vec4V_From_Vec3V(d12);
	Vec4V d224 = Vec4V_From_Vec3V(d22);
	Vec4V d324 = Vec4V_From_Vec3V(d32);

	Vec4V p024 = Vec4V_From_Vec3V(p02);
	Vec4V p124 = Vec4V_From_Vec3V(p12);
	Vec4V p224 = Vec4V_From_Vec3V(p22);
	Vec4V p324 = Vec4V_From_Vec3V(p32);

	Vec4V d0123X, d0123Y, d0123Z;
	Vec4V p0123X, p0123Y, p0123Z;

	PX_TRANSPOSE_44_34(d024, d124, d224, d324, d0123X, d0123Y, d0123Z);
	PX_TRANSPOSE_44_34(p024, p124, p224, p324, p0123X, p0123Y, p0123Z);

	const Vec4V rX = V4Sub(pX, p0123X);
	const Vec4V rY = V4Sub(pY, p0123Y);
	const Vec4V rZ = V4Sub(pZ, p0123Z);

	//TODO - store this in a transposed state and avoid so many dot products?

	const FloatV dd = V3Dot(d0, d0);

	const Vec4V e = V4MulAdd(d0123Z, d0123Z, V4MulAdd(d0123X, d0123X, V4Mul(d0123Y, d0123Y)));
	const Vec4V b = V4MulAdd(d0Z, d0123Z, V4MulAdd(d0X, d0123X, V4Mul(d0Y, d0123Y)));
	const Vec4V c = V4MulAdd(d0Z, rZ, V4MulAdd(d0X, rX, V4Mul(d0Y, rY)));
	const Vec4V f = V4MulAdd(d0123Z, rZ, V4MulAdd(d0123X, rX, V4Mul(d0123Y, rY))); 

	const Vec4V a(V4Splat(dd));

	const Vec4V aRecip(V4Recip(a));
	const Vec4V eRecip(V4Recip(e));

	//if segments not parallell, compute closest point on two segments and clamp to segment1
	const Vec4V denom = V4Sub(V4Mul(a, e), V4Mul(b, b));
	const Vec4V temp = V4Sub(V4Mul(b, f), V4Mul(c, e));
	const Vec4V s0 = V4Clamp(V4Div(temp, denom), zero, one);

	//test whether segments are parallel
	const BoolV con2 = V4IsGrtrOrEq(eps, denom);     
	const Vec4V sTmp = V4Sel(con2, half, s0);

	//compute point on segment2 closest to segment1
	const Vec4V tTmp = V4Mul(V4Add(V4Mul(b, sTmp), f), eRecip);

	//if t is in [zero, one], done. otherwise clamp t
	const Vec4V t2 = V4Clamp(tTmp, zero, one);

	//recompute s for the new value
	const Vec4V comp = V4Mul(V4Sub(V4Mul(b,t2), c), aRecip);
	const BoolV aaNearZero = V4IsGrtrOrEq(eps, a); // check if aRecip is valid (aa>eps)
	const Vec4V s2 = V4Sel(aaNearZero, V4Zero(), V4Clamp(comp, zero, one));

	/*  s = V4Sel(con0, zero, V4Sel(con1, cd, s2));
	t = V4Sel(con1, zero, V4Sel(con0, cg, t2));  */
	s = s2;
	t = t2;

	const Vec4V closest1X = V4MulAdd(d0X, s2, pX);
	const Vec4V closest1Y = V4MulAdd(d0Y, s2, pY);
	const Vec4V closest1Z = V4MulAdd(d0Z, s2, pZ);

	const Vec4V closest2X = V4MulAdd(d0123X, t2, p0123X);
	const Vec4V closest2Y = V4MulAdd(d0123Y, t2, p0123Y);
	const Vec4V closest2Z = V4MulAdd(d0123Z, t2, p0123Z);

	const Vec4V vvX = V4Sub(closest1X, closest2X);
	const Vec4V vvY = V4Sub(closest1Y, closest2Y);
	const Vec4V vvZ = V4Sub(closest1Z, closest2Z);

	const Vec4V vd = V4MulAdd(vvX, vvX, V4MulAdd(vvY, vvY, V4Mul(vvZ, vvZ)));

	return vd;
}
