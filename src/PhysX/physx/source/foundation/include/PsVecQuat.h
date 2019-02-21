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

#ifndef PSFOUNDATION_PSVECQUAT_H
#define PSFOUNDATION_PSVECQUAT_H

//#include "PsInlineAoS.h"

namespace physx
{
namespace shdfnd
{
namespace aos
{

#ifndef PX_PIDIV2
#define PX_PIDIV2 1.570796327f
#endif

//////////////////////////////////
// QuatV
//////////////////////////////////
PX_FORCE_INLINE QuatV QuatVLoadXYZW(const PxF32 x, const PxF32 y, const PxF32 z, const PxF32 w)
{
	return V4LoadXYZW(x, y, z, w);
}

PX_FORCE_INLINE QuatV QuatVLoadU(const PxF32* v)
{
	return V4LoadU(v);
}

PX_FORCE_INLINE QuatV QuatVLoadA(const PxF32* v)
{
	return V4LoadA(v);
}

PX_FORCE_INLINE QuatV QuatV_From_RotationAxisAngle(const Vec3V u, const FloatV a)
{
	// q = cos(a/2) + u*sin(a/2)
	const FloatV half = FLoad(0.5f);
	const FloatV hangle = FMul(a, half);
	const FloatV piByTwo(FLoad(PX_PIDIV2));
	const FloatV PiByTwoMinHangle(FSub(piByTwo, hangle));
	const Vec4V hangle2(Vec4V_From_Vec3V(V3Merge(hangle, PiByTwoMinHangle, hangle)));

	/*const FloatV sina = FSin(hangle);
	const FloatV cosa = FCos(hangle);*/

	const Vec4V _sina = V4Sin(hangle2);
	const FloatV sina = V4GetX(_sina);
	const FloatV cosa = V4GetY(_sina);

	const Vec3V v = V3Scale(u, sina);
	// return V4Sel(BTTTF(), Vec4V_From_Vec3V(v), V4Splat(cosa));
	return V4SetW(Vec4V_From_Vec3V(v), cosa);
}

// Normalize
PX_FORCE_INLINE QuatV QuatNormalize(const QuatV q)
{
	return V4Normalize(q);
}

PX_FORCE_INLINE FloatV QuatLength(const QuatV q)
{
	return V4Length(q);
}

PX_FORCE_INLINE FloatV QuatLengthSq(const QuatV q)
{
	return V4LengthSq(q);
}

PX_FORCE_INLINE FloatV QuatDot(const QuatV a, const QuatV b) // convert this PxQuat to a unit quaternion
{
	return V4Dot(a, b);
}

PX_FORCE_INLINE QuatV QuatConjugate(const QuatV q)
{
	return V4SetW(V4Neg(q), V4GetW(q));
}

PX_FORCE_INLINE Vec3V QuatGetImaginaryPart(const QuatV q)
{
	return Vec3V_From_Vec4V(q);
}

/** brief computes rotation of x-axis */
PX_FORCE_INLINE Vec3V QuatGetBasisVector0(const QuatV q)
{
	/*const PxF32 x2 = x*2.0f;
	const PxF32 w2 = w*2.0f;
	return PxVec3(	(w * w2) - 1.0f + x*x2,
	                (z * w2)        + y*x2,
	                (-y * w2)       + z*x2);*/

	const FloatV two = FLoad(2.f);
	const FloatV w = V4GetW(q);
	const Vec3V u = Vec3V_From_Vec4V(q);

	const FloatV x2 = FMul(V3GetX(u), two);
	const FloatV w2 = FMul(w, two);

	const Vec3V a = V3Scale(u, x2);
	const Vec3V tmp = V3Merge(w, V3GetZ(u), FNeg(V3GetY(u)));
	// const Vec3V b = V3Scale(tmp, w2);
	// const Vec3V ab = V3Add(a, b);
	const Vec3V ab = V3ScaleAdd(tmp, w2, a);
	return V3SetX(ab, FSub(V3GetX(ab), FOne()));
}

/** brief computes rotation of y-axis */
PX_FORCE_INLINE Vec3V QuatGetBasisVector1(const QuatV q)
{
	/*const PxF32 y2 = y*2.0f;
	const PxF32 w2 = w*2.0f;
	return PxVec3(	(-z * w2)       + x*y2,
	                (w * w2) - 1.0f + y*y2,
	                (x * w2)        + z*y2);*/

	const FloatV two = FLoad(2.f);
	const FloatV w = V4GetW(q);
	const Vec3V u = Vec3V_From_Vec4V(q);

	const FloatV y2 = FMul(V3GetY(u), two);
	const FloatV w2 = FMul(w, two);

	const Vec3V a = V3Scale(u, y2);
	const Vec3V tmp = V3Merge(FNeg(V3GetZ(u)), w, V3GetX(u));
	// const Vec3V b = V3Scale(tmp, w2);
	// const Vec3V ab = V3Add(a, b);
	const Vec3V ab = V3ScaleAdd(tmp, w2, a);
	return V3SetY(ab, FSub(V3GetY(ab), FOne()));
}

/** brief computes rotation of z-axis */
PX_FORCE_INLINE Vec3V QuatGetBasisVector2(const QuatV q)
{
	/*const PxF32 z2 = z*2.0f;
	const PxF32 w2 = w*2.0f;
	return PxVec3(	(y * w2)        + x*z2,
	                (-x * w2)       + y*z2,
	                (w * w2) - 1.0f + z*z2);*/

	const FloatV two = FLoad(2.f);
	const FloatV w = V4GetW(q);
	const Vec3V u = Vec3V_From_Vec4V(q);

	const FloatV z2 = FMul(V3GetZ(u), two);
	const FloatV w2 = FMul(w, two);

	const Vec3V a = V3Scale(u, z2);
	const Vec3V tmp = V3Merge(V3GetY(u), FNeg(V3GetX(u)), w);
	/*const Vec3V b = V3Scale(tmp, w2);
	const Vec3V ab = V3Add(a, b);*/
	const Vec3V ab = V3ScaleAdd(tmp, w2, a);
	return V3SetZ(ab, FSub(V3GetZ(ab), FOne()));
}

PX_FORCE_INLINE Vec3V QuatRotate(const QuatV q, const Vec3V v)
{
	/*
	const PxVec3 qv(x,y,z);
	return (v*(w*w-0.5f) + (qv.cross(v))*w + qv*(qv.dot(v)))*2;
	*/

	const FloatV two = FLoad(2.f);
	// const FloatV half = FloatV_From_F32(0.5f);
	const FloatV nhalf = FLoad(-0.5f);
	const Vec3V u = Vec3V_From_Vec4V(q);
	const FloatV w = V4GetW(q);
	// const FloatV w2 = FSub(FMul(w, w), half);
	const FloatV w2 = FScaleAdd(w, w, nhalf);
	const Vec3V a = V3Scale(v, w2);
	// const Vec3V b = V3Scale(V3Cross(u, v), w);
	// const Vec3V c = V3Scale(u, V3Dot(u, v));
	// return V3Scale(V3Add(V3Add(a, b), c), two);
	const Vec3V temp = V3ScaleAdd(V3Cross(u, v), w, a);
	return V3Scale(V3ScaleAdd(u, V3Dot(u, v), temp), two);
}

PX_FORCE_INLINE Vec3V QuatTransform(const QuatV q, const Vec3V p, const Vec3V v)
{
	// p + q.rotate(v)
	const FloatV two = FLoad(2.f);
	// const FloatV half = FloatV_From_F32(0.5f);
	const FloatV nhalf = FLoad(-0.5f);
	const Vec3V u = Vec3V_From_Vec4V(q);
	const FloatV w = V4GetW(q);
	// const FloatV w2 = FSub(FMul(w, w), half);
	const FloatV w2 = FScaleAdd(w, w, nhalf);
	const Vec3V a = V3Scale(v, w2);
	/*const Vec3V b = V3Scale(V3Cross(u, v), w);
	const Vec3V c = V3Scale(u, V3Dot(u, v));
	return V3ScaleAdd(V3Add(V3Add(a, b), c), two, p);*/
	const Vec3V temp = V3ScaleAdd(V3Cross(u, v), w, a);
	const Vec3V z = V3ScaleAdd(u, V3Dot(u, v), temp);
	return V3ScaleAdd(z, two, p);
}

PX_FORCE_INLINE Vec3V QuatRotateInv(const QuatV q, const Vec3V v)
{

	//	const PxVec3 qv(x,y,z);
	//	return (v*(w*w-0.5f) - (qv.cross(v))*w + qv*(qv.dot(v)))*2;

	const FloatV two = FLoad(2.f);
	const FloatV nhalf = FLoad(-0.5f);
	const Vec3V u = Vec3V_From_Vec4V(q);
	const FloatV w = V4GetW(q);
	const FloatV w2 = FScaleAdd(w, w, nhalf);
	const Vec3V a = V3Scale(v, w2);
	/*const Vec3V b = V3Scale(V3Cross(u, v), w);
	const Vec3V c = V3Scale(u, V3Dot(u, v));
	return V3Scale(V3Add(V3Sub(a, b), c), two);*/
	const Vec3V temp = V3NegScaleSub(V3Cross(u, v), w, a);
	return V3Scale(V3ScaleAdd(u, V3Dot(u, v), temp), two);
}

PX_FORCE_INLINE QuatV QuatMul(const QuatV a, const QuatV b)
{
	const Vec3V imagA = Vec3V_From_Vec4V(a);
	const Vec3V imagB = Vec3V_From_Vec4V(b);
	const FloatV rA = V4GetW(a);
	const FloatV rB = V4GetW(b);

	const FloatV real = FSub(FMul(rA, rB), V3Dot(imagA, imagB));
	const Vec3V v0 = V3Scale(imagA, rB);
	const Vec3V v1 = V3Scale(imagB, rA);
	const Vec3V v2 = V3Cross(imagA, imagB);
	const Vec3V imag = V3Add(V3Add(v0, v1), v2);

	return V4SetW(Vec4V_From_Vec3V(imag), real);
}

PX_FORCE_INLINE QuatV QuatAdd(const QuatV a, const QuatV b)
{
	return V4Add(a, b);
}

PX_FORCE_INLINE QuatV QuatNeg(const QuatV q)
{
	return V4Neg(q);
}

PX_FORCE_INLINE QuatV QuatSub(const QuatV a, const QuatV b)
{
	return V4Sub(a, b);
}

PX_FORCE_INLINE QuatV QuatScale(const QuatV a, const FloatV b)
{
	return V4Scale(a, b);
}

PX_FORCE_INLINE QuatV QuatMerge(const FloatV* const floatVArray)
{
	return V4Merge(floatVArray);
}

PX_FORCE_INLINE QuatV QuatMerge(const FloatVArg x, const FloatVArg y, const FloatVArg z, const FloatVArg w)
{
	return V4Merge(x, y, z, w);
}

PX_FORCE_INLINE QuatV QuatIdentity()
{
	return V4SetW(V4Zero(), FOne());
}

PX_FORCE_INLINE bool isFiniteQuatV(const QuatV q)
{
	return isFiniteVec4V(q);
}

PX_FORCE_INLINE bool isValidQuatV(const QuatV q)
{
	const FloatV unitTolerance = FLoad(1e-4f);
	const FloatV tmp = FAbs(FSub(QuatLength(q), FOne()));
	const BoolV con = FIsGrtr(unitTolerance, tmp);
	return isFiniteVec4V(q) & (BAllEqTTTT(con) == 1);
}

PX_FORCE_INLINE bool isSaneQuatV(const QuatV q)
{
	const FloatV unitTolerance = FLoad(1e-2f);
	const FloatV tmp = FAbs(FSub(QuatLength(q), FOne()));
	const BoolV con = FIsGrtr(unitTolerance, tmp);
	return isFiniteVec4V(q) & (BAllEqTTTT(con) == 1);
}

PX_FORCE_INLINE Mat33V QuatGetMat33V(const QuatVArg q)
{
	// const FloatV two = FloatV_From_F32(2.f);
	// const FloatV one = FOne();

	// const FloatV x = V4GetX(q);
	// const FloatV y = V4GetY(q);
	// const FloatV z = V4GetZ(q);
	// const Vec4V _q = V4Mul(q, two);
	//
	////const FloatV w = V4GetW(q);

	// const Vec4V t0 = V4Mul(_q, x); // 2xx, 2xy, 2xz, 2xw
	// const Vec4V t1 = V4Mul(_q, y); // 2xy, 2yy, 2yz, 2yw
	// const Vec4V t2 = V4Mul(_q, z); // 2xz, 2yz, 2zz, 2zw
	////const Vec4V t3 = V4Mul(_q, w); // 2xw, 2yw, 2zw, 2ww

	// const FloatV xx2 = V4GetX(t0);
	// const FloatV xy2 = V4GetY(t0);
	// const FloatV xz2 = V4GetZ(t0);
	// const FloatV xw2 = V4GetW(t0);

	// const FloatV yy2 = V4GetY(t1);
	// const FloatV yz2 = V4GetZ(t1);
	// const FloatV yw2 = V4GetW(t1);

	// const FloatV zz2 = V4GetZ(t2);
	// const FloatV zw2 = V4GetW(t2);

	////const FloatV ww2 = V4GetW(t3);

	// const FloatV c00 = FSub(one, FAdd(yy2, zz2));
	// const FloatV c01 = FSub(xy2, zw2);
	// const FloatV c02 = FAdd(xz2, yw2);

	// const FloatV c10 = FAdd(xy2, zw2);
	// const FloatV c11 = FSub(one, FAdd(xx2, zz2));
	// const FloatV c12 = FSub(yz2, xw2);

	// const FloatV c20 = FSub(xz2, yw2);
	// const FloatV c21 = FAdd(yz2, xw2);
	// const FloatV c22 = FSub(one, FAdd(xx2, yy2));

	// const Vec3V c0 = V3Merge(c00, c10, c20);
	// const Vec3V c1 = V3Merge(c01, c11, c21);
	// const Vec3V c2 = V3Merge(c02, c12, c22);

	// return Mat33V(c0, c1, c2);

	const FloatV one = FOne();
	const FloatV x = V4GetX(q);
	const FloatV y = V4GetY(q);
	const FloatV z = V4GetZ(q);
	const FloatV w = V4GetW(q);

	const FloatV x2 = FAdd(x, x);
	const FloatV y2 = FAdd(y, y);
	const FloatV z2 = FAdd(z, z);

	const FloatV xx = FMul(x2, x);
	const FloatV yy = FMul(y2, y);
	const FloatV zz = FMul(z2, z);

	const FloatV xy = FMul(x2, y);
	const FloatV xz = FMul(x2, z);
	const FloatV xw = FMul(x2, w);

	const FloatV yz = FMul(y2, z);
	const FloatV yw = FMul(y2, w);
	const FloatV zw = FMul(z2, w);

	const FloatV v = FSub(one, xx);

	const Vec3V column0 = V3Merge(FSub(FSub(one, yy), zz), FAdd(xy, zw), FSub(xz, yw));
	const Vec3V column1 = V3Merge(FSub(xy, zw), FSub(v, zz), FAdd(yz, xw));
	const Vec3V column2 = V3Merge(FAdd(xz, yw), FSub(yz, xw), FSub(v, yy));
	return Mat33V(column0, column1, column2);
}

PX_FORCE_INLINE QuatV Mat33GetQuatV(const Mat33V& a)
{
	const FloatV one = FOne();
	const FloatV zero = FZero();
	const FloatV half = FLoad(0.5f);
	const FloatV two = FLoad(2.f);
	const FloatV scale = FLoad(0.25f);
	const FloatV a00 = V3GetX(a.col0);
	const FloatV a11 = V3GetY(a.col1);
	const FloatV a22 = V3GetZ(a.col2);

	const FloatV a21 = V3GetZ(a.col1); // row=2, col=1;
	const FloatV a12 = V3GetY(a.col2); // row=1, col=2;
	const FloatV a02 = V3GetX(a.col2); // row=0, col=2;
	const FloatV a20 = V3GetZ(a.col0); // row=2, col=0;
	const FloatV a10 = V3GetY(a.col0); // row=1, col=0;
	const FloatV a01 = V3GetX(a.col1); // row=0, col=1;

	const Vec3V vec0 = V3Merge(a21, a02, a10);
	const Vec3V vec1 = V3Merge(a12, a20, a01);
	const Vec3V v = V3Sub(vec0, vec1);
	const Vec3V g = V3Add(vec0, vec1);

	const FloatV trace = FAdd(a00, FAdd(a11, a22));

	if(FAllGrtrOrEq(trace, zero))
	{
		const FloatV h = FSqrt(FAdd(trace, one));
		const FloatV w = FMul(half, h);
		const FloatV s = FMul(half, FRecip(h));
		const Vec3V u = V3Scale(v, s);
		return V4SetW(Vec4V_From_Vec3V(u), w);
	}
	else
	{
		const FloatV ntrace = FNeg(trace);
		const Vec3V d = V3Merge(a00, a11, a22);
		const BoolV con0 = BAllTrue3(V3IsGrtrOrEq(V3Splat(a00), d));
		const BoolV con1 = BAllTrue3(V3IsGrtrOrEq(V3Splat(a11), d));

		const FloatV t0 = FAdd(one, FScaleAdd(a00, two, ntrace));
		const FloatV t1 = FAdd(one, FScaleAdd(a11, two, ntrace));
		const FloatV t2 = FAdd(one, FScaleAdd(a22, two, ntrace));

		const FloatV t = FSel(con0, t0, FSel(con1, t1, t2));

		const FloatV h = FMul(two, FSqrt(t));
		const FloatV s = FRecip(h);
		const FloatV g0 = FMul(scale, h);
		const Vec3V vs = V3Scale(v, s);
		const Vec3V gs = V3Scale(g, s);
		const FloatV gsx = V3GetX(gs);
		const FloatV gsy = V3GetY(gs);
		const FloatV gsz = V3GetZ(gs);
		// vs.x= (a21 - a12)*s; vs.y=(a02 - a20)*s; vs.z=(a10 - a01)*s;
		// gs.x= (a21 + a12)*s; gs.y=(a02 + a20)*s; gs.z=(a10 + a01)*s;
		const Vec4V v0 = V4Merge(g0, gsz, gsy, V3GetX(vs));
		const Vec4V v1 = V4Merge(gsz, g0, gsx, V3GetY(vs));
		const Vec4V v2 = V4Merge(gsy, gsx, g0, V3GetZ(vs));
		return V4Sel(con0, v0, V4Sel(con1, v1, v2));
	}
}

} // namespace aos
} // namespace shdfnd
} // namespace physx

#endif
