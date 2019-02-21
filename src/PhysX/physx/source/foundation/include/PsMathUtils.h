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

#ifndef PSFOUNDATION_PSMATHUTILS_H
#define PSFOUNDATION_PSMATHUTILS_H

#include "foundation/PxPreprocessor.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMat33.h"
#include "Ps.h"
#include "PsIntrinsics.h"

// General guideline is: if it's an abstract math function, it belongs here.
// If it's a math function where the inputs have specific semantics (e.g.
// separateSwingTwist) it doesn't.

namespace physx
{
namespace shdfnd
{
/**
\brief sign returns the sign of its argument. The sign of zero is undefined.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 sign(const PxF32 a)
{
	return intrinsics::sign(a);
}

/**
\brief sign returns the sign of its argument. The sign of zero is undefined.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 sign(const PxF64 a)
{
	return (a >= 0.0) ? 1.0 : -1.0;
}

/**
\brief sign returns the sign of its argument. The sign of zero is undefined.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxI32 sign(const PxI32 a)
{
	return (a >= 0) ? 1 : -1;
}

/**
\brief Returns true if the two numbers are within eps of each other.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE bool equals(const PxF32 a, const PxF32 b, const PxF32 eps)
{
	return (PxAbs(a - b) < eps);
}

/**
\brief Returns true if the two numbers are within eps of each other.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE bool equals(const PxF64 a, const PxF64 b, const PxF64 eps)
{
	return (PxAbs(a - b) < eps);
}

/**
\brief The floor function returns a floating-point value representing the largest integer that is less than or equal to
x.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 floor(const PxF32 a)
{
	return floatFloor(a);
}

/**
\brief The floor function returns a floating-point value representing the largest integer that is less than or equal to
x.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 floor(const PxF64 a)
{
	return ::floor(a);
}

/**
\brief The ceil function returns a single value representing the smallest integer that is greater than or equal to x.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 ceil(const PxF32 a)
{
	return ::ceilf(a);
}

/**
\brief The ceil function returns a double value representing the smallest integer that is greater than or equal to x.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 ceil(const PxF64 a)
{
	return ::ceil(a);
}

/**
\brief mod returns the floating-point remainder of x / y.

If the value of y is 0.0, mod returns a quiet NaN.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 mod(const PxF32 x, const PxF32 y)
{
	return PxF32(::fmodf(x, y));
}

/**
\brief mod returns the floating-point remainder of x / y.

If the value of y is 0.0, mod returns a quiet NaN.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 mod(const PxF64 x, const PxF64 y)
{
	return ::fmod(x, y);
}

/**
\brief Square.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 sqr(const PxF32 a)
{
	return a * a;
}

/**
\brief Square.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 sqr(const PxF64 a)
{
	return a * a;
}

/**
\brief Calculates x raised to the power of y.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 pow(const PxF32 x, const PxF32 y)
{
	return ::powf(x, y);
}

/**
\brief Calculates x raised to the power of y.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 pow(const PxF64 x, const PxF64 y)
{
	return ::pow(x, y);
}

/**
\brief Calculates e^n
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 exp(const PxF32 a)
{
	return ::expf(a);
}
/**

\brief Calculates e^n
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 exp(const PxF64 a)
{
	return ::exp(a);
}

/**
\brief Calculates 2^n
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 exp2(const PxF32 a)
{
	return ::expf(a * 0.693147180559945309417f);
}
/**

\brief Calculates 2^n
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 exp2(const PxF64 a)
{
	return ::exp(a * 0.693147180559945309417);
}

/**
\brief Calculates logarithms.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 logE(const PxF32 a)
{
	return ::logf(a);
}

/**
\brief Calculates logarithms.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 logE(const PxF64 a)
{
	return ::log(a);
}

/**
\brief Calculates logarithms.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 log2(const PxF32 a)
{
	return ::logf(a) / 0.693147180559945309417f;
}

/**
\brief Calculates logarithms.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 log2(const PxF64 a)
{
	return ::log(a) / 0.693147180559945309417;
}

/**
\brief Calculates logarithms.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 log10(const PxF32 a)
{
	return ::log10f(a);
}

/**
\brief Calculates logarithms.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 log10(const PxF64 a)
{
	return ::log10(a);
}

/**
\brief Converts degrees to radians.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 degToRad(const PxF32 a)
{
	return 0.01745329251994329547f * a;
}

/**
\brief Converts degrees to radians.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 degToRad(const PxF64 a)
{
	return 0.01745329251994329547 * a;
}

/**
\brief Converts radians to degrees.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 radToDeg(const PxF32 a)
{
	return 57.29577951308232286465f * a;
}

/**
\brief Converts radians to degrees.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF64 radToDeg(const PxF64 a)
{
	return 57.29577951308232286465 * a;
}

//! \brief compute sine and cosine at the same time. There is a 'fsincos' on PC that we probably want to use here
PX_CUDA_CALLABLE PX_FORCE_INLINE void sincos(const PxF32 radians, PxF32& sin, PxF32& cos)
{
	/* something like:
	_asm fld  Local
	_asm fsincos
	_asm fstp LocalCos
	_asm fstp LocalSin
	*/
	sin = PxSin(radians);
	cos = PxCos(radians);
}

/**
\brief uniform random number in [a,b]
*/
PX_FORCE_INLINE PxI32 rand(const PxI32 a, const PxI32 b)
{
	return a + PxI32(::rand() % (b - a + 1));
}

/**
\brief uniform random number in [a,b]
*/
PX_FORCE_INLINE PxF32 rand(const PxF32 a, const PxF32 b)
{
	return a + (b - a) * ::rand() / RAND_MAX;
}

//! \brief return angle between two vectors in radians
PX_CUDA_CALLABLE PX_FORCE_INLINE PxF32 angle(const PxVec3& v0, const PxVec3& v1)
{
	const PxF32 cos = v0.dot(v1);                 // |v0|*|v1|*Cos(Angle)
	const PxF32 sin = (v0.cross(v1)).magnitude(); // |v0|*|v1|*Sin(Angle)
	return PxAtan2(sin, cos);
}

//! If possible use instead fsel on the dot product /*fsel(d.dot(p),onething,anotherthing);*/
//! Compares orientations (more readable, user-friendly function)
PX_CUDA_CALLABLE PX_FORCE_INLINE bool sameDirection(const PxVec3& d, const PxVec3& p)
{
	return d.dot(p) >= 0.0f;
}

//! Checks 2 values have different signs
PX_CUDA_CALLABLE PX_FORCE_INLINE IntBool differentSign(PxReal f0, PxReal f1)
{
#if !PX_EMSCRIPTEN
	union
	{
		PxU32 u;
		PxReal f;
	} u1, u2;
	u1.f = f0;
	u2.f = f1;
	return IntBool((u1.u ^ u2.u) & PX_SIGN_BITMASK);
#else
	// javascript floats are 64-bits...
	return IntBool( (f0*f1) < 0.0f );
#endif
}

PX_CUDA_CALLABLE PX_FORCE_INLINE PxMat33 star(const PxVec3& v)
{
	return PxMat33(PxVec3(0, v.z, -v.y), PxVec3(-v.z, 0, v.x), PxVec3(v.y, -v.x, 0));
}

PX_CUDA_CALLABLE PX_INLINE PxVec3 log(const PxQuat& q)
{
	const PxReal s = q.getImaginaryPart().magnitude();
	if(s < 1e-12f)
		return PxVec3(0.0f);
	// force the half-angle to have magnitude <= pi/2
	PxReal halfAngle = q.w < 0 ? PxAtan2(-s, -q.w) : PxAtan2(s, q.w);
	PX_ASSERT(halfAngle >= -PxPi / 2 && halfAngle <= PxPi / 2);

	return q.getImaginaryPart().getNormalized() * 2.f * halfAngle;
}

PX_CUDA_CALLABLE PX_INLINE PxQuat exp(const PxVec3& v)
{
	const PxReal m = v.magnitudeSquared();
	return m < 1e-24f ? PxQuat(PxIdentity) : PxQuat(PxSqrt(m), v * PxRecipSqrt(m));
}

// quat to rotate v0 t0 v1
PX_CUDA_CALLABLE PX_INLINE PxQuat rotationArc(const PxVec3& v0, const PxVec3& v1)
{
	const PxVec3 cross = v0.cross(v1);
	const PxReal d = v0.dot(v1);
	if(d <= -0.99999f)
		return (PxAbs(v0.x) < 0.1f ? PxQuat(0.0f, v0.z, -v0.y, 0.0f) : PxQuat(v0.y, -v0.x, 0.0, 0.0)).getNormalized();

	const PxReal s = PxSqrt((1 + d) * 2), r = 1 / s;

	return PxQuat(cross.x * r, cross.y * r, cross.z * r, s * 0.5f).getNormalized();
}

/**
\brief returns largest axis
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 largestAxis(const PxVec3& v)
{
	PxU32 m = PxU32(v.y > v.x ? 1 : 0);
	return v.z > v[m] ? 2 : m;
}

/**
\brief returns indices for the largest axis and 2 other axii
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 largestAxis(const PxVec3& v, PxU32& other1, PxU32& other2)
{
	if(v.x >= PxMax(v.y, v.z))
	{
		other1 = 1;
		other2 = 2;
		return 0;
	}
	else if(v.y >= v.z)
	{
		other1 = 0;
		other2 = 2;
		return 1;
	}
	else
	{
		other1 = 0;
		other2 = 1;
		return 2;
	}
}

/**
\brief returns axis with smallest absolute value
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 closestAxis(const PxVec3& v)
{
	PxU32 m = PxU32(PxAbs(v.y) > PxAbs(v.x) ? 1 : 0);
	return PxAbs(v.z) > PxAbs(v[m]) ? 2 : m;
}

PX_CUDA_CALLABLE PX_INLINE PxU32 closestAxis(const PxVec3& v, PxU32& j, PxU32& k)
{
	// find largest 2D plane projection
	const PxF32 absPx = PxAbs(v.x);
	const PxF32 absNy = PxAbs(v.y);
	const PxF32 absNz = PxAbs(v.z);

	PxU32 m = 0; // x biggest axis
	j = 1;
	k = 2;
	if(absNy > absPx && absNy > absNz)
	{
		// y biggest
		j = 2;
		k = 0;
		m = 1;
	}
	else if(absNz > absPx)
	{
		// z biggest
		j = 0;
		k = 1;
		m = 2;
	}
	return m;
}

/*!
Extend an edge along its length by a factor
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE void makeFatEdge(PxVec3& p0, PxVec3& p1, PxReal fatCoeff)
{
	PxVec3 delta = p1 - p0;

	const PxReal m = delta.magnitude();
	if(m > 0.0f)
	{
		delta *= fatCoeff / m;
		p0 -= delta;
		p1 += delta;
	}
}

//! Compute point as combination of barycentric coordinates
PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3
computeBarycentricPoint(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxReal u, PxReal v)
{
	// This seems to confuse the compiler...
	// return (1.0f - u - v)*p0 + u*p1 + v*p2;
	const PxF32 w = 1.0f - u - v;
	return PxVec3(w * p0.x + u * p1.x + v * p2.x, w * p0.y + u * p1.y + v * p2.y, w * p0.z + u * p1.z + v * p2.z);
}

// generates a pair of quaternions (swing, twist) such that in = swing * twist, with
// swing.x = 0
// twist.y = twist.z = 0, and twist is a unit quat
PX_FORCE_INLINE void separateSwingTwist(const PxQuat& q, PxQuat& swing, PxQuat& twist)
{
	twist = q.x != 0.0f ? PxQuat(q.x, 0, 0, q.w).getNormalized() : PxQuat(PxIdentity);
	swing = q * twist.getConjugate();
}

// generate two tangent vectors to a given normal
PX_FORCE_INLINE void normalToTangents(const PxVec3& normal, PxVec3& tangent0, PxVec3& tangent1)
{
	tangent0 = PxAbs(normal.x) < 0.70710678f ? PxVec3(0, -normal.z, normal.y) : PxVec3(-normal.y, normal.x, 0);
	tangent0.normalize();
	tangent1 = normal.cross(tangent0);
}

/**
\brief computes a oriented bounding box around the scaled basis.
\param basis Input = skewed basis, Output = (normalized) orthogonal basis.
\return Bounding box extent.
*/
PX_FOUNDATION_API PxVec3 optimizeBoundingBox(PxMat33& basis);

PX_FOUNDATION_API PxQuat slerp(const PxReal t, const PxQuat& left, const PxQuat& right);

PX_CUDA_CALLABLE PX_INLINE PxVec3 ellipseClamp(const PxVec3& point, const PxVec3& radii)
{
	// This function need to be implemented in the header file because
	// it is included in a spu shader program.

	// finds the closest point on the ellipse to a given point

	// (p.y, p.z) is the input point
	// (e.y, e.z) are the radii of the ellipse

	// lagrange multiplier method with Newton/Halley hybrid root-finder.
	// see http://www.geometrictools.com/Documentation/DistancePointToEllipse2.pdf
	// for proof of Newton step robustness and initial estimate.
	// Halley converges much faster but sometimes overshoots - when that happens we take
	// a newton step instead

	// converges in 1-2 iterations where D&C works well, and it's good with 4 iterations
	// with any ellipse that isn't completely crazy

	const PxU32 MAX_ITERATIONS = 20;
	const PxReal convergenceThreshold = 1e-4f;

	// iteration requires first quadrant but we recover generality later

	PxVec3 q(0, PxAbs(point.y), PxAbs(point.z));
	const PxReal tinyEps = 1e-6f; // very close to minor axis is numerically problematic but trivial
	if(radii.y >= radii.z)
	{
		if(q.z < tinyEps)
			return PxVec3(0, point.y > 0 ? radii.y : -radii.y, 0);
	}
	else
	{
		if(q.y < tinyEps)
			return PxVec3(0, 0, point.z > 0 ? radii.z : -radii.z);
	}

	PxVec3 denom, e2 = radii.multiply(radii), eq = radii.multiply(q);

	// we can use any initial guess which is > maximum(-e.y^2,-e.z^2) and for which f(t) is > 0.
	// this guess works well near the axes, but is weak along the diagonals.

	PxReal t = PxMax(eq.y - e2.y, eq.z - e2.z);

	for(PxU32 i = 0; i < MAX_ITERATIONS; i++)
	{
		denom = PxVec3(0, 1 / (t + e2.y), 1 / (t + e2.z));
		PxVec3 denom2 = eq.multiply(denom);

		PxVec3 fv = denom2.multiply(denom2);
		PxReal f = fv.y + fv.z - 1;

		// although in exact arithmetic we are guaranteed f>0, we can get here
		// on the first iteration via catastrophic cancellation if the point is
		// very close to the origin. In that case we just behave as if f=0

		if(f < convergenceThreshold)
			return e2.multiply(point).multiply(denom);

		PxReal df = fv.dot(denom) * -2.0f;
		t = t - f / df;
	}

	// we didn't converge, so clamp what we have
	PxVec3 r = e2.multiply(point).multiply(denom);
	return r * PxRecipSqrt(sqr(r.y / radii.y) + sqr(r.z / radii.z));
}

PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal tanHalf(PxReal sin, PxReal cos)
{
	// PT: avoids divide by zero for singularity. We return sqrt(FLT_MAX) instead of FLT_MAX
	// to make sure the calling code doesn't generate INF values when manipulating the returned value
	// (some joints multiply it by 4, etc).
	if(cos==-1.0f)
		return sin<0.0f ? -sqrtf(FLT_MAX) : sqrtf(FLT_MAX);

	// PT: half-angle formula: tan(a/2) = sin(a)/(1+cos(a))
	return sin / (1.0f + cos);
}

PX_INLINE PxQuat quatFromTanQVector(const PxVec3& v)
{
	PxReal v2 = v.dot(v);
	if(v2 < 1e-12f)
		return PxQuat(PxIdentity);
	PxReal d = 1 / (1 + v2);
	return PxQuat(v.x * 2, v.y * 2, v.z * 2, 1 - v2) * d;
}

PX_FORCE_INLINE PxVec3 cross100(const PxVec3& b)
{
	return PxVec3(0.0f, -b.z, b.y);
}
PX_FORCE_INLINE PxVec3 cross010(const PxVec3& b)
{
	return PxVec3(b.z, 0.0f, -b.x);
}
PX_FORCE_INLINE PxVec3 cross001(const PxVec3& b)
{
	return PxVec3(-b.y, b.x, 0.0f);
}

PX_INLINE void decomposeVector(PxVec3& normalCompo, PxVec3& tangentCompo, const PxVec3& outwardDir,
                               const PxVec3& outwardNormal)
{
	normalCompo = outwardNormal * (outwardDir.dot(outwardNormal));
	tangentCompo = outwardDir - normalCompo;
}

//! \brief Return (i+1)%3
// Avoid variable shift for XBox:
// PX_INLINE PxU32 Ps::getNextIndex3(PxU32 i)			{	return (1<<i) & 3;			}
PX_INLINE PxU32 getNextIndex3(PxU32 i)
{
	return (i + 1 + (i >> 1)) & 3;
}

PX_INLINE PxMat33 rotFrom2Vectors(const PxVec3& from, const PxVec3& to)
{
	// See bottom of http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/index.htm

	// Early exit if to = from
	if((from - to).magnitudeSquared() < 1e-4f)
		return PxMat33(PxIdentity);

	// Early exit if to = -from
	if((from + to).magnitudeSquared() < 1e-4f)
		return PxMat33::createDiagonal(PxVec3(1.0f, -1.0f, -1.0f));

	PxVec3 n = from.cross(to);

	PxReal C = from.dot(to), S = PxSqrt(1 - C * C), CC = 1 - C;

	PxReal xx = n.x * n.x, yy = n.y * n.y, zz = n.z * n.z, xy = n.x * n.y, yz = n.y * n.z, xz = n.x * n.z;

	PxMat33 R;

	R(0, 0) = 1 + CC * (xx - 1);
	R(0, 1) = -n.z * S + CC * xy;
	R(0, 2) = n.y * S + CC * xz;

	R(1, 0) = n.z * S + CC * xy;
	R(1, 1) = 1 + CC * (yy - 1);
	R(1, 2) = -n.x * S + CC * yz;

	R(2, 0) = -n.y * S + CC * xz;
	R(2, 1) = n.x * S + CC * yz;
	R(2, 2) = 1 + CC * (zz - 1);

	return R;
}

PX_FOUNDATION_API void integrateTransform(const PxTransform& curTrans, const PxVec3& linvel, const PxVec3& angvel,
                                          PxReal timeStep, PxTransform& result);

PX_INLINE void computeBasis(const PxVec3& dir, PxVec3& right, PxVec3& up)
{
	// Derive two remaining vectors
	if(PxAbs(dir.y) <= 0.9999f)
	{
		right = PxVec3(dir.z, 0.0f, -dir.x);
		right.normalize();

		// PT: normalize not needed for 'up' because dir & right are unit vectors,
		// and by construction the angle between them is 90 degrees (i.e. sin(angle)=1)
		up = PxVec3(dir.y * right.z, dir.z * right.x - dir.x * right.z, -dir.y * right.x);
	}
	else
	{
		right = PxVec3(1.0f, 0.0f, 0.0f);

		up = PxVec3(0.0f, dir.z, -dir.y);
		up.normalize();
	}
}

PX_INLINE void computeBasis(const PxVec3& p0, const PxVec3& p1, PxVec3& dir, PxVec3& right, PxVec3& up)
{
	// Compute the new direction vector
	dir = p1 - p0;
	dir.normalize();

	// Derive two remaining vectors
	computeBasis(dir, right, up);
}

PX_FORCE_INLINE bool isAlmostZero(const PxVec3& v)
{
	if(PxAbs(v.x) > 1e-6f || PxAbs(v.y) > 1e-6f || PxAbs(v.z) > 1e-6f)
		return false;
	return true;
}

} // namespace shdfnd
} // namespace physx

#endif
