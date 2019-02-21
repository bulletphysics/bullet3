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

#include "foundation/PxMat33.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxVec4.h"
#include "foundation/PxAssert.h"
#include "PsMathUtils.h"
#include "PsUtilities.h"
#include "PsBasicTemplates.h"

using namespace physx;
using namespace physx::shdfnd;
using namespace physx::intrinsics;

PX_FOUNDATION_API PxQuat physx::PxShortestRotation(const PxVec3& v0, const PxVec3& v1)
{
	const PxReal d = v0.dot(v1);
	const PxVec3 cross = v0.cross(v1);

	PxQuat q = d > -1 ? PxQuat(cross.x, cross.y, cross.z, 1 + d) : PxAbs(v0.x) < 0.1f ? PxQuat(0.0f, v0.z, -v0.y, 0.0f)
	                                                                                  : PxQuat(v0.y, -v0.x, 0.0f, 0.0f);

	return q.getNormalized();
}

namespace
{
// indexed rotation around axis, with sine and cosine of half-angle
PxQuat indexedRotation(PxU32 axis, PxReal s, PxReal c)
{
	PxReal v[3] = { 0, 0, 0 };
	v[axis] = s;
	return PxQuat(v[0], v[1], v[2], c);
}
}

PX_FOUNDATION_API PxVec3 physx::PxDiagonalize(const PxMat33& m, PxQuat& massFrame)
{
	// jacobi rotation using quaternions (from an idea of Stan Melax, with fix for precision issues)

	const PxU32 MAX_ITERS = 24;

	PxQuat q = PxQuat(PxIdentity);

	PxMat33 d;
	for(PxU32 i = 0; i < MAX_ITERS; i++)
	{
		PxMat33 axes(q);
		d = axes.getTranspose() * m * axes;

		PxReal d0 = PxAbs(d[1][2]), d1 = PxAbs(d[0][2]), d2 = PxAbs(d[0][1]);
		PxU32 a = PxU32(d0 > d1 && d0 > d2 ? 0 : d1 > d2 ? 1 : 2); // rotation axis index, from largest off-diagonal
		// element

		PxU32 a1 = shdfnd::getNextIndex3(a), a2 = shdfnd::getNextIndex3(a1);
		if(d[a1][a2] == 0.0f || PxAbs(d[a1][a1] - d[a2][a2]) > 2e6f * PxAbs(2.0f * d[a1][a2]))
			break;

		PxReal w = (d[a1][a1] - d[a2][a2]) / (2.0f * d[a1][a2]); // cot(2 * phi), where phi is the rotation angle
		PxReal absw = PxAbs(w);

		PxQuat r;
		if(absw > 1000)
			r = indexedRotation(a, 1 / (4 * w), 1.f); // h will be very close to 1, so use small angle approx instead
		else
		{
			PxReal t = 1 / (absw + PxSqrt(w * w + 1)); // absolute value of tan phi
			PxReal h = 1 / PxSqrt(t * t + 1);          // absolute value of cos phi

			PX_ASSERT(h != 1); // |w|<1000 guarantees this with typical IEEE754 machine eps (approx 6e-8)
			r = indexedRotation(a, PxSqrt((1 - h) / 2) * PxSign(w), PxSqrt((1 + h) / 2));
		}

		q = (q * r).getNormalized();
	}

	massFrame = q;
	return PxVec3(d.column0.x, d.column1.y, d.column2.z);
}

/**
\brief computes a oriented bounding box around the scaled basis.
\param basis Input = skewed basis, Output = (normalized) orthogonal basis.
\return Bounding box extent.
*/
PxVec3 physx::shdfnd::optimizeBoundingBox(PxMat33& basis)
{
	PxVec3* PX_RESTRICT vec = &basis[0]; // PT: don't copy vectors if not needed...

	// PT: since we store the magnitudes to memory, we can avoid the FCMPs afterwards
	PxVec3 magnitude(vec[0].magnitudeSquared(), vec[1].magnitudeSquared(), vec[2].magnitudeSquared());

	// find indices sorted by magnitude
	unsigned int i = magnitude[1] > magnitude[0] ? 1 : 0u;
	unsigned int j = magnitude[2] > magnitude[1 - i] ? 2 : 1 - i;
	const unsigned int k = 3 - i - j;

	if(magnitude[i] < magnitude[j])
		swap(i, j);

	PX_ASSERT(magnitude[i] >= magnitude[j] && magnitude[i] >= magnitude[k] && magnitude[j] >= magnitude[k]);

	// ortho-normalize basis

	PxReal invSqrt = PxRecipSqrt(magnitude[i]);
	magnitude[i] *= invSqrt;
	vec[i] *= invSqrt; // normalize the first axis
	PxReal dotij = vec[i].dot(vec[j]);
	PxReal dotik = vec[i].dot(vec[k]);
	magnitude[i] += PxAbs(dotij) + PxAbs(dotik); // elongate the axis by projection of the other two
	vec[j] -= vec[i] * dotij;                    // orthogonize the two remaining axii relative to vec[i]
	vec[k] -= vec[i] * dotik;

	magnitude[j] = vec[j].normalize();
	PxReal dotjk = vec[j].dot(vec[k]);
	magnitude[j] += PxAbs(dotjk); // elongate the axis by projection of the other one
	vec[k] -= vec[j] * dotjk;     // orthogonize vec[k] relative to vec[j]

	magnitude[k] = vec[k].normalize();

	return magnitude;
}

PxQuat physx::shdfnd::slerp(const PxReal t, const PxQuat& left, const PxQuat& right)
{
	const PxReal quatEpsilon = (PxReal(1.0e-8f));

	PxReal cosine = left.dot(right);
	PxReal sign = PxReal(1);
	if(cosine < 0)
	{
		cosine = -cosine;
		sign = PxReal(-1);
	}

	PxReal sine = PxReal(1) - cosine * cosine;

	if(sine >= quatEpsilon * quatEpsilon)
	{
		sine = PxSqrt(sine);
		const PxReal angle = PxAtan2(sine, cosine);
		const PxReal i_sin_angle = PxReal(1) / sine;

		const PxReal leftw = PxSin(angle * (PxReal(1) - t)) * i_sin_angle;
		const PxReal rightw = PxSin(angle * t) * i_sin_angle * sign;

		return left * leftw + right * rightw;
	}

	return left;
}

void physx::shdfnd::integrateTransform(const PxTransform& curTrans, const PxVec3& linvel, const PxVec3& angvel,
                                       PxReal timeStep, PxTransform& result)
{
	result.p = curTrans.p + linvel * timeStep;

	// from void DynamicsContext::integrateAtomPose(PxsRigidBody* atom, Cm::BitMap &shapeChangedMap) const:
	// Integrate the rotation using closed form quaternion integrator
	PxReal w = angvel.magnitudeSquared();

	if(w != 0.0f)
	{
		w = PxSqrt(w);
		if(w != 0.0f)
		{
			const PxReal v = timeStep * w * 0.5f;
			const PxReal q = PxCos(v);
			const PxReal s = PxSin(v) / w;

			const PxVec3 pqr = angvel * s;
			const PxQuat quatVel(pqr.x, pqr.y, pqr.z, 0);
			PxQuat out; // need to have temporary, otherwise we may overwrite input if &curTrans == &result.
			out = quatVel * curTrans.q;
			out.x += curTrans.q.x * q;
			out.y += curTrans.q.y * q;
			out.z += curTrans.q.z * q;
			out.w += curTrans.q.w * q;
			result.q = out;
			return;
		}
	}
	// orientation stays the same - convert from quat to matrix:
	result.q = curTrans.q;
}
