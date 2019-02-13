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

#include "GuIntersectionTriangleBox.h"
#include "GuIntersectionTriangleBoxRef.h"
#include "CmMatrix34.h"
#include "PsVecMath.h"
#include "GuBox.h"
#include "GuSIMDHelpers.h"

using namespace physx;


Ps::IntBool Gu::intersectTriangleBox_ReferenceCode(const PxVec3& boxcenter, const PxVec3& extents, const PxVec3& tp0, const PxVec3& tp1, const PxVec3& tp2)
{
	return intersectTriangleBox_RefImpl(boxcenter, extents, tp0, tp1, tp2);
}


using namespace Ps::aos;

static PX_FORCE_INLINE int testClassIIIAxes(const Vec4V& e0V, const Vec4V v0V, const Vec4V v1V, const Vec4V v2V, const PxVec3& extents)
{
	const Vec4V e0XZY_V = V4PermYZXW(e0V);

	const Vec4V v0XZY_V = V4PermYZXW(v0V);
	const Vec4V p0V = V4NegMulSub(v0XZY_V, e0V, V4Mul(v0V, e0XZY_V));

	const Vec4V v1XZY_V = V4PermYZXW(v1V);
	const Vec4V p1V = V4NegMulSub(v1XZY_V, e0V, V4Mul(v1V, e0XZY_V));

	const Vec4V v2XZY_V = V4PermYZXW(v2V);
	const Vec4V p2V = V4NegMulSub(v2XZY_V, e0V, V4Mul(v2V, e0XZY_V));

	Vec4V minV = V4Min(p0V, p1V);
	minV = V4Min(minV, p2V);

	const Vec4V extentsV = V4LoadU(&extents.x);
	const Vec4V fe0ZYX_V = V4Abs(e0V);

	const Vec4V fe0XZY_V = V4PermYZXW(fe0ZYX_V);
	const Vec4V extentsXZY_V = V4PermYZXW(extentsV);
	Vec4V radV = V4MulAdd(extentsV, fe0XZY_V, V4Mul(extentsXZY_V, fe0ZYX_V));

	if(V4AnyGrtr3(minV, radV))
		return 0;

	Vec4V maxV = V4Max(p0V, p1V);
	maxV = V4Max(maxV, p2V);

	radV = V4Sub(V4Zero(), radV);

	if(V4AnyGrtr3(radV, maxV))
		return 0;
	return 1;
}

static const VecU32V signV = U4LoadXYZW(0x80000000, 0x80000000, 0x80000000, 0x80000000);

static PX_FORCE_INLINE Ps::IntBool intersectTriangleBoxInternal(const Vec4V v0V, const Vec4V v1V, const Vec4V v2V, const PxVec3& extents)
{
	// Test box axes
	{
		Vec4V extentsV = V4LoadU(&extents.x);

		{
			const Vec4V cV = V4Abs(v0V);
			if(V4AllGrtrOrEq3(extentsV, cV))
				return 1;
		}

		Vec4V minV = V4Min(v0V, v1V);
		minV = V4Min(minV, v2V);

		if(V4AnyGrtr3(minV, extentsV))
			return 0;

		Vec4V maxV = V4Max(v0V, v1V);
		maxV = V4Max(maxV, v2V);
		extentsV = V4Sub(V4Zero(), extentsV);

		if(V4AnyGrtr3(extentsV, maxV))
			return 0;
	}

	// Test if the box intersects the plane of the triangle
	const Vec4V e0V = V4Sub(v1V, v0V);
	const Vec4V e1V = V4Sub(v2V, v1V);
	{
		const Vec4V normalV = V4Cross(e0V, e1V);
		const Vec4V dV = Vec4V_From_FloatV(V4Dot3(normalV, v0V));

		const Vec4V extentsV = V4LoadU(&extents.x);
		VecU32V normalSignsV = V4U32and(VecU32V_ReinterpretFrom_Vec4V(normalV), signV);
		const Vec4V maxV = Vec4V_ReinterpretFrom_VecU32V(V4U32or(VecU32V_ReinterpretFrom_Vec4V(extentsV), normalSignsV));

		Vec4V tmpV = Vec4V_From_FloatV(V4Dot3(normalV, maxV));
		if(V4AnyGrtr3(dV, tmpV))
			return 0;

		normalSignsV = V4U32xor(normalSignsV, signV);
		const Vec4V minV = Vec4V_ReinterpretFrom_VecU32V(V4U32or(VecU32V_ReinterpretFrom_Vec4V(extentsV), normalSignsV));

		tmpV = Vec4V_From_FloatV(V4Dot3(normalV, minV));
		if(V4AnyGrtr3(tmpV, dV))
			return 0;
	}

	// Edge-edge tests
	{
		if(!testClassIIIAxes(e0V, v0V, v1V, v2V, extents))
			return 0;
		if(!testClassIIIAxes(e1V, v0V, v1V, v2V, extents))
			return 0;
		const Vec4V e2V = V4Sub(v0V, v2V);
		if(!testClassIIIAxes(e2V, v0V, v1V, v2V, extents))
			return 0;
	}
	return 1;
}

// PT: a SIMD version of Tomas Moller's triangle-box SAT code
Ps::IntBool Gu::intersectTriangleBox_Unsafe(const PxVec3& center, const PxVec3& extents, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2)
{
	// Move everything so that the boxcenter is in (0,0,0)
	const Vec4V BoxCenterV = V4LoadU(&center.x);
	const Vec4V v0V = V4Sub(V4LoadU(&p0.x), BoxCenterV);
	const Vec4V v1V = V4Sub(V4LoadU(&p1.x), BoxCenterV);
	const Vec4V v2V = V4Sub(V4LoadU(&p2.x), BoxCenterV);

	return intersectTriangleBoxInternal(v0V, v1V, v2V, extents);
}

Ps::IntBool Gu::intersectTriangleBox(const BoxPadded& box, const PxVec3& p0_, const PxVec3& p1_, const PxVec3& p2_)
{
	// PT: TODO: SIMDify this part

	// Vec3p ensures we can safely V4LoadU the data
	const Vec3p p0 = box.rotateInv(p0_ - box.center);
	const Vec3p p1 = box.rotateInv(p1_ - box.center);
	const Vec3p p2 = box.rotateInv(p2_ - box.center);

	const Vec4V v0V = V4LoadU(&p0.x);
	const Vec4V v1V = V4LoadU(&p1.x);
	const Vec4V v2V = V4LoadU(&p2.x);

	return intersectTriangleBoxInternal(v0V, v1V, v2V, box.extents);
}

static PX_FORCE_INLINE Vec4V multiply3x3V(const Vec4V p, const PxMat33& mat)	
{
	const FloatV xxxV = V4GetX(p);
	const FloatV yyyV = V4GetY(p);
	const FloatV zzzV = V4GetZ(p);

	Vec4V ResV = V4Scale(V4LoadU(&mat.column0.x), xxxV);
	ResV = V4Add(ResV, V4Scale(V4LoadU(&mat.column1.x), yyyV));
	ResV = V4Add(ResV, V4Scale(V4LoadU(&mat.column2.x), zzzV));
	return ResV;
}

// PT: warning: all params must be safe to V4LoadU
Ps::IntBool intersectTriangleBoxBV4(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2,
									const PxMat33& rotModelToBox, const PxVec3& transModelToBox, const PxVec3& extents)
{
	const Vec4V transModelToBoxV = V4LoadU(&transModelToBox.x);
	const Vec4V v0V = V4Add(multiply3x3V(V4LoadU(&p0.x), rotModelToBox), transModelToBoxV);
	const Vec4V v1V = V4Add(multiply3x3V(V4LoadU(&p1.x), rotModelToBox), transModelToBoxV);
	const Vec4V v2V = V4Add(multiply3x3V(V4LoadU(&p2.x), rotModelToBox), transModelToBoxV);

	return intersectTriangleBoxInternal(v0V, v1V, v2V, extents);
}
