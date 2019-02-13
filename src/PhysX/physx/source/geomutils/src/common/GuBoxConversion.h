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

#ifndef GU_BOX_CONVERSION_H
#define GU_BOX_CONVERSION_H

#include "GuBox.h"
#include "PsMathUtils.h"
#include "CmMatrix34.h"
#include "PsVecMath.h"

namespace physx
{
	// PT: builds rot from quat. WARNING: writes 4 bytes after 'dst.rot'.
	PX_FORCE_INLINE void buildFrom(Gu::Box& dst, const PxQuat& q)
	{
		using namespace Ps::aos;
		const QuatV qV = V4LoadU(&q.x);
		Vec3V column0, column1, column2;
		QuatGetMat33V(qV, column0, column1, column2);
		// PT: TODO: investigate if these overlapping stores are a problem
		V4StoreU(Vec4V_From_Vec3V(column0), &dst.rot.column0.x);
		V4StoreU(Vec4V_From_Vec3V(column1), &dst.rot.column1.x);
		V4StoreU(Vec4V_From_Vec3V(column2), &dst.rot.column2.x);
	}

	PX_FORCE_INLINE void buildFrom(Gu::Box& dst, const PxVec3& center, const PxVec3& extents, const PxQuat& q)
	{
		using namespace Ps::aos;
		// PT: writes 4 bytes after 'rot' but it's safe since we then write 'center' just afterwards
		buildFrom(dst, q);
		dst.center	= center;
		dst.extents	= extents;
	}

	PX_FORCE_INLINE void buildMatrixFromBox(Cm::Matrix34& mat34, const Gu::Box& box)
	{
		mat34.m	= box.rot;
		mat34.p	= box.center;
	}

	// SD: function is now the same as FastVertex2ShapeScaling::transformQueryBounds
	// PT: lots of LHS in that one. TODO: revisit...
	PX_INLINE Gu::Box transform(const Cm::Matrix34& transfo, const Gu::Box& box)
	{
		Gu::Box ret;
		PxMat33& obbBasis = ret.rot;

		obbBasis.column0 = transfo.rotate(box.rot.column0 * box.extents.x);
		obbBasis.column1 = transfo.rotate(box.rot.column1 * box.extents.y);
		obbBasis.column2 = transfo.rotate(box.rot.column2 * box.extents.z);

		ret.center = transfo.transform(box.center);
		ret.extents = Ps::optimizeBoundingBox(obbBasis);
		return ret;
	}

	PX_INLINE Gu::Box transformBoxOrthonormal(const Gu::Box& box, const PxTransform& t)
	{
		Gu::Box ret;
		PxMat33& obbBasis = ret.rot;
		obbBasis.column0 = t.rotate(box.rot.column0);
		obbBasis.column1 = t.rotate(box.rot.column1);
		obbBasis.column2 = t.rotate(box.rot.column2);
		ret.center = t.transform(box.center);
		ret.extents = box.extents;
		return ret;
	}

	/**
	\brief recomputes the OBB after an arbitrary transform by a 4x4 matrix.
	\param	mtx		[in] the transform matrix
	\param	obb		[out] the transformed OBB
	*/
	PX_INLINE	void rotate(const Gu::Box& src, const Cm::Matrix34& mtx, Gu::Box& obb)
	{
		// The extents remain constant
		obb.extents = src.extents;
		// The center gets x-formed
		obb.center = mtx.transform(src.center);
		// Combine rotations
		obb.rot = mtx.m * src.rot;
	}

// PT: TODO: move this to a better place
	PX_FORCE_INLINE void getInverse(PxMat33& dstRot, PxVec3& dstTrans, const PxMat33& srcRot, const PxVec3& srcTrans)
	{
		const PxMat33 invRot = srcRot.getInverse();
		dstTrans = invRot.transform(-srcTrans);
		dstRot = invRot;
	}

}

#endif
