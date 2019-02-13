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

#ifndef GU_SIMD_HELPERS_H
#define GU_SIMD_HELPERS_H

#include "PxPhysXCommonConfig.h"
#include "CmPhysXCommon.h"
#include "geometry/PxTriangle.h"
#include "foundation/PxMat33.h"
#include "PsVecMath.h"

namespace physx
{
namespace Gu
{
	//! A padded version of PxTriangle, to safely load its data using SIMD
	class TrianglePadded : public PxTriangle
	{
	public:
		PX_FORCE_INLINE TrianglePadded()	{}
		PX_FORCE_INLINE ~TrianglePadded()	{}
		PxU32	padding;
	};

	// PT: wrapper helper class to make sure we can safely load a PxVec3 using SIMD loads
	// PT: TODO: refactor with PxVec3Pad
	class Vec3p : public PxVec3
	{
		public:
		PX_FORCE_INLINE	Vec3p()								{}
		PX_FORCE_INLINE	~Vec3p()							{}
		PX_FORCE_INLINE	Vec3p(const PxVec3& p) : PxVec3(p)	{}
		PX_FORCE_INLINE	Vec3p(float f) : PxVec3(f)			{}
		PxU32	padding;
	};
	PX_COMPILE_TIME_ASSERT(sizeof(Vec3p) == 16);

	//! A padded version of PxMat33, to safely load its data using SIMD
	class PxMat33Padded : public PxMat33
	{
	public:
		explicit PX_FORCE_INLINE PxMat33Padded(const PxQuat& q)
		{
			using namespace Ps::aos;
			const QuatV qV = V4LoadU(&q.x);
			Vec3V column0V, column1V, column2V;
			QuatGetMat33V(qV, column0V, column1V, column2V);
#if defined(PX_SIMD_DISABLED) || PX_ANDROID || (PX_LINUX && (PX_ARM || PX_A64))
			V3StoreU(column0V, column0);
			V3StoreU(column1V, column1);
			V3StoreU(column2V, column2);
#else
			V4StoreU(column0V, &column0.x);
			V4StoreU(column1V, &column1.x);
			V4StoreU(column2V, &column2.x);
#endif
		}
		PX_FORCE_INLINE ~PxMat33Padded()				{}
		PX_FORCE_INLINE void operator=(const PxMat33& other)
		{
			column0 = other.column0;
			column1 = other.column1;
			column2 = other.column2;
		}
		PxU32	padding;
	};

} // namespace Gu
}

#endif
