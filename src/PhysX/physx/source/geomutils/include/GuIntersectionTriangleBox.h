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

#ifndef GU_INTERSECTION_TRIANGLE_BOX_H
#define GU_INTERSECTION_TRIANGLE_BOX_H

#include "PxPhysXCommonConfig.h"
#include "CmPhysXCommon.h"
#include "foundation/PxMat33.h"

namespace physx
{
namespace Gu
{
	class Box;
	class BoxPadded;

	/**
	Tests if a triangle overlaps a box (AABB). This is the reference non-SIMD code.

	\param center	[in] the box center
	\param extents	[in] the box extents
	\param p0		[in] triangle's first point
	\param p1		[in] triangle's second point
	\param p2		[in] triangle's third point
	\return	true if triangle overlaps box
	*/
	PX_PHYSX_COMMON_API Ps::IntBool intersectTriangleBox_ReferenceCode(const PxVec3& center, const PxVec3& extents, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2);

	/**
	Tests if a triangle overlaps a box (AABB). This is the optimized SIMD code.

	WARNING: the function has various SIMD requirements, left to the calling code:
	- function will load 4 bytes after 'center'. Make sure it's safe to load from there.
	- function will load 4 bytes after 'extents'. Make sure it's safe to load from there.
	- function will load 4 bytes after 'p0'. Make sure it's safe to load from there.
	- function will load 4 bytes after 'p1'. Make sure it's safe to load from there.
	- function will load 4 bytes after 'p2'. Make sure it's safe to load from there.
	If you can't guarantee these requirements, please use the non-SIMD reference code instead.

	\param center	[in] the box center. 
	\param extents	[in] the box extents
	\param p0		[in] triangle's first point
	\param p1		[in] triangle's second point
	\param p2		[in] triangle's third point
	\return	true if triangle overlaps box
	*/
	PX_PHYSX_COMMON_API Ps::IntBool intersectTriangleBox_Unsafe(const PxVec3& center, const PxVec3& extents, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2);

	/**
	Tests if a triangle overlaps a box (OBB).

	There are currently no SIMD-related requirements for p0, p1, p2.

	\param box		[in] the box
	\param p0		[in] triangle's first point
	\param p1		[in] triangle's second point
	\param p2		[in] triangle's third point
	\return	true if triangle overlaps box
	*/
	PX_PHYSX_COMMON_API Ps::IntBool intersectTriangleBox(const BoxPadded& box, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2);
} // namespace Gu
}

#endif
