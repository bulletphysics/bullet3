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

#ifndef PXFOUNDATION_PXPLANE_H
#define PXFOUNDATION_PXPLANE_H

/** \addtogroup foundation
@{
*/

#include "foundation/PxMath.h"
#include "foundation/PxVec3.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Representation of a plane.

 Plane equation used: n.dot(v) + d = 0
*/
class PxPlane
{
  public:
	/**
	\brief Constructor
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxPlane()
	{
	}

	/**
	\brief Constructor from a normal and a distance
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxPlane(float nx, float ny, float nz, float distance) : n(nx, ny, nz), d(distance)
	{
	}

	/**
	\brief Constructor from a normal and a distance
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxPlane(const PxVec3& normal, float distance) : n(normal), d(distance)
	{
	}

	/**
	\brief Constructor from a point on the plane and a normal
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxPlane(const PxVec3& point, const PxVec3& normal)
	: n(normal), d(-point.dot(n)) // p satisfies normal.dot(p) + d = 0
	{
	}

	/**
	\brief Constructor from three points
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxPlane(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2)
	{
		n = (p1 - p0).cross(p2 - p0).getNormalized();
		d = -p0.dot(n);
	}

	/**
	\brief returns true if the two planes are exactly equal
	*/
	PX_CUDA_CALLABLE PX_INLINE bool operator==(const PxPlane& p) const
	{
		return n == p.n && d == p.d;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE float distance(const PxVec3& p) const
	{
		return p.dot(n) + d;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE bool contains(const PxVec3& p) const
	{
		return PxAbs(distance(p)) < (1.0e-7f);
	}

	/**
	\brief projects p into the plane
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 project(const PxVec3& p) const
	{
		return p - n * distance(p);
	}

	/**
	\brief find an arbitrary point in the plane
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 pointInPlane() const
	{
		return -n * d;
	}

	/**
	\brief equivalent plane with unit normal
	*/

	PX_CUDA_CALLABLE PX_FORCE_INLINE void normalize()
	{
		float denom = 1.0f / n.magnitude();
		n *= denom;
		d *= denom;
	}

	PxVec3 n; //!< The normal to the plane
	float d;  //!< The distance from the origin
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // #ifndef PXFOUNDATION_PXPLANE_H
