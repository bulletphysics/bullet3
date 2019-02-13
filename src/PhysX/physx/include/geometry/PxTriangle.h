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


#ifndef PX_PHYSICS_GEOMUTILS_PX_TRIANGLE
#define PX_PHYSICS_GEOMUTILS_PX_TRIANGLE
/** \addtogroup geomutils
  @{
*/

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxVec3.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Triangle class.
*/
class PxTriangle
{
	public:
	/**
	\brief Constructor
	*/
	PX_FORCE_INLINE			PxTriangle() {}

	/**
	\brief Constructor

	\param[in] p0 Point 0
	\param[in] p1 Point 1
	\param[in] p2 Point 2
	*/
	PX_FORCE_INLINE			PxTriangle(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2)
	{
		verts[0] = p0;
		verts[1] = p1;
		verts[2] = p2;
	}

	/**
	\brief Copy constructor

	\param[in] triangle Tri to copy
	*/
	PX_FORCE_INLINE			PxTriangle(const PxTriangle& triangle)
	{
		verts[0] = triangle.verts[0];
		verts[1] = triangle.verts[1];
		verts[2] = triangle.verts[2];
	}

	/**
	\brief Destructor
	*/
	PX_FORCE_INLINE			~PxTriangle() {}

	/**
	\brief Assignment operator
	*/
	PX_FORCE_INLINE void operator=(const PxTriangle& triangle)
	{
		verts[0] = triangle.verts[0];
		verts[1] = triangle.verts[1];
		verts[2] = triangle.verts[2];
	}

	/**
	\brief Compute the normal of the Triangle.

	\param[out] _normal Triangle normal.
	*/
	PX_FORCE_INLINE	void	normal(PxVec3& _normal) const
	{
		_normal = (verts[1]-verts[0]).cross(verts[2]-verts[0]);
		_normal.normalize();
	}

	/**
	\brief Compute the unnormalized normal of the triangle.

	\param[out] _normal Triangle normal (not normalized).
	*/
	PX_FORCE_INLINE	void	denormalizedNormal(PxVec3& _normal) const
	{
		_normal = (verts[1]-verts[0]).cross(verts[2]-verts[0]);
	}

	/**
	\brief Compute the area of the triangle.

	\return Area of the triangle.
	*/
	PX_FORCE_INLINE	PxReal	area() const
	{
		const PxVec3& p0 = verts[0];
		const PxVec3& p1 = verts[1];
		const PxVec3& p2 = verts[2];
		return ((p0 - p1).cross(p0 - p2)).magnitude() * 0.5f;
	}

	/**
	\return Computes a point on the triangle from u and v barycentric coordinates.
	*/
	PxVec3 pointFromUV(PxReal u, PxReal v) const { return (1.0f-u-v)*verts[0] + u*verts[1] + v*verts[2]; }

	/**
	\brief Array of Vertices.
	*/
	PxVec3		verts[3];

};


#if !PX_DOXYGEN
}
#endif

/** @} */
#endif
