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

#ifndef GU_SPHERE_H
#define GU_SPHERE_H
/** \addtogroup geomutils
@{
*/

#include "foundation/PxVec3.h"
#include "CmPhysXCommon.h"

namespace physx
{

/**
\brief Represents a sphere defined by its center point and radius.
*/
namespace Gu
{
	class Sphere
	{
	public:
		/**
		\brief Constructor
		*/
		PX_INLINE Sphere()
		{
		}

		/**
		\brief Constructor
		*/
		PX_INLINE Sphere(const PxVec3& _center, PxF32 _radius) : center(_center), radius(_radius)
		{
		}
		/**
		\brief Copy constructor
		*/
		PX_INLINE Sphere(const Sphere& sphere) : center(sphere.center), radius(sphere.radius)
		{
		}
		/**
		\brief Destructor
		*/
		PX_INLINE ~Sphere()
		{
		}

		PX_INLINE	void	set(const PxVec3& _center, float _radius)		{ center = _center; radius = _radius;	}

		/**
		\brief Checks the sphere is valid.

		\return		true if the sphere is valid
		*/
		PX_INLINE bool isValid() const
		{
			// Consistency condition for spheres: Radius >= 0.0f
			return radius >= 0.0f;
		}

		/**
		\brief Tests if a point is contained within the sphere.

		\param[in] p the point to test
		\return	true if inside the sphere
		*/
		PX_INLINE bool contains(const PxVec3& p) const
		{
			return (center-p).magnitudeSquared() <= radius*radius;
		}

		PxVec3	center;		//!< Sphere's center
		PxF32	radius;		//!< Sphere's radius
	};
}

}

/** @} */
#endif
