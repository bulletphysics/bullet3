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


#ifndef PX_PHYSICS_NX_SPHERE_GEOMETRY
#define PX_PHYSICS_NX_SPHERE_GEOMETRY
/** \addtogroup geomutils
@{
*/
#include "geometry/PxGeometry.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief A class representing the geometry of a sphere.

Spheres are defined by their radius.
\note The scaling of the sphere is expected to be baked into this value, there is no additional scaling parameter.
*/
class PxSphereGeometry : public PxGeometry 
{
public:
	PX_INLINE PxSphereGeometry() :							PxGeometry(PxGeometryType::eSPHERE), radius(0) {}
	PX_INLINE PxSphereGeometry(PxReal ir) :					PxGeometry(PxGeometryType::eSPHERE), radius(ir) {}

	/**
	\brief Returns true if the geometry is valid.

	\return True if the current settings are valid

	\note A valid sphere has radius > 0.  
	It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a sphere that has zero radius.

	@see PxRigidActor::createShape, PxPhysics::createShape
	*/
	PX_INLINE bool isValid() const;

public:

	/**
	\brief The radius of the sphere.
	*/
	PxReal radius;	
};


PX_INLINE bool PxSphereGeometry::isValid() const
{
	if (mType != PxGeometryType::eSPHERE)
		return false;
	if (!PxIsFinite(radius))
		return false;
	if (radius <= 0.0f)
		return false;

	return true;
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
