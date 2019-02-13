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


#ifndef PX_PHYSICS_NX_BOX_GEOMETRY
#define PX_PHYSICS_NX_BOX_GEOMETRY
/** \addtogroup geomutils
@{
*/
#include "geometry/PxGeometry.h"
#include "foundation/PxVec3.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Class representing the geometry of a box.  

The geometry of a box can be fully specified by its half extents.  This is the half of its width, height, and depth.
\note The scaling of the box is expected to be baked into these values, there is no additional scaling parameter.
*/
class PxBoxGeometry : public PxGeometry 
{
public:
	/**
	\brief Default constructor, initializes to a box with zero dimensions.
	*/
	PX_INLINE PxBoxGeometry() :									PxGeometry(PxGeometryType::eBOX), halfExtents(0,0,0)		{}
	
	/**
	\brief Constructor to initialize half extents from scalar parameters.
	\param hx Initial half extents' x component.
	\param hy Initial half extents' y component.
	\param hz Initial half extents' z component.
	*/
	PX_INLINE PxBoxGeometry(PxReal hx, PxReal hy, PxReal hz) :	PxGeometry(PxGeometryType::eBOX), halfExtents(hx, hy, hz)	{}

	/**
	\brief Constructor to initialize half extents from vector parameter.
	\param halfExtents_ Initial half extents.
	*/
	PX_INLINE PxBoxGeometry(PxVec3 halfExtents_) :				PxGeometry(PxGeometryType::eBOX), halfExtents(halfExtents_)	{}

	/**
	\brief Returns true if the geometry is valid.

	\return True if the current settings are valid

	\note A valid box has a positive extent in each direction (halfExtents.x > 0, halfExtents.y > 0, halfExtents.z > 0). 
	It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a box that has zero extent in any direction.

	@see PxRigidActor::createShape, PxPhysics::createShape
	*/
	PX_INLINE bool isValid() const;

public:
	/**
	\brief Half of the width, height, and depth of the box.
	*/
	PxVec3 halfExtents;
};


PX_INLINE bool PxBoxGeometry::isValid() const
{
	if (mType != PxGeometryType::eBOX)
		return false;
	if (!halfExtents.isFinite())
		return false;
	if (halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f)
		return false;

	return true;
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
