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


#ifndef PX_PHYSICS_EXTENSIONS_SHAPE_H
#define PX_PHYSICS_EXTENSIONS_SHAPE_H
/** \addtogroup extensions
  @{
*/

#include "PxPhysXConfig.h"

#include "PxShape.h"
#include "PxRigidActor.h"
#include "geometry/PxGeometryQuery.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief utility functions for use with PxShape

@see PxShape
*/

class PxShapeExt
{
public:
	/**
	\brief Retrieves the world space pose of the shape.

	\param[in] shape The shape for which to get the global pose.
	\param[in] actor The actor to which the shape is attached

	\return Global pose of shape.
	*/
	static PX_INLINE	PxTransform		getGlobalPose(const PxShape& shape, const PxRigidActor& actor)
	{
		return actor.getGlobalPose() * shape.getLocalPose();
	}

	/**
	\brief Raycast test against the shape.

	\param[in] shape the shape
	\param[in] actor the actor to which the shape is attached
	\param[in] rayOrigin The origin of the ray to test the geometry object against
	\param[in] rayDir The direction of the ray to test the geometry object against
	\param[in] maxDist Maximum ray length
	\param[in] hitFlags Specify which properties per hit should be computed and written to result hit array. Combination of #PxHitFlag flags
	\param[in] maxHits max number of returned hits = size of 'rayHits' buffer
	\param[out] rayHits Raycast hits information
	\return Number of hits between the ray and the shape

	@see PxRaycastHit PxTransform
	*/
	static PX_INLINE PxU32				raycast(const PxShape& shape, const PxRigidActor& actor, 
												const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist, PxHitFlags hitFlags,
												PxU32 maxHits, PxRaycastHit* rayHits)
	{
		return PxGeometryQuery::raycast(
			rayOrigin, rayDir, shape.getGeometry().any(), getGlobalPose(shape, actor), maxDist, hitFlags, maxHits, rayHits);
	}

	/**
	\brief Test overlap between the shape and a geometry object

	\param[in] shape the shape
	\param[in] actor the actor to which the shape is attached
	\param[in] otherGeom The other geometry object to test overlap with
	\param[in] otherGeomPose Pose of the other geometry object
	\return True if the shape overlaps the geometry object

	@see PxGeometry PxTransform
	*/
	static PX_INLINE bool				overlap(const PxShape& shape, const PxRigidActor& actor, 
												const PxGeometry& otherGeom, const PxTransform& otherGeomPose)
	{
		return PxGeometryQuery::overlap(shape.getGeometry().any(), getGlobalPose(shape, actor), otherGeom, otherGeomPose);
	}

	/**
	\brief Sweep a geometry object against the shape.

	Currently only box, sphere, capsule and convex mesh shapes are supported, i.e. the swept geometry object must be one of those types.

	\param[in] shape the shape
	\param[in] actor the actor to which the shape is attached
	\param[in] unitDir Normalized direction along which the geometry object should be swept.
	\param[in] distance Sweep distance. Needs to be larger than 0.
	\param[in] otherGeom The geometry object to sweep against the shape
	\param[in] otherGeomPose Pose of the geometry object
	\param[out] sweepHit The sweep hit information. Only valid if this method returns true.
	\param[in] hitFlags Specify which properties per hit should be computed and written to result hit array. Combination of #PxHitFlag flags
	\return True if the swept geometry object hits the shape

	@see PxGeometry PxTransform PxSweepHit
	*/
	static PX_INLINE bool			sweep(const PxShape& shape, const PxRigidActor& actor, 
										  const PxVec3& unitDir, const PxReal distance, const PxGeometry& otherGeom, const PxTransform& otherGeomPose,
										  PxSweepHit& sweepHit, PxHitFlags hitFlags)
	{
		return PxGeometryQuery::sweep(unitDir, distance, otherGeom, otherGeomPose, shape.getGeometry().any(), getGlobalPose(shape, actor), sweepHit, hitFlags);
	}


	/**
	\brief Retrieves the axis aligned bounding box enclosing the shape.

	\return The shape's bounding box.

	\param[in] shape the shape
	\param[in] actor the actor to which the shape is attached
	\param[in] inflation  Scale factor for computed world bounds. Box extents are multiplied by this value.

	@see PxBounds3
	*/
	static PX_INLINE PxBounds3		getWorldBounds(const PxShape& shape, const PxRigidActor& actor, float inflation=1.01f)
	{
		return PxGeometryQuery::getWorldBounds(shape.getGeometry().any(), getGlobalPose(shape, actor), inflation);
	}

};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
