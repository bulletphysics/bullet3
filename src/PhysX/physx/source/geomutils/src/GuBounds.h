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

#ifndef GU_BOUNDS_H
#define GU_BOUNDS_H

#include "foundation/PxBounds3.h"
#include "foundation/PxFlags.h"
#include "GuSIMDHelpers.h"
#include <stddef.h>
#include "PxGeometry.h"
#include "GuBox.h"
#include "GuCenterExtents.h"
#include "GuSphere.h"
#include "GuCapsule.h"

namespace physx
{
class PxGeometry;

namespace Gu
{

//For spheres, planes, capsules and boxes just set localSpaceBounds to NULL.
//For convex meshes, triangle meshes, and heightfields set localSpaceBounds to the relevant pointer if it has already been pre-fetched.  
//For convex meshes, triangle meshes, and heightfields set localSpaceBounds to NULL if it has not already been pre-fetched.   computeBounds will synchronously 
//prefetch the local space bounds if localSpaceBounds is NULL.
//'contactOffset' and 'inflation' should not be used at the same time, i.e. either contactOffset==0.0f, or inflation==1.0f
PX_PHYSX_COMMON_API void computeBounds(PxBounds3& bounds, const PxGeometry& geometry, const PxTransform& transform, float contactOffset, const CenterExtentsPadded* PX_RESTRICT localSpaceBounds, float inflation);	//AABB in world space.

//For spheres, planes, capsules and boxes just set localSpaceBounds to NULL.
//For convex meshes, triangle meshes, and heightfields set localSpaceBounds to the relevant pointer if it has not already been pre-fetched.  
//For convex meshes, triangle meshes, and heightfields set localSpaceBounds to NULL if it has not already been pre-fetched.   computeBounds will synchronously 
//prefetch the local space bounds if localSpaceBounds is NULL.
PX_PHYSX_COMMON_API PxF32 computeBoundsWithCCDThreshold(Vec3p& origin, Vec3p& extent, const PxGeometry& geometry, const PxTransform& transform, const CenterExtentsPadded* PX_RESTRICT localSpaceBounds);	//AABB in world space.


PX_FORCE_INLINE PxBounds3 computeBounds(const PxGeometry& geometry, const PxTransform& pose)
{
	PxBounds3 bounds;
	computeBounds(bounds, geometry, pose, 0.0f, NULL, 1.0f);
	return bounds;
}

class ShapeData
{
public:

	PX_PHYSX_COMMON_API						ShapeData(const PxGeometry& g, const PxTransform& t, PxReal inflation);	

	// PT: used by overlaps (box, capsule, convex)
	PX_FORCE_INLINE const PxVec3&			getPrunerBoxGeomExtentsInflated()	const	{ return mPrunerBoxGeomExtents; }

	// PT: used by overlaps (box, capsule, convex)
	PX_FORCE_INLINE const PxVec3&			getPrunerWorldPos()					const	{ return mGuBox.center;			}

	PX_FORCE_INLINE const PxBounds3&		getPrunerInflatedWorldAABB()		const	{ return mPrunerInflatedAABB;	}

	// PT: used by overlaps (box, capsule, convex)
	PX_FORCE_INLINE const PxMat33&			getPrunerWorldRot33()				const	{ return mGuBox.rot;			}

	// PT: this one only used by overlaps so far (for sphere shape, pruner level)
	PX_FORCE_INLINE const Gu::Sphere&		getGuSphere() const
	{
		PX_ASSERT(mType == PxGeometryType::eSPHERE);
		return reinterpret_cast<const Gu::Sphere&>(mGuSphere);
	}

	// PT: this one only used by sweeps so far (for box shape, NP level)
	PX_FORCE_INLINE const Gu::Box&			getGuBox() const
	{
		PX_ASSERT(mType == PxGeometryType::eBOX);
		return mGuBox;
	}

	// PT: this one used by sweeps (NP level) and overlaps (pruner level) - for capsule shape
	PX_FORCE_INLINE const Gu::Capsule&		getGuCapsule() const
	{
		PX_ASSERT(mType == PxGeometryType::eCAPSULE);
		return reinterpret_cast<const Gu::Capsule&>(mGuCapsule);
	}

	PX_FORCE_INLINE float					getCapsuleHalfHeight() const
	{
		PX_ASSERT(mType == PxGeometryType::eCAPSULE);
		return mGuBox.extents.x;
	}

	PX_FORCE_INLINE	PxU32					isOBB()		const { return PxU32(mIsOBB);				}
	PX_FORCE_INLINE	PxGeometryType::Enum	getType()	const { return PxGeometryType::Enum(mType);	}

	PX_NOCOPY(ShapeData)
private:

	// PT: box: pre-inflated box extents
	//     capsule: pre-inflated extents of box-around-capsule
	//     convex: pre-inflated extents of box-around-convex
	//     sphere: not used
	PxVec3				mPrunerBoxGeomExtents;	// used for pruners. This volume encloses but can differ from the original shape

	// PT:
	//
	// box center = unchanged copy of initial shape's position, except for convex (position of box around convex)
	// SIMD code will load it as a V4 (safe because member is not last of Gu structure)
	//
	// box rot = precomputed PxMat33 version of initial shape's rotation, except for convex (rotation of box around convex)
	// SIMD code will load it as V4s (safe because member is not last of Gu structure)
	//
	// box extents = non-inflated initial box extents for box shape, half-height for capsule, otherwise not used
	Gu::Box				mGuBox;

	PxBounds3			mPrunerInflatedAABB;	// precomputed AABB for the pruner shape
	PxU16				mIsOBB;					// true for OBB, false for AABB. Also used as padding for mPrunerInflatedAABB, don't move.
	PxU16				mType;					// shape's type

	// these union Gu shapes are only precomputed for narrow phase (not pruners), can be different from mPrunerVolume
	// so need separate storage
	union
	{
		PxU8 mGuCapsule[sizeof(Gu::Capsule)];	// 28
		PxU8 mGuSphere[sizeof(Gu::Sphere)];		// 16
	};
};

// PT: please make sure it fits in "one" cache line
PX_COMPILE_TIME_ASSERT(sizeof(ShapeData)==128);

}  // namespace Gu

}
#endif
