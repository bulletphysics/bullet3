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

#ifndef GU_BOX_H
#define GU_BOX_H

/** \addtogroup geomutils
@{
*/

#include "PxPhysXCommonConfig.h"
#include "CmPhysXCommon.h"
#include "CmScaling.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMat33.h"

namespace physx
{
namespace Gu
{
	class Capsule;

	PX_PHYSX_COMMON_API void computeOBBPoints(PxVec3* PX_RESTRICT pts, const PxVec3& center, const PxVec3& extents, const PxVec3& base0, const PxVec3& base1, const PxVec3& base2);


	/**
	\brief Represents an oriented bounding box. 

	As a center point, extents(radii) and a rotation. i.e. the center of the box is at the center point, 
	the box is rotated around this point with the rotation and it is 2*extents in width, height and depth.
	*/

	/**
	Box geometry

	The rot member describes the world space orientation of the box.
	The center member gives the world space position of the box.
	The extents give the local space coordinates of the box corner in the positive octant.
	Dimensions of the box are: 2*extent.
	Transformation to world space is: worldPoint = rot * localPoint + center
	Transformation to local space is: localPoint = T(rot) * (worldPoint - center)
	Where T(M) denotes the transpose of M.
	*/
#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
#endif
	class PX_PHYSX_COMMON_API Box
	{
	public:
		/**
		\brief Constructor
		*/
		PX_FORCE_INLINE Box()
		{
		}

		/**
		\brief Constructor

		\param origin Center of the OBB
		\param extent Extents/radii of the obb.
		\param base rotation to apply to the obb.
		*/
		//! Construct from center, extent and rotation
		PX_FORCE_INLINE Box(const PxVec3& origin, const PxVec3& extent, const PxMat33& base) : rot(base), center(origin), extents(extent)
		{}

		//! Copy constructor
		PX_FORCE_INLINE Box(const Box& other) : rot(other.rot), center(other.center), extents(other.extents)
		{}

		/**
		\brief Destructor
		*/
		PX_FORCE_INLINE ~Box()
		{
		}

		//! Assignment operator
		PX_FORCE_INLINE const Box& operator=(const Box& other)
		{
			rot		= other.rot;
			center	= other.center;
			extents	= other.extents;
			return *this;
		}

		/**
		\brief Setups an empty box.
		*/
		PX_INLINE void setEmpty()
		{
			center = PxVec3(0);
			extents = PxVec3(-PX_MAX_REAL, -PX_MAX_REAL, -PX_MAX_REAL);
			rot = PxMat33(PxIdentity);
		}

		/**
		\brief Checks the box is valid.

		\return	true if the box is valid
		*/
		PX_INLINE bool isValid() const
		{
			// Consistency condition for (Center, Extents) boxes: Extents >= 0.0f
			if(extents.x < 0.0f)	return false;
			if(extents.y < 0.0f)	return false;
			if(extents.z < 0.0f)	return false;
			return true;
		}

/////////////
		PX_FORCE_INLINE	void	setAxes(const PxVec3& axis0, const PxVec3& axis1, const PxVec3& axis2)
		{
			rot.column0 = axis0;
			rot.column1 = axis1;
			rot.column2 = axis2;
		}

		PX_FORCE_INLINE	PxVec3	rotate(const PxVec3& src)	const
		{
			return rot * src;
		}

		PX_FORCE_INLINE	PxVec3	rotateInv(const PxVec3& src)	const
		{
			return rot.transformTranspose(src);
		}

		PX_FORCE_INLINE	PxVec3	transform(const PxVec3& src)	const
		{
			return rot * src + center;
		}

		PX_FORCE_INLINE	PxTransform getTransform()	const
		{
			return PxTransform(center, PxQuat(rot));
		}

		PX_INLINE PxVec3 computeAABBExtent() const
		{
			const PxReal a00 = PxAbs(rot[0][0]);
			const PxReal a01 = PxAbs(rot[0][1]);
			const PxReal a02 = PxAbs(rot[0][2]);

			const PxReal a10 = PxAbs(rot[1][0]);
			const PxReal a11 = PxAbs(rot[1][1]);
			const PxReal a12 = PxAbs(rot[1][2]);

			const PxReal a20 = PxAbs(rot[2][0]);
			const PxReal a21 = PxAbs(rot[2][1]);
			const PxReal a22 = PxAbs(rot[2][2]);

			const PxReal ex = extents.x;
			const PxReal ey = extents.y;
			const PxReal ez = extents.z;

			return PxVec3(	a00 * ex + a10 * ey + a20 * ez,
							a01 * ex + a11 * ey + a21 * ez,
							a02 * ex + a12 * ey + a22 * ez);
		}

		/**
		Computes the obb points.
		\param		pts	[out] 8 box points
		*/
		PX_FORCE_INLINE void computeBoxPoints(PxVec3* PX_RESTRICT pts) const
		{
			Gu::computeOBBPoints(pts, center, extents, rot.column0, rot.column1, rot.column2);
		}

		void create(const Gu::Capsule& capsule);

		PxMat33	rot;
		PxVec3	center;
		PxVec3	extents;
	};
	PX_COMPILE_TIME_ASSERT(sizeof(Gu::Box) == 60);

	//! A padded version of Gu::Box, to safely load its data using SIMD
	class BoxPadded : public Box
	{
	public:
		PX_FORCE_INLINE BoxPadded()		{}
		PX_FORCE_INLINE ~BoxPadded()	{}
		PxU32	padding;
	};
	PX_COMPILE_TIME_ASSERT(sizeof(Gu::BoxPadded) == 64);

	//! Transforms a shape space AABB to a vertex space AABB (conservative).
	PX_FORCE_INLINE void computeVertexSpaceAABB(Gu::Box& vertexSpaceOBB, const PxBounds3& worldBounds, const PxTransform& world2Shape, const Cm::FastVertex2ShapeScaling& scaling, bool idtScaleMesh)
	{
		PX_ASSERT(!worldBounds.isEmpty());
		const PxBounds3 boundsInMesh = PxBounds3::transformFast(world2Shape, worldBounds);	// transform bounds from world to shape (excluding mesh scale)

		vertexSpaceOBB.rot = PxMat33(PxIdentity);
		if(idtScaleMesh)
		{
			vertexSpaceOBB.center = boundsInMesh.getCenter();
			vertexSpaceOBB.extents = boundsInMesh.getExtents();
		}
		else
		{
			const PxBounds3 bounds = PxBounds3::basisExtent(scaling.getShape2VertexSkew() * boundsInMesh.getCenter(), scaling.getShape2VertexSkew(), boundsInMesh.getExtents());
			vertexSpaceOBB.center = bounds.getCenter();
			vertexSpaceOBB.extents = bounds.getExtents();
		}
	}

#if PX_VC 
     #pragma warning(pop) 
#endif

}

}

/** @} */
#endif
