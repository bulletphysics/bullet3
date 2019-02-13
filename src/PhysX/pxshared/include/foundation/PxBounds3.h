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

#ifndef PXFOUNDATION_PXBOUNDS3_H
#define PXFOUNDATION_PXBOUNDS3_H

/** \addtogroup foundation
@{
*/

#include "foundation/PxTransform.h"
#include "foundation/PxMat33.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

// maximum extents defined such that floating point exceptions are avoided for standard use cases
#define PX_MAX_BOUNDS_EXTENTS (PX_MAX_REAL * 0.25f)

/**
\brief Class representing 3D range or axis aligned bounding box.

Stored as minimum and maximum extent corners. Alternate representation
would be center and dimensions.
May be empty or nonempty. For nonempty bounds, minimum <= maximum has to hold for all axes.
Empty bounds have to be represented as minimum = PX_MAX_BOUNDS_EXTENTS and maximum = -PX_MAX_BOUNDS_EXTENTS for all
axes.
All other representations are invalid and the behavior is undefined.
*/
class PxBounds3
{
  public:
	/**
	\brief Default constructor, not performing any initialization for performance reason.
	\remark Use empty() function below to construct empty bounds.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxBounds3()
	{
	}

	/**
	\brief Construct from two bounding points
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxBounds3(const PxVec3& minimum, const PxVec3& maximum);

	/**
	\brief Return empty bounds.
	*/
	static PX_CUDA_CALLABLE PX_FORCE_INLINE PxBounds3 empty();

	/**
	\brief returns the AABB containing v0 and v1.
	\param v0 first point included in the AABB.
	\param v1 second point included in the AABB.
	*/
	static PX_CUDA_CALLABLE PX_FORCE_INLINE PxBounds3 boundsOfPoints(const PxVec3& v0, const PxVec3& v1);

	/**
	\brief returns the AABB from center and extents vectors.
	\param center Center vector
	\param extent Extents vector
	*/
	static PX_CUDA_CALLABLE PX_FORCE_INLINE PxBounds3 centerExtents(const PxVec3& center, const PxVec3& extent);

	/**
	\brief Construct from center, extent, and (not necessarily orthogonal) basis
	*/
	static PX_CUDA_CALLABLE PX_INLINE PxBounds3
	basisExtent(const PxVec3& center, const PxMat33& basis, const PxVec3& extent);

	/**
	\brief Construct from pose and extent
	*/
	static PX_CUDA_CALLABLE PX_INLINE PxBounds3 poseExtent(const PxTransform& pose, const PxVec3& extent);

	/**
	\brief gets the transformed bounds of the passed AABB (resulting in a bigger AABB).

	This version is safe to call for empty bounds.

	\param[in] matrix Transform to apply, can contain scaling as well
	\param[in] bounds The bounds to transform.
	*/
	static PX_CUDA_CALLABLE PX_INLINE PxBounds3 transformSafe(const PxMat33& matrix, const PxBounds3& bounds);

	/**
	\brief gets the transformed bounds of the passed AABB (resulting in a bigger AABB).

	Calling this method for empty bounds leads to undefined behavior. Use #transformSafe() instead.

	\param[in] matrix Transform to apply, can contain scaling as well
	\param[in] bounds The bounds to transform.
	*/
	static PX_CUDA_CALLABLE PX_INLINE PxBounds3 transformFast(const PxMat33& matrix, const PxBounds3& bounds);

	/**
	\brief gets the transformed bounds of the passed AABB (resulting in a bigger AABB).

	This version is safe to call for empty bounds.

	\param[in] transform Transform to apply, can contain scaling as well
	\param[in] bounds The bounds to transform.
	*/
	static PX_CUDA_CALLABLE PX_INLINE PxBounds3 transformSafe(const PxTransform& transform, const PxBounds3& bounds);

	/**
	\brief gets the transformed bounds of the passed AABB (resulting in a bigger AABB).

	Calling this method for empty bounds leads to undefined behavior. Use #transformSafe() instead.

	\param[in] transform Transform to apply, can contain scaling as well
	\param[in] bounds The bounds to transform.
	*/
	static PX_CUDA_CALLABLE PX_INLINE PxBounds3 transformFast(const PxTransform& transform, const PxBounds3& bounds);

	/**
	\brief Sets empty to true
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE void setEmpty();

	/**
	\brief Sets the bounds to maximum size [-PX_MAX_BOUNDS_EXTENTS, PX_MAX_BOUNDS_EXTENTS].
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE void setMaximal();

	/**
	\brief expands the volume to include v
	\param v Point to expand to.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE void include(const PxVec3& v);

	/**
	\brief expands the volume to include b.
	\param b Bounds to perform union with.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE void include(const PxBounds3& b);

	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isEmpty() const;

	/**
	\brief indicates whether the intersection of this and b is empty or not.
	\param b Bounds to test for intersection.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool intersects(const PxBounds3& b) const;

	/**
	 \brief computes the 1D-intersection between two AABBs, on a given axis.
	 \param	a		the other AABB
	 \param	axis	the axis (0, 1, 2)
	 */
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool intersects1D(const PxBounds3& a, uint32_t axis) const;

	/**
	\brief indicates if these bounds contain v.
	\param v Point to test against bounds.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool contains(const PxVec3& v) const;

	/**
	 \brief	checks a box is inside another box.
	 \param	box		the other AABB
	 */
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isInside(const PxBounds3& box) const;

	/**
	\brief returns the center of this axis aligned box.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 getCenter() const;

	/**
	\brief get component of the box's center along a given axis
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE float getCenter(uint32_t axis) const;

	/**
	\brief get component of the box's extents along a given axis
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE float getExtents(uint32_t axis) const;

	/**
	\brief returns the dimensions (width/height/depth) of this axis aligned box.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 getDimensions() const;

	/**
	\brief returns the extents, which are half of the width/height/depth.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 getExtents() const;

	/**
	\brief scales the AABB.

	This version is safe to call for empty bounds.

	\param scale Factor to scale AABB by.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE void scaleSafe(float scale);

	/**
	\brief scales the AABB.

	Calling this method for empty bounds leads to undefined behavior. Use #scaleSafe() instead.

	\param scale Factor to scale AABB by.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE void scaleFast(float scale);

	/**
	fattens the AABB in all 3 dimensions by the given distance.

	This version is safe to call for empty bounds.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE void fattenSafe(float distance);

	/**
	fattens the AABB in all 3 dimensions by the given distance.

	Calling this method for empty bounds leads to undefined behavior. Use #fattenSafe() instead.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE void fattenFast(float distance);

	/**
	checks that the AABB values are not NaN
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isFinite() const;

	/**
	checks that the AABB values describe a valid configuration.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isValid() const;

	PxVec3 minimum, maximum;
};

PX_CUDA_CALLABLE PX_FORCE_INLINE PxBounds3::PxBounds3(const PxVec3& minimum_, const PxVec3& maximum_)
: minimum(minimum_), maximum(maximum_)
{
}

PX_CUDA_CALLABLE PX_FORCE_INLINE PxBounds3 PxBounds3::empty()
{
	return PxBounds3(PxVec3(PX_MAX_BOUNDS_EXTENTS), PxVec3(-PX_MAX_BOUNDS_EXTENTS));
}

PX_CUDA_CALLABLE PX_FORCE_INLINE bool PxBounds3::isFinite() const
{
	return minimum.isFinite() && maximum.isFinite();
}

PX_CUDA_CALLABLE PX_FORCE_INLINE PxBounds3 PxBounds3::boundsOfPoints(const PxVec3& v0, const PxVec3& v1)
{
	return PxBounds3(v0.minimum(v1), v0.maximum(v1));
}

PX_CUDA_CALLABLE PX_FORCE_INLINE PxBounds3 PxBounds3::centerExtents(const PxVec3& center, const PxVec3& extent)
{
	return PxBounds3(center - extent, center + extent);
}

PX_CUDA_CALLABLE PX_INLINE PxBounds3
PxBounds3::basisExtent(const PxVec3& center, const PxMat33& basis, const PxVec3& extent)
{
	// extended basis vectors
	PxVec3 c0 = basis.column0 * extent.x;
	PxVec3 c1 = basis.column1 * extent.y;
	PxVec3 c2 = basis.column2 * extent.z;

	PxVec3 w;
	// find combination of base vectors that produces max. distance for each component = sum of abs()
	w.x = PxAbs(c0.x) + PxAbs(c1.x) + PxAbs(c2.x);
	w.y = PxAbs(c0.y) + PxAbs(c1.y) + PxAbs(c2.y);
	w.z = PxAbs(c0.z) + PxAbs(c1.z) + PxAbs(c2.z);

	return PxBounds3(center - w, center + w);
}

PX_CUDA_CALLABLE PX_INLINE PxBounds3 PxBounds3::poseExtent(const PxTransform& pose, const PxVec3& extent)
{
	return basisExtent(pose.p, PxMat33(pose.q), extent);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE void PxBounds3::setEmpty()
{
	minimum = PxVec3(PX_MAX_BOUNDS_EXTENTS);
	maximum = PxVec3(-PX_MAX_BOUNDS_EXTENTS);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE void PxBounds3::setMaximal()
{
	minimum = PxVec3(-PX_MAX_BOUNDS_EXTENTS);
	maximum = PxVec3(PX_MAX_BOUNDS_EXTENTS);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE void PxBounds3::include(const PxVec3& v)
{
	PX_ASSERT(isValid());
	minimum = minimum.minimum(v);
	maximum = maximum.maximum(v);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE void PxBounds3::include(const PxBounds3& b)
{
	PX_ASSERT(isValid());
	minimum = minimum.minimum(b.minimum);
	maximum = maximum.maximum(b.maximum);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE bool PxBounds3::isEmpty() const
{
	PX_ASSERT(isValid());
	return minimum.x > maximum.x;
}

PX_CUDA_CALLABLE PX_FORCE_INLINE bool PxBounds3::intersects(const PxBounds3& b) const
{
	PX_ASSERT(isValid() && b.isValid());
	return !(b.minimum.x > maximum.x || minimum.x > b.maximum.x || b.minimum.y > maximum.y || minimum.y > b.maximum.y ||
	         b.minimum.z > maximum.z || minimum.z > b.maximum.z);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE bool PxBounds3::intersects1D(const PxBounds3& a, uint32_t axis) const
{
	PX_ASSERT(isValid() && a.isValid());
	return maximum[axis] >= a.minimum[axis] && a.maximum[axis] >= minimum[axis];
}

PX_CUDA_CALLABLE PX_FORCE_INLINE bool PxBounds3::contains(const PxVec3& v) const
{
	PX_ASSERT(isValid());

	return !(v.x < minimum.x || v.x > maximum.x || v.y < minimum.y || v.y > maximum.y || v.z < minimum.z ||
	         v.z > maximum.z);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE bool PxBounds3::isInside(const PxBounds3& box) const
{
	PX_ASSERT(isValid() && box.isValid());
	if(box.minimum.x > minimum.x)
		return false;
	if(box.minimum.y > minimum.y)
		return false;
	if(box.minimum.z > minimum.z)
		return false;
	if(box.maximum.x < maximum.x)
		return false;
	if(box.maximum.y < maximum.y)
		return false;
	if(box.maximum.z < maximum.z)
		return false;
	return true;
}

PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 PxBounds3::getCenter() const
{
	PX_ASSERT(isValid());
	return (minimum + maximum) * 0.5f;
}

PX_CUDA_CALLABLE PX_FORCE_INLINE float PxBounds3::getCenter(uint32_t axis) const
{
	PX_ASSERT(isValid());
	return (minimum[axis] + maximum[axis]) * 0.5f;
}

PX_CUDA_CALLABLE PX_FORCE_INLINE float PxBounds3::getExtents(uint32_t axis) const
{
	PX_ASSERT(isValid());
	return (maximum[axis] - minimum[axis]) * 0.5f;
}

PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 PxBounds3::getDimensions() const
{
	PX_ASSERT(isValid());
	return maximum - minimum;
}

PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 PxBounds3::getExtents() const
{
	PX_ASSERT(isValid());
	return getDimensions() * 0.5f;
}

PX_CUDA_CALLABLE PX_FORCE_INLINE void PxBounds3::scaleSafe(float scale)
{
	PX_ASSERT(isValid());
	if(!isEmpty())
		scaleFast(scale);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE void PxBounds3::scaleFast(float scale)
{
	PX_ASSERT(isValid());
	*this = centerExtents(getCenter(), getExtents() * scale);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE void PxBounds3::fattenSafe(float distance)
{
	PX_ASSERT(isValid());
	if(!isEmpty())
		fattenFast(distance);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE void PxBounds3::fattenFast(float distance)
{
	PX_ASSERT(isValid());
	minimum.x -= distance;
	minimum.y -= distance;
	minimum.z -= distance;

	maximum.x += distance;
	maximum.y += distance;
	maximum.z += distance;
}

PX_CUDA_CALLABLE PX_INLINE PxBounds3 PxBounds3::transformSafe(const PxMat33& matrix, const PxBounds3& bounds)
{
	PX_ASSERT(bounds.isValid());
	return !bounds.isEmpty() ? transformFast(matrix, bounds) : bounds;
}

PX_CUDA_CALLABLE PX_INLINE PxBounds3 PxBounds3::transformFast(const PxMat33& matrix, const PxBounds3& bounds)
{
	PX_ASSERT(bounds.isValid());
	return PxBounds3::basisExtent(matrix * bounds.getCenter(), matrix, bounds.getExtents());
}

PX_CUDA_CALLABLE PX_INLINE PxBounds3 PxBounds3::transformSafe(const PxTransform& transform, const PxBounds3& bounds)
{
	PX_ASSERT(bounds.isValid());
	return !bounds.isEmpty() ? transformFast(transform, bounds) : bounds;
}

PX_CUDA_CALLABLE PX_INLINE PxBounds3 PxBounds3::transformFast(const PxTransform& transform, const PxBounds3& bounds)
{
	PX_ASSERT(bounds.isValid());
	return PxBounds3::basisExtent(transform.transform(bounds.getCenter()), PxMat33(transform.q), bounds.getExtents());
}

PX_CUDA_CALLABLE PX_FORCE_INLINE bool PxBounds3::isValid() const
{
	return (isFinite() && (((minimum.x <= maximum.x) && (minimum.y <= maximum.y) && (minimum.z <= maximum.z)) ||
	                       ((minimum.x == PX_MAX_BOUNDS_EXTENTS) && (minimum.y == PX_MAX_BOUNDS_EXTENTS) &&
	                        (minimum.z == PX_MAX_BOUNDS_EXTENTS) && (maximum.x == -PX_MAX_BOUNDS_EXTENTS) &&
	                        (maximum.y == -PX_MAX_BOUNDS_EXTENTS) && (maximum.z == -PX_MAX_BOUNDS_EXTENTS))));
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // #ifndef PXFOUNDATION_PXBOUNDS3_H
