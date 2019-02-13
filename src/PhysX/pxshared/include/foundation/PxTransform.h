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

#ifndef PXFOUNDATION_PXTRANSFORM_H
#define PXFOUNDATION_PXTRANSFORM_H
/** \addtogroup foundation
  @{
*/

#include "foundation/PxQuat.h"
#include "foundation/PxPlane.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/*!
\brief class representing a rigid euclidean transform as a quaternion and a vector
*/

class PxTransform
{
  public:
	PxQuat q;
	PxVec3 p;

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform()
	{
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE explicit PxTransform(const PxVec3& position) : q(PxIdentity), p(position)
	{
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE explicit PxTransform(PxIDENTITY r) : q(PxIdentity), p(PxZero)
	{
		PX_UNUSED(r);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE explicit PxTransform(const PxQuat& orientation) : q(orientation), p(0)
	{
		PX_ASSERT(orientation.isSane());
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform(float x, float y, float z, PxQuat aQ = PxQuat(PxIdentity))
	: q(aQ), p(x, y, z)
	{
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform(const PxVec3& p0, const PxQuat& q0) : q(q0), p(p0)
	{
		PX_ASSERT(q0.isSane());
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE explicit PxTransform(const PxMat44& m); // defined in PxMat44.h

	/**
	\brief returns true if the two transforms are exactly equal
	*/
	PX_CUDA_CALLABLE PX_INLINE bool operator==(const PxTransform& t) const
	{
		return p == t.p && q == t.q;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform operator*(const PxTransform& x) const
	{
		PX_ASSERT(x.isSane());
		return transform(x);
	}

	//! Equals matrix multiplication
	PX_CUDA_CALLABLE PX_INLINE PxTransform& operator*=(PxTransform& other)
	{
		*this = *this * other;
		return *this;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform getInverse() const
	{
		PX_ASSERT(isFinite());
		return PxTransform(q.rotateInv(-p), q.getConjugate());
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 transform(const PxVec3& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotate(input) + p;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 transformInv(const PxVec3& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotateInv(input - p);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 rotate(const PxVec3& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotate(input);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 rotateInv(const PxVec3& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotateInv(input);
	}

	//! Transform transform to parent (returns compound transform: first src, then *this)
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform transform(const PxTransform& src) const
	{
		PX_ASSERT(src.isSane());
		PX_ASSERT(isSane());
		// src = [srct, srcr] -> [r*srct + t, r*srcr]
		return PxTransform(q.rotate(src.p) + p, q * src.q);
	}

	/**
	\brief returns true if finite and q is a unit quaternion
	*/

	PX_CUDA_CALLABLE bool isValid() const
	{
		return p.isFinite() && q.isFinite() && q.isUnit();
	}

	/**
	\brief returns true if finite and quat magnitude is reasonably close to unit to allow for some accumulation of error
	vs isValid
	*/

	PX_CUDA_CALLABLE bool isSane() const
	{
		return isFinite() && q.isSane();
	}

	/**
	\brief returns true if all elems are finite (not NAN or INF, etc.)
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isFinite() const
	{
		return p.isFinite() && q.isFinite();
	}

	//! Transform transform from parent (returns compound transform: first src, then this->inverse)
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform transformInv(const PxTransform& src) const
	{
		PX_ASSERT(src.isSane());
		PX_ASSERT(isFinite());
		// src = [srct, srcr] -> [r^-1*(srct-t), r^-1*srcr]
		PxQuat qinv = q.getConjugate();
		return PxTransform(qinv.rotate(src.p - p), qinv * src.q);
	}

	/**
	\brief transform plane
	*/

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxPlane transform(const PxPlane& plane) const
	{
		PxVec3 transformedNormal = rotate(plane.n);
		return PxPlane(transformedNormal, plane.d - p.dot(transformedNormal));
	}

	/**
	\brief inverse-transform plane
	*/

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxPlane inverseTransform(const PxPlane& plane) const
	{
		PxVec3 transformedNormal = rotateInv(plane.n);
		return PxPlane(transformedNormal, plane.d + p.dot(plane.n));
	}

	/**
	\brief return a normalized transform (i.e. one in which the quaternion has unit magnitude)
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform getNormalized() const
	{
		return PxTransform(p, q.getNormalized());
	}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // #ifndef PXFOUNDATION_PXTRANSFORM_H
