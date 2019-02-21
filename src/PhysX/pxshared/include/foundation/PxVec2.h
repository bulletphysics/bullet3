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

#ifndef PXFOUNDATION_PXVEC2_H
#define PXFOUNDATION_PXVEC2_H

/** \addtogroup foundation
@{
*/

#include "foundation/PxMath.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief 2 Element vector class.

This is a 2-dimensional vector class with public data members.
*/
class PxVec2
{
  public:
	/**
	\brief default constructor leaves data uninitialized.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2()
	{
	}

	/**
	\brief zero constructor.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2(PxZERO r) : x(0.0f), y(0.0f)
	{
		PX_UNUSED(r);
	}

	/**
	\brief Assigns scalar parameter to all elements.

	Useful to initialize to zero or one.

	\param[in] a Value to assign to elements.
	*/
	explicit PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2(float a) : x(a), y(a)
	{
	}

	/**
	\brief Initializes from 2 scalar parameters.

	\param[in] nx Value to initialize X component.
	\param[in] ny Value to initialize Y component.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2(float nx, float ny) : x(nx), y(ny)
	{
	}

	/**
	\brief Copy ctor.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2(const PxVec2& v) : x(v.x), y(v.y)
	{
	}

	// Operators

	/**
	\brief Assignment operator
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2& operator=(const PxVec2& p)
	{
		x = p.x;
		y = p.y;
		return *this;
	}

	/**
	\brief element access
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE float& operator[](int index)
	{
		PX_SHARED_ASSERT(index >= 0 && index <= 1);

		return reinterpret_cast<float*>(this)[index];
	}

	/**
	\brief element access
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE const float& operator[](int index) const
	{
		PX_SHARED_ASSERT(index >= 0 && index <= 1);

		return reinterpret_cast<const float*>(this)[index];
	}

	/**
	\brief returns true if the two vectors are exactly equal.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool operator==(const PxVec2& v) const
	{
		return x == v.x && y == v.y;
	}

	/**
	\brief returns true if the two vectors are not exactly equal.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool operator!=(const PxVec2& v) const
	{
		return x != v.x || y != v.y;
	}

	/**
	\brief tests for exact zero vector
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isZero() const
	{
		return x == 0.0f && y == 0.0f;
	}

	/**
	\brief returns true if all 2 elems of the vector are finite (not NAN or INF, etc.)
	*/
	PX_CUDA_CALLABLE PX_INLINE bool isFinite() const
	{
		return PxIsFinite(x) && PxIsFinite(y);
	}

	/**
	\brief is normalized - used by API parameter validation
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isNormalized() const
	{
		const float unitTolerance = 1e-4f;
		return isFinite() && PxAbs(magnitude() - 1) < unitTolerance;
	}

	/**
	\brief returns the squared magnitude

	Avoids calling PxSqrt()!
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE float magnitudeSquared() const
	{
		return x * x + y * y;
	}

	/**
	\brief returns the magnitude
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE float magnitude() const
	{
		return PxSqrt(magnitudeSquared());
	}

	/**
	\brief negation
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2 operator-() const
	{
		return PxVec2(-x, -y);
	}

	/**
	\brief vector addition
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2 operator+(const PxVec2& v) const
	{
		return PxVec2(x + v.x, y + v.y);
	}

	/**
	\brief vector difference
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2 operator-(const PxVec2& v) const
	{
		return PxVec2(x - v.x, y - v.y);
	}

	/**
	\brief scalar post-multiplication
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2 operator*(float f) const
	{
		return PxVec2(x * f, y * f);
	}

	/**
	\brief scalar division
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2 operator/(float f) const
	{
		f = 1.0f / f; // PT: inconsistent notation with operator /=
		return PxVec2(x * f, y * f);
	}

	/**
	\brief vector addition
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2& operator+=(const PxVec2& v)
	{
		x += v.x;
		y += v.y;
		return *this;
	}

	/**
	\brief vector difference
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2& operator-=(const PxVec2& v)
	{
		x -= v.x;
		y -= v.y;
		return *this;
	}

	/**
	\brief scalar multiplication
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2& operator*=(float f)
	{
		x *= f;
		y *= f;
		return *this;
	}
	/**
	\brief scalar division
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2& operator/=(float f)
	{
		f = 1.0f / f; // PT: inconsistent notation with operator /
		x *= f;
		y *= f;
		return *this;
	}

	/**
	\brief returns the scalar product of this and other.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE float dot(const PxVec2& v) const
	{
		return x * v.x + y * v.y;
	}

	/** return a unit vector */

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2 getNormalized() const
	{
		const float m = magnitudeSquared();
		return m > 0.0f ? *this * PxRecipSqrt(m) : PxVec2(0, 0);
	}

	/**
	\brief normalizes the vector in place
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE float normalize()
	{
		const float m = magnitude();
		if(m > 0.0f)
			*this /= m;
		return m;
	}

	/**
	\brief a[i] * b[i], for all i.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2 multiply(const PxVec2& a) const
	{
		return PxVec2(x * a.x, y * a.y);
	}

	/**
	\brief element-wise minimum
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2 minimum(const PxVec2& v) const
	{
		return PxVec2(PxMin(x, v.x), PxMin(y, v.y));
	}

	/**
	\brief returns MIN(x, y);
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE float minElement() const
	{
		return PxMin(x, y);
	}

	/**
	\brief element-wise maximum
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec2 maximum(const PxVec2& v) const
	{
		return PxVec2(PxMax(x, v.x), PxMax(y, v.y));
	}

	/**
	\brief returns MAX(x, y);
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE float maxElement() const
	{
		return PxMax(x, y);
	}

	float x, y;
};

PX_CUDA_CALLABLE static PX_FORCE_INLINE PxVec2 operator*(float f, const PxVec2& v)
{
	return PxVec2(f * v.x, f * v.y);
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // #ifndef PXFOUNDATION_PXVEC2_H
