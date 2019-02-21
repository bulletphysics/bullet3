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

#ifndef PXFOUNDATION_PXVEC4_H
#define PXFOUNDATION_PXVEC4_H
/** \addtogroup foundation
@{
*/
#include "foundation/PxMath.h"
#include "foundation/PxVec3.h"
#include "foundation/PxSharedAssert.h"

/**
\brief 4 Element vector class.

This is a 4-dimensional vector class with public data members.
*/
#if !PX_DOXYGEN
namespace physx
{
#endif

class PxVec4
{
  public:
	/**
	\brief default constructor leaves data uninitialized.
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4()
	{
	}

	/**
	\brief zero constructor.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec4(PxZERO r) : x(0.0f), y(0.0f), z(0.0f), w(0.0f)
	{
		PX_UNUSED(r);
	}

	/**
	\brief Assigns scalar parameter to all elements.

	Useful to initialize to zero or one.

	\param[in] a Value to assign to elements.
	*/
	explicit PX_CUDA_CALLABLE PX_INLINE PxVec4(float a) : x(a), y(a), z(a), w(a)
	{
	}

	/**
	\brief Initializes from 3 scalar parameters.

	\param[in] nx Value to initialize X component.
	\param[in] ny Value to initialize Y component.
	\param[in] nz Value to initialize Z component.
	\param[in] nw Value to initialize W component.
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4(float nx, float ny, float nz, float nw) : x(nx), y(ny), z(nz), w(nw)
	{
	}

	/**
	\brief Initializes from 3 scalar parameters.

	\param[in] v Value to initialize the X, Y, and Z components.
	\param[in] nw Value to initialize W component.
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4(const PxVec3& v, float nw) : x(v.x), y(v.y), z(v.z), w(nw)
	{
	}

	/**
	\brief Initializes from an array of scalar parameters.

	\param[in] v Value to initialize with.
	*/
	explicit PX_CUDA_CALLABLE PX_INLINE PxVec4(const float v[]) : x(v[0]), y(v[1]), z(v[2]), w(v[3])
	{
	}

	/**
	\brief Copy ctor.
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4(const PxVec4& v) : x(v.x), y(v.y), z(v.z), w(v.w)
	{
	}

	// Operators

	/**
	\brief Assignment operator
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4& operator=(const PxVec4& p)
	{
		x = p.x;
		y = p.y;
		z = p.z;
		w = p.w;
		return *this;
	}

	/**
	\brief element access
	*/
	PX_CUDA_CALLABLE PX_INLINE float& operator[](unsigned int index)
	{
		PX_SHARED_ASSERT(index <= 3);

		return reinterpret_cast<float*>(this)[index];
	}

	/**
	\brief element access
	*/
	PX_CUDA_CALLABLE PX_INLINE const float& operator[](unsigned int index) const
	{
		PX_SHARED_ASSERT(index <= 3);

		return reinterpret_cast<const float*>(this)[index];
	}

	/**
	\brief returns true if the two vectors are exactly equal.
	*/
	PX_CUDA_CALLABLE PX_INLINE bool operator==(const PxVec4& v) const
	{
		return x == v.x && y == v.y && z == v.z && w == v.w;
	}

	/**
	\brief returns true if the two vectors are not exactly equal.
	*/
	PX_CUDA_CALLABLE PX_INLINE bool operator!=(const PxVec4& v) const
	{
		return x != v.x || y != v.y || z != v.z || w != v.w;
	}

	/**
	\brief tests for exact zero vector
	*/
	PX_CUDA_CALLABLE PX_INLINE bool isZero() const
	{
		return x == 0 && y == 0 && z == 0 && w == 0;
	}

	/**
	\brief returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
	*/
	PX_CUDA_CALLABLE PX_INLINE bool isFinite() const
	{
		return PxIsFinite(x) && PxIsFinite(y) && PxIsFinite(z) && PxIsFinite(w);
	}

	/**
	\brief is normalized - used by API parameter validation
	*/
	PX_CUDA_CALLABLE PX_INLINE bool isNormalized() const
	{
		const float unitTolerance = 1e-4f;
		return isFinite() && PxAbs(magnitude() - 1) < unitTolerance;
	}

	/**
	\brief returns the squared magnitude

	Avoids calling PxSqrt()!
	*/
	PX_CUDA_CALLABLE PX_INLINE float magnitudeSquared() const
	{
		return x * x + y * y + z * z + w * w;
	}

	/**
	\brief returns the magnitude
	*/
	PX_CUDA_CALLABLE PX_INLINE float magnitude() const
	{
		return PxSqrt(magnitudeSquared());
	}

	/**
	\brief negation
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4 operator-() const
	{
		return PxVec4(-x, -y, -z, -w);
	}

	/**
	\brief vector addition
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4 operator+(const PxVec4& v) const
	{
		return PxVec4(x + v.x, y + v.y, z + v.z, w + v.w);
	}

	/**
	\brief vector difference
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4 operator-(const PxVec4& v) const
	{
		return PxVec4(x - v.x, y - v.y, z - v.z, w - v.w);
	}

	/**
	\brief scalar post-multiplication
	*/

	PX_CUDA_CALLABLE PX_INLINE PxVec4 operator*(float f) const
	{
		return PxVec4(x * f, y * f, z * f, w * f);
	}

	/**
	\brief scalar division
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4 operator/(float f) const
	{
		f = 1.0f / f;
		return PxVec4(x * f, y * f, z * f, w * f);
	}

	/**
	\brief vector addition
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4& operator+=(const PxVec4& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		w += v.w;
		return *this;
	}

	/**
	\brief vector difference
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4& operator-=(const PxVec4& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		w -= v.w;
		return *this;
	}

	/**
	\brief scalar multiplication
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4& operator*=(float f)
	{
		x *= f;
		y *= f;
		z *= f;
		w *= f;
		return *this;
	}
	/**
	\brief scalar division
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4& operator/=(float f)
	{
		f = 1.0f / f;
		x *= f;
		y *= f;
		z *= f;
		w *= f;
		return *this;
	}

	/**
	\brief returns the scalar product of this and other.
	*/
	PX_CUDA_CALLABLE PX_INLINE float dot(const PxVec4& v) const
	{
		return x * v.x + y * v.y + z * v.z + w * v.w;
	}

	/** return a unit vector */

	PX_CUDA_CALLABLE PX_INLINE PxVec4 getNormalized() const
	{
		float m = magnitudeSquared();
		return m > 0.0f ? *this * PxRecipSqrt(m) : PxVec4(0, 0, 0, 0);
	}

	/**
	\brief normalizes the vector in place
	*/
	PX_CUDA_CALLABLE PX_INLINE float normalize()
	{
		float m = magnitude();
		if(m > 0.0f)
			*this /= m;
		return m;
	}

	/**
	\brief a[i] * b[i], for all i.
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4 multiply(const PxVec4& a) const
	{
		return PxVec4(x * a.x, y * a.y, z * a.z, w * a.w);
	}

	/**
	\brief element-wise minimum
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4 minimum(const PxVec4& v) const
	{
		return PxVec4(PxMin(x, v.x), PxMin(y, v.y), PxMin(z, v.z), PxMin(w, v.w));
	}

	/**
	\brief element-wise maximum
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec4 maximum(const PxVec4& v) const
	{
		return PxVec4(PxMax(x, v.x), PxMax(y, v.y), PxMax(z, v.z), PxMax(w, v.w));
	}

	PX_CUDA_CALLABLE PX_INLINE PxVec3 getXYZ() const
	{
		return PxVec3(x, y, z);
	}

	/**
	\brief set vector elements to zero
	*/
	PX_CUDA_CALLABLE PX_INLINE void setZero()
	{
		x = y = z = w = 0.0f;
	}

	float x, y, z, w;
};

PX_CUDA_CALLABLE static PX_INLINE PxVec4 operator*(float f, const PxVec4& v)
{
	return PxVec4(f * v.x, f * v.y, f * v.z, f * v.w);
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // #ifndef PXFOUNDATION_PXVEC4_H
