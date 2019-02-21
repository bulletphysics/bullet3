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

#ifndef PXFOUNDATION_PXMAT44_H
#define PXFOUNDATION_PXMAT44_H
/** \addtogroup foundation
@{
*/

#include "foundation/PxQuat.h"
#include "foundation/PxVec4.h"
#include "foundation/PxMat33.h"
#include "foundation/PxTransform.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/*!
\brief 4x4 matrix class

This class is layout-compatible with D3D and OpenGL matrices. More notes on layout are given in the PxMat33

@see PxMat33 PxTransform
*/

class PxMat44
{
  public:
	//! Default constructor
	PX_CUDA_CALLABLE PX_INLINE PxMat44()
	{
	}

	//! identity constructor
	PX_CUDA_CALLABLE PX_INLINE PxMat44(PxIDENTITY r)
	: column0(1.0f, 0.0f, 0.0f, 0.0f)
	, column1(0.0f, 1.0f, 0.0f, 0.0f)
	, column2(0.0f, 0.0f, 1.0f, 0.0f)
	, column3(0.0f, 0.0f, 0.0f, 1.0f)
	{
		PX_UNUSED(r);
	}

	//! zero constructor
	PX_CUDA_CALLABLE PX_INLINE PxMat44(PxZERO r) : column0(PxZero), column1(PxZero), column2(PxZero), column3(PxZero)
	{
		PX_UNUSED(r);
	}

	//! Construct from four 4-vectors
	PX_CUDA_CALLABLE PxMat44(const PxVec4& col0, const PxVec4& col1, const PxVec4& col2, const PxVec4& col3)
	: column0(col0), column1(col1), column2(col2), column3(col3)
	{
	}

	//! constructor that generates a multiple of the identity matrix
	explicit PX_CUDA_CALLABLE PX_INLINE PxMat44(float r)
	: column0(r, 0.0f, 0.0f, 0.0f)
	, column1(0.0f, r, 0.0f, 0.0f)
	, column2(0.0f, 0.0f, r, 0.0f)
	, column3(0.0f, 0.0f, 0.0f, r)
	{
	}

	//! Construct from three base vectors and a translation
	PX_CUDA_CALLABLE PxMat44(const PxVec3& col0, const PxVec3& col1, const PxVec3& col2, const PxVec3& col3)
	: column0(col0, 0), column1(col1, 0), column2(col2, 0), column3(col3, 1.0f)
	{
	}

	//! Construct from float[16]
	explicit PX_CUDA_CALLABLE PX_INLINE PxMat44(float values[])
	: column0(values[0], values[1], values[2], values[3])
	, column1(values[4], values[5], values[6], values[7])
	, column2(values[8], values[9], values[10], values[11])
	, column3(values[12], values[13], values[14], values[15])
	{
	}

	//! Construct from a quaternion
	explicit PX_CUDA_CALLABLE PX_INLINE PxMat44(const PxQuat& q)
	{
		const float x = q.x;
		const float y = q.y;
		const float z = q.z;
		const float w = q.w;

		const float x2 = x + x;
		const float y2 = y + y;
		const float z2 = z + z;

		const float xx = x2 * x;
		const float yy = y2 * y;
		const float zz = z2 * z;

		const float xy = x2 * y;
		const float xz = x2 * z;
		const float xw = x2 * w;

		const float yz = y2 * z;
		const float yw = y2 * w;
		const float zw = z2 * w;

		column0 = PxVec4(1.0f - yy - zz, xy + zw, xz - yw, 0.0f);
		column1 = PxVec4(xy - zw, 1.0f - xx - zz, yz + xw, 0.0f);
		column2 = PxVec4(xz + yw, yz - xw, 1.0f - xx - yy, 0.0f);
		column3 = PxVec4(0.0f, 0.0f, 0.0f, 1.0f);
	}

	//! Construct from a diagonal vector
	explicit PX_CUDA_CALLABLE PX_INLINE PxMat44(const PxVec4& diagonal)
	: column0(diagonal.x, 0.0f, 0.0f, 0.0f)
	, column1(0.0f, diagonal.y, 0.0f, 0.0f)
	, column2(0.0f, 0.0f, diagonal.z, 0.0f)
	, column3(0.0f, 0.0f, 0.0f, diagonal.w)
	{
	}

	//! Construct from Mat33 and a translation
	PX_CUDA_CALLABLE PxMat44(const PxMat33& axes, const PxVec3& position)
	: column0(axes.column0, 0.0f), column1(axes.column1, 0.0f), column2(axes.column2, 0.0f), column3(position, 1.0f)
	{
	}

	PX_CUDA_CALLABLE PxMat44(const PxTransform& t)
	{
		*this = PxMat44(PxMat33(t.q), t.p);
	}

	/**
	\brief returns true if the two matrices are exactly equal
	*/
	PX_CUDA_CALLABLE PX_INLINE bool operator==(const PxMat44& m) const
	{
		return column0 == m.column0 && column1 == m.column1 && column2 == m.column2 && column3 == m.column3;
	}

	//! Copy constructor
	PX_CUDA_CALLABLE PX_INLINE PxMat44(const PxMat44& other)
	: column0(other.column0), column1(other.column1), column2(other.column2), column3(other.column3)
	{
	}

	//! Assignment operator
	PX_CUDA_CALLABLE PX_INLINE PxMat44& operator=(const PxMat44& other)
	{
		column0 = other.column0;
		column1 = other.column1;
		column2 = other.column2;
		column3 = other.column3;
		return *this;
	}

	//! Get transposed matrix
	PX_CUDA_CALLABLE PX_INLINE const PxMat44 getTranspose() const
	{
		return PxMat44(
		    PxVec4(column0.x, column1.x, column2.x, column3.x), PxVec4(column0.y, column1.y, column2.y, column3.y),
		    PxVec4(column0.z, column1.z, column2.z, column3.z), PxVec4(column0.w, column1.w, column2.w, column3.w));
	}

	//! Unary minus
	PX_CUDA_CALLABLE PX_INLINE const PxMat44 operator-() const
	{
		return PxMat44(-column0, -column1, -column2, -column3);
	}

	//! Add
	PX_CUDA_CALLABLE PX_INLINE const PxMat44 operator+(const PxMat44& other) const
	{
		return PxMat44(column0 + other.column0, column1 + other.column1, column2 + other.column2,
		               column3 + other.column3);
	}

	//! Subtract
	PX_CUDA_CALLABLE PX_INLINE const PxMat44 operator-(const PxMat44& other) const
	{
		return PxMat44(column0 - other.column0, column1 - other.column1, column2 - other.column2,
		               column3 - other.column3);
	}

	//! Scalar multiplication
	PX_CUDA_CALLABLE PX_INLINE const PxMat44 operator*(float scalar) const
	{
		return PxMat44(column0 * scalar, column1 * scalar, column2 * scalar, column3 * scalar);
	}

	friend PxMat44 operator*(float, const PxMat44&);

	//! Matrix multiplication
	PX_CUDA_CALLABLE PX_INLINE const PxMat44 operator*(const PxMat44& other) const
	{
		// Rows from this <dot> columns from other
		// column0 = transform(other.column0) etc
		return PxMat44(transform(other.column0), transform(other.column1), transform(other.column2),
		               transform(other.column3));
	}

	// a <op>= b operators

	//! Equals-add
	PX_CUDA_CALLABLE PX_INLINE PxMat44& operator+=(const PxMat44& other)
	{
		column0 += other.column0;
		column1 += other.column1;
		column2 += other.column2;
		column3 += other.column3;
		return *this;
	}

	//! Equals-sub
	PX_CUDA_CALLABLE PX_INLINE PxMat44& operator-=(const PxMat44& other)
	{
		column0 -= other.column0;
		column1 -= other.column1;
		column2 -= other.column2;
		column3 -= other.column3;
		return *this;
	}

	//! Equals scalar multiplication
	PX_CUDA_CALLABLE PX_INLINE PxMat44& operator*=(float scalar)
	{
		column0 *= scalar;
		column1 *= scalar;
		column2 *= scalar;
		column3 *= scalar;
		return *this;
	}

	//! Equals matrix multiplication
	PX_CUDA_CALLABLE PX_INLINE PxMat44& operator*=(const PxMat44& other)
	{
		*this = *this * other;
		return *this;
	}

	//! Element access, mathematical way!
	PX_CUDA_CALLABLE PX_FORCE_INLINE float operator()(unsigned int row, unsigned int col) const
	{
		return (*this)[col][row];
	}

	//! Element access, mathematical way!
	PX_CUDA_CALLABLE PX_FORCE_INLINE float& operator()(unsigned int row, unsigned int col)
	{
		return (*this)[col][row];
	}

	//! Transform vector by matrix, equal to v' = M*v
	PX_CUDA_CALLABLE PX_INLINE const PxVec4 transform(const PxVec4& other) const
	{
		return column0 * other.x + column1 * other.y + column2 * other.z + column3 * other.w;
	}

	//! Transform vector by matrix, equal to v' = M*v
	PX_CUDA_CALLABLE PX_INLINE const PxVec3 transform(const PxVec3& other) const
	{
		return transform(PxVec4(other, 1.0f)).getXYZ();
	}

	//! Rotate vector by matrix, equal to v' = M*v
	PX_CUDA_CALLABLE PX_INLINE const PxVec4 rotate(const PxVec4& other) const
	{
		return column0 * other.x + column1 * other.y + column2 * other.z; // + column3*0;
	}

	//! Rotate vector by matrix, equal to v' = M*v
	PX_CUDA_CALLABLE PX_INLINE const PxVec3 rotate(const PxVec3& other) const
	{
		return rotate(PxVec4(other, 1.0f)).getXYZ();
	}

	PX_CUDA_CALLABLE PX_INLINE const PxVec3 getBasis(int num) const
	{
		PX_SHARED_ASSERT(num >= 0 && num < 3);
		return (&column0)[num].getXYZ();
	}

	PX_CUDA_CALLABLE PX_INLINE const PxVec3 getPosition() const
	{
		return column3.getXYZ();
	}

	PX_CUDA_CALLABLE PX_INLINE void setPosition(const PxVec3& position)
	{
		column3.x = position.x;
		column3.y = position.y;
		column3.z = position.z;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE const float* front() const
	{
		return &column0.x;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec4& operator[](unsigned int num)
	{
		return (&column0)[num];
	}
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxVec4& operator[](unsigned int num) const
	{
		return (&column0)[num];
	}

	PX_CUDA_CALLABLE PX_INLINE void scale(const PxVec4& p)
	{
		column0 *= p.x;
		column1 *= p.y;
		column2 *= p.z;
		column3 *= p.w;
	}

	PX_CUDA_CALLABLE PX_INLINE const PxMat44 inverseRT(void) const
	{
		PxVec3 r0(column0.x, column1.x, column2.x), r1(column0.y, column1.y, column2.y),
		    r2(column0.z, column1.z, column2.z);

		return PxMat44(r0, r1, r2, -(r0 * column3.x + r1 * column3.y + r2 * column3.z));
	}

	PX_CUDA_CALLABLE PX_INLINE bool isFinite() const
	{
		return column0.isFinite() && column1.isFinite() && column2.isFinite() && column3.isFinite();
	}

	// Data, see above for format!

	PxVec4 column0, column1, column2, column3; // the four base vectors
};

// implementation from PxTransform.h
PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform::PxTransform(const PxMat44& m)
{
	PxVec3 column0 = PxVec3(m.column0.x, m.column0.y, m.column0.z);
	PxVec3 column1 = PxVec3(m.column1.x, m.column1.y, m.column1.z);
	PxVec3 column2 = PxVec3(m.column2.x, m.column2.y, m.column2.z);

	q = PxQuat(PxMat33(column0, column1, column2));
	p = PxVec3(m.column3.x, m.column3.y, m.column3.z);
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // #ifndef PXFOUNDATION_PXMAT44_H
