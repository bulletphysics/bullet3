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

#ifndef PXFOUNDATION_PXMAT33_H
#define PXFOUNDATION_PXMAT33_H
/** \addtogroup foundation
@{
*/

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
/*!
\brief 3x3 matrix class

Some clarifications, as there have been much confusion about matrix formats etc in the past.

Short:
- Matrix have base vectors in columns (vectors are column matrices, 3x1 matrices).
- Matrix is physically stored in column major format
- Matrices are concaternated from left

Long:
Given three base vectors a, b and c the matrix is stored as

|a.x b.x c.x|
|a.y b.y c.y|
|a.z b.z c.z|

Vectors are treated as columns, so the vector v is

|x|
|y|
|z|

And matrices are applied _before_ the vector (pre-multiplication)
v' = M*v

|x'|   |a.x b.x c.x|   |x|   |a.x*x + b.x*y + c.x*z|
|y'| = |a.y b.y c.y| * |y| = |a.y*x + b.y*y + c.y*z|
|z'|   |a.z b.z c.z|   |z|   |a.z*x + b.z*y + c.z*z|


Physical storage and indexing:
To be compatible with popular 3d rendering APIs (read D3d and OpenGL)
the physical indexing is

|0 3 6|
|1 4 7|
|2 5 8|

index = column*3 + row

which in C++ translates to M[column][row]

The mathematical indexing is M_row,column and this is what is used for _-notation
so _12 is 1st row, second column and operator(row, column)!

*/
class PxMat33
{
  public:
	//! Default constructor
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxMat33()
	{
	}

	//! identity constructor
	PX_CUDA_CALLABLE PX_INLINE PxMat33(PxIDENTITY r)
	: column0(1.0f, 0.0f, 0.0f), column1(0.0f, 1.0f, 0.0f), column2(0.0f, 0.0f, 1.0f)
	{
		PX_UNUSED(r);
	}

	//! zero constructor
	PX_CUDA_CALLABLE PX_INLINE PxMat33(PxZERO r) : column0(0.0f), column1(0.0f), column2(0.0f)
	{
		PX_UNUSED(r);
	}

	//! Construct from three base vectors
	PX_CUDA_CALLABLE PxMat33(const PxVec3& col0, const PxVec3& col1, const PxVec3& col2)
	: column0(col0), column1(col1), column2(col2)
	{
	}

	//! constructor from a scalar, which generates a multiple of the identity matrix
	explicit PX_CUDA_CALLABLE PX_INLINE PxMat33(float r)
	: column0(r, 0.0f, 0.0f), column1(0.0f, r, 0.0f), column2(0.0f, 0.0f, r)
	{
	}

	//! Construct from float[9]
	explicit PX_CUDA_CALLABLE PX_INLINE PxMat33(float values[])
	: column0(values[0], values[1], values[2])
	, column1(values[3], values[4], values[5])
	, column2(values[6], values[7], values[8])
	{
	}

	//! Construct from a quaternion
	explicit PX_CUDA_CALLABLE PX_FORCE_INLINE PxMat33(const PxQuat& q)
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

		column0 = PxVec3(1.0f - yy - zz, xy + zw, xz - yw);
		column1 = PxVec3(xy - zw, 1.0f - xx - zz, yz + xw);
		column2 = PxVec3(xz + yw, yz - xw, 1.0f - xx - yy);
	}

	//! Copy constructor
	PX_CUDA_CALLABLE PX_INLINE PxMat33(const PxMat33& other)
	: column0(other.column0), column1(other.column1), column2(other.column2)
	{
	}

	//! Assignment operator
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxMat33& operator=(const PxMat33& other)
	{
		column0 = other.column0;
		column1 = other.column1;
		column2 = other.column2;
		return *this;
	}

	//! Construct from diagonal, off-diagonals are zero.
	PX_CUDA_CALLABLE PX_INLINE static const PxMat33 createDiagonal(const PxVec3& d)
	{
		return PxMat33(PxVec3(d.x, 0.0f, 0.0f), PxVec3(0.0f, d.y, 0.0f), PxVec3(0.0f, 0.0f, d.z));
	}

	/**
	\brief returns true if the two matrices are exactly equal
	*/
	PX_CUDA_CALLABLE PX_INLINE bool operator==(const PxMat33& m) const
	{
		return column0 == m.column0 && column1 == m.column1 && column2 == m.column2;
	}

	//! Get transposed matrix
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxMat33 getTranspose() const
	{
		const PxVec3 v0(column0.x, column1.x, column2.x);
		const PxVec3 v1(column0.y, column1.y, column2.y);
		const PxVec3 v2(column0.z, column1.z, column2.z);

		return PxMat33(v0, v1, v2);
	}

	//! Get the real inverse
	PX_CUDA_CALLABLE PX_INLINE const PxMat33 getInverse() const
	{
		const float det = getDeterminant();
		PxMat33 inverse;

		if(det != 0)
		{
			const float invDet = 1.0f / det;

			inverse.column0.x = invDet * (column1.y * column2.z - column2.y * column1.z);
			inverse.column0.y = invDet * -(column0.y * column2.z - column2.y * column0.z);
			inverse.column0.z = invDet * (column0.y * column1.z - column0.z * column1.y);

			inverse.column1.x = invDet * -(column1.x * column2.z - column1.z * column2.x);
			inverse.column1.y = invDet * (column0.x * column2.z - column0.z * column2.x);
			inverse.column1.z = invDet * -(column0.x * column1.z - column0.z * column1.x);

			inverse.column2.x = invDet * (column1.x * column2.y - column1.y * column2.x);
			inverse.column2.y = invDet * -(column0.x * column2.y - column0.y * column2.x);
			inverse.column2.z = invDet * (column0.x * column1.y - column1.x * column0.y);

			return inverse;
		}
		else
		{
			return PxMat33(PxIdentity);
		}
	}

	//! Get determinant
	PX_CUDA_CALLABLE PX_INLINE float getDeterminant() const
	{
		return column0.dot(column1.cross(column2));
	}

	//! Unary minus
	PX_CUDA_CALLABLE PX_INLINE const PxMat33 operator-() const
	{
		return PxMat33(-column0, -column1, -column2);
	}

	//! Add
	PX_CUDA_CALLABLE PX_INLINE const PxMat33 operator+(const PxMat33& other) const
	{
		return PxMat33(column0 + other.column0, column1 + other.column1, column2 + other.column2);
	}

	//! Subtract
	PX_CUDA_CALLABLE PX_INLINE const PxMat33 operator-(const PxMat33& other) const
	{
		return PxMat33(column0 - other.column0, column1 - other.column1, column2 - other.column2);
	}

	//! Scalar multiplication
	PX_CUDA_CALLABLE PX_INLINE const PxMat33 operator*(float scalar) const
	{
		return PxMat33(column0 * scalar, column1 * scalar, column2 * scalar);
	}

	friend PxMat33 operator*(float, const PxMat33&);

	//! Matrix vector multiplication (returns 'this->transform(vec)')
	PX_CUDA_CALLABLE PX_INLINE const PxVec3 operator*(const PxVec3& vec) const
	{
		return transform(vec);
	}

	// a <op>= b operators

	//! Matrix multiplication
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxMat33 operator*(const PxMat33& other) const
	{
		// Rows from this <dot> columns from other
		// column0 = transform(other.column0) etc
		return PxMat33(transform(other.column0), transform(other.column1), transform(other.column2));
	}

	//! Equals-add
	PX_CUDA_CALLABLE PX_INLINE PxMat33& operator+=(const PxMat33& other)
	{
		column0 += other.column0;
		column1 += other.column1;
		column2 += other.column2;
		return *this;
	}

	//! Equals-sub
	PX_CUDA_CALLABLE PX_INLINE PxMat33& operator-=(const PxMat33& other)
	{
		column0 -= other.column0;
		column1 -= other.column1;
		column2 -= other.column2;
		return *this;
	}

	//! Equals scalar multiplication
	PX_CUDA_CALLABLE PX_INLINE PxMat33& operator*=(float scalar)
	{
		column0 *= scalar;
		column1 *= scalar;
		column2 *= scalar;
		return *this;
	}

	//! Equals matrix multiplication
	PX_CUDA_CALLABLE PX_INLINE PxMat33& operator*=(const PxMat33& other)
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

	// Transform etc

	//! Transform vector by matrix, equal to v' = M*v
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxVec3 transform(const PxVec3& other) const
	{
		return column0 * other.x + column1 * other.y + column2 * other.z;
	}

	//! Transform vector by matrix transpose, v' = M^t*v
	PX_CUDA_CALLABLE PX_INLINE const PxVec3 transformTranspose(const PxVec3& other) const
	{
		return PxVec3(column0.dot(other), column1.dot(other), column2.dot(other));
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE const float* front() const
	{
		return &column0.x;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3& operator[](unsigned int num)
	{
		return (&column0)[num];
	}
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxVec3& operator[](unsigned int num) const
	{
		return (&column0)[num];
	}

	// Data, see above for format!

	PxVec3 column0, column1, column2; // the three base vectors
};

// implementation from PxQuat.h
PX_CUDA_CALLABLE PX_INLINE PxQuat::PxQuat(const PxMat33& m)
{
	if(m.column2.z < 0)
	{
		if(m.column0.x > m.column1.y)
		{
			float t = 1 + m.column0.x - m.column1.y - m.column2.z;
			*this = PxQuat(t, m.column0.y + m.column1.x, m.column2.x + m.column0.z, m.column1.z - m.column2.y) *
			        (0.5f / PxSqrt(t));
		}
		else
		{
			float t = 1 - m.column0.x + m.column1.y - m.column2.z;
			*this = PxQuat(m.column0.y + m.column1.x, t, m.column1.z + m.column2.y, m.column2.x - m.column0.z) *
			        (0.5f / PxSqrt(t));
		}
	}
	else
	{
		if(m.column0.x < -m.column1.y)
		{
			float t = 1 - m.column0.x - m.column1.y + m.column2.z;
			*this = PxQuat(m.column2.x + m.column0.z, m.column1.z + m.column2.y, t, m.column0.y - m.column1.x) *
			        (0.5f / PxSqrt(t));
		}
		else
		{
			float t = 1 + m.column0.x + m.column1.y + m.column2.z;
			*this = PxQuat(m.column1.z - m.column2.y, m.column2.x - m.column0.z, m.column0.y - m.column1.x, t) *
			        (0.5f / PxSqrt(t));
		}
	}
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // #ifndef PXFOUNDATION_PXMAT33_H
