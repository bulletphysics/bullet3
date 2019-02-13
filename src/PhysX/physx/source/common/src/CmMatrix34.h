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


#ifndef PX_PHYSICS_COMMON_MATRIX34
#define PX_PHYSICS_COMMON_MATRIX34

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMat33.h"

#include "CmPhysXCommon.h"

namespace physx
{
namespace Cm
{

/*!
Basic mathematical 3x4 matrix, implemented as a 3x3 rotation matrix and a translation

See PxMat33 for the format of the rotation matrix.

*/

class Matrix34
{
public:
	//! Default constructor
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34()
	{}

	//! Construct from four base vectors
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34(const PxVec3& b0, const PxVec3& b1, const PxVec3& b2, const PxVec3& b3)
		: m(b0, b1, b2), p(b3)
	{}

	//! Construct from float[12]
	explicit PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34(PxReal values[]):
		m(values), p(values[9], values[10], values[11])
	{		
	}

	//! Construct from a 3x3 matrix
	explicit PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34(const PxMat33& other)
		: m(other), p(PxZero)
	{
	}

	//! Construct from a 3x3 matrix and a translation vector
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34(const PxMat33& other, const PxVec3& t)
		: m(other), p(t)
	{}

	//! Construct from a PxTransform
	explicit PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34(const PxTransform& other):
		m(other.q), p(other.p)
	{
	}

	//! Construct from a quaternion
	explicit PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34(const PxQuat& q):
		m(q), p(PxZero)
	{
	}

	//! Copy constructor
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34(const Matrix34& other):
		m(other.m), p(other.p)
	{
	}

	//! Assignment operator
	PX_CUDA_CALLABLE PX_FORCE_INLINE const Matrix34& operator=(const Matrix34& other)
	{
		m = other.m;
		p = other.p;
		return *this;
	}

	//! Set to identity matrix
	PX_CUDA_CALLABLE PX_FORCE_INLINE void setIdentity()
	{
		m = PxMat33(PxIdentity);
		p = PxVec3(0);
	}
	
	// Simpler operators
	//! Equality operator
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool operator==(const Matrix34& other) const
	{
		return m == other.m && p == other.p;
	}

	//! Inequality operator
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool operator!=(const Matrix34& other) const
	{
		return !operator==(other);
	}

	//! Unary minus
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34 operator-() const
	{
		return Matrix34(-m, -p);
	}

	//! Add
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34 operator+(const Matrix34& other) const
	{
		return Matrix34(m + other.m, p + other.p);
	}

	//! Subtract
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34 operator-(const Matrix34& other) const
	{
		return Matrix34(m - other.m, p - other.p);
	}

	//! Scalar multiplication
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34 operator*(PxReal scalar) const
	{
		return Matrix34(m*scalar, p*scalar);
	}

	friend Matrix34 operator*(PxReal, const Matrix34&);

	//! Matrix multiplication
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34 operator*(const Matrix34& other) const
	{
		//Rows from this <dot> columns from other
		//base0 = rotate(other.m.column0) etc
		return Matrix34(m*other.m, m*other.p + p);
	}

	//! Matrix multiplication, extend the second matrix
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34 operator*(const PxMat33& other) const
	{
		//Rows from this <dot> columns from other
		//base0 = transform(other.m.column0) etc
		return Matrix34(m*other, p);
	}

	friend Matrix34 operator*(const PxMat33& a, const Matrix34& b);
	
	// a <op>= b operators

	//! Equals-add
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34& operator+=(const Matrix34& other)
	{
		m += other.m;
		p += other.p;
		return *this;
	}

	//! Equals-sub
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34& operator-=(const Matrix34& other)
	{
		m -= other.m;
		p -= other.p;
		return *this;
	}

	//! Equals scalar multiplication
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34& operator*=(PxReal scalar)
	{
		m *= scalar;
		p *= scalar;

		return *this;
	}

	//! Element access, mathematical way!
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal operator()(PxU32 row, PxU32 col) const
	{
		return (*this)[col][row];
	}

	//! Element access, mathematical way!
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal& operator()(PxU32 row, PxU32 col)
	{
		return (*this)[col][row];
	}

	// Transform etc
	
	//! Transform vector by matrix, equal to v' = M*v
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 rotate(const PxVec3& other) const
	{
		return m*other;
	}

	//! Transform vector by transpose of matrix, equal to v' = M^t*v
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 rotateTranspose(const PxVec3& other) const
	{
		return m.transformTranspose(other);
	}

	//! Transform point by matrix
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 transform(const PxVec3& other) const
	{
		return m*other + p;
	}

	//! Transform point by transposed matrix
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 transformTranspose(const PxVec3& other) const
	{
		return m.transformTranspose(other - p);
	}

	//! Transform point by transposed matrix
	PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::Matrix34 transformTranspose(const Cm::Matrix34& other) const
	{
		return Cm::Matrix34(m.transformTranspose(other.m.column0), 
							m.transformTranspose(other.m.column1), 
							m.transformTranspose(other.m.column2), 
							m.transformTranspose(other.p - p));
	}


	//! Invert matrix treating it as a rotation+translation matrix only
	PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34 getInverseRT() const
	{
		return Matrix34(m.getTranspose(), m.transformTranspose(-p));
	}


	// Conversion
	//! Set matrix from quaternion
	PX_CUDA_CALLABLE PX_FORCE_INLINE void set(const PxQuat& q) 
	{
		m = PxMat33(q);
		p = PxVec3(PxZero);
	}


	PX_CUDA_CALLABLE PX_FORCE_INLINE		PxVec3& operator[](unsigned int num){return (&m.column0)[num];}
	PX_CUDA_CALLABLE PX_FORCE_INLINE		PxVec3& operator[](int num)			{ return (&m.column0)[num]; }
	PX_CUDA_CALLABLE PX_FORCE_INLINE const	PxVec3& operator[](unsigned int num) const	{ return (&m.column0)[num]; }
	PX_CUDA_CALLABLE PX_FORCE_INLINE const	PxVec3& operator[](int num) const	{ return (&m.column0)[num]; }

	//Data, see above for format!

	PxMat33 m;
	PxVec3 p;

};


//! Multiply a*b, a is extended
PX_INLINE Matrix34 operator*(const PxMat33& a, const Matrix34& b)
{
	return Matrix34(a * b.m, a * b.p);
}


} // namespace Cm

}

#endif
