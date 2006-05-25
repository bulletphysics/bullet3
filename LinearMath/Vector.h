// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Vector.h
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
#ifndef BULLET_VECTOR_H
#define BULLET_VECTOR_H

#include "VectorBase.h"

class Point3;
class Vector4;

////////////////////////////////////////////////////////////////////////////////
// Vector3
__declspec(align(16)) class Vector3 : public Vector4Base
{
public:
	BULLET_ALIGNED_NEW_AND_DELETE

	// constructors
	Vector3();
	Vector3(const Vector3& v);
	Vector3(float x, float y, float z);
	Vector3(const Scalar& x, const Scalar& y, const Scalar& z);

	// construction to constant
	Vector3(const Maths::ZeroTag&);
	Vector3(const Maths::UnitXTag&);
	Vector3(const Maths::UnitYTag&);
	Vector3(const Maths::UnitZTag&);
	Vector3(const Maths::UnitNegXTag&);
	Vector3(const Maths::UnitNegYTag&);
	Vector3(const Maths::UnitNegZTag&);

	// explicit constructors
	explicit Vector3(const __m128 b);
	explicit Vector3(const Vector4Base& v);
	explicit Vector3(const Scalar& v);
	explicit Vector3(const Point3& v);
	explicit Vector3(const Vector4& v);
	explicit Vector3(const float* p);

	// assignment
	const Vector3& operator=(const Vector3& v);

	// assignment to constant
	const Vector3& operator=(const Maths::ZeroTag&);
	const Vector3& operator=(const Maths::UnitXTag&);
	const Vector3& operator=(const Maths::UnitYTag&);
	const Vector3& operator=(const Maths::UnitZTag&);
	const Vector3& operator=(const Maths::UnitNegXTag&);
	const Vector3& operator=(const Maths::UnitNegYTag&);
	const Vector3& operator=(const Maths::UnitNegZTag&);

	// in place operations
	void operator+=(const Vector3& b);
	void operator-=(const Vector3& b);
	void operator*=(const Vector3& b);
	void operator/=(const Vector3& b);
	void operator*=(const Scalar& s);
	void operator/=(const Scalar& s);
	
	// operations
	friend const Vector3 operator-(const Vector3& a);

	friend const Vector3 operator+(const Vector3& a, const Vector3& b);
	friend const Vector3 operator-(const Vector3& a, const Vector3& b);
	friend const Vector3 operator*(const Vector3& a, const Vector3& b);
	friend const Vector3 operator/(const Vector3& a, const Vector3& b);
	friend const Vector3 operator*(const Vector3& a, const Scalar& s);
	friend const Vector3 operator*(const Scalar& s,  const Vector3& a);
	friend const Vector3 operator/(const Vector3& a, const Scalar& s);

	friend const Vector3 Abs(const Vector3& a);
	friend const Vector3 Rcp(const Vector3& a);
	friend const Vector3 Rsqrt(const Vector3& a);
	friend const Vector3 Sqrt(const Vector3& a);
	friend const Vector3 RcpNr(const Vector3& a);
	friend const Vector3 RsqrtNr(const Vector3& a);

	friend const Scalar Sum(const Vector3& a);
	friend const Scalar Dot(const Vector3& a, const Vector3& b);

	friend const Vector3 Cross(const Vector3& a, const Vector3& b);

	friend const Vector3 Min(const Vector3& a, const Vector3& b);
	friend const Vector3 Max(const Vector3& a, const Vector3& b);
	friend const Vector3 Clamp(const Vector3& v, const Vector3& min, const Vector3& max);

	friend const Scalar MinComp(const Vector3& a);
	friend const Scalar MaxComp(const Vector3& a);
	friend const int MinCompIndex(const Vector3& a);
	friend const int MaxCompIndex(const Vector3& a);

	friend const Scalar Length(const Vector3& a);
	friend const Scalar LengthSqr(const Vector3& a);
	friend const Scalar LengthRcp(const Vector3& a);
	friend const Vector3 Normalize(const Vector3& a);

	friend const Scalar LengthFast(const Vector3& a);
	friend const Scalar LengthRcpFast(const Vector3& a);
	friend const Vector3 NormalizeFast(const Vector3& a);

	friend const Vector3 Lerp(const Vector3& a, const Vector3& b, const Scalar& t);

	// returns an arbitrary perpendicular vector (not normalized)
	friend const Vector3 Perp(const Vector3& v);

	// comparisons (return 1 bit per component)
	friend int operator==(const Vector3& a, const Vector3& b);
	friend int operator!=(const Vector3& a, const Vector3& b);
	friend int operator<(const Vector3& a, const Vector3& b);
	friend int operator<=(const Vector3& a, const Vector3& b);
	friend int operator>(const Vector3& a, const Vector3& b);
	friend int operator>=(const Vector3& a, const Vector3& b);

	// validation
	bool IsFinite() const;
};


////////////////////////////////////////////////////////////////////////////////
// Point3
__declspec(align(16)) class Point3 : public Vector4Base
{
public:
	// constructors
	Point3();
	Point3(const Point3& v);
	Point3(float x, float y, float z);
	Point3(const Scalar& x, const Scalar& y, const Scalar& z);

	// construction to constant
	Point3(const Maths::ZeroTag&);
	Point3(const Maths::UnitXTag&);
	Point3(const Maths::UnitYTag&);
	Point3(const Maths::UnitZTag&);
	Point3(const Maths::UnitNegXTag&);
	Point3(const Maths::UnitNegYTag&);
	Point3(const Maths::UnitNegZTag&);

	// explicit constructors
	explicit Point3(const __m128 b);
	explicit Point3(const Vector4Base& v);
	explicit Point3(const Scalar& v);
	explicit Point3(const Vector3& v);
	explicit Point3(const Vector4& v);
	explicit Point3(const float* p);

	// assignment
	const Point3& operator=(const Point3& v);

	// assignment to constant
	const Point3& operator=(const Maths::ZeroTag&);
	const Point3& operator=(const Maths::UnitXTag&);
	const Point3& operator=(const Maths::UnitYTag&);
	const Point3& operator=(const Maths::UnitZTag&);
	const Point3& operator=(const Maths::UnitNegXTag&);
	const Point3& operator=(const Maths::UnitNegYTag&);
	const Point3& operator=(const Maths::UnitNegZTag&);

	// in place operations
	void operator+=(const Vector3& b);
	void operator-=(const Vector3& b);
	
	// operations
	friend const Point3 operator-(const Point3& a);

	friend const Point3 operator+(const Point3& a, const Vector3& b);
	friend const Point3 operator+(const Vector3& a, const Point3& b);

	friend const Point3 operator-(const Point3& a, const Vector3& b);
	friend const Vector3 operator-(const Point3& a, const Point3& b);

	friend const Vector3 operator*(const Point3& a, const Vector3& b);
	friend const Vector3 operator*(const Vector3& a, const Point3& b);

	friend const Point3 Abs(const Point3& a);

	friend const Scalar Sum(const Point3& a);
	friend const Scalar Dot(const Point3& a, const Point3& b);
	friend const Scalar Dot(const Vector3& a, const Point3& b);
	friend const Scalar Dot(const Point3& a, const Vector3& b);

	friend const Point3 Min(const Point3& a, const Point3& b);
	friend const Point3 Max(const Point3& a, const Point3& b);
	friend const Point3 Clamp(const Point3& v, const Point3& min, const Point3& max);

	friend const Scalar Dist(const Point3& a, const Point3& b);
	friend const Scalar DistSqr(const Point3& a, const Point3& b);
	friend const Scalar DistRcp(const Point3& a, const Point3& b);
	
	friend const Scalar DistFast(const Point3& a, const Point3& b);
	friend const Scalar DistRcpFast(const Point3& a, const Point3& b);

	friend const Point3 Lerp(const Point3& a, const Point3& b, const Scalar& t);

	friend const Point3 Homogenize(const Vector4& v);
	friend const Point3 HomogenizeFast(const Vector4& v);

	// comparisons (return 1 bit per component)
	friend int operator==(const Point3& a, const Point3& b);
	friend int operator!=(const Point3& a, const Point3& b);
	friend int operator<(const Point3& a, const Point3& b);
	friend int operator<=(const Point3& a, const Point3& b);
	friend int operator>(const Point3& a, const Point3& b);
	friend int operator>=(const Point3& a, const Point3& b);

	// validation
	bool IsFinite() const;
};


////////////////////////////////////////////////////////////////////////////////
// Vector4
__declspec(align(16)) class Vector4 : public Vector4Base
{
public:
	// constructors
	Vector4();
	Vector4(const Vector4& v);
	Vector4(float x, float y, float z, float w);
	Vector4(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w);
	Vector4(const Vector3& xyz, const Scalar& w);

	// construction to constant
	Vector4(const Maths::ZeroTag&);
	Vector4(const Maths::UnitXTag&);
	Vector4(const Maths::UnitYTag&);
	Vector4(const Maths::UnitZTag&);
	Vector4(const Maths::UnitWTag&);
	Vector4(const Maths::UnitNegXTag&);
	Vector4(const Maths::UnitNegYTag&);
	Vector4(const Maths::UnitNegZTag&);
	Vector4(const Maths::UnitNegWTag&);

	// explicit constructors
	explicit Vector4(const __m128 b);
	explicit Vector4(const Vector4Base& v);
	explicit Vector4(const Scalar& v);
	explicit Vector4(const Vector3& v);
	explicit Vector4(const Point3& v);
	explicit Vector4(const float* p);

	// assignment
	const Vector4& operator=(const Vector4& v);

	// assignment to constant
	const Vector4& operator=(const Maths::ZeroTag&);
	const Vector4& operator=(const Maths::UnitXTag&);
	const Vector4& operator=(const Maths::UnitYTag&);
	const Vector4& operator=(const Maths::UnitZTag&);
	const Vector4& operator=(const Maths::UnitWTag&);
	const Vector4& operator=(const Maths::UnitNegXTag&);
	const Vector4& operator=(const Maths::UnitNegYTag&);
	const Vector4& operator=(const Maths::UnitNegZTag&);
	const Vector4& operator=(const Maths::UnitNegWTag&);

	// in place operations
	void operator+=(const Vector4& b);
	void operator-=(const Vector4& b);
	void operator*=(const Vector4& b);
	void operator/=(const Vector4& b);
	void operator*=(const Scalar& s);
	void operator/=(const Scalar& s);
	
	// operations
	friend const Vector4 operator-(const Vector4& a);

	friend const Vector4 operator+(const Vector4& a, const Vector4& b);
	friend const Vector4 operator-(const Vector4& a, const Vector4& b);
	friend const Vector4 operator*(const Vector4& a, const Vector4& b);
	friend const Vector4 operator/(const Vector4& a, const Vector4& b);
	friend const Vector4 operator*(const Vector4& a, const Scalar& s);
	friend const Vector4 operator*(const Scalar& s,  const Vector4& a);
	friend const Vector4 operator/(const Vector4& a, const Scalar& s);

	friend const Vector4 Abs(const Vector4& a);
	friend const Vector4 Rcp(const Vector4& a);
	friend const Vector4 Rsqrt(const Vector4& a);
	friend const Vector4 Sqrt(const Vector4& a);
	friend const Vector4 RcpNr(const Vector4& a);
	friend const Vector4 RsqrtNr(const Vector4& a);

	friend const Scalar Sum(const Vector4& a);
	friend const Scalar Dot(const Vector4& a, const Vector4& b);

	friend const Vector4 Min(const Vector4& a, const Vector4& b);
	friend const Vector4 Max(const Vector4& a, const Vector4& b);
	friend const Vector4 Clamp(const Vector4& v, const Vector4& min, const Vector4& max);

	friend const Scalar MinComp(const Vector4& a);
	friend const Scalar MaxComp(const Vector4& a);

	friend const Scalar Length(const Vector4& a);
	friend const Scalar LengthSqr(const Vector4& a);
	friend const Scalar LengthRcp(const Vector4& a);
	friend const Vector4 Normalize(const Vector4& a);

	friend const Scalar LengthFast(const Vector4& a);
	friend const Scalar LengthRcpFast(const Vector4& a);
	friend const Vector4 NormalizeFast(const Vector4& a);

	friend const Vector4 Lerp(const Vector4& a, const Vector4& b, const Scalar& t);

	// comparisons (return 1 bit per component)
	friend int operator==(const Vector4& a, const Vector4& b);
	friend int operator!=(const Vector4& a, const Vector4& b);
	friend int operator<(const Vector4& a, const Vector4& b);
	friend int operator<=(const Vector4& a, const Vector4& b);
	friend int operator>(const Vector4& a, const Vector4& b);
	friend int operator>=(const Vector4& a, const Vector4& b);

	// validation
	bool IsFinite() const;
};


////////////////////////////////////////////////////////////////////////////////
// Vector2
class Vector2 : public Vector2Base
{
public:
};


////////////////////////////////////////////////////////////////////////////////
// Point2
class Point2 : public Vector2Base
{
public:

};


#include "Vector.inl"
#endif //BULLET_VECTOR_H
