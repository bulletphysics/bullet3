// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Matrix.h
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
#ifndef BULLET_MATRIX_H
#define BULLET_MATRIX_H

#ifdef WIN32

#include "Vector.h"
#include "Memory2.h"

class Quat;

#include <malloc.h>


////////////////////////////////////////////////////////////////////////////////
/// Matrix33
__declspec(align(16)) class Matrix33
{
private:
	Vector3 m_rows[3];

public:

	BULLET_ALIGNED_NEW_AND_DELETE

	
	// constructors
	Matrix33();
	Matrix33(const Vector3& x, const Vector3& y, const Vector3& z);

	// explicit constructors
	explicit Matrix33(const Quat& q);

	explicit Matrix33(const Maths::ZeroTag&);
	explicit Matrix33(const Maths::IdentityTag&);
	explicit Matrix33(const Maths::RotateXTag&, float radians);
	explicit Matrix33(const Maths::RotateYTag&, float radians);
	explicit Matrix33(const Maths::RotateZTag&, float radians);
	explicit Matrix33(const Maths::ScaleTag&, const Vector3& scale);
	explicit Matrix33(const Maths::SkewTag&, const Vector3& v);

	// assignment
	const Matrix33& operator=(const Matrix33& m);

	// assignment to constant
	const Matrix33& operator=(const Maths::ZeroTag&);
	const Matrix33& operator=(const Maths::IdentityTag&);

	// element access
	Vector3& operator[](int row);
	const Vector3& operator[](int row) const;

	Vector3& GetAxisX();
	const Vector3& GetAxisX() const;
	void SetAxisX(const Vector3& v);

	Vector3& GetAxisY();
	const Vector3& GetAxisY() const;
	void SetAxisY(const Vector3& v);

	Vector3& GetAxisZ();
	const Vector3& GetAxisZ() const;
	void SetAxisZ(const Vector3& v);

	// operations
	void operator*=(const Matrix33& a);
	void operator*=(const Scalar& s);
	void operator+=(const Matrix33& a);
	void operator-=(const Matrix33& a);

	friend const Vector3 operator*(const Vector3& v, const Matrix33& m);
	friend const Vector3 operator*(const Matrix33& m, const Vector3& vT);
	friend const Matrix33 operator*(const Matrix33& a, const Matrix33& b);
	friend const Matrix33 operator*(const Matrix33& m, const Scalar& s);
	friend const Matrix33 operator+(const Matrix33& a, const Matrix33& b);
	friend const Matrix33 operator-(const Matrix33& a, const Matrix33& b);

	friend const Matrix33 Transpose(const Matrix33& m);
	friend const Matrix33 Inv(const Matrix33& m);
	friend const Scalar Det(const Matrix33& m);
};


////////////////////////////////////////////////////////////////////////////////
// Matrix44

__declspec(align(16)) class Matrix44
{
private:
	Vector4 m_rows[4];

public:
	// constructors
	Matrix44();
	Matrix44(const Vector4& x, const Vector4& y, const Vector4& z, const Vector4& w);

	// explicit constructors
	explicit Matrix44(const Maths::ZeroTag&);
	explicit Matrix44(const Maths::IdentityTag&);
	explicit Matrix44(const Maths::RotateXTag&, float radians);
	explicit Matrix44(const Maths::RotateYTag&, float radians);
	explicit Matrix44(const Maths::RotateZTag&, float radians);

	// assignment
	const Matrix44& operator=(const Matrix44& m);

	// assignment to constant
	const Matrix44& operator=(const Maths::ZeroTag&);
	const Matrix44& operator=(const Maths::IdentityTag&);

	// element access
	Vector4& operator[](int row);
	const Vector4& operator[](int row) const;

	// operations
	void operator*=(const Matrix44& a);
	void operator*=(const Scalar& s);
	void operator+=(const Matrix44& a);
	void operator-=(const Matrix44& a);

	friend const Vector3 operator*(const Vector3& v, const Matrix44& m);
	friend const Point3 operator*(const Point3& v, const Matrix44& m);
	friend const Vector4 operator*(const Vector4& v, const Matrix44& m);

	friend const Matrix44 operator*(const Matrix44& a, const Matrix44& b);
	friend const Matrix44 operator*(const Scalar& s, const Matrix44& m);
	friend const Matrix44 operator*(const Matrix44& m, const Scalar& s);
	friend const Matrix44 operator+(const Matrix44& a, const Matrix44& b);
	friend const Matrix44 operator-(const Matrix44& a, const Matrix44& b);

	friend const Matrix44 Transpose(const Matrix44& m);
	friend const Matrix44 Inv(const Matrix44& m);
};


////////////////////////////////////////////////////////////////////////////////
// Transform

__declspec(align(16)) class Transform
{
private:
	Matrix33 m_rotation;
	Point3 m_translation;

public:
	// constructors
	Transform();
	Transform(const Matrix33& xyz, const Point3& w);
	Transform(const Vector3& x, const Vector3& y, const Vector3& z, const Point3& w);

	// explicit constructors
	explicit Transform(const Maths::IdentityTag&);

	// assignment
	const Transform& operator=(const Transform& m);

	// assignment to constant
	const Transform& operator=(const Maths::IdentityTag&);

	// element access
	Matrix33& GetRotation();
	const Matrix33& GetRotation() const;
	void SetRotation(const Matrix33& m);
	void SetRotation(const Quat& q);

	Point3& GetTranslation();
	const Point3& GetTranslation() const;
	void SetTranslation(const Point3& p);

	Vector3& GetAxisX();
	const Vector3& GetAxisX() const;

	Vector3& GetAxisY();
	const Vector3& GetAxisY() const;

	Vector3& GetAxisZ();
	const Vector3& GetAxisZ() const;

	// operations
	friend const Vector3 operator*(const Vector3& v, const Transform& m);
	friend const Point3 operator*(const Point3& v, const Transform& m);

	friend const Transform operator*(const Transform& v, const Transform& m);

	friend const Transform Inv(const Transform& m);
};



#include "matrix.inl"

#endif //#ifdef WIN32
#endif //BULLET_MATRIX_H