// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
//
// Matrix.inl
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
#pragma once

#include <assert.h>
#include "math.h"

////////////////////////////////////////////////////////////////////////////////
// Matrix33

inline Matrix33::Matrix33()
{
}

inline Matrix33::Matrix33(const Vector3& x, const Vector3& y, const Vector3& z)
{
	m_rows[0] = x;
	m_rows[1] = y;
	m_rows[2] = z;
}

inline Matrix33::Matrix33(const Maths::ZeroTag&)
{
	m_rows[0] = Maths::Zero;
	m_rows[1] = Maths::Zero;
	m_rows[2] = Maths::Zero;
}

inline Matrix33::Matrix33(const Maths::IdentityTag&)
{
	m_rows[0] = Maths::UnitX;
	m_rows[1] = Maths::UnitY;
	m_rows[2] = Maths::UnitZ;
}

inline Matrix33::Matrix33(const Maths::RotateXTag&, float radians)
{
	float c = cosf(radians);
	float s = sinf(radians);

	m_rows[0] = Maths::UnitX;
	m_rows[1] = Vector3(0.0f, c, s);
	m_rows[2] = Vector3(0.0f, -s, c);
}

inline Matrix33::Matrix33(const Maths::RotateYTag&, float radians)
{
	float c = cosf(radians);
	float s = sinf(radians);

	m_rows[0] = Vector3(c, 0.0f, -s);
	m_rows[1] = Maths::UnitY;
	m_rows[2] = Vector3(s, 0.0f, c);
}

inline Matrix33::Matrix33(const Maths::RotateZTag&, float radians)
{
	float c = cosf(radians);
	float s = sinf(radians);

	m_rows[0] = Vector3(c, s, 0.0f);
	m_rows[1] = Vector3(-s, c, 0.0f);
	m_rows[2] = Maths::UnitZ;
}

inline Matrix33::Matrix33(const Maths::ScaleTag&, const Vector3& scale)
{
	m_rows[0] = scale * Vector3(Maths::UnitX);
	m_rows[1] = scale * Vector3(Maths::UnitY);
	m_rows[2] = scale * Vector3(Maths::UnitZ);
}

inline Matrix33::Matrix33(const Maths::SkewTag&, const Vector3& v)
{
	m_rows[0] = Vector3(0.0f, v[2], -v[1]);
	m_rows[1] = Vector3(-v[2], 0.0f, v[0]);
	m_rows[2] = Vector3(v[1], -v[0], 0.0f);
}

inline const Matrix33& Matrix33::operator=(const Matrix33& m)
{
	m_rows[0] = m.m_rows[0];
	m_rows[1] = m.m_rows[1];
	m_rows[2] = m.m_rows[2];
	return *this;
}

inline const Matrix33& Matrix33::operator=(const Maths::ZeroTag&)
{
	m_rows[0] = m_rows[1] = m_rows[2] = Maths::Zero;
	return *this;
}

inline const Matrix33& Matrix33::operator=(const Maths::IdentityTag&)
{
	m_rows[0] = Maths::UnitX;
	m_rows[1] = Maths::UnitY;
	m_rows[2] = Maths::UnitZ;
	return *this;
}

inline Vector3& Matrix33::operator[](int row)
{
	return m_rows[row];
}

inline const Vector3& Matrix33::operator[](int row) const
{
	return m_rows[row];
}

inline Vector3& Matrix33::GetAxisX()
{
	return m_rows[0];
}

inline const Vector3& Matrix33::GetAxisX() const
{
	return m_rows[0];
}

inline void Matrix33::SetAxisX(const Vector3& v)
{
	m_rows[0] = v;
}

inline Vector3& Matrix33::GetAxisY()
{
	return m_rows[1];
}

inline const Vector3& Matrix33::GetAxisY() const
{
	return m_rows[1];
}

inline void Matrix33::SetAxisY(const Vector3& v)
{
	m_rows[1] = v;
}

inline Vector3& Matrix33::GetAxisZ()
{
	return m_rows[2];
}

inline const Vector3& Matrix33::GetAxisZ() const
{
	return m_rows[2];
}

inline void Matrix33::SetAxisZ(const Vector3& v)
{
	m_rows[2] = v;
}

inline const Vector3 operator*(const Vector3& v, const Matrix33& m)
{
	Scalar xxxx = v.GetX();
	Scalar yyyy = v.GetY();
	Scalar zzzz = v.GetZ();

	return xxxx * m[0] + yyyy * m[1] + zzzz * m[2];
}

inline const Vector3 operator*(const Matrix33& m, const Vector3& vT)
{
	return Vector3(Dot(m[0], vT), Dot(m[1], vT), Dot(m[2], vT));
}

inline void Matrix33::operator*=(const Matrix33& a)
{
	*this = *this * a;
}

inline void Matrix33::operator*=(const Scalar& s)
{
	m_rows[0] *= s;
	m_rows[1] *= s;
	m_rows[2] *= s;
}

inline void Matrix33::operator+=(const Matrix33& a)
{
	m_rows[0] += a[0];
	m_rows[1] += a[1];
	m_rows[2] += a[2];
}

inline void Matrix33::operator-=(const Matrix33& a)
{
	m_rows[0] -= a[0];
	m_rows[1] -= a[1];
	m_rows[2] -= a[2];
}

inline const Matrix33 operator*(const Scalar& s, const Matrix33& m)
{
	Vector3 scale(s);
	return Matrix33(scale * m[0], scale * m[1], scale * m[2]);
}

inline const Matrix33 operator*(const Matrix33& m, const Scalar& s)
{
	Vector3 scale(s);
	return Matrix33(scale * m[0], scale * m[1], scale * m[2]);
}

inline const Matrix33 operator*(const Matrix33& a, const Matrix33& b)
{
	return Matrix33(a[0] * b, a[1] * b, a[2] * b);
}

inline const Matrix33 operator+(const Matrix33& a, const Matrix33& b)
{
	return Matrix33(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

inline const Matrix33 operator-(const Matrix33& a, const Matrix33& b)
{
	return Matrix33(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

inline const Matrix33 Transpose(const Matrix33& m)
{
	// easiest way is to actually do a 4 * 4 transpose with an implied zero bottom row:

	// a b c d			a e i 0
	// e f g h	 --->	b f j 0
	// i j k l			c g k 0
	// 0 0 0 0

	// shuffle the rows to make 4 quarters
	__m128 abef = _mm_shuffle_ps(m[0].base, m[1].base, _MM_SHUFFLE(1, 0, 1, 0));
	__m128 cdgh = _mm_shuffle_ps(m[0].base, m[1].base, _MM_SHUFFLE(3, 2, 3, 2));
	__m128 ij00 = _mm_shuffle_ps(m[2].base, _mm_setzero_ps(), _MM_SHUFFLE(1, 0, 1, 0));
	__m128 kl00 = _mm_shuffle_ps(m[2].base, _mm_setzero_ps(), _MM_SHUFFLE(3, 2, 3, 2));

	// shuffle the quarters to make new rows
	__m128 aei0 = _mm_shuffle_ps(abef, ij00, _MM_SHUFFLE(2, 0, 2, 0));
	__m128 bfj0 = _mm_shuffle_ps(abef, ij00, _MM_SHUFFLE(3, 1, 3, 1));
	__m128 cgk0 = _mm_shuffle_ps(cdgh, kl00, _MM_SHUFFLE(2, 0, 2, 0));

	return Matrix33(Vector3(aei0), Vector3(bfj0), Vector3(cgk0));
}

inline const Scalar Det(const Matrix33& m)
{
	return Dot(Cross(m.GetAxisX(), m.GetAxisY()),m.GetAxisZ());
}


////////////////////////////////////////////////////////////////////////////////
// Matrix44

inline Matrix44::Matrix44()
{
}

inline Matrix44::Matrix44(const Vector4& x, const Vector4& y, const Vector4& z, const Vector4& w)
{
	m_rows[0] = x;
	m_rows[1] = y;
	m_rows[2] = z;
	m_rows[3] = w;
}

inline Matrix44::Matrix44(const Maths::ZeroTag&)
{
	m_rows[0] = Maths::Zero;
	m_rows[1] = Maths::Zero;
	m_rows[2] = Maths::Zero;
	m_rows[3] = Maths::Zero;
}

inline Matrix44::Matrix44(const Maths::IdentityTag&)
{
	m_rows[0] = Maths::UnitX;
	m_rows[1] = Maths::UnitY;
	m_rows[2] = Maths::UnitZ;
	m_rows[3] = Maths::UnitW;
}

inline Matrix44::Matrix44(const Maths::RotateXTag&, float radians)
{
	float c = cosf(radians);
	float s = sinf(radians);

	m_rows[0] = Maths::UnitX;
	m_rows[1] = Vector4(0.0f, c, s, 0.0f);
	m_rows[2] = Vector4(0.0f, -s, c, 0.0f);
	m_rows[3] = Maths::UnitW;
}

inline Matrix44::Matrix44(const Maths::RotateYTag&, float radians)
{
	float c = cosf(radians);
	float s = sinf(radians);

	m_rows[0] = Vector4(c, 0.0f, -s, 0.0f);
	m_rows[1] = Maths::UnitY;
	m_rows[2] = Vector4(s, 0.0f, c, 0.0f);
	m_rows[3] = Maths::UnitW;
}

inline Matrix44::Matrix44(const Maths::RotateZTag&, float radians)
{
	float c = cosf(radians);
	float s = sinf(radians);

	m_rows[0] = Vector4(c, s, 0.0f, 0.0f);
	m_rows[1] = Vector4(-s, c, 0.0f, 0.0f);
	m_rows[2] = Maths::UnitZ;
	m_rows[3] = Maths::UnitW;
}

inline const Matrix44& Matrix44::operator=(const Matrix44& m)
{
	m_rows[0] = m.m_rows[0];
	m_rows[1] = m.m_rows[1];
	m_rows[2] = m.m_rows[2];
	m_rows[3] = m.m_rows[3];
	return *this;
}

inline const Matrix44& Matrix44::operator=(const Maths::ZeroTag&)
{
	m_rows[0] = m_rows[1] = m_rows[2] = m_rows[3] = Maths::Zero;
	return *this;
}

inline const Matrix44& Matrix44::operator=(const Maths::IdentityTag&)
{
	m_rows[0] = Maths::UnitX;
	m_rows[1] = Maths::UnitY;
	m_rows[2] = Maths::UnitZ;
	m_rows[3] = Maths::UnitW;
	return *this;
}

inline Vector4& Matrix44::operator[](int row)
{
	return m_rows[row];
}

inline const Vector4& Matrix44::operator[](int row) const
{
	return m_rows[row];
}

inline void Matrix44::operator*=(const Matrix44& a)
{
	*this = *this * a;
}

inline void Matrix44::operator*=(const Scalar& s)
{
	m_rows[0] *= s;
	m_rows[1] *= s;
	m_rows[2] *= s;
	m_rows[3] *= s;
}

inline void Matrix44::operator+=(const Matrix44& a)
{
	m_rows[0] += a[0];
	m_rows[1] += a[1];
	m_rows[2] += a[2];
	m_rows[3] += a[3];
}

inline void Matrix44::operator-=(const Matrix44& a)
{
	m_rows[0] -= a[0];
	m_rows[1] -= a[1];
	m_rows[2] -= a[2];
	m_rows[3] -= a[3];
}

inline const Vector3 operator*(const Vector3& v, const Matrix44& m)
{
	Scalar xxxx = v.GetX();
	Scalar yyyy = v.GetY();
	Scalar zzzz = v.GetZ();

	return xxxx * Vector3(m[0]) + yyyy * Vector3(m[1]) + zzzz * Vector3(m[2]);
}

inline const Point3 operator*(const Point3& v, const Matrix44& m)
{
	Scalar xxxx = v.GetX();
	Scalar yyyy = v.GetY();
	Scalar zzzz = v.GetZ();

	return Point3(xxxx * m[0] + yyyy * m[1] + zzzz * m[2] + m[3]);
}

inline const Vector4 operator*(const Vector4& v, const Matrix44& m)
{
	Scalar xxxx = v.GetX();
	Scalar yyyy = v.GetY();
	Scalar zzzz = v.GetZ();
	Scalar wwww = v.GetW();

	return xxxx * m[0] + yyyy * m[1] + zzzz * m[2] + wwww * m[3];
}

inline const Matrix44 operator*(const Matrix44& a, const Matrix44& b)
{
	return Matrix44(a[0] * b, a[1] * b, a[2] * b, a[3] * b);
}

inline const Matrix44 operator*(const Matrix44& m, const Scalar& s)
{
	Vector4 scale(s);
	return Matrix44(scale * m[0], scale * m[1], scale * m[2], scale * m[3]);
}

inline const Matrix44 operator*(const Scalar& s, const Matrix44& m)
{
	Vector4 scale(s);
	return Matrix44(scale * m[0], scale * m[1], scale * m[2], scale * m[3]);
}

inline const Matrix44 operator+(const Matrix44& a, const Matrix44& b)
{
	return Matrix44(a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]);
}

inline const Matrix44 operator-(const Matrix44& a, const Matrix44& b)
{
	return Matrix44(a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]);
}

inline const Matrix44 Transpose(const Matrix44& m)
{
	// a b c d			a e i m
	// e f g h	 --->	b f j n
	// i j k l			c g k o
	// m n o p			d h l p

	// shuffle the rows to make 4 quarters
	__m128 abef = _mm_shuffle_ps(m[0].base, m[1].base, _MM_SHUFFLE(1, 0, 1, 0));
	__m128 cdgh = _mm_shuffle_ps(m[0].base, m[1].base, _MM_SHUFFLE(3, 2, 3, 2));
	__m128 ijmn = _mm_shuffle_ps(m[2].base, m[3].base, _MM_SHUFFLE(1, 0, 1, 0));
	__m128 klop = _mm_shuffle_ps(m[2].base, m[3].base, _MM_SHUFFLE(3, 2, 3, 2));

	// shuffle the quarters to make new rows
	__m128 aeim = _mm_shuffle_ps(abef, ijmn, _MM_SHUFFLE(2, 0, 2, 0));
	__m128 bfjn = _mm_shuffle_ps(abef, ijmn, _MM_SHUFFLE(3, 1, 3, 1));
	__m128 cgko = _mm_shuffle_ps(cdgh, klop, _MM_SHUFFLE(2, 0, 2, 0));
	__m128 dhlp = _mm_shuffle_ps(cdgh, klop, _MM_SHUFFLE(3, 1, 3, 1));

	return Matrix44(Vector4(aeim), Vector4(bfjn), Vector4(cgko), Vector4(dhlp));
}


////////////////////////////////////////////////////////////////////////////////
// Transform

inline Transform::Transform()
{
}

inline Transform::Transform(const Matrix33& xyz, const Point3& w)
{
	m_rotation = xyz;
	m_translation = w;
}

inline Transform::Transform(const Vector3& x, const Vector3& y, const Vector3& z, const Point3& w)
{
	m_rotation[0] = x;
	m_rotation[1] = y;
	m_rotation[2] = z;
	m_translation = w;
}

inline Transform::Transform(const Maths::IdentityTag&)
{
	m_rotation = Maths::Identity;
	m_translation = Maths::Zero;
}

inline const Transform& Transform::operator=(const Transform& m)
{
	m_rotation = m.m_rotation;
	m_translation = m.m_translation;
	return *this;
}

inline const Transform& Transform::operator=(const Maths::IdentityTag&)
{
	m_rotation = Maths::Identity;
	m_translation = Maths::Zero;
	return *this;
}

inline Matrix33& Transform::GetRotation()
{
	return m_rotation;
}

inline const Matrix33& Transform::GetRotation() const
{
	return m_rotation;
}

inline void Transform::SetRotation(const Matrix33& m)
{
	m_rotation = m;
}

inline void Transform::SetRotation(const Quat& q)
{
	m_rotation = Matrix33(q);
}

inline Point3& Transform::GetTranslation()
{
	return m_translation;
}

inline const Point3& Transform::GetTranslation() const
{
	return m_translation;
}

inline void Transform::SetTranslation(const Point3& t)
{
	m_translation = t;
}

inline Vector3& Transform::GetAxisX()
{
	return m_rotation[0];
}

inline const Vector3& Transform::GetAxisX() const
{
	return m_rotation[0];
}

inline Vector3& Transform::GetAxisY()
{
	return m_rotation[1];
}

inline const Vector3& Transform::GetAxisY() const
{
	return m_rotation[1];
}

inline Vector3& Transform::GetAxisZ()
{
	return m_rotation[2];
}

inline const Vector3& Transform::GetAxisZ() const
{
	return m_rotation[2];
}

inline const Vector3 operator*(const Vector3& v, const Transform& m)
{
	Scalar xxxx = v.GetX();
	Scalar yyyy = v.GetY();
	Scalar zzzz = v.GetZ();

	return xxxx * m.GetAxisX() + yyyy * m.GetAxisY() + zzzz * m.GetAxisZ();
}

inline const Point3 operator*(const Point3& v, const Transform& m)
{
	Scalar xxxx = v.GetX();
	Scalar yyyy = v.GetY();
	Scalar zzzz = v.GetZ();

	return xxxx * m.GetAxisX() + yyyy * m.GetAxisY() + zzzz * m.GetAxisZ() + m.GetTranslation();
}

inline const Transform operator*(const Transform& a, const Transform& b)
{
	return Transform(a.GetAxisX() * b, a.GetAxisY() * b, a.GetAxisZ() * b, a.GetTranslation() * b);
}

inline const Transform Inv(const Transform& m)
{
	Matrix33 r(Transpose(m.GetRotation()));
	Point3 t = m.GetTranslation();
	t = Point3(-(t.GetX() * r.GetAxisX() + t.GetY() * r.GetAxisY() + t.GetZ() * r.GetAxisZ()));

	return Transform(r, t); 
}
