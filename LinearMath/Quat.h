// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Quat.h
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
#ifndef BULLET_QUAT_H
#define BULLET_QUAT_H

#include "Vector.h"

class Matrix33;


class Quat : public Vector4Base
{
public:
	BULLET_ALIGNED_NEW_AND_DELETE

	// constructors
	Quat();
	Quat(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w);
	Quat(const Vector3& axis, const Scalar& angle);

	// construction to constant
	Quat(const Maths::IdentityTag&);

	// explicit constructors
	explicit Quat(const __m128 b);
	explicit Quat(const Vector3& v);
	explicit Quat(const Vector4& v);
	explicit Quat(const Matrix33& m);
	explicit Quat(const float* p);

	// assignment
	const Quat& operator=(const Quat& v);
	const Quat& operator=(const Maths::IdentityTag&);

	// in place operations
	void operator+=(const Quat& b);
	void operator-=(const Quat& b);
	void operator*=(const Quat& b);
	void operator*=(const Scalar& s);

	// operations
	friend const Quat operator-(const Quat& a);
	friend const Quat operator+(const Quat& a, const Quat& b);
	friend const Quat operator-(const Quat& a, const Quat& b);
	friend const Quat operator*(const Quat& a, const Quat& b);
	friend const Quat operator*(const Quat& a, const Scalar& s);
	friend const Quat operator*(const Scalar& s, const Quat& a);

	friend const Quat Inv(const Quat& a);

	friend const Scalar Dot(const Quat& a, const Quat& b);

	friend const Scalar Length(const Quat& a);
	friend const Quat Normalize(const Quat& a);

	friend const Scalar LengthFast(const Quat& a);
	friend const Quat NormalizeFast(const Quat& a);

	friend const Quat Slerp(const Quat& a, const Quat& b, const Scalar& t);
	friend const Quat Lerp(const Quat& a, const Quat& b, const Scalar& t);

	const Vector3 Rotate(const Vector3& v) const;
	void GetAngleAxis(Vector3& axis, Scalar& angle) const;

	// validation
	bool IsFinite() const;
};


#include "Quat.inl"


#endif //BULLET_QUAT_H
