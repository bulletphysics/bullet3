// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Quat.inl
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

inline Quat::Quat()
{
}

inline  Quat::Quat(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w)
{
	Set(x.base, y.base, z.base, w.base);
}

inline Quat::Quat(const Vector3& axis, const Scalar& angle)
{
	assert(axis.IsFinite());
	Set((sinf(0.5f * (float)angle) * axis).base, Scalar(cosf(0.5f * (float)angle)).base);
}

inline Quat::Quat(const Maths::IdentityTag&)
{
	base = Vector4Base::Consts::k0001;
}

inline Quat::Quat(const Vector3& v)
{
	base = v.base;
}

inline Quat::Quat(const Vector4& v)
{
	base = v.base;
}

inline Quat::Quat(const __m128 b)
{
	base = b;
}

inline Quat::Quat(const float* p)
{
	base = _mm_load_ps(p);
}

inline const Quat& Quat::operator=(const Quat &q)
{		
	base = q.base;
	return *this;
}

inline const Quat& Quat::operator=(const Maths::IdentityTag&)
{
	base = Vector4Base::Consts::k0001;
	return *this;
}

inline void Quat::operator+=(const Quat& b)
{
	base = _mm_add_ps(base, b.base);
}

inline void Quat::operator-=(const Quat& b)
{
	base = _mm_sub_ps(base, b.base);
}

inline void Quat::operator*=(const Quat& b)
{
	*this = (*this * b);
}

inline void Quat::operator*=(const Scalar& s)
{
	base = _mm_mul_ps(base, _mm_shuffle_ps(s.base, s.base, _MM_SHUFFLE(0, 0, 0, 0)));
}

inline const Quat operator-(const Quat& a)
{
	return Quat(_mm_sub_ps(_mm_setzero_ps(), a.base));
}

inline const Quat operator+(const Quat& a, const Quat& b)
{
	return Quat(_mm_add_ps(a.base, b.base));
}

inline const Quat operator-(const Quat& a, const Quat& b)
{
	return Quat(_mm_sub_ps(a.base, b.base));
}

inline const Quat operator*(const Quat& a, const Quat& b)
{
	// TODO: not happy with this
	Vector3 va(a.base);
	Vector3 vb(b.base);
	Scalar wa(va.GetW());
	Scalar wb(vb.GetW());
	return Quat(Vector4(Cross(va, vb) + (va * wb) + (vb * wa), (wa * wb) - Dot(va, vb)));
}

inline const Quat operator*(const Quat& a, const Scalar& s)
{
	return Quat(_mm_mul_ps(a.base, _mm_shuffle_ps(s.base, s.base, _MM_SHUFFLE(0, 0, 0, 0))));
}

inline const Quat operator*(const Scalar& s, const Quat& a)
{
	return Quat(_mm_mul_ps(a.base, _mm_shuffle_ps(s.base, s.base, _MM_SHUFFLE(0, 0, 0, 0))));
}

inline const Quat Inv(const Quat& a)
{
	return Quat(_mm_mul_ps(a.base, Vector4Base::Consts::kNeg111_1));
}

inline const Scalar Dot(const Quat& a, const Quat& b)
{
	return Scalar(Vector4Base::Dot4(a.base, b.base));
}

inline const Scalar Length(const Quat& a)
{
	return RsqrtNr(Dot(a, a));
}

inline const Quat Normalize(const Quat& a)
{
	return a * RsqrtNr(Dot(a, a)); 
}

inline const Scalar LengthFast(const Quat& a)
{
	return Rsqrt(Dot(a, a));
}

inline const Quat NormalizeFast(const Quat& a)
{
	return a * Rsqrt(Dot(a, a)); 
}

inline const Quat Lerp(const Quat& a, const Quat& b, const Scalar& t)
{
	Quat e;

	// go the shortest route
	if (IsNegative(Dot(a, b)))
		e = -b;
	else
		e = b;

	return Normalize(a + (e - a) * t);
}

inline const Vector3 Quat::Rotate(const Vector3& v) const
{
	return Vector3(*this * Quat(v) * Inv(*this));
}

inline void Quat::GetAngleAxis(Vector3& axis, Scalar& angle) const
{
	float cosa = GetW();

	angle = acosf(cosa);
	angle += angle;		// * 2;

	float sina = sqrtf(1.0f - cosa * cosa);

	//    if ( fabs( sina ) < 0.0005 ) sina = 1;

	axis = RcpNr(sina) * Vector3(*this);
}
