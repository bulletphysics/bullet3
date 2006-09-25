// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
//
// Scalar.inl
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


inline Scalar::Scalar()
{
}

inline Scalar::Scalar(float f)
{
	base = _mm_set1_ps(f);
}


inline Scalar::Scalar(int i, bool forceNoConvert)
{
	__declspec(align(16)) int iv[4] = {i, i, i, i};
	*(Scalar*)&base = *(Scalar*)&iv;
}

inline Scalar::Scalar(__m128 s)
{
	base = s;
}

inline Scalar::Scalar(int i)
{
	base = _mm_cvtsi32_ss(base, i);
	base = _mm_shuffle_ps(base, base, _MM_SHUFFLE(0, 0, 0, 0));
}

inline const Scalar& Scalar::operator=(const Scalar &a)
{
	base = a.base;
	return *this;
}

inline Scalar::operator const float() const
{
	float f;
	_mm_store_ss(&f, base);
	return f;
}

inline Scalar::operator const float()
{
	float f;
	_mm_store_ss(&f, base);
	return f;
}

inline void Scalar::operator+=(const Scalar& b)
{
	base = _mm_add_ps(base, b.base);
}

inline void Scalar::operator-=(const Scalar& b)
{
	base = _mm_sub_ps(base, b.base);
}

inline void Scalar::operator*=(const Scalar& b)
{
	base = _mm_mul_ps(base, b.base);
}

inline void Scalar::operator/=(const Scalar& b)
{
	base = _mm_div_ps(base, b.base);
}

inline const Scalar operator-(const Scalar& a)
{
	return Scalar(_mm_sub_ps(_mm_setzero_ps(), a.base));
}

inline const Scalar Abs(const Scalar& a)
{
	return Scalar(_mm_and_ps(a.base, Scalar::Consts::AbsMask.base));
}

inline const Scalar Rcp(const Scalar& a)
{
	return Scalar(_mm_rcp_ps(a.base));
}

inline const Scalar Rsqrt(const Scalar& a)
{
	return Scalar(_mm_rsqrt_ps(a.base));
}

inline const Scalar Sqrt(const Scalar& a)
{
	return Scalar(_mm_sqrt_ps(a.base));
}

// Newton Raphson Reciprocal
// (2 * Rcp(x)) - (x * Rcp(x) * Rcp(x))]
inline const Scalar RcpNr(const Scalar& a)
{
	Scalar rcp = Rcp(a);
	return (rcp + rcp) - (a * rcp * rcp);
}

// Newton Raphson Reciprocal Square Root
// 0.5 * Rsqrt * (3 - x * Rsqrt(x) * Rsqrt(x))
inline const Scalar RsqrtNr(const Scalar& a)
{
	Scalar rcp = Rsqrt(a);
	return (Scalar::Consts::Half * rcp) * (Scalar::Consts::Three - (a * rcp) * rcp);
}

// binary
inline const Scalar operator+(const Scalar& a, const Scalar& b)
{
	return Scalar(_mm_add_ps(a.base, b.base));
}

inline const Scalar operator-(const Scalar& a, const Scalar& b)
{
	return Scalar(_mm_sub_ps(a.base, b.base));
}

inline const Scalar operator*(const Scalar& a, const Scalar& b)
{
	return Scalar(_mm_mul_ps(a.base, b.base));
}

inline const Scalar operator/(const Scalar& a, const Scalar& b)
{
	return Scalar(_mm_div_ps(a.base, b.base));
}

inline const Scalar Min(const Scalar& a, const Scalar& b)
{
	return Scalar(_mm_min_ps(a.base, b.base));
}

inline const Scalar Max(const Scalar& a, const Scalar& b)
{
	return Scalar(_mm_max_ps(a.base, b.base));
}

inline const Scalar Clamp(const Scalar& a, const Scalar& min, const Scalar& max)
{
	return Scalar(_mm_min_ps(max.base, _mm_max_ps(min.base, a.base)));
}

inline const Scalar Lerp(const Scalar& a, const Scalar& b, const Scalar& t)
{
	return Scalar(a + (b - a) * t);
}

inline const int IsNegative(const Scalar& a)
{
	return _mm_movemask_ps(a.base) & 1;
}

// warning. this only checks for quiet nan
inline const int IsNan(const Scalar& a)
{
	int aInt = *(int*)&a;
	return ((aInt & 0x7fc00000) == 0x7fc00000);
}

inline const int IsInfinity(const Scalar& a)
{
	return (a == Scalar::Consts::PosInfinity || a == Scalar::Consts::NegInfinity);
}

inline const int IsPosInfinity(const Scalar& a)
{
	return (a == Scalar::Consts::PosInfinity);
}

inline const int IsNegInfinity(const Scalar& a)
{
	return (a == Scalar::Consts::NegInfinity);
}

