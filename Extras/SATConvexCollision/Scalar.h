// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Scalar.h
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
#ifndef BULLET_SCALAR_H
#define BULLET_SCALAR_H


#include <xmmintrin.h>
#include "Memory2.h"

// resolved overload found with Koenig lookup
#pragma warning (disable : 4675)


__declspec(align(16)) class Scalar
{
public:
	BULLET_ALIGNED_NEW_AND_DELETE

	__m128 base;

	// constants
	struct Consts{
		static const Scalar
			MinusOne, 
			Zero,
			Half,
			One,
			Three,
			MinValue,
			MaxValue,
			Epsilon,
			NegInfinity,
			PosInfinity,
			AbsMask;
	};

	// constructors
	Scalar();
	Scalar(float f);
	Scalar(int i, bool forceNoConvert);

	// explicit constructors
	explicit Scalar(__m128 s);
	explicit Scalar(int i);

	// assignment
	const Scalar& operator=(const Scalar& a);

	// conversion
	operator const float() const;
	operator const float();

	// in place operations
	void operator+=(const Scalar& b);
	void operator-=(const Scalar& b);
	void operator*=(const Scalar& b);
	void operator/=(const Scalar& b);

	// operations
	friend const Scalar operator-(const Scalar& a);

	friend const Scalar operator+(const Scalar& a, const Scalar& b);
	friend const Scalar operator-(const Scalar& a, const Scalar& b);
	friend const Scalar operator*(const Scalar& a, const Scalar& b);
	friend const Scalar operator/(const Scalar& a, const Scalar& b);

	friend const Scalar Abs(const Scalar& a);
	friend const Scalar Rcp(const Scalar& a);
	friend const Scalar Rsqrt(const Scalar& a);
	friend const Scalar Sqrt(const Scalar& a);
	friend const Scalar RcpNr(const Scalar& a);
	friend const Scalar RsqrtNr(const Scalar& a);

	friend const Scalar Min(const Scalar& a, const Scalar& b);
	friend const Scalar Max(const Scalar& a, const Scalar& b);
	friend const Scalar Clamp(const Scalar& a, const Scalar& min, const Scalar& max);

	friend const Scalar Lerp(const Scalar& a, const Scalar& b, const Scalar& t);

	// comparison
	friend const int IsNegative(const Scalar& a);

	friend const int IsNan(const Scalar& a);
	friend const int IsInfinity(const Scalar& a);
	friend const int IsPosInfinity(const Scalar& a);
	friend const int IsNegInfinity(const Scalar& a);
};


#include "Scalar.inl"

#endif //BULLET_SCALAR_H
