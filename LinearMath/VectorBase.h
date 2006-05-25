// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
//
// VectorBase.h
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
#ifndef BULLET_VECTOR_BASE_H
#define BULLET_VECTOR_BASE_H

#include "Scalar.h"
#include "Memory2.h"

// vector constants
namespace Maths
{
	static const enum ZeroTag { } Zero;
	static const enum UnitXTag { } UnitX;
	static const enum UnitYTag { } UnitY;
	static const enum UnitZTag { } UnitZ;
	static const enum UnitWTag { } UnitW;
	static const enum UnitNegXTag { } UnitNegX;
	static const enum UnitNegYTag { } UnitNegY;
	static const enum UnitNegZTag { } UnitNegZ;
	static const enum UnitNegWTag { } UnitNegW;

	static const enum IdentityTag { } Identity;
	static const enum RotateXTag { } RotateX;
	static const enum RotateYTag { } RotateY;
	static const enum RotateZTag { } RotateZ;
	static const enum ScaleTag { } Scale;
	static const enum SkewTag { } Skew;
};

////////////////////////////////////////////////////////////////////////////////
// Vector4Base
__declspec(align(16)) class Vector4Base
{
public:
	BULLET_ALIGNED_NEW_AND_DELETE

	__m128 base;

//protected:
	// useful constants for internal use
	struct Consts
	{
		static const unsigned int maskAbs;
		static const unsigned int mask1110[4];
		static const unsigned int mask0001[4];

		static const __m128
			kZero,
			kHalf,
			kThree,

			k1000,
			k0100,
			k0010,
			k0001,

			kNeg1000,
			kNeg0100,
			kNeg0010,
			kNeg0001,

			kNeg111_1,

			k1110,
			kMaskAbs,
			kMask1110,
			kMask0001;
	};

	// can't construct a Vector4Base
	Vector4Base();

	// compound operations helpers for use by derived classes
	void Set(const __m128& x, const __m128& y, const __m128& z, const __m128& w);
	void Set(const __m128& xyz, const __m128& w);

	static __m128 Dot3(const __m128& v0, const __m128& v1);
	static __m128 Dot4(const __m128& v0, const __m128& v1);

	static __m128 Sum3(const __m128& a);
	static __m128 Sum4(const __m128& a);

	static __m128 MinComp3(const __m128& a);
	static __m128 MinComp4(const __m128& a);

	static __m128 MaxComp3(const __m128& a);
	static __m128 MaxComp4(const __m128& a);

public:
	// element access
	const float& operator[](int i) const;
	float& operator[](int i);

	// get/set elements
	const Scalar GetX() const;
	const Scalar GetY() const;
	const Scalar GetZ() const;
	const Scalar GetW() const;
	const Scalar Get(int i) const;

	void SetX(const Scalar& s);
	void SetY(const Scalar& s);
	void SetZ(const Scalar& s);
	void SetW(const Scalar& s);
	void Set(int i, const Scalar& s);

	// unaligned load/store
	void LoadUnaligned3(const float* p);
	void LoadUnaligned4(const float* p);

	void StoreUnaligned3(float* p) const;
	void StoreUnaligned4(float* p) const;
};


////////////////////////////////////////////////////////////////////////////////
// Vector2Base
class __declspec(align(8)) Vector2Base
{
public:
	float x, y;
};


#include "VectorBase.inl"

#endif //BULLET_VECTOR_BASE_H
