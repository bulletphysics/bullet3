// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
//
// VectorBase.inl
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


////////////////////////////////////////////////////////////////////////////////
// Vector4Base

inline Vector4Base::Vector4Base()
{
}

inline const float& Vector4Base::operator[](int i) const
{
	return *(((float*)&base) + i);
}

inline float& Vector4Base::operator[](int i)
{
	return *(((float*)&base) + i);
}

inline const Scalar Vector4Base::GetX() const
{
	return Scalar(_mm_shuffle_ps(base, base, _MM_SHUFFLE(0, 0, 0, 0)));
}

inline const Scalar Vector4Base::GetY() const
{
	return Scalar(_mm_shuffle_ps(base, base, _MM_SHUFFLE(1, 1, 1, 1)));
}

inline const Scalar Vector4Base::GetZ() const
{
	return Scalar(_mm_shuffle_ps(base, base, _MM_SHUFFLE(2, 2, 2, 2)));
}

inline const Scalar Vector4Base::GetW() const
{
	return Scalar(_mm_shuffle_ps(base, base, _MM_SHUFFLE(3, 3, 3, 3)));
}

inline const Scalar Vector4Base::Get(int i) const
{
	__m128 res;

	switch (i)
	{
	case 0: res = _mm_shuffle_ps(base, base, _MM_SHUFFLE(0, 0, 0, 0)); break;
	case 1: res = _mm_shuffle_ps(base, base, _MM_SHUFFLE(1, 1, 1, 1)); break;
	case 2: res = _mm_shuffle_ps(base, base, _MM_SHUFFLE(2, 2, 2, 2)); break;
	case 3: res = _mm_shuffle_ps(base, base, _MM_SHUFFLE(3, 3, 3, 3)); break;
	}

	return Scalar(res);
}

inline void Vector4Base::SetX(const Scalar& s)
{
	__m128 xxyy = _mm_shuffle_ps(s.base, base, _MM_SHUFFLE(1, 1, 0, 0));
	base = _mm_shuffle_ps(xxyy, base, _MM_SHUFFLE(3, 2, 2, 0));
}

inline void Vector4Base::SetY(const Scalar& s)
{
	__m128 xxyy = _mm_shuffle_ps(base, s.base, _MM_SHUFFLE(0, 0, 0, 0));
	base = _mm_shuffle_ps(xxyy, base, _MM_SHUFFLE(3, 2, 2, 0));
}

inline void Vector4Base::SetZ(const Scalar& s)
{
	__m128 zzww = _mm_shuffle_ps(s.base, base, _MM_SHUFFLE(3, 3, 0, 0));
	base = _mm_shuffle_ps(base, zzww, _MM_SHUFFLE(2, 0, 1, 0));
}

inline void Vector4Base::SetW(const Scalar& s)
{
	__m128 zzww = _mm_shuffle_ps(base, s.base, _MM_SHUFFLE(0, 0, 2, 2));
	base = _mm_shuffle_ps(base, zzww, _MM_SHUFFLE(2, 0, 1, 0));
}

inline void Vector4Base::Set(int i, const Scalar& s)
{
	switch (i)
	{
	case 0: SetX(s); break;
	case 1: SetY(s); break;
	case 2: SetZ(s); break;
	case 3: SetW(s); break;
	}
}

inline void Vector4Base::LoadUnaligned3(const float* p)
{
	int* dst = (int*)this;
	dst[0] = ((const int*)p)[0];
	dst[1] = ((const int*)p)[1];
	dst[2] = ((const int*)p)[2];
}

inline void Vector4Base::LoadUnaligned4(const float* p)
{
	base = _mm_loadu_ps(p);
}

inline void Vector4Base::StoreUnaligned3(float* p) const
{
	const int* src = (const int*)this;
	((int*)p)[0] = src[0];
	((int*)p)[1] = src[1];
	((int*)p)[2] = src[2];
}

inline  void Vector4Base::StoreUnaligned4(float* p) const
{
	_mm_storeu_ps(p, base);
}

__forceinline void Vector4Base::Set(const __m128& x, const __m128& y, const __m128& z, const __m128& w)
{
	__m128 xy = _mm_unpacklo_ps(x, y);
	__m128 zw = _mm_unpacklo_ps(z, w);
	base = _mm_shuffle_ps(xy, zw, _MM_SHUFFLE(1, 0, 1, 0));
}

__forceinline void Vector4Base::Set(const __m128& xyz, const __m128& w)
{
	base = _mm_shuffle_ps(xyz, xyz, _MM_SHUFFLE(0, 1, 2, 3));
	base = _mm_move_ss(base, w);
	base = _mm_shuffle_ps(base, base, _MM_SHUFFLE(0, 1, 2, 3));
}

__forceinline __m128 Vector4Base::Dot3(const __m128& v0, const __m128& v1)
{
	__m128 a = _mm_mul_ps(v0, v1);
	__m128 b = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0));
	__m128 c = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1));
	__m128 d = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2));
	return _mm_add_ps(b, _mm_add_ps(c, d));
}

__forceinline __m128 Vector4Base::Dot4(const __m128& v0, const __m128& v1)
{
	__m128 a = _mm_mul_ps(v0, v1);
	__m128 b = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 3, 2, 1));
	__m128 c = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 0, 3, 2));
	__m128 d = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 1, 0, 3));
	return _mm_add_ps(a, _mm_add_ps(b, _mm_add_ps(c, d)));
}

__forceinline __m128 Vector4Base::Sum3(const __m128& a)
{
	__m128 b = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0));
	__m128 c = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1));
	__m128 d = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2));
	return _mm_add_ps(b, _mm_add_ps(c, d));
}

__forceinline __m128 Vector4Base::Sum4(const __m128& a)
{
	__m128 b = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 3, 2, 1));
	__m128 c = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 0, 3, 2));
	__m128 d = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 1, 0, 3));
	return _mm_add_ps(a, _mm_add_ps(b, _mm_add_ps(c, d)));
}

__forceinline __m128 Vector4Base::MinComp3(const __m128& a)
{
	__m128 b = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0));
	__m128 c = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1));
	__m128 d = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2));
	return _mm_min_ps(b, _mm_min_ps(c, d));
}

__forceinline __m128 Vector4Base::MinComp4(const __m128& a)
{
	__m128 b = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 3, 2, 1));
	__m128 c = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 0, 3, 2));
	__m128 d = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 1, 0, 3));
	return _mm_min_ps(a, _mm_min_ps(b, _mm_min_ps(c, d)));
}

__forceinline __m128 Vector4Base::MaxComp3(const __m128& a)
{
	__m128 b = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0));
	__m128 c = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1));
	__m128 d = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2));
	return _mm_max_ps(b, _mm_max_ps(c, d));
}

__forceinline __m128 Vector4Base::MaxComp4(const __m128& a)
{
	__m128 b = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 3, 2, 1));
	__m128 c = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 0, 3, 2));
	__m128 d = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 1, 0, 3));
	return _mm_max_ps(a, _mm_max_ps(b, _mm_max_ps(c, d)));
}

