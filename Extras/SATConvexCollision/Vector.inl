// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
//
// Vector.inl
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
// Vector3

inline Vector3::Vector3()
{
}

inline Vector3::Vector3(const Vector3& v)
{
	base = v.base;
}

inline Vector3::Vector3(float x, float y, float z)
{
	base = _mm_setr_ps(x, y, z, z);
}

inline Vector3::Vector3(const Scalar& x, const Scalar& y, const Scalar& z)
{
	__m128 xy = _mm_unpacklo_ps(x.base, y.base);
	base = _mm_shuffle_ps(xy, z.base, _MM_SHUFFLE(1, 0, 1, 0));
}

inline Vector3::Vector3(const Maths::ZeroTag&)
{
	base = _mm_setzero_ps();
}

inline Vector3::Vector3(const Maths::UnitXTag&)
{
	base = Vector4Base::Consts::k1000;
}

inline Vector3::Vector3(const Maths::UnitYTag&)
{
	base = Vector4Base::Consts::k0100;
}

inline Vector3::Vector3(const Maths::UnitZTag&)
{
	base = Vector4Base::Consts::k0010;
}

inline Vector3::Vector3(const Maths::UnitNegXTag&)
{
	base = Vector4Base::Consts::kNeg1000;
}

inline Vector3::Vector3(const Maths::UnitNegYTag&)
{
	base = Vector4Base::Consts::kNeg0100;
}

inline Vector3::Vector3(const Maths::UnitNegZTag&)
{
	base = Vector4Base::Consts::kNeg0010;
}

inline Vector3::Vector3(const Vector4Base& v)
{
	base = v.base;
}

inline Vector3::Vector3(const __m128 b)
{
	base = b;
}

inline Vector3::Vector3(const Scalar& v)
{
	base = v.base;
}

inline Vector3::Vector3(const Point3& v)
{
	base = v.base;
}

inline Vector3::Vector3(const Vector4& v)
{
	base = v.base;
}

inline Vector3::Vector3(const float* p)
{
	base = _mm_load_ps(p);
}

inline const Vector3& Vector3::operator=(const Vector3& v)
{
	base = v.base;
	return *this;
}

inline const Vector3& Vector3::operator=(const Maths::ZeroTag&)
{
	base = _mm_setzero_ps();
	return *this;
}

inline const Vector3& Vector3::operator=(const Maths::UnitXTag&)
{
	base = Vector4Base::Consts::k1000;
	return *this;
}

inline const Vector3& Vector3::operator=(const Maths::UnitYTag&)
{
	base = Vector4Base::Consts::k0100;
	return *this;
}

inline const Vector3& Vector3::operator=(const Maths::UnitZTag&)
{
	base = Vector4Base::Consts::k0010;
	return *this;
}

inline const Vector3& Vector3::operator=(const Maths::UnitNegXTag&)
{
	base = Vector4Base::Consts::kNeg1000;
	return *this;
}

inline const Vector3& Vector3::operator=(const Maths::UnitNegYTag&)
{
	base = Vector4Base::Consts::kNeg0100;
	return *this;
}

inline const Vector3& Vector3::operator=(const Maths::UnitNegZTag&)
{
	base = Vector4Base::Consts::kNeg0010;
	return *this;
}

inline void Vector3::operator+=(const Vector3& b)
{
	base = _mm_add_ps(base, b.base);
}

inline void Vector3::operator-=(const Vector3& b)
{
	base = _mm_sub_ps(base, b.base);
}

inline void Vector3::operator*=(const Vector3& b)
{
	base = _mm_mul_ps(base, b.base);
}

inline void Vector3::operator/=(const Vector3& b)
{
	base = _mm_div_ps(base, b.base);
}

inline void Vector3::operator*=(const Scalar& s)
{
	base = _mm_mul_ps(base, s.base);
}

inline void Vector3::operator/=(const Scalar& s)
{
	base = _mm_div_ps(base, s.base);
}

inline const Vector3 operator-(const Vector3& a)
{
	return Vector3(_mm_sub_ps(_mm_setzero_ps(), a.base));
}

inline const Vector3 operator+(const Vector3& a, const Vector3& b)
{
	return Vector3(_mm_add_ps(a.base, b.base));
}

inline const Vector3 operator-(const Vector3& a, const Vector3& b)
{
	return Vector3(_mm_sub_ps(a.base, b.base));
}

inline const Vector3 operator*(const Vector3& a, const Vector3& b)
{
	return Vector3(_mm_mul_ps(a.base, b.base));
}

inline const Vector3 operator/(const Vector3& a, const Vector3& b)
{
	return Vector3(_mm_div_ps(a.base, b.base));
}

inline const Vector3 operator*(const Vector3& a, const Scalar& s)
{
	return Vector3(_mm_mul_ps(a.base, s.base));
}

inline const Vector3 operator*(const Scalar& s,  const Vector3& a)
{
	return Vector3(_mm_mul_ps(a.base, s.base));
}

inline const Vector3 operator/(const Vector3& a, const Scalar& s)
{
	return Vector3(_mm_div_ps(a.base, s.base));
}


inline const Vector3 SplatX(const Vector3& v)
{
	return Vector3(_mm_shuffle_ps(v.base, v.base, _MM_SHUFFLE(3, 0, 0, 0)));
}

inline const Vector3 SplatY(const Vector3& v)
{
	return Vector3(_mm_shuffle_ps(v.base, v.base, _MM_SHUFFLE(3, 1, 1, 1)));
}

inline const Vector3 SplatZ(const Vector3& v)
{
	return Vector3(_mm_shuffle_ps(v.base, v.base, _MM_SHUFFLE(3, 2, 2, 2)));
}

inline const Vector3 Abs(const Vector3& a)
{
	return Vector3(_mm_and_ps(a.base, Vector4Base::Consts::kMaskAbs));
}

inline const Vector3 Rcp(const Vector3& a)
{
	return Vector3(_mm_rcp_ps(a.base));
}

inline const Vector3 Rsqrt(const Vector3& a)
{
	return Vector3(_mm_rsqrt_ps(a.base));
}

inline const Vector3 Sqrt(const Vector3& a)
{
	return Vector3(_mm_sqrt_ps(a.base));
}

inline const Vector3 RcpNr(const Vector3& a)
{
	Vector3 rcp = Rcp(a);
	return (rcp + rcp) - (a * rcp * rcp);
}

inline const Vector3 RsqrtNr(const Vector3& a)
{
	Vector3 rcp = Rsqrt(a);
	return (Vector3(Vector4Base::Consts::kHalf) * rcp) * (Vector3(Vector4Base::Consts::kThree) - (a * rcp) * rcp);
}

inline const Scalar Sum(const Vector3& a)
{
	return Scalar(Vector4Base::Sum3(a.base));
}

inline const Scalar Dot(const Vector3& a, const Vector3& b)
{
	return Scalar(Vector4Base::Dot3(a.base, b.base));
}

inline const Vector3 Cross(const Vector3& a, const Vector3& b)
{
	__m128 va = a.base;
	__m128 vb = b.base;
	__m128 v0yzx = _mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 0, 2, 1));
	__m128 v1zxy = _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 1, 0, 2));
	__m128 v0zxy = _mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 1, 0, 2));
	__m128 v1yzx = _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 0, 2, 1));
	return Vector3(_mm_sub_ps(_mm_mul_ps(v0yzx, v1zxy), _mm_mul_ps(v0zxy, v1yzx)));
}

inline const Vector3 Min(const Vector3& a, const Vector3& b)
{
	return Vector3(_mm_min_ps(a.base, b.base));
}

inline const Vector3 Max(const Vector3& a, const Vector3& b)
{
	return Vector3(_mm_max_ps(a.base, b.base));
}

inline const Vector3 Clamp(const Vector3& v, const Vector3& min, const Vector3& max)
{
	return Vector3(_mm_min_ps(max.base, _mm_max_ps(min.base, v.base)));
}

inline const Scalar MinComp(const Vector3& a)
{
	return Scalar(Vector4Base::MinComp3(a.base));
}

inline const Scalar MaxComp(const Vector3& a)
{
	return Scalar(Vector4Base::MaxComp3(a.base));
}

inline const int MinCompIndex(const Vector3& a)
{
	int index = 0;

	if (a[2] < a[1])
	{
		if (a[2] < a[0])
			index = 2;
	}
	else if (a[1] < a[0])
		index = 1;

	return index;
}

inline const int MaxCompIndex(const Vector3& a)
{
	int index = 0;

	if (a[2] > a[1])
	{
		if (a[2] > a[0])
			index = 2;
	}
	else if (a[1] > a[0])
		index = 1;

	return index;
}


inline const Scalar Length(const Vector3& a)
{
	return Sqrt(Dot(a, a));
}

inline const Scalar LengthSqr(const Vector3& a)
{
	return Dot(a, a);
}

inline const Scalar LengthRcp(const Vector3& a)
{
	return RsqrtNr(Dot(a, a));
}

inline const Vector3 Normalize(const Vector3& a)
{
	return RsqrtNr(Dot(a, a)) * a;
}

inline const Vector3 Lerp(const Vector3& a, const Vector3& b, const Scalar& t)
{
	return a + (b - a) * t;
}

inline const Vector3 Perp(const Vector3& v)
{
	Vector3 vtmp;
	if ((float)Abs(v[1]) < 0.707f)
		vtmp = Vector3(0.0f, 1.0f, 0.0f);
	else
		vtmp = Vector3(1.0f, 0.0f, 0.0f);
	return Cross(v, vtmp);
}


inline const Scalar LengthFast(const Vector3& a)
{
	return Rcp(Rsqrt(Dot(a, a)));
}

inline const Scalar LengthRcpFast(const Vector3& a)
{
	return Rsqrt(Dot(a, a));
}

inline const Vector3 NormalizeFast(const Vector3& a)
{
	return Rsqrt(Dot(a, a)) * a;
}

// comparisons return 1 bit per element
inline int operator==(const Vector3& a, const Vector3& b)
{
	return (_mm_movemask_ps(_mm_cmpeq_ps(a.base, b.base)) & 7);
}

inline int operator!=(const Vector3& a, const Vector3& b)
{
	return (_mm_movemask_ps(_mm_cmpneq_ps(a.base, b.base)) & 7);
}

inline int operator<(const Vector3& a, const Vector3& b)
{
	return (_mm_movemask_ps(_mm_cmplt_ps(a.base, b.base)) & 7);
}

inline int operator<=(const Vector3& a, const Vector3& b)
{
	return (_mm_movemask_ps(_mm_cmple_ps(a.base, b.base)) & 7);
}

inline int operator>(const Vector3& a, const Vector3& b)
{
	return (_mm_movemask_ps(_mm_cmpgt_ps(a.base, b.base)) & 7);
}

inline int operator>=(const Vector3& a, const Vector3& b)
{
	return (_mm_movemask_ps(_mm_cmpge_ps(a.base, b.base)) & 7);
}

////////////////////////////////////////////////////////////////////////////////
// Point3

inline Point3::Point3()
{
}

inline Point3::Point3(const Point3& v)
{
	base = v.base;
}

inline Point3::Point3(float x, float y, float z)
{
	base = _mm_setr_ps(x, y, z, 1.f);
}

inline Point3::Point3(const Scalar& x, const Scalar& y, const Scalar& z)
{
	__m128 xy = _mm_unpacklo_ps(x.base, y.base);
	base = _mm_shuffle_ps(xy, z.base, _MM_SHUFFLE(1, 0, 1, 0));
}

inline Point3::Point3(const Maths::ZeroTag&)
{
	base = _mm_setzero_ps();
}

inline Point3::Point3(const Maths::UnitXTag&)
{
	base = Vector4Base::Consts::k1000;
}

inline Point3::Point3(const Maths::UnitYTag&)
{
	base = Vector4Base::Consts::k0100;
}

inline Point3::Point3(const Maths::UnitZTag&)
{
	base = Vector4Base::Consts::k0010;
}

inline Point3::Point3(const Maths::UnitNegXTag&)
{
	base = Vector4Base::Consts::kNeg1000;
}

inline Point3::Point3(const Maths::UnitNegYTag&)
{
	base = Vector4Base::Consts::kNeg0100;
}

inline Point3::Point3(const Maths::UnitNegZTag&)
{
	base = Vector4Base::Consts::kNeg0010;
}

inline Point3::Point3(const Vector4Base& v)
{
	base = v.base;
}

inline Point3::Point3(const __m128 b)
{
	base = b;
}

inline Point3::Point3(const Scalar& v)
{
	base = v.base;
}

inline Point3::Point3(const Vector3& v)
{
	base = v.base;
}

inline Point3::Point3(const Vector4& v)
{
	base = v.base;
}

inline Point3::Point3(const float* p)
{
	base = _mm_load_ps(p);
}

inline const Point3& Point3::operator=(const Point3& v)
{
	base = v.base;
	return *this;
}

inline const Point3& Point3::operator=(const Maths::ZeroTag&)
{
	base = _mm_setzero_ps();
	return *this;
}

inline const Point3& Point3::operator=(const Maths::UnitXTag&)
{
	base = Vector4Base::Consts::k1000;
	return *this;
}

inline const Point3& Point3::operator=(const Maths::UnitYTag&)
{
	base = Vector4Base::Consts::k0100;
	return *this;
}

inline const Point3& Point3::operator=(const Maths::UnitZTag&)
{
	base = Vector4Base::Consts::k0010;
	return *this;
}

inline const Point3& Point3::operator=(const Maths::UnitNegXTag&)
{
	base = Vector4Base::Consts::kNeg1000;
	return *this;
}

inline const Point3& Point3::operator=(const Maths::UnitNegYTag&)
{
	base = Vector4Base::Consts::kNeg0100;
	return *this;
}

inline const Point3& Point3::operator=(const Maths::UnitNegZTag&)
{
	base = Vector4Base::Consts::kNeg0010;
	return *this;
}

inline void Point3::operator+=(const Vector3& b)
{
	base = _mm_add_ps(base, b.base);
}

inline void Point3::operator-=(const Vector3& b)
{
	base = _mm_sub_ps(base, b.base);
}

inline const Point3 operator-(const Point3& a)
{
	return Point3(_mm_sub_ps(_mm_setzero_ps(), a.base));
}

inline const Point3 operator+(const Point3& a, const Vector3& b)
{
	return Point3(_mm_add_ps(a.base, b.base));
}

inline const Point3 operator+(const Vector3& a, const Point3& b)
{
	return Point3(_mm_add_ps(a.base, b.base));
}

inline const Point3 operator-(const Point3& a, const Vector3& b)
{
	return Point3(_mm_sub_ps(a.base, b.base));
}

inline const Vector3 operator-(const Point3& a, const Point3& b)
{
	return Vector3(_mm_sub_ps(a.base, b.base));
}

inline const Vector3 operator*(const Point3& a, const Vector3& b)
{
	return Vector3(_mm_mul_ps(a.base, b.base));
}

inline const Vector3 operator*(const Vector3& a, const Point3& b)
{
	return Vector3(_mm_mul_ps(a.base, b.base));
}

inline const Point3 SplatX(const Point3& v)
{
	return Point3(_mm_shuffle_ps(v.base, v.base, _MM_SHUFFLE(3, 0, 0, 0)));
}

inline const Point3 SplatY(const Point3& v)
{
	return Point3(_mm_shuffle_ps(v.base, v.base, _MM_SHUFFLE(3, 1, 1, 1)));
}

inline const Point3 SplatZ(const Point3& v)
{
	return Point3(_mm_shuffle_ps(v.base, v.base, _MM_SHUFFLE(3, 2, 2, 2)));
}

inline const Point3 Abs(const Point3& a)
{
	return Point3(_mm_and_ps(a.base, Vector4Base::Consts::kMaskAbs));
}

inline const Scalar Sum(const Point3& a)
{
	return Scalar(Vector4Base::Sum3(a.base));
}

inline const Scalar Dot(const Point3& a, const Point3& b)
{
	return Scalar(Vector4Base::Dot3(a.base, b.base));
}

inline const Scalar Dot(const Vector3& a, const Point3& b)
{
	return Scalar(Vector4Base::Dot3(a.base, b.base));
}

inline const Scalar Dot(const Point3& a, const Vector3& b)
{
	return Scalar(Vector4Base::Dot3(a.base, b.base));
}

inline const Point3 Min(const Point3& a, const Point3& b)
{
	return Point3(_mm_min_ps(a.base, b.base));
}

inline const Point3 Max(const Point3& a, const Point3& b)
{
	return Point3(_mm_max_ps(a.base, b.base));
}

inline const Point3 Clamp(const Point3& v, const Point3& min, const Point3& max)
{
	return Point3(_mm_min_ps(max.base, _mm_max_ps(min.base, v.base)));
}

inline const Scalar Dist(const Point3& a, const Point3& b)
{
	return Length(a - b);
}

inline const Scalar DistSqr(const Point3& a, const Point3& b)
{
	return LengthSqr(a - b);
}

inline const Scalar DistRcp(const Point3& a, const Point3& b)
{
	return LengthRcp(a - b);
}

inline const Scalar DistFast(const Point3& a, const Point3& b)
{
	return LengthFast(a - b);
}

inline const Scalar DistRcpFast(const Point3& a, const Point3& b)
{
	return LengthRcpFast(a - b);
}

inline const Point3 Lerp(const Point3& a, const Point3& b, const Scalar& t)
{
	return a + (b - a) * t;
}

inline const Point3 Homogenize(const Vector4& v)
{
	return Point3(v * RcpNr(v.GetW()));
}

inline const Point3 HomogenizeFast(const Vector4& v)
{
	return Point3(v * Rcp(v.GetW()));
}

// comparisons return 1 bit per element
inline int operator==(const Point3& a, const Point3& b)
{
	return (_mm_movemask_ps(_mm_cmpeq_ps(a.base, b.base)) & 7);
}

inline int operator!=(const Point3& a, const Point3& b)
{
	return (_mm_movemask_ps(_mm_cmpneq_ps(a.base, b.base)) & 7);
}

inline int operator<(const Point3& a, const Point3& b)
{
	return (_mm_movemask_ps(_mm_cmplt_ps(a.base, b.base)) & 7);
}

inline int operator<=(const Point3& a, const Point3& b)
{
	return (_mm_movemask_ps(_mm_cmple_ps(a.base, b.base)) & 7);
}

inline int operator>(const Point3& a, const Point3& b)
{
	return (_mm_movemask_ps(_mm_cmpgt_ps(a.base, b.base)) & 7);
}

inline int operator>=(const Point3& a, const Point3& b)
{
	return (_mm_movemask_ps(_mm_cmpge_ps(a.base, b.base)) & 7);
}


////////////////////////////////////////////////////////////////////////////////
// Vector4

inline Vector4::Vector4()
{
}

inline Vector4::Vector4(const Vector4& v)
{
	base = v.base;
}

inline Vector4::Vector4(float x, float y, float z, float w)
{
	base = _mm_setr_ps(x, y, z, w);
}

inline Vector4::Vector4(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w)
{
	Set(x.base, y.base, z.base, w.base);
}

inline Vector4::Vector4(const Point3& v)
{
	Set(v.base, Vector4Base::Consts::k1000);
}

inline Vector4::Vector4(const Vector3& v)
{
	Set(v.base, _mm_setzero_ps());
}

inline Vector4::Vector4(const Vector3& xyz, const Scalar& w)
{
	Set(xyz.base, w.base);
}

inline Vector4::Vector4(const Maths::ZeroTag&)
{
	base = _mm_setzero_ps();
}

inline Vector4::Vector4(const Maths::UnitXTag&)
{
	base = Vector4Base::Consts::k1000;
}

inline Vector4::Vector4(const Maths::UnitYTag&)
{
	base = Vector4Base::Consts::k0100;
}

inline Vector4::Vector4(const Maths::UnitZTag&)
{
	base = Vector4Base::Consts::k0010;
}

inline Vector4::Vector4(const Maths::UnitWTag&)
{
	base = Vector4Base::Consts::k0001;
}

inline Vector4::Vector4(const Maths::UnitNegXTag&)
{
	base = Vector4Base::Consts::kNeg1000;
}

inline Vector4::Vector4(const Maths::UnitNegYTag&)
{
	base = Vector4Base::Consts::kNeg0100;
}

inline Vector4::Vector4(const Maths::UnitNegZTag&)
{
	base = Vector4Base::Consts::kNeg0010;
}

inline Vector4::Vector4(const Maths::UnitNegWTag&)
{
	base = Vector4Base::Consts::kNeg0001;
}

inline Vector4::Vector4(const Vector4Base& v)
{
	base = v.base;
}

inline Vector4::Vector4(const __m128 b)
{
	base = b;
}

inline Vector4::Vector4(const Scalar& v)
{
	base = v.base;
}

inline Vector4::Vector4(const float* p)
{
	base = _mm_load_ps(p);
}

inline const Vector4& Vector4::operator=(const Vector4& v)
{
	base = v.base;
	return *this;
}

inline const Vector4& Vector4::operator=(const Maths::ZeroTag&)
{
	base = _mm_setzero_ps();
	return *this;
}

inline const Vector4& Vector4::operator=(const Maths::UnitXTag&)
{
	base = Vector4Base::Consts::k1000;
	return *this;
}

inline const Vector4& Vector4::operator=(const Maths::UnitYTag&)
{
	base = Vector4Base::Consts::k0100;
	return *this;
}

inline const Vector4& Vector4::operator=(const Maths::UnitZTag&)
{
	base = Vector4Base::Consts::k0010;
	return *this;
}

inline const Vector4& Vector4::operator=(const Maths::UnitWTag&)
{
	base = Vector4Base::Consts::k0001;
	return *this;
}

inline const Vector4& Vector4::operator=(const Maths::UnitNegXTag&)
{
	base = Vector4Base::Consts::kNeg1000;
	return *this;
}

inline const Vector4& Vector4::operator=(const Maths::UnitNegYTag&)
{
	base = Vector4Base::Consts::kNeg0100;
	return *this;
}

inline const Vector4& Vector4::operator=(const Maths::UnitNegZTag&)
{
	base = Vector4Base::Consts::kNeg0010;
	return *this;
}

inline const Vector4& Vector4::operator=(const Maths::UnitNegWTag&)
{
	base = Vector4Base::Consts::kNeg0001;
	return *this;
}

inline void Vector4::operator+=(const Vector4& b)
{
	base = _mm_add_ps(base, b.base);
}

inline void Vector4::operator-=(const Vector4& b)
{
	base = _mm_sub_ps(base, b.base);
}

inline void Vector4::operator*=(const Vector4& b)
{
	base = _mm_mul_ps(base, b.base);
}

inline void Vector4::operator/=(const Vector4& b)
{
	base = _mm_div_ps(base, b.base);
}

inline void Vector4::operator*=(const Scalar& s)
{
	base = _mm_mul_ps(base, s.base);
}

inline void Vector4::operator/=(const Scalar& s)
{
	base = _mm_div_ps(base, s.base);
}

inline const Vector4 operator-(const Vector4& a)
{
	return Vector4(_mm_sub_ps(_mm_setzero_ps(), a.base));
}

inline const Vector4 operator+(const Vector4& a, const Vector4& b)
{
	return Vector4(_mm_add_ps(a.base, b.base));
}

inline const Vector4 operator-(const Vector4& a, const Vector4& b)
{
	return Vector4(_mm_sub_ps(a.base, b.base));
}

inline const Vector4 operator*(const Vector4& a, const Vector4& b)
{
	return Vector4(_mm_mul_ps(a.base, b.base));
}

inline const Vector4 operator/(const Vector4& a, const Vector4& b)
{
	return Vector4(_mm_div_ps(a.base, b.base));
}

inline const Vector4 operator*(const Vector4& a, const Scalar& s)
{
	return Vector4(_mm_mul_ps(a.base, s.base));
}

inline const Vector4 operator*(const Scalar& s,  const Vector4& a)
{
	return Vector4(_mm_mul_ps(a.base, s.base));
}

inline const Vector4 operator/(const Vector4& a, const Scalar& s)
{
	return Vector4(_mm_div_ps(a.base, s.base));
}

inline const Vector4 Abs(const Vector4& a)
{
	return Vector4(_mm_and_ps(a.base, Vector4Base::Consts::kMaskAbs));
}

inline const Vector4 Rcp(const Vector4& a)
{
	return Vector4(_mm_rcp_ps(a.base));
}

inline const Vector4 Rsqrt(const Vector4& a)
{
	return Vector4(_mm_rsqrt_ps(a.base));
}

inline const Vector4 Sqrt(const Vector4& a)
{
	return Vector4(_mm_sqrt_ps(a.base));
}

inline const Vector4 RcpNr(const Vector4& a)
{
	Vector4 rcp = Rcp(a);
	return (rcp + rcp) - (a * rcp * rcp);
}

inline const Vector4 RsqrtNr(const Vector4& a)
{
	Vector4 rcp = Rsqrt(a);
	return (Vector4(Vector4Base::Consts::kHalf) * rcp) * (Vector4(Vector4Base::Consts::kThree) - (a * rcp) * rcp);
}

inline const Scalar Sum(const Vector4& a)
{
	return Scalar(Vector4Base::Sum4(a.base));
}

inline const Scalar Dot(const Vector4& a, const Vector4& b)
{
	return Scalar(Vector4Base::Dot4(a.base, b.base));
}

inline const Vector4 Min(const Vector4& a, const Vector4& b)
{
	return Vector4(_mm_min_ps(a.base, b.base));
}

inline const Vector4 Max(const Vector4& a, const Vector4& b)
{
	return Vector4(_mm_max_ps(a.base, b.base));
}

inline const Vector4 Clamp(const Vector4& v, const Vector4& min, const Vector4& max)
{
	return Vector4(_mm_min_ps(max.base, _mm_max_ps(min.base, v.base)));
}

inline const Scalar MinComp(const Vector4& a)
{
	return Scalar(Vector4Base::MinComp4(a.base));
}

inline const Scalar MaxComp(const Vector4& a)
{
	return Scalar(Vector4Base::MaxComp4(a.base));
}

inline const Scalar Length(const Vector4& a)
{
	return Sqrt(Dot(a, a));
}

inline const Scalar LengthSqr(const Vector4& a)
{
	return Dot(a, a);
}

inline const Scalar LengthRcp(const Vector4& a)
{
	return RsqrtNr(Dot(a, a));
}

inline const Vector4 Normalize(const Vector4& a)
{
	return RsqrtNr(Dot(a, a)) * a;
}

inline const Vector4 Lerp(const Vector4& a, const Vector4& b, const Scalar& t)
{
	return a + (b - a) * t;
}

inline const Scalar LengthFast(const Vector4& a)
{
	return Rcp(Rsqrt(Dot(a, a)));
}

inline const Scalar LengthRcpFast(const Vector4& a)
{
	return Rsqrt(Dot(a, a));
}

inline const Vector4 NormalizeFast(const Vector4& a)
{
	return Rsqrt(Dot(a, a)) * a;
}

// comparisons return 1 bit per element
inline int operator==(const Vector4& a, const Vector4& b)
{
	return _mm_movemask_ps(_mm_cmpeq_ps(a.base, b.base));
}

inline int operator!=(const Vector4& a, const Vector4& b)
{
	return _mm_movemask_ps(_mm_cmpneq_ps(a.base, b.base));
}

inline int operator<(const Vector4& a, const Vector4& b)
{
	return _mm_movemask_ps(_mm_cmplt_ps(a.base, b.base));
}

inline int operator<=(const Vector4& a, const Vector4& b)
{
	return _mm_movemask_ps(_mm_cmple_ps(a.base, b.base));
}

inline int operator>(const Vector4& a, const Vector4& b)
{
	return _mm_movemask_ps(_mm_cmpgt_ps(a.base, b.base));
}

inline int operator>=(const Vector4& a, const Vector4& b)
{
	return _mm_movemask_ps(_mm_cmpge_ps(a.base, b.base));
}
