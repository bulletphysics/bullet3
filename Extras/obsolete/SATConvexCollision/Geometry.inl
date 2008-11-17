// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Geometry.inl
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
// Line

inline Line::Line()
{
}

inline Line::Line(const Point3& start, const Point3& end)
{
	m_start = start;
	m_end = end;
}

inline Vector3 GetProjectionVector(const Line& ln, const Point3& pt)
{
	Vector3 d = Normalize(ln.m_end - ln.m_start);
	Vector3 v = pt - ln.m_start;
	Scalar s = Dot(v, d);
	return -(v - d * s);
}

////////////////////////////////////////////////////////////////////////////////
// Ray

inline Ray::Ray()
{
}

inline Ray::Ray(const Point3& start, const Vector3& dir)
{
	m_start = start;
	m_dir = dir;
}

inline Ray::Ray(const Line& line)
{
	m_start = line.m_start;
	m_dir = Normalize(line.m_end - line.m_start);
}


////////////////////////////////////////////////////////////////////////////////
// Plane

inline Plane::Plane()
{
}

inline Plane::Plane(const Plane& p)
{
	base = p.base;
}

inline Plane::Plane(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w)
{
	Set(x.base, y.base, z.base, w.base);
}

inline Plane::Plane(const Vector3& xyz, const Scalar& w)
{
	Set(xyz.base, w.base);
}

inline Plane::Plane(const Point3& a, const Point3& b, const Point3& c)
{
	Vector3 normal = Normalize(Cross(c - a, b - a));
	*this = Plane(normal, -Dot(normal, Vector3(a)));
}

inline Plane::Plane(const Vector3& normal, const Point3& pt)
{
	*this = Plane(normal, -Dot(normal, Vector3(pt)));
}

inline Plane::Plane(const Maths::ZeroTag&)
{
	base = _mm_setzero_ps();
}

inline Plane::Plane(const Maths::UnitXTag&)
{
	base = Vector4Base::Consts::k1000;
}

inline Plane::Plane(const Maths::UnitYTag&)
{
	base = Vector4Base::Consts::k0100;
}

inline Plane::Plane(const Maths::UnitZTag&)
{
	base = Vector4Base::Consts::k0010;
}

inline Plane::Plane(const Maths::UnitNegXTag&)
{
	base = Vector4Base::Consts::kNeg1000;
}

inline Plane::Plane(const Maths::UnitNegYTag&)
{
	base = Vector4Base::Consts::kNeg0100;
}

inline Plane::Plane(const Maths::UnitNegZTag&)
{
	base = Vector4Base::Consts::kNeg0010;
}

inline Plane::Plane(const __m128 b)
{
	base = b;
}

inline Plane::Plane(const Vector3& v)
{
	base = v.base;
}

inline Plane::Plane(const Vector4& v)
{
	base = v.base;
}

inline Plane::Plane(const float* p)
{
	base = _mm_load_ps(p);
}

inline const Plane& Plane::operator=(const Plane& v)
{
	base = v.base;
	return *this;
}

inline const Plane& Plane::operator=(const Maths::ZeroTag&)
{
	base = _mm_setzero_ps();
	return *this;
}

inline const Plane& Plane::operator=(const Maths::UnitXTag&)
{
	base = Vector4Base::Consts::k1000;
	return *this;
}

inline const Plane& Plane::operator=(const Maths::UnitYTag&)
{
	base = Vector4Base::Consts::k0100;
	return *this;
}

inline const Plane& Plane::operator=(const Maths::UnitZTag&)
{
	base = Vector4Base::Consts::k0010;
	return *this;
}

inline const Plane& Plane::operator=(const Maths::UnitNegXTag&)
{
	base = Vector4Base::Consts::kNeg1000;
	return *this;
}

inline const Plane& Plane::operator=(const Maths::UnitNegYTag&)
{
	base = Vector4Base::Consts::kNeg0100;
	return *this;
}

inline const Plane& Plane::operator=(const Maths::UnitNegZTag&)
{
	base = Vector4Base::Consts::kNeg0010;
	return *this;
}

inline const Vector3 Plane::GetNormal() const
{
	return Vector3(base);
}

inline const Scalar Plane::getDistance() const
{
	return GetW();
}

inline const Plane operator-(const Plane& p)
{
	return Plane(_mm_sub_ps(_mm_setzero_ps(), p.base));
}

inline const Plane operator*(const Plane& p, const Transform& m)
{
	Vector3 np = Vector3(p) * m;
	return Plane(np, p.GetW() - Dot(Vector3(np), Vector3(m.GetTranslation())));
}

inline const Scalar Dot(const Plane& p, const Point3& v)
{
	return Scalar(Vector4Base::Dot3(p.base, v.base)) + p.GetW();
}

inline const Scalar Dot(const Point3& v, const Plane& p)
{
	return Scalar(Vector4Base::Dot3(p.base, v.base)) + p.GetW();
}

inline const Scalar Dot(const Plane& p, const Vector3& v)
{
	return Scalar(Vector4Base::Dot3(p.base, v.base));
}

inline const Scalar Dot(const Vector3& v, const Plane& p)
{
	return Scalar(Vector4Base::Dot3(p.base, v.base));
}

inline const Scalar Dot(const Plane& p, const Vector4& v)
{
	return Scalar(Vector4Base::Dot4(p.base, v.base));
}

inline const Scalar Dot(const Vector4& v, const Plane& p)
{
	return Scalar(Vector4Base::Dot4(p.base, v.base));
}

// returns NaN if ray is perpendicular to ray
inline const Scalar Intersect(const Plane& p, const Ray& ray)
{
	Scalar ds = Dot(p, ray.m_start);
	Scalar dd = Dot(p, ray.m_dir);
	return -ds * RcpNr(dd);
}

// returns NaN if line is perpendicular to ray
inline const Scalar Intersect(const Plane& p, const Line& line)
{
	Scalar ds = Dot(p, line.m_start);
	Scalar de = Dot(p, line.m_end);
	return ds * RcpNr(ds - de);
}


////////////////////////////////////////////////////////////////////////////////
// Bounds3 - axis aligned bounding box

inline Bounds3::Bounds3()
{
}

inline Bounds3::Bounds3(const Bounds3& aabb)
{
	*this = aabb;
}

inline Bounds3::Bounds3(const Point3& min, const Point3& max)
{
	m_min = min;
	m_max = max;
}

inline Bounds3::Bounds3(const OriginTag&)
{
	m_min = m_max = Maths::Zero;
}

inline Bounds3::Bounds3(const EmptyTag&)
{
	// max maximal inverted aabb - ready to have points accumulated into it
	m_min = Point3(Scalar::Consts::MaxValue);
	m_max = Point3(Scalar::Consts::MinValue);
}

inline Bounds3::Bounds3(const Point3& minMax)
{
	m_min = m_max = minMax;
}

inline const Bounds3& Bounds3::operator=(const Bounds3& aabb)
{
	m_min = aabb.m_min;
	m_max = aabb.m_max;
	return *this;
}

inline const Bounds3& Bounds3::operator=(const Point3& pt)
{
	m_min = m_max = pt;
	return *this;
}

inline const Bounds3& Bounds3::operator=(const OriginTag&)
{
	m_min = m_max = Maths::Zero;
}

inline const Bounds3& Bounds3::operator=(const EmptyTag&)
{
	// max maximal inverted aabb - ready to have points accumulated into it
	m_min = Point3(Scalar::Consts::MaxValue);
	m_max = Point3(Scalar::Consts::MinValue);
	return *this;
}

inline void Bounds3::operator+=(const Point3& pt)
{
	m_min = Min(m_min, pt);
	m_max = Max(m_max, pt);
}

inline void Bounds3::operator+=(const Bounds3& aabb)
{
	m_min = Min(m_min, aabb.m_min);
	m_max = Max(m_max, aabb.m_max);
}

inline Bounds3 operator+(const Bounds3& aabb, const Point3& pt)
{
	return Bounds3(Min(aabb.m_min, pt), Max(aabb.m_max, pt));
}

inline Bounds3 operator+(const Point3& pt, const Bounds3& aabb)
{
	return Bounds3(Min(aabb.m_min, pt), Max(aabb.m_max, pt));
}

inline Bounds3 operator+(const Bounds3& aabb, const Bounds3& aabb2)
{
	return Bounds3(Min(aabb.m_min, aabb2.m_min), Max(aabb.m_max, aabb2.m_max));
}

inline bool Bounds3::Contains(const Point3& pt) const
{
	return ((pt >= m_min) & (pt <= m_max)) == 7;
}

inline bool Bounds3::Contains(const Bounds3& aabb) const
{
	return ((aabb.m_min >= m_min) & (aabb.m_max <= m_max)) == 7;
}

inline bool Bounds3::Touches(const Bounds3& aabb) const
{
	return ((aabb.m_max >= m_min) & (aabb.m_min <= m_max)) == 7;
}

// returns intersection of 2 lines or nearest point to it
// t1 and t2 are set to the times of the nearest points on each line
// asserts if rays are parallel
inline const Point3 IntersectPrev(const Line& la, const Line& lb, float& ta, float& tb)
{
	Vector3 ea = la.m_end - la.m_start;
	Vector3 eb = lb.m_end - lb.m_start;
	Vector3 u = la.m_start - lb.m_start;

	Scalar a = Dot(ea, ea);
	Scalar b = Dot(ea, eb);
	Scalar c = Dot(eb, eb);
	Scalar d = Dot(ea, u);
	Scalar e = Dot(eb, u);

	Scalar det = (a * c - b * b);

	// assert if rays are parallel
	assert(Abs(det) > Scalar(0.0001f));

	Scalar invDet = RcpNr(det);
	ta = (b * e - c * d) * invDet;
	tb = (a * e - b * d) * invDet;

	return la.m_start + ea * ta;
}

inline const Point3 IntersectPrev(const Line& a, const Line& b)
{
	float ta, tb;
	return IntersectPrev(a, b, ta, tb);
}

inline bool Bounds3::Intersect(const Line& line, float& tnear, float& tfar) const
{
	return Intersect(Ray(line), tnear, tfar);
}

inline Point3 Bounds3::GetCenter() const
{
	return Lerp(m_min, m_max, Scalar::Consts::Half);
}

inline Vector3 Bounds3::GetExtent() const
{
	return (m_max - m_min) * Scalar::Consts::Half;
}

inline Vector3 Bounds3::GetSize() const
{
	return m_max - m_min;
}

inline bool Bounds3::IsFinite() const
{
	return m_min.IsFinite() && m_max.IsFinite();
}

inline bool Bounds3::HasVolume() const
{
	return (((m_min <= m_max) & 7) == 7);
}



////////////////////////////////////////////////////////////////////////////////
// OrientedBounds3 - oriented bounding box


