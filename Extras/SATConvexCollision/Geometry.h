// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Geometry.h
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

#ifndef BULLET_MATH_GEOMETRY_H
#define BULLET_MATH_GEOMETRY_H


#ifdef WIN32

#include "Vector.h"
#include "Matrix.h"

class Matrix44;
class Transform;


////////////////////////////////////////////////////////////////////////////////
// Line
class Line
{
public:
	Point3 m_start;
	Point3 m_end;

	Line();
	Line(const Point3& start, const Point3& end);

	// returns false if the lines are parallel
	friend bool Intersect(const Line& la, const Line& lb, float& ta, float& tb);
	friend bool IntersectSegments(const Line& la, const Line& lb, float& ta, float& tb);

	// get projection vector between a point and a line
	// (i.e. if you add the vector to the point, then the new point will lie on the line)
	friend Vector3 GetProjectionVector(const Line& ln, const Point3& pt);

	// get distance from point to line (and time along line)
	friend float Distance(const Line& ln, const Point3& pt, float& t);
};


////////////////////////////////////////////////////////////////////////////////
// Ray
class Ray
{
public:
	Point3 m_start;
	Vector3 m_dir;

	Ray();
	Ray(const Point3& start, const Vector3& dir);

	explicit Ray(const Line& line);

	// returns false if the rays are parallel
	friend bool Intersect(const Ray& ra, const Ray& rb, float& ta, float& tb);
};


////////////////////////////////////////////////////////////////////////////////
// Plane
class Plane : public Vector4Base
{
public:
	// constructors
	Plane();
	Plane(const Plane& p);
	Plane(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w);
	Plane(const Vector3& xyz, const Scalar& w);
	Plane(const Point3& a, const Point3& b, const Point3& c);
	Plane(const Vector3& normal, const Point3& pt);

	// construction to constant
	Plane(const Maths::ZeroTag&);
	Plane(const Maths::UnitXTag&);
	Plane(const Maths::UnitYTag&);
	Plane(const Maths::UnitZTag&);
	Plane(const Maths::UnitNegXTag&);
	Plane(const Maths::UnitNegYTag&);
	Plane(const Maths::UnitNegZTag&);

	// explicit constructors
	explicit Plane(const __m128 b);
	explicit Plane(const Vector3& v);
	explicit Plane(const Vector4& v);
	explicit Plane(const float* p);

	// assignment
	const Plane& operator=(const Plane& v);
	const Plane& operator=(const Maths::ZeroTag&);
	const Plane& operator=(const Maths::UnitXTag&);
	const Plane& operator=(const Maths::UnitYTag&);
	const Plane& operator=(const Maths::UnitZTag&);
	const Plane& operator=(const Maths::UnitNegXTag&);
	const Plane& operator=(const Maths::UnitNegYTag&);
	const Plane& operator=(const Maths::UnitNegZTag&);

	// element access
	const Vector3 GetNormal() const;
	const Scalar getDistance() const;

	// transformations
	friend const Plane operator-(const Plane& p);
	friend const Plane operator*(const Plane& p, const Transform& m);

	// operations
	friend const Scalar Dot(const Plane& p, const Point3& v);
	friend const Scalar Dot(const Point3& v, const Plane& p);

	friend const Scalar Dot(const Plane& p, const Vector3& v);
	friend const Scalar Dot(const Vector3& v, const Plane& p);

	friend const Scalar Dot(const Plane& p, const Vector4& v);
	friend const Scalar Dot(const Vector4& v, const Plane& p);

	friend const Scalar Intersect(const Plane& p, const Ray& ray);
	friend const Scalar Intersect(const Plane& p, const Line& line);

	// validation
	bool IsFinite() const;
};


////////////////////////////////////////////////////////////////////////////////
// Bounds3 - axis aligned bounding box
class Bounds3
{
public:
	Point3 m_min, m_max;

	static const enum OriginTag { } Origin;
	static const enum EmptyTag { } Empty;

	// constructors
	Bounds3();
	Bounds3(const Bounds3& aabb);
	Bounds3(const Point3& min, const Point3& max);

	// construction to constant
	Bounds3(const OriginTag&);
	Bounds3(const EmptyTag&);

	// explicit constructors
	explicit Bounds3(const Point3& minMax);

	// assignment
	const Bounds3& operator=(const Bounds3& aabb);
	const Bounds3& operator=(const Point3& pt);

	const Bounds3& operator=(const OriginTag&);
	const Bounds3& operator=(const EmptyTag&);
	
	// in place operations
	void operator+=(const Point3& pt);
	void operator+=(const Bounds3& aabb);

	// operations
	friend Bounds3 operator+(const Bounds3& aabb, const Point3& pt);
	friend Bounds3 operator+(const Point3& pt, const Bounds3& aabb);
	friend Bounds3 operator+(const Bounds3& aabb, const Bounds3& aabb2);

	bool Contains(const Point3& pt) const;
	bool Contains(const Bounds3& aabb) const;
	bool Touches(const Bounds3& aabb) const;

	bool Intersect(const Ray& ray, float& tnear, float& tfar) const;
	bool Intersect(const Line& line, float& tnear, float& tfar) const;

	Point3 GetCenter() const;
	Vector3 GetExtent() const;
	Vector3 GetSize() const;

	// validation
	bool IsFinite() const;
	bool HasVolume() const;
};


#include "Geometry.inl"

#endif //WIN32
#endif //BULLET_MATH_GEOMETRY_H
