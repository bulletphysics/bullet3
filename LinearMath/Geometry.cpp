// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Geometry.cpp
//
// Copyright (c) 2006 Simon Hobbs
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.


///for now this is windows only, Intel SSE SIMD intrinsics
#ifdef WIN32
#if _MSC_VER >= 1310


#include "Geometry.h"
#include "Maths.h"
#include <assert.h>


////////////////////////////////////////////////////////////////////////////////
// Line



////////////////////////////////////////////////////////////////////////////////
// Ray


// returns false if the lines are parallel
// t1 and t2 are set to the times of the nearest points on each line
bool Intersect(const Line& la, const Line& lb, float& ta, float& tb)
{
	Vector3 ea = la.m_end - la.m_start;
	Vector3 eb = lb.m_end - lb.m_start;
	Vector3 u = la.m_start - lb.m_start;

	float a = Dot(ea, ea);
	float b = Dot(ea, eb);
	float c = Dot(eb, eb);
	float d = Dot(ea, u);
	float e = Dot(eb, u);

	float det = (a * c - b * b);

	if (Abs(det) < 0.001f)
		return false;

	float invDet = RcpNr(det);
	ta = (b * e - c * d) * invDet;
	tb = (a * e - b * d) * invDet;

	return true;
}

bool IntersectSegments(const Line& la, const Line& lb, float& ta, float& tb)
{
	Vector3 ea = la.m_end - la.m_start;
	Vector3 eb = lb.m_end - lb.m_start;
	Vector3 u = la.m_start - lb.m_start;

	float a = Dot(ea, ea);
	float b = Dot(ea, eb);
	float c = Dot(eb, eb);
	float d = Dot(ea, u);
	float e = Dot(eb, u);

	float det = (a * c - b * b);

	if (Abs(det) < 0.001f)
		return false;

	float numa = (b * e - c * d);
	float numb = (a * e - b * d);

	// clip a
	float dena = det, denb = det;
	if (numa < 0.0f)
	{
		numa = 0.0f;
		numb = e;
		denb = c;
	}
	else if (numa > det)
	{
		numa = det;
		numb = e + b;
		denb = c;
	}
	else
		denb = det;

	// clip b
	if (numb < 0.0f)
	{
		numb = 0.0f;
		if (-d < 0.0f)
		{
			numa = 0.0f;
		}
		else if (-d > a)
		{
			numa = dena;
		}
		else
		{
			numa = -d;
			dena = a;
		}
	}
	else if (numb > denb)
	{
		numb = denb;
		if ((-d + b) < 0.0f)
		{
			numa = 0.0f;
		}
		else if ((-d + b) > a)
		{
			numa = dena;
		}
		else
		{
			numa = -d + b;
			dena = a;
		}
	}

	// compute the times
	ta = numa / dena;
	tb = numb / denb;

	return true;
}

// returns intersection of 2 rays or nearest point to it
// t1 and t2 are set to the times of the nearest points on each ray (not clamped to ray though)
// asserts if rays are parallel
bool Intersect(const Ray& ra, const Ray& rb, float& ta, float& tb)
{
	Vector3 u = ra.m_start - rb.m_start;

	Scalar a = Dot(ra.m_dir, ra.m_dir);
	Scalar b = Dot(ra.m_dir, rb.m_dir);
	Scalar c = Dot(rb.m_dir, rb.m_dir);
	Scalar d = Dot(ra.m_dir, u);
	Scalar e = Dot(rb.m_dir, u);

	Scalar det = (a * c - b * b);

	if (Abs(det) < 0.001f)
		return false;

	Scalar invDet = RcpNr(det);
	ta = (b * e - c * d) * invDet;
	tb = (a * e - b * d) * invDet;

	return true;
}


////////////////////////////////////////////////////////////////////////////////
// Plane
bool Plane::IsFinite() const
{
	if (IsNan(GetX()) || IsNan(GetY()) || IsNan(GetZ()) || IsNan(GetW()))
		return false;
	return true;
}

////////////////////////////////////////////////////////////////////////////////
// Bounds3 - axis aligned bounding box

Bounds3::OriginTag Bounds3::Origin;
Bounds3::EmptyTag Bounds3::Empty;

bool Bounds3::Intersect(const Ray& ray, float& tnear, float& tfar) const
{
	Vector3 rcpDir = RcpNr(ray.m_dir);

	Vector3 v1 = (m_min - ray.m_start) * rcpDir;
	Vector3 v2 = (m_max - ray.m_start) * rcpDir;

	Vector3 vmin = Min(v1, v2);
	Vector3 vmax = Max(v1, v2);

	Scalar snear = MaxComp(vmin);

	// handle ray being parallel to any axis
	// (most rays don't need this)
	if (IsNan(snear))
	{
		int inside = (ray.m_start >= m_min) & (ray.m_start <= m_max);

		for (int i = 0; i < 3; i++)
		{
			if (IsNan(rcpDir.Get(i)))
			{
				if ((inside & (1 << i)) == 0)
					return false;
				vmin.Set(i, Scalar::Consts::MinValue);
				vmax.Set(i, Scalar::Consts::MaxValue);
			}
		}

		snear = MaxComp(vmin);
	}

	tnear = snear;
	tfar = MinComp(vmax);

	if (tnear > tfar)
		return false;

	if (tfar < 0.0f)
		return false;

	return true;
}


////////////////////////////////////////////////////////////////////////////////
// OrientedBounds3 - oriented bounding box

#endif
#endif //WIN32
