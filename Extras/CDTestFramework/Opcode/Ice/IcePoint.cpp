/*
 *	ICE / OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains code for 3D vectors.
 *	\file		IcePoint.cpp
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	3D point.
 *
 *	The name is "Point" instead of "Vector" since a vector is N-dimensional, whereas a point is an implicit "vector of dimension 3".
 *	So the choice was between "Point" and "Vector3", the first one looked better (IMHO).
 *
 *	Some people, then, use a typedef to handle both points & vectors using the same class: typedef Point Vector3;
 *	This is bad since it opens the door to a lot of confusion while reading the code. I know it may sounds weird but check this out:
 *
 *	\code
 *		Point P0,P1 = some 3D points;
 *		Point Delta = P1 - P0;
 *	\endcode
 *
 *	This compiles fine, although you should have written:
 *
 *	\code
 *		Point P0,P1 = some 3D points;
 *		Vector3 Delta = P1 - P0;
 *	\endcode
 *
 *	Subtle things like this are not caught at compile-time, and when you find one in the code, you never know whether it's a mistake
 *	from the author or something you don't get.
 *
 *	One way to handle it at compile-time would be to use different classes for Point & Vector3, only overloading operator "-" for vectors.
 *	But then, you get a lot of redundant code in thoses classes, and basically it's really a lot of useless work.
 *
 *	Another way would be to use homogeneous points: w=1 for points, w=0 for vectors. That's why the HPoint class exists. Now, to store
 *	your model's vertices and in most cases, you really want to use Points to save ram.
 *
 *	\class		Point
 *	\author		Pierre Terdiman
 *	\version	1.0
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Creates a positive unit random vector.
 *	\return		Self-reference
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Point& Point::PositiveUnitRandomVector()
{
	x = UnitRandomFloat();
	y = UnitRandomFloat();
	z = UnitRandomFloat();
	Normalize();
	return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Creates a unit random vector.
 *	\return		Self-reference
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Point& Point::UnitRandomVector()
{
	x = UnitRandomFloat() - 0.5f;
	y = UnitRandomFloat() - 0.5f;
	z = UnitRandomFloat() - 0.5f;
	Normalize();
	return *this;
}

// Cast operator
// WARNING: not inlined
Point::operator HPoint() const	{ return HPoint(x, y, z, 0.0f); }

Point& Point::Refract(const Point& eye, const Point& n, float refractindex, Point& refracted)
{
	//	Point EyePt = eye position
	//	Point p = current vertex
	//	Point n = vertex normal
	//	Point rv = refracted vector
	//	Eye vector - doesn't need to be normalized
	Point Env;
	Env.x = eye.x - x;
	Env.y = eye.y - y;
	Env.z = eye.z - z;

	float NDotE = n|Env;
	float NDotN = n|n;
	NDotE /= refractindex;

	// Refracted vector
	refracted = n*NDotE - Env*NDotN;

	return *this;
}

Point& Point::ProjectToPlane(const Plane& p)
{
	*this-= (p.d + (*this|p.n))*p.n;
	return *this;
}

void Point::ProjectToScreen(float halfrenderwidth, float halfrenderheight, const Matrix4x4& mat, HPoint& projected) const
{
	projected = HPoint(x, y, z, 1.0f) * mat;
	projected.w = 1.0f / projected.w;

	projected.x*=projected.w;
	projected.y*=projected.w;
	projected.z*=projected.w;

	projected.x *= halfrenderwidth;		projected.x += halfrenderwidth;
	projected.y *= -halfrenderheight;	projected.y += halfrenderheight;
}

void Point::SetNotUsed()
{
	// We use a particular integer pattern : 0xffffffff everywhere. This is a NAN.
	IR(x) = 0xffffffff;
	IR(y) = 0xffffffff;
	IR(z) = 0xffffffff;
}

BOOL Point::IsNotUsed()	const
{
	if(IR(x)!=0xffffffff)	return FALSE;
	if(IR(y)!=0xffffffff)	return FALSE;
	if(IR(z)!=0xffffffff)	return FALSE;
	return TRUE;
}

Point& Point::Mult(const Matrix3x3& mat, const Point& a)
{
	x = a.x * mat.m[0][0] + a.y * mat.m[0][1] + a.z * mat.m[0][2];
	y = a.x * mat.m[1][0] + a.y * mat.m[1][1] + a.z * mat.m[1][2];
	z = a.x * mat.m[2][0] + a.y * mat.m[2][1] + a.z * mat.m[2][2];
	return *this;
}

Point& Point::Mult2(const Matrix3x3& mat1, const Point& a1, const Matrix3x3& mat2, const Point& a2)
{
	x = a1.x * mat1.m[0][0] + a1.y * mat1.m[0][1] + a1.z * mat1.m[0][2] + a2.x * mat2.m[0][0] + a2.y * mat2.m[0][1] + a2.z * mat2.m[0][2];
	y = a1.x * mat1.m[1][0] + a1.y * mat1.m[1][1] + a1.z * mat1.m[1][2] + a2.x * mat2.m[1][0] + a2.y * mat2.m[1][1] + a2.z * mat2.m[1][2];
	z = a1.x * mat1.m[2][0] + a1.y * mat1.m[2][1] + a1.z * mat1.m[2][2] + a2.x * mat2.m[2][0] + a2.y * mat2.m[2][1] + a2.z * mat2.m[2][2];
	return *this;
}

Point& Point::Mac(const Matrix3x3& mat, const Point& a)
{
	x += a.x * mat.m[0][0] + a.y * mat.m[0][1] + a.z * mat.m[0][2];
	y += a.x * mat.m[1][0] + a.y * mat.m[1][1] + a.z * mat.m[1][2];
	z += a.x * mat.m[2][0] + a.y * mat.m[2][1] + a.z * mat.m[2][2];
	return *this;
}

Point& Point::TransMult(const Matrix3x3& mat, const Point& a)
{
	x = a.x * mat.m[0][0] + a.y * mat.m[1][0] + a.z * mat.m[2][0];
	y = a.x * mat.m[0][1] + a.y * mat.m[1][1] + a.z * mat.m[2][1];
	z = a.x * mat.m[0][2] + a.y * mat.m[1][2] + a.z * mat.m[2][2];
	return *this;
}

Point& Point::Transform(const Point& r, const Matrix3x3& rotpos, const Point& linpos)
{
	x = r.x * rotpos.m[0][0] + r.y * rotpos.m[0][1] + r.z * rotpos.m[0][2] + linpos.x;
	y = r.x * rotpos.m[1][0] + r.y * rotpos.m[1][1] + r.z * rotpos.m[1][2] + linpos.y;
	z = r.x * rotpos.m[2][0] + r.y * rotpos.m[2][1] + r.z * rotpos.m[2][2] + linpos.z;
	return *this;
}

Point& Point::InvTransform(const Point& r, const Matrix3x3& rotpos, const Point& linpos)
{
	float sx = r.x - linpos.x;
	float sy = r.y - linpos.y;
	float sz = r.z - linpos.z;
	x = sx * rotpos.m[0][0] + sy * rotpos.m[1][0] + sz * rotpos.m[2][0];
	y = sx * rotpos.m[0][1] + sy * rotpos.m[1][1] + sz * rotpos.m[2][1];
	z = sx * rotpos.m[0][2] + sy * rotpos.m[1][2] + sz * rotpos.m[2][2];
	return *this;
}
