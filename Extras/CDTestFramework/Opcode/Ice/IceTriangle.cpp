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
 *	Contains a handy triangle class.
 *	\file		IceTriangle.cpp
 *	\author		Pierre Terdiman
 *	\date		January, 17, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains a triangle class.
 *
 *	\class		Tri
 *	\author		Pierre Terdiman
 *	\version	1.0
 *	\date		08.15.98
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static sdword VPlaneSideEps(const Point& v, const Plane& plane, float epsilon)
{
	// Compute distance from current vertex to the plane
	float Dist = plane.Distance(v);
	// Compute side:
	// 1	= the vertex is on the positive side of the plane
	// -1	= the vertex is on the negative side of the plane
	// 0	= the vertex is on the plane (within epsilon)
	return Dist > epsilon ? 1 : Dist < -epsilon ? -1 : 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Flips the winding order.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Triangle::Flip()
{
	Point Tmp = mVerts[1];
	mVerts[1] = mVerts[2];
	mVerts[2] = Tmp;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the triangle area.
 *	\return		the area
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float Triangle::Area() const
{
	const Point& p0 = mVerts[0];
	const Point& p1 = mVerts[1];
	const Point& p2 = mVerts[2];
	return ((p0 - p1)^(p0 - p2)).Magnitude() * 0.5f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the triangle perimeter.
 *	\return		the perimeter
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float Triangle::Perimeter()	const
{
	const Point& p0 = mVerts[0];
	const Point& p1 = mVerts[1];
	const Point& p2 = mVerts[2];
	return		p0.Distance(p1)
			+	p0.Distance(p2)
			+	p1.Distance(p2);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the triangle compacity.
 *	\return		the compacity
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float Triangle::Compacity() const
{
	float P = Perimeter();
	if(P==0.0f)	return 0.0f;
	return (4.0f*PI*Area()/(P*P));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the triangle normal.
 *	\param		normal	[out] the computed normal
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Triangle::Normal(Point& normal) const
{
	const Point& p0 = mVerts[0];
	const Point& p1 = mVerts[1];
	const Point& p2 = mVerts[2];
	normal = ((p0 - p1)^(p0 - p2)).Normalize();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the triangle denormalized normal.
 *	\param		normal	[out] the computed normal
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Triangle::DenormalizedNormal(Point& normal) const
{
	const Point& p0 = mVerts[0];
	const Point& p1 = mVerts[1];
	const Point& p2 = mVerts[2];
	normal = ((p0 - p1)^(p0 - p2));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the triangle center.
 *	\param		center	[out] the computed center
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Triangle::Center(Point& center) const
{
	const Point& p0 = mVerts[0];
	const Point& p1 = mVerts[1];
	const Point& p2 = mVerts[2];
	center = (p0 + p1 + p2)*INV3;
}

PartVal Triangle::TestAgainstPlane(const Plane& plane, float epsilon) const
{
	bool Pos = false, Neg = false;

	// Loop through all vertices
	for(udword i=0;i<3;i++)
	{
		// Compute side:
		sdword Side = VPlaneSideEps(mVerts[i], plane, epsilon);

				if (Side < 0)	Neg = true;
		else	if (Side > 0)	Pos = true;
	}

			if (!Pos && !Neg)	return TRI_ON_PLANE;
	else	if (Pos && Neg)		return TRI_INTERSECT;
	else	if (Pos && !Neg)	return TRI_PLUS_SPACE;
	else	if (!Pos && Neg)	return TRI_MINUS_SPACE;

	// What?!
	return TRI_FORCEDWORD;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the triangle moment.
 *	\param		m	[out] the moment
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void Triangle::ComputeMoment(Moment& m)
{
	// Compute the area of the triangle
	m.mArea = Area();

	// Compute the centroid
	Center(m.mCentroid);

	// Second-order components. Handle zero-area faces.
	Point& p = mVerts[0];
	Point& q = mVerts[1];
	Point& r = mVerts[2];
	if(m.mArea==0.0f)
	{
		// This triangle has zero area. The second order components would be eliminated with the usual formula, so, for the 
		// sake of robustness we use an alternative form. These are the centroid and second-order components of the triangle's vertices.
		m.mCovariance.m[0][0] = (p.x*p.x + q.x*q.x + r.x*r.x);
		m.mCovariance.m[0][1] = (p.x*p.y + q.x*q.y + r.x*r.y);
		m.mCovariance.m[0][2] = (p.x*p.z + q.x*q.z + r.x*r.z);
		m.mCovariance.m[1][1] = (p.y*p.y + q.y*q.y + r.y*r.y);
		m.mCovariance.m[1][2] = (p.y*p.z + q.y*q.z + r.y*r.z);
		m.mCovariance.m[2][2] = (p.z*p.z + q.z*q.z + r.z*r.z);      
		m.mCovariance.m[2][1] = m.mCovariance.m[1][2];
		m.mCovariance.m[1][0] = m.mCovariance.m[0][1];
		m.mCovariance.m[2][0] = m.mCovariance.m[0][2];
	}
	else
	{
		const float OneOverTwelve = 1.0f / 12.0f;
		m.mCovariance.m[0][0] = m.mArea * (9.0f * m.mCentroid.x*m.mCentroid.x + p.x*p.x + q.x*q.x + r.x*r.x) * OneOverTwelve;
		m.mCovariance.m[0][1] = m.mArea * (9.0f * m.mCentroid.x*m.mCentroid.y + p.x*p.y + q.x*q.y + r.x*r.y) * OneOverTwelve;
		m.mCovariance.m[1][1] = m.mArea * (9.0f * m.mCentroid.y*m.mCentroid.y + p.y*p.y + q.y*q.y + r.y*r.y) * OneOverTwelve;
		m.mCovariance.m[0][2] = m.mArea * (9.0f * m.mCentroid.x*m.mCentroid.z + p.x*p.z + q.x*q.z + r.x*r.z) * OneOverTwelve;
		m.mCovariance.m[1][2] = m.mArea * (9.0f * m.mCentroid.y*m.mCentroid.z + p.y*p.z + q.y*q.z + r.y*r.z) * OneOverTwelve;
		m.mCovariance.m[2][2] = m.mArea * (9.0f * m.mCentroid.z*m.mCentroid.z + p.z*p.z + q.z*q.z + r.z*r.z) * OneOverTwelve;
		m.mCovariance.m[2][1] = m.mCovariance.m[1][2];
		m.mCovariance.m[1][0] = m.mCovariance.m[0][1];
		m.mCovariance.m[2][0] = m.mCovariance.m[0][2];
	}
}
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the triangle's smallest edge length.
 *	\return		the smallest edge length
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float Triangle::MinEdgeLength()	const
{
	float Min = MAX_FLOAT;
	float Length01 = mVerts[0].Distance(mVerts[1]);
	float Length02 = mVerts[0].Distance(mVerts[2]);
	float Length12 = mVerts[1].Distance(mVerts[2]);
	if(Length01 < Min)	Min = Length01;
	if(Length02 < Min)	Min = Length02;
	if(Length12 < Min)	Min = Length12;
	return Min;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the triangle's largest edge length.
 *	\return		the largest edge length
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float Triangle::MaxEdgeLength()	const
{
	float Max = MIN_FLOAT;
	float Length01 = mVerts[0].Distance(mVerts[1]);
	float Length02 = mVerts[0].Distance(mVerts[2]);
	float Length12 = mVerts[1].Distance(mVerts[2]);
	if(Length01 > Max)	Max = Length01;
	if(Length02 > Max)	Max = Length02;
	if(Length12 > Max)	Max = Length12;
	return Max;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes a point on the triangle according to the stabbing information.
 *	\param		u,v			[in] point's barycentric coordinates
 *	\param		pt			[out] point on triangle
 *	\param		nearvtx		[out] index of nearest vertex
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Triangle::ComputePoint(float u, float v, Point& pt, udword* nearvtx)	const
{
	// Compute point coordinates
	pt = (1.0f - u - v)*mVerts[0] + u*mVerts[1] + v*mVerts[2];

	// Compute nearest vertex if needed
	if(nearvtx)
	{
		// Compute distance vector
		Point d(mVerts[0].SquareDistance(pt),	// Distance^2 from vertex 0 to point on the face
				mVerts[1].SquareDistance(pt),	// Distance^2 from vertex 1 to point on the face
				mVerts[2].SquareDistance(pt));	// Distance^2 from vertex 2 to point on the face

		// Get smallest distance
		*nearvtx = d.SmallestAxis();
	}
}

void Triangle::Inflate(float fat_coeff, bool constant_border)
{
	// Compute triangle center
	Point TriangleCenter;
	Center(TriangleCenter);

	// Don't normalize?
	// Normalize => add a constant border, regardless of triangle size
	// Don't => add more to big triangles
	for(udword i=0;i<3;i++)
	{
		Point v = mVerts[i] - TriangleCenter;

		if(constant_border)	v.Normalize();

		mVerts[i] += v * fat_coeff;
	}
}
