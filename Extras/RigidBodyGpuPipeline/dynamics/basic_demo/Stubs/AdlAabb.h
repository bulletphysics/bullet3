/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada

#ifndef AABB_H
#define AABB_H

#include "Stubs/AdlMath.h"
#include "Stubs/AdlQuaternion.h"

enum AdlCollisionShapeTypes
{
	ADL_SHAPE_SPHERE=2,
	ADL_SHAPE_HEIGHT_FIELD,
	SHAPE_CONVEX_HEIGHT_FIELD,
};

_MEM_CLASSALIGN16
struct Aabb
{
	public:
		_MEM_ALIGNED_ALLOCATOR16;

		__inline
		void setEmpty();
		__inline
		void includeVolume( const Aabb& aabb );
		__inline
		void includePoint( const float4& p );
		__inline
		bool overlaps( const float4& p ) const;
		__inline
		bool overlaps( const Aabb& aabb ) const;
		__inline
		float4 center() const;
		__inline
		int getMajorAxis() const;
		__inline
		float4 getExtent() const;
		__inline
		void expandBy( const float4& r );

		__inline
		static bool overlaps( const Aabb& a, const Aabb& b );

		__inline
		bool intersect(const float4* from, const float4* to, const float4* invRay) const;

		__inline
		void transform(const float4& translation, const Quaternion& quat);

		__inline
		void transform(const float4& translation, const Matrix3x3& rot);

	public:
		float4 m_max;
		float4 m_min;
};

void Aabb::setEmpty()
{
	m_max = make_float4( -FLT_MAX );
	m_min = make_float4( FLT_MAX );
}

void Aabb::includeVolume(const Aabb& aabb)
{
	m_max.x = max2( m_max.x, aabb.m_max.x );
	m_min.x = min2( m_min.x, aabb.m_min.x );

	m_max.y = max2( m_max.y, aabb.m_max.y );
	m_min.y = min2( m_min.y, aabb.m_min.y );

	m_max.z = max2( m_max.z, aabb.m_max.z );
	m_min.z = min2( m_min.z, aabb.m_min.z );
}

void Aabb::includePoint( const float4& p )
{
	m_max.x = max2( m_max.x, p.x );
	m_min.x = min2( m_min.x, p.x );

	m_max.y = max2( m_max.y, p.y );
	m_min.y = min2( m_min.y, p.y );

	m_max.z = max2( m_max.z, p.z );
	m_min.z = min2( m_min.z, p.z );
}

bool Aabb::overlaps( const float4& p ) const
{
	float4 dx = m_max-p;
	float4 dm = p-m_min;

	return (dx.x >= 0 && dx.y >= 0 && dx.z >= 0)
		&& (dm.x >= 0 && dm.y >= 0 && dm.z >= 0);
}

bool Aabb::overlaps( const Aabb& in ) const
{
/*
	if( m_max.x < in.m_min.x || m_min.x > in.m_max.x ) return false;
	if( m_max.y < in.m_min.y || m_min.y > in.m_max.y ) return false;
	if( m_max.z < in.m_min.z || m_min.z > in.m_max.z ) return false;

	return true;
*/
	return overlaps( *this, in );
}

bool Aabb::overlaps( const Aabb& a, const Aabb& b )
{
	if( a.m_max.x < b.m_min.x || a.m_min.x > b.m_max.x ) return false;
	if( a.m_max.y < b.m_min.y || a.m_min.y > b.m_max.y ) return false;
	if( a.m_max.z < b.m_min.z || a.m_min.z > b.m_max.z ) return false;

	return true;
}

float4 Aabb::center() const
{
	return 0.5f*(m_max+m_min);
}

int Aabb::getMajorAxis() const
{
	float4 extent = getExtent();

	int majorAxis = 0;
	if( extent.s[1] > extent.s[0] )
		majorAxis = 1;
	if( extent.s[2] > extent.s[majorAxis] )
		majorAxis = 2;

	return majorAxis;
}

float4 Aabb::getExtent() const
{
	return m_max-m_min;
}

void Aabb::expandBy( const float4& r )
{
	m_max += r;
	m_min -= r;
}

bool Aabb::intersect(const float4* from, const float4* to, const float4* invRay) const
{
	float4 dFar;
	dFar = (m_max - *from);
	dFar *= *invRay;
	float4 dNear;
	dNear = (m_min - *from);
	dNear *= *invRay;
		
	float4 tFar; 
	tFar = max2(dFar, dNear);
	float4 tNear; 
	tNear = min2(dFar, dNear);

	float farf[] = { tFar.x, tFar.y, tFar.z };

	float nearf[] = { tNear.x, tNear.y, tNear.z };

	float minFar = min2(farf[0], min2(farf[1], farf[2]));
	float maxNear = max2(nearf[0], max2(nearf[1], nearf[2]));
	
	minFar = min2(1.0f, minFar );
	maxNear = max2(0.0f, maxNear);
	
	return (minFar >= maxNear);
}

void Aabb::transform(const float4& translation, const Matrix3x3& m)
{
	float4 c = center();

	Aabb& ans = *this;

	float4 e[] = { m.m_row[0]*m_min, m.m_row[1]*m_min, m.m_row[2]*m_min };
	float4 f[] = { m.m_row[0]*m_max, m.m_row[1]*m_max, m.m_row[2]*m_max };
	ans.m_max = ans.m_min = translation;

	{	int j=0;
		float4 mi = make_float4( min2( e[j].x, f[j].x ), min2( e[j].y, f[j].y ), min2( e[j].z, f[j].z ) );
		float4 ma = make_float4( max2( e[j].x, f[j].x ), max2( e[j].y, f[j].y ), max2( e[j].z, f[j].z ) );

		ans.m_min.x += mi.x+mi.y+mi.z;
		ans.m_max.x += ma.x+ma.y+ma.z;
	}

	{	int j=1;
		float4 mi = make_float4( min2( e[j].x, f[j].x ), min2( e[j].y, f[j].y ), min2( e[j].z, f[j].z ) );
		float4 ma = make_float4( max2( e[j].x, f[j].x ), max2( e[j].y, f[j].y ), max2( e[j].z, f[j].z ) );

		ans.m_min.y += mi.x+mi.y+mi.z;
		ans.m_max.y += ma.x+ma.y+ma.z;
	}

	{	int j=2;
		float4 mi = make_float4( min2( e[j].x, f[j].x ), min2( e[j].y, f[j].y ), min2( e[j].z, f[j].z ) );
		float4 ma = make_float4( max2( e[j].x, f[j].x ), max2( e[j].y, f[j].y ), max2( e[j].z, f[j].z ) );

		ans.m_min.z += mi.x+mi.y+mi.z;
		ans.m_max.z += ma.x+ma.y+ma.z;
	}
}

void Aabb::transform(const float4& translation, const Quaternion& quat)
{
	Matrix3x3 m = qtGetRotationMatrix( quat );

	transform( translation, m );
}

#endif

