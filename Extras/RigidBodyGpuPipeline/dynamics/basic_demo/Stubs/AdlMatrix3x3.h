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


#ifndef MATRIX3X3_H
#define MATRIX3X3_H

#include "AdlMath.h"

///////////////////////////////////////
//	Matrix3x3
///////////////////////////////////////

typedef 
_MEM_CLASSALIGN16 struct
{
	_MEM_ALIGNED_ALLOCATOR16;
	float4 m_row[3];
}Matrix3x3;

__inline
Matrix3x3 mtZero();

__inline
Matrix3x3 mtIdentity();

__inline
Matrix3x3 mtDiagonal(float a, float b, float c);

__inline
Matrix3x3 mtTranspose(const Matrix3x3& m);

__inline
Matrix3x3 mtMul(const Matrix3x3& a, const Matrix3x3& b);

__inline
float4 mtMul1(const Matrix3x3& a, const float4& b);

__inline
Matrix3x3 mtMul2(float a, const Matrix3x3& b);

__inline
float4 mtMul3(const float4& b, const Matrix3x3& a);

__inline
Matrix3x3 mtInvert(const Matrix3x3& m);

__inline
Matrix3x3 mtZero()
{
	Matrix3x3 m;
	m.m_row[0] = make_float4(0.f);
	m.m_row[1] = make_float4(0.f);
	m.m_row[2] = make_float4(0.f);
	return m;
}

__inline
Matrix3x3 mtIdentity()
{
	Matrix3x3 m;
	m.m_row[0] = make_float4(1,0,0);
	m.m_row[1] = make_float4(0,1,0);
	m.m_row[2] = make_float4(0,0,1);
	return m;
}

__inline
Matrix3x3 mtDiagonal(float a, float b, float c)
{
	Matrix3x3 m;
	m.m_row[0] = make_float4(a,0,0);
	m.m_row[1] = make_float4(0,b,0);
	m.m_row[2] = make_float4(0,0,c);
	return m;
}

__inline
Matrix3x3 mtTranspose(const Matrix3x3& m)
{
	Matrix3x3 out;
	out.m_row[0] = make_float4(m.m_row[0].s[0], m.m_row[1].s[0], m.m_row[2].s[0], 0.f);
	out.m_row[1] = make_float4(m.m_row[0].s[1], m.m_row[1].s[1], m.m_row[2].s[1], 0.f);
	out.m_row[2] = make_float4(m.m_row[0].s[2], m.m_row[1].s[2], m.m_row[2].s[2], 0.f);
	return out;
}

__inline
Matrix3x3 mtMul(const Matrix3x3& a, const Matrix3x3& b)
{
	Matrix3x3 transB;
	transB = mtTranspose( b );
	Matrix3x3 ans;
	for(int i=0; i<3; i++)
	{
		ans.m_row[i].s[0] = dot3F4(a.m_row[i],transB.m_row[0]);
		ans.m_row[i].s[1] = dot3F4(a.m_row[i],transB.m_row[1]);
		ans.m_row[i].s[2] = dot3F4(a.m_row[i],transB.m_row[2]);
	}
	return ans;
}

__inline
float4 mtMul1(const Matrix3x3& a, const float4& b)
{
	float4 ans;
	ans.s[0] = dot3F4( a.m_row[0], b );
	ans.s[1] = dot3F4( a.m_row[1], b );
	ans.s[2] = dot3F4( a.m_row[2], b );
	return ans;
}

__inline
Matrix3x3 mtMul2(float a, const Matrix3x3& b)
{
	Matrix3x3 ans;
	ans.m_row[0] = a*b.m_row[0];
	ans.m_row[1] = a*b.m_row[1];
	ans.m_row[2] = a*b.m_row[2];
	return ans;
}

__inline
float4 mtMul3(const float4& a, const Matrix3x3& b)
{
	float4 ans;
	ans.x = a.x*b.m_row[0].x + a.y*b.m_row[1].x + a.z*b.m_row[2].x;
	ans.y = a.x*b.m_row[0].y + a.y*b.m_row[1].y + a.z*b.m_row[2].y;
	ans.z = a.x*b.m_row[0].z + a.y*b.m_row[1].z + a.z*b.m_row[2].z;
	return ans;
}

__inline
Matrix3x3 mtInvert(const Matrix3x3& m)
{
	float det = m.m_row[0].s[0]*m.m_row[1].s[1]*m.m_row[2].s[2]+m.m_row[1].s[0]*m.m_row[2].s[1]*m.m_row[0].s[2]+m.m_row[2].s[0]*m.m_row[0].s[1]*m.m_row[1].s[2]
	-m.m_row[0].s[0]*m.m_row[2].s[1]*m.m_row[1].s[2]-m.m_row[2].s[0]*m.m_row[1].s[1]*m.m_row[0].s[2]-m.m_row[1].s[0]*m.m_row[0].s[1]*m.m_row[2].s[2];

	CLASSERT( det );

	Matrix3x3 ans;
	ans.m_row[0].s[0] = m.m_row[1].s[1]*m.m_row[2].s[2] - m.m_row[1].s[2]*m.m_row[2].s[1];
	ans.m_row[0].s[1] = m.m_row[0].s[2]*m.m_row[2].s[1] - m.m_row[0].s[1]*m.m_row[2].s[2];
	ans.m_row[0].s[2] = m.m_row[0].s[1]*m.m_row[1].s[2] - m.m_row[0].s[2]*m.m_row[1].s[1];
	ans.m_row[0].w = 0.f;

	ans.m_row[1].s[0] = m.m_row[1].s[2]*m.m_row[2].s[0] - m.m_row[1].s[0]*m.m_row[2].s[2];
	ans.m_row[1].s[1] = m.m_row[0].s[0]*m.m_row[2].s[2] - m.m_row[0].s[2]*m.m_row[2].s[0];
	ans.m_row[1].s[2] = m.m_row[0].s[2]*m.m_row[1].s[0] - m.m_row[0].s[0]*m.m_row[1].s[2];
	ans.m_row[1].w = 0.f;

	ans.m_row[2].s[0] = m.m_row[1].s[0]*m.m_row[2].s[1] - m.m_row[1].s[1]*m.m_row[2].s[0];
	ans.m_row[2].s[1] = m.m_row[0].s[1]*m.m_row[2].s[0] - m.m_row[0].s[0]*m.m_row[2].s[1];
	ans.m_row[2].s[2] = m.m_row[0].s[0]*m.m_row[1].s[1] - m.m_row[0].s[1]*m.m_row[1].s[0];
	ans.m_row[2].w = 0.f;

	ans = mtMul2((1.0f/det), ans);
	return ans;
}

__inline
Matrix3x3 mtSet( const float4& a, const float4& b, const float4& c )
{
	Matrix3x3 m;
	m.m_row[0] = a;
	m.m_row[1] = b;
	m.m_row[2] = c;
	return m;
}

__inline
Matrix3x3 operator+(const Matrix3x3& a, const Matrix3x3& b)
{
	Matrix3x3 out;
	out.m_row[0] = a.m_row[0] + b.m_row[0];
	out.m_row[1] = a.m_row[1] + b.m_row[1];
	out.m_row[2] = a.m_row[2] + b.m_row[2];
	return out;
}

#endif

