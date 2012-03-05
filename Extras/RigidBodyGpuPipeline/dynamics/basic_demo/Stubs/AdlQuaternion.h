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


#ifndef QUATERNION_H
#define QUATERNION_H

#include "AdlMatrix3x3.h"


typedef float4 Quaternion;

__inline
Quaternion qtSet(const float4& axis, float angle);

__inline
Quaternion qtMul(const Quaternion& a, const Quaternion& b);

__inline
float4 qtRotate(const Quaternion& q, const float4& vec);

__inline
float4 qtInvRotate(const Quaternion& q, const float4& vec);

__inline
Quaternion qtInvert(const Quaternion& q);

__inline
Matrix3x3 qtGetRotationMatrix(const Quaternion& quat);

__inline
Quaternion qtNormalize(const Quaternion& q);

__inline
Quaternion qtGetIdentity() { return make_float4(0,0,0,1); }

__inline
Quaternion qtSet(const float4& axis, float angle)
{
	float4 nAxis = normalize3( axis );

	Quaternion q;
	q.s[0] = nAxis.s[0]*sin(angle/2);
	q.s[1] = nAxis.s[1]*sin(angle/2);
	q.s[2] = nAxis.s[2]*sin(angle/2);
	q.s[3] = cos(angle/2);
	return q;
}

__inline
Quaternion qtMul(const Quaternion& a, const Quaternion& b)
{
	Quaternion ans;
	ans = cross3( a, b );
	ans += a.s[3]*b + b.s[3]*a;
	ans.s[3] = a.s[3]*b.s[3] - (a.s[0]*b.s[0]+a.s[1]*b.s[1]+a.s[2]*b.s[2]);
	return ans;
}

__inline
float4 qtRotate(const Quaternion& q, const float4& vec)
{
	Quaternion vecQ = vec;
	vecQ.s[3] = 0.f;
	Quaternion qInv = qtInvert( q );
	float4 out = qtMul(qtMul(q,vecQ),qInv);
	return out;
}

__inline
float4 qtInvRotate(const Quaternion& q, const float4& vec)
{
	return qtRotate( qtInvert( q ), vec );
}

__inline
Quaternion qtInvert(const Quaternion& q)
{
	Quaternion ans;
	ans.s[0] = -q.s[0];
	ans.s[1] = -q.s[1];
	ans.s[2] = -q.s[2];
	ans.s[3] = q.s[3];
	return ans;
}

__inline
Matrix3x3 qtGetRotationMatrix(const Quaternion& quat)
{
	float4 quat2 = make_float4(quat.s[0]*quat.s[0], quat.s[1]*quat.s[1], quat.s[2]*quat.s[2], 0.f);
	Matrix3x3 out;

	out.m_row[0].s[0]=1-2*quat2.s[1]-2*quat2.s[2];
	out.m_row[0].s[1]=2*quat.s[0]*quat.s[1]-2*quat.s[3]*quat.s[2];
	out.m_row[0].s[2]=2*quat.s[0]*quat.s[2]+2*quat.s[3]*quat.s[1];
	out.m_row[0].s[3] = 0.f;

	out.m_row[1].s[0]=2*quat.s[0]*quat.s[1]+2*quat.s[3]*quat.s[2];
	out.m_row[1].s[1]=1-2*quat2.s[0]-2*quat2.s[2];
	out.m_row[1].s[2]=2*quat.s[1]*quat.s[2]-2*quat.s[3]*quat.s[0];
	out.m_row[1].s[3] = 0.f;

	out.m_row[2].s[0]=2*quat.s[0]*quat.s[2]-2*quat.s[3]*quat.s[1];
	out.m_row[2].s[1]=2*quat.s[1]*quat.s[2]+2*quat.s[3]*quat.s[0];
	out.m_row[2].s[2]=1-2*quat2.s[0]-2*quat2.s[1];
	out.m_row[2].s[3] = 0.f;

	return out;
}

__inline
Quaternion qtGetQuaternion(const Matrix3x3* m)
{
	Quaternion q;
	q.w = sqrtf( m[0].m_row[0].x + m[0].m_row[1].y + m[0].m_row[2].z + 1 ) * 0.5f;
	float inv4w = 1.f/(4.f*q.w);
	q.x = (m[0].m_row[2].y-m[0].m_row[1].z)*inv4w;
	q.y = (m[0].m_row[0].z-m[0].m_row[2].x)*inv4w;
	q.z = (m[0].m_row[1].x-m[0].m_row[0].y)*inv4w;

	return q;
}

__inline
Quaternion qtNormalize(const Quaternion& q)
{
	return normalize4(q);
}

__inline
float4 transform(const float4& p, const float4& translation, const Quaternion& orientation)
{
	return qtRotate( orientation, p ) + translation;
}

__inline
float4 invTransform(const float4& p, const float4& translation, const Quaternion& orientation)
{
	return qtRotate( qtInvert( orientation ), p-translation ); // use qtInvRotate
}

#endif

