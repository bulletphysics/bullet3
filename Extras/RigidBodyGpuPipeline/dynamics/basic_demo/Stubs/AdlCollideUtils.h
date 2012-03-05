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


#ifndef COLLIDE_UTILS_H
#define COLLIDE_UTILS_H

#include "Stubs/AdlMath.h"


class CollideUtils
{
	public:
		template<bool FLIPSIGN>
		static bool collide(const float4& a, const float4& b, const float4& c, const float4& p, float4& normalOut, float margin = 0.f);

		__inline
		static float castRay(const float4& v0, const float4& v1, const float4& v2,
			 const float4& rayFrom, const float4& rayTo, float margin = 0.0f, float4* bCrdOut = NULL);

};


template<bool FLIPSIGN>
bool CollideUtils::collide(const float4& a, const float4& b, const float4& c, const float4& p, float4& normalOut, float margin)
{
	float4 ab, bc, ca;
	ab = b-a;
	bc = c-b;
	ca = a-c;

	float4 ap, bp, cp;
	ap = p-a;
	bp = p-b;
	cp = p-c;

	float4 n;
	n = cross3(ab, -1.f*ca);

	float4 abp = cross3( ab, ap );
	float4 bcp = cross3( bc, bp );
	float4 cap = cross3( ca, cp );

	float s0 = dot3F4(n,abp);
	float s1 = dot3F4(n,bcp);
	float s2 = dot3F4(n,cap);

//	if(( s0<0.f && s1<0.f && s2<0.f ) || ( s0>0.f && s1>0.f && s2>0.f ))
	if(( s0<margin && s1<margin && s2<margin ) || ( s0>-margin && s1>-margin && s2>-margin ))
	{
		n = normalize3( n );
		n.w = dot3F4(n,ap);

		normalOut = (FLIPSIGN)? -n : n;
		return true;
	}

	return false;
}

__inline
float CollideUtils::castRay(const float4& v0, const float4& v1, const float4& v2,
			 const float4& rayFrom, const float4& rayTo, float margin, float4* bCrdOut)
{
	float t, v, w;
	float4 ab; ab = v1 - v0;
	float4 ac; ac = v2 - v0;
	float4 qp; qp = rayFrom - rayTo;
	float4 normal = cross3( ab, ac );
	float d = dot3F4( qp, normal );
	float odd = 1.f/d;
	float4 ap; ap = rayFrom - v0;
	t = dot3F4( ap, normal );
	t *= odd;
//	if( t < 0.f || t > 1.f ) return -1;

	float4 e = cross3( qp, ap );
	v = dot3F4( ac, e );
	v *= odd;
	if( v < -margin || v > 1.f+margin ) return -1;
	w = -dot3F4( ab, e );
	w *= odd;
//	if( w < 0.f || w > 1.f ) return -1;
	if( w < -margin || w > 1.f+margin ) return -1;

	float u = 1.f-v-w;
	if( u < -margin || u > 1.f+margin ) return -1;
	
	if( bCrdOut )
	{
		bCrdOut->x = u;
		bCrdOut->y = v;
		bCrdOut->z = w;
	}
	return t;
}

#endif

