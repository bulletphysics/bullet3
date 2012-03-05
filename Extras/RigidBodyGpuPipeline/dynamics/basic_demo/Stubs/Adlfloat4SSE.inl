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


//#define CHECK_ALIGNMENT(a) CLASSERT((u32(&(a)) & 0xf) == 0);
#define CHECK_ALIGNMENT(a) a;


__inline
float4 make_float4(float x, float y, float z, float w = 0.f)
{
	float4 v;
	v.m_quad = _mm_set_ps(w,z,y,x);

	return v;
}

__inline
float4 make_float4(float x)
{
	return make_float4(x,x,x,x);
}

__inline
float4 make_float4(const int4& x)
{
	return make_float4((float)x.s[0], (float)x.s[1], (float)x.s[2], (float)x.s[3]);
}

__inline
float2 make_float2(float x, float y)
{
	float2 v;
	v.s[0] = x; v.s[1] = y;
	return v;
}

__inline
float2 make_float2(float x)
{
	return make_float2(x,x);
}

__inline
float2 make_float2(const int2& x)
{
	return make_float2((float)x.s[0], (float)x.s[1]);
}

__inline
int4 make_int4(int x, int y, int z, int w = 0)
{
	int4 v;
	v.s[0] = x; v.s[1] = y; v.s[2] = z; v.s[3] = w;
	return v;
}

__inline
int4 make_int4(int x)
{
	return make_int4(x,x,x,x);
}

__inline
int4 make_int4(const float4& x)
{
	return make_int4((int)x.x, (int)x.y, (int)x.z, (int)x.w);
}

__inline
int2 make_int2(int a, int b)
{
	int2 ans; ans.x = a; ans.y = b;
	return ans;
}

__inline
float4 operator-(const float4& a)
{
	float4 zero; zero.m_quad = _mm_setzero_ps();
	float4 ans; ans.m_quad = _mm_sub_ps( zero.m_quad, a.m_quad );
	return ans;
}

__inline
float4 operator*(const float4& a, const float4& b)
{
	CHECK_ALIGNMENT(a);

	float4 out;
	out.m_quad = _mm_mul_ps( a.m_quad, b.m_quad );
	return out;
}

__inline
float4 operator*(float a, const float4& b)
{
	float4 av; av.m_quad = _mm_set1_ps( a );
	return av*b;
}

__inline
float4 operator*(const float4& b, float a)
{
	CHECK_ALIGNMENT(b);

	float4 av; av.m_quad = _mm_set1_ps( a );
	return av*b;
}

__inline
void operator*=(float4& a, const float4& b)
{
	CHECK_ALIGNMENT(a);

	a = a*b;
}

__inline
void operator*=(float4& a, float b)
{
	CHECK_ALIGNMENT(a);

	float4 bv; bv.m_quad = _mm_set1_ps( b );
	a = a*bv;
}

//
__inline
float4 operator/(const float4& a, const float4& b)
{
	CHECK_ALIGNMENT(a);

	float4 out;
	out.m_quad = _mm_div_ps( a.m_quad, b.m_quad );
	return out;
}

__inline
float4 operator/(const float4& b, float a)
{
	CHECK_ALIGNMENT(b);

	float4 av; av.m_quad = _mm_set1_ps( a );
	float4 out;
	out = b/av;
	return out;
}

__inline
void operator/=(float4& a, const float4& b)
{
	a = a/b;
}

__inline
void operator/=(float4& a, float b)
{
	CLASSERT((u32(&a) & 0xf) == 0);

	float4 bv; bv.m_quad = _mm_set1_ps( b );
	a = a/bv;
}
//

__inline
float4 operator+(const float4& a, const float4& b)
{
	CHECK_ALIGNMENT(a);

	float4 out;
	out.m_quad = _mm_add_ps( a.m_quad, b.m_quad );
	return out;
}

__inline
float4 operator+(const float4& a, float b)
{
	CHECK_ALIGNMENT(a);

	float4 bv; bv.m_quad = _mm_set1_ps( b );
	return a+bv;
}

__inline
float4 operator-(const float4& a, const float4& b)
{
	CHECK_ALIGNMENT(a);

	float4 out;
	out.m_quad = _mm_sub_ps( a.m_quad, b.m_quad );
	return out;
}

__inline
float4 operator-(const float4& a, float b)
{
	CHECK_ALIGNMENT(a);

	float4 bv; bv.m_quad = _mm_set1_ps( b );
	return a-bv;
}

__inline
void operator+=(float4& a, const float4& b)
{
	CHECK_ALIGNMENT(a);

	a = a + b;
}

__inline
void operator+=(float4& a, float b)
{
	CHECK_ALIGNMENT(a);

	float4 bv; bv.m_quad = _mm_set1_ps( b );

	a = a + bv;
}

__inline
void operator-=(float4& a, const float4& b)
{
	CHECK_ALIGNMENT(a);

	a = a - b;
}

__inline
void operator-=(float4& a, float b)
{
	CHECK_ALIGNMENT(a);

	float4 bv; bv.m_quad = _mm_set1_ps( b );

	a = a - bv;
}





__inline
float4 cross3(const float4& a, const float4& b)
{	//	xnamathvector.inl
	union IntVec
	{
		unsigned int m_i[4];
		__m128 m_v;
	};

	IntVec mask3 = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000};
	__m128 V1 = a.m_quad;
	__m128 V2 = b.m_quad;

    __m128 vTemp1 = _mm_shuffle_ps(V1,V1,_MM_SHUFFLE(3,0,2,1));
    // z2,x2,y2,w2
    __m128 vTemp2 = _mm_shuffle_ps(V2,V2,_MM_SHUFFLE(3,1,0,2));
    // Perform the left operation
    __m128 vResult = _mm_mul_ps(vTemp1,vTemp2);
    // z1,x1,y1,w1
    vTemp1 = _mm_shuffle_ps(vTemp1,vTemp1,_MM_SHUFFLE(3,0,2,1));
    // y2,z2,x2,w2
    vTemp2 = _mm_shuffle_ps(vTemp2,vTemp2,_MM_SHUFFLE(3,1,0,2));
    // Perform the right operation
    vTemp1 = _mm_mul_ps(vTemp1,vTemp2);
    // Subract the right from left, and return answer
    vResult = _mm_sub_ps(vResult,vTemp1);
    // Set w to zero
	float4 ans; ans.m_quad = _mm_and_ps(vResult,mask3.m_v);
	return ans;
}

__inline
float dot3F4(const float4& a, const float4& b)
{
//	return a.x*b.x+a.y*b.y+a.z*b.z;
    // Perform the dot product
	__m128 V1 = a.m_quad;
	__m128 V2 = b.m_quad;

	__m128 vDot = _mm_mul_ps(V1,V2);
    // x=Dot.vector4_f32[1], y=Dot.vector4_f32[2]
    __m128 vTemp = _mm_shuffle_ps(vDot,vDot,_MM_SHUFFLE(2,1,2,1));
    // Result.vector4_f32[0] = x+y
    vDot = _mm_add_ss(vDot,vTemp);
    // x=Dot.vector4_f32[2]
    vTemp = _mm_shuffle_ps(vTemp,vTemp,_MM_SHUFFLE(1,1,1,1));
    // Result.vector4_f32[0] = (x+y)+z
    vDot = _mm_add_ss(vDot,vTemp);
    // Splat x
	float4 ans; ans.m_quad = _mm_shuffle_ps(vDot,vDot,_MM_SHUFFLE(0,0,0,0));
	return ans.x;
}

__inline
float length3(const float4& a)
{
	return sqrtf(dot3F4(a,a));
}

__inline
float dot4(const float4& a, const float4& b)
{
	return a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;
}

//	for height
__inline
float dot3w1(const float4& point, const float4& eqn)
{
	return point.x*eqn.x+point.y*eqn.y+point.z*eqn.z+eqn.w;
}

__inline
float4 normalize3(const float4& a)
{
	float length = sqrtf(dot3F4(a, a));
	return 1.f/length * a;
}

__inline
float4 normalize4(const float4& a)
{
	float length = sqrtf(dot4(a, a));
	return 1.f/length * a;
}

__inline
float4 createEquation(const float4& a, const float4& b, const float4& c)
{
	float4 eqn;
	float4 ab = b-a;
	float4 ac = c-a;
	eqn = normalize3( cross3(ab, ac) );
	eqn.w = -dot3F4(eqn,a);
	return eqn;
}


template<typename T>
__inline
T max2(const T& a, const T& b)
{
	return (a>b)? a:b;
}

template<typename T>
__inline
T min2(const T& a, const T& b)
{
	return (a<b)? a:b;
}

template<>
__inline
float4 max2(const float4& a, const float4& b)
{
	return make_float4( max2(a.x,b.x), max2(a.y,b.y), max2(a.z,b.z), max2(a.w,b.w) );
}

template<>
__inline
float4 min2(const float4& a, const float4& b)
{
	return make_float4( min2(a.x,b.x), min2(a.y,b.y), min2(a.z,b.z), min2(a.w,b.w) );
}

