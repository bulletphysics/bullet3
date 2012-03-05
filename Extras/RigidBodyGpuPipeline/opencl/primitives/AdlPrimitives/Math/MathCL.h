
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


#pragma OPENCL EXTENSION cl_amd_printf : enable
#pragma OPENCL EXTENSION cl_khr_local_int32_base_atomics : enable
#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable
#pragma OPENCL EXTENSION cl_khr_local_int32_extended_atomics : enable
#pragma OPENCL EXTENSION cl_khr_global_int32_extended_atomics : enable
#pragma OPENCL EXTENSION cl_ext_atomic_counters_32 : enable

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

#define GET_GROUP_IDX get_group_id(0)
#define GET_LOCAL_IDX get_local_id(0)
#define GET_GLOBAL_IDX get_global_id(0)
#define GET_GROUP_SIZE get_local_size(0)
#define GET_NUM_GROUPS get_num_groups(0)
#define GROUP_LDS_BARRIER barrier(CLK_LOCAL_MEM_FENCE)
#define GROUP_MEM_FENCE mem_fence(CLK_LOCAL_MEM_FENCE)
#define AtomInc(x) atom_inc(&(x))
#define AtomInc1(x, out) out = atom_inc(&(x))
#define AppendInc(x, out) out = atomic_inc(x)
#define AtomAdd(x, value) atom_add(&(x), value)
#define AtomCmpxhg(x, cmp, value) atom_cmpxchg( &(x), cmp, value )
#define AtomXhg(x, value) atom_xchg ( &(x), value )


#define SELECT_UINT4( b, a, condition ) select( b,a,condition )

#define make_float4 (float4)
#define make_float2 (float2)
#define make_uint4 (uint4)
#define make_int4 (int4)
#define make_uint2 (uint2)
#define make_int2 (int2)


#define max2 max
#define min2 min


///////////////////////////////////////
//	Vector
///////////////////////////////////////
__inline
float fastDiv(float numerator, float denominator)
{
	return native_divide(numerator, denominator);	
//	return numerator/denominator;	
}

__inline
float4 fastDiv4(float4 numerator, float4 denominator)
{
	return native_divide(numerator, denominator);	
}

__inline
float fastSqrtf(float f2)
{
	return native_sqrt(f2);
//	return sqrt(f2);
}

__inline
float fastRSqrt(float f2)
{
	return native_rsqrt(f2);
}

__inline
float fastLength4(float4 v)
{
	return fast_length(v);
}

__inline
float4 fastNormalize4(float4 v)
{
	return fast_normalize(v);
}


__inline
float sqrtf(float a)
{
//	return sqrt(a);
	return native_sqrt(a);
}

__inline
float4 cross3(float4 a, float4 b)
{
	return cross(a,b);
}

__inline
float dot3F4(float4 a, float4 b)
{
	float4 a1 = make_float4(a.xyz,0.f);
	float4 b1 = make_float4(b.xyz,0.f);
	return dot(a1, b1);
}

__inline
float length3(const float4 a)
{
	return sqrtf(dot3F4(a,a));
}

__inline
float dot4(const float4 a, const float4 b)
{
	return dot( a, b );
}

//	for height
__inline
float dot3w1(const float4 point, const float4 eqn)
{
	return dot3F4(point,eqn) + eqn.w;
}

__inline
float4 normalize3(const float4 a)
{
	float4 n = make_float4(a.x, a.y, a.z, 0.f);
	return fastNormalize4( n );
//	float length = sqrtf(dot3F4(a, a));
//	return 1.f/length * a;
}

__inline
float4 normalize4(const float4 a)
{
	float length = sqrtf(dot4(a, a));
	return 1.f/length * a;
}

__inline
float4 createEquation(const float4 a, const float4 b, const float4 c)
{
	float4 eqn;
	float4 ab = b-a;
	float4 ac = c-a;
	eqn = normalize3( cross3(ab, ac) );
	eqn.w = -dot3F4(eqn,a);
	return eqn;
}

///////////////////////////////////////
//	Matrix3x3
///////////////////////////////////////

typedef struct
{
	float4 m_row[3];
}Matrix3x3;

__inline
Matrix3x3 mtZero();

__inline
Matrix3x3 mtIdentity();

__inline
Matrix3x3 mtTranspose(Matrix3x3 m);

__inline
Matrix3x3 mtMul(Matrix3x3 a, Matrix3x3 b);

__inline
float4 mtMul1(Matrix3x3 a, float4 b);

__inline
float4 mtMul3(float4 a, Matrix3x3 b);

__inline
Matrix3x3 mtZero()
{
	Matrix3x3 m;
	m.m_row[0] = (float4)(0.f);
	m.m_row[1] = (float4)(0.f);
	m.m_row[2] = (float4)(0.f);
	return m;
}

__inline
Matrix3x3 mtIdentity()
{
	Matrix3x3 m;
	m.m_row[0] = (float4)(1,0,0,0);
	m.m_row[1] = (float4)(0,1,0,0);
	m.m_row[2] = (float4)(0,0,1,0);
	return m;
}

__inline
Matrix3x3 mtTranspose(Matrix3x3 m)
{
	Matrix3x3 out;
	out.m_row[0] = (float4)(m.m_row[0].x, m.m_row[1].x, m.m_row[2].x, 0.f);
	out.m_row[1] = (float4)(m.m_row[0].y, m.m_row[1].y, m.m_row[2].y, 0.f);
	out.m_row[2] = (float4)(m.m_row[0].z, m.m_row[1].z, m.m_row[2].z, 0.f);
	return out;
}

__inline
Matrix3x3 mtMul(Matrix3x3 a, Matrix3x3 b)
{
	Matrix3x3 transB;
	transB = mtTranspose( b );
	Matrix3x3 ans;
	//	why this doesn't run when 0ing in the for{}
	a.m_row[0].w = 0.f;
	a.m_row[1].w = 0.f;
	a.m_row[2].w = 0.f;
	for(int i=0; i<3; i++)
	{
//	a.m_row[i].w = 0.f;
		ans.m_row[i].x = dot3F4(a.m_row[i],transB.m_row[0]);
		ans.m_row[i].y = dot3F4(a.m_row[i],transB.m_row[1]);
		ans.m_row[i].z = dot3F4(a.m_row[i],transB.m_row[2]);
		ans.m_row[i].w = 0.f;
	}
	return ans;
}

__inline
float4 mtMul1(Matrix3x3 a, float4 b)
{
	float4 ans;
	ans.x = dot3F4( a.m_row[0], b );
	ans.y = dot3F4( a.m_row[1], b );
	ans.z = dot3F4( a.m_row[2], b );
	ans.w = 0.f;
	return ans;
}

__inline
float4 mtMul3(float4 a, Matrix3x3 b)
{
	float4 colx = make_float4(b.m_row[0].x, b.m_row[1].x, b.m_row[2].x, 0);
	float4 coly = make_float4(b.m_row[0].y, b.m_row[1].y, b.m_row[2].y, 0);
	float4 colz = make_float4(b.m_row[0].z, b.m_row[1].z, b.m_row[2].z, 0);

	float4 ans;
	ans.x = dot3F4( a, colx );
	ans.y = dot3F4( a, coly );
	ans.z = dot3F4( a, colz );
	return ans;
}

///////////////////////////////////////
//	Quaternion
///////////////////////////////////////

typedef float4 Quaternion;

__inline
Quaternion qtMul(Quaternion a, Quaternion b);

__inline
Quaternion qtNormalize(Quaternion in);

__inline
float4 qtRotate(Quaternion q, float4 vec);

__inline
Quaternion qtInvert(Quaternion q);

__inline
Matrix3x3 qtGetRotationMatrix(Quaternion q);



__inline
Quaternion qtMul(Quaternion a, Quaternion b)
{
	Quaternion ans;
	ans = cross3( a, b );
	ans += a.w*b+b.w*a;
//	ans.w = a.w*b.w - (a.x*b.x+a.y*b.y+a.z*b.z);
	ans.w = a.w*b.w - dot3F4(a, b);
	return ans;
}

__inline
Quaternion qtNormalize(Quaternion in)
{
	return fastNormalize4(in);
//	in /= length( in );
//	return in;
}
__inline
float4 qtRotate(Quaternion q, float4 vec)
{
	Quaternion qInv = qtInvert( q );
	float4 vcpy = vec;
	vcpy.w = 0.f;
	float4 out = qtMul(qtMul(q,vcpy),qInv);
	return out;
}

__inline
Quaternion qtInvert(Quaternion q)
{
	return (Quaternion)(-q.xyz, q.w);
}

__inline
float4 qtInvRotate(const Quaternion q, float4 vec)
{
	return qtRotate( qtInvert( q ), vec );
}

__inline
Matrix3x3 qtGetRotationMatrix(Quaternion quat)
{
	float4 quat2 = (float4)(quat.x*quat.x, quat.y*quat.y, quat.z*quat.z, 0.f);
	Matrix3x3 out;

	out.m_row[0].x=1-2*quat2.y-2*quat2.z;
	out.m_row[0].y=2*quat.x*quat.y-2*quat.w*quat.z;
	out.m_row[0].z=2*quat.x*quat.z+2*quat.w*quat.y;
	out.m_row[0].w = 0.f;

	out.m_row[1].x=2*quat.x*quat.y+2*quat.w*quat.z;
	out.m_row[1].y=1-2*quat2.x-2*quat2.z;
	out.m_row[1].z=2*quat.y*quat.z-2*quat.w*quat.x;
	out.m_row[1].w = 0.f;

	out.m_row[2].x=2*quat.x*quat.z-2*quat.w*quat.y;
	out.m_row[2].y=2*quat.y*quat.z+2*quat.w*quat.x;
	out.m_row[2].z=1-2*quat2.x-2*quat2.y;
	out.m_row[2].w = 0.f;

	return out;
}


