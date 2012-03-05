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


#ifdef cl_ext_atomic_counters_32
#pragma OPENCL EXTENSION cl_ext_atomic_counters_32 : enable
#else
#define counter32_t volatile global int*
#endif

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




#define WG_SIZE 64

typedef struct
{
	float4 m_pos;
	Quaternion m_quat;
	float4 m_linVel;
	float4 m_angVel;

	u32 m_shapeIdx;
	u32 m_shapeType;
	float m_invMass;
	float m_restituitionCoeff;
	float m_frictionCoeff;
} Body;

typedef struct
{
	Matrix3x3 m_invInertia;
	Matrix3x3 m_initInvInertia;
} Shape;

typedef struct
{
	float4 m_linear;
	float4 m_worldPos[4];
	float4 m_center;	
	float m_jacCoeffInv[4];
	float m_b[4];
	float m_appliedRambdaDt[4];

	float m_fJacCoeffInv[2];	
	float m_fAppliedRambdaDt[2];	

	u32 m_bodyA;
	u32 m_bodyB;

	int m_batchIdx;
	u32 m_paddings[1];
} Constraint4;

typedef struct
{
	float4 m_worldPos[4];
	float4 m_worldNormal;
	u32 m_coeffs;
	int m_batchIdx;

	u32 m_bodyAPtr;
	u32 m_bodyBPtr;
} Contact4;

typedef struct
{
	int m_nConstraints;
	int m_start;
	int m_batchIdx;
	int m_nSplit;
//	int m_paddings[1];
} ConstBuffer;

typedef struct
{
	int m_solveFriction;
	int m_maxBatch;	//	long batch really kills the performance
	int m_batchIdx;
	int m_nSplit;
//	int m_paddings[1];
} ConstBufferBatchSolve;


void setLinearAndAngular( float4 n, float4 r0, float4 r1, float4* linear, float4* angular0, float4* angular1)
{
	*linear = -n;
	*angular0 = -cross3(r0, n);
	*angular1 = cross3(r1, n);
}


float calcRelVel( float4 l0, float4 l1, float4 a0, float4 a1, float4 linVel0, float4 angVel0, float4 linVel1, float4 angVel1 )
{
	return dot3F4(l0, linVel0) + dot3F4(a0, angVel0) + dot3F4(l1, linVel1) + dot3F4(a1, angVel1);
}


float calcJacCoeff(const float4 linear0, const float4 linear1, const float4 angular0, const float4 angular1,
					float invMass0, const Matrix3x3* invInertia0, float invMass1, const Matrix3x3* invInertia1)
{
	//	linear0,1 are normlized
	float jmj0 = invMass0;//dot3F4(linear0, linear0)*invMass0;
	float jmj1 = dot3F4(mtMul3(angular0,*invInertia0), angular0);
	float jmj2 = invMass1;//dot3F4(linear1, linear1)*invMass1;
	float jmj3 = dot3F4(mtMul3(angular1,*invInertia1), angular1);
	return -1.f/(jmj0+jmj1+jmj2+jmj3);
}



void solveContact(__global Constraint4* cs,
			float4 posA, float4* linVelA, float4* angVelA, float invMassA, Matrix3x3 invInertiaA,
			float4 posB, float4* linVelB, float4* angVelB, float invMassB, Matrix3x3 invInertiaB)
{
	float minRambdaDt = 0;
	float maxRambdaDt = FLT_MAX;

	for(int ic=0; ic<4; ic++)
	{
		if( cs->m_jacCoeffInv[ic] == 0.f ) continue;

		float4 angular0, angular1, linear;
		float4 r0 = cs->m_worldPos[ic] - posA;
		float4 r1 = cs->m_worldPos[ic] - posB;
		setLinearAndAngular( -cs->m_linear, r0, r1, &linear, &angular0, &angular1 );

		float rambdaDt = calcRelVel( cs->m_linear, -cs->m_linear, angular0, angular1, 
			*linVelA, *angVelA, *linVelB, *angVelB ) + cs->m_b[ic];
		rambdaDt *= cs->m_jacCoeffInv[ic];

		{
			float prevSum = cs->m_appliedRambdaDt[ic];
			float updated = prevSum;
			updated += rambdaDt;
			updated = max2( updated, minRambdaDt );
			updated = min2( updated, maxRambdaDt );
			rambdaDt = updated - prevSum;
			cs->m_appliedRambdaDt[ic] = updated;
		}

		float4 linImp0 = invMassA*linear*rambdaDt;
		float4 linImp1 = invMassB*(-linear)*rambdaDt;
		float4 angImp0 = mtMul1(invInertiaA, angular0)*rambdaDt;
		float4 angImp1 = mtMul1(invInertiaB, angular1)*rambdaDt;

		*linVelA += linImp0;
		*angVelA += angImp0;
		*linVelB += linImp1;
		*angVelB += angImp1;
	}
}


void solveFriction(__global Constraint4* cs,
			float4 posA, float4* linVelA, float4* angVelA, float invMassA, Matrix3x3 invInertiaA,
			float4 posB, float4* linVelB, float4* angVelB, float invMassB, Matrix3x3 invInertiaB,
			float maxRambdaDt[4], float minRambdaDt[4])
{
	if( cs->m_fJacCoeffInv[0] == 0 && cs->m_fJacCoeffInv[0] == 0 ) return;
	const float4 center = cs->m_center;

	float4 n = -cs->m_linear;

	float4 tangent[2];
	tangent[0] = cross3( n, cs->m_worldPos[0]-center );
	tangent[1] = cross3( tangent[0], n );
	tangent[0] = normalize3( tangent[0] );
	tangent[1] = normalize3( tangent[1] );

	float4 angular0, angular1, linear;
	float4 r0 = center - posA;
	float4 r1 = center - posB;
	for(int i=0; i<2; i++)
	{
		setLinearAndAngular( tangent[i], r0, r1, &linear, &angular0, &angular1 );
		float rambdaDt = calcRelVel(linear, -linear, angular0, angular1,
			*linVelA, *angVelA, *linVelB, *angVelB );
		rambdaDt *= cs->m_fJacCoeffInv[i];

		{
			float prevSum = cs->m_fAppliedRambdaDt[i];
			float updated = prevSum;
			updated += rambdaDt;
			updated = max2( updated, minRambdaDt[i] );
			updated = min2( updated, maxRambdaDt[i] );
			rambdaDt = updated - prevSum;
			cs->m_fAppliedRambdaDt[i] = updated;
		}

		float4 linImp0 = invMassA*linear*rambdaDt;
		float4 linImp1 = invMassB*(-linear)*rambdaDt;
		float4 angImp0 = mtMul1(invInertiaA, angular0)*rambdaDt;
		float4 angImp1 = mtMul1(invInertiaB, angular1)*rambdaDt;

		*linVelA += linImp0;
		*angVelA += angImp0;
		*linVelB += linImp1;
		*angVelB += angImp1;
	}
	{	//	angular damping for point constraint
		float4 ab = normalize3( posB - posA );
		float4 ac = normalize3( center - posA );
		if( dot3F4( ab, ac ) > 0.95f  || (invMassA == 0.f || invMassB == 0.f))
		{
			float angNA = dot3F4( n, *angVelA );
			float angNB = dot3F4( n, *angVelB );

			*angVelA -= (angNA*0.1f)*n;
			*angVelB -= (angNB*0.1f)*n;
		}
	}
}

void solveAConstraint(__global Body* gBodies, __global Shape* gShapes, __global Constraint4* ldsCs)
{
	float frictionCoeff = ldsCs[0].m_linear.w;
	int aIdx = ldsCs[0].m_bodyA;
	int bIdx = ldsCs[0].m_bodyB;

	float4 posA = gBodies[aIdx].m_pos;
	float4 linVelA = gBodies[aIdx].m_linVel;
	float4 angVelA = gBodies[aIdx].m_angVel;
	float invMassA = gBodies[aIdx].m_invMass;
	Matrix3x3 invInertiaA = gShapes[aIdx].m_invInertia;

	float4 posB = gBodies[bIdx].m_pos;
	float4 linVelB = gBodies[bIdx].m_linVel;
	float4 angVelB = gBodies[bIdx].m_angVel;
	float invMassB = gBodies[bIdx].m_invMass;
	Matrix3x3 invInertiaB = gShapes[bIdx].m_invInertia;
		
		
	{
		solveContact( ldsCs, posA, &linVelA, &angVelA, invMassA, invInertiaA,
			posB, &linVelB, &angVelB, invMassB, invInertiaB );
	}

	{
		float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
		float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

		float sum = 0;
		for(int j=0; j<4; j++)
		{
			sum +=ldsCs[0].m_appliedRambdaDt[j];
		}
		frictionCoeff = 0.7f;
		for(int j=0; j<4; j++)
		{
			maxRambdaDt[j] = frictionCoeff*sum;
			minRambdaDt[j] = -maxRambdaDt[j];
		}

		solveFriction( ldsCs, posA, &linVelA, &angVelA, invMassA, invInertiaA,
			posB, &linVelB, &angVelB, invMassB, invInertiaB, maxRambdaDt, minRambdaDt );
	}

	gBodies[aIdx].m_linVel = linVelA;
	gBodies[aIdx].m_angVel = angVelA;
	gBodies[bIdx].m_linVel = linVelB;
	gBodies[bIdx].m_angVel = angVelB;
}

void solveContactConstraint(__global Body* gBodies, __global Shape* gShapes, __global Constraint4* ldsCs)
{
	float frictionCoeff = ldsCs[0].m_linear.w;
	int aIdx = ldsCs[0].m_bodyA;
	int bIdx = ldsCs[0].m_bodyB;

	float4 posA = gBodies[aIdx].m_pos;
	float4 linVelA = gBodies[aIdx].m_linVel;
	float4 angVelA = gBodies[aIdx].m_angVel;
	float invMassA = gBodies[aIdx].m_invMass;
	Matrix3x3 invInertiaA = gShapes[aIdx].m_invInertia;

	float4 posB = gBodies[bIdx].m_pos;
	float4 linVelB = gBodies[bIdx].m_linVel;
	float4 angVelB = gBodies[bIdx].m_angVel;
	float invMassB = gBodies[bIdx].m_invMass;
	Matrix3x3 invInertiaB = gShapes[bIdx].m_invInertia;

	solveContact( ldsCs, posA, &linVelA, &angVelA, invMassA, invInertiaA,
			posB, &linVelB, &angVelB, invMassB, invInertiaB );

	gBodies[aIdx].m_linVel = linVelA;
	gBodies[aIdx].m_angVel = angVelA;
	gBodies[bIdx].m_linVel = linVelB;
	gBodies[bIdx].m_angVel = angVelB;
}

void solveFrictionConstraint(__global Body* gBodies, __global Shape* gShapes, __global Constraint4* ldsCs)
{
	float frictionCoeff = ldsCs[0].m_linear.w;
	int aIdx = ldsCs[0].m_bodyA;
	int bIdx = ldsCs[0].m_bodyB;

	float4 posA = gBodies[aIdx].m_pos;
	float4 linVelA = gBodies[aIdx].m_linVel;
	float4 angVelA = gBodies[aIdx].m_angVel;
	float invMassA = gBodies[aIdx].m_invMass;
	Matrix3x3 invInertiaA = gShapes[aIdx].m_invInertia;

	float4 posB = gBodies[bIdx].m_pos;
	float4 linVelB = gBodies[bIdx].m_linVel;
	float4 angVelB = gBodies[bIdx].m_angVel;
	float invMassB = gBodies[bIdx].m_invMass;
	Matrix3x3 invInertiaB = gShapes[bIdx].m_invInertia;

	{
		float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
		float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

		float sum = 0;
		for(int j=0; j<4; j++)
		{
			sum +=ldsCs[0].m_appliedRambdaDt[j];
		}
		frictionCoeff = 0.7f;
		for(int j=0; j<4; j++)
		{
			maxRambdaDt[j] = frictionCoeff*sum;
			minRambdaDt[j] = -maxRambdaDt[j];
		}

		solveFriction( ldsCs, posA, &linVelA, &angVelA, invMassA, invInertiaA,
			posB, &linVelB, &angVelB, invMassB, invInertiaB, maxRambdaDt, minRambdaDt );
	}

	gBodies[aIdx].m_linVel = linVelA;
	gBodies[aIdx].m_angVel = angVelA;
	gBodies[bIdx].m_linVel = linVelB;
	gBodies[bIdx].m_angVel = angVelB;
}

typedef struct 
{
	int m_valInt0;
	int m_valInt1;
	int m_valInt2;
	int m_valInt3;

	float m_val0;
	float m_val1;
	float m_val2;
	float m_val3;
} SolverDebugInfo;


__kernel
__attribute__((reqd_work_group_size(WG_SIZE,1,1)))
//void BatchSolveKernel(__global Body* gBodies, __global Shape* gShapes, __global Constraint4* gConstraints, 	__global int* gN, __global int* gOffsets, __global SolverDebugInfo* debugInfo, ConstBufferBatchSolve cb)
void BatchSolveKernel(__global Body* gBodies, 
__global Shape* gShapes, 
__global Constraint4* gConstraints, 	
__global int* gN, 
__global int* gOffsets, 
ConstBufferBatchSolve cb)
{
	__local int ldsBatchIdx[WG_SIZE+1];

	__local int ldsCurBatch;
	__local int ldsNextBatch;
	__local int ldsStart;

	int lIdx = GET_LOCAL_IDX;
	int wgIdx = GET_GROUP_IDX;

	int gIdx = GET_GLOBAL_IDX;
//	debugInfo[gIdx].m_valInt0 = gIdx;
	//debugInfo[gIdx].m_valInt1 = GET_GROUP_SIZE;

	const int solveFriction = cb.m_solveFriction;
	const int maxBatch = cb.m_maxBatch;
	const int bIdx = cb.m_batchIdx;
	const int nSplit = cb.m_nSplit;

	int xIdx = (wgIdx/(nSplit/2))*2 + (bIdx&1);
	int yIdx = (wgIdx%(nSplit/2))*2 + (bIdx>>1);
	int cellIdx = xIdx+yIdx*nSplit;
	
	if( gN[cellIdx] == 0 ) 
		return;

	const int start = gOffsets[cellIdx];
	const int end = start + gN[cellIdx];

	
	if( lIdx == 0 )
	{
		ldsCurBatch = 0;
		ldsNextBatch = 0;
		ldsStart = start;
	}


	GROUP_LDS_BARRIER;

	int idx=ldsStart+lIdx;
	while (ldsCurBatch < maxBatch)
	{
		for(; idx<end; )
		{
			if (gConstraints[idx].m_batchIdx == ldsCurBatch)
			{
				if( solveFriction )
					solveFrictionConstraint( gBodies, gShapes, &gConstraints[idx] );
				else
					solveContactConstraint( gBodies, gShapes, &gConstraints[idx] );
				idx+=64;
			} else
			{
				break;
			}
		}
		GROUP_LDS_BARRIER;
		if( lIdx == 0 )
		{
			ldsCurBatch++;
		}
		GROUP_LDS_BARRIER;
	}
	
	return;


	int counter0 = 0;
	int counter1 = 0;

	do
	{
		counter0++;
		int curStart = ldsStart;
		int curBatch = ldsCurBatch;

		GROUP_LDS_BARRIER;

		while( curBatch == ldsNextBatch && curStart < end )
		{
			counter1++;
			int idx = curStart + lIdx;
			if( lIdx == 0 ) 
				ldsBatchIdx[0] = curBatch;
				GROUP_LDS_BARRIER;
			ldsBatchIdx[(lIdx == 0)? lIdx:lIdx+1] = curBatch;
			GROUP_LDS_BARRIER;
//			debugInfo[gIdx].m_valInt2 = gConstraints[idx].m_batchIdx;
//			debugInfo[gIdx].m_valInt3 = idx;


			ldsBatchIdx[lIdx+1] = (idx<end)? gConstraints[idx].m_batchIdx : -1;

			GROUP_LDS_BARRIER;


			if( ldsBatchIdx[lIdx+1] == curBatch )
			{
				//	solve

				if( solveFriction )
				{
					solveFrictionConstraint( gBodies, gShapes, &gConstraints[idx] );
					}
				else
				{
					solveContactConstraint( gBodies, gShapes, &gConstraints[idx] );
					}
			}
			else
			{
				if( ldsBatchIdx[lIdx] == curBatch )
				{
					ldsStart = idx;
					ldsNextBatch = ldsBatchIdx[lIdx+1];
				}
			}

			GROUP_LDS_BARRIER;

			curStart += GET_GROUP_SIZE;
		}

		if( lIdx == 0 )
		{
			ldsCurBatch = ldsNextBatch;
		}

		GROUP_LDS_BARRIER;
	}
//	while( ldsCurBatch != -1 && iter <= 10 );
	while( ldsCurBatch != -1 && ldsCurBatch <= maxBatch );

}



//	others
__kernel
__attribute__((reqd_work_group_size(WG_SIZE,1,1)))
void ReorderContactKernel(__global Contact4* in, __global Contact4* out, __global int2* sortData, int4 cb )
{
	int nContacts = cb.x;
	int gIdx = GET_GLOBAL_IDX;

	if( gIdx < nContacts )
	{
		int srcIdx = sortData[gIdx].y;
		out[gIdx] = in[srcIdx];
	}
}

typedef struct
{
	int m_nContacts;
	int m_staticIdx;
	float m_scale;
	int m_nSplit;
} ConstBufferSSD;

__kernel
__attribute__((reqd_work_group_size(WG_SIZE,1,1)))
void SetSortDataKernel(__global Contact4* gContact, __global Body* gBodies, __global int2* gSortDataOut, ConstBufferSSD cb )
{
	int gIdx = GET_GLOBAL_IDX;
	int nContacts = cb.m_nContacts;
	int staticIdx = cb.m_staticIdx;
	float scale = cb.m_scale;
	int N_SPLIT = cb.m_nSplit;

	if( gIdx < nContacts )
	{
		int aIdx = gContact[gIdx].m_bodyAPtr;
		int bIdx = gContact[gIdx].m_bodyBPtr;

		int idx = (aIdx == staticIdx)? bIdx: aIdx;
		float4 p = gBodies[idx].m_pos;
		int xIdx = (int)((p.x-((p.x<0.f)?1.f:0.f))*scale) & (N_SPLIT-1);
		int zIdx = (int)((p.z-((p.z<0.f)?1.f:0.f))*scale) & (N_SPLIT-1);

		gSortDataOut[gIdx].x = (xIdx+zIdx*N_SPLIT);
		gSortDataOut[gIdx].y = gIdx;
	}
	else
	{
		gSortDataOut[gIdx].x = 0xffffffff;
	}
}


void setConstraint4( const float4 posA, const float4 linVelA, const float4 angVelA, float invMassA, const Matrix3x3 invInertiaA, 
	const float4 posB, const float4 linVelB, const float4 angVelB, float invMassB, const Matrix3x3 invInertiaB, 
	Contact4 src, float dt, float positionDrift, float positionConstraintCoeff,
	Constraint4* dstC )
{
	dstC->m_bodyA = src.m_bodyAPtr;
	dstC->m_bodyB = src.m_bodyBPtr;

	float dtInv = 1.f/dt;
	for(int ic=0; ic<4; ic++)
	{
		dstC->m_appliedRambdaDt[ic] = 0.f;
	}
	dstC->m_fJacCoeffInv[0] = dstC->m_fJacCoeffInv[1] = 0.f;


	dstC->m_linear = -src.m_worldNormal;
	dstC->m_linear.w = 0.7f ;//src.getFrictionCoeff() );
	for(int ic=0; ic<4; ic++)
	{
		float4 r0 = src.m_worldPos[ic] - posA;
		float4 r1 = src.m_worldPos[ic] - posB;

		if( ic >= src.m_worldNormal.w )//npoints
		{
			dstC->m_jacCoeffInv[ic] = 0.f;
			continue;
		}

		float relVelN;
		{
			float4 linear, angular0, angular1;
			setLinearAndAngular(src.m_worldNormal, r0, r1, &linear, &angular0, &angular1);

			dstC->m_jacCoeffInv[ic] = calcJacCoeff(linear, -linear, angular0, angular1,
				invMassA, &invInertiaA, invMassB, &invInertiaB );

			relVelN = calcRelVel(linear, -linear, angular0, angular1,
				linVelA, angVelA, linVelB, angVelB);

			float e = 0.f;//src.getRestituitionCoeff();
			if( relVelN*relVelN < 0.004f ) e = 0.f;

			dstC->m_b[ic] = e*relVelN;
			//float penetration = src.m_worldPos[ic].w;
			dstC->m_b[ic] += (src.m_worldPos[ic].w + positionDrift)*positionConstraintCoeff*dtInv;
			dstC->m_appliedRambdaDt[ic] = 0.f;
		}
	}

	if( src.m_worldNormal.w > 1 )//npoints
	{	//	prepare friction
		float4 center = make_float4(0.f);
		for(int i=0; i<src.m_worldNormal.w; i++) center += src.m_worldPos[i];
		center /= (float)src.m_worldNormal.w;

		float4 tangent[2];
		tangent[0] = cross3( src.m_worldNormal, src.m_worldPos[0]-center );
		tangent[1] = cross3( tangent[0], src.m_worldNormal );
		tangent[0] = normalize3( tangent[0] );
		tangent[1] = normalize3( tangent[1] );
		float4 r[2];
		r[0] = center - posA;
		r[1] = center - posB;

		for(int i=0; i<2; i++)
		{
			float4 linear, angular0, angular1;
			setLinearAndAngular(tangent[i], r[0], r[1], &linear, &angular0, &angular1);

			dstC->m_fJacCoeffInv[i] = calcJacCoeff(linear, -linear, angular0, angular1,
				invMassA, &invInertiaA, invMassB, &invInertiaB );
			dstC->m_fAppliedRambdaDt[i] = 0.f;
		}
		dstC->m_center = center;
	}
	else
	{
		//	single point constraint
	}

	for(int i=0; i<4; i++)
	{
		if( i<src.m_worldNormal.w )
		{
			dstC->m_worldPos[i] = src.m_worldPos[i];
		}
		else
		{
			dstC->m_worldPos[i] = make_float4(0.f);
		}
	}
}

typedef struct
{
	int m_nContacts;
	float m_dt;
	float m_positionDrift;
	float m_positionConstraintCoeff;
} ConstBufferCTC;

__kernel
__attribute__((reqd_work_group_size(WG_SIZE,1,1)))
void ContactToConstraintKernel(__global Contact4* gContact, __global Body* gBodies, __global Shape* gShapes, __global Constraint4* gConstraintOut, ConstBufferCTC cb)
{
	int gIdx = GET_GLOBAL_IDX;
	int nContacts = cb.m_nContacts;
	float dt = cb.m_dt;
	float positionDrift = cb.m_positionDrift;
	float positionConstraintCoeff = cb.m_positionConstraintCoeff;

	if( gIdx < nContacts )
	{
		int aIdx = gContact[gIdx].m_bodyAPtr;
		int bIdx = gContact[gIdx].m_bodyBPtr;

		float4 posA = gBodies[aIdx].m_pos;
		float4 linVelA = gBodies[aIdx].m_linVel;
		float4 angVelA = gBodies[aIdx].m_angVel;
		float invMassA = gBodies[aIdx].m_invMass;
		Matrix3x3 invInertiaA = gShapes[aIdx].m_invInertia;

		float4 posB = gBodies[bIdx].m_pos;
		float4 linVelB = gBodies[bIdx].m_linVel;
		float4 angVelB = gBodies[bIdx].m_angVel;
		float invMassB = gBodies[bIdx].m_invMass;
		Matrix3x3 invInertiaB = gShapes[bIdx].m_invInertia;

		Constraint4 cs;

		setConstraint4( posA, linVelA, angVelA, invMassA, invInertiaA, posB, linVelB, angVelB, invMassB, invInertiaB,
			gContact[gIdx], dt, positionDrift, positionConstraintCoeff, 
			&cs );
		
		cs.m_batchIdx = gContact[gIdx].m_batchIdx;

		gConstraintOut[gIdx] = cs;
	}
}

__kernel
__attribute__((reqd_work_group_size(WG_SIZE,1,1)))
void CopyConstraintKernel(__global Contact4* gIn, __global Contact4* gOut, int4 cb )
{
	int gIdx = GET_GLOBAL_IDX;
	if( gIdx < cb.x )
	{
		gOut[gIdx] = gIn[gIdx];
	}
}