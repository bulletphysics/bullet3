/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "LinearMath/btMinMax.h"

//----------------------------------------------------------------------------------------

#define USE_FRICTION 1
#define FRICTION_BOX_GROUND_FACT 0.01f
#define FRICTION_BOX_BOX_FACT 0.01f
//#define FRICTION_BOX_BOX_FACT 0.05f
#define USE_CENTERS 1

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------   C o n s t r a i n t   s o l v e r    d e m o  3D --------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

// kernel functions


BT_GPU___global__ void clearAccumulationOfLambdaDtD(float* lambdaDtBox, int numConstraints, int numContPoints)
{
    int index = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
	if(index < numConstraints)
	{
		for(int i=0; i < numContPoints; i++)
			lambdaDtBox[numContPoints * index + i] = 0;
	}
} // clearAccumulationOfLambdaDtD()

//----------------------------------------------------------------------------------------

BT_GPU___device__ float computeImpulse3D(float3 rVel,
								 float positionConstraint,
								 float3 cNormal,
								 float dt)
{
	const float collisionConstant	=	0.1f;
	const float baumgarteConstant	=	0.1f;
	const float penetrationError	=	0.02f;

	float lambdaDt=0;
	float3 impulse=BT_GPU_make_float3(0.f,0.f,0.f);

	if(positionConstraint >= 0)
		return lambdaDt;

	positionConstraint = btMin(0.0f,positionConstraint+penetrationError);
	
	lambdaDt	=	-(BT_GPU_dot(cNormal,rVel)*(collisionConstant));
	lambdaDt	-=	(baumgarteConstant/dt*positionConstraint);

	return lambdaDt;
} // computeImpulse3D()

//----------------------------------------------------------------------------------------

#if 0
#define VLIM 1000.f
void BT_GPU___device__ chk_vect(float4* v)
{
	if(v->x < -VLIM) v->x = 0.f;
	if(v->x >  VLIM) v->x = 0.f;
	if(v->y < -VLIM) v->y = 0.f;
	if(v->y >  VLIM) v->y = 0.f;
	if(v->z < -VLIM) v->z = 0.f;
	if(v->z >  VLIM) v->z = 0.f;
} // chk_vect()
#endif

//----------------------------------------------------------------------------------------

BT_GPU___global__ void collisionWithWallBox3DD(float4 *trans,
								   float4 *vel,
								   float4* angVel,
								   btCudaPartProps pProp,
								   btCudaBoxProps gProp,
								   int nParticles,
								   float dt)
{
    int idx = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
	float3 aPos;
	float positionConstraint;
	float3 impulse;
	
	if(idx < nParticles)
	{
		aPos = BT_GPU_make_float34(trans[idx * 4 + 3]);
		for(int iVtx=0; iVtx < 8; iVtx++)
		{
			float3 dx = BT_GPU_make_float34(trans[idx * 4 + 0]);
			float3 dy = BT_GPU_make_float34(trans[idx * 4 + 1]);
			float3 dz = BT_GPU_make_float34(trans[idx * 4 + 2]);
			float3 rerVertex = ((iVtx & 1) ? dx : dx * (-1.f));
			
			rerVertex += ((iVtx & 2) ? dy : dy * (-1.f));
			rerVertex += ((iVtx & 4) ? dz : dz * (-1.f));
			float3 vPos = aPos + rerVertex;
			float3 aVel	= BT_GPU_make_float3(vel[idx].x, vel[idx].y, vel[idx].z);
			float3 aAngVel	= BT_GPU_make_float34(angVel[idx]);
			float3 vVel	=aVel+BT_GPU_cross(aAngVel, rerVertex);
			float restitution=0.5;
			{
				positionConstraint  = vPos.y - gProp.minY;
				impulse				= BT_GPU_make_float31(0.0f);
				if(positionConstraint < 0)
				{
					float3 groundNormal;
					groundNormal = BT_GPU_make_float3(0.0f,1.0f,0.0f);
					impulse	= groundNormal * restitution * computeImpulse3D(vVel, positionConstraint, groundNormal, dt);
#if USE_FRICTION	// only with ground for now
					float3 lat_vel = vVel - groundNormal * BT_GPU_dot(groundNormal,vVel);
					float lat_vel_len = BT_GPU_dot(lat_vel, lat_vel);
					if (lat_vel_len > 0)
					{
						lat_vel_len = sqrtf(lat_vel_len);
						lat_vel *= 1.f/lat_vel_len;	
						impulse	-= lat_vel * BT_GPU_dot(lat_vel, vVel) * FRICTION_BOX_GROUND_FACT;
					}
#endif //USE_FRICTION
					vel[idx]	+=	BT_GPU_make_float42(impulse,0.0f);
					angVel[idx]	+=	BT_GPU_make_float42(BT_GPU_cross(rerVertex,impulse), 0.0f);
				}
			}
			{
				positionConstraint	= vPos.x - gProp.minX;
				impulse	= BT_GPU_make_float31(0.0f);
				if(positionConstraint < 0)
				{
					float3 normal = BT_GPU_make_float3(1.0f,0.0f,0.0f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	BT_GPU_make_float42(impulse,0.0f);
					angVel[idx]	+=	BT_GPU_make_float42(BT_GPU_cross(rerVertex,impulse), 0.0f);
				}
			}
			{
				positionConstraint	= gProp.maxX - vPos.x;
				impulse	= BT_GPU_make_float31(0.0f);
				if(positionConstraint < 0)
				{
					float3 normal = BT_GPU_make_float3(-1.0f,0.0f,0.0f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	BT_GPU_make_float42(impulse,0.0f);
					angVel[idx]	+=	BT_GPU_make_float42(BT_GPU_cross(rerVertex,impulse), 0.0f);
				}
			}
			{
				positionConstraint	= vPos.z - gProp.minZ;
				impulse	= BT_GPU_make_float31(0.0f);
				if(positionConstraint < 0)
				{
					float3 normal = BT_GPU_make_float3(0.0f,0.0f,1.0f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	BT_GPU_make_float42(impulse,0.0f);
					angVel[idx]	+=	BT_GPU_make_float42(BT_GPU_cross(rerVertex,impulse), 0.0f);
				}
			}
			{
				positionConstraint	= gProp.maxZ - vPos.z;
				impulse	= BT_GPU_make_float31(0.0f);
				if(positionConstraint < 0)
				{
					float3 normal = BT_GPU_make_float3(0.0f,0.0f,-1.0f);
					impulse	= normal * restitution * computeImpulse3D(vVel,positionConstraint,normal,dt);
					vel[idx]	+=	BT_GPU_make_float42(impulse,0.0f);
					angVel[idx]	+=	BT_GPU_make_float42(BT_GPU_cross(rerVertex,impulse), 0.0f);
				}
			}
		}
	}
} // collisionWithWallBox3DD()

//----------------------------------------------------------------------------------------

BT_GPU___global__ void collisionBatchResolutionBox3DD(int2 *constraints,
										 int *batch,
										 int nConstraints,
										 float4 *trans,
										 float4 *vel,
										 float4 *angularVel,
										 float *lambdaDtBox,
										 float *iPositionConstraint,
										 float3 *normal,
										 float3 *contact,
										 btCudaPartProps pProp,
										 int iBatch,
										 float dt)
{
	float3 relVel;
	float3 impulse;
	float lambdaDt;
	float positionConstraint;
    int k_idx = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
	if(k_idx < nConstraints)
	{
		int idx = batch[k_idx];
		int aId=constraints[idx].x;
		int bId=constraints[idx].y;
		float3 aPos = BT_GPU_make_float34(trans[aId * 4 + 3]);
		float3 bPos = BT_GPU_make_float34(trans[bId * 4 + 3]);
		float3 aVel = BT_GPU_make_float34(vel[aId]);
		float3 bVel = BT_GPU_make_float34(vel[bId]);
		float3 aAngVel = BT_GPU_make_float34(angularVel[aId]);
		float3 bAngVel = BT_GPU_make_float34(angularVel[bId]);
		for(int iVtx = 0; iVtx < 4; iVtx++)
		{
			float3 contactPoint	= contact[idx * 4 + iVtx] - aPos;
			positionConstraint = iPositionConstraint[idx * 4 + iVtx];
			if(positionConstraint > 0)
			{
				float3 contactNormal = normal[idx * 4 + iVtx];
				relVel = (aVel + BT_GPU_cross(aAngVel, contactPoint))
				 -(bVel + BT_GPU_cross(bAngVel, contactPoint+aPos-bPos));

				lambdaDt=	computeImpulse3D(relVel, -positionConstraint, contactNormal, dt);
				{
					float rLambdaDt=lambdaDtBox[idx * 4 + iVtx];
					float pLambdaDt=rLambdaDt;
					rLambdaDt=btMax(pLambdaDt+lambdaDt,0.0f);
					lambdaDt=rLambdaDt-pLambdaDt;
					lambdaDtBox[idx * 4 + iVtx]=rLambdaDt;
				}
				impulse = contactNormal*lambdaDt*0.5;
#if USE_FRICTION
				float3 lat_vel = relVel - contactNormal * BT_GPU_dot(contactNormal, relVel);
				float lat_vel_len = BT_GPU_dot(lat_vel, lat_vel);
				if (lat_vel_len > 0)
				{
					lat_vel_len = sqrtf(lat_vel_len);
					lat_vel *= 1.f/lat_vel_len;
					impulse	-= lat_vel * BT_GPU_dot(lat_vel , relVel) * FRICTION_BOX_BOX_FACT;
				}
#endif //USE_FRICTION
				aVel+=	impulse;
				bVel-=	impulse;
				aAngVel += BT_GPU_cross(contactPoint, impulse);
				bAngVel -= BT_GPU_cross(contactPoint+aPos-bPos, impulse);
			}
		}
		vel[aId]=BT_GPU_make_float42(aVel,0.0f);
		vel[bId]=BT_GPU_make_float42(bVel,0.0f);
		angularVel[aId]=BT_GPU_make_float42(aAngVel,0.0f);
		angularVel[bId]=BT_GPU_make_float42(bAngVel,0.0f);
	}
} // collisionBatchResolutionBox3DD()

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------


extern "C"
{

// global functions

//----------------------------------------------------------------------------------------

void BT_GPU_PREF(clearAccumulationOfLambdaDt(float* lambdaDtBox, int numConstraints, int numContPoints))
{
	if(!numConstraints) 
	{
		return;
	}
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numConstraints, 256, numBlocks, numThreads);
    // execute the kernel
    BT_GPU_EXECKERNEL(numBlocks, numThreads, clearAccumulationOfLambdaDtD, (lambdaDtBox, numConstraints, numContPoints));
    // check if kernel invocation generated an error
    BT_GPU_CHECK_ERROR("clearAccumulationOfLambdaDtD kernel execution failed");
    
} // clearAccumulationOfLambdaDt()

//----------------------------------------------------------------------------------------

void BT_GPU_PREF(collisionWithWallBox3D(void* trans,void* vel,void* angVel,btCudaPartProps pProp,	btCudaBoxProps gProp,int numObjs,float dt))
{
	if(!numObjs) 
	{
		return;
	}
	float4* pTrans = (float4*)trans;
	float4* pVel = (float4*)vel;
	float4* pAngVel = (float4*)angVel;
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numObjs, 256, numBlocks, numThreads);
    // execute the kernel
    BT_GPU_EXECKERNEL(numBlocks, numThreads, collisionWithWallBox3DD, (pTrans,pVel,pAngVel,pProp,gProp,numObjs,dt));
    // check if kernel invocation generated an error
    BT_GPU_CHECK_ERROR("collisionWithWallBox3DD kernel execution failed");
} // collisionWithWallBox3D()

//----------------------------------------------------------------------------------------

void BT_GPU_PREF(collisionBatchResolutionBox3D(void* constraints,int *batch,int numConstraints,void *trans,void *vel,
											void *angularVel,float *lambdaDtBox,float *positionConstraint,void* normal,void* contact,
											btCudaPartProps pProp,int iBatch,float dt))
{
	if(!numConstraints) 
	{
		return;
	}
	int2* pConstr = (int2*)constraints;
	float4* pTrans = (float4*)trans;
	float4* pVel = (float4*)vel;
	float4* pAngVel = (float4*)angularVel;
	float3* pNorm = (float3*)normal;
	float3* pContact = (float3*)contact;
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numConstraints, 128, numBlocks, numThreads);
    // execute the kernel
    BT_GPU_EXECKERNEL(numBlocks, numThreads, collisionBatchResolutionBox3DD, (pConstr,batch,numConstraints,pTrans,pVel,pAngVel,lambdaDtBox,positionConstraint,pNorm,pContact,pProp,iBatch,dt));
    // check if kernel invocation generated an error
    BT_GPU_CHECK_ERROR("collisionBatchResolutionBox3DD kernel execution failed");
} // collisionBatchResolutionBox3D()

//----------------------------------------------------------------------------------------

} // extern "C"

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------   M o t i o n   i n t e g r a t o r   d e m o   -----------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

// kernel functions

BT_GPU___global__ void integrVelD(float4* pForceTorqueDamp, float4* pInvInertiaMass, float4* pVel, float4* pAngVel, float timeStep, unsigned int numBodies)
{
    int index = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
	// unpack input data
	float3 force =  BT_GPU_make_float34(pForceTorqueDamp[index * 2]);
	float lin_damp = pForceTorqueDamp[index * 2].w;
	float3 torque =  BT_GPU_make_float34(pForceTorqueDamp[index * 2 + 1]);
	float ang_damp = pForceTorqueDamp[index * 2 + 1].w;
	float3 linVel =  BT_GPU_make_float34(pVel[index]);
	float3 angVel =  BT_GPU_make_float34(pAngVel[index]);
	float3 in_mass_0 = BT_GPU_make_float34(pInvInertiaMass[index * 3]);
	float3 in_mass_1 = BT_GPU_make_float34(pInvInertiaMass[index * 3 + 1]);
	float3 in_mass_2 = BT_GPU_make_float34(pInvInertiaMass[index * 3 + 2]);
	float mass = pInvInertiaMass[index * 3].w;
	// integrate linear velocity
	float3 outLinVel, outAngVel;
	outLinVel = linVel + force * mass * timeStep;
	// integrate angular velocity
	outAngVel.x = BT_GPU_dot(in_mass_0, torque);
	outAngVel.y = BT_GPU_dot(in_mass_1, torque);
	outAngVel.z = BT_GPU_dot(in_mass_2, torque);
	outAngVel += angVel;
	/// clamp angular velocity. collision calculations will fail on higher angular velocities	
	#if(!defined(M_PI))
	#define M_PI 3.1415926f
	#endif
	#define BT_CUDA_MAX_SQ_ANGVEL (M_PI*M_PI)
	float sq_angvel = BT_GPU_dot(outAngVel, outAngVel);
	sq_angvel *= timeStep * timeStep;
	float fact;
	if(sq_angvel > BT_CUDA_MAX_SQ_ANGVEL)
	{
		fact = sqrtf(BT_CUDA_MAX_SQ_ANGVEL/sq_angvel) / timeStep;
		outAngVel *= fact;
	}
	// now apply damping
	fact = powf(1.0f - lin_damp, timeStep);
	outLinVel *= fact;
	fact = powf(1.0f - ang_damp, timeStep);
	outAngVel *= fact;
	// pack results
	pVel[index] = BT_GPU_make_float42(outLinVel, 0.f);
	pAngVel[index] = BT_GPU_make_float42(outAngVel, 0.f);
} // integrVelD()

#define BT_GPU__ANGULAR_MOTION_THRESHOLD (0.25f * M_PI)

//----------------------------------------------------------------------------------------

BT_GPU___device__ float4 getRotation(float4* trans)
{
	float trace = trans[0].x + trans[1].y + trans[2].z;
	float temp[4];
	if(trace > 0.0f)
	{
		float s = sqrtf(trace + 1.0f);
		temp[3] = s * 0.5f;
		s = 0.5f / s;
		temp[0] = (trans[1].z - trans[2].y) * s;
		temp[1] = (trans[2].x - trans[0].z) * s;
		temp[2] = (trans[0].y - trans[1].x) * s;
	}
	else
	{
			typedef float btMatrRow[4];
			btMatrRow* m_el = (btMatrRow*)trans;
			int i = m_el[0][0] < m_el[1][1] ? 
				(m_el[1][1] < m_el[2][2] ? 2 : 1) :
				(m_el[0][0] < m_el[2][2] ? 2 : 0); 
			int j = (i + 1) % 3;  
			int k = (i + 2) % 3;
			float s = sqrtf(m_el[i][i] - m_el[j][j] - m_el[k][k] + 1.0f);
			temp[i] = s * 0.5f;
			s = 0.5f / s;
			temp[3] = (m_el[j][k] - m_el[k][j]) * s;
			temp[j] = (m_el[i][j] + m_el[j][i]) * s;
			temp[k] = (m_el[i][k] + m_el[k][i]) * s;
	}
	float4 q = BT_GPU_make_float44(temp[0],temp[1],temp[2],temp[3]);
	return q;
} // getRotation()

//----------------------------------------------------------------------------------------

BT_GPU___device__ float4 quatMult(float4& q1, float4& q2)
{
	return BT_GPU_make_float44(q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z,
		q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x,
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z); 
} // quatMult()

//----------------------------------------------------------------------------------------

BT_GPU___device__ void quatNorm(float4& q)
{
	float len = sqrtf(BT_GPU_dot4(q, q));
	q *= 1.f / len;
} // quatNorm()

//----------------------------------------------------------------------------------------

BT_GPU___device__ void setRotation(float4& q, float4* trans) 
{
	float d = BT_GPU_dot4(q, q);
	float s = 2.0f / d;
	float xs = q.x * s,   ys = q.y * s,   zs = q.z * s;
	float wx = q.w * xs,  wy = q.w * ys,  wz = q.w * zs;
	float xx = q.x * xs,  xy = q.x * ys,  xz = q.x * zs;
	float yy = q.y * ys,  yz = q.y * zs,  zz = q.z * zs;
    trans[0].x = 1.0f - (yy + zz);
	trans[1].x = xy - wz;
	trans[2].x = xz + wy;
	trans[0].y = xy + wz;
	trans[1].y = 1.0f - (xx + zz);
	trans[2].y = yz - wx;
	trans[0].z = xz - wy;
	trans[1].z = yz + wx;
	trans[2].z = 1.0f - (xx + yy);
	trans[0].w = trans[1].w = trans[2].w = 0.0f;
} // setRotation()

//----------------------------------------------------------------------------------------

BT_GPU___global__ void integrTransD(float4* pTrans, float4* pVel, float4* pAngVel, float timeStep, unsigned int numBodies)
{
    int index = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
	float3 pos = BT_GPU_make_float34(pTrans[index * 4 + 3]);
	float3 linvel = BT_GPU_make_float34(pVel[index]);
	pos += linvel * timeStep;

	float3 axis;
	float3 angvel = BT_GPU_make_float34(pAngVel[index]);
	float fAngle = sqrtf(BT_GPU_dot(angvel, angvel));
	//limit the angular motion
	if(fAngle*timeStep > BT_GPU__ANGULAR_MOTION_THRESHOLD)
	{
		fAngle = BT_GPU__ANGULAR_MOTION_THRESHOLD / timeStep;
	}
	if(fAngle < 0.001f)
	{
		// use Taylor's expansions of sync function
		axis = angvel * (0.5f*timeStep-(timeStep*timeStep*timeStep)*0.020833333333f * fAngle * fAngle);
	}
	else
	{
		// sync(fAngle) = sin(c*fAngle)/t
		axis = angvel * ( sinf(0.5f * fAngle * timeStep) / fAngle);
	}
	float4 dorn = BT_GPU_make_float42(axis, cosf(fAngle * timeStep * 0.5f));
	float4 orn0 = getRotation(pTrans + index * 4);
	float4 predictedOrn = quatMult(dorn, orn0);
	quatNorm(predictedOrn);
	setRotation(predictedOrn, pTrans + index * 4);
	pTrans[index * 4 + 3] = BT_GPU_make_float42(pos, 0.f);
} // integrTransD()


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

// global functions

extern "C"
{

//----------------------------------------------------------------------------------------

void BT_GPU_PREF(integrVel(float* pForceTorqueDamp, float* pInvInertiaMass, void* pVel, void* pAngVel, float timeStep, unsigned int numBodies))
{
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
    BT_GPU_EXECKERNEL(numBlocks, numThreads, integrVelD, ((float4*)pForceTorqueDamp, (float4*)pInvInertiaMass, (float4*)pVel, (float4*)pAngVel, timeStep, numBodies));
    BT_GPU_CHECK_ERROR("Kernel execution failed: btCuda_integrVelD");
} // integrVel()

//----------------------------------------------------------------------------------------

void BT_GPU_PREF(integrTrans(void* trans, void* vel, void* angVel, float timeStep, int numBodies))
{
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
    BT_GPU_EXECKERNEL(numBlocks, numThreads, integrTransD, ((float4*)trans, (float4*)vel, (float4*)angVel, timeStep, numBodies));
    BT_GPU_CHECK_ERROR("Kernel execution failed: btCuda_integrTransD");
} // integrTrans()

//----------------------------------------------------------------------------------------

} // extern "C"

//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
