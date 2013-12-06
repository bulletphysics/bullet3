/*
Impulse based Rigid body simulation using CUDA
Copyright (c) 2007 Takahiro Harada  http://www.iii.u-tokyo.ac.jp/~takahiroharada/projects/impulseCUDA.html

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#define USE_FRICTION 1
#define FRICTION_BOX_GROUND_FACT 0.05f
#define FRICTION_BOX_BOX_FACT 0.05f
#define USE_CENTERS 1
//#include "LinearMath/btMinMax.h"

//----------   C o n s t r a i n t   s o l v e r    d e m o   ----------------------------

#define MAX_VTX_PER_OBJ 8

/*
BT_GPU___device__ void kill_me()
{
	char* badPtr = (char*)0xFFFFFFFF;
	*badPtr = 10;
}
*/


BT_GPU___global__ void clearAccumulationOfLambdaDtD(float* lambdaDtBox, int numConstraints, int numContPoints)
{
    int index = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
	if(index < numConstraints)
	{
		for(int i=0; i < numContPoints; i++)
			lambdaDtBox[numContPoints * index + i] = 0;
	}
}

#define SPHERE_FACT 1.0f

BT_GPU___device__ void testSphSph(float3 aPos, float3 bPos, float radA, float radB, float4* pOut)
{
	float3 del = bPos - aPos;
	float dist = BT_GPU_dot(del, del);
	dist = sqrtf(dist);
	float maxD = radA + radB;
	
	if(dist > maxD)
	{
		return;
	}
	float penetration = (radA + radB - dist) * SPHERE_FACT;
//	float penetration = (dist - radA - radB) * SPHERE_FACT;
	float3 normal;
	if(dist > 0.f) 
	{
		float fact = -1.0f/dist;
//		float fact = 1.0f/dist;
		normal = del * fact; 
	}
	else
	{
		normal = BT_GPU_make_float3(1.f, 0.f, 0.f);
	}
//	float3 contact = (bPos + aPos + normal * (radB - radA)) * 0.5f;
	float3 tmp = (normal * radA);
	float3 contact = aPos - tmp;
	
	// now add point
	int numPoints = 0;
	for(int i = 0; i < MAX_VTX_PER_OBJ; i++)
	{
		if(pOut[i*2].w >= 0.f)
		{
			numPoints++;
		}
	}
	if(numPoints < MAX_VTX_PER_OBJ)
	{
		pOut[numPoints * 2] = BT_GPU_make_float42(contact, penetration);
		pOut[numPoints * 2 + 1] = BT_GPU_make_float42(normal, 0.f);
	}
} // testSphSph()



BT_GPU___global__ void setConstraintDataD(int2 *constraints,
								   int numConstraints,
								   float4 *pos,
								   float *rotation,
								   char* shapes,
								   int2* shapeIds,
								   btCudaPartProps pProp,
								   float4 *contact)
{
    int idx = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
	int aId,bId;
	float3 aPos,bPos;
//	float positionConstraint;
//	float3 normal;
	float aRot,bRot;
	float sideLength2	=	pProp.m_diameter*0.5f/sqrt(2.0f);

	if(idx < numConstraints)
	{
		aId=constraints[idx].x;
		bId=constraints[idx].y;
		
		aPos=BT_GPU_make_float34(BT_GPU_FETCH4(pos,aId));
		bPos=BT_GPU_make_float34(BT_GPU_FETCH4(pos,bId));
		aRot= rotation[aId];
		bRot= rotation[bId];
		float cosA = cosf(aRot);
		float sinA = sinf(aRot);
		float cosB = cosf(bRot);
		float sinB = sinf(bRot);
		float4* shapeA = (float4*)(shapes + shapeIds[aId].x);
		int numSphA = shapeIds[aId].y;
		float4* shapeB = (float4*)(shapes + shapeIds[bId].x);
		int numSphB = shapeIds[bId].y;
		int i, j;
		float3 ai =	BT_GPU_make_float3(cosA, sinA, 0.f);
		float3 aj =	BT_GPU_make_float3(-sinA, cosA, 0.f);
		float3 bi =	BT_GPU_make_float3(cosB, sinB, 0.f);
		float3 bj =	BT_GPU_make_float3(-sinB, cosB, 0.f);
		float4* pOut = contact + idx * MAX_VTX_PER_OBJ * 2;
		for(i = 0; i < MAX_VTX_PER_OBJ; i++)
		{
			pOut[i * 2].w = -1.f;
			pOut[i * 2 + 1].w = 0.f;
		}
		for(i = 0; i < numSphA; i++)
		{	
			float3 va = aPos;
			float3 tmp = ai * shapeA[i].x; 
			float3 tmp2 = aj * shapeA[i].y;
			
			va += tmp;
			va += tmp2;
			
			float radA = shapeA[i].w;
			for(j = 0; j < numSphB; j++)
			{
				float3 vb = bPos;
				float3 tmp =bi * shapeB[j].x;
				float3 tmp2 = bj * shapeB[j].y;
				vb += tmp;
				vb += tmp2;
				float radB = shapeB[j].w;
				testSphSph(va, vb, radA, radB, pOut);
			}
		}
	}
}


BT_GPU___device__ float computeImpulse1(float3 rVel,
								 float positionConstraint,
								 float3 cNormal,
								 float dt)
{
//	const float collisionConstant	=	0.1f;
//	const float baumgarteConstant	=	0.5f;
//	const float penetrationError	=	0.02f;
	const float collisionConstant	=	-0.1f;
	const float baumgarteConstant	=	0.3f;
	const float penetrationError	=	0.02f;

	float lambdaDt=0;
	float3 impulse=BT_GPU_make_float3(0.f,0.f,0.f);

	if(positionConstraint > 0)
		return lambdaDt;

//	positionConstraint = btMin(0.0f,positionConstraint+penetrationError);
	positionConstraint = (positionConstraint+penetrationError) < 0.f ? (positionConstraint+penetrationError) : 0.0f;
	
	lambdaDt	=	-(BT_GPU_dot(cNormal,rVel)*(1+collisionConstant));
	lambdaDt	-=	(baumgarteConstant/dt*positionConstraint);

	return lambdaDt;
}


BT_GPU___global__ void collisionWithWallBoxD(float4 *pos,
								   float4 *vel,
								   float *rotation,
								   float *angVel,
								   char* shapes,
								   int2* shapeIds,
								   float* invMass,
								   btCudaPartProps pProp,
								   btCudaBoxProps gProp,
								   int nParticles,
								   float dt)
{
    int idx = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
	float3 aPos;
	float aRot;
	float positionConstraint;
	float3 impulse;
	

	if((idx > 0) && (idx < nParticles))
	{
		float inv_mass = invMass[idx];
		if(inv_mass <= 0.f)
		{
			return;
		}
		aPos=BT_GPU_make_float34(BT_GPU_FETCH4(pos,idx));
		aRot=rotation[idx];
		float4* shape = (float4*)(shapes + shapeIds[idx].x);
		int numSph = shapeIds[idx].y;
		float cosA = cosf(aRot);
		float sinA = sinf(aRot);
		float3 ai =	BT_GPU_make_float3(cosA, sinA, 0.f);
		float3 aj =	BT_GPU_make_float3(-sinA, cosA, 0.f);

		for(int iVtx=0;iVtx < numSph; iVtx++){
			float3 aVel = BT_GPU_make_float3(vel[idx].x, vel[idx].y, vel[idx].z);
			float aAngVel = angVel[idx];
			float3 rerVertex = ai * shape[iVtx].x;
			float3 tmp = aj * shape[iVtx].y;
			rerVertex += tmp;
			float3 vPos = aPos + rerVertex;
			float rad = shape[iVtx].w;
			float3 vVel	=aVel+BT_GPU_cross(BT_GPU_make_float3(0.0f,0.0f,aAngVel),rerVertex);
//			float restitution=1.0;
			float restitution=0.3f;
			{
				positionConstraint	=vPos.y - rad - gProp.minY;
				impulse				=BT_GPU_make_float31(0.0f);

				if(positionConstraint < 0)
				{
					float3 groundNormal;
					groundNormal = BT_GPU_make_float3(0.0f,1.0f,0.0f);
					impulse	=groundNormal*
						restitution * computeImpulse1(vVel,positionConstraint,
						groundNormal,
						dt);
#if USE_FRICTION	// only with ground for now
					float3 lat_vel = vVel - groundNormal * BT_GPU_dot(groundNormal,vVel);
					float lat_vel_len = BT_GPU_dot(lat_vel, lat_vel);
					if (lat_vel_len > 0)
					{
						lat_vel_len = sqrtf(lat_vel_len);
						lat_vel *= 1.f/lat_vel_len;	
						float3 tmp = lat_vel * BT_GPU_dot(lat_vel, vVel) * FRICTION_BOX_GROUND_FACT;
						impulse	-= tmp;
					}
#endif //USE_FRICTION
					float4 tmp = BT_GPU_make_float42(impulse,0.0f);
					vel[idx]	+=	tmp;
					float tmp2 = BT_GPU_cross(rerVertex,impulse).z;
					angVel[idx]	+=	tmp2;
				}
			}

			{
				positionConstraint	=vPos.x - rad - gProp.minX;
				impulse				=BT_GPU_make_float31(0.0f);

				if(positionConstraint < 0){
					impulse	=BT_GPU_make_float3(1.0f,0.0f,0.0f)* restitution * 
						computeImpulse1(vVel,positionConstraint,
						BT_GPU_make_float3(1.0f,0.0f,0.0f),
						dt);

					float4 tmp = BT_GPU_make_float42(impulse,0.0f);
					vel[idx]	+=	tmp;
					angVel[idx]	+=	BT_GPU_cross(rerVertex,impulse).z;
				}
			}

			{
				positionConstraint	= gProp.maxX - vPos.x - rad;
				impulse				=BT_GPU_make_float31(0.0f);

				if(positionConstraint < 0){
					impulse	=BT_GPU_make_float3(-1.0f,0.0f,0.0f)* restitution * 
						computeImpulse1(vVel,positionConstraint,
						BT_GPU_make_float3(-1.0f,0.0f,0.0f),
						dt);

					float4 tmp = BT_GPU_make_float42(impulse,0.0f);
					vel[idx]	+=	tmp;
					angVel[idx]	+=	BT_GPU_cross(rerVertex,impulse).z;
				}
			}
		}
	}
}

BT_GPU___device__ void collisionResolutionBox(	int constrId,
												int2* constraints,
												float4 *pos,
												float4 *vel,
												float *rotation,
												float *angularVel,
												float *lambdaDtBox,
												float4* contact,
												float* invMass,
												btCudaPartProps pProp,
												float dt)
{
#if 1
	float3 relVel;
	float3 impulse;
	float lambdaDt;
	float positionConstraint;
	int aId=constraints[constrId].x;
	int bId=constraints[constrId].y;
	float3 aPos=BT_GPU_make_float34(BT_GPU_FETCH4(pos,aId));
	float3 bPos=BT_GPU_make_float34(BT_GPU_FETCH4(pos,bId));
	float3 aVel=BT_GPU_make_float34(vel[aId]);
	float3 bVel=BT_GPU_make_float34(vel[bId]);
	float aAngVel=angularVel[aId];
	float bAngVel=angularVel[bId];
	float4* pCont = contact + constrId * MAX_VTX_PER_OBJ * 2;
	//	test Vertices in A to Box B
	for(int iVtx=0;iVtx<MAX_VTX_PER_OBJ;iVtx++){
		float3 contactPoint	= BT_GPU_make_float34(pCont[iVtx * 2]);
		contactPoint = contactPoint - aPos;
		positionConstraint = pCont[iVtx * 2].w;
		if(positionConstraint >= 0)
		{
			float3 contactNormal = BT_GPU_make_float34(pCont[iVtx * 2 + 1]);
			relVel=(aVel+BT_GPU_cross(BT_GPU_make_float3(0.0f,0.0f,aAngVel),
				contactPoint))
				-(bVel+BT_GPU_cross(BT_GPU_make_float3(0.0f,0.0f,bAngVel),
				contactPoint+aPos-bPos));

			lambdaDt=	computeImpulse1(relVel,-positionConstraint,
				contactNormal,dt);

			{
				float rLambdaDt=lambdaDtBox[(MAX_VTX_PER_OBJ)*(2*constrId)+iVtx];
				float pLambdaDt=rLambdaDt;
//				rLambdaDt=btMax(pLambdaDt+lambdaDt,0.0f);
				rLambdaDt=(pLambdaDt+lambdaDt) > 0.0f ? (pLambdaDt+lambdaDt) : 0.0f;
				lambdaDt=rLambdaDt-pLambdaDt;
				lambdaDtBox[(MAX_VTX_PER_OBJ)*(2*constrId)+iVtx]=rLambdaDt;
			}
			impulse=	contactNormal*lambdaDt*0.5;
#if USE_FRICTION
			if(pCont[iVtx * 2 + 1].w <= 0)
			{
				float3 lat_vel = relVel - contactNormal * BT_GPU_dot(contactNormal, relVel);
				float lat_vel_len = BT_GPU_dot(lat_vel, lat_vel);
				if (lat_vel_len > 0)
				{
					lat_vel_len = sqrtf(lat_vel_len);
					lat_vel *= 1.f/lat_vel_len;
					float3 tmp = lat_vel * BT_GPU_dot(lat_vel , relVel) * FRICTION_BOX_BOX_FACT;
					impulse	-= tmp;
				}
			}
#endif //USE_FRICTION
			if(aId && (invMass[aId] > 0.f))
			{
				aVel+=	impulse;
				aAngVel+=	BT_GPU_cross(contactPoint, impulse).z;
			}
			if(bId && (invMass[bId] > 0.f))
			{
				bVel-=	impulse;
				bAngVel-=	BT_GPU_cross(contactPoint+aPos-bPos,	impulse).z;
			}
		}
	}
	vel[aId]=BT_GPU_make_float42(aVel,0.0f);
	vel[bId]=BT_GPU_make_float42(bVel,0.0f);
	angularVel[aId]=aAngVel;
	angularVel[bId]=bAngVel;
#endif
}

BT_GPU___global__ void collisionBatchResolutionBoxD(int2 *constraints,
										 int *batch,
										 int nConstraints,
										 float4 *pos,
										 float4 *vel,
										 float *rotation,
										 float *angularVel,
										 float *lambdaDtBox,
										 float4* contact,
										 float* invMass,
										 btCudaPartProps pProp,
										 int iBatch,
										 float dt)
{
    int k_idx = BT_GPU___mul24(BT_GPU_blockIdx.x, BT_GPU_blockDim.x) + BT_GPU_threadIdx.x;
	if(k_idx < nConstraints)
	{
		int idx = batch[k_idx];
		collisionResolutionBox(	idx, constraints, pos, vel, rotation, angularVel, lambdaDtBox,
								contact, invMass, pProp, dt);
	}
}


extern "C"
{

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
    
}

void BT_GPU_PREF(setConstraintData(void* constraints,int numConstraints,int numObjs,void* pos,float *rotation,char* shapes,void* shapeIds,btCudaPartProps pProp,void* contact))
{
	if(!numConstraints) 
	{
		return;
	}
	int2* pConst = (int2*)constraints;
	float4* pPos = (float4*)pos;
	float4* pCont = (float4*)contact;
	int2* pShapeIds = (int2*)shapeIds;

    BT_GPU_SAFE_CALL(BT_GPU_BindTexture(0, posTex, pPos, numObjs * sizeof(float4)));
	
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numConstraints, 256, numBlocks, numThreads);
    // execute the kernel
    BT_GPU_EXECKERNEL(numBlocks, numThreads, setConstraintDataD, (pConst,numConstraints,pPos,rotation,shapes,pShapeIds,pProp,pCont));
    BT_GPU_SAFE_CALL(BT_GPU_UnbindTexture(posTex));
    // check if kernel invocation generated an error
    BT_GPU_CHECK_ERROR("setConstraintDataD kernel execution failed");
}

void BT_GPU_PREF(collisionWithWallBox(void* pos,void* vel,float *rotation,float *angVel,char* shapes,void* shapeIds,void* invMass,btCudaPartProps pProp,	btCudaBoxProps gProp,int numObjs,float dt))
{
	if(!numObjs) 
	{
		return;
	}
	float4* pPos = (float4*)pos;
	float4* pVel = (float4*)vel;
	int2* pShapeIds = (int2*)shapeIds;
	float* pInvMass = (float*)invMass;
    BT_GPU_SAFE_CALL(BT_GPU_BindTexture(0, posTex, pPos, numObjs * sizeof(float4)));

    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numObjs, 256, numBlocks, numThreads);
    // execute the kernel
    BT_GPU_EXECKERNEL(numBlocks, numThreads, collisionWithWallBoxD, (pPos,pVel,rotation,angVel,shapes, pShapeIds,pInvMass,pProp,gProp,numObjs,dt));

    BT_GPU_SAFE_CALL(BT_GPU_UnbindTexture(posTex));
    // check if kernel invocation generated an error
    BT_GPU_CHECK_ERROR("collisionWithWallBoxD kernel execution failed");
}

void BT_GPU_PREF(collisionBatchResolutionBox(void* constraints,int *batch,int numConstraints,int numObjs,void *pos,void *vel,float *rotation,float *angularVel,float *lambdaDtBox,void* contact,void* invMass,btCudaPartProps pProp,int iBatch,float dt))
{
	if(!numConstraints) 
	{
		return;
	}
	int2* pConstr = (int2*)constraints;
	float4* pPos = (float4*)pos;
	float4* pVel = (float4*)vel;
	float4* pCont = (float4*)contact;
	float* pInvMass = (float*)invMass;
    int numThreads, numBlocks;
    BT_GPU_PREF(computeGridSize)(numConstraints, 128, numBlocks, numThreads);
    BT_GPU_SAFE_CALL(BT_GPU_BindTexture(0, posTex, pPos, numObjs * sizeof(float4)));
    // execute the kernel
    BT_GPU_EXECKERNEL(numBlocks, numThreads, collisionBatchResolutionBoxD, (pConstr,batch,numConstraints,pPos,pVel,rotation,angularVel,lambdaDtBox,pCont,pInvMass,pProp,iBatch,dt));
    // check if kernel invocation generated an error
    BT_GPU_CHECK_ERROR("collisionBatchResolutionBox2D kernel execution failed");
    BT_GPU_SAFE_CALL(BT_GPU_UnbindTexture(posTex));

}


}   // extern "C"
