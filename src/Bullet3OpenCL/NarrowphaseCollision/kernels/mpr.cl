
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3MprPenetration.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3Contact4Data.h"

#define AppendInc(x, out) out = atomic_inc(x)
#define GET_NPOINTS(x) (x).m_worldNormalOnB.w
#ifdef cl_ext_atomic_counters_32
	#pragma OPENCL EXTENSION cl_ext_atomic_counters_32 : enable
#else
	#define counter32_t volatile __global int*
#endif


__kernel void   mprPenetrationKernel( __global int4* pairs,
																					__global const b3RigidBodyData_t* rigidBodies, 
																					__global const b3Collidable_t* collidables,
																					__global const b3ConvexPolyhedronData_t* convexShapes, 
																					__global const float4* vertices,
																					__global float4* separatingNormals,
																					__global int* hasSeparatingAxis,
																					__global struct b3Contact4Data* restrict globalContactsOut,
																					counter32_t nGlobalContactsOut,
																					int contactCapacity,
																					int numPairs)
{
	int i = get_global_id(0);
	int pairIndex = i;
	if (i<numPairs)
	{
		int bodyIndexA = pairs[i].x;
		int bodyIndexB = pairs[i].y;

		int collidableIndexA = rigidBodies[bodyIndexA].m_collidableIdx;
		int collidableIndexB = rigidBodies[bodyIndexB].m_collidableIdx;
	
		int shapeIndexA = collidables[collidableIndexA].m_shapeIndex;
		int shapeIndexB = collidables[collidableIndexB].m_shapeIndex;
		
		
		//once the broadphase avoids static-static pairs, we can remove this test
		if ((rigidBodies[bodyIndexA].m_invMass==0) &&(rigidBodies[bodyIndexB].m_invMass==0))
		{
			return;
		}
		

		if ((collidables[collidableIndexA].m_shapeType!=SHAPE_CONVEX_HULL) ||(collidables[collidableIndexB].m_shapeType!=SHAPE_CONVEX_HULL))
		{
			return;
		}

		float depthOut;
		b3Float4 dirOut;
		b3Float4 posOut;


		int res = b3MprPenetration(pairIndex, bodyIndexA, bodyIndexB,rigidBodies,convexShapes,collidables,vertices,separatingNormals,hasSeparatingAxis,&depthOut, &dirOut, &posOut);
		
		
		
		

		if (res==0)
		{
			//add a contact

			int dstIdx;
			AppendInc( nGlobalContactsOut, dstIdx );
			if (dstIdx<contactCapacity)
			{
				__global struct b3Contact4Data* c = globalContactsOut + dstIdx;
				c->m_worldNormalOnB = -dirOut;//normal;
				c->m_restituitionCoeffCmp = (0.f*0xffff);c->m_frictionCoeffCmp = (0.7f*0xffff);
				c->m_batchIdx = pairIndex;
				int bodyA = pairs[pairIndex].x;
				int bodyB = pairs[pairIndex].y;
				c->m_bodyAPtrAndSignBit = rigidBodies[bodyA].m_invMass==0 ? -bodyA:bodyA;
				c->m_bodyBPtrAndSignBit = rigidBodies[bodyB].m_invMass==0 ? -bodyB:bodyB;
				c->m_childIndexA = -1;
				c->m_childIndexB = -1;
				//for (int i=0;i<nContacts;i++)
				posOut.w = -depthOut;
				c->m_worldPosB[0] = posOut;//localPoints[contactIdx[i]];
				GET_NPOINTS(*c) = 1;//nContacts;
			}
		}

	}
}
