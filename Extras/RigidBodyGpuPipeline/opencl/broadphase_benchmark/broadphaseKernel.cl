MSTRINGIFY(

typedef struct
{
	float4	m_row[3];
} Matrix3x3;

typedef unsigned int u32;


typedef struct
{
	float4 m_pos;
	float4 m_quat;
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


__inline
Matrix3x3 qtGetRotationMatrix(float4 quat)
{
	float4 quat2 = (float4)(quat.x*quat.x, quat.y*quat.y, quat.z*quat.z, 0.f);
	Matrix3x3 out;

	out.m_row[0].x=fabs(1-2*quat2.y-2*quat2.z);
	out.m_row[0].y=fabs(2*quat.x*quat.y-2*quat.w*quat.z);
	out.m_row[0].z=fabs(2*quat.x*quat.z+2*quat.w*quat.y);
	out.m_row[0].w = 0.f;

	out.m_row[1].x=fabs(2*quat.x*quat.y+2*quat.w*quat.z);
	out.m_row[1].y=fabs(1-2*quat2.x-2*quat2.z);
	out.m_row[1].z=fabs(2*quat.y*quat.z-2*quat.w*quat.x);
	out.m_row[1].w = 0.f;

	out.m_row[2].x=fabs(2*quat.x*quat.z-2*quat.w*quat.y);
	out.m_row[2].y=fabs(2*quat.y*quat.z+2*quat.w*quat.x);
	out.m_row[2].z=fabs(1-2*quat2.x-2*quat2.y);
	out.m_row[2].w = 0.f;

	return out;
}


typedef struct 
{
	float			fx;
	float			fy;
	float			fz;
	unsigned int	uw;
} btAABBCL;

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
float dot3F4(float4 a, float4 b)
{
	float4 a1 = (float4)(a.xyz,0.f);
	float4 b1 = (float4)(b.xyz,0.f);
	return dot(a1, b1);
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


//apply gravity
//update world inverse inertia tensor
//copy velocity from arrays to bodies
//copy transforms from buffer to bodies

__kernel void 
  setupBodiesKernel( const int startOffset, const int numNodes, __global float4 *g_vertexBuffer,
		   __global float4 *linVel,
		   __global float4 *pAngVel,
		   __global Body* gBodies, __global Shape* bodyInertias
		   )
{
	int nodeID = get_global_id(0);
		
	float timeStep = 0.0166666f;
	float BT_GPU_ANGULAR_MOTION_THRESHOLD = (0.25f * 3.14159254);

	if( nodeID < numNodes )
	{
		float inverseMass = gBodies[nodeID].m_invMass;
		if (inverseMass != 0.f)
		{
			float4 position = g_vertexBuffer[nodeID + startOffset/4];
			float4 orientation = g_vertexBuffer[nodeID + startOffset/4+numNodes];

			float4 gravityAcceleration = (float4)(0.f,-9.8f,0.f,0.f);
			linVel[nodeID] += gravityAcceleration * timeStep;
		
			gBodies[nodeID].m_pos = position;
			gBodies[nodeID].m_quat = orientation;

			gBodies[nodeID].m_linVel = (float4)(linVel[nodeID].xyz,0.f);
			gBodies[nodeID].m_angVel = (float4)(pAngVel[nodeID].xyz,0.f);

			Matrix3x3 m = qtGetRotationMatrix( orientation);
			Matrix3x3 mT = mtTranspose( m );

			Matrix3x3 tmp = mtMul(m, bodyInertias[nodeID].m_initInvInertia);
			Matrix3x3 tmp2 = mtMul(tmp, mT);
			bodyInertias[nodeID].m_invInertia = tmp2;

			//shapeInfo.m_invInertia = mtMul( mtMul( m, shapeInfo.m_initInvInertia ), mT );


		} else
		{
			gBodies[nodeID].m_linVel = (float4)(0.f,0.f,0.f,0.f);
			gBodies[nodeID].m_angVel = (float4)(0.f,0.f,0.f,0.f);
		}


	}
}


__kernel void 
  copyVelocitiesKernel( const int startOffset, const int numNodes, __global float4 *g_vertexBuffer,
		   __global float4 *linVel,
		   __global float4 *pAngVel,
		   __global Body* gBodies, __global Shape* bodyInertias
		   )
{
	int nodeID = get_global_id(0);
	if( nodeID < numNodes )
	{
		float inverseMass = gBodies[nodeID].m_invMass;
		if (inverseMass != 0.f)
		{
			linVel[nodeID] = (float4)(gBodies[nodeID].m_linVel.xyz,0.f);
			pAngVel[nodeID] = (float4)(gBodies[nodeID].m_angVel.xyz,0.f);
		}
	}
}



__kernel void 
  initializeGpuAabbsSimple( const int startOffset, const int numNodes, __global float4 *g_vertexBuffer, __global btAABBCL* pAABB)
{
	int nodeID = get_global_id(0);
		
	if( nodeID < numNodes )
	{
		float4 position = g_vertexBuffer[nodeID + startOffset/4];
		float4 orientation = g_vertexBuffer[nodeID + startOffset/4+numNodes];
		float4 color = g_vertexBuffer[nodeID + startOffset/4+numNodes+numNodes];
		
		float4 green = (float4)(.4f,1.f,.4f,1.f);
		g_vertexBuffer[nodeID + startOffset/4+numNodes+numNodes] = green;
		

		float4 halfExtents = (float4)(1.01f,1.01f,1.01f,0.f);
		//float4 extent=(float4)(1.f,1.f,1.f,0.f);

		Matrix3x3 abs_b = qtGetRotationMatrix(orientation);

		float4 extent = (float4) (
			dot(abs_b.m_row[0],halfExtents),
			dot(abs_b.m_row[1],halfExtents),
			dot(abs_b.m_row[2],halfExtents),
			0.f);
		

		pAABB[nodeID*2].fx = position.x-extent.x;
		pAABB[nodeID*2].fy = position.y-extent.y;
		pAABB[nodeID*2].fz = position.z-extent.z;
		pAABB[nodeID*2].uw = nodeID;

		pAABB[nodeID*2+1].fx = position.x+extent.x;
		pAABB[nodeID*2+1].fy = position.y+extent.y;
		pAABB[nodeID*2+1].fz = position.z+extent.z;
		pAABB[nodeID*2+1].uw = nodeID;		
	}
}



__kernel void 
  initializeGpuAabbsFull( const int startOffset, const int numNodes, __global float4 *g_vertexBuffer, __global Body* gBodies, __global btAABBCL* plocalShapeAABB, __global btAABBCL* pAABB)
{
	int nodeID = get_global_id(0);
		
	if( nodeID < numNodes )
	{
		float4 position = g_vertexBuffer[nodeID + startOffset/4];
		float4 orientation = g_vertexBuffer[nodeID + startOffset/4+numNodes];
		float4 color = g_vertexBuffer[nodeID + startOffset/4+numNodes+numNodes];
		
		float4 green = (float4)(.4f,1.f,.4f,1.f);
		g_vertexBuffer[nodeID + startOffset/4+numNodes+numNodes] = green;
		
		int shapeIndex = gBodies[nodeID].m_shapeIdx;
		if (shapeIndex>=0)
		{
			btAABBCL minAabb = plocalShapeAABB[shapeIndex*2];
			btAABBCL maxAabb = plocalShapeAABB[shapeIndex*2+1];
			
			float4 halfExtents = ((float4)(maxAabb.fx - minAabb.fx,maxAabb.fy - minAabb.fy,maxAabb.fz - minAabb.fz,0.f))*0.5f;

			Matrix3x3 abs_b = qtGetRotationMatrix(orientation);
			float4 extent = (float4) (	dot(abs_b.m_row[0],halfExtents),dot(abs_b.m_row[1],halfExtents),dot(abs_b.m_row[2],halfExtents),0.f);
		

			pAABB[nodeID*2].fx = position.x-extent.x;
			pAABB[nodeID*2].fy = position.y-extent.y;
			pAABB[nodeID*2].fz = position.z-extent.z;
			pAABB[nodeID*2].uw = nodeID;

			pAABB[nodeID*2+1].fx = position.x+extent.x;
			pAABB[nodeID*2+1].fy = position.y+extent.y;
			pAABB[nodeID*2+1].fz = position.z+extent.z;
			pAABB[nodeID*2+1].uw = nodeID;		
		}
	}
}


__kernel void 
  broadphaseColorKernel( const int startOffset, const int numNodes, __global float4 *g_vertexBuffer, __global int2* pOverlappingPairs, const int numOverlap)
{
	int nodeID = get_global_id(0);
	if( nodeID < numOverlap )
	{
		int2 pair = pOverlappingPairs[nodeID];
		float4 red = (float4)(1.f,0.4f,0.4f,1.f);
		
		g_vertexBuffer[pair.x + startOffset/4+numNodes+numNodes] = red;
		g_vertexBuffer[pair.y + startOffset/4+numNodes+numNodes] = red;
	}
}



__kernel void 
  broadphaseKernel( const int startOffset, const int numNodes, __global float4 *g_vertexBuffer)
{
	int nodeID = get_global_id(0);
	
//	float BT_GPU_ANGULAR_MOTION_THRESHOLD = (0.25f * 3.14159254);
	
	if( nodeID < numNodes )
	{
		float4 position = g_vertexBuffer[nodeID + startOffset/4];
		//float4 orientation = g_vertexBuffer[nodeID + startOffset/4+numNodes];
		float4 color = g_vertexBuffer[nodeID + startOffset/4+numNodes+numNodes];
		
		float4 red = (float4)(1.f,0.f,0.f,0.f);
		float4 green = (float4)(0.f,1.f,0.f,0.f);
		float4 blue = (float4)(0.f,0.f,1.f,0.f);
		float  overlap=0;
		int equal = 0;
		
		g_vertexBuffer[nodeID + startOffset/4+numNodes+numNodes] = green;
		
		for (int i=0;i<numNodes;i++)
		{
			if (i!=nodeID)
			{
				float4 otherPosition = g_vertexBuffer[i + startOffset/4];
				if ((otherPosition.x == position.x)&&
					(otherPosition.y == position.y)&&
					(otherPosition.z == position.z))
						equal=1;
				
				
				float distsqr = 
						((otherPosition.x - position.x)* (otherPosition.x - position.x))+
						((otherPosition.y - position.y)* (otherPosition.y - position.y))+
						((otherPosition.z - position.z)* (otherPosition.z - position.z));
				
				if (distsqr<7.f)
					overlap+=0.25f;
			}
		}
		
		
		if (equal)
		{
				g_vertexBuffer[nodeID + startOffset/4+numNodes+numNodes]=blue;
		} else
		{
			if (overlap>0.f)
				g_vertexBuffer[nodeID + startOffset/4+numNodes+numNodes]=red*overlap;
			else
				g_vertexBuffer[nodeID + startOffset/4+numNodes+numNodes]=green;
		}
	}
}

);