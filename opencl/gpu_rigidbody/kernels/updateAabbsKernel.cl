
#define SHAPE_CONVEX_HULL 3

typedef float4 Quaternion;

__inline
float4 cross3(float4 a, float4 b)
{
	return cross(a,b);
}

__inline
float dot3F4(float4 a, float4 b)
{
	float4 a1 = (float4)(a.xyz,0.f);
	float4 b1 = (float4)(b.xyz,0.f);
	return dot(a1, b1);
}


__inline
Quaternion qtMul(Quaternion a, Quaternion b)
{
	Quaternion ans;
	ans = cross3( a, b );
	ans += a.w*b+b.w*a;
	ans.w = a.w*b.w - dot3F4(a, b);
	return ans;
}

__inline
Quaternion qtInvert(Quaternion q)
{
	return (Quaternion)(-q.xyz, q.w);
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
float4 transform(const float4* p, const float4* translation, const Quaternion* orientation)
{
	return qtRotate( *orientation, *p ) + (*translation);
}

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

	u32 m_collidableIdx;
	float m_invMass;
	float m_restituitionCoeff;
	float m_frictionCoeff;
} Body;

typedef struct Collidable
{
	int m_unused1;
	int m_unused2;
	int m_shapeType;
	int m_shapeIndex;
} Collidable;


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
	int	uw;
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


__kernel void initializeGpuAabbsFull(  const int numNodes, __global Body* gBodies,__global Collidable* collidables, __global btAABBCL* plocalShapeAABB, __global btAABBCL* pAABB)
{
	int nodeID = get_global_id(0);
		
	if( nodeID < numNodes )
	{
		float4 position = gBodies[nodeID].m_pos;
		float4 orientation = gBodies[nodeID].m_quat;
		
			
		int collidableIndex = gBodies[nodeID].m_collidableIdx;
		int shapeIndex = collidables[collidableIndex].m_shapeIndex;
			
		if (shapeIndex>=0)
		{
			btAABBCL minAabb = plocalShapeAABB[collidableIndex*2];
			btAABBCL maxAabb = plocalShapeAABB[collidableIndex*2+1];
				
			float4 halfExtents = ((float4)(maxAabb.fx - minAabb.fx,maxAabb.fy - minAabb.fy,maxAabb.fz - minAabb.fz,0.f))*0.5f;
			float4 localCenter = ((float4)(maxAabb.fx + minAabb.fx,maxAabb.fy + minAabb.fy,maxAabb.fz + minAabb.fz,0.f))*0.5f;
				
			float4 worldCenter = transform(&localCenter,&position,&orientation);
				
			Matrix3x3 abs_b = qtGetRotationMatrix(orientation);
			float4 extent = (float4) (	dot(abs_b.m_row[0],halfExtents),dot(abs_b.m_row[1],halfExtents),dot(abs_b.m_row[2],halfExtents),0.f);
			
	
			pAABB[nodeID*2].fx = worldCenter.x-extent.x;
			pAABB[nodeID*2].fy = worldCenter.y-extent.y;
			pAABB[nodeID*2].fz = worldCenter.z-extent.z;
			pAABB[nodeID*2].uw = nodeID;
	
			pAABB[nodeID*2+1].fx = worldCenter.x+extent.x;
			pAABB[nodeID*2+1].fy = worldCenter.y+extent.y;
			pAABB[nodeID*2+1].fz = worldCenter.z+extent.z;
			pAABB[nodeID*2+1].uw = gBodies[nodeID].m_invMass==0.f? 0 : 1;
		}
	} 
}
