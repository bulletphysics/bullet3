//keep this enum in sync with the CPU version (in btCollidable.h)
//written by Erwin Coumans

#define SHAPE_CONVEX_HULL 3
#define SHAPE_CONCAVE_TRIMESH 5
#define TRIANGLE_NUM_CONVEX_FACES 5
#define SHAPE_COMPOUND_OF_CONVEX_HULLS 6

typedef unsigned int u32;

///keep this in sync with btCollidable.h
typedef struct
{
	int m_numChildShapes;
	int blaat2;
	int m_shapeType;
	int m_shapeIndex;
	
} btCollidableGpu;

typedef struct
{
	float4	m_childPosition;
	float4	m_childOrientation;
	int m_shapeIndex;
	int m_unused0;
	int m_unused1;
	int m_unused2;
} btGpuChildShape;


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
} BodyData;

typedef struct 
{
	union
	{
		float4	m_min;
		float   m_minElems[4];
		int			m_minIndices[4];
	};
	union
	{
		float4	m_max;
		float   m_maxElems[4];
		int			m_maxIndices[4];
	};
} btAabbCL;

// work-in-progress
__kernel void   bvhTraversalKernel( __global const int2* pairs, 
																					__global const BodyData* rigidBodies, 
																					__global const btCollidableGpu* collidables,
																					__global btAabbCL* aabbs,
																					__global int4* concavePairsOut,
																					__global volatile int* numConcavePairsOut,
																					int numPairs,
																					int maxNumConcavePairsCapacity
																					)
{

	int i = get_global_id(0);
	
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
		
		if ((collidables[collidableIndexA].m_shapeType==SHAPE_CONCAVE_TRIMESH))// && (collidables[collidableIndexB].m_shapeType==SHAPE_CONVEX_HULL))
		{
			int pairIdx = atomic_inc(numConcavePairsOut);
			if (pairIdx<maxNumConcavePairsCapacity)
			{
				//int4 newPair;
				concavePairsOut[pairIdx].x = bodyIndexA;
				concavePairsOut[pairIdx].y = bodyIndexB;
				concavePairsOut[pairIdx].z = 5;
				concavePairsOut[pairIdx].w = 3;
			}
		}//SHAPE_CONCAVE_TRIMESH
		
	}//i<numpairs
}