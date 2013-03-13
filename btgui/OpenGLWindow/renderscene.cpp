int NUM_OBJECTS_X = 35;
int NUM_OBJECTS_Y = 35;
int NUM_OBJECTS_Z = 35;


float X_GAP = 2.3f;
float Y_GAP = 2.f;
float Z_GAP = 2.3f;

bool keepStaticObjects = false;
#include <stdio.h>

#include "OpenGLInclude.h"

#include "renderscene.h"

#include "GLInstancingRenderer.h"
//#include "LinearMath/btQuickprof.h"
#include "BulletCommon/btQuaternion.h"
#include "BulletCommon/btMatrix3x3.h"
//#include "../opencl/gpu_rigidbody_pipeline/btConvexUtility.h"
#include "ShapeData.h"
///work-in-progress 
///This ReadBulletSample is kept as simple as possible without dependencies to the Bullet SDK.
///It can be used to load .bullet data for other physics SDKs
///For a more complete example how to load and convert Bullet data using the Bullet SDK check out
///the Bullet/Demos/SerializeDemo and Bullet/Serialize/BulletWorldImporter


//using namespace Bullet;

struct GraphicsVertex
{
	float xyzw[4];
	float normal[3];
	float uv[2];
};
struct GraphicsShape
{
	const float*	m_vertices;
	int				m_numvertices;
	const int*		m_indices;
	int				m_numIndices;
	float			m_scaling[4];
};

struct InstanceGroup
{
//	Bullet::btCollisionShapeData* m_shape;
	int		m_collisionShapeIndex;

//	btAlignedObjectArray<bParse::bStructHandle*> m_rigidBodies;
};




#define MY_UNITSPHERE_POINTS 42

static btVector3	sUnitSpherePoints[MY_UNITSPHERE_POINTS] = 
{
	btVector3(btScalar(0.000000) , btScalar(-0.000000),btScalar(-1.000000)),
	btVector3(btScalar(0.723608) , btScalar(-0.525725),btScalar(-0.447219)),
	btVector3(btScalar(-0.276388) , btScalar(-0.850649),btScalar(-0.447219)),
	btVector3(btScalar(-0.894426) , btScalar(-0.000000),btScalar(-0.447216)),
	btVector3(btScalar(-0.276388) , btScalar(0.850649),btScalar(-0.447220)),
	btVector3(btScalar(0.723608) , btScalar(0.525725),btScalar(-0.447219)),
	btVector3(btScalar(0.276388) , btScalar(-0.850649),btScalar(0.447220)),
	btVector3(btScalar(-0.723608) , btScalar(-0.525725),btScalar(0.447219)),
	btVector3(btScalar(-0.723608) , btScalar(0.525725),btScalar(0.447219)),
	btVector3(btScalar(0.276388) , btScalar(0.850649),btScalar(0.447219)),
	btVector3(btScalar(0.894426) , btScalar(0.000000),btScalar(0.447216)),
	btVector3(btScalar(-0.000000) , btScalar(0.000000),btScalar(1.000000)),
	btVector3(btScalar(0.425323) , btScalar(-0.309011),btScalar(-0.850654)),
	btVector3(btScalar(-0.162456) , btScalar(-0.499995),btScalar(-0.850654)),
	btVector3(btScalar(0.262869) , btScalar(-0.809012),btScalar(-0.525738)),
	btVector3(btScalar(0.425323) , btScalar(0.309011),btScalar(-0.850654)),
	btVector3(btScalar(0.850648) , btScalar(-0.000000),btScalar(-0.525736)),
	btVector3(btScalar(-0.525730) , btScalar(-0.000000),btScalar(-0.850652)),
	btVector3(btScalar(-0.688190) , btScalar(-0.499997),btScalar(-0.525736)),
	btVector3(btScalar(-0.162456) , btScalar(0.499995),btScalar(-0.850654)),
	btVector3(btScalar(-0.688190) , btScalar(0.499997),btScalar(-0.525736)),
	btVector3(btScalar(0.262869) , btScalar(0.809012),btScalar(-0.525738)),
	btVector3(btScalar(0.951058) , btScalar(0.309013),btScalar(0.000000)),
	btVector3(btScalar(0.951058) , btScalar(-0.309013),btScalar(0.000000)),
	btVector3(btScalar(0.587786) , btScalar(-0.809017),btScalar(0.000000)),
	btVector3(btScalar(0.000000) , btScalar(-1.000000),btScalar(0.000000)),
	btVector3(btScalar(-0.587786) , btScalar(-0.809017),btScalar(0.000000)),
	btVector3(btScalar(-0.951058) , btScalar(-0.309013),btScalar(-0.000000)),
	btVector3(btScalar(-0.951058) , btScalar(0.309013),btScalar(-0.000000)),
	btVector3(btScalar(-0.587786) , btScalar(0.809017),btScalar(-0.000000)),
	btVector3(btScalar(-0.000000) , btScalar(1.000000),btScalar(-0.000000)),
	btVector3(btScalar(0.587786) , btScalar(0.809017),btScalar(-0.000000)),
	btVector3(btScalar(0.688190) , btScalar(-0.499997),btScalar(0.525736)),
	btVector3(btScalar(-0.262869) , btScalar(-0.809012),btScalar(0.525738)),
	btVector3(btScalar(-0.850648) , btScalar(0.000000),btScalar(0.525736)),
	btVector3(btScalar(-0.262869) , btScalar(0.809012),btScalar(0.525738)),
	btVector3(btScalar(0.688190) , btScalar(0.499997),btScalar(0.525736)),
	btVector3(btScalar(0.525730) , btScalar(0.000000),btScalar(0.850652)),
	btVector3(btScalar(0.162456) , btScalar(-0.499995),btScalar(0.850654)),
	btVector3(btScalar(-0.425323) , btScalar(-0.309011),btScalar(0.850654)),
	btVector3(btScalar(-0.425323) , btScalar(0.309011),btScalar(0.850654)),
	btVector3(btScalar(0.162456) , btScalar(0.499995),btScalar(0.850654))
};


void createSceneProgrammatically(GLInstancingRenderer& renderer)
{
	int strideInBytes = sizeof(float)*9;

	bool noHeightField = false;
	int barrelShapeIndex = -1;
	int cubeShapeIndex = -1;
	int tetraShapeIndex = -1;

	float position[4]={0,0,0,0};
	btQuaternion born(btVector3(1,0,0),SIMD_PI*0.25*0.5);

	float orn[4] = {0,0,0,1};
//	float rotOrn[4] = {born.getX(),born.getY(),born.getZ(),born.getW()};//
	float rotOrn[4] ={0,0,0,1};
	

	float color[4] = {1,1,1,1};
	int index=0;


	




	float cubeScaling[4] = {1,1,1,1};
	{
		int numVertices = sizeof(cube_vertices)/strideInBytes;
		int numIndices = sizeof(cube_indices)/sizeof(int);
		cubeShapeIndex = renderer.registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	}

	


	if (1)
	for (int i=0;i<NUM_OBJECTS_X;i++)
	{
		for (int j=0;j<NUM_OBJECTS_Y;j++)
		{
			int k=0;
			
			for (;k<NUM_OBJECTS_Z;k++)
			{

				float mass = 1.f;//j? 1.f : 0.f;

				position[0]=(i*X_GAP-NUM_OBJECTS_X/2)+(j&1);
				position[1]=1+(j*Y_GAP);//-NUM_OBJECTS_Y/2);
				position[2]=(k*Z_GAP-NUM_OBJECTS_Z/2)+(j&1);
				position[3] = 0.f;
				
				renderer.registerGraphicsInstance(cubeShapeIndex,position,rotOrn,color,cubeScaling);
				
				index++;
			}
		}
	}

	
	{

		{
			int numVertices = sizeof(tetra_vertices)/strideInBytes;
			int numIndices = sizeof(tetra_indices)/sizeof(int);
			tetraShapeIndex = renderer.registerShape(&tetra_vertices[0],numVertices,tetra_indices,numIndices);
		}

		{
			float groundScaling[4] = {2.5,2,2.5,1};
		
			for (int i=0;i<50;i++)
				for (int j=0;j<50;j++)
			if (1)
			{
				void* ptr = (void*) index;
				float posnew[4];
				posnew[0] = i*5.0-120;
				posnew[1] = 0;
				posnew[2] = j*5.0-120;
				posnew[3] = 1.f;

				color[0] = 1.f;
				color[1] = 0.f;
				color[2] = 0.f;
				renderer.registerGraphicsInstance(tetraShapeIndex,posnew,orn,color,groundScaling);
			}
		}
	}


}
