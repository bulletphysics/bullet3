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
//#include "LinearMath/b3Quickprof.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Matrix3x3.h"
//#include "../opencl/gpu_rigidbody_pipeline/b3ConvexUtility.h"
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
//	Bullet::b3CollisionShapeData* m_shape;
	int		m_collisionShapeIndex;

//	b3AlignedObjectArray<bParse::bStructHandle*> m_rigidBodies;
};




#define MY_UNITSPHERE_POINTS 42

static b3Vector3	sUnitSpherePoints[MY_UNITSPHERE_POINTS] = 
{
	b3Vector3(b3Scalar(0.000000) , b3Scalar(-0.000000),b3Scalar(-1.000000)),
	b3Vector3(b3Scalar(0.723608) , b3Scalar(-0.525725),b3Scalar(-0.447219)),
	b3Vector3(b3Scalar(-0.276388) , b3Scalar(-0.850649),b3Scalar(-0.447219)),
	b3Vector3(b3Scalar(-0.894426) , b3Scalar(-0.000000),b3Scalar(-0.447216)),
	b3Vector3(b3Scalar(-0.276388) , b3Scalar(0.850649),b3Scalar(-0.447220)),
	b3Vector3(b3Scalar(0.723608) , b3Scalar(0.525725),b3Scalar(-0.447219)),
	b3Vector3(b3Scalar(0.276388) , b3Scalar(-0.850649),b3Scalar(0.447220)),
	b3Vector3(b3Scalar(-0.723608) , b3Scalar(-0.525725),b3Scalar(0.447219)),
	b3Vector3(b3Scalar(-0.723608) , b3Scalar(0.525725),b3Scalar(0.447219)),
	b3Vector3(b3Scalar(0.276388) , b3Scalar(0.850649),b3Scalar(0.447219)),
	b3Vector3(b3Scalar(0.894426) , b3Scalar(0.000000),b3Scalar(0.447216)),
	b3Vector3(b3Scalar(-0.000000) , b3Scalar(0.000000),b3Scalar(1.000000)),
	b3Vector3(b3Scalar(0.425323) , b3Scalar(-0.309011),b3Scalar(-0.850654)),
	b3Vector3(b3Scalar(-0.162456) , b3Scalar(-0.499995),b3Scalar(-0.850654)),
	b3Vector3(b3Scalar(0.262869) , b3Scalar(-0.809012),b3Scalar(-0.525738)),
	b3Vector3(b3Scalar(0.425323) , b3Scalar(0.309011),b3Scalar(-0.850654)),
	b3Vector3(b3Scalar(0.850648) , b3Scalar(-0.000000),b3Scalar(-0.525736)),
	b3Vector3(b3Scalar(-0.525730) , b3Scalar(-0.000000),b3Scalar(-0.850652)),
	b3Vector3(b3Scalar(-0.688190) , b3Scalar(-0.499997),b3Scalar(-0.525736)),
	b3Vector3(b3Scalar(-0.162456) , b3Scalar(0.499995),b3Scalar(-0.850654)),
	b3Vector3(b3Scalar(-0.688190) , b3Scalar(0.499997),b3Scalar(-0.525736)),
	b3Vector3(b3Scalar(0.262869) , b3Scalar(0.809012),b3Scalar(-0.525738)),
	b3Vector3(b3Scalar(0.951058) , b3Scalar(0.309013),b3Scalar(0.000000)),
	b3Vector3(b3Scalar(0.951058) , b3Scalar(-0.309013),b3Scalar(0.000000)),
	b3Vector3(b3Scalar(0.587786) , b3Scalar(-0.809017),b3Scalar(0.000000)),
	b3Vector3(b3Scalar(0.000000) , b3Scalar(-1.000000),b3Scalar(0.000000)),
	b3Vector3(b3Scalar(-0.587786) , b3Scalar(-0.809017),b3Scalar(0.000000)),
	b3Vector3(b3Scalar(-0.951058) , b3Scalar(-0.309013),b3Scalar(-0.000000)),
	b3Vector3(b3Scalar(-0.951058) , b3Scalar(0.309013),b3Scalar(-0.000000)),
	b3Vector3(b3Scalar(-0.587786) , b3Scalar(0.809017),b3Scalar(-0.000000)),
	b3Vector3(b3Scalar(-0.000000) , b3Scalar(1.000000),b3Scalar(-0.000000)),
	b3Vector3(b3Scalar(0.587786) , b3Scalar(0.809017),b3Scalar(-0.000000)),
	b3Vector3(b3Scalar(0.688190) , b3Scalar(-0.499997),b3Scalar(0.525736)),
	b3Vector3(b3Scalar(-0.262869) , b3Scalar(-0.809012),b3Scalar(0.525738)),
	b3Vector3(b3Scalar(-0.850648) , b3Scalar(0.000000),b3Scalar(0.525736)),
	b3Vector3(b3Scalar(-0.262869) , b3Scalar(0.809012),b3Scalar(0.525738)),
	b3Vector3(b3Scalar(0.688190) , b3Scalar(0.499997),b3Scalar(0.525736)),
	b3Vector3(b3Scalar(0.525730) , b3Scalar(0.000000),b3Scalar(0.850652)),
	b3Vector3(b3Scalar(0.162456) , b3Scalar(-0.499995),b3Scalar(0.850654)),
	b3Vector3(b3Scalar(-0.425323) , b3Scalar(-0.309011),b3Scalar(0.850654)),
	b3Vector3(b3Scalar(-0.425323) , b3Scalar(0.309011),b3Scalar(0.850654)),
	b3Vector3(b3Scalar(0.162456) , b3Scalar(0.499995),b3Scalar(0.850654))
};


void createSceneProgrammatically(GLInstancingRenderer& renderer)
{
	int strideInBytes = sizeof(float)*9;

	bool noHeightField = false;
	int barrelShapeIndex = -1;
	int cubeShapeIndex = -1;
	int tetraShapeIndex = -1;

	float position[4]={0,0,0,0};
	b3Quaternion born(b3Vector3(1,0,0),B3_PI*0.25*0.5);

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
