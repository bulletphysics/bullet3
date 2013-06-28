#include "GpuConvexScene.h"
#include "GpuRigidBodyDemo.h"
#include "OpenGLWindow/ShapeData.h"

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "Bullet3Common/b3Quaternion.h"
#include "OpenGLWindow/b3gWindowInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "../GpuDemoInternalData.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3OpenCL/RigidBody/b3Config.h"
#include "GpuRigidBodyDemoInternalData.h"
#include "../gwenUserInterface.h"
#include "Bullet3Dynamics/ConstraintSolver/b3Point2PointConstraint.h"
#include "OpenGLWindow/GLPrimitiveRenderer.h"
#include "Bullet3OpenCL/Raycast/b3GpuRaycast.h"
#include "Bullet3OpenCL/NarrowphaseCollision/b3ConvexUtility.h"

#include "OpenGLWindow/GLRenderToTexture.h"

void GpuConvexScene::setupScene(const ConstructionInfo& ci)
{
	m_primRenderer = ci.m_primRenderer;

	m_raycaster = new b3GpuRaycast(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);

	int index=0;
	createStaticEnvironment(ci);

	index+=createDynamicsObjects(ci);

	m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();

	float camPos[4]={ci.arraySizeX,ci.arraySizeY/2,ci.arraySizeZ,0};
	//float camPos[4]={1,12.5,1.5,0};
	
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(100);
	

	m_instancingRenderer->updateCamera();

	char msg[1024];
	int numInstances = index;
	sprintf(msg,"Num objects = %d",numInstances);
	ci.m_gui->setStatusBarMessage(msg,true);
}

void	GpuConvexScene::destroyScene()
{
	delete m_raycaster;
	m_raycaster = 0;
}

int	GpuConvexScene::createDynamicsObjects(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(barrel_vertices)/strideInBytes;
	int numIndices = sizeof(barrel_indices)/sizeof(int);
	return createDynamicsObjects2(ci,barrel_vertices,numVertices,barrel_indices,numIndices);
}

int	GpuBoxPlaneScene::createDynamicsObjects(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	return createDynamicsObjects2(ci,cube_vertices,numVertices,cube_indices,numIndices);
}


int	GpuConvexScene::createDynamicsObjects2(const ConstructionInfo& ci, const float* vertices, int numVertices, const int* indices, int numIndices)
{
	int strideInBytes = 9*sizeof(float);

	int shapeId = ci.m_instancingRenderer->registerShape(&vertices[0],numVertices,indices,numIndices);
	int group=1;
	int mask=1;
	int index=0;





	{
		b3Vector4 colors[4] =
		{
			b3Vector4(1,0,0,1),
			b3Vector4(0,1,0,1),
			b3Vector4(0,1,1,1),
			b3Vector4(1,1,0,1),
		};

		int curColor = 0;
		float scaling[4] = {1,1,1,1};
		int prevBody = -1;
		int insta = 0;

		b3ConvexUtility* utilPtr = new b3ConvexUtility();

		{
			b3AlignedObjectArray<b3Vector3> verts;

			unsigned char* vts = (unsigned char*) vertices;
			for (int i=0;i<numVertices;i++)
			{
				float* vertex = (float*) &vts[i*strideInBytes];
				verts.push_back(b3Vector3(vertex[0]*scaling[0],vertex[1]*scaling[1],vertex[2]*scaling[2]));
			}

			bool merge = true;
			if (numVertices)
			{
				utilPtr->initializePolyhedralFeatures(&verts[0],verts.size(),merge);
			}
		}

		int colIndex=-1;
		if (ci.m_useInstancedCollisionShapes)
			colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

		//int colIndex = m_data->m_np->registerSphereShape(1);
		for (int i=0;i<ci.arraySizeX;i++)
		{


			//printf("%d of %d\n", i, ci.arraySizeX);
			for (int j=0;j<ci.arraySizeY;j++)
			{

				for (int k=0;k<ci.arraySizeZ;k++)
				{
					//int colIndex = m_data->m_np->registerConvexHullShape(&vertices[0],strideInBytes,numVertices, scaling);
					if (!ci.m_useInstancedCollisionShapes)
						colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

					float mass = 1.f;
					if (j==0)//ci.arraySizeY-1)
					{
						//mass=0.f;
					}
					b3Vector3 position((j&1)+i*2.2,1+j*2.,(j&1)+k*2.2);
					//b3Vector3 position(i*2.2,10+j*1.9,k*2.2);

					b3Quaternion orn(0,0,0,1);

					b3Vector4 color = colors[curColor];
					curColor++;
					curColor&=3;
					b3Vector4 scaling(1,1,1,1);
					int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
					int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index,false);


					if (prevBody>=0)
					{
						//b3Point2PointConstraint* p2p = new b3Point2PointConstraint(pid,prevBody,b3Vector3(0,-1.1,0),b3Vector3(0,1.1,0));
//						 m_data->m_rigidBodyPipeline->addConstraint(p2p);//,false);
					}
					prevBody = pid;

					index++;
				}
			}
		}
	}
	return index;
}


void GpuConvexScene::createStaticEnvironment(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	int index=0;


	{
		b3Vector4 scaling(400,400,400,1);
		int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		b3Vector3 position(0,-400,0);
		b3Quaternion orn(0,0,0,1);

		b3Vector4 color(0,0,1,1);

		int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
		int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f,position,orn,colIndex,index,false);

	}
}

void GpuConvexPlaneScene::createStaticEnvironment(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	int index=0;


	{
		b3Vector4 scaling(400,400,400,1);
		int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		b3Vector3 position(0,-400,0);
		b3Quaternion orn(0,0,0,1);

		b3Vector4 color(0,0,1,1);

		int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
		int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f,position,orn,colIndex,index,false);

	}

}

