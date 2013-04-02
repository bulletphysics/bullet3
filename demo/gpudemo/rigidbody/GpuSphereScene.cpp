#include "GpuSphereScene.h"
#include "GpuRigidBodyDemo.h"
#include "BulletCommon/btQuickprof.h"
#include "OpenGLWindow/ShapeData.h"

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "BulletCommon/btQuaternion.h"
#include "OpenGLWindow/btgWindowInterface.h"
#include "gpu_broadphase/host/btGpuSapBroadphase.h"
#include "../GpuDemoInternalData.h"
#include "basic_initialize/btOpenCLUtils.h"
#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"
#include "parallel_primitives/host/btLauncherCL.h"
#include "gpu_rigidbody/host/btGpuRigidBodyPipeline.h"
#include "gpu_rigidbody/host/btGpuNarrowPhase.h"
#include "gpu_rigidbody/host/btConfig.h"
#include "GpuRigidBodyDemoInternalData.h"
#include "../gwenUserInterface.h"




void GpuSphereScene::setupScene(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	
	int group=1;
	int mask=1;
	int index=0;
	
	if (0)
	{
		int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
		btVector4 scaling(400,0.01,400,1);
		//int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		btVector3 normal(0,1,0);
		float constant=0.01;
		
		int colIndex = m_data->m_np->registerPlaneShape(normal,constant);//>registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		btVector4 position(0,0,0,0);
		btQuaternion orn(0,0,0,1);
				
		btVector4 color(0,0,1,1);
		
		int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
		int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f,position,orn,colIndex,index);
				
		index++;
	}

	

	{



		int prevGraphicsShapeIndex = -1;
		float radius = 1;
		if (radius>=100)
		{
			int numVertices = sizeof(detailed_sphere_vertices)/strideInBytes;
			int numIndices = sizeof(detailed_sphere_indices)/sizeof(int);
			prevGraphicsShapeIndex = ci.m_instancingRenderer->registerShape(&detailed_sphere_vertices[0],numVertices,detailed_sphere_indices,numIndices);
		} else
		{
			bool usePointSprites = false;
			if (usePointSprites)
			{
				int numVertices = sizeof(point_sphere_vertices)/strideInBytes;
				int numIndices = sizeof(point_sphere_indices)/sizeof(int);
				prevGraphicsShapeIndex = ci.m_instancingRenderer->registerShape(&point_sphere_vertices[0],numVertices,point_sphere_indices,numIndices,BT_GL_POINTS);
			} else
			{
				if (radius>=10)
				{
					int numVertices = sizeof(medium_sphere_vertices)/strideInBytes;
					int numIndices = sizeof(medium_sphere_indices)/sizeof(int);
					prevGraphicsShapeIndex = ci.m_instancingRenderer->registerShape(&medium_sphere_vertices[0],numVertices,medium_sphere_indices,numIndices);
				} else
				{
					int numVertices = sizeof(low_sphere_vertices)/strideInBytes;
					int numIndices = sizeof(low_sphere_indices)/sizeof(int);
					prevGraphicsShapeIndex = ci.m_instancingRenderer->registerShape(&low_sphere_vertices[0],numVertices,low_sphere_indices,numIndices);
				}
			}
		}




		btVector4 colors[4] = 
	{
		btVector4(1,0,0,1),
		btVector4(0,1,0,1),
		btVector4(0,1,1,1),
		btVector4(1,1,0,1),
	};
		








		int curColor = 0;
		float scaling[4] = {1,1,1,1};
		//int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		int colIndex = m_data->m_np->registerSphereShape(radius);//>registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		for (int i=0;i<ci.arraySizeX;i++)
		{
			for (int j=0;j<ci.arraySizeY;j++)
			{
				for (int k=0;k<ci.arraySizeZ;k++)
				{
					float mass = 1.f;
					if (j==0)
						mass=0.f;

					//btVector3 position((j&1)+i*2.2,2+j*2.,(j&1)+k*2.2);
					btVector3 position(i*2.2,2+j*2.,k*2.2);
					
					btQuaternion orn(0,0,0,1);
				
					btVector4 color = colors[curColor];
					curColor++;
					curColor&=3;
					btVector4 scaling(1,1,1,1);
					int id = ci.m_instancingRenderer->registerGraphicsInstance(prevGraphicsShapeIndex,position,orn,color,scaling);
					int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index);
				
					index++;
				}
			}
		}
	}
	float camPos[4]={ci.arraySizeX,ci.arraySizeY/2,ci.arraySizeZ,0};
	//float camPos[4]={1,12.5,1.5,0};
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(20);


	char msg[1024];
	int numInstances = index;
	sprintf(msg,"Num objects = %d",numInstances);
	ci.m_gui->setStatusBarMessage(msg,true);
}