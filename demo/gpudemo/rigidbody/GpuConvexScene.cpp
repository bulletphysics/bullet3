#include "GpuConvexScene.h"
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

void GpuConvexScene::setupScene(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	int index=10;
	float scaling[4] = {1,1,1,1};

	int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);

		
	for (int i=0;i<ci.arraySizeX;i++)
	{
		for (int j=0;j<ci.arraySizeY;j++)
		{
			for (int k=0;k<ci.arraySizeZ;k++)
			{
				float mass = j==0? 0.f : 1.f;

				btVector3 position(i*ci.gapX,j*ci.gapY,k*ci.gapZ);
				btQuaternion orn(1,0,0,0);
				
				btVector4 color(0,1,0,1);
				btVector4 scaling(1,1,1,1);
				int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
				int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index);
				
				index++;
			}
		}
	}

	float camPos[4]={65.5,4.5,65.5,0};
	//float camPos[4]={1,12.5,1.5,0};
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(90);
	
}