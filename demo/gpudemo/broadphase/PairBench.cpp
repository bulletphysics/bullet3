#include "PairBench.h"
#include "BulletCommon/btQuickprof.h"
#include "OpenGLWindow/ShapeData.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "BulletCommon/btQuaternion.h"

PairBench::PairBench()
:m_instancingRenderer(0)
{
}
PairBench::~PairBench()
{
}


void	PairBench::initPhysics(const ConstructionInfo& ci)
{
	m_instancingRenderer = ci.m_instancingRenderer;

	CProfileManager::CleanupMemory();
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_vertices)/sizeof(int);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	for (int i=0;i<ci.arraySizeX;i++)
	{
		for (int j=0;j<ci.arraySizeY;j++)
		{
			for (int k=0;k<ci.arraySizeZ;k++)
			{
				btVector3 position(k*3,i*3,j*3);
				btQuaternion orn(1,0,0,0);
				
				btVector4 color(0,0,1,1);
				btVector4 scaling(1,1,1,1);
				int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
			}
		}
	}
	
		float camPos[4]={15.5,12.5,15.5,0};
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(40);

	m_instancingRenderer->writeTransforms();
}

void	PairBench::exitPhysics()
{
	
}


void PairBench::renderScene()
{
	m_instancingRenderer->RenderScene();
}

void PairBench::clientMoveAndDisplay()
{
	
}
