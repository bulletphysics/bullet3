#include "PairBench.h"
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

btKeyboardCallback oldCallback = 0;
extern bool gReset;

#define MSTRINGIFY(A) #A

const char* s_pairBenchKernelString = MSTRINGIFY(
__kernel void moveObjectsKernel(__global float4* posOrnColors, int numObjects)
{
	int iGID = get_global_id(0);
	if (iGID>=numObjects)
		return;
	__global float4* positions = &posOrnColors[0];
	positions[iGID].y +=0.01f;
	__global float4* colors = &posOrnColors[numObjects*2];
	float fi = iGID;
	float c = fi/numObjects;
	colors[iGID] = (float4)(0,0,c,1);

}
);


struct	PairBenchInternalData
{
	btGpuSapBroadphase*	m_broadphaseGPU;

	cl_kernel	m_moveObjectsKernel;
	cl_kernel	m_colorObjectsKernel;
	cl_kernel	m_colorPairsKernel;

	btOpenCLArray<btVector4>*	m_instancePosOrnColor;

	PairBenchInternalData()
		:m_broadphaseGPU(0),
		m_moveObjectsKernel(0),
		m_colorObjectsKernel(0),
		m_colorPairsKernel(0),
		m_instancePosOrnColor(0)
	{
	}
};


PairBench::PairBench()
:m_instancingRenderer(0),
m_window(0)
{
	m_data = new PairBenchInternalData;
}
PairBench::~PairBench()
{
	delete m_data;
}







void PairKeyboardCallback(int key, int state)
{
	if (key=='R' && state)
	{
		gReset = true;
	}
	
	//btDefaultKeyboardCallback(key,state);
	oldCallback(key,state);
}



void	PairBench::initPhysics(const ConstructionInfo& ci)
{
	initCL(ci.preferredOpenCLDeviceIndex,ci.preferredOpenCLPlatformIndex);
	if (m_clData->m_clContext)
	{
		m_data->m_broadphaseGPU = new btGpuSapBroadphase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);
		cl_program pairBenchProg=0;
		int errNum=0;
		m_data->m_moveObjectsKernel = btOpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,s_pairBenchKernelString,"moveObjectsKernel",&errNum,pairBenchProg);

	}

	if (ci.m_window)
	{
		m_window = ci.m_window;
		oldCallback = ci.m_window->getKeyboardCallback();
		ci.m_window->setKeyboardCallback(PairKeyboardCallback);

	}

	m_instancingRenderer = ci.m_instancingRenderer;

	CProfileManager::CleanupMemory();
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_vertices)/sizeof(int);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	int index=10;

	for (int i=0;i<ci.arraySizeX;i++)
	{
		for (int j=0;j<ci.arraySizeY;j++)
		{
			for (int k=0;k<ci.arraySizeZ;k++)
			{
				btVector3 position(k*2,i*2,j*2);
				btQuaternion orn(1,0,0,0);
				
				btVector4 color(0,0,1,1);
				btVector4 scaling(1,1,1,1);
				int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
				btVector3 aabbHalfExtents(1,1,1);

				btVector3 aabbMin = position-aabbHalfExtents;
				btVector3 aabbMax = position+aabbHalfExtents;
				

				m_data->m_broadphaseGPU->createProxy(aabbMin,aabbMax,index,group,mask);
				index++;
			}
		}
	}
	
	float camPos[4]={15.5,12.5,15.5,0};
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(40);

	m_instancingRenderer->writeTransforms();
	m_data->m_broadphaseGPU->writeAabbsToGpu();
}

void	PairBench::exitPhysics()
{
	delete m_data->m_broadphaseGPU;
	delete m_data->m_instancePosOrnColor;
	m_data->m_broadphaseGPU = 0;

	m_window->setKeyboardCallback(oldCallback);
	exitCL();
}


void PairBench::renderScene()
{
	m_instancingRenderer->RenderScene();
}

void PairBench::clientMoveAndDisplay()
{
	//color all objects blue

	bool animate=true;
	if (animate)
	{
		GLuint vbo = m_instancingRenderer->getInternalData()->m_vbo;
		int numObjects= m_instancingRenderer->getInternalData()->m_totalNumInstances;
			

		int arraySizeInBytes  = numObjects * (3)*sizeof(btVector4);

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		cl_bool blocking=  CL_TRUE;
		char* hostPtr=  (char*)glMapBufferRange( GL_ARRAY_BUFFER,m_instancingRenderer->getMaxShapeCapacity(),arraySizeInBytes, GL_MAP_WRITE_BIT|GL_MAP_READ_BIT );//GL_READ_WRITE);//GL_WRITE_ONLY
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);

		btVector4* positions = (btVector4*)hostPtr;
		if (m_data->m_instancePosOrnColor && m_data->m_instancePosOrnColor->size() != 3*numObjects)
		{
			delete m_data->m_instancePosOrnColor;
			m_data->m_instancePosOrnColor=0;
		}
		if (!m_data->m_instancePosOrnColor)
		{
			m_data->m_instancePosOrnColor = new btOpenCLArray<btVector4>(m_clData->m_clContext,m_clData->m_clQueue);
			m_data->m_instancePosOrnColor->resize(3*numObjects);
			m_data->m_instancePosOrnColor->copyFromHostPointer(positions,3*numObjects,0);
		}

		bool animateHost = false;
		if (animateHost)
		{
			for (int i=0;i<numObjects;i++)
			{
				positions[i][1]+=0.1f;
			}
			btVector4* colors= (btVector4*)(hostPtr+8*sizeof(float)*numObjects);
			for (int i=0;i<numObjects;i++)
			{
				colors[i].setValue(0,0,float(i)/float(numObjects),1);
			}
		} else
		{
			
			btLauncherCL launcher(m_clData->m_clQueue, m_data->m_moveObjectsKernel);

			launcher.setBuffer(m_data->m_instancePosOrnColor->getBufferCL() );
			launcher.setConst( numObjects);
			launcher.launch1D( numObjects);
			clFinish(m_clData->m_clQueue);
			m_data->m_instancePosOrnColor->copyToHostPointer(positions,3*numObjects,0);
		}
	}

	{
		BT_PROFILE("calculateOverlappingPairs");
		m_data->m_broadphaseGPU->calculateOverlappingPairs();
		int numPairs = m_data->m_broadphaseGPU->getNumOverlap();
	}
	
	if (animate)
	{
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		//color overlapping objects in red
		glUnmapBuffer( GL_ARRAY_BUFFER);
		err = glGetError();
		assert(err==GL_NO_ERROR);
	}

}
