#include "PairBench.h"
#include "Bullet3Common/b3Quickprof.h"
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

static b3KeyboardCallback oldCallback = 0;
extern bool gReset;

#define MSTRINGIFY(A) #A

static const char* s_pairBenchKernelString = MSTRINGIFY(
__kernel void moveObjectsKernel(__global float4* posOrnColors, int numObjects)
{
	int iGID = get_global_id(0);
	if (iGID>=numObjects)
		return;
	__global float4* positions = &posOrnColors[0];
	if (iGID<0.5*numObjects)
	{
		positions[iGID].y +=0.01f;
	}
	__global float4* colors = &posOrnColors[numObjects*2];
	colors[iGID] = (float4)(0,0,1,1);
}

__kernel void colorPairsKernel(__global float4* posOrnColors, int numObjects, __global const int2* pairs, int numPairs)
{
	int iPairId = get_global_id(0);
	if (iPairId>=numPairs)
		return;
	__global float4* colors = &posOrnColors[numObjects*2];

	int iObjectA = pairs[iPairId].x;
	int iObjectB = pairs[iPairId].y;
	colors[iObjectA] = (float4)(1,0,0,1);
	colors[iObjectB] = (float4)(1,0,0,1);
}

__kernel void 
  sineWaveKernel( __global float4* posOrnColors, __global float* pBodyTimes,const int numNodes)
{
	int nodeID = get_global_id(0);
	float timeStepPos = 0.000166666;
	float mAmplitude = 26.f;
	if( nodeID < numNodes )
	{
		pBodyTimes[nodeID] += timeStepPos;
		float4 position = posOrnColors[nodeID];
		position.x = native_cos(pBodyTimes[nodeID]*2.17f)*mAmplitude + native_sin(pBodyTimes[nodeID])*mAmplitude*0.5f;
		position.y = native_cos(pBodyTimes[nodeID]*1.38f)*mAmplitude + native_sin(pBodyTimes[nodeID]*mAmplitude);
		position.z = native_cos(pBodyTimes[nodeID]*2.17f)*mAmplitude + native_sin(pBodyTimes[nodeID]*0.777f)*mAmplitude;
		
		posOrnColors[nodeID] = position;
		__global float4* colors = &posOrnColors[numNodes*2];
		colors[nodeID] = (float4)(0,0,1,1);
	}
}

typedef struct 
{
	float			fx;
	float			fy;
	float			fz;
	int	uw;
} b3AABBCL;

__kernel void updateAabbSimple( __global float4* posOrnColors, const int numNodes, __global b3AABBCL* pAABB)
{
	int nodeId = get_global_id(0);
	if( nodeId < numNodes )
	{
		float4 position = posOrnColors[nodeId];
		float4 halfExtents = (float4)(1.01f,1.01f,1.01f,0.f);
		pAABB[nodeId*2].fx = position.x-halfExtents.x;
		pAABB[nodeId*2].fy = position.y-halfExtents.y;
		pAABB[nodeId*2].fz = position.z-halfExtents.z;
		pAABB[nodeId*2].uw = nodeId;
		pAABB[nodeId*2+1].fx = position.x+halfExtents.x;
		pAABB[nodeId*2+1].fy = position.y+halfExtents.y;
		pAABB[nodeId*2+1].fz = position.z+halfExtents.z;
		pAABB[nodeId*2+1].uw = nodeId;		
	}
}

);


struct	PairBenchInternalData
{
	b3GpuSapBroadphase*	m_broadphaseGPU;

	cl_kernel	m_moveObjectsKernel;
	cl_kernel	m_sineWaveKernel;
	cl_kernel	m_colorPairsKernel;
	cl_kernel	m_updateAabbSimple;

	b3OpenCLArray<b3Vector4>*	m_instancePosOrnColor;
	b3OpenCLArray<float>*		m_bodyTimes;
	PairBenchInternalData()
		:m_broadphaseGPU(0),
		m_moveObjectsKernel(0),
		m_sineWaveKernel(0),
		m_colorPairsKernel(0),
		m_instancePosOrnColor(0),
		m_bodyTimes(0),
		m_updateAabbSimple(0)
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







static void PairKeyboardCallback(int key, int state)
{
	if (key=='R' && state)
	{
		gReset = true;
	}
	
	//b3DefaultKeyboardCallback(key,state);
	oldCallback(key,state);
}



void	PairBench::initPhysics(const ConstructionInfo& ci)
{
	initCL(ci.preferredOpenCLDeviceIndex,ci.preferredOpenCLPlatformIndex);
	if (m_clData->m_clContext)
	{
		m_data->m_broadphaseGPU = new b3GpuSapBroadphase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);
		cl_program pairBenchProg=0;
		int errNum=0;
		m_data->m_moveObjectsKernel = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,s_pairBenchKernelString,"moveObjectsKernel",&errNum,pairBenchProg);
		m_data->m_sineWaveKernel = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,s_pairBenchKernelString,"sineWaveKernel",&errNum,pairBenchProg);
		m_data->m_colorPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,s_pairBenchKernelString,"colorPairsKernel",&errNum,pairBenchProg);
		m_data->m_updateAabbSimple = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,s_pairBenchKernelString,"updateAabbSimple",&errNum,pairBenchProg);
			
	}

	if (ci.m_window)
	{
		m_window = ci.m_window;
		oldCallback = ci.m_window->getKeyboardCallback();
		ci.m_window->setKeyboardCallback(PairKeyboardCallback);

	}

	m_instancingRenderer = ci.m_instancingRenderer;

	b3ProfileManager::CleanupMemory();
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
				b3Vector3 position(k*3,i*3,j*3);
				b3Quaternion orn(0,0,0,1);
				
				b3Vector4 color(0,1,0,1);
				b3Vector4 scaling(1,1,1,1);
				int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
				b3Vector3 aabbHalfExtents(1,1,1);

				b3Vector3 aabbMin = position-aabbHalfExtents;
				b3Vector3 aabbMax = position+aabbHalfExtents;
				

				m_data->m_broadphaseGPU->createProxy(aabbMin,aabbMax,index,group,mask);
				index++;
			}
		}
	}
	
	float camPos[4]={15.5,12.5,15.5,0};
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(60);

	m_instancingRenderer->writeTransforms();
	m_data->m_broadphaseGPU->writeAabbsToGpu();
}

void	PairBench::exitPhysics()
{
	delete m_data->m_broadphaseGPU;
	delete m_data->m_instancePosOrnColor;
	delete m_data->m_bodyTimes;
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
	int numObjects= m_instancingRenderer->getInternalData()->m_totalNumInstances;
	b3Vector4* positions = 0;
	if (animate)
	{
		GLuint vbo = m_instancingRenderer->getInternalData()->m_vbo;
		
			

		int arraySizeInBytes  = numObjects * (3)*sizeof(b3Vector4);

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		cl_bool blocking=  CL_TRUE;
		char* hostPtr=  (char*)glMapBufferRange( GL_ARRAY_BUFFER,m_instancingRenderer->getMaxShapeCapacity(),arraySizeInBytes, GL_MAP_WRITE_BIT|GL_MAP_READ_BIT );//GL_READ_WRITE);//GL_WRITE_ONLY
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		positions = (b3Vector4*)hostPtr;
		
		if (m_data->m_instancePosOrnColor && m_data->m_instancePosOrnColor->size() != 3*numObjects)
		{
			delete m_data->m_instancePosOrnColor;
			m_data->m_instancePosOrnColor=0;
		}
		if (!m_data->m_instancePosOrnColor)
		{
			m_data->m_instancePosOrnColor = new b3OpenCLArray<b3Vector4>(m_clData->m_clContext,m_clData->m_clQueue);
			m_data->m_instancePosOrnColor->resize(3*numObjects);
			m_data->m_instancePosOrnColor->copyFromHostPointer(positions,3*numObjects,0);
			m_data->m_bodyTimes = new b3OpenCLArray<float>(m_clData->m_clContext,m_clData->m_clQueue);
			m_data->m_bodyTimes ->resize(numObjects);
			b3AlignedObjectArray<float> tmp;
			tmp.resize(numObjects);
			for (int i=0;i<numObjects;i++)
			{
				tmp[i] = float(i)*(1024.f/numObjects);
			}
			m_data->m_bodyTimes->copyFromHost(tmp);
		}

		if (1)
		{
			if (1)
			{
			
				b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_sineWaveKernel);
				launcher.setBuffer(m_data->m_instancePosOrnColor->getBufferCL() );
				launcher.setBuffer(m_data->m_bodyTimes->getBufferCL() );
				launcher.setConst( numObjects);
				launcher.launch1D( numObjects);
				clFinish(m_clData->m_clQueue);
			}
			else
			{
			
				b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_moveObjectsKernel);
				launcher.setBuffer(m_data->m_instancePosOrnColor->getBufferCL() );
				launcher.setConst( numObjects);
				launcher.launch1D( numObjects);
				clFinish(m_clData->m_clQueue);
			}
		}
	}

	{
		b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_updateAabbSimple);
			launcher.setBuffer(m_data->m_instancePosOrnColor->getBufferCL() );
			launcher.setConst( numObjects);
			launcher.setBuffer(m_data->m_broadphaseGPU->getAabbBufferWS());
			launcher.launch1D( numObjects);
			clFinish(m_clData->m_clQueue);
		
	}
	{
		B3_PROFILE("calculateOverlappingPairs");
		m_data->m_broadphaseGPU->calculateOverlappingPairs(64*numObjects);
		//int numPairs = m_data->m_broadphaseGPU->getNumOverlap();
		//printf("numPairs = %d\n", numPairs);
	}
	
	if (animate)
	{
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		//color overlapping objects in red

		
		if (m_data->m_broadphaseGPU->getNumOverlap())
		{
			bool colorPairsOnHost = false;
			if (colorPairsOnHost )
			{

			} else
			{
				int numPairs = m_data->m_broadphaseGPU->getNumOverlap();
				cl_mem pairBuf = m_data->m_broadphaseGPU->getOverlappingPairBuffer();

				b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_colorPairsKernel);
				launcher.setBuffer(m_data->m_instancePosOrnColor->getBufferCL() );
				launcher.setConst( numObjects);
				launcher.setBuffer( pairBuf);
				launcher.setConst( numPairs);
				launcher.launch1D( numPairs);
				clFinish(m_clData->m_clQueue);
			}
		}

		m_data->m_instancePosOrnColor->copyToHostPointer(positions,3*numObjects,0);

		glUnmapBuffer( GL_ARRAY_BUFFER);
		err = glGetError();
		assert(err==GL_NO_ERROR);
	}

}
