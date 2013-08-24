#include "PairBench.h"
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
#include "../../../btgui/Timing/b3Quickprof.h"
#include "../gwenUserInterface.h"
#include <string.h>

static b3KeyboardCallback oldCallback = 0;

char* gPairBenchFileName = 0;

float maxExtents = -1e30f;
int largeCount = 0;

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

__kernel void colorPairsKernel(__global float4* posOrnColors, int numObjects, __global const int4* pairs, int numPairs)
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
	float mAmplitude = 51.f;
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

	GwenUserInterface*	m_gui;
	
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

static inline float parseFloat(const char*& token)
{
  token += strspn(token, " \t");
  float f = (float)atof(token);
  token += strcspn(token, " \t\r");
  return f;
}

extern bool useShadowMap;

void	PairBench::initPhysics(const ConstructionInfo& ci)
{
	useShadowMap = false;

	m_data->m_gui = ci.m_gui;


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

#ifndef B3_NO_PROFILE
	b3ProfileManager::CleanupMemory();
#endif //B3_NO_PROFILE

	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_vertices)/sizeof(int);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	int index=10;

	
	if (gPairBenchFileName)
	{
		
		
		//char* fileName = "32006GPUAABBs.txt";
		char relativeFileName[1024];
		const char* prefix[]={"./data/","../data/","../../data/","../../../data/","../../../../data/"};
		int prefixIndex=-1;
		{
	
			int numPrefixes = sizeof(prefix)/sizeof(char*);

			for (int i=0;i<numPrefixes;i++)
			{
				FILE* f = 0;
				sprintf(relativeFileName,"%s%s",prefix[i],gPairBenchFileName);
				f = fopen(relativeFileName,"rb");
				if (f)
				{
					fseek( f, 0L, SEEK_END );
					int size = ftell( f);
					rewind( f);
					char* buf = (char*)malloc(size);
					
					int actualReadBytes =0;

					while (actualReadBytes<size)
					{	int left =  size-actualReadBytes;
						int chunk = 8192;
						int numPlannedRead= left < chunk? left : chunk;
						actualReadBytes += fread(&buf[actualReadBytes],1,numPlannedRead,f);
					}

					fclose(f);
					

					char pattern[1024];
					pattern[0] = 0x0a;
					pattern[1] = 0;			
					size_t const patlen = strlen(pattern);
  					size_t patcnt = 0;
					char * oriptr;
					char * patloc;

					
					for (oriptr = buf; patloc = strstr(oriptr, pattern); oriptr = patloc + patlen)
					{
						if (patloc)
						{
							*patloc=0;
							const char* token = oriptr;

							b3Vector3 aabbMin;
							b3Vector3 aabbMax;

							aabbMin.x = parseFloat(token);
							aabbMin.y = parseFloat(token);
							aabbMin.z = parseFloat(token);
							aabbMin.w = 0.f;
							aabbMax.x = parseFloat(token);
							aabbMax.y = parseFloat(token);
							aabbMax.z = parseFloat(token);
							aabbMax.w = 0.f;

							aabbMin*=0.1;
							aabbMax*=0.1;

							b3Vector3 extents = aabbMax-aabbMin;
							
							//printf("%s\n", oriptr);

							b3Vector3 position=0.5*(aabbMax+aabbMin);
							b3Quaternion orn(0,0,0,1);
				
							
							b3Vector4 scaling = b3MakeVector4(0.5*extents.x,0.5*extents.y,0.5*extents.z,1);//b3MakeVector4(1,1,1,1);
							
							
							float l = extents.length();
							if (l>500)
							{
								b3Vector4 color=b3MakeVector4(0,1,0,0.1);
								int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
								m_data->m_broadphaseGPU->createLargeProxy(aabbMin,aabbMax,index,group,mask);
							} else
							{
								b3Vector4 color=b3MakeVector4(1,0,0,1);
								int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
								m_data->m_broadphaseGPU->createProxy(aabbMin,aabbMax,index,group,mask);
									index++;
							}

							
						

							patcnt++;
						}
					}
					prefixIndex = i;
					break;
				}
			
			}
			
			if (prefixIndex<0)
			{
				b3Printf("Cannot find %s\n",gPairBenchFileName);
			}
			
		}

		
	}
	else
	{
		for (int i=0;i<ci.arraySizeX;i++)
		{
			for (int j=0;j<ci.arraySizeY;j++)
			{
				for (int k=0;k<ci.arraySizeZ;k++)
				{
					b3Vector3 position=b3MakeVector3(k*3,i*3,j*3);
					b3Quaternion orn(0,0,0,1);
				
					b3Vector4 color=b3MakeVector4(0,1,0,1);
					b3Vector4 scaling=b3MakeVector4(1,1,1,1);
					int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
					b3Vector3 aabbHalfExtents=b3MakeVector3(1,1,1);

					b3Vector3 aabbMin = position-aabbHalfExtents;
					b3Vector3 aabbMax = position+aabbHalfExtents;
				

					m_data->m_broadphaseGPU->createProxy(aabbMin,aabbMax,index,group,mask);
					index++;
				}
			}
		}
	}
	
	float camPos[4]={15.5,12.5,15.5,0};
	m_instancingRenderer->setCameraTargetPosition(camPos);
	if (gPairBenchFileName)
	{
		m_instancingRenderer->setCameraDistance(830);
	} else
	{
		m_instancingRenderer->setCameraDistance(130);
	}

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
	m_instancingRenderer->renderScene();
}

void PairBench::clientMoveAndDisplay()
{
	//color all objects blue

	bool animate=true;
	int numObjects= m_instancingRenderer->getInternalData()->m_totalNumInstances;
	b3Vector4* positions = 0;
	if (numObjects)
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

		if (!gPairBenchFileName)
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

	bool updateOnGpu=false;

	if (updateOnGpu)
	{
		B3_PROFILE("updateOnGpu");
		b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_updateAabbSimple);
			launcher.setBuffer(m_data->m_instancePosOrnColor->getBufferCL() );
			launcher.setConst( numObjects);
			launcher.setBuffer(m_data->m_broadphaseGPU->getAabbBufferWS());
			launcher.launch1D( numObjects);
			clFinish(m_clData->m_clQueue);
		
	} else
	{
		B3_PROFILE("updateOnCpu");
		if (!gPairBenchFileName)
		{
		int allAabbs = m_data->m_broadphaseGPU->m_allAabbsCPU.size();
			

		b3AlignedObjectArray<b3Vector4> posOrnColorsCpu;
		m_data->m_instancePosOrnColor->copyToHost(posOrnColorsCpu);
		
		
		
		for (int nodeId=0;nodeId<numObjects;nodeId++)
		{
			{
				b3Vector3 position = posOrnColorsCpu[nodeId];
				b3Vector3 halfExtents = b3MakeFloat4(1.01f,1.01f,1.01f,0.f);
				m_data->m_broadphaseGPU->m_allAabbsCPU[nodeId].m_minVec = position-halfExtents;
				m_data->m_broadphaseGPU->m_allAabbsCPU[nodeId].m_minIndices[3] = nodeId;
				m_data->m_broadphaseGPU->m_allAabbsCPU[nodeId].m_maxVec = position+halfExtents;
				m_data->m_broadphaseGPU->m_allAabbsCPU[nodeId].m_signedMaxIndices[3]= nodeId;		
			}
		}
		m_data->m_broadphaseGPU->writeAabbsToGpu();
		}

		
		
	}

	unsigned long dt = 0;
	if (numObjects)
	{
		b3Clock cl;
		dt = cl.getTimeMicroseconds();
		B3_PROFILE("calculateOverlappingPairs");
		int sz = sizeof(b3Int4)*64*numObjects;

		m_data->m_broadphaseGPU->calculateOverlappingPairs(16*numObjects);
		//int numPairs = m_data->m_broadphaseGPU->getNumOverlap();
		//printf("numPairs = %d\n", numPairs);
		dt = cl.getTimeMicroseconds()-dt;
	}
	
			
	if (m_data->m_gui)
	{
		int allAabbs = m_data->m_broadphaseGPU->m_allAabbsCPU.size();
		int numOverlap = m_data->m_broadphaseGPU->getNumOverlap();

		float time = dt/1000.f;
		//printf("time = %f\n", time);

		char msg[1024];
		sprintf(msg,"#objects = %d, #overlapping pairs = %d, time = %f ms", allAabbs,numOverlap,time );
		//printf("msg=%s\n",msg);
		m_data->m_gui->setStatusBarMessage(msg,true);
	}


	if (numObjects)
	{
		B3_PROFILE("animate");
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

		if (numObjects)
		{
			m_data->m_instancePosOrnColor->copyToHostPointer(positions,3*numObjects,0);
		}

		glUnmapBuffer( GL_ARRAY_BUFFER);
		err = glGetError();
		assert(err==GL_NO_ERROR);
	}

}
