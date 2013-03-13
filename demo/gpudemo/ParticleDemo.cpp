#include "ParticleDemo.h"

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/ShapeData.h"
#include "basic_initialize/btOpenCLUtils.h"

#define MSTRINGIFY(A) #A
static char* particleKernelsString = 
#include "ParticleKernels.cl"

#define INTEROPKERNEL_SRC_PATH "demo/gpudemo/ParticleKernels.cl"
#include "BulletCommon/btVector3.h"
#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"
#include "parallel_primitives/host/btLauncherCL.h"
//#include "../../opencl/primitives/AdlPrimitives/Math/Math.h"
//#include "../../opencl/broadphase_benchmark/btGridBroadphaseCL.h"
#include "gpu_broadphase/host/btGpuSapBroadphase.h"



#include "BulletCommon/btQuickprof.h"

//1000000 particles
//#define NUM_PARTICLES_X 100
//#define NUM_PARTICLES_Y 100
//#define NUM_PARTICLES_Z 100

//512k particles
//#define NUM_PARTICLES_X 80
//#define NUM_PARTICLES_Y 80
//#define NUM_PARTICLES_Z 80

//256k particles
//#define NUM_PARTICLES_X 60
//#define NUM_PARTICLES_Y 60
//#define NUM_PARTICLES_Z 60

//27k particles
#define NUM_PARTICLES_X 30
#define NUM_PARTICLES_Y 30
#define NUM_PARTICLES_Z 30

	

ATTRIBUTE_ALIGNED16(struct) btSimParams
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btVector3	m_gravity;
	float m_worldMin[4];
	float m_worldMax[4];
	
	float m_particleRad;
	float m_globalDamping;
	float m_boundaryDamping;
	float m_collisionDamping;

	float m_spring;
	float m_shear;
	float m_attraction;
	float m_dummy;

		
	btSimParams()
	{
		m_gravity.setValue(0,-0.03,0.f);
		m_particleRad = 0.023f;
		m_globalDamping = 1.0f;
		m_boundaryDamping = -0.5f;
		m_collisionDamping = 0.025f;//0.02f;
		m_spring = 0.5f;
		m_shear = 0.1f;
		m_attraction = 0.001f;
		m_worldMin[0] = -1.f;
		m_worldMin[1] = -2*m_particleRad;
		m_worldMin[2] =-1.f;

		m_worldMax[0] = 5.f;
		m_worldMax[1] = 5.f;
		m_worldMax[2] = 5.f;

	}
};

struct ParticleInternalData
{
	cl_context m_clContext;
	cl_device_id m_clDevice;
	cl_command_queue m_clQueue;
	cl_kernel m_updatePositionsKernel;
	cl_kernel m_updatePositionsKernel2;

	cl_kernel m_updateAabbsKernel;

	cl_kernel m_collideParticlesKernel;

	btGpuSapBroadphase*	m_broadphaseGPU;
	

	cl_mem		m_clPositionBuffer;

	btAlignedObjectArray<btVector3> m_velocitiesCPU;
	btOpenCLArray<btVector3>*	m_velocitiesGPU;

	btAlignedObjectArray<btSimParams>	m_simParamCPU;
	btOpenCLArray<btSimParams>*	m_simParamGPU;

	bool m_clInitialized;

	ParticleInternalData()
		:m_clInitialized(false),
		m_clPositionBuffer(0),
		m_velocitiesGPU(0),
		m_simParamGPU(0),
		m_updatePositionsKernel(0),
		m_updatePositionsKernel2(0),
		m_updateAabbsKernel(0),
		m_collideParticlesKernel(0)
	{
		m_simParamCPU.resize(1);
	}

	char*	m_clDeviceName;

};


ParticleDemo::ParticleDemo()
:m_instancingRenderer(0)
{
	m_data = new ParticleInternalData;
}

ParticleDemo::~ParticleDemo()
{
	exitCL();

	delete m_data;

}

void ParticleDemo::exitCL()
{
	if (m_data->m_clInitialized)
	{
		m_data->m_clInitialized = false;
		clReleaseCommandQueue(m_data->m_clQueue);
		clReleaseKernel(m_data->m_updatePositionsKernel);
		clReleaseKernel(m_data->m_updatePositionsKernel2);
		clReleaseKernel(m_data->m_updateAabbsKernel);
		clReleaseKernel(m_data->m_collideParticlesKernel);

		clReleaseContext(m_data->m_clContext);
	}
}

void ParticleDemo::initCL(int preferredDeviceIndex, int preferredPlatformIndex)
{
	void* glCtx=0;
	void* glDC = 0;


    
	int ciErrNum = 0;
//#ifdef CL_PLATFORM_INTEL
//	cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
//#else
	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
//#endif

	

//	if (useInterop)
//	{
//		m_data->m_clContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
//	} else
	{
		m_data->m_clContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0,0,preferredDeviceIndex, preferredPlatformIndex);
	}


	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	int numDev = btOpenCLUtils::getNumDevices(m_data->m_clContext);

	if (numDev>0)
	{
		m_data->m_clDevice= btOpenCLUtils::getDevice(m_data->m_clContext,0);
		m_data->m_clQueue = clCreateCommandQueue(m_data->m_clContext, m_data->m_clDevice, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
        
        btOpenCLUtils::printDeviceInfo(m_data->m_clDevice);
		btOpenCLDeviceInfo info;
		btOpenCLUtils::getDeviceInfo(m_data->m_clDevice,&info);
		m_data->m_clDeviceName = info.m_deviceName;
		m_data->m_clInitialized = true;

	}

}


void ParticleDemo::setupScene(const ConstructionInfo& ci)
{

	initCL(ci.preferredOpenCLDeviceIndex,ci.preferredOpenCLPlatformIndex);
	
	int numParticles = NUM_PARTICLES_X*NUM_PARTICLES_Y*NUM_PARTICLES_Z;

	
	int maxObjects = NUM_PARTICLES_X*NUM_PARTICLES_Y*NUM_PARTICLES_Z+1024;
	
	int maxPairsSmallProxy = 32;
	float radius = 3.f*m_data->m_simParamCPU[0].m_particleRad;

	m_data->m_broadphaseGPU = new btGpuSapBroadphase(m_data->m_clContext ,m_data->m_clDevice,m_data->m_clQueue);//overlappingPairCache,btVector3(4.f, 4.f, 4.f), 128, 128, 128,maxObjects, maxObjects, maxPairsSmallProxy, 100.f, 128,

	/*m_data->m_broadphaseGPU = new btGridBroadphaseCl(overlappingPairCache,btVector3(radius,radius,radius), 128, 128, 128,
		maxObjects, maxObjects, maxPairsSmallProxy, 100.f, 128,
			m_data->m_clContext ,m_data->m_clDevice,m_data->m_clQueue);
			*/

	m_data->m_velocitiesGPU = new btOpenCLArray<btVector3>(m_data->m_clContext,m_data->m_clQueue,numParticles);
	m_data->m_velocitiesCPU.resize(numParticles);
	for (int i=0;i<numParticles;i++)
	{
		m_data->m_velocitiesCPU[i].setValue(0,0,0);
	}
	m_data->m_velocitiesGPU->copyFromHost(m_data->m_velocitiesCPU);

	m_data->m_simParamGPU = new btOpenCLArray<btSimParams>(m_data->m_clContext,m_data->m_clQueue,1,false);
	m_data->m_simParamGPU->copyFromHost(m_data->m_simParamCPU);

	cl_int pErrNum;

	cl_program prog = btOpenCLUtils::compileCLProgramFromString(m_data->m_clContext,m_data->m_clDevice,particleKernelsString,0,"",INTEROPKERNEL_SRC_PATH);
	m_data->m_updatePositionsKernel = btOpenCLUtils::compileCLKernelFromString(m_data->m_clContext, m_data->m_clDevice,particleKernelsString, "updatePositionsKernel" ,&pErrNum,prog);
	oclCHECKERROR(pErrNum, CL_SUCCESS);
	m_data->m_updatePositionsKernel2 = btOpenCLUtils::compileCLKernelFromString(m_data->m_clContext, m_data->m_clDevice,particleKernelsString, "integrateMotionKernel" ,&pErrNum,prog);
	oclCHECKERROR(pErrNum, CL_SUCCESS);

	m_data->m_updateAabbsKernel= btOpenCLUtils::compileCLKernelFromString(m_data->m_clContext, m_data->m_clDevice,particleKernelsString, "updateAabbsKernel" ,&pErrNum,prog);
	oclCHECKERROR(pErrNum, CL_SUCCESS);

	m_data->m_collideParticlesKernel = btOpenCLUtils::compileCLKernelFromString(m_data->m_clContext, m_data->m_clDevice,particleKernelsString, "collideParticlesKernel" ,&pErrNum,prog);
	oclCHECKERROR(pErrNum, CL_SUCCESS);

	m_instancingRenderer = ci.m_instancingRenderer;

	int strideInBytes = 9*sizeof(float);
	bool pointSprite = true;
	int shapeId =-1;

	if (pointSprite)
	{
		int numVertices = sizeof(point_sphere_vertices)/strideInBytes;
		int numIndices = sizeof(point_sphere_indices)/sizeof(int);
		shapeId = m_instancingRenderer->registerShape(&point_sphere_vertices[0],numVertices,point_sphere_indices,numIndices,BT_GL_POINTS);
	} else
	{
		int numVertices = sizeof(low_sphere_vertices)/strideInBytes;
		int numIndices = sizeof(low_sphere_indices)/sizeof(int);
		shapeId = m_instancingRenderer->registerShape(&low_sphere_vertices[0],numVertices,low_sphere_indices,numIndices);
	}

	float position[4] = {0,0,0,0};
	float quaternion[4] = {0,0,0,1};
	float color[4]={1,0,0,1};
	float scaling[4] = {0.023,0.023,0.023,1};

	int userIndex = 0;
	for (int x=0;x<NUM_PARTICLES_X;x++)
	{
		for (int y=0;y<NUM_PARTICLES_Y;y++)
		{
			for (int z=0;z<NUM_PARTICLES_Z;z++)
			{
				float rad = m_data->m_simParamCPU[0].m_particleRad;
				position[0] = x*(rad*3);
				position[1] = y*(rad*3);
				position[2] = z*(rad*3);

				color[0] = float(x)/float(NUM_PARTICLES_X);
				color[1] = float(y)/float(NUM_PARTICLES_Y);
				color[2] = float(z)/float(NUM_PARTICLES_Z);

				int id = m_instancingRenderer->registerGraphicsInstance(shapeId,position,quaternion,color,scaling);
				
				void* userPtr = (void*)userIndex;
				int collidableIndex = userIndex;
				btVector3 aabbMin,aabbMax;
				btVector3 particleRadius(rad,rad,rad);

				aabbMin = btVector3(position[0],position[1],position[2])-particleRadius;
				aabbMax = btVector3(position[0],position[1],position[2])+particleRadius;
				m_data->m_broadphaseGPU->createProxy(aabbMin,aabbMax,collidableIndex,1,1);
				userIndex++;

			}
		}
	}
	m_data->m_broadphaseGPU->writeAabbsToGpu();

	float camPos[4]={1.5,0.5,2.5,0};
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(4);
	m_instancingRenderer->writeTransforms();

}

void	ParticleDemo::initPhysics(const ConstructionInfo& ci)
{
	setupScene(ci);
}

void	ParticleDemo::exitPhysics()
{
}

void	ParticleDemo::renderScene()
{
	
	if (m_instancingRenderer)
	{
		m_instancingRenderer->RenderScene();
	}

}


void ParticleDemo::clientMoveAndDisplay()
{
	int numParticles = NUM_PARTICLES_X*NUM_PARTICLES_Y*NUM_PARTICLES_Z;
	GLuint vbo = m_instancingRenderer->getInternalData()->m_vbo;
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glFlush();

	int posArraySize = numParticles*sizeof(float)*4;

	cl_bool blocking=  CL_TRUE;
	char* hostPtr=  (char*)glMapBufferRange( GL_ARRAY_BUFFER,m_instancingRenderer->getMaxShapeCapacity(),posArraySize, GL_MAP_WRITE_BIT|GL_MAP_READ_BIT );//GL_READ_WRITE);//GL_WRITE_ONLY
		GLint err = glGetError();
    assert(err==GL_NO_ERROR);
	glFinish();

	

#if 1



	//do some stuff using the OpenCL buffer

	bool useCpu = false;
	if (useCpu)
	{
		

		float* posBuffer = (float*)hostPtr;
		
		for (int i=0;i<numParticles;i++)
		{
			posBuffer[i*4+1] += 0.1;
		}
	}
	else
	{
		cl_int ciErrNum;
		if (!m_data->m_clPositionBuffer)
		{
			m_data->m_clPositionBuffer = clCreateBuffer(m_data->m_clContext, CL_MEM_READ_WRITE,
				posArraySize, 0, &ciErrNum);

			clFinish(m_data->m_clQueue);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
			ciErrNum = clEnqueueWriteBuffer (	m_data->m_clQueue,m_data->m_clPositionBuffer,
 				blocking,0,posArraySize,hostPtr,0,0,0
			);
			clFinish(m_data->m_clQueue);
		}
	

		



		if (0)
		{
			btBufferInfoCL bInfo[] = { 
				btBufferInfoCL( m_data->m_velocitiesGPU->getBufferCL(), true ),
				btBufferInfoCL( m_data->m_clPositionBuffer)
			};
			
			btLauncherCL launcher(m_data->m_clQueue, m_data->m_updatePositionsKernel );

			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
			launcher.setConst( numParticles);

			launcher.launch1D( numParticles);
			clFinish(m_data->m_clQueue);
	
		}


		if (1)
		{
			btBufferInfoCL bInfo[] = { 
				btBufferInfoCL( m_data->m_clPositionBuffer),
				btBufferInfoCL( m_data->m_velocitiesGPU->getBufferCL() ),
				btBufferInfoCL( m_data->m_simParamGPU->getBufferCL(),true)
			};
			
			btLauncherCL launcher(m_data->m_clQueue, m_data->m_updatePositionsKernel2 );

			launcher.setConst( numParticles);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
			float timeStep = 1.f/60.f;
			launcher.setConst( timeStep);

			launcher.launch1D( numParticles);
			clFinish(m_data->m_clQueue);
	
		}

		{
			btBufferInfoCL bInfo[] = { 
				btBufferInfoCL( m_data->m_clPositionBuffer),
				btBufferInfoCL( m_data->m_broadphaseGPU->getAabbBuffer()),
			};
			
			btLauncherCL launcher(m_data->m_clQueue, m_data->m_updateAabbsKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
			launcher.setConst( m_data->m_simParamCPU[0].m_particleRad);
			launcher.setConst( numParticles);
			
			launcher.launch1D( numParticles);
			clFinish(m_data->m_clQueue);
		}

		//broadphase
		int numPairsGPU=0;
		cl_mem pairsGPU  = 0;

		{
			m_data->m_broadphaseGPU->calculateOverlappingPairs();
			pairsGPU = m_data->m_broadphaseGPU->getOverlappingPairBuffer();
			numPairsGPU = m_data->m_broadphaseGPU->getNumOverlap();
		}

		if (numPairsGPU)
		{
			btBufferInfoCL bInfo[] = { 
				btBufferInfoCL( m_data->m_clPositionBuffer),
				btBufferInfoCL( m_data->m_velocitiesGPU->getBufferCL() ),
				btBufferInfoCL( m_data->m_broadphaseGPU->getOverlappingPairBuffer(),true),
			};
			
			btLauncherCL launcher(m_data->m_clQueue, m_data->m_collideParticlesKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
			launcher.setConst( numPairsGPU);
			launcher.launch1D( numPairsGPU);
			clFinish(m_data->m_clQueue);

			//__kernel void collideParticlesKernel(  __global float4* pPos, __global float4* pVel, __global int2* pairs, const int numPairs)
		}


		if (1)
		{
			ciErrNum = clEnqueueReadBuffer (	m_data->m_clQueue,
				m_data->m_clPositionBuffer,
	 			blocking,
 				0,
 				posArraySize,
 			hostPtr,0,0,0);

			//clReleaseMemObject(clBuffer);
			clFinish(m_data->m_clQueue);

			
		}
	}
	
#endif

	glUnmapBuffer( GL_ARRAY_BUFFER);
	glFlush();

	/*
	int numParticles = NUM_PARTICLES_X*NUM_PARTICLES_Y*NUM_PARTICLES_Z;
	for (int objectIndex=0;objectIndex<numParticles;objectIndex++)
	{
		float pos[4]={0,0,0,0};
		float orn[4]={0,0,0,1};

//		m_instancingRenderer->writeSingleInstanceTransformToGPU(pos,orn,i);
		{
			glBindBuffer(GL_ARRAY_BUFFER, m_instancingRenderer->getInternalData()->m_vbo);
			glFlush();

			char* orgBase =  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_READ_WRITE);
			//btGraphicsInstance* gfxObj = m_graphicsInstances[k];
			int totalNumInstances= numParticles;
	

			int POSITION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);

			char* base = orgBase;
			int capInBytes = m_instancingRenderer->getMaxShapeCapacity();

			float* positions = (float*)(base+capInBytes);
			float* orientations = (float*)(base+capInBytes+ POSITION_BUFFER_SIZE);

			positions[objectIndex*4+1] += 0.1f;
			glUnmapBuffer( GL_ARRAY_BUFFER);
			glFlush();
		}
	}
	*/

	
}

//	m_data->m_positionOffsetInBytes = demo.m_maxShapeBufferCapacity/4;
