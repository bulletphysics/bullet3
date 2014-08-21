#include "ParticleDemo.h"

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/ShapeData.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

#define MSTRINGIFY(A) #A
static const char* particleKernelsString =
#include "ParticleKernels.cl"

#define INTEROPKERNEL_SRC_PATH "demo/gpudemo/ParticleKernels.cl"
#include "Bullet3Common/b3Vector3.h"
#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
//#include "../../opencl/primitives/AdlPrimitives/Math/Math.h"
//#include "../../opencl/broadphase_benchmark/b3GridBroadphaseCL.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "GpuDemoInternalData.h"
#include "Bullet3Common/b3Random.h"


//1000000 particles
//#define NUM_PARTICLES_X 100
//#define NUM_PARTICLES_Y 100
//#define NUM_PARTICLES_Z 100

//512k particles
//#define NUM_PARTICLES_X 80
//#define NUM_PARTICLES_Y 80
//#define NUM_PARTICLES_Z 80

//256k particles
#define NUM_PARTICLES_X 60
#define NUM_PARTICLES_Y 60
#define NUM_PARTICLES_Z 60

//27k particles
//#define NUM_PARTICLES_X 30
//#define NUM_PARTICLES_Y 30
//#define NUM_PARTICLES_Z 30



B3_ATTRIBUTE_ALIGNED16(struct) b3SimParams
{
	B3_DECLARE_ALIGNED_ALLOCATOR();
	b3Vector3	m_gravity;
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


	b3SimParams()
	{
		m_gravity.setValue(0,-.3,0.f);
		m_particleRad = 0.01f;
		m_globalDamping = 1.0f;
		m_boundaryDamping = -0.99f;
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

	cl_kernel m_updatePositionsKernel;
	cl_kernel m_updatePositionsKernel2;

	cl_kernel m_updateAabbsKernel;

	cl_kernel m_collideParticlesKernel;

	b3GpuSapBroadphase*	m_broadphaseGPU;


	cl_mem		m_clPositionBuffer;

	b3AlignedObjectArray<b3Vector3> m_velocitiesCPU;
	b3OpenCLArray<b3Vector3>*	m_velocitiesGPU;

	b3AlignedObjectArray<b3SimParams>	m_simParamCPU;
	b3OpenCLArray<b3SimParams>*	m_simParamGPU;



	ParticleInternalData()
		:
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
	if (m_clData->m_clInitialized)
	{
		clReleaseKernel(m_data->m_updatePositionsKernel);
		clReleaseKernel(m_data->m_updatePositionsKernel2);
		clReleaseKernel(m_data->m_updateAabbsKernel);
		clReleaseKernel(m_data->m_collideParticlesKernel);
	}

	GpuDemo::exitCL();
}

void ParticleDemo::initCL(int preferredDeviceIndex, int preferredPlatformIndex)
{
	GpuDemo::initCL(preferredDeviceIndex,preferredPlatformIndex);
}


void ParticleDemo::setupScene(const ConstructionInfo& ci)
{

	initCL(ci.preferredOpenCLDeviceIndex,ci.preferredOpenCLPlatformIndex);

	int numParticles = NUM_PARTICLES_X*NUM_PARTICLES_Y*NUM_PARTICLES_Z;


	int maxObjects = NUM_PARTICLES_X*NUM_PARTICLES_Y*NUM_PARTICLES_Z+1024;

	int maxPairsSmallProxy = 32;
	float radius = m_data->m_simParamCPU[0].m_particleRad;

	m_data->m_broadphaseGPU = new b3GpuSapBroadphase(m_clData->m_clContext ,m_clData->m_clDevice,m_clData->m_clQueue);//overlappingPairCache,b3Vector3(4.f, 4.f, 4.f), 128, 128, 128,maxObjects, maxObjects, maxPairsSmallProxy, 100.f, 128,

	/*m_data->m_broadphaseGPU = new b3GridBroadphaseCl(overlappingPairCache,b3Vector3(radius,radius,radius), 128, 128, 128,
		maxObjects, maxObjects, maxPairsSmallProxy, 100.f, 128,
			m_clData->m_clContext ,m_clData->m_clDevice,m_clData->m_clQueue);
			*/

	m_data->m_velocitiesGPU = new b3OpenCLArray<b3Vector3>(m_clData->m_clContext,m_clData->m_clQueue,numParticles);
	m_data->m_velocitiesCPU.resize(numParticles);
	for (int i=0;i<numParticles;i++)
	{
		m_data->m_velocitiesCPU[i].setValue(0,0,0);
	}
	m_data->m_velocitiesGPU->copyFromHost(m_data->m_velocitiesCPU);

	m_data->m_simParamGPU = new b3OpenCLArray<b3SimParams>(m_clData->m_clContext,m_clData->m_clQueue,1,false);
	m_data->m_simParamGPU->copyFromHost(m_data->m_simParamCPU);

	cl_int pErrNum;

	cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_clData->m_clContext,m_clData->m_clDevice,particleKernelsString,0,"",INTEROPKERNEL_SRC_PATH,true);
	m_data->m_updatePositionsKernel = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext, m_clData->m_clDevice,particleKernelsString, "updatePositionsKernel" ,&pErrNum,prog);
	oclCHECKERROR(pErrNum, CL_SUCCESS);
	m_data->m_updatePositionsKernel2 = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext, m_clData->m_clDevice,particleKernelsString, "integrateMotionKernel" ,&pErrNum,prog);
	oclCHECKERROR(pErrNum, CL_SUCCESS);

	m_data->m_updateAabbsKernel= b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext, m_clData->m_clDevice,particleKernelsString, "updateAabbsKernel" ,&pErrNum,prog);
	oclCHECKERROR(pErrNum, CL_SUCCESS);

	m_data->m_collideParticlesKernel = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext, m_clData->m_clDevice,particleKernelsString, "collideParticlesKernel" ,&pErrNum,prog);
	oclCHECKERROR(pErrNum, CL_SUCCESS);

	m_instancingRenderer = ci.m_instancingRenderer;

	int strideInBytes = 9*sizeof(float);
	bool pointSprite = true;
	int shapeId =-1;

	if (pointSprite)
	{
		int numVertices = sizeof(point_sphere_vertices)/strideInBytes;
		int numIndices = sizeof(point_sphere_indices)/sizeof(int);
		shapeId = m_instancingRenderer->registerShape(&point_sphere_vertices[0],numVertices,point_sphere_indices,numIndices,B3_GL_POINTS);
	} else
	{
		int numVertices = sizeof(low_sphere_vertices)/strideInBytes;
		int numIndices = sizeof(low_sphere_indices)/sizeof(int);
		shapeId = m_instancingRenderer->registerShape(&low_sphere_vertices[0],numVertices,low_sphere_indices,numIndices);
	}

	float position[4] = {0,0,0,0};
	float quaternion[4] = {0,0,0,1};

	float scaling[4] = {radius,radius,radius,1};

	int userIndex = 0;

	int totalParticles = NUM_PARTICLES_X*NUM_PARTICLES_Y*NUM_PARTICLES_Z;

	int curColor = 0;
	b3Vector4 colors[4] =
	{
		b3MakeVector4(1,1,1,1),
		b3MakeVector4(1,1,0.3,1),
		b3MakeVector4(0.3,1,1,1),
		b3MakeVector4(0.3,0.3,1,1),
	};


	{
		srand(1234);
		float angle = b3RandRange(-B3_PI, B3_PI);
		for (int ii=0;ii<totalParticles;ii++)
		{

			float arg = b3RandRange(-B3_PI,B3_PI);

			float rad = m_data->m_simParamCPU[0].m_particleRad;
			position[0] =  arg*b3Cos(arg + angle);
			position[1] = 3.0f + arg;
			position[2] =  arg*b3Sin(arg + angle);

			b3Vector4 color = colors[curColor];
			curColor++;
			curColor&=3;


			int id = m_instancingRenderer->registerGraphicsInstance(shapeId,position,quaternion,color,scaling);


			int collidableIndex = userIndex;
			b3Vector3 aabbMin,aabbMax;
			b3Vector3 particleRadius=b3MakeVector3(rad,rad,rad);

			aabbMin = b3MakeVector3(position[0],position[1],position[2])-particleRadius;
			aabbMax = b3MakeVector3(position[0],position[1],position[2])+particleRadius;
			//m_data->m_broadphaseGPU->createProxy(aabbMin,aabbMax,collidableIndex,1,1);
			userIndex++;
			angle += b3RandRange(-(float)B3_PI, (float)B3_PI);
		}
	}


	m_data->m_broadphaseGPU->writeAabbsToGpu();

	//float camPos[4]={1.5,0.5,2.5,0};
	float camPos[4]={0,0.5,0,0};
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
		m_instancingRenderer->renderScene();
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
			m_data->m_clPositionBuffer = clCreateBuffer(m_clData->m_clContext, CL_MEM_READ_WRITE,
				posArraySize, 0, &ciErrNum);

			clFinish(m_clData->m_clQueue);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
			ciErrNum = clEnqueueWriteBuffer (	m_clData->m_clQueue,m_data->m_clPositionBuffer,
 				blocking,0,posArraySize,hostPtr,0,0,0
			);
			clFinish(m_clData->m_clQueue);
		}






		if (0)
		{
			b3BufferInfoCL bInfo[] = {
				b3BufferInfoCL( m_data->m_velocitiesGPU->getBufferCL(), true ),
				b3BufferInfoCL( m_data->m_clPositionBuffer)
			};

			b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_updatePositionsKernel,"m_updatePositionsKernel" );

			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst( numParticles);

			launcher.launch1D( numParticles);
			clFinish(m_clData->m_clQueue);

		}


		if (1)
		{
			b3BufferInfoCL bInfo[] = {
				b3BufferInfoCL( m_data->m_clPositionBuffer),
				b3BufferInfoCL( m_data->m_velocitiesGPU->getBufferCL() ),
				b3BufferInfoCL( m_data->m_simParamGPU->getBufferCL(),true)
			};

			b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_updatePositionsKernel2 ,"m_updatePositionsKernel2");

			launcher.setConst( numParticles);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			float timeStep = 1.f/60.f;
			launcher.setConst( timeStep);

			launcher.launch1D( numParticles);
			clFinish(m_clData->m_clQueue);

		}

		if (0)
		{
			b3BufferInfoCL bInfo[] = {
				b3BufferInfoCL( m_data->m_clPositionBuffer),
				b3BufferInfoCL( m_data->m_broadphaseGPU->getAabbBufferWS()),
			};

			b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_updateAabbsKernel,"m_updateAabbsKernel" );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst( m_data->m_simParamCPU[0].m_particleRad);
			launcher.setConst( numParticles);

			launcher.launch1D( numParticles);
			clFinish(m_clData->m_clQueue);
		}

		//broadphase
		int numPairsGPU=0;
		cl_mem pairsGPU  = 0;

		{
			//m_data->m_broadphaseGPU->calculateOverlappingPairs(64*numParticles);
			pairsGPU = m_data->m_broadphaseGPU->getOverlappingPairBuffer();
			numPairsGPU = m_data->m_broadphaseGPU->getNumOverlap();
		}

		if (numPairsGPU)
		{
			b3BufferInfoCL bInfo[] = {
				b3BufferInfoCL( m_data->m_clPositionBuffer),
				b3BufferInfoCL( m_data->m_velocitiesGPU->getBufferCL() ),
				b3BufferInfoCL( m_data->m_broadphaseGPU->getOverlappingPairBuffer(),true),
			};

			b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_collideParticlesKernel,"m_collideParticlesKernel");
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst( numPairsGPU);
			launcher.launch1D( numPairsGPU);
			clFinish(m_clData->m_clQueue);

			//__kernel void collideParticlesKernel(  __global float4* pPos, __global float4* pVel, __global int2* pairs, const int numPairs)
		}


		if (1)
		{
			ciErrNum = clEnqueueReadBuffer (	m_clData->m_clQueue,
				m_data->m_clPositionBuffer,
	 			blocking,
 				0,
 				posArraySize,
 			hostPtr,0,0,0);

			//clReleaseMemObject(clBuffer);
			clFinish(m_clData->m_clQueue);


		}
	}

#endif

	glUnmapBuffer( GL_ARRAY_BUFFER);
	glFlush();



}
