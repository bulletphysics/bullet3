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

static btKeyboardCallback oldCallback = 0;
extern bool gReset;

#define MSTRINGIFY(A) #A

static const char* s_rigidBodyKernelString = MSTRINGIFY(

typedef struct
{
	float4 m_pos;
	float4 m_quat;
	float4 m_linVel;
	float4 m_angVel;
	unsigned int m_collidableIdx;
	float m_invMass;
	float m_restituitionCoeff;
	float m_frictionCoeff;
} Body;

__kernel void 
	copyTransformsToVBOKernel( __global Body* gBodies, __global float4* posOrnColor, const int numNodes)
{
	int nodeID = get_global_id(0);
	if( nodeID < numNodes )
	{
		posOrnColor[nodeID] = (float4) (gBodies[nodeID].m_pos.xyz,1.0);
		posOrnColor[nodeID + numNodes] = gBodies[nodeID].m_quat;
	}
}
);





GpuRigidBodyDemo::GpuRigidBodyDemo()
:m_instancingRenderer(0),
m_window(0)
{
	m_data = new GpuRigidBodyDemoInternalData;
}
GpuRigidBodyDemo::~GpuRigidBodyDemo()
{
	
	delete m_data;
}







static void PairKeyboardCallback(int key, int state)
{
	if (key=='R' && state)
	{
		gReset = true;
	}
	
	//btDefaultKeyboardCallback(key,state);
	oldCallback(key,state);
}

void	GpuRigidBodyDemo::setupScene(const ConstructionInfo& ci)
{

}

void	GpuRigidBodyDemo::initPhysics(const ConstructionInfo& ci)
{

	if (ci.m_window)
	{
		m_window = ci.m_window;
		oldCallback = ci.m_window->getKeyboardCallback();
		ci.m_window->setKeyboardCallback(PairKeyboardCallback);

	}

	m_instancingRenderer = ci.m_instancingRenderer;

	initCL(ci.preferredOpenCLDeviceIndex,ci.preferredOpenCLPlatformIndex);

	if (m_clData->m_clContext)
	{
		int errNum=0;

		cl_program rbProg=0;
		m_data->m_copyTransformsToVBOKernel = btOpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,s_rigidBodyKernelString,"copyTransformsToVBOKernel",&errNum,rbProg);
		
		btConfig config;
		btGpuNarrowPhase* np = new btGpuNarrowPhase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue,config);
		btGpuSapBroadphase* bp = new btGpuSapBroadphase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);
		m_data->m_np = np;
		m_data->m_bp = bp;

		m_data->m_rigidBodyPipeline = new btGpuRigidBodyPipeline(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue, np, bp);


		setupScene(ci);

		np->writeAllBodiesToGpu();
		bp->writeAabbsToGpu();
	}


	m_instancingRenderer->writeTransforms();
	
	

}

void	GpuRigidBodyDemo::exitPhysics()
{
	delete m_data->m_instancePosOrnColor;
	delete m_data->m_rigidBodyPipeline;

	m_window->setKeyboardCallback(oldCallback);
	
	delete m_data->m_np;
	m_data->m_np = 0;
	delete m_data->m_bp;
	m_data->m_bp = 0;

	exitCL();
}


void GpuRigidBodyDemo::renderScene()
{
	m_instancingRenderer->RenderScene();
}

void GpuRigidBodyDemo::clientMoveAndDisplay()
{
	bool animate=true;
	int numObjects= m_data->m_rigidBodyPipeline->getNumBodies();
//m_instancingRenderer->getInternalData()->m_totalNumInstances;
	btVector4* positions = 0;
	if (animate && numObjects)
	{
		GLuint vbo = m_instancingRenderer->getInternalData()->m_vbo;
		int arraySizeInBytes  = numObjects * (3)*sizeof(btVector4);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		cl_bool blocking=  CL_TRUE;
		positions=  (btVector4*)glMapBufferRange( GL_ARRAY_BUFFER,m_instancingRenderer->getMaxShapeCapacity(),arraySizeInBytes, GL_MAP_WRITE_BIT|GL_MAP_READ_BIT );//GL_READ_WRITE);//GL_WRITE_ONLY
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		if (!m_data->m_instancePosOrnColor)
		{
			m_data->m_instancePosOrnColor = new btOpenCLArray<btVector4>(m_clData->m_clContext,m_clData->m_clQueue);
			m_data->m_instancePosOrnColor->resize(3*numObjects);
			m_data->m_instancePosOrnColor->copyFromHostPointer(positions,3*numObjects,0);
		}
	}
	
	m_data->m_rigidBodyPipeline->stepSimulation(1./60.f);

	if (numObjects)
	{
		int ciErrNum = 0;
		cl_mem bodies = m_data->m_rigidBodyPipeline->getBodyBuffer();
		btLauncherCL launch(m_clData->m_clQueue,m_data->m_copyTransformsToVBOKernel);
		launch.setBuffer(bodies);
		launch.setBuffer(m_data->m_instancePosOrnColor->getBufferCL());
		launch.setConst(numObjects);
		launch.launch1D(numObjects);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}

	if (animate && numObjects)
	{
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		m_data->m_instancePosOrnColor->copyToHostPointer(positions,3*numObjects,0);
		glUnmapBuffer( GL_ARRAY_BUFFER);
		err = glGetError();
		assert(err==GL_NO_ERROR);
	}

}
