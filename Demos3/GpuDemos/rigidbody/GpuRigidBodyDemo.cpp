#include "GpuRigidBodyDemo.h"
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
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3OpenCL/RigidBody/b3Config.h"
#include "GpuRigidBodyDemoInternalData.h"
#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"

static b3KeyboardCallback oldCallback = 0;
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
	
	//b3DefaultKeyboardCallback(key,state);
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
		m_data->m_copyTransformsToVBOKernel = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,s_rigidBodyKernelString,"copyTransformsToVBOKernel",&errNum,rbProg);
		
		b3Config config;
		b3GpuNarrowPhase* np = new b3GpuNarrowPhase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue,config);
		b3GpuSapBroadphase* bp = new b3GpuSapBroadphase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);
		m_data->m_np = np;
		m_data->m_bp = bp;
		b3DynamicBvhBroadphase* broadphaseDbvt = new b3DynamicBvhBroadphase(config.m_maxConvexBodies);

		m_data->m_rigidBodyPipeline = new b3GpuRigidBodyPipeline(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue, np, bp,broadphaseDbvt,config);


		setupScene(ci);

		m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();
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
	b3Vector4* positions = 0;
	if (animate && numObjects)
	{
		B3_PROFILE("gl2cl");
		
		if (!m_data->m_instancePosOrnColor)
		{
			GLuint vbo = m_instancingRenderer->getInternalData()->m_vbo;
			int arraySizeInBytes  = numObjects * (3)*sizeof(b3Vector4);
			glBindBuffer(GL_ARRAY_BUFFER, vbo);
			cl_bool blocking=  CL_TRUE;
			positions=  (b3Vector4*)glMapBufferRange( GL_ARRAY_BUFFER,m_instancingRenderer->getMaxShapeCapacity(),arraySizeInBytes, GL_MAP_READ_BIT );//GL_READ_WRITE);//GL_WRITE_ONLY
			GLint err = glGetError();
			assert(err==GL_NO_ERROR);
			m_data->m_instancePosOrnColor = new b3OpenCLArray<b3Vector4>(m_clData->m_clContext,m_clData->m_clQueue);
			m_data->m_instancePosOrnColor->resize(3*numObjects);
			m_data->m_instancePosOrnColor->copyFromHostPointer(positions,3*numObjects,0);
			glUnmapBuffer( GL_ARRAY_BUFFER);
			err = glGetError();
			assert(err==GL_NO_ERROR);
		}
	}
	
	{
		B3_PROFILE("stepSimulation");
		m_data->m_rigidBodyPipeline->stepSimulation(1./60.f);
	}

	if (numObjects)
	{
		B3_PROFILE("cl2gl_convert");
		int ciErrNum = 0;
		cl_mem bodies = m_data->m_rigidBodyPipeline->getBodyBuffer();
		b3LauncherCL launch(m_clData->m_clQueue,m_data->m_copyTransformsToVBOKernel);
		launch.setBuffer(bodies);
		launch.setBuffer(m_data->m_instancePosOrnColor->getBufferCL());
		launch.setConst(numObjects);
		launch.launch1D(numObjects);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}

	if (animate && numObjects)
	{
		B3_PROFILE("cl2gl_upload");
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		GLuint vbo = m_instancingRenderer->getInternalData()->m_vbo;
		int arraySizeInBytes  = numObjects * (3)*sizeof(b3Vector4);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		cl_bool blocking=  CL_TRUE;
		positions=  (b3Vector4*)glMapBufferRange( GL_ARRAY_BUFFER,m_instancingRenderer->getMaxShapeCapacity(),arraySizeInBytes, GL_MAP_WRITE_BIT );//GL_READ_WRITE);//GL_WRITE_ONLY
		err = glGetError();
		assert(err==GL_NO_ERROR);
		m_data->m_instancePosOrnColor->copyToHostPointer(positions,3*numObjects,0);
		glUnmapBuffer( GL_ARRAY_BUFFER);
		err = glGetError();
		assert(err==GL_NO_ERROR);
	}

}
