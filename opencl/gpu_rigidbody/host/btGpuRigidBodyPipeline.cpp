#include "btGpuRigidBodyPipeline.h"
#include "btGpuRigidBodyPipelineInternalData.h"
#include "../kernels/integrateKernel.h"
#include "../../basic_initialize/btOpenCLUtils.h"
#include "btGpuNarrowPhase.h"
#include "BulletGeometry/btAabbUtil2.h"
#include "../../gpu_broadphase/host/btSapAabb.h"
#include "../../gpu_broadphase/host/btGpuSapBroadphase.h"
#include "parallel_primitives/host/btLauncherCL.h"

btGpuRigidBodyPipeline::btGpuRigidBodyPipeline(cl_context ctx,cl_device_id device, cl_command_queue  q,class btGpuNarrowPhase* narrowphase, class btGpuSapBroadphase* broadphaseSap )
{
	m_data = new btGpuRigidBodyPipelineInternalData;
	m_data->m_context = ctx;
	m_data->m_device = device;
	m_data->m_queue = q;

	m_data->m_broadphaseSap = broadphaseSap;
	m_data->m_narrowphase = narrowphase;

	cl_int errNum=0;

	{
		cl_program prog = btOpenCLUtils::compileCLProgramFromString(m_data->m_context,m_data->m_device,integrateKernelCL,&errNum,"","opencl/gpu_rigidbody/kernels/integrateKernel.cl");
		btAssert(errNum==CL_SUCCESS);
		m_data->m_integrateTransformsKernel = btOpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,integrateKernelCL, "integrateTransformsKernel",&errNum,prog);
		btAssert(errNum==CL_SUCCESS);
		clReleaseProgram(prog);
	}


}

btGpuRigidBodyPipeline::~btGpuRigidBodyPipeline()
{
	clReleaseKernel(m_data->m_integrateTransformsKernel);

	delete m_data;
}

void	btGpuRigidBodyPipeline::stepSimulation(float deltaTime)
{
	btLauncherCL launcher(m_data->m_queue,m_data->m_integrateTransformsKernel);
	//integrateTransformsKernel( __global Body* bodies,const int numNodes, float timeStep, float angularDamping)

	launcher.setBuffer(m_data->m_narrowphase->getBodiesGpu());
	int numBodies = m_data->m_narrowphase->getNumBodiesGpu();
	launcher.setConst(numBodies);
	float timeStep = 1./60.f;
	launcher.setConst(timeStep);
	float angularDamp = 0.99f;
	launcher.setConst(angularDamp);
	launcher.launch1D(numBodies);

}


cl_mem	btGpuRigidBodyPipeline::getBodyBuffer()
{
	return m_data->m_narrowphase->getBodiesGpu();
}

int	btGpuRigidBodyPipeline::getNumBodies() const
{
	return m_data->m_narrowphase->getNumBodiesGpu();
}



int		btGpuRigidBodyPipeline::registerConvexPolyhedron(btConvexUtility* utilPtr)
{
/*
	int collidableIndex = m_narrowphaseAndSolver->allocateCollidable();

	btCollidable& col = m_narrowphaseAndSolver->getCollidableCpu(collidableIndex);
	col.m_shapeType = CollisionShape::SHAPE_CONVEX_HULL;
	col.m_shapeIndex = -1;
	

	if (m_narrowphaseAndSolver)
	{
		btVector3 localCenter(0,0,0);
		for (int i=0;i<utilPtr->m_vertices.size();i++)
			localCenter+=utilPtr->m_vertices[i];
		localCenter*= (1.f/utilPtr->m_vertices.size());
		utilPtr->m_localCenter = localCenter;

		col.m_shapeIndex = m_narrowphaseAndSolver->registerConvexHullShape(utilPtr,col);
	}

	if (col.m_shapeIndex>=0)
	{
		btAABBHost aabbMin, aabbMax;
		btVector3 myAabbMin(1e30f,1e30f,1e30f);
		btVector3 myAabbMax(-1e30f,-1e30f,-1e30f);

		for (int i=0;i<utilPtr->m_vertices.size();i++)
		{
			myAabbMin.setMin(utilPtr->m_vertices[i]);
			myAabbMax.setMax(utilPtr->m_vertices[i]);
		}
		aabbMin.fx = myAabbMin[0];//s_convexHeightField->m_aabb.m_min.x;
		aabbMin.fy = myAabbMin[1];//s_convexHeightField->m_aabb.m_min.y;
		aabbMin.fz= myAabbMin[2];//s_convexHeightField->m_aabb.m_min.z;
		aabbMin.uw = 0;

		aabbMax.fx = myAabbMax[0];//s_convexHeightField->m_aabb.m_max.x;
		aabbMax.fy = myAabbMax[1];//s_convexHeightField->m_aabb.m_max.y;
		aabbMax.fz= myAabbMax[2];//s_convexHeightField->m_aabb.m_max.z;
		aabbMax.uw = 0;

		m_data->m_localShapeAABBCPU->push_back(aabbMin);
		m_data->m_localShapeAABBGPU->push_back(aabbMin);

		m_data->m_localShapeAABBCPU->push_back(aabbMax);
		m_data->m_localShapeAABBGPU->push_back(aabbMax);

		//m_data->m_localShapeAABB->copyFromHostPointer(&aabbMin,1,shapeIndex*2);
		//m_data->m_localShapeAABB->copyFromHostPointer(&aabbMax,1,shapeIndex*2+1);
		clFinish(g_cqCommandQue);
	}

	delete[] eqn;
	
	
	
	return collidableIndex;
	*/
	return 0;
}


int		btGpuRigidBodyPipeline::registerPhysicsInstance(float mass, const float* position, const float* orientation, int collidableIndex, int userIndex)
{
	btVector3 aabbMin(0,0,0),aabbMax(0,0,0);
	if (collidableIndex>=0)
	{
		btSapAabb localAabb = m_data->m_narrowphase->getLocalSpaceAabb(collidableIndex);
		btVector3 localAabbMin(localAabb.m_min[0],localAabb.m_min[1],localAabb.m_min[2]);
		btVector3 localAabbMax(localAabb.m_max[0],localAabb.m_max[1],localAabb.m_max[2]);
		
		btScalar margin = 0.01f;
		btTransform t;
		t.setIdentity();
		t.setOrigin(btVector3(position[0],position[1],position[2]));
		t.setRotation(btQuaternion(orientation[0],orientation[1],orientation[2],orientation[3]));
		btTransformAabb(localAabbMin,localAabbMax, margin,t,aabbMin,aabbMax);
		if (mass)
		{
			m_data->m_broadphaseSap->createProxy(aabbMin,aabbMax,userIndex,1,1);//m_dispatcher);
		} else
		{
			m_data->m_broadphaseSap->createLargeProxy(aabbMin,aabbMax,userIndex,1,1);//m_dispatcher);	
		}
	}
			
	bool writeToGpu = false;
	int bodyIndex = -1;


	bodyIndex = m_data->m_narrowphase->registerRigidBody(collidableIndex,mass,position,orientation,&aabbMin.getX(),&aabbMax.getX(),writeToGpu);

	/*
	if (mass>0.f)
		m_numDynamicPhysicsInstances++;

	m_numPhysicsInstances++;
	*/

	return bodyIndex;
}