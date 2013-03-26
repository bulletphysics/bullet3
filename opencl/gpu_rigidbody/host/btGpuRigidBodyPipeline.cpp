#include "btGpuRigidBodyPipeline.h"
#include "btGpuRigidBodyPipelineInternalData.h"
#include "../kernels/integrateKernel.h"
#include "../kernels/updateAabbsKernel.h"

#include "../../basic_initialize/btOpenCLUtils.h"
#include "btGpuNarrowPhase.h"
#include "BulletGeometry/btAabbUtil2.h"
#include "../../gpu_broadphase/host/btSapAabb.h"
#include "../../gpu_broadphase/host/btGpuSapBroadphase.h"
#include "parallel_primitives/host/btLauncherCL.h"
#include "btPgsJacobiSolver.h"
#include "../../gpu_sat/host/btRigidBodyCL.h"
#include "../../gpu_sat/host/btContact4.h"
#include "btGpuBatchingPgsSolver.h"
#include "Solver.h"
#include "btGpuJacobiSolver.h"

#include "btConfig.h"

btGpuRigidBodyPipeline::btGpuRigidBodyPipeline(cl_context ctx,cl_device_id device, cl_command_queue  q,class btGpuNarrowPhase* narrowphase, class btGpuSapBroadphase* broadphaseSap )
{
	m_data = new btGpuRigidBodyPipelineInternalData;
	m_data->m_context = ctx;
	m_data->m_device = device;
	m_data->m_queue = q;
	m_data->m_solver = new btPgsJacobiSolver();
	btConfig config;
	
	m_data->m_solver2 = new btGpuBatchingPgsSolver(ctx,device,q,config.m_maxBroadphasePairs);
	m_data->m_solver3 = new btGpuJacobiSolver(ctx,device,q,config.m_maxBroadphasePairs);
	
	
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
	{
		cl_program prog = btOpenCLUtils::compileCLProgramFromString(m_data->m_context,m_data->m_device,updateAabbsKernelCL,&errNum,"","opencl/gpu_rigidbody/kernels/updateAabbsKernel.cl");
		btAssert(errNum==CL_SUCCESS);
		m_data->m_updateAabbsKernel = btOpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,updateAabbsKernelCL, "initializeGpuAabbsFull",&errNum,prog);
		btAssert(errNum==CL_SUCCESS);
		clReleaseProgram(prog);
	}


}

btGpuRigidBodyPipeline::~btGpuRigidBodyPipeline()
{
	clReleaseKernel(m_data->m_integrateTransformsKernel);
	
	delete m_data->m_solver;
	delete m_data->m_solver2;
	delete m_data->m_solver3;
	delete m_data;
}

void	btGpuRigidBodyPipeline::stepSimulation(float deltaTime)
{

	//update worldspace AABBs from local AABB/worldtransform
	{
		setupGpuAabbsFull();
	}

	//compute overlapping pairs
	{
		//m_data->m_broadphaseSap->calculateOverlappingPairsHost();
		m_data->m_broadphaseSap->calculateOverlappingPairs();
	}

	//compute contact points
	
	int numPairs = m_data->m_broadphaseSap->getNumOverlap();
	int numContacts  = 0;

	int numBodies = m_data->m_narrowphase->getNumBodiesGpu();

	if (numPairs)
	{
		cl_mem pairs = m_data->m_broadphaseSap->getOverlappingPairBuffer();
		cl_mem aabbsWS = m_data->m_broadphaseSap->getAabbBufferWS();
		

		m_data->m_narrowphase->computeContacts(pairs,numPairs,aabbsWS,numBodies);
		numContacts = m_data->m_narrowphase->getNumContactsGpu();
		//if (numContacts)
		//	printf("numContacts = %d\n", numContacts);
	}
	

	//convert contact points to contact constraints
	
	//solve constraints
	
	if (numContacts)
	{
		btOpenCLArray<btRigidBodyCL> gpuBodies(m_data->m_context,m_data->m_queue,0,true);
		gpuBodies.setFromOpenCLBuffer(m_data->m_narrowphase->getBodiesGpu(),m_data->m_narrowphase->getNumBodiesGpu());
		btOpenCLArray<btInertiaCL> gpuInertias(m_data->m_context,m_data->m_queue,0,true);
		gpuInertias.setFromOpenCLBuffer(m_data->m_narrowphase->getBodyInertiasGpu(),m_data->m_narrowphase->getNumBodiesGpu());
		btOpenCLArray<btContact4> gpuContacts(m_data->m_context,m_data->m_queue,0,true);
		gpuContacts.setFromOpenCLBuffer(m_data->m_narrowphase->getContactsGpu(),m_data->m_narrowphase->getNumContactsGpu());

		bool useJacobi = false;//true;
		if (useJacobi)
		{
			bool useGpu = true;
			if (useGpu)
			{
				btAlignedObjectArray<btRigidBodyCL> hostBodies;
				gpuBodies.copyToHost(hostBodies);
				btAlignedObjectArray<btInertiaCL> hostInertias;
				gpuInertias.copyToHost(hostInertias);
				btAlignedObjectArray<btContact4> hostContacts;
				gpuContacts.copyToHost(hostContacts);
				{
					btJacobiSolverInfo solverInfo;
					m_data->m_solver3->solveGroup(&hostBodies[0], &hostInertias[0], hostBodies.size(),&hostContacts[0],hostContacts.size(),0,0,solverInfo);
				}
				gpuBodies.copyFromHost(hostBodies);
			} else
			{
				btAlignedObjectArray<btRigidBodyCL> hostBodies;
				gpuBodies.copyToHost(hostBodies);
				btAlignedObjectArray<btInertiaCL> hostInertias;
				gpuInertias.copyToHost(hostInertias);
				btAlignedObjectArray<btContact4> hostContacts;
				gpuContacts.copyToHost(hostContacts);
				{
					m_data->m_solver->solveContacts(m_data->m_narrowphase->getNumBodiesGpu(),&hostBodies[0],&hostInertias[0],numContacts,&hostContacts[0]);
				}
				gpuBodies.copyFromHost(hostBodies);
			}
		
		} else
		{
			btConfig config;
			m_data->m_solver2->solveContacts(numBodies, gpuBodies.getBufferCL(),gpuInertias.getBufferCL(),numContacts, gpuContacts.getBufferCL(),config);
			
			//m_data->m_solver4->solveContacts(m_data->m_narrowphase->getNumBodiesGpu(), gpuBodies.getBufferCL(), gpuInertias.getBufferCL(), numContacts, gpuContacts.getBufferCL());
			
			
			/*m_data->m_solver3->solveContactConstraintHost(
			(btOpenCLArray<RigidBodyBase::Body>*)&gpuBodies,
			(btOpenCLArray<RigidBodyBase::Inertia>*)&gpuInertias,
			(btOpenCLArray<Constraint4>*) &gpuContacts,
			0,numContacts,256);
			*/
		}
	}

	integrate(deltaTime);

}

void	btGpuRigidBodyPipeline::integrate(float timeStep)
{
	//integrate

	btLauncherCL launcher(m_data->m_queue,m_data->m_integrateTransformsKernel);
	launcher.setBuffer(m_data->m_narrowphase->getBodiesGpu());
	int numBodies = m_data->m_narrowphase->getNumBodiesGpu();
	launcher.setConst(numBodies);
	launcher.setConst(timeStep);
	float angularDamp = 0.99f;
	launcher.setConst(angularDamp);
	
	btVector3 gravity(0.f,-9.8f,0.f);
	launcher.setConst(gravity);

	launcher.launch1D(numBodies);
}



void	btGpuRigidBodyPipeline::setupGpuAabbsFull()
{
	cl_int ciErrNum=0;

	int numBodies = m_data->m_narrowphase->getNumBodiesGpu();
	if (!numBodies)
		return;

	//__kernel void initializeGpuAabbsFull(  const int numNodes, __global Body* gBodies,__global Collidable* collidables, __global btAABBCL* plocalShapeAABB, __global btAABBCL* pAABB)
	btLauncherCL launcher(m_data->m_queue,m_data->m_updateAabbsKernel);
	launcher.setConst(numBodies);
	cl_mem bodies = m_data->m_narrowphase->getBodiesGpu();
	launcher.setBuffer(bodies);
	cl_mem collidables = m_data->m_narrowphase->getCollidablesGpu();
	launcher.setBuffer(collidables);
	cl_mem localAabbs = m_data->m_narrowphase->getAabbBufferGpu();
	launcher.setBuffer(localAabbs);
	cl_mem worldAabbs = m_data->m_broadphaseSap->getAabbBufferWS();
	launcher.setBuffer(worldAabbs);
	launcher.launch1D(numBodies);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}



cl_mem	btGpuRigidBodyPipeline::getBodyBuffer()
{
	return m_data->m_narrowphase->getBodiesGpu();
}

int	btGpuRigidBodyPipeline::getNumBodies() const
{
	return m_data->m_narrowphase->getNumBodiesGpu();
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
