#include "b3GpuRigidBodyPipeline.h"
#include "b3GpuRigidBodyPipelineInternalData.h"
#include "../kernels/integrateKernel.h"
#include "../kernels/updateAabbsKernel.h"

#include "../../basic_initialize/b3OpenCLUtils.h"
#include "b3GpuNarrowPhase.h"
#include "Bullet3Geometry/b3AabbUtil.h"
#include "../../gpu_broadphase/host/b3SapAabb.h"
#include "../../gpu_broadphase/host/b3GpuSapBroadphase.h"
#include "parallel_primitives/host/btLauncherCL.h"
#include "Bullet3Dynamics/ConstraintSolver/b3PgsJacobiSolver.h"


//#define TEST_OTHER_GPU_SOLVER

#ifdef TEST_OTHER_GPU_SOLVER
#include "btGpuJacobiSolver.h"
#include "b3PgsJacobiSolver.h"
#endif //TEST_OTHER_GPU_SOLVER

#include "Bullet3Collision/NarrowPhaseCollision/b3RigidBodyCL.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Contact4.h"
#include "b3GpuBatchingPgsSolver.h"
#include "b3Solver.h"

#include "Bullet3Common/b3Quickprof.h"
#include "b3Config.h"

bool dumpContactStats = false;

b3GpuRigidBodyPipeline::b3GpuRigidBodyPipeline(cl_context ctx,cl_device_id device, cl_command_queue  q,class b3GpuNarrowPhase* narrowphase, class b3GpuSapBroadphase* broadphaseSap )
{
	m_data = new b3GpuRigidBodyPipelineInternalData;
	m_data->m_context = ctx;
	m_data->m_device = device;
	m_data->m_queue = q;

	m_data->m_solver = new b3PgsJacobiSolver();

	
#ifdef TEST_OTHER_GPU_SOLVER
	m_data->m_solver3 = new btGpuJacobiSolver(ctx,device,q,config.m_maxBroadphasePairs);	
#endif //	TEST_OTHER_GPU_SOLVER
	b3Config config;
	m_data->m_solver2 = new b3GpuBatchingPgsSolver(ctx,device,q,config.m_maxBroadphasePairs);

	
	
	m_data->m_broadphaseSap = broadphaseSap;
	m_data->m_narrowphase = narrowphase;

	cl_int errNum=0;

	{
		cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context,m_data->m_device,integrateKernelCL,&errNum,"","opencl/gpu_rigidbody/kernels/integrateKernel.cl");
		btAssert(errNum==CL_SUCCESS);
		m_data->m_integrateTransformsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,integrateKernelCL, "integrateTransformsKernel",&errNum,prog);
		btAssert(errNum==CL_SUCCESS);
		clReleaseProgram(prog);
	}
	{
		cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context,m_data->m_device,updateAabbsKernelCL,&errNum,"","opencl/gpu_rigidbody/kernels/updateAabbsKernel.cl");
		btAssert(errNum==CL_SUCCESS);
		m_data->m_updateAabbsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,updateAabbsKernelCL, "initializeGpuAabbsFull",&errNum,prog);
		btAssert(errNum==CL_SUCCESS);
		clReleaseProgram(prog);
	}


}

b3GpuRigidBodyPipeline::~b3GpuRigidBodyPipeline()
{
	clReleaseKernel(m_data->m_integrateTransformsKernel);

	delete m_data->m_solver;

#ifdef TEST_OTHER_GPU_SOLVER
	delete m_data->m_solver3;
#endif //TEST_OTHER_GPU_SOLVER
	
	delete m_data->m_solver2;
	
	
	delete m_data;
}

void	b3GpuRigidBodyPipeline::stepSimulation(float deltaTime)
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

		if (dumpContactStats && numContacts)
		{
			m_data->m_narrowphase->getContactsGpu();
			
			printf("numContacts = %d\n", numContacts);

			int totalPoints  = 0;
			const b3Contact4* contacts = m_data->m_narrowphase->getContactsCPU();

			for (int i=0;i<numContacts;i++)
			{
				totalPoints += contacts->getNPoints();
			}
			printf("totalPoints=%d\n",totalPoints);

		}
	}
	

	//convert contact points to contact constraints
	
	//solve constraints
	
	if (numContacts)
	{
		btOpenCLArray<b3RigidBodyCL> gpuBodies(m_data->m_context,m_data->m_queue,0,true);
		gpuBodies.setFromOpenCLBuffer(m_data->m_narrowphase->getBodiesGpu(),m_data->m_narrowphase->getNumBodiesGpu());
		btOpenCLArray<btInertiaCL> gpuInertias(m_data->m_context,m_data->m_queue,0,true);
		gpuInertias.setFromOpenCLBuffer(m_data->m_narrowphase->getBodyInertiasGpu(),m_data->m_narrowphase->getNumBodiesGpu());
		btOpenCLArray<b3Contact4> gpuContacts(m_data->m_context,m_data->m_queue,0,true);
		gpuContacts.setFromOpenCLBuffer(m_data->m_narrowphase->getContactsGpu(),m_data->m_narrowphase->getNumContactsGpu());

		bool useBullet2CpuSolver = false;
		if (useBullet2CpuSolver)
		{
			b3AlignedObjectArray<b3RigidBodyCL> hostBodies;
			gpuBodies.copyToHost(hostBodies);
			b3AlignedObjectArray<btInertiaCL> hostInertias;
			gpuInertias.copyToHost(hostInertias);
			b3AlignedObjectArray<b3Contact4> hostContacts;
			gpuContacts.copyToHost(hostContacts);
			{
				m_data->m_solver->solveContacts(m_data->m_narrowphase->getNumBodiesGpu(),&hostBodies[0],&hostInertias[0],numContacts,&hostContacts[0]);
			}
			gpuBodies.copyFromHost(hostBodies);
		} else
#ifdef TEST_OTHER_GPU_SOLVER
		if (useJacobi)
		{
			bool useGpu = true;
			if (useGpu)
			{
				bool forceHost = false;
				if (forceHost)
				{
					b3AlignedObjectArray<b3RigidBodyCL> hostBodies;
					b3AlignedObjectArray<btInertiaCL> hostInertias;
					b3AlignedObjectArray<b3Contact4> hostContacts;
				
					{
						BT_PROFILE("copyToHost");
						gpuBodies.copyToHost(hostBodies);
						gpuInertias.copyToHost(hostInertias);
						gpuContacts.copyToHost(hostContacts);
					}

					{
						btJacobiSolverInfo solverInfo;
						m_data->m_solver3->solveGroupHost(&hostBodies[0], &hostInertias[0], hostBodies.size(),&hostContacts[0],hostContacts.size(),0,0,solverInfo);

						
					}
					{
						BT_PROFILE("copyFromHost");
						gpuBodies.copyFromHost(hostBodies);
					}
				} else
				{
					btJacobiSolverInfo solverInfo;
					m_data->m_solver3->solveGroup(&gpuBodies, &gpuInertias, &gpuContacts,solverInfo);
				}
			} else
			{
				b3AlignedObjectArray<b3RigidBodyCL> hostBodies;
				gpuBodies.copyToHost(hostBodies);
				b3AlignedObjectArray<btInertiaCL> hostInertias;
				gpuInertias.copyToHost(hostInertias);
				b3AlignedObjectArray<b3Contact4> hostContacts;
				gpuContacts.copyToHost(hostContacts);
				{
					m_data->m_solver->solveContacts(m_data->m_narrowphase->getNumBodiesGpu(),&hostBodies[0],&hostInertias[0],numContacts,&hostContacts[0]);
				}
				gpuBodies.copyFromHost(hostBodies);
			}
		
		} else
#endif //TEST_OTHER_GPU_SOLVER
		{
			b3Config config;
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

void	b3GpuRigidBodyPipeline::integrate(float timeStep)
{
	//integrate

	btLauncherCL launcher(m_data->m_queue,m_data->m_integrateTransformsKernel);
	launcher.setBuffer(m_data->m_narrowphase->getBodiesGpu());
	int numBodies = m_data->m_narrowphase->getNumBodiesGpu();
	launcher.setConst(numBodies);
	launcher.setConst(timeStep);
	float angularDamp = 0.99f;
	launcher.setConst(angularDamp);
	
	b3Vector3 gravity(0.f,-9.8f,0.f);
	launcher.setConst(gravity);

	launcher.launch1D(numBodies);
}



void	b3GpuRigidBodyPipeline::setupGpuAabbsFull()
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



cl_mem	b3GpuRigidBodyPipeline::getBodyBuffer()
{
	return m_data->m_narrowphase->getBodiesGpu();
}

int	b3GpuRigidBodyPipeline::getNumBodies() const
{
	return m_data->m_narrowphase->getNumBodiesGpu();
}





int		b3GpuRigidBodyPipeline::registerPhysicsInstance(float mass, const float* position, const float* orientation, int collidableIndex, int userIndex)
{
	b3Vector3 aabbMin(0,0,0),aabbMax(0,0,0);
	if (collidableIndex>=0)
	{
		b3SapAabb localAabb = m_data->m_narrowphase->getLocalSpaceAabb(collidableIndex);
		b3Vector3 localAabbMin(localAabb.m_min[0],localAabb.m_min[1],localAabb.m_min[2]);
		b3Vector3 localAabbMax(localAabb.m_max[0],localAabb.m_max[1],localAabb.m_max[2]);
		
		b3Scalar margin = 0.01f;
		b3Transform t;
		t.setIdentity();
		t.setOrigin(b3Vector3(position[0],position[1],position[2]));
		t.setRotation(b3Quaternion(orientation[0],orientation[1],orientation[2],orientation[3]));
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
