/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//#define USE_PARALLEL_SOLVER 1 //experimental parallel solver
#define USE_PARALLEL_DISPATCHER 1

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"

#ifdef USE_PARALLEL_DISPATCHER
#include "../../Extras/BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#ifdef WIN32
#endif //WIN32

#ifdef USE_LIBSPE2
#include "../../Extras/BulletMultiThreaded/SpuLibspe2Support.h"
#elif defined (WIN32)
#include "../../Extras/BulletMultiThreaded/Win32ThreadSupport.h"
#include "../../Extras/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#else
//other platforms run the parallel code sequentially (until pthread support or other parallel implementation is added)
#include "../../Extras/BulletMultiThreaded/SequentialThreadSupport.h"
#include "../../Extras/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif //USE_LIBSPE2

#ifdef USE_PARALLEL_SOLVER
#include "../../Extras/BulletMultiThreaded/SpuParallelSolver.h"
#include "../../Extras/BulletMultiThreaded/SpuSolverTask/SpuParallellSolverTask.h"
#endif //USE_PARALLEL_SOLVER

#endif//USE_PARALLEL_DISPATCHER



#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"


#include "BMF_Api.h"
#include <stdio.h> //printf debugging

#include "MultiThreadedDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"


extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;
const int maxOverlap = 65535;




#ifdef _DEBUG
const int gNumObjects = 120;
#else
const int gNumObjects = 120;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif


const int maxNumObjects = 32760;

static int	shapeIndex[maxNumObjects];


#define CUBE_HALF_EXTENTS 0.5

#define EXTRA_HEIGHT -10.f
//GL_LineSegmentShape shapeE(btPoint3(-50,0,0),
//						   btPoint3(50,0,0));


void MultiThreadedDemo::createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos )
{
	btTransform trans;
	trans.setIdentity();

	for(int i=0; i<size; i++)
	{
		// This constructs a row, from left to right
		int rowSize = size - i;
		for(int j=0; j< rowSize; j++)
		{
			btVector4 pos;
			pos.setValue(
				-rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
				halfCubeSize + i * halfCubeSize * 2.0f,
				zPos);
			
			trans.setOrigin(pos);
			btScalar mass = 1.f;

			btRigidBody* body = 0;
			body = localCreateRigidBody(mass,trans,boxShape);

		}
	}
}


////////////////////////////////////





//experimental jitter damping (1 = no damping, 0 = total damping once motion below threshold)
extern btScalar gJitterVelocityDampingFactor;




extern int gNumManifold;
extern int gOverlappingPairs;
extern int gTotalContactPoints;

void MultiThreadedDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 



	float dt = getDeltaTimeMicroseconds() * 0.000001f;
	
//	printf("dt = %f: ",dt);
	
	if (m_dynamicsWorld)
	{

//#define FIXED_STEP 1
#ifdef FIXED_STEP
  		m_dynamicsWorld->stepSimulation(1.0f/60.f,0);
	
#else
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			dt = 1.0/420.f;

		int numSimSteps = 0;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
		
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();

#ifdef VERBOSE_TIMESTEPPING_CONSOLEOUTPUT
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_TIMESTEPPING_CONSOLEOUTPUT

#endif
	}
	
#ifdef USE_QUICKPROF 
        btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 
	
	renderme(); 


	//render the graphics objects, with center of mass shift

		updateCamera();



#ifdef USE_QUICKPROF 
        btProfiler::endBlock("render"); 
#endif 
	glFlush();

	gTotalContactPoints = 0;
	glutSwapBuffers();

}



void MultiThreadedDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	
	renderme();

	glFlush();
	glutSwapBuffers();
}





void	MultiThreadedDemo::initPhysics()
{
#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	m_threadSupportSolver = 0;
	m_threadSupportCollision = 0;
#endif //
#endif

//#define USE_GROUND_PLANE 1
#ifdef USE_GROUND_PLANE
	m_collisionShapes.push_back(new btStaticPlaneShape(btVector3(0,1,0),0.5));
#else

	///Please don't make the box sizes larger then 1000: the collision detection will be inaccurate.
	///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=346
	m_collisionShapes.push_back(new btBoxShape (btVector3(200,CUBE_HALF_EXTENTS,200)));
#endif

	m_collisionShapes.push_back(new btBoxShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
	


	setCameraDistance(32.5f);

	m_azi = 90.f;

	m_dispatcher=0;
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	
#ifdef USE_PARALLEL_DISPATCHER
int maxNumOutstandingTasks = 4;

#ifdef USE_WIN32_THREADING

m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#else

#ifdef USE_LIBSPE2

   spe_program_handle_t * program_handle;
#ifndef USE_CESOF
                        program_handle = spe_image_open ("./spuCollision.elf");
                        if (program_handle == NULL)
                    {
                                perror( "SPU OPEN IMAGE ERROR\n");
                    }
                        else
                        {
                                printf( "IMAGE OPENED\n");
                        }
#else
                        extern spe_program_handle_t spu_program;
                        program_handle = &spu_program;
#endif
        SpuLibspe2Support* threadSupportCollision  = new SpuLibspe2Support( program_handle, maxNumOutstandingTasks);
#else

	SequentialThreadSupport::SequentialThreadConstructionInfo colCI("collision",processCollisionTask,createCollisionLocalStoreMemory);
	SequentialThreadSupport* m_threadSupportCollision = new SequentialThreadSupport(colCI);
		
#endif //USE_LIBSPE2

///Playstation 3 SPU (SPURS)  version is available through PS3 Devnet
/// For Unix/Mac someone could implement a pthreads version of btThreadSupportInterface?
///you can hook it up to your custom task scheduler by deriving from btThreadSupportInterface
#endif


	m_dispatcher = new	SpuGatheringCollisionDispatcher(m_threadSupportCollision,maxNumOutstandingTasks,m_collisionConfiguration);
//	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#else
	
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER


	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);


	
#ifdef USE_PARALLEL_SOLVER

#ifdef USE_WIN32_THREADING
	m_threadSupportSolver = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"solver",
								processSolverTask,
								createSolverLocalStoreMemory,
								maxNumOutstandingTasks));

#else
	//for now use sequential version	
	SequentialThreadSupport::SequentialThreadConstructionInfo solverCI("solver",processSolverTask,createSolverLocalStoreMemory);
	m_threadSupportSolver = new SequentialThreadSupport(solverCI);

#endif //USE_WIN32_THREADING
	m_solver = new btParallelSequentialImpulseSolver(m_threadSupportSolver,maxNumOutstandingTasks);

#else

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;
	//default solverMode is SOLVER_RANDMIZE_ORDER. Warmstarting seems not to improve convergence, see 
	//solver->setSolverMode(0);//btSequentialImpulseConstraintSolver::SOLVER_USE_WARMSTARTING | btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
	
#endif //USE_PARALLEL_SOLVER

		btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
		m_dynamicsWorld = world;

		world->getSolverInfo().m_numIterations = 4;

		m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
		m_dynamicsWorld->setGravity(btVector3(0,-10,0));



	int i;

	btTransform tr;
	tr.setIdentity();

	
	for (i=0;i<gNumObjects;i++)
	{
		if (i>0)
		{
			shapeIndex[i] = 1;//sphere
		}
		else
			shapeIndex[i] = 0;
	}


	btTransform trans;
	trans.setIdentity();
	
	btScalar halfExtents = CUBE_HALF_EXTENTS;

	trans.setOrigin(btVector3(0,-halfExtents,0));



	localCreateRigidBody(0.f,trans,m_collisionShapes[shapeIndex[0]]);

	int numWalls = 15;
	int wallHeight = 15;
	float wallDistance = 3;


	for (int i=0;i<numWalls;i++)
	{
		float zPos = (i-numWalls/2) * wallDistance;
		createStack(m_collisionShapes[shapeIndex[1]],halfExtents,wallHeight,zPos);
	}
#define DESTROYER_BALL 1
#ifdef DESTROYER_BALL
	btTransform sphereTrans;
	sphereTrans.setIdentity();
	sphereTrans.setOrigin(btVector3(0,2,40));
	btSphereShape* ball = new btSphereShape(2.f);
	m_collisionShapes.push_back(ball);
	btRigidBody* ballBody = localCreateRigidBody(10000.f,sphereTrans,ball);
	ballBody->setLinearVelocity(btVector3(0,0,-10));
#endif 
//	clientResetScene();


}
	





void	MultiThreadedDemo::exitPhysics()
{


	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;
#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	if (m_threadSupportSolver)
	{
		delete m_threadSupportSolver;
	}
#endif
#endif

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	if (m_threadSupportCollision)
	{
		delete m_threadSupportCollision;
	}
#endif
#endif

	delete m_collisionConfiguration;

	
}


