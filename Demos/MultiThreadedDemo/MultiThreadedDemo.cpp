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

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include <stdio.h> //printf debugging

#include "MultiThreadedDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"

//#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
//#include "BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#define USE_PARALLEL_SOLVER 0 //experimental parallel solver
#define USE_PARALLEL_DISPATCHER 0

//#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
//#include "BulletMultiThreaded/PlatformDefinitions.h"

//#ifdef _WIN32
//
//#include "BulletMultiThreaded/Win32ThreadSupport.h"
//
//#elif defined (USE_PTHREADS)
//
//#include "BulletMultiThreaded/PosixThreadSupport.h"
//
//#else
////other platforms run the parallel code sequentially (until pthread support or other parallel implementation is added)
//
//#endif

//#include "BulletMultiThreaded/SequentialThreadSupport.h"
//#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
//#include "BulletMultiThreaded/btParallelConstraintSolver.h"



#if USE_PARALLEL_SOLVER
btThreadSupportInterface* createSolverThreadSupport(int maxNumThreads)
{
#ifdef _WIN32
    Win32ThreadSupport::Win32ThreadConstructionInfo constructionInfo( "solver", SolverThreadFunc, SolverlsMemoryFunc, maxNumThreads );
    Win32ThreadSupport* threadSupport = new Win32ThreadSupport( constructionInfo );
	threadSupport->startSPU();
#elif defined (USE_PTHREADS)
    PosixThreadSupport::ThreadConstructionInfo constructionInfo("solver", SolverThreadFunc,
																	  SolverlsMemoryFunc, maxNumThreads);
    PosixThreadSupport* threadSupport = new PosixThreadSupport(constructionInfo);
#else
    SequentialThreadSupport::SequentialThreadConstructionInfo constructionInfo("solver",SolverThreadFunc,SolverlsMemoryFunc);
    SequentialThreadSupport* threadSupport = new SequentialThreadSupport(constructionInfo);
	threadSupport->startSPU();
#endif
	return threadSupport;
}
#endif

#if USE_PARALLEL_DISPATCHER
btThreadSupportInterface* createCollisionThreadSupport( int maxNumThreads )
{
#ifdef _WIN32
    Win32ThreadSupport::Win32ThreadConstructionInfo constructionInfo( "collision",
                                                                      processCollisionTask,
                                                                      createCollisionLocalStoreMemory,
                                                                      maxNumThreads );
    Win32ThreadSupport* threadSupport = new Win32ThreadSupport( constructionInfo );
    threadSupport->startSPU();
#elif defined (USE_PTHREADS)
    PosixThreadSupport::ThreadConstructionInfo constructionInfo( "collision",
                                                                 processCollisionTask,
                                                                 createCollisionLocalStoreMemory,
                                                                 maxNumThreads );
    PosixThreadSupport* threadSupport  = new PosixThreadSupport( constructionInfo );
#else

    SequentialThreadSupport::SequentialThreadConstructionInfo colCI( "collision", processCollisionTask, createCollisionLocalStoreMemory );
    SequentialThreadSupport* threadSupport  = new SequentialThreadSupport( colCI );

#endif
    return threadSupport;
}
#endif

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
//GL_LineSegmentShape shapeE(btVector3(-50,0,0),
//						   btVector3(50,0,0));

void MultiThreadedDemo::addBenchSample( float samp )
{
    if ( m_benchIndex < 0 )
    {
        for ( int i = 0; i < kBenchSamples; ++i )
        {
            m_benchHistory[ i ] = samp;
        }
        m_benchIndex = 0;
    }
    else
    {
        m_benchIndex++;
        if ( m_benchIndex >= kBenchSamples )
        {
            m_benchIndex = 0;
        }
        m_benchHistory[ m_benchIndex ] = samp;
    }
}

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
			btVector3 pos;
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



void MultiThreadedDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

//	float dt = getDeltaTimeMicroseconds() * 0.000001f;
	
//	printf("dt = %f: ",dt);
	
	if (m_dynamicsWorld)
	{

        btClock clock;
        clock.reset();
#define FIXED_STEP 1
#ifdef FIXED_STEP
  		m_dynamicsWorld->stepSimulation(1.0f/60.f,0);
        btScalar tSim = clock.getTimeSeconds();
        addBenchSample( tSim );
        float tMin = tSim;
        float tMax = tSim;
        float tSum = 0.0f;
        for ( int i = 0; i < kBenchSamples; ++i )
        {
            tMin = min( tMin, m_benchHistory[ i ] );
            tMax = max( tMax, m_benchHistory[ i ] );
            tSum += m_benchHistory[ i ];
        }
        float tAvg = tSum / kBenchSamples;
        char msg[ 128 ];
        sprintf( msg, "sim time=%5.5f / %5.5f / %5.5f ms (min/avg/max)", tMin*1000.0f, tAvg*1000.0f, tMax*1000.0f );
        displayProfileString( 10, 20, msg );
		//CProfileManager::dumpAll();
	
#else
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			dt = 1.0/420.f;

		int numSimSteps = 0;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
		

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
	   //optional but useful: debug drawing
               m_dynamicsWorld->debugDrawWorld();

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

	
	glutSwapBuffers();

}



void MultiThreadedDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	renderme();

	glFlush();
	glutSwapBuffers();
}



void	MultiThreadedDemo::initPhysics()
{
	m_threadSupportSolver = NULL;
	m_threadSupportCollision = NULL;
    int maxNumOutstandingTasks = 4;

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

    m_benchIndex = -1;
	m_dispatcher=NULL;
	btDefaultCollisionConstructionInfo cci;
	cci.m_defaultMaxPersistentManifoldPoolSize = 32768;
	m_collisionConfiguration = new btDefaultCollisionConfiguration(cci);
	
#if USE_PARALLEL_DISPATCHER
    m_threadSupportCollision = createCollisionThreadSupport( maxNumOutstandingTasks );
	m_dispatcher = new	SpuGatheringCollisionDispatcher(m_threadSupportCollision,maxNumOutstandingTasks,m_collisionConfiguration);
#else
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

#if USE_PARALLEL_SOLVER
	m_threadSupportSolver = createSolverThreadSupport(maxNumOutstandingTasks);
	m_solver = new btParallelConstraintSolver(m_threadSupportSolver);
	//this solver requires the contacts to be in a contiguous pool, so avoid dynamic allocation
	m_dispatcher->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);
#else

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	m_solver = solver;
	//default solverMode is SOLVER_RANDMIZE_ORDER. Warmstarting seems not to improve convergence, see 
	//solver->setSolverMode(0);//btSequentialImpulseConstraintSolver::SOLVER_USE_WARMSTARTING | btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
#endif //USE_PARALLEL_SOLVER

    btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld( m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration );
    m_dynamicsWorld = world;

    world->getSimulationIslandManager()->setSplitIslands( false );
    world->getSolverInfo().m_numIterations = 4;
    world->getSolverInfo().m_solverMode = SOLVER_SIMD + SOLVER_USE_WARMSTARTING;//+SOLVER_RANDMIZE_ORDER;

    m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
    m_dynamicsWorld->setGravity( btVector3( 0, -10, 0 ) );

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

	for ( i=0;i<numWalls;i++)
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
#if USE_PARALLEL_SOLVER
    if ( m_threadSupportSolver )
	{
		delete m_threadSupportSolver;
	}
#endif

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

#if USE_PARALLEL_DISPATCHER
    deleteCollisionLocalStoreMemory();
	if (m_threadSupportCollision)
	{
		delete m_threadSupportCollision;
	}
#endif

	delete m_collisionConfiguration;
}


