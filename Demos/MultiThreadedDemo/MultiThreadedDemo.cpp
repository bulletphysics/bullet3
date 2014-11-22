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

#define USE_TBB 1

#if USE_TBB

#include <tbb/tbb.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

tbb::task_scheduler_init* gTaskSchedulerInit;

#endif

#define USE_PARALLEL_SOLVER 0 //experimental parallel solver
#define USE_PARALLEL_DISPATCHER 1

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



#if USE_PARALLEL_DISPATCHER

class MyCollisionDispatcher : public btCollisionDispatcher
{
public:
#if USE_TBB
    struct TbbUpdater
    {
        btBroadphasePair* mPairArray = nullptr;
        btNearCallback mCallback = nullptr;
        btCollisionDispatcher* mDispatcher = nullptr;
        const btDispatcherInfo* mInfo = nullptr;

        void operator()( const tbb::blocked_range<int>& range ) const
        {
            for ( int i = range.begin(); i != range.end(); ++i )
            {
                btBroadphasePair* pair = &mPairArray[ i ];
                mCallback( *pair, *mDispatcher, *mInfo );
            }
        }
    };
#endif

    MyCollisionDispatcher( btCollisionConfiguration* config ) : btCollisionDispatcher( config )
    {
    }

    virtual ~MyCollisionDispatcher()
    {
    }

    virtual void dispatchAllCollisionPairs( btOverlappingPairCache* pairCache, const btDispatcherInfo& info, btDispatcher* dispatcher )
    {
#if USE_TBB
        using namespace tbb;
        int pairCount = pairCache->getNumOverlappingPairs();
        TbbUpdater tbbUpdate;
        tbbUpdate.mCallback = getNearCallback();
        tbbUpdate.mPairArray = pairCache->getOverlappingPairArrayPtr();
        tbbUpdate.mDispatcher = this;
        tbbUpdate.mInfo = &info;

        parallel_for( blocked_range<int>( 0, pairCount ), tbbUpdate, simple_partitioner() );
#else
        btNearCallback callback = getNearCallback();
        int pairCount = pairCache->getNumOverlappingPairs();
        btBroadphasePair* pairArray = pairCache->getOverlappingPairArrayPtr();

        for ( int i = 0; i < pairCount; ++i )
        {
            btBroadphasePair* pair = &pairArray[ i ];
            callback( *pair, *this, info );
        }
#endif
    }
};

#endif

#define PROFILE( x )

class Profiler
{
public:
    static const int kSamples = 20;
    btClock mClock;
    int mIndex;
    float mHistory[ kSamples ];

    Profiler()
    {
        mIndex = 0;
    }
    void addSample( float samp )
    {
        if ( mIndex < 0 )
        {
            for ( int i = 0; i < kSamples; ++i )
            {
                mHistory[ i ] = samp;
            }
            mIndex = 0;
        }
        else
        {
            mIndex++;
            if ( mIndex >= kSamples )
            {
                mIndex = 0;
            }
            mHistory[ mIndex ] = samp;
        }
    }
    void begin()
    {
        mClock.reset();
    }
    void end()
    {
        btScalar tSim = mClock.getTimeSeconds();
        addSample( tSim );
    }
    float getMin() const
    {
        float tMin = mHistory[ 0 ];
        for ( int i = 0; i < kSamples; ++i )
        {
            tMin = min( tMin, mHistory[ i ] );
        }
        return tMin;
    }
    float getMax() const
    {
        float tMax = mHistory[ 0 ];
        for ( int i = 0; i < kSamples; ++i )
        {
            tMax = max( tMax, mHistory[ i ] );
        }
        return tMax;
    }
    float getAvg() const
    {
        float tSum = 0.0f;
        for ( int i = 0; i < kSamples; ++i )
        {
            tSum += mHistory[ i ];
        }
        return tSum / kSamples;
    }
};

Profiler gProfInternalSingleStepSimulation;
Profiler gProfPerformDiscreteCollisionDetection;
Profiler gProfUpdateAabbs;
Profiler gProfSolveConstraints;
Profiler gProfComputeOverlappingPairs;
Profiler gProfStepSimulation;

ATTRIBUTE_ALIGNED16( class ) MyDiscreteDynamicsWorld : public btDiscreteDynamicsWorld
{
    typedef btDiscreteDynamicsWorld ParentClass;
protected:
    virtual void predictUnconstraintMotion( btScalar timeStep ) override
    {
        PROFILE( predictUnconstraintMotion );
        ParentClass::predictUnconstraintMotion( timeStep );
    }
    virtual void integrateTransforms( btScalar timeStep ) override
    {
        PROFILE( integrateTransforms );
        ParentClass::integrateTransforms( timeStep );
    }
    virtual void calculateSimulationIslands() override
    {
        PROFILE( calculateSimulationIslands );
        ParentClass::calculateSimulationIslands();
    }
    virtual void solveConstraints( btContactSolverInfo& solverInfo ) override
    {
        gProfSolveConstraints.begin();
        ParentClass::solveConstraints( solverInfo );
        gProfSolveConstraints.end();
    }
    virtual void internalSingleStepSimulation( btScalar timeStep ) override
    {
        gProfInternalSingleStepSimulation.begin();
        ParentClass::internalSingleStepSimulation( timeStep );
        gProfInternalSingleStepSimulation.end();
    }

public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    MyDiscreteDynamicsWorld( btDispatcher* dispatcher,
                             btBroadphaseInterface* pairCache,
                             btConstraintSolver* constraintSolver,
                             btCollisionConfiguration* collisionConfiguration
                             ) :
                             btDiscreteDynamicsWorld( dispatcher, pairCache, constraintSolver, collisionConfiguration )
    {
    }

    virtual void performDiscreteCollisionDetection() override
    {
        gProfPerformDiscreteCollisionDetection.begin();
        ParentClass::performDiscreteCollisionDetection();
        gProfPerformDiscreteCollisionDetection.end();
    }

    virtual void updateAabbs()
    {
        gProfUpdateAabbs.begin();
        ParentClass::updateAabbs();
        gProfUpdateAabbs.end();
    }

    virtual void computeOverlappingPairs()
    {
        gProfComputeOverlappingPairs.begin();
        ParentClass::computeOverlappingPairs();
        gProfComputeOverlappingPairs.end();
    }

    virtual void synchronizeMotionStates() override
    {
        PROFILE( synchronizeMotionStates );
        ParentClass::synchronizeMotionStates();
    }
    virtual int	stepSimulation( btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep ) override
    {
        return ParentClass::stepSimulation( timeStep, maxSubSteps, fixedTimeStep );
    }

};


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

#define FIXED_STEP 1
#ifdef FIXED_STEP
        gProfStepSimulation.begin();
  		m_dynamicsWorld->stepSimulation(1.0f/60.f,0);
        gProfStepSimulation.end();
        int y = 20;
        int yStep = 20;
        char msg[ 128 ];
        {
            const Profiler* prof = &gProfStepSimulation;
            sprintf( msg, "stepSimulation time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
                     prof->getMin()*1000.0f,
                     prof->getAvg()*1000.0f,
                     prof->getMax()*1000.0f
                     );
            displayProfileString( 10, y, msg );
            y += yStep;
        }
        {
            const Profiler* prof = &gProfPerformDiscreteCollisionDetection;
            sprintf( msg, "collision detection time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
                     prof->getMin()*1000.0f,
                     prof->getAvg()*1000.0f,
                     prof->getMax()*1000.0f
                     );
            displayProfileString( 10, y, msg );
            y += yStep;
        }
        {
            const Profiler* prof = &gProfSolveConstraints;
            sprintf( msg, "solve constraints time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
                     prof->getMin()*1000.0f,
                     prof->getAvg()*1000.0f,
                     prof->getMax()*1000.0f
                     );
            displayProfileString( 10, y, msg );
            y += yStep;
        }
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
#if USE_TBB
    gTaskSchedulerInit = new tbb::task_scheduler_init(4);
#endif
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

	m_dispatcher=NULL;
	btDefaultCollisionConstructionInfo cci;
	cci.m_defaultMaxPersistentManifoldPoolSize = 32768;
	m_collisionConfiguration = new btDefaultCollisionConfiguration(cci);
	
#if USE_PARALLEL_DISPATCHER
	m_dispatcher = new	MyCollisionDispatcher(m_collisionConfiguration);
#else
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

#if USE_PARALLEL_SOLVER
	m_solver = new btParallelConstraintSolver(m_threadSupportSolver);
	//this solver requires the contacts to be in a contiguous pool, so avoid dynamic allocation
	m_dispatcher->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);
#else

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	m_solver = solver;
	//default solverMode is SOLVER_RANDMIZE_ORDER. Warmstarting seems not to improve convergence, see 
	//solver->setSolverMode(0);//btSequentialImpulseConstraintSolver::SOLVER_USE_WARMSTARTING | btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
#endif //USE_PARALLEL_SOLVER

    btDiscreteDynamicsWorld* world = new MyDiscreteDynamicsWorld( m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration );
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

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
#if USE_TBB
    delete gTaskSchedulerInit;
#endif
}


