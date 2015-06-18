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
#include <algorithm>

class btCollisionShape;

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"  // for setSplitIslands()
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "MultiThreadedDemo.h"

#include "ParallelFor.h"


#define USE_PARALLEL_NARROWPHASE 1  // detect collisions in parallel
#define USE_PARALLEL_ISLAND_SOLVER 1   // solve simulation islands in parallel
#define USE_PARALLEL_CREATE_PREDICTIVE_CONTACTS 1
#define USE_PARALLEL_INTEGRATE_TRANSFORMS 1
#define USE_PARALLEL_PREDICT_UNCONSTRAINED_MOTION 1

#if defined (_MSC_VER) && _MSC_VER >= 1600
// give us a compile error if any signatures of overriden methods is changed
#define BT_OVERRIDE override
#else
#define BT_OVERRIDE
#endif



#if USE_PARALLEL_NARROWPHASE

class MyCollisionDispatcher : public btCollisionDispatcher
{
public:
    MyCollisionDispatcher( btCollisionConfiguration* config ) : btCollisionDispatcher( config )
    {
    }

    virtual ~MyCollisionDispatcher()
    {
    }

    struct Updater
    {
        btBroadphasePair* mPairArray;
        btNearCallback mCallback;
        btCollisionDispatcher* mDispatcher;
        const btDispatcherInfo* mInfo;

        Updater()
        {
            mPairArray = NULL;
            mCallback = NULL;
            mDispatcher = NULL;
            mInfo = NULL;
        }
        void forLoop( int iBegin, int iEnd ) const
        {
            for ( int i = iBegin; i < iEnd; ++i )
            {
                btBroadphasePair* pair = &mPairArray[ i ];
                mCallback( *pair, *mDispatcher, *mInfo );
            }
        }
    };

    virtual void dispatchAllCollisionPairs( btOverlappingPairCache* pairCache, const btDispatcherInfo& info, btDispatcher* dispatcher ) BT_OVERRIDE
    {
        int grainSize = 40;  // iterations per task
        int pairCount = pairCache->getNumOverlappingPairs();
        Updater updater;
        updater.mCallback = getNearCallback();
        updater.mPairArray = pairCount > 0 ? pairCache->getOverlappingPairArrayPtr() : NULL;
        updater.mDispatcher = this;
        updater.mInfo = &info;

        btPushThreadsAreRunning();
        parallelFor( 0, pairCount, grainSize, updater );
        btPopThreadsAreRunning();
    }
};

#endif


#if USE_PARALLEL_ISLAND_SOLVER
///
/// MyConstraintSolverPool - masquerades as a constraint solver, but really it is a threadsafe pool of them.
///
///  Each solver in the pool is protected by a mutex.  When solveGroup is called from a thread,
///  the pool looks for a solver that isn't being used by another thread, locks it, and dispatches the
///  call to the solver.
///  So long as there are at least as many solvers as there are hardware threads, it should never need to
///  spin wait.
///
class MyConstraintSolverPool : public btConstraintSolver
{
    const static size_t kCacheLineSize = 128;
    struct ThreadSolver
    {
        btConstraintSolver* solver;
        btMutex mutex;
        char _cachelinePadding[ kCacheLineSize - sizeof( btMutex ) - sizeof( void* ) ];  // keep mutexes from sharing a cache line
    };
    btAlignedObjectArray<ThreadSolver> m_solvers;
    btConstraintSolverType m_solverType;

    ThreadSolver* getAndLockThreadSolver()
    {
        while ( true )
        {
            for ( int i = 0; i < m_solvers.size(); ++i )
            {
                ThreadSolver& solver = m_solvers[ i ];
                if ( btMutexTryLock( &solver.mutex ) )
                {
                    return &solver;
                }
            }
        }
        return NULL;
    }
    void init( btConstraintSolver** solvers, int numSolvers )
    {
        m_solverType = BT_SEQUENTIAL_IMPULSE_SOLVER;
        m_solvers.resize( numSolvers );
        for ( int i = 0; i < numSolvers; ++i )
        {
            m_solvers[ i ].solver = solvers[ i ];
        }
        if ( numSolvers > 0 )
        {
            m_solverType = solvers[ 0 ]->getSolverType();
        }
    }
public:
    // create the solvers for me
    explicit MyConstraintSolverPool( int numSolvers )
    {
        btAlignedObjectArray<btConstraintSolver*> solvers;
        solvers.reserve( numSolvers );
        for ( int i = 0; i < numSolvers; ++i )
        {
            btConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
            solvers.push_back( solver );
        }
        init( &solvers[ 0 ], numSolvers );
    }

    // pass in fully constructed solvers (destructor will delete them)
    MyConstraintSolverPool( btConstraintSolver** solvers, int numSolvers )
    {
        init( solvers, numSolvers );
    }
    virtual ~MyConstraintSolverPool()
    {
        // delete all solvers
        for ( int i = 0; i < m_solvers.size(); ++i )
        {
            ThreadSolver& solver = m_solvers[ i ];
            delete solver.solver;
            solver.solver = NULL;
        }
    }

    //virtual void prepareSolve( int /* numBodies */, int /* numManifolds */ ) { ; } // does nothing

    ///solve a group of constraints
    virtual btScalar solveGroup( btCollisionObject** bodies,
                                 int numBodies,
                                 btPersistentManifold** manifolds,
                                 int numManifolds,
                                 btTypedConstraint** constraints,
                                 int numConstraints,
                                 const btContactSolverInfo& info,
                                 btIDebugDraw* debugDrawer,
                                 btDispatcher* dispatcher
                                 )
    {
        ThreadSolver* solver = getAndLockThreadSolver();
        solver->solver->solveGroup( bodies, numBodies, manifolds, numManifolds, constraints, numConstraints, info, debugDrawer, dispatcher );
        btMutexUnlock( &solver->mutex );
        return 0.0f;
    }

    //virtual void allSolved( const btContactSolverInfo& /* info */, class btIDebugDraw* /* debugDrawer */ ) { ; } // does nothing

    ///clear internal cached data and reset random seed
    virtual	void reset()
    {
        for ( int i = 0; i < m_solvers.size(); ++i )
        {
            ThreadSolver& solver = m_solvers[ i ];
            btMutexLock( &solver.mutex );
            solver.solver->reset();
            btMutexUnlock( &solver.mutex );
        }
    }

    virtual btConstraintSolverType getSolverType() const
    {
        return m_solverType;
    }
};

struct UpdateIslandDispatcher
{
    btAlignedObjectArray<btSimulationIslandManager::Island*>* islandsPtr;
    btSimulationIslandManager::IslandCallback* callback;

    void forLoop( int iBegin, int iEnd ) const
    {
        for ( int i = iBegin; i < iEnd; ++i )
        {
            btSimulationIslandManager::Island* island = ( *islandsPtr )[ i ];
            btPersistentManifold** manifolds = island->manifoldArray.size() ? &island->manifoldArray[ 0 ] : NULL;
            btTypedConstraint** constraintsPtr = island->constraintArray.size() ? &island->constraintArray[ 0 ] : NULL;
            callback->processIsland( &island->bodyArray[ 0 ],
                                     island->bodyArray.size(),
                                     manifolds,
                                     island->manifoldArray.size(),
                                     constraintsPtr,
                                     island->constraintArray.size(),
                                     island->id
                                     );
        }
    }
};

void parallelIslandDispatch( btAlignedObjectArray<btSimulationIslandManager::Island*>* islandsPtr, btSimulationIslandManager::IslandCallback* callback )
{
    int grainSize = 1;  // iterations per task
    UpdateIslandDispatcher dispatcher;
    dispatcher.islandsPtr = islandsPtr;
    dispatcher.callback = callback;
    btPushThreadsAreRunning();
    parallelFor( 0, islandsPtr->size(), grainSize, dispatcher );
    btPopThreadsAreRunning();
}
#endif //#if USE_PARALLEL_ISLAND_SOLVER


///
/// MyDiscreteDynamicsWorld
///
///  Should function exactly like btDiscreteDynamicsWorld.
///  3 methods that iterate over all of the rigidbodies can run in parallel:
///     - predictUnconstraintMotion
///     - integrateTransforms
///     - createPredictiveContacts
///
ATTRIBUTE_ALIGNED16( class ) MyDiscreteDynamicsWorld : public btDiscreteDynamicsWorld
{
    typedef btDiscreteDynamicsWorld ParentClass;

protected:
#if USE_PARALLEL_PREDICT_UNCONSTRAINED_MOTION
    struct UpdaterUnconstrainedMotion
    {
        btScalar timeStep;
        btRigidBody** rigidBodies;

        void forLoop( int iBegin, int iEnd ) const
        {
            for ( int i = iBegin; i < iEnd; ++i )
            {
                btRigidBody* body = rigidBodies[ i ];
                if ( !body->isStaticOrKinematicObject() )
                {
                    //don't integrate/update velocities here, it happens in the constraint solver
                    body->applyDamping( timeStep );
                    body->predictIntegratedTransform( timeStep, body->getInterpolationWorldTransform() );
                }
            }
        }
    };

    virtual void predictUnconstraintMotion( btScalar timeStep ) BT_OVERRIDE
    {
        BT_PROFILE( "predictUnconstraintMotion" );
        int grainSize = 50;  // num of iterations per task for TBB
        int bodyCount = m_nonStaticRigidBodies.size();
        UpdaterUnconstrainedMotion update;
        update.timeStep = timeStep;
        update.rigidBodies = bodyCount ? &m_nonStaticRigidBodies[ 0 ] : NULL;
        btPushThreadsAreRunning();
        parallelFor( 0, bodyCount, grainSize, update );
        btPopThreadsAreRunning();
    }
#endif // #if USE_PARALLEL_PREDICT_UNCONSTRAINED_MOTION

#if USE_PARALLEL_CREATE_PREDICTIVE_CONTACTS
    struct UpdaterCreatePredictiveContacts
    {
        btScalar timeStep;
        btRigidBody** rigidBodies;
        MyDiscreteDynamicsWorld* world;

        void forLoop( int iBegin, int iEnd ) const
        {
            world->createPredictiveContactsInternal( &rigidBodies[ iBegin ], iEnd - iBegin, timeStep );
        }
    };

    virtual void createPredictiveContacts( btScalar timeStep )
    {
        int grainSize = 50;  // num of iterations per task for TBB or OPENMP
        if ( int bodyCount = m_nonStaticRigidBodies.size() )
        {
            UpdaterCreatePredictiveContacts update;
            update.world = this;
            update.timeStep = timeStep;
            update.rigidBodies = &m_nonStaticRigidBodies[ 0 ];
            btPushThreadsAreRunning();
            parallelFor( 0, bodyCount, grainSize, update );
            btPopThreadsAreRunning();
        }
    }
#endif // #if USE_PARALLEL_CREATE_PREDICTIVE_CONTACTS

#if USE_PARALLEL_INTEGRATE_TRANSFORMS
    struct UpdaterIntegrateTransforms
    {
        btScalar timeStep;
        btRigidBody** rigidBodies;
        const MyDiscreteDynamicsWorld* world;

        void forLoop( int iBegin, int iEnd ) const
        {
            world->integrateTransformsInternal( &rigidBodies[ iBegin ], iEnd - iBegin, timeStep );
        }
    };

    virtual void integrateTransforms( btScalar timeStep ) BT_OVERRIDE
    {
        BT_PROFILE( "integrateTransforms" );
        int grainSize = 50;  // num of iterations per task for TBB or OPENMP
        if ( int bodyCount = m_nonStaticRigidBodies.size() )
        {
            UpdaterIntegrateTransforms update;
            update.world = this;
            update.timeStep = timeStep;
            update.rigidBodies = &m_nonStaticRigidBodies[ 0 ];
            btPushThreadsAreRunning();
            parallelFor( 0, bodyCount, grainSize, update );
            btPopThreadsAreRunning();
        }
    }
#endif // #if USE_PARALLEL_INTEGRATE_TRANSFORMS

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

};

/// MultiThreadedDemo shows how to setup and use multithreading
class MultiThreadedDemo  : public CommonExampleInterface
{
    static const int kUpAxis = 1;

    btDiscreteDynamicsWorld* m_dynamicsWorld;
    btBroadphaseInterface*	m_broadphase;
    btCollisionDispatcher*	m_dispatcher;
    btConstraintSolver*	m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    //keep the collision shapes, for deletion/cleanup
    btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

    int m_numThreads;

	btDiscreteDynamicsWorld* getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

	GUIHelperInterface* m_guiHelper;

	btVector3	m_loadStartPos;

    btVector3 m_cameraTargetPos;
    float m_cameraPitch;
    float m_cameraYaw;
    float m_cameraDist;

    void createStack( const btVector3& pos, btCollisionShape* boxShape, const btVector3& halfBoxSize, int size );
    void createSceneObjects();
    void destroySceneObjects();
    void displayProfileString( const btVector3& pos, const char* msg );

public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    MultiThreadedDemo( struct GUIHelperInterface* helper );

	virtual ~MultiThreadedDemo();

	virtual void stepSimulation(float deltaTime) BT_OVERRIDE;
	virtual bool mouseMoveCallback(float x,float y) BT_OVERRIDE
	{
		return false;
	}

    virtual bool mouseButtonCallback( int button, int state, float x, float y ) BT_OVERRIDE
	{
		return false;
	}

    virtual bool keyboardCallback( int key, int state ) BT_OVERRIDE;
    virtual void renderScene() BT_OVERRIDE;
    virtual void physicsDebugDraw( int debugFlags ) BT_OVERRIDE;

    virtual void initPhysics() BT_OVERRIDE;
    virtual void exitPhysics() BT_OVERRIDE;

    virtual void resetCamera() BT_OVERRIDE
	{
        m_guiHelper->resetCamera( m_cameraDist,
                                  m_cameraPitch,
                                  m_cameraYaw,
                                  m_cameraTargetPos.x(),
                                  m_cameraTargetPos.y(),
                                  m_cameraTargetPos.z()
                                  );
	}

};


////////////////////////////////////


MultiThreadedDemo::MultiThreadedDemo(struct GUIHelperInterface* helper)
{
    m_dynamicsWorld = NULL;
    m_broadphase = NULL;
    m_dispatcher = NULL;
    m_solver = NULL;
    m_collisionConfiguration = NULL;
    m_guiHelper = helper;
    m_numThreads = 1;
    m_cameraTargetPos = btVector3( 0.0f, 0.0f, 0.0f );
    m_cameraPitch = 90.0f;
    m_cameraYaw = 30.0f;
    m_cameraDist = 48.0f;
    helper->setUpAxis( kUpAxis );
    initTaskScheduler();
    m_numThreads = setNumThreads( getMaxNumThreads() );
}


MultiThreadedDemo::~MultiThreadedDemo()
{
    cleanupTaskScheduler();
}


void MultiThreadedDemo::exitPhysics()
{
    destroySceneObjects();

    //delete dynamics world
    delete m_dynamicsWorld;

    //delete solver
    delete m_solver;

    //delete broadphase
    delete m_broadphase;

    //delete dispatcher
    delete m_dispatcher;

    delete m_collisionConfiguration;
}

void MultiThreadedDemo::initPhysics()
{
    m_dispatcher = NULL;
    btDefaultCollisionConstructionInfo cci;
    // it isn't threadsafe to resize these pools while threads are using them
    cci.m_defaultMaxPersistentManifoldPoolSize = 32768;
    cci.m_defaultMaxCollisionAlgorithmPoolSize = 32768;
    m_collisionConfiguration = new btDefaultCollisionConfiguration( cci );

#if USE_PARALLEL_NARROWPHASE
    m_dispatcher = new	MyCollisionDispatcher( m_collisionConfiguration );
#else
    m_dispatcher = new	btCollisionDispatcher( m_collisionConfiguration );
#endif //USE_PARALLEL_NARROWPHASE

    if ( false )
    {
        const int maxProxies = 32766;

        btVector3 worldAabbMin( -1000, -1000, -1000 );
        btVector3 worldAabbMax( 1000, 1000, 1000 );

        m_broadphase = new btAxisSweep3( worldAabbMin, worldAabbMax, maxProxies );
    }
    else
    {
        m_broadphase = new btDbvtBroadphase();
    }

    // this solver requires the contacts to be in a contiguous pool, so avoid dynamic allocation
    // (this may no longer be an issue, not sure)
    m_dispatcher->setDispatcherFlags( btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION );

#if USE_PARALLEL_ISLAND_SOLVER
    m_solver = new MyConstraintSolverPool( m_numThreads );
#else
    m_solver = new btSequentialImpulseConstraintSolver();
#endif //#if USE_PARALLEL_ISLAND_SOLVER
    btDiscreteDynamicsWorld* world = new MyDiscreteDynamicsWorld( m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration );
    m_dynamicsWorld = world;

#if USE_PARALLEL_ISLAND_SOLVER
    world->getSimulationIslandManager()->setIslandDispatchFunction( parallelIslandDispatch );
    //world->getSolverInfo().m_numIterations = 4;
    //default solverMode is SOLVER_RANDMIZE_ORDER. Warmstarting seems not to improve convergence, see 
    //solver->setSolverMode(0);//btSequentialImpulseConstraintSolver::SOLVER_USE_WARMSTARTING | btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
    //world->getSolverInfo().m_solverMode = SOLVER_SIMD + SOLVER_USE_WARMSTARTING;//+SOLVER_RANDMIZE_ORDER;
#endif //#if USE_PARALLEL_ISLAND_SOLVER

    m_dynamicsWorld->setGravity( btVector3( 0, -10, 0 ) );
	
    createSceneObjects();

    m_guiHelper->createPhysicsDebugDrawer( m_dynamicsWorld );
}

void MultiThreadedDemo::physicsDebugDraw(int debugFlags)
{
	if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		m_dynamicsWorld->debugDrawWorld();
	}
    btVector3 pos( -10.0f, 30.50f, -10.0f );
    btVector3 posStep( 0.0f, -1.0f, 0.0f );
    char msg[ 128 ];
    {
        int numManifolds = m_dispatcher->getNumManifolds();
        int numContacts = 0;
        for ( int i = 0; i < numManifolds; ++i )
        {
            const btPersistentManifold* man = m_dispatcher->getManifoldByIndexInternal( i );
            numContacts += man->getNumContacts();
        }
        const char* mtApi = getTaskApiName( gTaskApi );
        sprintf( msg, "bodies %d manifolds %d contacts %d [%s] threads %d",
                 m_dynamicsWorld->getNumCollisionObjects(),
                 numManifolds,
                 numContacts,
                 mtApi,
                 gTaskApi != apiNone ? m_numThreads : 1
                 );
        displayProfileString( pos, msg );
        pos += posStep;
    }
}

void MultiThreadedDemo::displayProfileString( const btVector3& pos, const char* msg )
{
    // This is kind of a hack.
    // 2D text would be preferable, but the current 2D text is far too large to be useable
    btVector3 wp = pos;
    m_guiHelper->drawText3D( msg, wp.x(), wp.y(), wp.z(), 1.0f );
}

//to be implemented by the demo
void MultiThreadedDemo::renderScene()
{
	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
	m_guiHelper->render(m_dynamicsWorld);
}

void MultiThreadedDemo::stepSimulation(float deltaTime)
{
	float dt = deltaTime;
	if (m_dynamicsWorld)
	{
        m_dynamicsWorld->stepSimulation(1.0f/60.0f, 0);
	}
}


bool MultiThreadedDemo::keyboardCallback(int key, int state)
{
    bool handled = false;
    if ( state )
    {
        if ( key == 'n' )
        {
            // 'n' single-threading
            setTaskApi( apiNone );
            handled = true;
        }
        if ( key == 'o' )
        {
            // 'o' use OpenMP
            setTaskApi( apiOpenMP );
            handled = true;
        }
        if ( key == 'p' )
        {
            // 'p' use PPL
            setTaskApi( apiPpl );
            handled = true;
        }
        if ( key == 't' )
        {
            // 't' use TBB
            setTaskApi( apiTbb );
            handled = true;
        }
        if ( gTaskApi != apiNone )
        {
            if ( key == '+' )
            {
                m_numThreads = setNumThreads( m_numThreads + 1 );
                handled = true;
            }
            if ( key == '-' )
            {
                m_numThreads = setNumThreads( (std::max)( 1, m_numThreads - 1 ) );
                handled = true;
            }
        }
    }

    return handled;
}


btRigidBody* MultiThreadedDemo::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
    if ( isDynamic )
    {
        shape->calculateLocalInertia( mass, localInertia );
    }

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);
	body->setWorldTransform(startTransform);
#endif//

    if ( isDynamic )
    {
        body->forceActivationState( DISABLE_DEACTIVATION );
    }
	m_dynamicsWorld->addRigidBody(body);
	return body;
}

void MultiThreadedDemo::createStack( const btVector3& center, btCollisionShape* boxShape, const btVector3& halfBoxSize, int size )
{
    btTransform trans;
    trans.setIdentity();
    float halfBoxHeight = halfBoxSize.y();
    float halfBoxWidth = halfBoxSize.x();

    for ( int i = 0; i<size; i++ )
    {
        // This constructs a row, from left to right
        int rowSize = size - i;
        for ( int j = 0; j< rowSize; j++ )
        {
            btVector3 pos = center + btVector3( halfBoxWidth*( 1 + j * 2 - rowSize ),
                halfBoxHeight * ( 1 + i * 2),
                0.0f
                );

            trans.setOrigin( pos );
            btScalar mass = 1.f;

            btRigidBody* body = localCreateRigidBody( mass, trans, boxShape );
        }
    }
}


void MultiThreadedDemo::createSceneObjects()
{
    {
        // create ground box
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin( btVector3( 0, -3, 0 ) );

        //either use heightfield or triangle mesh

        btVector3 groundExtents( 400, 400, 400 );
        groundExtents[ kUpAxis ] = 3;
        btCollisionShape* groundShape = new btBoxShape( groundExtents );
        m_collisionShapes.push_back( groundShape );

        //create ground object
        localCreateRigidBody( 0, tr, groundShape );
    }

    {
        // create walls of cubes
        const btVector3 halfExtents = btVector3( 0.5f, 0.25f, 0.5f );
        int numStackRows = 16;
        int numStackCols = 6;
        int stackHeight = 15;
        float stackZSpacing = 3.0f;
        float stackXSpacing = 20.0f;

        btBoxShape* boxShape = new btBoxShape( halfExtents );
        m_collisionShapes.push_back( boxShape );

        for ( int iX = 0; iX < numStackCols; ++iX )
        {
            for ( int iZ = 0; iZ < numStackRows; ++iZ )
            {
                btVector3 center = btVector3( iX * stackXSpacing, 0.0f, ( iZ - numStackRows / 2 ) * stackZSpacing );
                createStack( center, boxShape, halfExtents, stackHeight );
            }
        }
    }

    if ( false )
    {
        // destroyer ball
        btTransform sphereTrans;
        sphereTrans.setIdentity();
        sphereTrans.setOrigin( btVector3( 0, 2, 40 ) );
        btSphereShape* ball = new btSphereShape( 2.f );
        m_collisionShapes.push_back( ball );
        btRigidBody* ballBody = localCreateRigidBody( 10000.f, sphereTrans, ball );
        ballBody->setLinearVelocity( btVector3( 0, 0, -10 ) );
    }
    if ( false )
    {
        btCompoundShape* loadCompound = new btCompoundShape();
        m_collisionShapes.push_back( loadCompound );
        btCollisionShape* loadShapeA = new btBoxShape( btVector3( 2.0f, 0.5f, 0.5f ) );
        m_collisionShapes.push_back( loadShapeA );
        btTransform loadTrans;
        loadTrans.setIdentity();
        loadCompound->addChildShape( loadTrans, loadShapeA );
        btCollisionShape* loadShapeB = new btBoxShape( btVector3( 0.1f, 1.0f, 1.0f ) );
        m_collisionShapes.push_back( loadShapeB );
        loadTrans.setIdentity();
        loadTrans.setOrigin( btVector3( 2.1f, 0.0f, 0.0f ) );
        loadCompound->addChildShape( loadTrans, loadShapeB );
        btCollisionShape* loadShapeC = new btBoxShape( btVector3( 0.1f, 1.0f, 1.0f ) );
        m_collisionShapes.push_back( loadShapeC );
        loadTrans.setIdentity();
        loadTrans.setOrigin( btVector3( -2.1f, 0.0f, 0.0f ) );
        loadCompound->addChildShape( loadTrans, loadShapeC );
        loadTrans.setIdentity();
        m_loadStartPos = btVector3( 0.0f, 3.5f, 7.0f );
        loadTrans.setOrigin( m_loadStartPos );
        btScalar loadMass = 350.f;//
        localCreateRigidBody( loadMass, loadTrans, loadCompound );
    }
    m_guiHelper->autogenerateGraphicsObjects( m_dynamicsWorld );

}

void MultiThreadedDemo::destroySceneObjects()
{
    //cleanup in the reverse order of creation/initialization

    //remove the rigidbodies from the dynamics world and delete them
    for ( int i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i-- )
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[ i ];
        btRigidBody* body = btRigidBody::upcast( obj );
        if ( body && body->getMotionState() )
        {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }

    //delete collision shapes
    for ( int j = 0; j<m_collisionShapes.size(); j++ )
    {
        btCollisionShape* shape = m_collisionShapes[ j ];
        m_collisionShapes[ j ] = 0;
        delete shape;
    }

    m_collisionShapes.clear();
}

CommonExampleInterface*    MultiThreadedDemoCreateFunc( struct CommonExampleOptions& options )
{
	return new MultiThreadedDemo(options.m_guiHelper);
}
