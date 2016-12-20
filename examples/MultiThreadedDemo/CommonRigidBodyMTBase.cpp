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
#include "LinearMath/btIDebugDraw.h"

#include <stdio.h>
#include <algorithm>

class btCollisionShape;

#include "CommonRigidBodyMTBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "ParallelFor.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btPoolAllocator.h"
#include "btBulletCollisionCommon.h"
#include "BulletDynamics/Dynamics/btSimulationIslandManagerMt.h"  // for setSplitIslands()
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"

TaskManager gTaskMgr;

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


class Profiler
{
public:
    enum RecordType
    {
        kRecordInternalTimeStep,
        kRecordDispatchAllCollisionPairs,
        kRecordDispatchIslands,
        kRecordPredictUnconstrainedMotion,
        kRecordCreatePredictiveContacts,
        kRecordIntegrateTransforms,
        kRecordCount
    };

private:
    btClock mClock;

    struct Record
    {
        int mCallCount;
        unsigned long long mAccum;
        unsigned int mStartTime;
        unsigned int mHistory[8];

        void begin(unsigned int curTime)
        {
            mStartTime = curTime;
        }
        void end(unsigned int curTime)
        {
            unsigned int endTime = curTime;
            unsigned int elapsed = endTime - mStartTime;
            mAccum += elapsed;
            mHistory[ mCallCount & 7 ] = elapsed;
            ++mCallCount;
        }
        float getAverageTime() const
        {
            int count = btMin( 8, mCallCount );
            if ( count > 0 )
            {
                unsigned int sum = 0;
                for ( int i = 0; i < count; ++i )
                {
                    sum += mHistory[ i ];
                }
                float avg = float( sum ) / float( count );
                return avg;
            }
            return 0.0;
        }
    };
    Record mRecords[ kRecordCount ];

public:
    void begin(RecordType rt)
    {
        mRecords[rt].begin(mClock.getTimeMicroseconds());
    }
    void end(RecordType rt)
    {
        mRecords[rt].end(mClock.getTimeMicroseconds());
    }
    float getAverageTime(RecordType rt) const
    {
        return mRecords[rt].getAverageTime();
    }
};


Profiler gProfiler;

class ProfileHelper
{
    Profiler::RecordType mRecType;
public:
    ProfileHelper(Profiler::RecordType rt)
    {
        mRecType = rt;
        gProfiler.begin( mRecType );
    }
    ~ProfileHelper()
    {
        gProfiler.end( mRecType );
    }
};

int gThreadsRunningCounter = 0;
btSpinMutex gThreadsRunningCounterMutex;

void btPushThreadsAreRunning()
{
    gThreadsRunningCounterMutex.lock();
    gThreadsRunningCounter++;
    gThreadsRunningCounterMutex.unlock();
}

void btPopThreadsAreRunning()
{
    gThreadsRunningCounterMutex.lock();
    gThreadsRunningCounter--;
    gThreadsRunningCounterMutex.unlock();
}

bool btThreadsAreRunning()
{
    return gThreadsRunningCounter != 0;
}


#if USE_PARALLEL_NARROWPHASE

class MyCollisionDispatcher : public btCollisionDispatcher
{
    btSpinMutex m_manifoldPtrsMutex;

public:
    MyCollisionDispatcher( btCollisionConfiguration* config ) : btCollisionDispatcher( config )
    {
    }

    virtual ~MyCollisionDispatcher()
    {
    }

    btPersistentManifold* getNewManifold( const btCollisionObject* body0, const btCollisionObject* body1 ) BT_OVERRIDE
    {
        // added spin-locks
        //optional relative contact breaking threshold, turned on by default (use setDispatcherFlags to switch off feature for improved performance)

        btScalar contactBreakingThreshold = ( m_dispatcherFlags & btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD ) ?
            btMin( body0->getCollisionShape()->getContactBreakingThreshold( gContactBreakingThreshold ), body1->getCollisionShape()->getContactBreakingThreshold( gContactBreakingThreshold ) )
            : gContactBreakingThreshold;

        btScalar contactProcessingThreshold = btMin( body0->getContactProcessingThreshold(), body1->getContactProcessingThreshold() );

        void* mem = m_persistentManifoldPoolAllocator->allocate( sizeof( btPersistentManifold ) );
        if (NULL == mem)
        {
            //we got a pool memory overflow, by default we fallback to dynamically allocate memory. If we require a contiguous contact pool then assert.
            if ( ( m_dispatcherFlags&CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION ) == 0 )
            {
                mem = btAlignedAlloc( sizeof( btPersistentManifold ), 16 );
            }
            else
            {
                btAssert( 0 );
                //make sure to increase the m_defaultMaxPersistentManifoldPoolSize in the btDefaultCollisionConstructionInfo/btDefaultCollisionConfiguration
                return 0;
            }
        }
        btPersistentManifold* manifold = new(mem) btPersistentManifold( body0, body1, 0, contactBreakingThreshold, contactProcessingThreshold );
        m_manifoldPtrsMutex.lock();
        manifold->m_index1a = m_manifoldsPtr.size();
        m_manifoldsPtr.push_back( manifold );
        m_manifoldPtrsMutex.unlock();

        return manifold;
    }

    void releaseManifold( btPersistentManifold* manifold ) BT_OVERRIDE
    {
        clearManifold( manifold );

        m_manifoldPtrsMutex.lock();
        int findIndex = manifold->m_index1a;
        btAssert( findIndex < m_manifoldsPtr.size() );
        m_manifoldsPtr.swap( findIndex, m_manifoldsPtr.size() - 1 );
        m_manifoldsPtr[ findIndex ]->m_index1a = findIndex;
        m_manifoldsPtr.pop_back();
        m_manifoldPtrsMutex.unlock();

        manifold->~btPersistentManifold();
        if ( m_persistentManifoldPoolAllocator->validPtr( manifold ) )
        {
            m_persistentManifoldPoolAllocator->freeMemory( manifold );
        }
        else
        {
            btAlignedFree( manifold );
        }
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
        ProfileHelper prof(Profiler::kRecordDispatchAllCollisionPairs);
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
        
        if (m_manifoldsPtr.size() < 1)
            return;

        // reconstruct the manifolds array to ensure determinism
        m_manifoldsPtr.resizeNoInitialize(0);
        btBroadphasePair* pairs = pairCache->getOverlappingPairArrayPtr();
        for (int i = 0; i < pairCount; ++i)
        {
            btCollisionAlgorithm* algo = pairs[i].m_algorithm;
            if (algo) algo->getAllContactManifolds(m_manifoldsPtr);
        }

        // update the indices (used when releasing manifolds)
        for (int i = 0; i < m_manifoldsPtr.size(); ++i)
            m_manifoldsPtr[i]->m_index1a = i;
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
        btSpinMutex mutex;
        char _cachelinePadding[ kCacheLineSize - sizeof( btSpinMutex ) - sizeof( void* ) ];  // keep mutexes from sharing a cache line
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
                if ( solver.mutex.tryLock() )
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
        solver->mutex.unlock();
        return 0.0f;
    }

    //virtual void allSolved( const btContactSolverInfo& /* info */, class btIDebugDraw* /* debugDrawer */ ) { ; } // does nothing

    ///clear internal cached data and reset random seed
    virtual	void reset()
    {
        for ( int i = 0; i < m_solvers.size(); ++i )
        {
            ThreadSolver& solver = m_solvers[ i ];
            solver.mutex.lock();
            solver.solver->reset();
            solver.mutex.unlock();
        }
    }

    virtual btConstraintSolverType getSolverType() const
    {
        return m_solverType;
    }
};

struct UpdateIslandDispatcher
{
    btAlignedObjectArray<btSimulationIslandManagerMt::Island*>* islandsPtr;
    btSimulationIslandManagerMt::IslandCallback* callback;

    void forLoop( int iBegin, int iEnd ) const
    {
        for ( int i = iBegin; i < iEnd; ++i )
        {
            btSimulationIslandManagerMt::Island* island = ( *islandsPtr )[ i ];
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

static int gNumIslands = 0;

void parallelIslandDispatch( btAlignedObjectArray<btSimulationIslandManagerMt::Island*>* islandsPtr, btSimulationIslandManagerMt::IslandCallback* callback )
{
    ProfileHelper prof(Profiler::kRecordDispatchIslands);
    gNumIslands = islandsPtr->size();
    int grainSize = 1;  // iterations per task
    UpdateIslandDispatcher dispatcher;
    dispatcher.islandsPtr = islandsPtr;
    dispatcher.callback = callback;
    btPushThreadsAreRunning();
    parallelFor( 0, islandsPtr->size(), grainSize, dispatcher );
    btPopThreadsAreRunning();
}
#endif //#if USE_PARALLEL_ISLAND_SOLVER


void profileBeginCallback(btDynamicsWorld *world, btScalar timeStep)
{
    gProfiler.begin(Profiler::kRecordInternalTimeStep);
}

void profileEndCallback(btDynamicsWorld *world, btScalar timeStep)
{
    gProfiler.end(Profiler::kRecordInternalTimeStep);
}

///
/// MyDiscreteDynamicsWorld
///
///  Should function exactly like btDiscreteDynamicsWorld.
///  3 methods that iterate over all of the rigidbodies can run in parallel:
///     - predictUnconstraintMotion
///     - integrateTransforms
///     - createPredictiveContacts
///
ATTRIBUTE_ALIGNED16( class ) MyDiscreteDynamicsWorld : public btDiscreteDynamicsWorldMt
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
        ProfileHelper prof( Profiler::kRecordPredictUnconstrainedMotion );
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
        ProfileHelper prof( Profiler::kRecordCreatePredictiveContacts );
        releasePredictiveContacts();
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
        MyDiscreteDynamicsWorld* world;

        void forLoop( int iBegin, int iEnd ) const
        {
            world->integrateTransformsInternal( &rigidBodies[ iBegin ], iEnd - iBegin, timeStep );
        }
    };

    virtual void integrateTransforms( btScalar timeStep ) BT_OVERRIDE
    {
        ProfileHelper prof( Profiler::kRecordIntegrateTransforms );
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
                             btDiscreteDynamicsWorldMt( dispatcher, pairCache, constraintSolver, collisionConfiguration )
    {
    }

};

static bool gMultithreadedWorld = false;
static bool gDisplayProfileInfo = false;
static btScalar gSliderNumThreads = 1.0f;  // should be int
static btScalar gSliderSolverIterations = 10.0f; // should be int


////////////////////////////////////
CommonRigidBodyMTBase::CommonRigidBodyMTBase( struct GUIHelperInterface* helper )
    :m_broadphase( 0 ),
    m_dispatcher( 0 ),
    m_solver( 0 ),
    m_collisionConfiguration( 0 ),
    m_dynamicsWorld( 0 ),
    m_pickedBody( 0 ),
    m_pickedConstraint( 0 ),
    m_guiHelper( helper )
{
    m_multithreadedWorld = false;
    m_multithreadCapable = false;
    gTaskMgr.init();
}

CommonRigidBodyMTBase::~CommonRigidBodyMTBase()
{
    gTaskMgr.shutdown();
}

void boolPtrButtonCallback(int buttonId, bool buttonState, void* userPointer)
{
    if (bool* val = static_cast<bool*>(userPointer))
    {
        *val = ! *val;
    }
}

void apiSelectButtonCallback(int buttonId, bool buttonState, void* userPointer)
{
    gTaskMgr.setApi(static_cast<TaskManager::Api>(buttonId));
    if (gTaskMgr.getApi()==TaskManager::apiNone)
    {
        gSliderNumThreads = 1.0f;
    }
    else
    {
        gSliderNumThreads = float(gTaskMgr.getNumThreads());
    }
}

void setThreadCountCallback(float val, void* userPtr)
{
    if (gTaskMgr.getApi()==TaskManager::apiNone)
    {
        gSliderNumThreads = 1.0f;
    }
    else
    {
        gTaskMgr.setNumThreads( int( gSliderNumThreads ) );
    }
}

void setSolverIterationCountCallback(float val, void* userPtr)
{
    if (btDiscreteDynamicsWorld* world = reinterpret_cast<btDiscreteDynamicsWorld*>(userPtr))
    {
       	world->getSolverInfo().m_numIterations = btMax(1, int(gSliderSolverIterations));
    }
}

void CommonRigidBodyMTBase::createEmptyDynamicsWorld()
{
    gNumIslands = 0;
#if BT_THREADSAFE && (BT_USE_OPENMP || BT_USE_PPL || BT_USE_TBB)
    m_multithreadCapable = true;
#endif
    if ( gMultithreadedWorld )
    {
        m_dispatcher = NULL;
        btDefaultCollisionConstructionInfo cci;
        cci.m_defaultMaxPersistentManifoldPoolSize = 80000;
        cci.m_defaultMaxCollisionAlgorithmPoolSize = 80000;
        m_collisionConfiguration = new btDefaultCollisionConfiguration( cci );

#if USE_PARALLEL_NARROWPHASE
        m_dispatcher = new	MyCollisionDispatcher( m_collisionConfiguration );
#else
        m_dispatcher = new	btCollisionDispatcher( m_collisionConfiguration );
#endif //USE_PARALLEL_NARROWPHASE

        m_broadphase = new btDbvtBroadphase();

#if USE_PARALLEL_ISLAND_SOLVER
        m_solver = new MyConstraintSolverPool( TaskManager::getMaxNumThreads() );
#else
        m_solver = new btSequentialImpulseConstraintSolver();
#endif //#if USE_PARALLEL_ISLAND_SOLVER
        btDiscreteDynamicsWorld* world = new MyDiscreteDynamicsWorld( m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration );
        m_dynamicsWorld = world;

#if USE_PARALLEL_ISLAND_SOLVER
        if ( btSimulationIslandManagerMt* islandMgr = dynamic_cast<btSimulationIslandManagerMt*>( world->getSimulationIslandManager() ) )
        {
            islandMgr->setIslandDispatchFunction( parallelIslandDispatch );
            m_multithreadedWorld = true;
        }
#endif //#if USE_PARALLEL_ISLAND_SOLVER
    }
    else
    {
        // single threaded world
        m_multithreadedWorld = false;

        ///collision configuration contains default setup for memory, collision setup
        m_collisionConfiguration = new btDefaultCollisionConfiguration();
        //m_collisionConfiguration->setConvexConvexMultipointIterations();

        ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
        m_dispatcher = new	btCollisionDispatcher( m_collisionConfiguration );

        m_broadphase = new btDbvtBroadphase();

        ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
        btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
        m_solver = sol;

        m_dynamicsWorld = new btDiscreteDynamicsWorld( m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration );
    }
    m_dynamicsWorld->setInternalTickCallback( profileBeginCallback, NULL, true );
    m_dynamicsWorld->setInternalTickCallback( profileEndCallback, NULL, false );
    m_dynamicsWorld->setGravity( btVector3( 0, -10, 0 ) );
    createDefaultParameters();
}


void CommonRigidBodyMTBase::createDefaultParameters()
{
    if (m_multithreadCapable)
    {
        // create a button to toggle multithreaded world
        ButtonParams button( "Multithreaded world enable", 0, true );
        button.m_userPointer = &gMultithreadedWorld;
        button.m_callback = boolPtrButtonCallback;
        m_guiHelper->getParameterInterface()->registerButtonParameter( button );
    }
    {
        // create a button to toggle profile printing
        ButtonParams button( "Display profile timings", 0, true );
        button.m_userPointer = &gDisplayProfileInfo;
        button.m_callback = boolPtrButtonCallback;
        m_guiHelper->getParameterInterface()->registerButtonParameter( button );
    }
    {
        SliderParams slider( "Solver iterations", &gSliderSolverIterations );
        slider.m_minVal = 1.0f;
        slider.m_maxVal = 30.0f;
        slider.m_callback = setSolverIterationCountCallback;
        slider.m_userPointer = m_dynamicsWorld;
        slider.m_clampToIntegers = true;
        m_guiHelper->getParameterInterface()->registerSliderFloatParameter( slider );
    }
    if (m_multithreadedWorld)
    {
        // create a button for each supported threading API
        for (int iApi = 0; iApi < TaskManager::apiCount; ++iApi)
        {
            TaskManager::Api api = static_cast<TaskManager::Api>(iApi);
            if (gTaskMgr.isSupported(api))
            {
                char str[1024];
                sprintf(str, "API %s", gTaskMgr.getApiName(api));
                ButtonParams button( str, iApi, false );
                button.m_callback = apiSelectButtonCallback;
                m_guiHelper->getParameterInterface()->registerButtonParameter( button );
            }
        }
        {
            // create a slider to set the number of threads to use
            gSliderNumThreads = float(gTaskMgr.getNumThreads());
			SliderParams slider("Thread count", &gSliderNumThreads);
			slider.m_minVal = 1.0f;
			slider.m_maxVal = float(gTaskMgr.getMaxNumThreads()*2);
			slider.m_callback = setThreadCountCallback;
            slider.m_clampToIntegers = true;
            m_guiHelper->getParameterInterface()->registerSliderFloatParameter( slider );
        }
    }
}


void CommonRigidBodyMTBase::drawScreenText()
{
    char msg[ 1024 ];
    int xCoord = 400;
    int yCoord = 30;
    int yStep = 30;
    if (m_multithreadCapable)
    {
        if ( m_multithreadedWorld != gMultithreadedWorld )
        {
            sprintf( msg, "restart example to begin in %s mode",
                     gMultithreadedWorld ? "multithreaded" : "single threaded"
                     );
            m_guiHelper->getAppInterface()->drawText( msg, 300, yCoord, 0.4f );
            yCoord += yStep;
        }
    }
    if (gDisplayProfileInfo)
    {
        if ( m_multithreadedWorld )
        {
            int numManifolds = m_dispatcher->getNumManifolds();
            int numContacts = 0;
            for ( int i = 0; i < numManifolds; ++i )
            {
                const btPersistentManifold* man = m_dispatcher->getManifoldByIndexInternal( i );
                numContacts += man->getNumContacts();
            }
            const char* mtApi = TaskManager::getApiName( gTaskMgr.getApi() );
            sprintf( msg, "islands=%d bodies=%d manifolds=%d contacts=%d [%s] threads=%d",
                     gNumIslands,
                     m_dynamicsWorld->getNumCollisionObjects(),
                     numManifolds,
                     numContacts,
                     mtApi,
                     gTaskMgr.getApi() == TaskManager::apiNone ? 1 : gTaskMgr.getNumThreads()
                     );
            m_guiHelper->getAppInterface()->drawText( msg, 100, yCoord, 0.4f );
            yCoord += yStep;
        }

        sprintf( msg, "internalSimStep %5.3f ms",
                 gProfiler.getAverageTime( Profiler::kRecordInternalTimeStep )*0.001f
                 );
        m_guiHelper->getAppInterface()->drawText( msg, xCoord, yCoord, 0.4f );
        yCoord += yStep;

        if ( m_multithreadedWorld )
        {
            sprintf( msg,
                     "DispatchCollisionPairs %5.3f ms",
                     gProfiler.getAverageTime( Profiler::kRecordDispatchAllCollisionPairs )*0.001f
                     );
            m_guiHelper->getAppInterface()->drawText( msg, xCoord, yCoord, 0.4f );
            yCoord += yStep;

            sprintf( msg,
                     "SolveAllIslands %5.3f ms",
                     gProfiler.getAverageTime( Profiler::kRecordDispatchIslands )*0.001f
                     );
            m_guiHelper->getAppInterface()->drawText( msg, xCoord, yCoord, 0.4f );
            yCoord += yStep;

            sprintf( msg,
                     "PredictUnconstrainedMotion %5.3f ms",
                     gProfiler.getAverageTime( Profiler::kRecordPredictUnconstrainedMotion )*0.001f
                     );
            m_guiHelper->getAppInterface()->drawText( msg, xCoord, yCoord, 0.4f );
            yCoord += yStep;

            sprintf( msg,
                     "CreatePredictiveContacts %5.3f ms",
                     gProfiler.getAverageTime( Profiler::kRecordCreatePredictiveContacts )*0.001f
                     );
            m_guiHelper->getAppInterface()->drawText( msg, xCoord, yCoord, 0.4f );
            yCoord += yStep;

            sprintf( msg,
                     "IntegrateTransforms %5.3f ms",
                     gProfiler.getAverageTime( Profiler::kRecordIntegrateTransforms )*0.001f
                     );
            m_guiHelper->getAppInterface()->drawText( msg, xCoord, yCoord, 0.4f );
            yCoord += yStep;
        }
    }
}


void CommonRigidBodyMTBase::physicsDebugDraw(int debugFlags)
{
	if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		m_dynamicsWorld->debugDrawWorld();
	}
	drawScreenText();
}


void CommonRigidBodyMTBase::renderScene()
{
    m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
    m_guiHelper->render(m_dynamicsWorld);
    drawScreenText();
}
