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

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"  // for setSplitIslands()
#include "BulletDynamics/ConstraintSolver/btSolverBody.h"  // for BT_ENABLE_PARALLEL_SOLVER

#include <algorithm>

// choose threading provider (set one of these macros to 1):
#define USE_TBB 1     // use Intel Threading Building Blocks for thread management
#define USE_OPENMP 0  // use OpenMP (also need to change compiler options for OpenMP support)

#if USE_OPENMP

#include <omp.h>
#undef USE_TBB

#elif USE_TBB

#define __TBB_NO_IMPLICIT_LINKAGE 1
#include <tbb/tbb.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

tbb::task_scheduler_init* gTaskSchedulerInit;

#endif

bool gEnableThreading = true;

#define USE_PARALLEL_DISPATCHER 1  // detect collisions in parallel
#define USE_PARALLEL_SOLVER 0      // experimental parallel solver (convert manifold contacts to solver constraints in parallel)
#define USE_PARALLEL_ISLAND_SOLVER 1   // solve simulation islands in parallel


class ProfileHistory
{
public:
    static const int kSamples = 20;
    float mTimeThisFrame;
    int mCallCount;
    int mIndex;
    btMutex mMutex;
    float mHistory[ kSamples ];

    ProfileHistory()
    {
        mIndex = -1;
        mTimeThisFrame = 0.0f;
        mCallCount = 0;
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
    void addCallTime( float time )
    {
#if BT_THREADSAFE
        // user provided mutex lock func might be instrumented with profiling, so 
        // we can't call it or we risk infinite recursion
        btMutexLockInternal( &mMutex );
#endif
        mTimeThisFrame += time;
        mCallCount++;
#if BT_THREADSAFE
        btMutexUnlock( &mMutex );
#endif
    }
    void nextFrame()
    {
        addSample( mTimeThisFrame );
        mTimeThisFrame = 0.0f;
        mCallCount = 0;
    }
    float getMin() const
    {
        float tMin = mHistory[ 0 ];
        for ( int i = 0; i < kSamples; ++i )
        {
            tMin = (std::min)( tMin, mHistory[ i ] );
        }
        return tMin;
    }
    float getMax() const
    {
        float tMax = mHistory[ 0 ];
        for ( int i = 0; i < kSamples; ++i )
        {
			tMax = (std::max)(tMax, mHistory[i]);
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

class ProfileInstance
{
    ProfileHistory* mHistory;
    btClock mClock;

public:
    ProfileInstance( ProfileHistory* hist )
    {
        mHistory = hist;
    }
    ~ProfileInstance()
    {
        float time = mClock.getTimeSeconds();
        mHistory->addCallTime( time );
    }
};

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

    virtual void dispatchAllCollisionPairs( btOverlappingPairCache* pairCache, const btDispatcherInfo& info, btDispatcher* dispatcher ) BT_OVERRIDE
    {
#if USE_OPENMP
        if ( gEnableThreading )
        {
            btNearCallback callback = getNearCallback();
            int pairCount = pairCache->getNumOverlappingPairs();
            btBroadphasePair* pairArray = pairCache->getOverlappingPairArrayPtr();

            btPushThreadsAreRunning();
            {
#pragma omp parallel for schedule(static, 40)
                for ( int i = 0; i < pairCount; ++i )
                {
                    btBroadphasePair* pair = &pairArray[ i ];
                    callback( *pair, *this, info );
                }
            }
            btPopThreadsAreRunning();
        }
        else
#elif USE_TBB
        if ( gEnableThreading )
        {
            btPushThreadsAreRunning();
            using namespace tbb;
            int grainSize = 40;  // iterations per task
            int pairCount = pairCache->getNumOverlappingPairs();
            TbbUpdater tbbUpdate;
            tbbUpdate.mCallback = getNearCallback();
            tbbUpdate.mPairArray = pairCache->getOverlappingPairArrayPtr();
            tbbUpdate.mDispatcher = this;
            tbbUpdate.mInfo = &info;

            parallel_for( blocked_range<int>( 0, pairCount, grainSize ), tbbUpdate, simple_partitioner() );
            btPopThreadsAreRunning();
        }
        else
#endif
        {
            // serial dispatching
            btNearCallback callback = getNearCallback();
            int pairCount = pairCache->getNumOverlappingPairs();
            btBroadphasePair* pairArray = pairCache->getOverlappingPairArrayPtr();

            for ( int i = 0; i < pairCount; ++i )
            {
                btBroadphasePair* pair = &pairArray[ i ];
                callback( *pair, *this, info );
            }
        }
    }
};

#endif


#if USE_PARALLEL_SOLVER

ATTRIBUTE_ALIGNED16( class ) MySequentialImpulseConstraintSolver : public btSequentialImpulseConstraintSolver
{
    typedef btSequentialImpulseConstraintSolver ParentClass;
#if USE_TBB
    struct TbbUpdater
    {
        MySequentialImpulseConstraintSolver* mThis = nullptr;
        btPersistentManifold** mManifoldPtr = nullptr;
        const btContactSolverInfo* mInfo = nullptr;

        void operator()( const tbb::blocked_range<int>& range ) const
        {
            for ( int i = range.begin(); i != range.end(); ++i )
            {
                btPersistentManifold* manifold = mManifoldPtr[ i ];
                mThis->convertContact( manifold, *mInfo );
            }
        }
    };
#endif

public:
    BT_DECLARE_ALIGNED_ALLOCATOR();
    MySequentialImpulseConstraintSolver() {}
    virtual ~MySequentialImpulseConstraintSolver() {}

    virtual void convertContacts( btPersistentManifold** manifoldPtr, int numManifolds, const btContactSolverInfo& infoGlobal ) BT_OVERRIDE
    {
        //ProfileInstance prof( &gProfConvertContacts );
#if USE_OPENMP && BT_ENABLE_PARALLEL_SOLVER
        if ( gEnableThreading )
        {
            btPushThreadsAreRunning();
#pragma omp parallel for schedule(static, 40)
            for ( int i = 0; i < numManifolds; i++ )
            {
                btPersistentManifold* manifold = manifoldPtr[ i ];
                convertContact( manifold, infoGlobal );
            }
            btPopThreadsAreRunning();
        }
        else
#elif USE_TBB && BT_ENABLE_PARALLEL_SOLVER
        if ( gEnableThreading )
        {
            btPushThreadsAreRunning();
            using namespace tbb;
            TbbUpdater tbbUpdate;
            tbbUpdate.mManifoldPtr = manifoldPtr;
            tbbUpdate.mThis = this;
            tbbUpdate.mInfo = &infoGlobal;

            parallel_for( blocked_range<int>( 0, numManifolds, 40 ), tbbUpdate, simple_partitioner() );
            btPopThreadsAreRunning();
        }
        else
#endif
        {
            // single threaded
            for ( int i = 0; i < numManifolds; i++ )
            {
                btPersistentManifold* manifold = manifoldPtr[ i ];
                convertContact( manifold, infoGlobal );
            }
        }
    }
};

#endif //#if USE_PARALLEL_SOLVER

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
            btConstraintSolver* solver;
#if USE_PARALLEL_SOLVER
            solver = new MySequentialImpulseConstraintSolver();
#else
            solver = new btSequentialImpulseConstraintSolver();
#endif //USE_PARALLEL_SOLVER
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

#if USE_TBB
struct TbbIslandDispatcher
{
    btAlignedObjectArray<btSimulationIslandManager::Island*>* islandsPtr;
    btSimulationIslandManager::IslandCallback* callback;

    void operator()( const tbb::blocked_range<int>& range ) const
    {
        for ( int i = range.begin(); i != range.end(); ++i )
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
#endif

void parallelIslandDispatch( btAlignedObjectArray<btSimulationIslandManager::Island*>* islandsPtr, btSimulationIslandManager::IslandCallback* callback )
{
#if USE_OPENMP
    if ( gEnableThreading )
    {
        btAlignedObjectArray<btSimulationIslandManager::Island*>& islands = *islandsPtr;
#pragma omp parallel for schedule(static, 1)
        for ( int i = 0; i < islands.size(); ++i )
        {
            btSimulationIslandManager::Island* island = islands[ i ];
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
    else
#elif USE_TBB
    if ( gEnableThreading )
    {
        using namespace tbb;
        int grainSize = 1;  // iterations per task
        TbbIslandDispatcher dispatcher;
        dispatcher.islandsPtr = islandsPtr;
        dispatcher.callback = callback;
        parallel_for( blocked_range<int>( 0, islandsPtr->size(), grainSize ), dispatcher, simple_partitioner() );
    }
    else
#endif
    {
        btSimulationIslandManager::defaultIslandDispatch( islandsPtr, callback );
    }
}


ProfileHistory gProfInternalSingleStepSimulation;
ProfileHistory gProfPerformDiscreteCollisionDetection;
ProfileHistory gProfUpdateAabbs;
ProfileHistory gProfPredictUnconstraintMotion;
ProfileHistory gProfIntegrateTransforms;
ProfileHistory gProfCreatePredictiveContacts;
ProfileHistory gProfSolveConstraints;
ProfileHistory gProfComputeOverlappingPairs;
ProfileHistory gProfStepSimulation;

///
/// MyDiscreteDynamicsWorld
///
///  Need to override a few methods of btDiscreteDynamicsWorld for profiling
///
ATTRIBUTE_ALIGNED16( class ) MyDiscreteDynamicsWorld : public btDiscreteDynamicsWorld
{
    typedef btDiscreteDynamicsWorld ParentClass;

#if USE_TBB
    struct TbbUpdaterUnconstrainedMotion
    {
        btScalar timeStep;
        btRigidBody** rigidBodies;

        void operator()( const tbb::blocked_range<int>& range ) const
        {
            for ( int i = range.begin(); i != range.end(); ++i )
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
    struct TbbUpdaterIntegrateTransforms
    {
        btScalar timeStep;
        btRigidBody** rigidBodies;
        const MyDiscreteDynamicsWorld* world;

        void operator()( const tbb::blocked_range<int>& range ) const
        {
            world->integrateTransformsInternal( &rigidBodies[ range.begin() ], range.end() - range.begin(), timeStep );
        }
    };
    struct TbbUpdaterCreatePredictiveContacts
    {
        btScalar timeStep;
        btRigidBody** rigidBodies;
        MyDiscreteDynamicsWorld* world;

        void operator()( const tbb::blocked_range<int>& range ) const
        {
            world->createPredictiveContactsInternal( &rigidBodies[ range.begin() ], range.end() - range.begin(), timeStep );
        }
    };
#endif // USE_TBB

protected:
    virtual void predictUnconstraintMotion( btScalar timeStep ) override
    {
        ProfileInstance prof( &gProfPredictUnconstraintMotion );
#if USE_OPENMP
        if ( gEnableThreading )
        {
#pragma omp parallel for schedule(static, 50)
            for ( int i = 0; i<m_nonStaticRigidBodies.size(); i++ )
            {
                btRigidBody* body = m_nonStaticRigidBodies[i];
                if (!body->isStaticOrKinematicObject())
                {
                    //don't integrate/update velocities here, it happens in the constraint solver
                    body->applyDamping(timeStep);
                    body->predictIntegratedTransform(timeStep,body->getInterpolationWorldTransform());
                }
            }
        }
        else
#elif USE_TBB
        if ( gEnableThreading )
        {
            using namespace tbb;
            int grainSize = 50;  // num of iterations per task for TBB
            int bodyCount = m_nonStaticRigidBodies.size();
            TbbUpdaterUnconstrainedMotion tbbUpdate;
            tbbUpdate.timeStep = timeStep;
            tbbUpdate.rigidBodies = bodyCount ? &m_nonStaticRigidBodies[ 0 ] : nullptr;
            btPushThreadsAreRunning();
            parallel_for( blocked_range<int>( 0, bodyCount, grainSize ), tbbUpdate, simple_partitioner() );
            btPopThreadsAreRunning();
        }
        else
#endif
        {
            ParentClass::predictUnconstraintMotion( timeStep );
        }
    }
    virtual void createPredictiveContacts( btScalar timeStep )
    {
        ProfileInstance prof( &gProfCreatePredictiveContacts );
        int grainSize = 50;  // num of iterations per task for TBB or OPENMP
#if USE_OPENMP
        if ( gEnableThreading )
        {
            int numBodies = m_nonStaticRigidBodies.size();
#pragma omp parallel for schedule(static, 1)
            for ( int i = 0; i < numBodies; i += grainSize )
            {
                createPredictiveContactsInternal( &m_nonStaticRigidBodies[ i ], min(grainSize, numBodies - i), timeStep );
            }
        }
        else
#elif USE_TBB
        if ( gEnableThreading )
        {
            if ( int bodyCount = m_nonStaticRigidBodies.size() )
            {
                using namespace tbb;
                TbbUpdaterCreatePredictiveContacts tbbUpdate;
                tbbUpdate.world = this;
                tbbUpdate.timeStep = timeStep;
                tbbUpdate.rigidBodies = &m_nonStaticRigidBodies[ 0 ];
                btPushThreadsAreRunning();
                parallel_for( blocked_range<int>( 0, bodyCount, grainSize ), tbbUpdate, simple_partitioner() );
                btPopThreadsAreRunning();
            }
        }
        else
#endif
        {
            ParentClass::createPredictiveContacts( timeStep );
        }
    }
    virtual void integrateTransforms( btScalar timeStep ) override
    {
        ProfileInstance prof( &gProfIntegrateTransforms );
        int grainSize = 50;  // num of iterations per task for TBB or OPENMP
#if USE_OPENMP
        if ( gEnableThreading )
        {
            int numBodies = m_nonStaticRigidBodies.size();
#pragma omp parallel for schedule(static, 1)
            for ( int i = 0; i < numBodies; i += grainSize )
            {
                integrateTransformsInternal( &m_nonStaticRigidBodies[ i ], min( grainSize, numBodies - i ), timeStep );
            }
        }
        else
#elif USE_TBB
        if ( gEnableThreading )
        {
            if ( int bodyCount = m_nonStaticRigidBodies.size() )
            {
                using namespace tbb;
                TbbUpdaterIntegrateTransforms tbbUpdate;
                tbbUpdate.world = this;
                tbbUpdate.timeStep = timeStep;
                tbbUpdate.rigidBodies = &m_nonStaticRigidBodies[ 0 ];
                btPushThreadsAreRunning();
                parallel_for( blocked_range<int>( 0, bodyCount, grainSize ), tbbUpdate, simple_partitioner() );
                btPopThreadsAreRunning();
            }
        }
        else
#endif
        {
            ParentClass::integrateTransforms( timeStep );
        }
    }
    virtual void calculateSimulationIslands() BT_OVERRIDE
    {
        ParentClass::calculateSimulationIslands();
    }
    virtual void solveConstraints( btContactSolverInfo& solverInfo ) BT_OVERRIDE
    {
        ProfileInstance prof( &gProfSolveConstraints );
        ParentClass::solveConstraints( solverInfo );
    }
    virtual void internalSingleStepSimulation( btScalar timeStep ) BT_OVERRIDE
    {
        ProfileInstance prof( &gProfInternalSingleStepSimulation );
        ParentClass::internalSingleStepSimulation( timeStep );
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

    virtual void performDiscreteCollisionDetection() BT_OVERRIDE
    {
        ProfileInstance prof( &gProfPerformDiscreteCollisionDetection );
        ParentClass::performDiscreteCollisionDetection();
    }

    virtual void updateAabbs() BT_OVERRIDE
    {
        ProfileInstance prof( &gProfUpdateAabbs );
        ParentClass::updateAabbs();
    }

    virtual void computeOverlappingPairs() BT_OVERRIDE
    {
        ProfileInstance prof( &gProfComputeOverlappingPairs );
        ParentClass::computeOverlappingPairs();
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

	if (m_dynamicsWorld)
	{
        {
            ProfileInstance prof( &gProfStepSimulation );
            m_dynamicsWorld->stepSimulation( 1.0f / 60.f, 0 );
        }
        int y = 20;
        int yStep = 20;
        char msg[ 128 ];
        {
            int numManifolds = m_dispatcher->getNumManifolds();
            int numContacts = 0;
            for ( int i = 0; i < numManifolds; ++i )
            {
                const btPersistentManifold* man = m_dispatcher->getManifoldByIndexInternal( i );
                numContacts += man->getNumContacts();
            }
#if USE_OPENMP
            const char* mtApi = "OpenMP";
#elif USE_TBB
            const char* mtApi = "TBB";
#else
            const char* mtApi = "";
#endif
            sprintf( msg, "manifolds %d contacts %d [%s]",
                     numManifolds,
                     numContacts,
                     gEnableThreading ? mtApi : "single-threaded"
                     );
            displayProfileString( 10, y, msg );
            y += yStep;
        }
        {
            ProfileHistory* prof = &gProfStepSimulation;
            prof->nextFrame();
            sprintf( msg, "stepSimulation time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
                     prof->getMin()*1000.0f,
                     prof->getAvg()*1000.0f,
                     prof->getMax()*1000.0f
                     );
            displayProfileString( 10, y, msg );
            y += yStep;
        }
        {
            ProfileHistory* prof = &gProfPredictUnconstraintMotion;
            prof->nextFrame();
            sprintf( msg, "pred. unconstrained motion time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
                     prof->getMin()*1000.0f,
                     prof->getAvg()*1000.0f,
                     prof->getMax()*1000.0f
                     );
            displayProfileString( 10, y, msg );
            y += yStep;
        }
        {
            ProfileHistory* prof = &gProfCreatePredictiveContacts;
            prof->nextFrame();
            sprintf( msg, "create predictive contacts time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
                     prof->getMin()*1000.0f,
                     prof->getAvg()*1000.0f,
                     prof->getMax()*1000.0f
                     );
            displayProfileString( 10, y, msg );
            y += yStep;
        }
        {
            ProfileHistory* prof = &gProfIntegrateTransforms;
            prof->nextFrame();
            sprintf( msg, "integrate transforms time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
                     prof->getMin()*1000.0f,
                     prof->getAvg()*1000.0f,
                     prof->getMax()*1000.0f
                     );
            displayProfileString( 10, y, msg );
            y += yStep;
        }
        {
            ProfileHistory* prof = &gProfPerformDiscreteCollisionDetection;
            prof->nextFrame();
            sprintf( msg, "collision detection time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
                     prof->getMin()*1000.0f,
                     prof->getAvg()*1000.0f,
                     prof->getMax()*1000.0f
                     );
            displayProfileString( 10, y, msg );
            y += yStep;
        }
        {
            ProfileHistory* prof = &gProfSolveConstraints;
            prof->nextFrame();
            sprintf( msg, "solve constraints time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
                     prof->getMin()*1000.0f,
                     prof->getAvg()*1000.0f,
                     prof->getMax()*1000.0f
                     );
            displayProfileString( 10, y, msg );
            y += yStep;
        }
        // if parallel island dispatch is used, these profile results are misleading because
        //  timings from various hardware threads get added together
        //{
        //    ProfileHistory* prof = &gProfConvertContacts;
        //    prof->nextFrame();
        //    sprintf( msg, "convert contacts time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
        //             prof->getMin()*1000.0f,
        //             prof->getAvg()*1000.0f,
        //             prof->getMax()*1000.0f
        //             );
        //    displayProfileString( 10, y, msg );
        //    y += yStep;
        //}
        //{
        //    ProfileHistory* prof = &gProfSolveGroupCacheFriendlyIterations;
        //    prof->nextFrame();
        //    sprintf( msg, "solve cf iterations time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
        //             prof->getMin()*1000.0f,
        //             prof->getAvg()*1000.0f,
        //             prof->getMax()*1000.0f
        //             );
        //    displayProfileString( 10, y, msg );
        //    y += yStep;
        //}
        //{
        //    ProfileHistory* prof = &gProfSolveGroupCacheFriendlyFinish;
        //    prof->nextFrame();
        //    sprintf( msg, "solve cf finish time=%5.5f / %5.5f / %5.5f ms (min/avg/max)",
        //             prof->getMin()*1000.0f,
        //             prof->getAvg()*1000.0f,
        //             prof->getMax()*1000.0f
        //             );
        //    displayProfileString( 10, y, msg );
        //    y += yStep;
        //}
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

void MultiThreadedDemo::keyboardCallback( unsigned char key, int x, int y )
{
    PlatformDemoApplication::keyboardCallback( key, x, y );
    if ( key == 'm' )
    {
        // 'm' toggles between multi-threading and single-threading
        gEnableThreading = !gEnableThreading;
    }
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

MultiThreadedDemo* gMultiThreadedDemo;

void	MultiThreadedDemo::initPhysics()
{
    gMultiThreadedDemo = this; // for debugging
    int numThreads = 4;
#if USE_OPENMP
    omp_set_num_threads( numThreads );
#elif USE_TBB
    gTaskSchedulerInit = new tbb::task_scheduler_init( numThreads );
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

	m_dispatcher=NULL;
	btDefaultCollisionConstructionInfo cci;
	cci.m_defaultMaxPersistentManifoldPoolSize = 32768;
    cci.m_defaultMaxCollisionAlgorithmPoolSize = 32768;
	m_collisionConfiguration = new btDefaultCollisionConfiguration(cci);
	
#if USE_PARALLEL_DISPATCHER
	m_dispatcher = new	MyCollisionDispatcher(m_collisionConfiguration);
#else
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER

    if ( false )
    {
        btVector3 worldAabbMin( -1000, -1000, -1000 );
        btVector3 worldAabbMax( 1000, 1000, 1000 );

        m_broadphase = new btAxisSweep3( worldAabbMin, worldAabbMax, maxProxies );
    }
    else
    {
        m_broadphase = new btDbvtBroadphase();
    }

    //this solver requires the contacts to be in a contiguous pool, so avoid dynamic allocation
    m_dispatcher->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);

#if USE_PARALLEL_ISLAND_SOLVER
    m_solver = new MyConstraintSolverPool( numThreads );
#elif USE_PARALLEL_SOLVER
    m_solver = new MySequentialImpulseConstraintSolver();
#else
    m_solver = new btSequentialImpulseConstraintSolver();
#endif //#if USE_PARALLEL_ISLAND_SOLVER
    btDiscreteDynamicsWorld* world = new MyDiscreteDynamicsWorld( m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration );
    m_dynamicsWorld = world;

    world->getSimulationIslandManager()->setIslandDispatchFunction( parallelIslandDispatch );
    //world->getSolverInfo().m_numIterations = 4;
    //default solverMode is SOLVER_RANDMIZE_ORDER. Warmstarting seems not to improve convergence, see 
    //solver->setSolverMode(0);//btSequentialImpulseConstraintSolver::SOLVER_USE_WARMSTARTING | btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
    world->getSolverInfo().m_solverMode = SOLVER_SIMD + SOLVER_USE_WARMSTARTING;//+SOLVER_RANDMIZE_ORDER;

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


