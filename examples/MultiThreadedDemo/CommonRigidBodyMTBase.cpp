/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

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
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btPoolAllocator.h"
#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletDynamics/Dynamics/btSimulationIslandManagerMt.h"  // for setSplitIslands()
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"

static int gNumIslands = 0;
bool gAllowNestedParallelForLoops = false;

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
		kRecordSolverTotal,
		kRecordSolverSetup,
		kRecordSolverIterations,
		kRecordSolverFinish,
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
			mHistory[mCallCount & 7] = elapsed;
			++mCallCount;
		}
		float getAverageTime() const
		{
			int count = btMin(8, mCallCount);
			if (count > 0)
			{
				unsigned int sum = 0;
				for (int i = 0; i < count; ++i)
				{
					sum += mHistory[i];
				}
				float avg = float(sum) / float(count);
				return avg;
			}
			return 0.0;
		}
	};
	Record mRecords[kRecordCount];

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

static Profiler gProfiler;

class ProfileHelper
{
	Profiler::RecordType mRecType;

public:
	ProfileHelper(Profiler::RecordType rt)
	{
		mRecType = rt;
		gProfiler.begin(mRecType);
	}
	~ProfileHelper()
	{
		gProfiler.end(mRecType);
	}
};

static void profileBeginCallback(btDynamicsWorld* /*world*/, btScalar /*timeStep*/)
{
	gProfiler.begin(Profiler::kRecordInternalTimeStep);
}

static void profileEndCallback(btDynamicsWorld* /*world*/, btScalar /*timeStep*/)
{
	gProfiler.end(Profiler::kRecordInternalTimeStep);
}

class MySequentialImpulseConstraintSolverMt : public btSequentialImpulseConstraintSolverMt
{
	typedef btSequentialImpulseConstraintSolverMt ParentClass;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	MySequentialImpulseConstraintSolverMt() {}

	// for profiling
	virtual btScalar solveGroupCacheFriendlySetup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer) BT_OVERRIDE
	{
		ProfileHelper prof(Profiler::kRecordSolverSetup);
		btScalar ret = ParentClass::solveGroupCacheFriendlySetup(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
		return ret;
	}
	virtual btScalar solveGroupCacheFriendlyIterations(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer) BT_OVERRIDE
	{
		ProfileHelper prof(Profiler::kRecordSolverIterations);
		btScalar ret = ParentClass::solveGroupCacheFriendlyIterations(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
		return ret;
	}
	virtual btScalar solveGroupCacheFriendlyFinish(btCollisionObject** bodies, int numBodies, const btContactSolverInfo& infoGlobal) BT_OVERRIDE
	{
		ProfileHelper prof(Profiler::kRecordSolverFinish);
		btScalar ret = ParentClass::solveGroupCacheFriendlyFinish(bodies, numBodies, infoGlobal);
		return ret;
	}
	virtual btScalar solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher) BT_OVERRIDE
	{
		ProfileHelper prof(Profiler::kRecordSolverTotal);
		btScalar ret = ParentClass::solveGroup(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer, dispatcher);
		return ret;
	}
};

///
/// MyCollisionDispatcher -- subclassed for profiling purposes
///
class MyCollisionDispatcher : public btCollisionDispatcherMt
{
	typedef btCollisionDispatcherMt ParentClass;

public:
	MyCollisionDispatcher(btCollisionConfiguration* config, int grainSize) : btCollisionDispatcherMt(config, grainSize)
	{
	}

	virtual void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& info, btDispatcher* dispatcher) BT_OVERRIDE
	{
		ProfileHelper prof(Profiler::kRecordDispatchAllCollisionPairs);
		ParentClass::dispatchAllCollisionPairs(pairCache, info, dispatcher);
	}
};

///
/// myParallelIslandDispatch -- wrap default parallel dispatch for profiling and to get the number of simulation islands
//
void myParallelIslandDispatch(btAlignedObjectArray<btSimulationIslandManagerMt::Island*>* islandsPtr, const btSimulationIslandManagerMt::SolverParams& solverParams)
{
	ProfileHelper prof(Profiler::kRecordDispatchIslands);
	gNumIslands = islandsPtr->size();
	btSimulationIslandManagerMt::parallelIslandDispatch(islandsPtr, solverParams);
}

///
/// MyDiscreteDynamicsWorld -- subclassed for profiling purposes
///
ATTRIBUTE_ALIGNED16(class)
MyDiscreteDynamicsWorld : public btDiscreteDynamicsWorldMt
{
	typedef btDiscreteDynamicsWorldMt ParentClass;

protected:
	virtual void predictUnconstraintMotion(btScalar timeStep) BT_OVERRIDE
	{
		ProfileHelper prof(Profiler::kRecordPredictUnconstrainedMotion);
		ParentClass::predictUnconstraintMotion(timeStep);
	}
	virtual void createPredictiveContacts(btScalar timeStep) BT_OVERRIDE
	{
		ProfileHelper prof(Profiler::kRecordCreatePredictiveContacts);
		ParentClass::createPredictiveContacts(timeStep);
	}
	virtual void integrateTransforms(btScalar timeStep) BT_OVERRIDE
	{
		ProfileHelper prof(Profiler::kRecordIntegrateTransforms);
		ParentClass::integrateTransforms(timeStep);
	}

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	MyDiscreteDynamicsWorld(btDispatcher * dispatcher,
							btBroadphaseInterface * pairCache,
							btConstraintSolverPoolMt * constraintSolver,
							btSequentialImpulseConstraintSolverMt * constraintSolverMt,
							btCollisionConfiguration * collisionConfiguration) : btDiscreteDynamicsWorldMt(dispatcher, pairCache, constraintSolver, constraintSolverMt, collisionConfiguration)
	{
		btSimulationIslandManagerMt* islandMgr = static_cast<btSimulationIslandManagerMt*>(m_islandManager);
		islandMgr->setIslandDispatchFunction(myParallelIslandDispatch);
	}
};

btConstraintSolver* createSolverByType(SolverType t)
{
	btMLCPSolverInterface* mlcpSolver = NULL;
	switch (t)
	{
		case SOLVER_TYPE_SEQUENTIAL_IMPULSE:
			return new btSequentialImpulseConstraintSolver();
		case SOLVER_TYPE_SEQUENTIAL_IMPULSE_MT:
			return new MySequentialImpulseConstraintSolverMt();
		case SOLVER_TYPE_NNCG:
			return new btNNCGConstraintSolver();
		case SOLVER_TYPE_MLCP_PGS:
			mlcpSolver = new btSolveProjectedGaussSeidel();
			break;
		case SOLVER_TYPE_MLCP_DANTZIG:
			mlcpSolver = new btDantzigSolver();
			break;
		case SOLVER_TYPE_MLCP_LEMKE:
			mlcpSolver = new btLemkeSolver();
			break;
		default:
		{
		}
	}
	if (mlcpSolver)
	{
		return new btMLCPSolver(mlcpSolver);
	}
	return NULL;
}

///
/// btTaskSchedulerManager -- manage a number of task schedulers so we can switch between them
///
class btTaskSchedulerManager
{
	btAlignedObjectArray<btITaskScheduler*> m_taskSchedulers;
	btAlignedObjectArray<btITaskScheduler*> m_allocatedTaskSchedulers;

public:
	btTaskSchedulerManager() {}
	void init()
	{
		addTaskScheduler(btGetSequentialTaskScheduler());
#if BT_THREADSAFE
		if (btITaskScheduler* ts = btCreateDefaultTaskScheduler())
		{
			m_allocatedTaskSchedulers.push_back(ts);
			addTaskScheduler(ts);
		}
		addTaskScheduler(btGetOpenMPTaskScheduler());
		addTaskScheduler(btGetTBBTaskScheduler());
		addTaskScheduler(btGetPPLTaskScheduler());
		if (getNumTaskSchedulers() > 1)
		{
			// prefer a non-sequential scheduler if available
			btSetTaskScheduler(m_taskSchedulers[1]);
		}
		else
		{
			btSetTaskScheduler(m_taskSchedulers[0]);
		}
#endif  // #if BT_THREADSAFE
	}
	void shutdown()
	{
		for (int i = 0; i < m_allocatedTaskSchedulers.size(); ++i)
		{
			delete m_allocatedTaskSchedulers[i];
		}
		m_allocatedTaskSchedulers.clear();
	}

	void addTaskScheduler(btITaskScheduler* ts)
	{
		if (ts)
		{
#if BT_THREADSAFE
			// if initial number of threads is 0 or 1,
			if (ts->getNumThreads() <= 1)
			{
				// for OpenMP, TBB, PPL set num threads to number of logical cores
				ts->setNumThreads(ts->getMaxNumThreads());
			}
#endif  // #if BT_THREADSAFE
			m_taskSchedulers.push_back(ts);
		}
	}
	int getNumTaskSchedulers() const { return m_taskSchedulers.size(); }
	btITaskScheduler* getTaskScheduler(int i) { return m_taskSchedulers[i]; }
};

static btTaskSchedulerManager gTaskSchedulerMgr;

#if BT_THREADSAFE
static bool gMultithreadedWorld = true;
static bool gDisplayProfileInfo = true;
static SolverType gSolverType = SOLVER_TYPE_SEQUENTIAL_IMPULSE_MT;
#else
static bool gMultithreadedWorld = false;
static bool gDisplayProfileInfo = false;
static SolverType gSolverType = SOLVER_TYPE_SEQUENTIAL_IMPULSE;
#endif
static int gSolverMode = SOLVER_SIMD |
						 SOLVER_USE_WARMSTARTING |
						 // SOLVER_RANDMIZE_ORDER |
						 // SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS |
						 // SOLVER_USE_2_FRICTION_DIRECTIONS |
						 0;
static btScalar gSliderSolverIterations = 10.0f;                                                        // should be int
#if BT_THREADSAFE
static btScalar gSliderNumThreads = 1.0f;                                                               // should be int
static btScalar gSliderIslandBatchingThreshold = 0.0f;                                                  // should be int
static btScalar gSliderMinBatchSize = btScalar(btSequentialImpulseConstraintSolverMt::s_minBatchSize);  // should be int
static btScalar gSliderMaxBatchSize = btScalar(btSequentialImpulseConstraintSolverMt::s_maxBatchSize);  // should be int
#endif
static btScalar gSliderLeastSquaresResidualThreshold = 0.0f;

////////////////////////////////////
CommonRigidBodyMTBase::CommonRigidBodyMTBase(struct GUIHelperInterface* helper)
	: m_broadphase(0),
	  m_dispatcher(0),
	  m_solver(0),
	  m_collisionConfiguration(0),
	  m_dynamicsWorld(0),
	  m_pickedBody(0),
	  m_pickedConstraint(0),
	  m_guiHelper(helper)
{
	m_multithreadedWorld = false;
	m_multithreadCapable = false;
	if (gTaskSchedulerMgr.getNumTaskSchedulers() == 0)
	{
		gTaskSchedulerMgr.init();
	}
}

CommonRigidBodyMTBase::~CommonRigidBodyMTBase()
{
}

static void boolPtrButtonCallback(int /*buttonId*/, bool /*buttonState*/, void* userPointer)
{
	if (bool* val = static_cast<bool*>(userPointer))
	{
		*val = !*val;
	}
}

static void toggleSolverModeCallback(int buttonId, bool buttonState, void* userPointer)
{
	if (buttonState)
	{
		gSolverMode |= buttonId;
	}
	else
	{
		gSolverMode &= ~buttonId;
	}
	if (CommonRigidBodyMTBase* crb = reinterpret_cast<CommonRigidBodyMTBase*>(userPointer))
	{
		if (crb->m_dynamicsWorld)
		{
			crb->m_dynamicsWorld->getSolverInfo().m_solverMode = gSolverMode;
		}
	}
}

void setSolverTypeComboBoxCallback(int /*combobox*/, const char* item, void* userPointer)
{
	const char** items = static_cast<const char**>(userPointer);
	for (int i = 0; i < SOLVER_TYPE_COUNT; ++i)
	{
		if (strcmp(item, items[i]) == 0)
		{
			gSolverType = static_cast<SolverType>(i);
			break;
		}
	}
}

#if BT_THREADSAFE
static void setNumThreads(int numThreads)
{
	int newNumThreads = (std::min)(numThreads, int(BT_MAX_THREAD_COUNT));
	int oldNumThreads = btGetTaskScheduler()->getNumThreads();
	// only call when the thread count is different
	if (newNumThreads != oldNumThreads)
	{
		btGetTaskScheduler()->setNumThreads(newNumThreads);
	}
}
#endif  // #if BT_THREADSAFE

void setTaskSchedulerComboBoxCallback(int /*combobox*/, const char* item, void* userPointer)
{
#if BT_THREADSAFE
	const char** items = static_cast<const char**>(userPointer);
	for (int i = 0; i < 20; ++i)
	{
		if (strcmp(item, items[i]) == 0)
		{
			// change the task scheduler
			btITaskScheduler* ts = gTaskSchedulerMgr.getTaskScheduler(i);
			btSetTaskScheduler(ts);
			gSliderNumThreads = float(ts->getNumThreads());
			break;
		}
	}
#else
	(void)item;
	(void)userPointer;
#endif  // #if BT_THREADSAFE
}

void setBatchingMethodComboBoxCallback(int /*combobox*/, const char* item, void* userPointer)
{
#if BT_THREADSAFE
	const char** items = static_cast<const char**>(userPointer);
	for (int i = 0; i < btBatchedConstraints::BATCHING_METHOD_COUNT; ++i)
	{
		if (strcmp(item, items[i]) == 0)
		{
			// change the task scheduler
			btSequentialImpulseConstraintSolverMt::s_contactBatchingMethod = static_cast<btBatchedConstraints::BatchingMethod>(i);
			break;
		}
	}
#else
	(void)item;
	(void)userPointer;
#endif  // #if BT_THREADSAFE
}

#if BT_THREADSAFE
static void setThreadCountCallback(float /*val*/, void* /*userPtr*/)
{
	setNumThreads(int(gSliderNumThreads));
	gSliderNumThreads = float(btGetTaskScheduler()->getNumThreads());
}
#endif

static void setSolverIterationCountCallback(float /*val*/, void* userPtr)
{
	if (btDiscreteDynamicsWorld* world = reinterpret_cast<btDiscreteDynamicsWorld*>(userPtr))
	{
		world->getSolverInfo().m_numIterations = btMax(1, int(gSliderSolverIterations));
	}
}

#if BT_THREADSAFE
static void setLargeIslandManifoldCountCallback(float /*val*/, void* /*userPtr*/)
{
	btSequentialImpulseConstraintSolverMt::s_minimumContactManifoldsForBatching = int(gSliderIslandBatchingThreshold);
}

static void setMinBatchSizeCallback(float /*val*/, void* /*userPtr*/)
{
	gSliderMaxBatchSize = (std::max)(gSliderMinBatchSize, gSliderMaxBatchSize);
	btSequentialImpulseConstraintSolverMt::s_minBatchSize = int(gSliderMinBatchSize);
	btSequentialImpulseConstraintSolverMt::s_maxBatchSize = int(gSliderMaxBatchSize);
}

static void setMaxBatchSizeCallback(float /*val*/, void* /*userPtr*/)
{
	gSliderMinBatchSize = (std::min)(gSliderMinBatchSize, gSliderMaxBatchSize);
	btSequentialImpulseConstraintSolverMt::s_minBatchSize = int(gSliderMinBatchSize);
	btSequentialImpulseConstraintSolverMt::s_maxBatchSize = int(gSliderMaxBatchSize);
}
#endif

static void setLeastSquaresResidualThresholdCallback(float /*val*/, void* userPtr)
{
	if (btDiscreteDynamicsWorld* world = reinterpret_cast<btDiscreteDynamicsWorld*>(userPtr))
	{
		world->getSolverInfo().m_leastSquaresResidualThreshold = gSliderLeastSquaresResidualThreshold;
	}
}

void CommonRigidBodyMTBase::createEmptyDynamicsWorld()
{
	gNumIslands = 0;
	m_solverType = gSolverType;
#if BT_THREADSAFE
	btAssert(btGetTaskScheduler() != NULL);
	if (NULL != btGetTaskScheduler() && gTaskSchedulerMgr.getNumTaskSchedulers() > 1)
	{
		m_multithreadCapable = true;
	}
#endif
	if (gMultithreadedWorld)
	{
#if BT_THREADSAFE
		m_dispatcher = NULL;
		btDefaultCollisionConstructionInfo cci;
		cci.m_defaultMaxPersistentManifoldPoolSize = 80000;
		cci.m_defaultMaxCollisionAlgorithmPoolSize = 80000;
		m_collisionConfiguration = new btDefaultCollisionConfiguration(cci);

		m_dispatcher = new MyCollisionDispatcher(m_collisionConfiguration, 40);
		m_broadphase = new btDbvtBroadphase();

		btConstraintSolverPoolMt* solverPool;
		{
			SolverType poolSolverType = m_solverType;
			if (poolSolverType == SOLVER_TYPE_SEQUENTIAL_IMPULSE_MT)
			{
				// pool solvers shouldn't be parallel solvers, we don't allow that kind of
				// nested parallelism because of performance issues
				poolSolverType = SOLVER_TYPE_SEQUENTIAL_IMPULSE;
			}
			btConstraintSolver* solvers[BT_MAX_THREAD_COUNT];
			int maxThreadCount = BT_MAX_THREAD_COUNT;
			for (int i = 0; i < maxThreadCount; ++i)
			{
				solvers[i] = createSolverByType(poolSolverType);
			}
			solverPool = new btConstraintSolverPoolMt(solvers, maxThreadCount);
			m_solver = solverPool;
		}
		btSequentialImpulseConstraintSolverMt* solverMt = NULL;
		if (m_solverType == SOLVER_TYPE_SEQUENTIAL_IMPULSE_MT)
		{
			solverMt = new MySequentialImpulseConstraintSolverMt();
		}
		btDiscreteDynamicsWorld* world = new MyDiscreteDynamicsWorld(m_dispatcher, m_broadphase, solverPool, solverMt, m_collisionConfiguration);
		m_dynamicsWorld = world;
		m_multithreadedWorld = true;
		btAssert(btGetTaskScheduler() != NULL);
#endif  // #if BT_THREADSAFE
	}
	else
	{
		// single threaded world
		m_multithreadedWorld = false;

		///collision configuration contains default setup for memory, collision setup
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		//m_collisionConfiguration->setConvexConvexMultipointIterations();

		///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

		m_broadphase = new btDbvtBroadphase();

		SolverType solverType = m_solverType;
		if (solverType == SOLVER_TYPE_SEQUENTIAL_IMPULSE_MT)
		{
			// using the parallel solver with the single-threaded world works, but is
			// disabled here to avoid confusion
			solverType = SOLVER_TYPE_SEQUENTIAL_IMPULSE;
		}
		m_solver = createSolverByType(solverType);

		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	}
	m_dynamicsWorld->setInternalTickCallback(profileBeginCallback, NULL, true);
	m_dynamicsWorld->setInternalTickCallback(profileEndCallback, NULL, false);
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	m_dynamicsWorld->getSolverInfo().m_solverMode = gSolverMode;
	m_dynamicsWorld->getSolverInfo().m_numIterations = btMax(1, int(gSliderSolverIterations));
	createDefaultParameters();
}

void CommonRigidBodyMTBase::createDefaultParameters()
{
	if (m_multithreadCapable)
	{
		// create a button to toggle multithreaded world
		ButtonParams button("Multithreaded world enable", 0, true);
		bool* ptr = &gMultithreadedWorld;
		button.m_initialState = *ptr;
		button.m_userPointer = ptr;
		button.m_callback = boolPtrButtonCallback;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
	}
	{
		// create a button to toggle profile printing
		ButtonParams button("Display solver info", 0, true);
		bool* ptr = &gDisplayProfileInfo;
		button.m_initialState = *ptr;
		button.m_userPointer = ptr;
		button.m_callback = boolPtrButtonCallback;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
	}

	{
		// create a combo box for selecting the solver type
		static const char* sSolverTypeComboBoxItems[SOLVER_TYPE_COUNT];
		for (int i = 0; i < SOLVER_TYPE_COUNT; ++i)
		{
			SolverType solverType = static_cast<SolverType>(i);
			sSolverTypeComboBoxItems[i] = getSolverTypeName(solverType);
		}
		ComboBoxParams comboParams;
		comboParams.m_userPointer = sSolverTypeComboBoxItems;
		comboParams.m_numItems = SOLVER_TYPE_COUNT;
		comboParams.m_startItem = gSolverType;
		comboParams.m_items = sSolverTypeComboBoxItems;
		comboParams.m_callback = setSolverTypeComboBoxCallback;
		m_guiHelper->getParameterInterface()->registerComboBox(comboParams);
	}
	{
		// a slider for the number of solver iterations
		SliderParams slider("Solver iterations", &gSliderSolverIterations);
		slider.m_minVal = 1.0f;
		slider.m_maxVal = 30.0f;
		slider.m_callback = setSolverIterationCountCallback;
		slider.m_userPointer = m_dynamicsWorld;
		slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		// a slider for the solver leastSquaresResidualThreshold (used to run fewer solver iterations when convergence is good)
		SliderParams slider("Solver residual thresh", &gSliderLeastSquaresResidualThreshold);
		slider.m_minVal = 0.0f;
		slider.m_maxVal = 0.25f;
		slider.m_callback = setLeastSquaresResidualThresholdCallback;
		slider.m_userPointer = m_dynamicsWorld;
		slider.m_clampToIntegers = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		ButtonParams button("Solver use SIMD", 0, true);
		button.m_buttonId = SOLVER_SIMD;
		button.m_initialState = !!(gSolverMode & button.m_buttonId);
		button.m_callback = toggleSolverModeCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
	}
	{
		ButtonParams button("Solver randomize order", 0, true);
		button.m_buttonId = SOLVER_RANDMIZE_ORDER;
		button.m_initialState = !!(gSolverMode & button.m_buttonId);
		button.m_callback = toggleSolverModeCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
	}
	{
		ButtonParams button("Solver interleave contact/friction", 0, true);
		button.m_buttonId = SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS;
		button.m_initialState = !!(gSolverMode & button.m_buttonId);
		button.m_callback = toggleSolverModeCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
	}
	{
		ButtonParams button("Solver 2 friction directions", 0, true);
		button.m_buttonId = SOLVER_USE_2_FRICTION_DIRECTIONS;
		button.m_initialState = !!(gSolverMode & button.m_buttonId);
		button.m_callback = toggleSolverModeCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
	}
	{
		ButtonParams button("Solver friction dir caching", 0, true);
		button.m_buttonId = SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;
		button.m_initialState = !!(gSolverMode & button.m_buttonId);
		button.m_callback = toggleSolverModeCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
	}
	{
		ButtonParams button("Solver warmstarting", 0, true);
		button.m_buttonId = SOLVER_USE_WARMSTARTING;
		button.m_initialState = !!(gSolverMode & button.m_buttonId);
		button.m_callback = toggleSolverModeCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
	}
	if (m_multithreadedWorld)
	{
#if BT_THREADSAFE
		if (gTaskSchedulerMgr.getNumTaskSchedulers() >= 1)
		{
			// create a combo box for selecting the task scheduler
			const int maxNumTaskSchedulers = 20;
			static const char* sTaskSchedulerComboBoxItems[maxNumTaskSchedulers];
			int startingItem = 0;
			for (int i = 0; i < gTaskSchedulerMgr.getNumTaskSchedulers(); ++i)
			{
				sTaskSchedulerComboBoxItems[i] = gTaskSchedulerMgr.getTaskScheduler(i)->getName();
				if (gTaskSchedulerMgr.getTaskScheduler(i) == btGetTaskScheduler())
				{
					startingItem = i;
				}
			}
			ComboBoxParams comboParams;
			comboParams.m_userPointer = sTaskSchedulerComboBoxItems;
			comboParams.m_numItems = gTaskSchedulerMgr.getNumTaskSchedulers();
			comboParams.m_startItem = startingItem;
			comboParams.m_items = sTaskSchedulerComboBoxItems;
			comboParams.m_callback = setTaskSchedulerComboBoxCallback;
			m_guiHelper->getParameterInterface()->registerComboBox(comboParams);
		}
		{
			// if slider has not been set yet (by another demo),
			if (gSliderNumThreads <= 1.0f)
			{
				// create a slider to set the number of threads to use
				int numThreads = btGetTaskScheduler()->getNumThreads();
				gSliderNumThreads = float(numThreads);
			}
			int maxNumThreads = btGetTaskScheduler()->getMaxNumThreads();
			SliderParams slider("Thread count", &gSliderNumThreads);
			slider.m_minVal = 1.0f;
			slider.m_maxVal = float(maxNumThreads);
			slider.m_callback = setThreadCountCallback;
			slider.m_clampToIntegers = true;
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
		}
		{
			// a slider for the number of manifolds an island needs to be too large for parallel dispatch
			if (gSliderIslandBatchingThreshold < 1.0)
			{
				gSliderIslandBatchingThreshold = float(btSequentialImpulseConstraintSolverMt::s_minimumContactManifoldsForBatching);
			}
			SliderParams slider("IslandBatchThresh", &gSliderIslandBatchingThreshold);
			slider.m_minVal = 1.0f;
			slider.m_maxVal = 2000.0f;
			slider.m_callback = setLargeIslandManifoldCountCallback;
			slider.m_userPointer = NULL;
			slider.m_clampToIntegers = true;
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
		}
		{
			// create a combo box for selecting the batching method
			static const char* sBatchingMethodComboBoxItems[btBatchedConstraints::BATCHING_METHOD_COUNT];
			{
				sBatchingMethodComboBoxItems[btBatchedConstraints::BATCHING_METHOD_SPATIAL_GRID_2D] = "Batching: 2D Grid";
				sBatchingMethodComboBoxItems[btBatchedConstraints::BATCHING_METHOD_SPATIAL_GRID_3D] = "Batching: 3D Grid";
			};
			ComboBoxParams comboParams;
			comboParams.m_userPointer = sBatchingMethodComboBoxItems;
			comboParams.m_numItems = btBatchedConstraints::BATCHING_METHOD_COUNT;
			comboParams.m_startItem = static_cast<int>(btSequentialImpulseConstraintSolverMt::s_contactBatchingMethod);
			comboParams.m_items = sBatchingMethodComboBoxItems;
			comboParams.m_callback = setBatchingMethodComboBoxCallback;
			m_guiHelper->getParameterInterface()->registerComboBox(comboParams);
		}
		{
			// a slider for the sequentialImpulseConstraintSolverMt min batch size (when batching)
			SliderParams slider("Min batch size", &gSliderMinBatchSize);
			slider.m_minVal = 1.0f;
			slider.m_maxVal = 1000.0f;
			slider.m_callback = setMinBatchSizeCallback;
			slider.m_userPointer = NULL;
			slider.m_clampToIntegers = true;
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
		}
		{
			// a slider for the sequentialImpulseConstraintSolverMt max batch size (when batching)
			SliderParams slider("Max batch size", &gSliderMaxBatchSize);
			slider.m_minVal = 1.0f;
			slider.m_maxVal = 1000.0f;
			slider.m_callback = setMaxBatchSizeCallback;
			slider.m_userPointer = NULL;
			slider.m_clampToIntegers = true;
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
		}
		{
			// create a button to toggle debug drawing of batching visualization
			ButtonParams button("Visualize batching", 0, true);
			bool* ptr = &btBatchedConstraints::s_debugDrawBatches;
			button.m_initialState = *ptr;
			button.m_userPointer = ptr;
			button.m_callback = boolPtrButtonCallback;
			m_guiHelper->getParameterInterface()->registerButtonParameter(button);
		}
		{
			ButtonParams button("Allow Nested ParallelFor", 0, true);
			button.m_initialState = btSequentialImpulseConstraintSolverMt::s_allowNestedParallelForLoops;
			button.m_userPointer = &btSequentialImpulseConstraintSolverMt::s_allowNestedParallelForLoops;
			button.m_callback = boolPtrButtonCallback;
			m_guiHelper->getParameterInterface()->registerButtonParameter(button);
		}
#endif  // #if BT_THREADSAFE
	}
}

void CommonRigidBodyMTBase::drawScreenText()
{
	char msg[1024];
	int xCoord = 400;
	int yCoord = 30;
	int yStep = 30;
	int indent = 30;
	if (m_solverType != gSolverType)
	{
		sprintf(msg, "restart example to change solver type");
		m_guiHelper->getAppInterface()->drawText(msg, 300, yCoord, 0.4f);
		yCoord += yStep;
	}
	if (m_multithreadCapable)
	{
		if (m_multithreadedWorld != gMultithreadedWorld)
		{
			sprintf(msg, "restart example to begin in %s mode",
					gMultithreadedWorld ? "multithreaded" : "single threaded");
			m_guiHelper->getAppInterface()->drawText(msg, 300, yCoord, 0.4f);
			yCoord += yStep;
		}
	}
	if (gDisplayProfileInfo)
	{
		if (m_multithreadedWorld)
		{
#if BT_THREADSAFE
			int numManifolds = m_dispatcher->getNumManifolds();
			int numContacts = 0;
			for (int i = 0; i < numManifolds; ++i)
			{
				const btPersistentManifold* man = m_dispatcher->getManifoldByIndexInternal(i);
				numContacts += man->getNumContacts();
			}
			const char* mtApi = btGetTaskScheduler()->getName();
			sprintf(msg, "islands=%d bodies=%d manifolds=%d contacts=%d [%s] threads=%d",
					gNumIslands,
					m_dynamicsWorld->getNumCollisionObjects(),
					numManifolds,
					numContacts,
					mtApi,
					btGetTaskScheduler()->getNumThreads());
			m_guiHelper->getAppInterface()->drawText(msg, 100, yCoord, 0.4f);
			yCoord += yStep;
#endif  // #if BT_THREADSAFE
		}
		{
			int sm = gSolverMode;
			sprintf(msg, "solver %s mode [%s%s%s%s%s%s]",
					getSolverTypeName(m_solverType),
					sm & SOLVER_SIMD ? "SIMD" : "",
					sm & SOLVER_RANDMIZE_ORDER ? " randomize" : "",
					sm & SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS ? " interleave" : "",
					sm & SOLVER_USE_2_FRICTION_DIRECTIONS ? " friction2x" : "",
					sm & SOLVER_ENABLE_FRICTION_DIRECTION_CACHING ? " frictionDirCaching" : "",
					sm & SOLVER_USE_WARMSTARTING ? " warm" : "");
			m_guiHelper->getAppInterface()->drawText(msg, xCoord, yCoord, 0.4f);
			yCoord += yStep;
		}
		sprintf(msg, "internalSimStep %5.3f ms",
				gProfiler.getAverageTime(Profiler::kRecordInternalTimeStep) * 0.001f);
		m_guiHelper->getAppInterface()->drawText(msg, xCoord, yCoord, 0.4f);
		yCoord += yStep;

		if (m_multithreadedWorld)
		{
			sprintf(msg,
					"DispatchCollisionPairs %5.3f ms",
					gProfiler.getAverageTime(Profiler::kRecordDispatchAllCollisionPairs) * 0.001f);
			m_guiHelper->getAppInterface()->drawText(msg, xCoord, yCoord, 0.4f);
			yCoord += yStep;

			sprintf(msg,
					"SolveAllIslands %5.3f ms",
					gProfiler.getAverageTime(Profiler::kRecordDispatchIslands) * 0.001f);
			m_guiHelper->getAppInterface()->drawText(msg, xCoord, yCoord, 0.4f);
			yCoord += yStep;

			sprintf(msg,
					"SolverTotal %5.3f ms",
					gProfiler.getAverageTime(Profiler::kRecordSolverTotal) * 0.001f);
			m_guiHelper->getAppInterface()->drawText(msg, xCoord, yCoord, 0.4f);
			yCoord += yStep;

			sprintf(msg,
					"SolverSetup %5.3f ms",
					gProfiler.getAverageTime(Profiler::kRecordSolverSetup) * 0.001f);
			m_guiHelper->getAppInterface()->drawText(msg, xCoord + indent, yCoord, 0.4f);
			yCoord += yStep;

			sprintf(msg,
					"SolverIterations %5.3f ms",
					gProfiler.getAverageTime(Profiler::kRecordSolverIterations) * 0.001f);
			m_guiHelper->getAppInterface()->drawText(msg, xCoord + indent, yCoord, 0.4f);
			yCoord += yStep;

			sprintf(msg,
					"SolverFinish %5.3f ms",
					gProfiler.getAverageTime(Profiler::kRecordSolverFinish) * 0.001f);
			m_guiHelper->getAppInterface()->drawText(msg, xCoord + indent, yCoord, 0.4f);
			yCoord += yStep;

			sprintf(msg,
					"PredictUnconstrainedMotion %5.3f ms",
					gProfiler.getAverageTime(Profiler::kRecordPredictUnconstrainedMotion) * 0.001f);
			m_guiHelper->getAppInterface()->drawText(msg, xCoord, yCoord, 0.4f);
			yCoord += yStep;

			sprintf(msg,
					"CreatePredictiveContacts %5.3f ms",
					gProfiler.getAverageTime(Profiler::kRecordCreatePredictiveContacts) * 0.001f);
			m_guiHelper->getAppInterface()->drawText(msg, xCoord, yCoord, 0.4f);
			yCoord += yStep;

			sprintf(msg,
					"IntegrateTransforms %5.3f ms",
					gProfiler.getAverageTime(Profiler::kRecordIntegrateTransforms) * 0.001f);
			m_guiHelper->getAppInterface()->drawText(msg, xCoord, yCoord, 0.4f);
			yCoord += yStep;
			(void)yCoord;
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
