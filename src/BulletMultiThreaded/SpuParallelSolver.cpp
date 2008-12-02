/*
Bullet Continuous Collision Detection and Physics Library - Parallel solver
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#include "SpuParallelSolver.h"

//#include "SpuFakeDma.h"
#include "SpuSync.h"

#include "LinearMath/btVector3.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "LinearMath/btMinMax.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "LinearMath/btQuickprof.h"

#include "SpuSolverTask/SpuParallellSolverTask.h"

#include <stdio.h>

enum
{
	PARALLEL_SOLVER_BODIES_PER_TASK = 64,
	PARALLEL_SOLVER_CELLS_PER_TASK = SPU_HASH_NUMCELLS >> 3
};


//-- Hash handling
static void recordDependency(SpuSolverHash* hash, unsigned int i, unsigned int j)
{
	hash->m_dependencyMatrix[i][j >> 5] |= (1 << (j & 31));
	hash->m_dependencyMatrix[j][i >> 5] |= (1 << (i & 31));
}


// Clear the given hash
static void clearHash (SpuSolverHash* hash)
{
	size_t hashSize = sizeof(SpuSolverHash);
	memset(hash, 0, hashSize);
	int i;

	// Setup basic dependency
	for ( i = 0; i < SPU_HASH_NUMCELLS; ++i)
	{
		hash->m_dependencyMatrix[i][i >> 5] |= (1 << (i & 31));
	}

	// Set some ones to "unused cells"
	for ( i = SPU_HASH_WORDWIDTH-SPU_HASH_NUMUNUSEDBITS; i < SPU_HASH_WORDWIDTH; ++i)
	{
		hash->m_currentMask[0][SPU_HASH_NUMCELLDWORDS-1] |= (1 << i);
	}
}
/*
static bool getDependency(SpuSolverHash* hash, unsigned int i, unsigned int j)
{
	return (hash->m_dependencyMatrix[i][j >> 5] & (1 << (j & 31))) != 0;
}
*/


static unsigned int getObjectIndex (btCollisionObject* object)
{
	btVector3 center = object->getWorldTransform().getOrigin();
	int cx = (int)floorf(center.x() / SPU_HASH_PHYSSIZE);
	int cy = (int)floorf(center.y() / SPU_HASH_PHYSSIZE);
	int cz = (int)floorf(center.z() / SPU_HASH_PHYSSIZE);

	return spuGetHashCellIndex(cx, cy, cz);
}





btParallelSequentialImpulseSolver::btParallelSequentialImpulseSolver (btThreadSupportInterface* threadIf, int maxOutstandingTasks)
: m_numberOfContacts(0), m_taskScheduler (threadIf, maxOutstandingTasks)
{
	m_solverHash = new SpuSolverHash;
	clearHash(m_solverHash);
}

btParallelSequentialImpulseSolver::~btParallelSequentialImpulseSolver ()
{
	delete m_solverHash;
}


void btParallelSequentialImpulseSolver::prepareSolve(int numBodies, int numManifolds)
{
	m_sortedManifolds.reserve(numManifolds);
	m_allObjects.reserve(numBodies);
}

btScalar btParallelSequentialImpulseSolver::solveGroup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifold,int numManifolds,btTypedConstraint** constraints,int numConstraints, const btContactSolverInfo& info,class btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc,btDispatcher* dispatcher)
{
	BT_PROFILE("parallel_solveGroup");

	if (!numManifolds && !numConstraints)
		return 0;
	int i;

///refresh contact points is not needed anymore, it has been moved into the processCollision detection part.
#ifdef FORCE_REFESH_CONTACT_MANIFOLDS
	for ( i = 0; i < numManifolds; ++i)
	{
		btPersistentManifold* currManifold = manifold[i];
		btRigidBody* rb0 = (btRigidBody*)currManifold->getBody0();
		btRigidBody* rb1 = (btRigidBody*)currManifold->getBody1();

		currManifold->refreshContactPoints(rb0->getCenterOfMassTransform(),rb1->getCenterOfMassTransform());
	}
#endif //FORCE_REFESH_CONTACT_MANIFOLDS

	// Record and mark the manifolds to the cells
	for ( i = 0; i < numManifolds; ++i)
	{
		// Compute a hash cell for this manifold
		btPersistentManifold* currManifold = manifold[i];

		btCollisionObject *ownerObject, *otherObject;

		btRigidBody* rb0 = (btRigidBody*)currManifold->getBody0();
		btRigidBody* rb1 = (btRigidBody*)currManifold->getBody1();

		if (rb0->getIslandTag() >= 0)
		{
			ownerObject = rb0;
			otherObject = rb1;
		}
		else
		{
			ownerObject = rb1;
			otherObject = rb0;
		}

		// Save the cell
		unsigned int ownerCellIdx = getObjectIndex(ownerObject);
		ManifoldCellHolder holder = {ownerCellIdx, currManifold};
		m_sortedManifolds.push_back(holder);
		m_solverHash->m_Hash[ownerCellIdx].m_numManifolds++;

		// Record dependency
		if (rb0->getIslandTag() >= 0 && rb1->getIslandTag() >= 0)
		{
			unsigned int otherCellIdx = getObjectIndex(otherObject);
			recordDependency(m_solverHash, ownerCellIdx, otherCellIdx);
		}
		
		// Save statistics
		int numContacts = currManifold->getNumContacts();
		m_solverHash->m_Hash[ownerCellIdx].m_numContacts += numContacts;
		m_numberOfContacts += numContacts;
	}

	// Record and mark constraints to the cells
	for ( i = 0; i < numConstraints; ++i)
	{
		// Compute a hash cell for this manifold
		btTypedConstraint* currConstraint = constraints[i];

		if (!constraintTypeSupported(currConstraint->getConstraintType()))
			continue;

		btCollisionObject *ownerObject, *otherObject;

		btRigidBody* rb0 = &currConstraint->getRigidBodyA();
		btRigidBody* rb1 = &currConstraint->getRigidBodyB();

		if (rb0->getIslandTag() >= 0)
		{
			ownerObject = rb0;
			otherObject = rb1;
		}
		else
		{
			ownerObject = rb1;
			otherObject = rb0;
		}

		// Save the cell
		unsigned int ownerCellIdx = getObjectIndex(ownerObject);
		ConstraintCellHolder holder = {ownerCellIdx, currConstraint->getConstraintType(), currConstraint};
		m_sortedConstraints.push_back(holder);
		m_solverHash->m_Hash[ownerCellIdx].m_numConstraints++;

		// Record dependency
		if (rb0 && rb1 && rb0->getIslandTag() >= 0 && rb1->getIslandTag() >= 0)
		{
			unsigned int otherCellIdx = getObjectIndex(otherObject);
			recordDependency(m_solverHash, ownerCellIdx, otherCellIdx);
		}
	}

	// Save all RBs
	for ( i = 0; i < numBodies; ++i)
	{
		btCollisionObject* obj = bodies[i];
		//unsigned int cellIdx = getObjectIndex(obj);

		btRigidBody* rb = btRigidBody::upcast(obj);
		m_allObjects.push_back(rb);
	}

	return 0;
}

template<typename T>
class CellHolderPredicate
{
public:
	SIMD_FORCE_INLINE bool operator() ( const T& lhs, const T& rhs )
	{
		return lhs.m_hashCellIndex < rhs.m_hashCellIndex;
	}
};


/*static void printDependencyMatrix(SpuSolverHash* hash)
{
	for (int r = 0; r < SPU_HASH_NUMCELLS; ++r)
	{
		for (int c = 0; c < SPU_HASH_NUMCELLS; ++c)
		{
			if (getDependency(hash, r, c))
			{
				printf("1");
			}
			else
			{
				printf("0");
			}
		}

		printf("\n");
	}
	printf("\n");
	fflush(stdout);
}
*/

// Solver caches
btAlignedObjectArray<btSolverBody> solverBodyPool_persist;
btAlignedObjectArray<uint32_t> solverBodyOffsetList_persist;
btAlignedObjectArray<btSolverConstraint> solverInternalConstraintPool_persist;
btAlignedObjectArray<btSolverConstraint> solverConstraintPool_persist;


void btParallelSequentialImpulseSolver::allSolved (const btContactSolverInfo& info,class btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc)
{
	BT_PROFILE("parallel_allSolved");

	if (!m_numberOfContacts && !m_sortedConstraints.size())
	{
		m_sortedManifolds.clear();
		m_sortedConstraints.clear();
		m_allObjects.clear();
		clearHash(m_solverHash);
		return;
	}


	//printDependencyMatrix(m_solverHash);

	// Sort the manifolds list
	int numManifolds = m_sortedManifolds.size();
	m_sortedManifolds.quickSort(CellHolderPredicate<ManifoldCellHolder>());

	// Sort the constraint list
	int numConstraints = m_sortedConstraints.size();
	m_sortedConstraints.quickSort(CellHolderPredicate<ConstraintCellHolder>());


	// Sort the body list
	int numBodies = m_allObjects.size();
	
	// Reassign the hash offset
	uint32_t emptyCellMask[SPU_HASH_NUMCELLDWORDS] = {0};
	int numBodyOffsets = 0;
	{
		int manifoldRunner = 0;
		int bodyOffsetRunner = 0;
		int internalConstraintRunner = 0;
		int constraintRunner = 0;
		
		for (int i = 0; i < SPU_HASH_NUMCELLS; ++i)
		{
			bool empty = true;

			SpuSolverHashCell& hashCell = m_solverHash->m_Hash[i];
			hashCell.m_solverBodyOffsetListOffset = bodyOffsetRunner;

			if (hashCell.m_numManifolds)
			{
				hashCell.m_manifoldListOffset = manifoldRunner;
				manifoldRunner += hashCell.m_numManifolds;
				
				bodyOffsetRunner += hashCell.m_numManifolds*2;
			}			
			if (hashCell.m_numContacts)
			{
				hashCell.m_internalConstraintListOffset = internalConstraintRunner*3;
				internalConstraintRunner += hashCell.m_numContacts;
				empty = false;
			}

			if (hashCell.m_numConstraints)
			{
				hashCell.m_constraintListOffset = constraintRunner;
				constraintRunner += hashCell.m_numConstraints;

				bodyOffsetRunner += hashCell.m_numConstraints*2;

				empty = false;
			}
			

			emptyCellMask[i >> 5] |= (empty ? (1 << (i&31)) : 0);
			// Align the bodyOffsetRunner to a whole number of 4 for right alignment in the list
			bodyOffsetRunner = (bodyOffsetRunner+3)&~0x3;
		}

		numBodyOffsets = bodyOffsetRunner;
	}

	// Setup rigid bodies
	// Allocate temporary data
	solverBodyPool_persist.resize(numBodies + numManifolds + numConstraints);
	btSolverBody* solverBodyPool = &solverBodyPool_persist[0];

	solverBodyOffsetList_persist.resize(numBodyOffsets);
	uint32_t* solverBodyOffsetList = &solverBodyOffsetList_persist[0];

	solverInternalConstraintPool_persist.resize(m_numberOfContacts*3);
	btSolverConstraint* solverInternalConstraintPool = &solverInternalConstraintPool_persist[0];
	
	solverConstraintPool_persist.resize(numConstraints);
	btSolverConstraint* solverConstraintPool = &solverConstraintPool_persist[0];

	// Setup all the moving rigid bodies
	{
		BT_PROFILE("setup moving rigidbodies");

		int bodiesPerTask = PARALLEL_SOLVER_BODIES_PER_TASK;
		int bodiesToSchedule = numBodies;
		int startBody = 0;

		while (bodiesToSchedule > 0)
		{
			// Schedule a bunch of hash cells
			int numBodiesInTask = bodiesToSchedule > bodiesPerTask ? bodiesPerTask : bodiesToSchedule;

			SpuSolverTaskDesc* desc = m_taskScheduler.getTask();

			desc->m_solverCommand = CMD_SOLVER_SETUP_BODIES;
			desc->m_solverData.m_solverHash = m_solverHash;
			desc->m_solverData.m_solverBodyList = solverBodyPool;

			desc->m_commandData.m_bodySetup.m_startBody = startBody;
			desc->m_commandData.m_bodySetup.m_numBodies = numBodiesInTask;
			desc->m_commandData.m_bodySetup.m_rbList = &m_allObjects[0];

			m_taskScheduler.issueTask();
			bodiesToSchedule -= numBodiesInTask;
			startBody += numBodiesInTask;
		}
		
		m_taskScheduler.flushTasks();
	}

	// Manifold setup
	{
		int cellsPerTask = PARALLEL_SOLVER_CELLS_PER_TASK;
		int cellsToSchedule = SPU_HASH_NUMCELLS;
		int startCell = 0;

		while (cellsToSchedule > 0)
		{
			int numCellsInTask = cellsToSchedule > cellsPerTask ? cellsPerTask : cellsToSchedule;
			
			SpuSolverTaskDesc* desc = m_taskScheduler.getTask();

			desc->m_solverCommand = CMD_SOLVER_MANIFOLD_SETUP;
			desc->m_solverData.m_solverHash = m_solverHash;
			desc->m_solverData.m_solverBodyList = solverBodyPool;
			desc->m_solverData.m_solverBodyOffsetList = solverBodyOffsetList;
			desc->m_solverData.m_solverInternalConstraintList = solverInternalConstraintPool;
			desc->m_solverData.m_solverConstraintList = solverConstraintPool;

			desc->m_commandData.m_manifoldSetup.m_startCell = startCell;
			desc->m_commandData.m_manifoldSetup.m_numCells = numCellsInTask;
			desc->m_commandData.m_manifoldSetup.m_numBodies = numBodies;
			desc->m_commandData.m_manifoldSetup.m_numManifolds = numManifolds;
			desc->m_commandData.m_manifoldSetup.m_manifoldHolders = &m_sortedManifolds[0];
			desc->m_commandData.m_manifoldSetup.m_constraintHolders = &m_sortedConstraints[0];
			desc->m_commandData.m_manifoldSetup.m_solverInfo = info;

			m_taskScheduler.issueTask();
			cellsToSchedule -= numCellsInTask;
			startCell += numCellsInTask;
		}
		m_taskScheduler.flushTasks();
	}

	{
		BT_PROFILE("parallel_solve_iterations");

		btSpinlock::SpinVariable* spinVar = (btSpinlock::SpinVariable*)btAlignedAlloc(sizeof(btSpinlock::SpinVariable), 128);
		for (int iter = 0; iter < info.m_numIterations; ++iter)
		{
			btSpinlock lock (spinVar);
			lock.Init();

			// Clear the "processed cells" part of the hash
			memcpy(m_solverHash->m_currentMask[0], emptyCellMask, sizeof(uint32_t)*SPU_HASH_NUMCELLDWORDS);

			for (int task = 0; task < m_taskScheduler.getMaxOutstandingTasks(); ++task)
			{
				SpuSolverTaskDesc* desc = m_taskScheduler.getTask();
				desc->m_solverCommand = CMD_SOLVER_SOLVE_ITERATE;

				desc->m_solverData.m_solverHash = m_solverHash;
				desc->m_solverData.m_solverBodyList = solverBodyPool;
				desc->m_solverData.m_solverBodyOffsetList = solverBodyOffsetList;
				desc->m_solverData.m_solverInternalConstraintList = solverInternalConstraintPool;
				desc->m_solverData.m_solverConstraintList = solverConstraintPool;

				desc->m_commandData.m_iterate.m_spinLockVar = spinVar;

				m_taskScheduler.issueTask();
			} 
			m_taskScheduler.flushTasks();


		}
		btAlignedFree((void*)spinVar);
	}
	
	// Write back velocity
	{
		int bodiesPerTask = PARALLEL_SOLVER_BODIES_PER_TASK;
		int bodiesToSchedule = numBodies;
		int startBody = 0;

		while (bodiesToSchedule > 0)
		{
			// Schedule a bunch of hash cells
			int numBodiesInTask = bodiesToSchedule > bodiesPerTask ? bodiesPerTask : bodiesToSchedule;

			SpuSolverTaskDesc* desc = m_taskScheduler.getTask();

			desc->m_solverCommand = CMD_SOLVER_COPYBACK_BODIES;
			desc->m_solverData.m_solverHash = m_solverHash;
			desc->m_solverData.m_solverBodyList = solverBodyPool;

			desc->m_commandData.m_bodyCopyback.m_startBody = startBody;
			desc->m_commandData.m_bodyCopyback.m_numBodies = numBodiesInTask;
			desc->m_commandData.m_bodyCopyback.m_rbList = &m_allObjects[0];

			m_taskScheduler.issueTask();
			bodiesToSchedule -= numBodiesInTask;
			startBody += numBodiesInTask;
		}

		m_taskScheduler.flushTasks();
	}




	{
		BT_PROFILE("warmstart_writeback");

		btSpinlock::SpinVariable* spinVar = (btSpinlock::SpinVariable*)btAlignedAlloc(sizeof(btSpinlock::SpinVariable), 128);
		for (int iter = 0; iter < info.m_numIterations; ++iter)
		{
			btSpinlock lock (spinVar);
			lock.Init();

			// Clear the "processed cells" part of the hash
			memcpy(m_solverHash->m_currentMask[0], emptyCellMask, sizeof(uint32_t)*SPU_HASH_NUMCELLDWORDS);

			for (int task = 0; task < m_taskScheduler.getMaxOutstandingTasks(); ++task)
			{
				SpuSolverTaskDesc* desc = m_taskScheduler.getTask();
				desc->m_solverCommand = CMD_SOLVER_MANIFOLD_WARMSTART_WRITEBACK;
				desc->m_solverData.m_solverHash = m_solverHash;
				desc->m_solverData.m_solverInternalConstraintList = solverInternalConstraintPool;
				desc->m_solverData.m_solverConstraintList = solverConstraintPool;
				desc->m_commandData.m_manifoldSetup.m_manifoldHolders = &m_sortedManifolds[0];
				desc->m_commandData.m_iterate.m_spinLockVar = spinVar;

				m_taskScheduler.issueTask();
			} 
			m_taskScheduler.flushTasks();		
		}
		btAlignedFree((void*)spinVar);
	}

	



	// Clean up
	m_sortedManifolds.resize(0);
	m_sortedConstraints.resize(0);
	m_allObjects.resize(0);
	clearHash(m_solverHash);


	m_numberOfContacts = 0;
}

void btParallelSequentialImpulseSolver::reset()
{
	m_sortedManifolds.clear();
	m_allObjects.clear();
	m_numberOfContacts = 0;
	clearHash(m_solverHash);

	solverBodyPool_persist.clear();
	solverBodyOffsetList_persist.clear();
	solverConstraintPool_persist.clear();
	solverInternalConstraintPool_persist.clear();
}


SolverTaskScheduler::SolverTaskScheduler(btThreadSupportInterface* threadIf, int maxOutstandingTasks)
: m_threadInterface (threadIf), m_maxNumOutstandingTasks (maxOutstandingTasks > SPU_MAX_SPUS ? SPU_MAX_SPUS : maxOutstandingTasks), 
m_currentTask (0), m_numBusyTasks (0)
{
	m_taskDescriptors.resize(m_maxNumOutstandingTasks);
	m_taskBusy.resize(m_maxNumOutstandingTasks);

	m_threadInterface->startSPU();
}


SolverTaskScheduler::~SolverTaskScheduler()
{
	m_threadInterface->stopSPU();
}

SpuSolverTaskDesc* SolverTaskScheduler::getTask()
{
	int taskIdx = -1;

	if (m_taskBusy[m_currentTask])
	{
		//try to find a new one
		for (int i = 0; i < m_maxNumOutstandingTasks; ++i)
		{
			if (!m_taskBusy[i])
			{
				taskIdx = i;
				break;
			}
		}

		if (taskIdx < 0)
		{
			// Have to wait
			unsigned int taskId;
			unsigned int outputSize;

			for (int i=0;i<m_maxNumOutstandingTasks;i++)
			  {
				  if (m_taskBusy[i])
				  {
					  taskId = i;
					  break;
				  }
			  }

			m_threadInterface->waitForResponse(&taskId, &outputSize);

			m_taskBusy[taskId] = false;
			m_numBusyTasks--;

			taskIdx = taskId;
		}

		m_currentTask = taskIdx;
	}


	SpuSolverTaskDesc* result = &m_taskDescriptors[m_currentTask];
	int so = sizeof(SpuSolverTaskDesc);

	memset(result, 0, so);
	result->m_taskId = m_currentTask;

	return result;
}

void SolverTaskScheduler::issueTask()
{
	m_taskBusy[m_currentTask] = true;
	m_numBusyTasks++;

	SpuSolverTaskDesc& desc = m_taskDescriptors[m_currentTask];
	
	m_threadInterface->sendRequest(1, (ppu_address_t)&desc, m_currentTask);
}

void SolverTaskScheduler::flushTasks()
{
	while (m_numBusyTasks > 0)
	{
		unsigned int taskId;
		unsigned int outputSize;
		for (int i=0;i<m_maxNumOutstandingTasks;i++)
	  {
		  if (m_taskBusy[i])
		  {
			  taskId = i;
			  break;
		  }
	  }

		m_threadInterface->waitForResponse(&taskId, &outputSize);

		m_taskBusy[taskId] = false;
		m_numBusyTasks--;
	}
}