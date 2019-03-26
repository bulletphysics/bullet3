#include "btBlockSolver.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"

#include "LinearMath/btQuickprof.h"

void setupHelper(btSISolverSingleIterationData& siData,
				 btCollisionObject** bodies, int numBodies,
				 const btContactSolverInfo& info,
				 btTypedConstraint** constraintStart, int constrainNums,
				 btPersistentManifold** manifoldPtr, int numManifolds);

struct btBlockSolverInternalData
{
	btAlignedObjectArray<btSolverBody> m_tmpSolverBodyPool;
	btConstraintArray m_tmpSolverContactConstraintPool;
	btConstraintArray m_tmpSolverNonContactConstraintPool;
	btConstraintArray m_tmpSolverContactFrictionConstraintPool;
	btConstraintArray m_tmpSolverContactRollingFrictionConstraintPool;

	btAlignedObjectArray<int> m_orderTmpConstraintPool;
	btAlignedObjectArray<int> m_orderNonContactConstraintPool;
	btAlignedObjectArray<int> m_orderFrictionConstraintPool;
	btAlignedObjectArray<btTypedConstraint::btConstraintInfo1>
		m_tmpConstraintSizesPool;

	unsigned long m_btSeed2;
	int m_fixedBodyId;
	int m_maxOverrideNumSolverIterations;
	btAlignedObjectArray<int>
		m_kinematicBodyUniqueIdToSolverBodyTable;  // only used for multithreading

	btSingleConstraintRowSolver m_resolveSingleConstraintRowGeneric;
	btSingleConstraintRowSolver m_resolveSingleConstraintRowLowerLimit;
	btSingleConstraintRowSolver m_resolveSplitPenetrationImpulse;

	btBlockSolverInternalData()
		: m_btSeed2(0),
		  m_fixedBodyId(-1),
		  m_maxOverrideNumSolverIterations(0),
		  m_resolveSingleConstraintRowGeneric(
			  btSequentialImpulseConstraintSolver::
				  getScalarConstraintRowSolverGeneric()),
		  m_resolveSingleConstraintRowLowerLimit(
			  btSequentialImpulseConstraintSolver::
				  getScalarConstraintRowSolverLowerLimit()),
		  m_resolveSplitPenetrationImpulse(
			  btSequentialImpulseConstraintSolver::
				  getScalarSplitPenetrationImpulseGeneric()) {}
};

btBlockSolver::btBlockSolver()
{
	m_data21 = new btBlockSolverInternalData;
	m_data22 = new btBlockSolverInternalData;
}

btBlockSolver::~btBlockSolver()
{
	delete m_data21;
	delete m_data22;
}

btScalar btBlockSolver::solveGroupInternalBlock(
	btCollisionObject** bodies, int numBodies,
	btPersistentManifold** manifoldPtr, int numManifolds,
	btTypedConstraint** constraints, int numConstraints,
	const btContactSolverInfo& info, btIDebugDraw* debugDrawer,
	btDispatcher* dispatcher)
{
	// initialize data for two children solvers
	btSISolverSingleIterationData siData1(
		m_data21->m_tmpSolverBodyPool, m_data21->m_tmpSolverContactConstraintPool,
		m_data21->m_tmpSolverNonContactConstraintPool,
		m_data21->m_tmpSolverContactFrictionConstraintPool,
		m_data21->m_tmpSolverContactRollingFrictionConstraintPool,
		m_data21->m_orderTmpConstraintPool,
		m_data21->m_orderNonContactConstraintPool,
		m_data21->m_orderFrictionConstraintPool,
		m_data21->m_tmpConstraintSizesPool,
		m_data21->m_resolveSingleConstraintRowGeneric,
		m_data21->m_resolveSingleConstraintRowLowerLimit,
		m_data21->m_resolveSplitPenetrationImpulse,
		m_data21->m_kinematicBodyUniqueIdToSolverBodyTable, m_data21->m_btSeed2,
		m_data21->m_fixedBodyId, m_data21->m_maxOverrideNumSolverIterations);

	btSISolverSingleIterationData siData2(
		m_data22->m_tmpSolverBodyPool, m_data22->m_tmpSolverContactConstraintPool,
		m_data22->m_tmpSolverNonContactConstraintPool,
		m_data22->m_tmpSolverContactFrictionConstraintPool,
		m_data22->m_tmpSolverContactRollingFrictionConstraintPool,
		m_data22->m_orderTmpConstraintPool,
		m_data22->m_orderNonContactConstraintPool,
		m_data22->m_orderFrictionConstraintPool,
		m_data22->m_tmpConstraintSizesPool,
		m_data22->m_resolveSingleConstraintRowGeneric,
		m_data22->m_resolveSingleConstraintRowLowerLimit,
		m_data22->m_resolveSplitPenetrationImpulse,
		m_data22->m_kinematicBodyUniqueIdToSolverBodyTable, m_data22->m_btSeed2,
		m_data22->m_fixedBodyId, m_data22->m_maxOverrideNumSolverIterations);

	m_data21->m_fixedBodyId = -1;
	m_data22->m_fixedBodyId = -1;

	// set up
	int halfNumConstraints1 = numConstraints / 2;
	int halfNumConstraints2 = numConstraints - halfNumConstraints1;

	int halfNumManifolds1 = numConstraints / 2;
	int halfNumManifolds2 = numManifolds - halfNumManifolds1;

	setupHelper(siData1, bodies, numBodies, info, constraints,
				halfNumConstraints1, manifoldPtr, halfNumManifolds1);

	setupHelper(siData2, bodies, numBodies, info,
				constraints + halfNumConstraints1, halfNumConstraints2,
				manifoldPtr + halfNumManifolds1, halfNumManifolds2);
	// set up complete

	// begin solve
	btScalar leastSquaresResidual = 0;
	{
		BT_PROFILE("solveGroupCacheFriendlyIterations");
		/// this is a special step to resolve penetrations (just for contacts)
		btSequentialImpulseConstraintSolver::
			solveGroupCacheFriendlySplitImpulseIterationsInternal(
				siData1, bodies, numBodies, manifoldPtr, halfNumManifolds1,
				constraints, halfNumConstraints1, info, debugDrawer);

		btSequentialImpulseConstraintSolver::
			solveGroupCacheFriendlySplitImpulseIterationsInternal(
				siData2, bodies, numBodies, manifoldPtr + halfNumManifolds1,
				halfNumManifolds2, constraints + halfNumConstraints1,
				halfNumConstraints2, info, debugDrawer);

		int maxIterations =
			siData1.m_maxOverrideNumSolverIterations > info.m_numIterations
				? siData1.m_maxOverrideNumSolverIterations
				: info.m_numIterations;

		for (int iteration = 0; iteration < maxIterations; iteration++)
		{
			btScalar res1 =
				btSequentialImpulseConstraintSolver::solveSingleIterationInternal(
					siData1, iteration, constraints, halfNumConstraints1, info);

			btScalar res2 =
				btSequentialImpulseConstraintSolver::solveSingleIterationInternal(
					siData2, iteration, constraints + halfNumConstraints1,
					halfNumConstraints2, info);
			leastSquaresResidual = btMax(res1, res2);

			if (leastSquaresResidual <= info.m_leastSquaresResidualThreshold ||
				(iteration >= (maxIterations - 1)))
			{
#ifdef VERBOSE_RESIDUAL_PRINTF
				printf("residual = %f at iteration #%d\n", m_leastSquaresResidual,
					   iteration);
#endif
				break;
			}
		}
	}

	btScalar res = btSequentialImpulseConstraintSolver::
		solveGroupCacheFriendlyFinishInternal(siData1, bodies, numBodies, info);
	+btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyFinishInternal(
		siData2, bodies, numBodies, info);

	return res;
}

void setupHelper(btSISolverSingleIterationData& siData,
				 btCollisionObject** bodies, int numBodies,
				 const btContactSolverInfo& info,
				 btTypedConstraint** constraintStart, int constrainNums,
				 btPersistentManifold** manifoldPtr, int numManifolds)
{
	btSequentialImpulseConstraintSolver::convertBodiesInternal(siData, bodies,
															   numBodies, info);
	btSequentialImpulseConstraintSolver::convertJointsInternal(
		siData, constraintStart, constrainNums, info);

	int i;
	btPersistentManifold* manifold = 0;

	for (i = 0; i < numManifolds; i++)
	{
		manifold = manifoldPtr[i];
		btSequentialImpulseConstraintSolver::convertContactInternal(siData,
																	manifold, info);

		int numNonContactPool = siData.m_tmpSolverNonContactConstraintPool.size();
		int numConstraintPool = siData.m_tmpSolverContactConstraintPool.size();
		int numFrictionPool =
			siData.m_tmpSolverContactFrictionConstraintPool.size();

		siData.m_orderNonContactConstraintPool.resizeNoInitialize(
			numNonContactPool);
		if ((info.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
			siData.m_orderTmpConstraintPool.resizeNoInitialize(numConstraintPool * 2);
		else
			siData.m_orderTmpConstraintPool.resizeNoInitialize(numConstraintPool);

		siData.m_orderFrictionConstraintPool.resizeNoInitialize(numFrictionPool);
		{
			int i;
			for (i = 0; i < numNonContactPool; i++)
			{
				siData.m_orderNonContactConstraintPool[i] = i;
			}
			for (i = 0; i < numConstraintPool; i++)
			{
				siData.m_orderTmpConstraintPool[i] = i;
			}
			for (i = 0; i < numFrictionPool; i++)
			{
				siData.m_orderFrictionConstraintPool[i] = i;
			}
		}
	}
}

btScalar btBlockSolver::solveGroup(btCollisionObject** bodies, int numBodies,
								   btPersistentManifold** manifoldPtr,
								   int numManifolds,
								   btTypedConstraint** constraints,
								   int numConstraints,
								   const btContactSolverInfo& info,
								   btIDebugDraw* debugDrawer,
								   btDispatcher* dispatcher)
{
	// if (m_childSolvers.size())
	// hard code to use block solver for now
	return solveGroupInternalBlock(bodies, numBodies, manifoldPtr, numManifolds,
								   constraints, numConstraints, info, debugDrawer,
								   dispatcher);
	//  else
	//    return solveGroupInternal(bodies, numBodies, manifoldPtr, numManifolds,
	//                             constraints, numConstraints, info, debugDrawer,
	//                             dispatcher);
}

btScalar btBlockSolver::solveGroupInternal(
	btCollisionObject** bodies, int numBodies,
	btPersistentManifold** manifoldPtr, int numManifolds,
	btTypedConstraint** constraints, int numConstraints,
	const btContactSolverInfo& info, btIDebugDraw* debugDrawer,
	btDispatcher* dispatcher)
{
	btSISolverSingleIterationData siData(
		m_data21->m_tmpSolverBodyPool, m_data21->m_tmpSolverContactConstraintPool,
		m_data21->m_tmpSolverNonContactConstraintPool,
		m_data21->m_tmpSolverContactFrictionConstraintPool,
		m_data21->m_tmpSolverContactRollingFrictionConstraintPool,
		m_data21->m_orderTmpConstraintPool,
		m_data21->m_orderNonContactConstraintPool,
		m_data21->m_orderFrictionConstraintPool,
		m_data21->m_tmpConstraintSizesPool,
		m_data21->m_resolveSingleConstraintRowGeneric,
		m_data21->m_resolveSingleConstraintRowLowerLimit,
		m_data21->m_resolveSplitPenetrationImpulse,
		m_data21->m_kinematicBodyUniqueIdToSolverBodyTable, m_data21->m_btSeed2,
		m_data21->m_fixedBodyId, m_data21->m_maxOverrideNumSolverIterations);

	m_data21->m_fixedBodyId = -1;
	// todo: setup sse2/4 constraint row methods

	btSequentialImpulseConstraintSolver::convertBodiesInternal(siData, bodies,
															   numBodies, info);
	btSequentialImpulseConstraintSolver::convertJointsInternal(
		siData, constraints, numConstraints, info);

	int i;
	btPersistentManifold* manifold = 0;
	// btCollisionObject* colObj0=0,*colObj1=0;

	for (i = 0; i < numManifolds; i++)
	{
		manifold = manifoldPtr[i];
		btSequentialImpulseConstraintSolver::convertContactInternal(siData,
																	manifold, info);
	}

	int numNonContactPool = siData.m_tmpSolverNonContactConstraintPool.size();
	int numConstraintPool = siData.m_tmpSolverContactConstraintPool.size();
	int numFrictionPool = siData.m_tmpSolverContactFrictionConstraintPool.size();

	// @todo: use stack allocator for such temporarily memory, same for solver
	// bodies/constraints
	siData.m_orderNonContactConstraintPool.resizeNoInitialize(numNonContactPool);
	if ((info.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
		siData.m_orderTmpConstraintPool.resizeNoInitialize(numConstraintPool * 2);
	else
		siData.m_orderTmpConstraintPool.resizeNoInitialize(numConstraintPool);

	siData.m_orderFrictionConstraintPool.resizeNoInitialize(numFrictionPool);
	{
		int i;
		for (i = 0; i < numNonContactPool; i++)
		{
			siData.m_orderNonContactConstraintPool[i] = i;
		}
		for (i = 0; i < numConstraintPool; i++)
		{
			siData.m_orderTmpConstraintPool[i] = i;
		}
		for (i = 0; i < numFrictionPool; i++)
		{
			siData.m_orderFrictionConstraintPool[i] = i;
		}
	}

	btScalar leastSquaresResidual = 0;

	{
		BT_PROFILE("solveGroupCacheFriendlyIterations");
		/// this is a special step to resolve penetrations (just for contacts)
		btSequentialImpulseConstraintSolver::
			solveGroupCacheFriendlySplitImpulseIterationsInternal(
				siData, bodies, numBodies, manifoldPtr, numManifolds, constraints,
				numConstraints, info, debugDrawer);

		int maxIterations =
			siData.m_maxOverrideNumSolverIterations > info.m_numIterations
				? siData.m_maxOverrideNumSolverIterations
				: info.m_numIterations;

		for (int iteration = 0; iteration < maxIterations; iteration++)
		{
			leastSquaresResidual =
				btSequentialImpulseConstraintSolver::solveSingleIterationInternal(
					siData, iteration, constraints, numConstraints, info);

			if (leastSquaresResidual <= info.m_leastSquaresResidualThreshold ||
				(iteration >= (maxIterations - 1)))
			{
#ifdef VERBOSE_RESIDUAL_PRINTF
				printf("residual = %f at iteration #%d\n", m_leastSquaresResidual,
					   iteration);
#endif
				break;
			}
		}
	}

	btScalar res = btSequentialImpulseConstraintSolver::
		solveGroupCacheFriendlyFinishInternal(siData, bodies, numBodies, info);
	return res;
}

void btBlockSolver::solveMultiBodyGroup(
	btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold,
	int numManifolds, btTypedConstraint** constraints, int numConstraints,
	btMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints,
	const btContactSolverInfo& info, btIDebugDraw* debugDrawer,
	btDispatcher* dispatcher)
{
	btMultiBodyConstraintSolver::solveMultiBodyGroup(
		bodies, numBodies, manifold, numManifolds, constraints, numConstraints,
		multiBodyConstraints, numMultiBodyConstraints, info, debugDrawer,
		dispatcher);
}

void btBlockSolver::reset()
{
	// or just set m_data2->m_btSeed2=0?
	delete m_data21;
	delete m_data22;
	m_data21 = new btBlockSolverInternalData;
	m_data22 = new btBlockSolverInternalData;
}
