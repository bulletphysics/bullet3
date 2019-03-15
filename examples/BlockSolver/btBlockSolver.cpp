#include "btBlockSolver.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "LinearMath/btQuickprof.h"

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
	btAlignedObjectArray<btTypedConstraint::btConstraintInfo1> m_tmpConstraintSizesPool;
	
	
	unsigned long m_btSeed2;
	int m_fixedBodyId;
	int m_maxOverrideNumSolverIterations;
	btAlignedObjectArray<int> m_kinematicBodyUniqueIdToSolverBodyTable;  // only used for multithreading

	btSingleConstraintRowSolver m_resolveSingleConstraintRowGeneric;
	btSingleConstraintRowSolver m_resolveSingleConstraintRowLowerLimit;
	btSingleConstraintRowSolver m_resolveSplitPenetrationImpulse;

	btBlockSolverInternalData()
		:m_btSeed2(0),
		m_fixedBodyId(-1),
		m_maxOverrideNumSolverIterations(0),
		m_resolveSingleConstraintRowGeneric(btSequentialImpulseConstraintSolver::getScalarConstraintRowSolverGeneric()),
		m_resolveSingleConstraintRowLowerLimit(btSequentialImpulseConstraintSolver::getScalarConstraintRowSolverLowerLimit()),
		m_resolveSplitPenetrationImpulse(btSequentialImpulseConstraintSolver::getScalarSplitPenetrationImpulseGeneric())
	{
	}
};




btBlockSolver::btBlockSolver()
{
	m_data2 = new btBlockSolverInternalData;
}

btBlockSolver::~btBlockSolver()
{
	delete m_data2;
}


btScalar btBlockSolver::solveGroup(btCollisionObject * *bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher)
{

	btSISolverSingleIterationData siData(m_data2->m_tmpSolverBodyPool,
		m_data2->m_tmpSolverContactConstraintPool,
		m_data2->m_tmpSolverNonContactConstraintPool,
		m_data2->m_tmpSolverContactFrictionConstraintPool,
		m_data2->m_tmpSolverContactRollingFrictionConstraintPool,
		m_data2->m_orderTmpConstraintPool,
		m_data2->m_orderNonContactConstraintPool,
		m_data2->m_orderFrictionConstraintPool,
		m_data2->m_tmpConstraintSizesPool,
		m_data2->m_resolveSingleConstraintRowGeneric,
		m_data2->m_resolveSingleConstraintRowLowerLimit,
		m_data2->m_resolveSplitPenetrationImpulse,
		m_data2->m_kinematicBodyUniqueIdToSolverBodyTable,
		m_data2->m_btSeed2,
		m_data2->m_fixedBodyId,
		m_data2->m_maxOverrideNumSolverIterations);
	
	m_data2->m_fixedBodyId = -1;
	//todo: setup sse2/4 constraint row methods

	btSequentialImpulseConstraintSolver::convertBodiesInternal(siData, bodies, numBodies, info);
	btSequentialImpulseConstraintSolver::convertJointsInternal(siData, constraints, numConstraints, info);
	
	int i;
	btPersistentManifold* manifold = 0;
	//			btCollisionObject* colObj0=0,*colObj1=0;

	for (i = 0; i < numManifolds; i++)
	{
		manifold = manifoldPtr[i];
		btSequentialImpulseConstraintSolver::convertContactInternal(siData, manifold, info);
	}



	int numNonContactPool = siData.m_tmpSolverNonContactConstraintPool.size();
	int numConstraintPool = siData.m_tmpSolverContactConstraintPool.size();
	int numFrictionPool = siData.m_tmpSolverContactFrictionConstraintPool.size();

	///@todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
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
		///this is a special step to resolve penetrations (just for contacts)
		btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySplitImpulseIterationsInternal(siData, bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, info, debugDrawer);

		int maxIterations = siData.m_maxOverrideNumSolverIterations > info.m_numIterations ? siData.m_maxOverrideNumSolverIterations : info.m_numIterations;

		for (int iteration = 0; iteration < maxIterations; iteration++)
		{
			leastSquaresResidual = btSequentialImpulseConstraintSolver::solveSingleIterationInternal(siData, iteration, constraints, numConstraints, info);

			if (leastSquaresResidual <= info.m_leastSquaresResidualThreshold || (iteration >= (maxIterations - 1)))
			{
#ifdef VERBOSE_RESIDUAL_PRINTF
				printf("residual = %f at iteration #%d\n", m_leastSquaresResidual, iteration);
#endif
				break;
			}
		}
	}

	btScalar res = btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyFinishInternal(siData, bodies, numBodies, info);
	return res;
}



void btBlockSolver::solveMultiBodyGroup(btCollisionObject * *bodies, int numBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, btMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher)
{
	btMultiBodyConstraintSolver::solveMultiBodyGroup(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, multiBodyConstraints, numMultiBodyConstraints, info, debugDrawer, dispatcher);
}

void btBlockSolver::reset()
{
	//or just set m_data2->m_btSeed2=0?
	delete m_data2;
	m_data2 = new btBlockSolverInternalData;
}
