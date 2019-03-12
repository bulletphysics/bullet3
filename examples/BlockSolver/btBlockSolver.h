#ifndef BT_BLOCK_SOLVER_H
#define BT_BLOCK_SOLVER_H

#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"

class btBlockSolver : public btMultiBodyConstraintSolver
{
	struct btBlockSolverInternalData* m_data2;

public:
	btBlockSolver();
	virtual ~btBlockSolver();

	//btRigidBody
	virtual btScalar solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info, class btIDebugDraw* debugDrawer, btDispatcher* dispatcher);

	//btMultibody
	virtual void solveMultiBodyGroup(btCollisionObject * *bodies, int numBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, btMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher);

	///clear internal cached data and reset random seed
	virtual void reset();

	virtual btConstraintSolverType getSolverType() const
	{
		return BT_BLOCK_SOLVER;
	}
};

#endif //BT_BLOCK_SOLVER_H
