#ifndef BT_BLOCK_SOLVER_H
#define BT_BLOCK_SOLVER_H

#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "Bullet3Common/b3Logging.h"

enum BlockSolverOptions
{
	BLOCK_SOLVER_SI = 1 << 0,
	BLOCK_SOLVER_MLCP_PGS = 1 << 1,
	BLOCK_SOLVER_MLCP_DANTZIG = 1 << 2,
	BLOCK_SOLVER_BLOCK = 1 << 3,

	BLOCK_SOLVER_SCENE_MB_STACK = 1 << 5,
	BLOCK_SOLVER_SCENE_CHAIN = 1 << 6,
};

class btBlockSolver : public btMultiBodyConstraintSolver
{
  struct btBlockSolverInternalData* m_data21;
  struct btBlockSolverInternalData* m_data22;
 public
     : btBlockSolver();

  virtual ~btBlockSolver();

  // btRigidBody
  virtual btScalar solveGroup(btCollisionObject** bodies, int numBodies,
                              btPersistentManifold** manifoldPtr,
                              int numManifolds, btTypedConstraint** constraints,
                              int numConstraints,
                              const btContactSolverInfo& info,
                              class btIDebugDraw* debugDrawer,
                              btDispatcher* dispatcher);

  btScalar solveGroupInternal(btCollisionObject** bodies, int numBodies,
                              btPersistentManifold** manifoldPtr,
                              int numManifolds,
                              btTypedConstraint** constraints,
                              int numConstraints,
                              const btContactSolverInfo& info,
                              btIDebugDraw* debugDrawer,
                              btDispatcher* dispatcher);

  btScalar solveGroupInternalBlock(btCollisionObject** bodies, int numBodies,
                                   btPersistentManifold** manifoldPtr,
                                   int numManifolds,
                                   btTypedConstraint** constraints,
                                   int numConstraints,
                                   const btContactSolverInfo& info,
                                   btIDebugDraw* debugDrawer,
                                   btDispatcher* dispatcher);

  // btMultibody
  virtual void solveMultiBodyGroup(
      btCollisionObject** bodies, int numBodies,
      btPersistentManifold** manifold, int numManifolds,
      btTypedConstraint** constraints, int numConstraints,
      btMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints,
      const btContactSolverInfo& info, btIDebugDraw* debugDrawer,
      btDispatcher* dispatcher);

  /// clear internal cached data and reset random seed
  virtual void reset();

	virtual btConstraintSolverType getSolverType() const
	{
		return BT_BLOCK_SOLVER;
	}
};

#endif  //BT_BLOCK_SOLVER_H
