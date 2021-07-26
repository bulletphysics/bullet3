#ifndef BT_REDUCED_SOFT_BODY_SOLVER_H
#define BT_REDUCED_SOFT_BODY_SOLVER_H

#include "btReducedSoftBody.h"
#include "../btDeformableBodySolver.h"

class btReducedSoftBody;

class btReducedSoftBodySolver : public btDeformableBodySolver
{
 protected:
  void applyForce();

 public:
  btReducedSoftBodySolver() {}
  ~btReducedSoftBodySolver() {}

  virtual SolverTypes getSolverType() const
  {
    return REDUCED_DEFORMABLE_SOLVER;
  }

  // virtual void predictMotion(btScalar solver_dt);

  // virtual void solveConstraints(btScalar solver_dt);

  virtual void predictMotion(btScalar solverdt);

  virtual void applyExplicitForce();

  virtual void applyTransforms(btScalar timeStep);

};

#endif // BT_REDUCED_SOFT_BODY_DYNAMICS_WORLD_H