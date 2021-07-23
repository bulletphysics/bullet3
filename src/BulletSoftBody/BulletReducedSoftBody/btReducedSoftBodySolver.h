#ifndef BT_REDUCED_SOFT_BODY_SOLVER_H
#define BT_REDUCED_SOFT_BODY_SOLVER_H

#include "btSoftBodySolver.h"
#include "btDeformableMultiBodyDynamicsWorld.h"

class btReducedSoftBodySolver : public btSoftBodySolver
{
  typedef btAlignedObjectArray<btReduceSoftBody*> btReducedSoftBodyArray;
  typedef btAlignedObjectArray<btVector3> TVStack;

 protected:
  btReducedSoftBodyArray m_reducedSoftBodies;
  btScalar m_dt;

 public:
  btReducedSoftBodySolver() : m_dt(0) {}
  ~btReducedSoftBodySolver() {}

  virtual SolverTypes getSolverType() const
  {
    return REDUCED_DEFORMABLE_SOLVER;
  }

  virtual void predictMotion(btScalar solver_dt);

  virtual void solveConstraints(btScalar solver_dt);

};

#endif // BT_REDUCED_SOFT_BODY_DYNAMICS_WORLD_H