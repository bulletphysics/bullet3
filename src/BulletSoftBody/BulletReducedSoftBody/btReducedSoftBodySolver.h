#ifndef BT_REDUCED_SOFT_BODY_SOLVER_H
#define BT_REDUCED_SOFT_BODY_SOLVER_H

#include "../btDeformableBodySolver.h"
#include "btReducedSoftBody.h"
#include "btReducedDeformableContactConstraint.h"

class btReducedSoftBody;

class btReducedSoftBodySolver : public btDeformableBodySolver
{
 protected:
  btScalar m_dampingAlpha;
  btScalar m_dampingBeta;

  btVector3 m_gravity;

  void predictReduceDeformableMotion(btScalar solverdt);

  void applyExplicitForce(btScalar solverdt);

 public:
  btAlignedObjectArray<btAlignedObjectArray<btReducedDeformableStaticConstraint> > m_staticConstraints;
  btAlignedObjectArray<btAlignedObjectArray<btReducedDeformableNodeRigidContactConstraint> > m_nodeRigidConstraints;
  btAlignedObjectArray<btAlignedObjectArray<btReducedDeformableFaceRigidContactConstraint> > m_faceRigidConstraints;
  
  btReducedSoftBodySolver();
  ~btReducedSoftBodySolver() {}

  void setGravity(const btVector3& gravity);

  virtual SolverTypes getSolverType() const
  {
    return REDUCED_DEFORMABLE_SOLVER;
  }

  // resize/clear data structures
	virtual void reinitialize(const btAlignedObjectArray<btSoftBody*>& bodies, btScalar dt);

  virtual void predictMotion(btScalar solverdt);

  virtual void applyTransforms(btScalar timeStep);

  // set up contact constraints
	virtual void setConstraints(const btContactSolverInfo& infoGlobal);

  // pair rigid contact constraint with solver body
  virtual void pairConstraintWithSolverBody(btSolverBody& solverBody);

  // solve all constraints (fixed and contact)
  virtual btScalar solveContactConstraints(btCollisionObject** deformableBodies, int numDeformableBodies, const btContactSolverInfo& infoGlobal);

  // apply all the delta velocities
  virtual void deformableBodyInternalWriteBack();

  // virtual void setProjection() {}

  // virtual void setLagrangeMultiplier() {}

  // virtual void setupDeformableSolve(bool implicit);

};

#endif // BT_REDUCED_SOFT_BODY_DYNAMICS_WORLD_H