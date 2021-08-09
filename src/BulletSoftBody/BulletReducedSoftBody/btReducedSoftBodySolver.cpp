#include "btReducedSoftBodySolver.h"
#include "../btDeformableMultiBodyDynamicsWorld.h"

btReducedSoftBodySolver::btReducedSoftBodySolver()
{
  m_dampingAlpha = 0;
  m_dampingBeta = 0;
  m_simTime = 0;
  m_gravity = btVector3(0, 0, 0);
}

void btReducedSoftBodySolver::setDamping(btScalar alpha, btScalar beta)
{
  m_dampingAlpha = alpha;
  m_dampingBeta = beta;
}

void btReducedSoftBodySolver::setGravity(const btVector3& gravity)
{
  m_gravity = gravity;
}

void btReducedSoftBodySolver::predictMotion(btScalar solverdt)
{
  applyExplicitForce(solverdt);

  // predict new mesh location
  predictReduceDeformableMotion(solverdt);
}

void btReducedSoftBodySolver::predictReduceDeformableMotion(btScalar solverdt)
{
  for (int i = 0; i < m_softBodies.size(); ++i)
  {
    btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);

    // apply damping
    rsb->applyDamping(solverdt);

    // rigid motion
    rsb->predictIntegratedTransform(solverdt, rsb->getInterpolationWorldTransform());

    // std::cout << "reduced_dofs: " << rsb->m_reducedDofs[0] << '\t' << rsb->m_reducedDofs[1] << '\n';
    // std::cout << "reduced_vels: " << rsb->m_reducedVelocity[0] << '\t' << rsb->m_reducedVelocity[1] << '\n';

    // update reduced velocity and dofs
    rsb->updateReducedVelocity(solverdt); // TODO: add back

    // update reduced dofs
    rsb->updateReducedDofs(solverdt);

    // update local moment arm
    rsb->updateLocalMomentArm();
    rsb->updateExternalForceProjectMatrix(true);

    // predict full space velocity (needed for constraints)
    rsb->mapToFullVelocity(rsb->getInterpolationWorldTransform());

    // apply fixed constraints
    // rsb->applyFixedContraints(solverdt);

    // TODO: update mesh nodal position. need it for collision
    // rsb->updateMeshNodePositions(solverdt);
  }
}

void btReducedSoftBodySolver::applyExplicitForce(btScalar solverdt)
{
  static bool applied = false;
  for (int i = 0; i < m_softBodies.size(); ++i)
  {
    btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);

    // apply gravity to the rigid frame
    rsb->applyRigidGravity(m_gravity, solverdt);

    // add internal force (elastic force & damping force)
    rsb->applyReducedInternalForce(m_dampingAlpha, m_dampingBeta);

    // apply external force or impulses
    // if (!applied && m_simTime > 2)
    // {
    //   rsb->applyFullSpaceImpulse(btVector3(0, -5, 0), 0, solverdt);
    //   applied = true;
    // }
  }
}

void btReducedSoftBodySolver::applyTransforms(btScalar timeStep)
{
  for (int i = 0; i < m_softBodies.size(); ++i)
  {
    btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);

    // update reduced dofs for the next time step
    // rsb->updateReducedDofs(timeStep); // TODO: add back

    // rigid motion
    // btTransform predictedTrans;
    // rsb->predictIntegratedTransform(timeStep, predictedTrans);
    // rsb->proceedToTransform(rsb->getInterpolationWorldTransform());
    rsb->proceedToTransform(timeStep, true);

    // update mesh nodal positions for the next time step
    rsb->mapToFullDofs(rsb->getRigidTransform());

    // end of time step clean up and update
    // rsb->updateLocalMomentArm();
    // rsb->updateExternalForceProjectMatrix(true);
    rsb->endOfTimeStepZeroing();
  }
  m_simTime += timeStep;
}

void btReducedSoftBodySolver::solveConstraints(btScalar timeStep)
{
  for (int i = 0; i < m_softBodies.size(); ++i)
  {
    btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);

    rsb->applyFixedContraints(timeStep);

//    std::cout << rsb->m_reducedForce[0] << '\t' << rsb->m_reducedForce[1] << '\n';
  }
}