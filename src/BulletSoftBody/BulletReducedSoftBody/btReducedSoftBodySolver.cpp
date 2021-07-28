#include "btReducedSoftBodySolver.h"
#include "../btDeformableMultiBodyDynamicsWorld.h"

btReducedSoftBodySolver::btReducedSoftBodySolver()
{
  m_dampingAlpha = 0;
  m_dampingBeta = 0;
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
  applyExplicitForce();
}

void btReducedSoftBodySolver::applyForce()
{
  for (int i = 0; i < m_softBodies.size(); ++i)
  {
    btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);

    // get reduced force
    btAlignedObjectArray<btScalar> reduced_force;
    reduced_force.resize(rsb->m_reducedDofs.size(), 0);

    // add internal force (elastic force & damping force)
    rsb->applyReducedInternalForce(reduced_force, m_dampingAlpha, m_dampingBeta);

    // apply impulses to reduced deformable objects
    static btScalar sim_time = 0;
    static int apply_impulse = 0;
    // if (rsb->m_reducedModel && apply_impulse < 4)
    // {
    //   if (sim_time > 1 && apply_impulse == 0) 
    //   {
    //     rsb->applyFullSpaceImpulse(btVector3(0, 1, 0), 0, 2.0 * m_dt, reduced_force);
    //     apply_impulse++;
    //   }
    //   if (sim_time > 2 && apply_impulse == 1) 
    //   {
    //     rsb->applyFullSpaceImpulse(btVector3(0, -1.2, 0), 0, m_dt, reduced_force);
    //     apply_impulse++;
    //   }
    //   if (sim_time > 3 && apply_impulse == 2) 
    //   {
    //     rsb->applyFullSpaceImpulse(btVector3(1.1, 0, 0), 0, m_dt, reduced_force);
    //     apply_impulse++;
    //   }
    //   if (sim_time > 4 && apply_impulse == 3) 
    //   {
    //     rsb->applyFullSpaceImpulse(btVector3(-1, 0, 0), 0, 2.0 * m_dt, reduced_force);
    //     apply_impulse++;
    //   }
    // }

    // apply fixed contraints
    rsb->applyFixedContraints(m_dt, reduced_force);

    // update reduced velocity
    for (int r = 0; r < rsb->m_reducedDofs.size(); ++r)
    {
      btScalar mass_inv = (rsb->m_Mr[r] == 0) ? 0 : 1.0 / rsb->m_Mr[r];
      btScalar delta_v = m_dt * mass_inv * reduced_force[r];
      
      sim_time += m_dt;
      rsb->m_reducedVelocity[r] -= delta_v;
    }
  }
}

void btReducedSoftBodySolver::applyExplicitForce()
{
  // apply gravity to the rigid frame
  for (int i = 0; i < m_softBodies.size(); ++i)
  {
    btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);
    rsb->applyRigidGravity(m_gravity, m_dt);
  }

  // apply internal forces and impulses
  applyForce();
}

void btReducedSoftBodySolver::applyTransforms(btScalar timeStep)
{
  for (int i = 0; i < m_softBodies.size(); ++i)
  {
    btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);

    for (int r = 0; r < rsb->m_reducedDofs.size(); ++r)
      rsb->m_reducedDofs[r] += timeStep * rsb->m_reducedVelocity[r];

    // rigid motion
    rsb->predictIntegratedTransform(timeStep, rsb->getInterpolationWorldTransform());

    rsb->proceedToTransform(rsb->getInterpolationWorldTransform());

    // map reduced dof back to full space
    rsb->updateFullDofs();
  }
}