#include "btReducedSoftBodySolver.h"
#include "../btDeformableMultiBodyDynamicsWorld.h"

void btReducedSoftBodySolver::predictMotion(btScalar solverdt)
{
  applyForce();
  
  // apply rigid motion
  for (int i = 0; i < m_softBodies.size(); ++i)
  {
    btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);

    // rsb->predictIntegratedTransform(solverdt, rsb->getInterpolationWorldTransform());
  }
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
    for (int r = 0; r < rsb->m_reducedDofs.size(); ++r) {
      // map all force to reduced
      // for (int i = 0; i < force.size(); ++i)
      // 	for (int k = 0; k < 3; ++k)
      // 		reduced_force[r] += scale * rsb->m_modes[r][3 * i + k] * force[i][k];

      // std::cout << reduced_force[r] << '\t';

      reduced_force[r] += rsb->m_Kr[r] * (rsb->m_reducedDofs[r] + 0.1 * rsb->m_reducedVelocity[r]);
      // std::cout << reduced_force[r] << '\n';
      // std::cout << rsb->m_Kr[r] << "\t" << rsb->m_reducedDofs[r] << "\n";
    }


    // apply impulses to reduced deformable objects
    static btScalar sim_time = 0;
    static btScalar target_vel = 5;
    static int apply_impulse = 0;
    if (rsb->m_reducedModel && apply_impulse < 4 && sim_time > 1)
    {
      btScalar f_imp = 0;
      btVector3 imp(0, 0, 0);
      if (sim_time > 1 && apply_impulse == 0) 
      {
        f_imp = rsb->m_nodes[i].m_im * (target_vel - rsb->m_nodes[0].m_v[1]) / m_dt;
        imp[1] = f_imp;
        apply_impulse++;
        std::cout << "hit" << "\n";
      }
      if (sim_time > 2 && apply_impulse == 1) 
      {
        f_imp = - 1.2 * rsb->m_nodes[i].m_im * (target_vel - rsb->m_nodes[0].m_v[1]) / m_dt;
        imp[1] = f_imp;
        apply_impulse++;
        std::cout << "hit2" << "\n";
      }
      if (sim_time > 3 && apply_impulse == 2) 
      {
        f_imp = 1 * rsb->m_nodes[i].m_im * (target_vel - rsb->m_nodes[0].m_v[0]) / m_dt;
        imp[2] = f_imp;
        apply_impulse++;
        std::cout << "hit3" << "\n";
      }
      if (sim_time > 4 && apply_impulse == 3) 
      {
        f_imp = -1.1 * rsb->m_nodes[i].m_im * (target_vel - rsb->m_nodes[0].m_v[0]) / m_dt;
        imp[2] = f_imp;
        apply_impulse++;
        std::cout << "hit4" << "\n";
      }
      for (int i = 0; i < rsb->m_reducedDofs.size(); ++i)
      {
        reduced_force[i] += rsb->m_modes[i][0 * 3 + 1] * f_imp;
      }

      
      rsb->applyImpulse(imp, rsb->m_nodes[0].m_x);
    }

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
  }
}