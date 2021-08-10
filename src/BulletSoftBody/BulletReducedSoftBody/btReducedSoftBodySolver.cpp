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

    // clear contacts variables
		rsb->m_nodeRigidContacts.resize(0);
		rsb->m_faceRigidContacts.resize(0);
		rsb->m_faceNodeContacts.resize(0);
    
    // calculate inverse mass matrix for all nodes
    if (rsb->isActive())
    {
      for (int j = 0; j < rsb->m_nodes.size(); ++j)
      {
        if (rsb->m_nodes[j].m_im > 0)
        {
          rsb->m_nodes[j].m_effectiveMass_inv = rsb->m_nodes[j].m_effectiveMass.inverse();
        }
      }
    }

    // apply damping
    rsb->applyDamping(solverdt);

    // rigid motion
    rsb->predictIntegratedTransform(solverdt, rsb->getInterpolationWorldTransform());

    // update reduced velocity and dofs
    rsb->updateReducedVelocity(solverdt);

    // update reduced dofs
    rsb->updateReducedDofs(solverdt);

    // update local moment arm
    rsb->updateLocalMomentArm();
    rsb->updateExternalForceProjectMatrix(true);

    // predict full space velocity (needed for constraints)
    rsb->mapToFullVelocity(rsb->getInterpolationWorldTransform());

    // update full space nodal position
    rsb->mapToFullPosition(rsb->getInterpolationWorldTransform());

    // update bounding box
    rsb->updateBounds();

    // std::cout << "bounds\n";
    // std::cout << rsb->m_bounds[0][0] << '\t' << rsb->m_bounds[0][1] << '\t' << rsb->m_bounds[0][2] << '\n';
    // std::cout << rsb->m_bounds[1][0] << '\t' << rsb->m_bounds[1][1] << '\t' << rsb->m_bounds[1][2] << '\n';


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
    rsb->proceedToTransform(timeStep, true);

    // update mesh nodal positions for the next time step
    rsb->mapToFullPosition(rsb->getRigidTransform());

    // end of time step clean up and update
    rsb->endOfTimeStepZeroing();
  }
  m_simTime += timeStep;
}

void btReducedSoftBodySolver::setConstraints(const btContactSolverInfo& infoGlobal)
{
  for (int i = 0; i < m_softBodies.size(); ++i)
  {
    btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);

    // set fixed constraints
    for (int j = 0; j < rsb->m_nodes.size(); ++j)
		{
			// if (rsb->m_nodes[j].m_im == 0)
			// {
			// 	btDeformableStaticConstraint static_constraint(&rsb->m_nodes[j], infoGlobal);
			// 	m_staticConstraints[i].push_back(static_constraint);
			// }
		}

    // set Deformable Node vs. Rigid constraint
    rsb->m_contactNodesList.clear();
		for (int j = 0; j < rsb->m_nodeRigidContacts.size(); ++j)
		{
      rsb->m_contactNodesList.push_back(rsb->m_nodeRigidContacts[j].m_node->index);
		// 	const btSoftBody::DeformableNodeRigidContact& contact = rsb->m_nodeRigidContacts[j];
		// 	// skip fixed points
		// 	if (contact.m_node->m_im == 0)
		// 	{
		// 		continue;
		// 	}
		// 	btDeformableNodeRigidContactConstraint constraint(contact, infoGlobal);
		// 	m_nodeRigidConstraints[i].push_back(constraint);
		}
    std::cout << "#contact nodes: " << rsb->m_contactNodesList.size() << "\n";

  }
}

void btReducedSoftBodySolver::solveDeformableConstraints(btScalar timeStep)
{
  for (int iter = 0; iter < 100; ++iter)  // TODO: add a fllag for this.
  {
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
      btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);

      rsb->applyFixedContraints(timeStep);
    }
  }
}