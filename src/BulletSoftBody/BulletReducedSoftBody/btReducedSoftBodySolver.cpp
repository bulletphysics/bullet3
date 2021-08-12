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

void btReducedSoftBodySolver::reinitialize(const btAlignedObjectArray<btSoftBody*>& bodies, btScalar dt)
{
  m_softBodies.copyFromArray(bodies);
	bool nodeUpdated = updateNodes();

	if (nodeUpdated)
	{
		m_dv.resize(m_numNodes, btVector3(0, 0, 0));
		m_ddv.resize(m_numNodes, btVector3(0, 0, 0));
		m_residual.resize(m_numNodes, btVector3(0, 0, 0));
		m_backupVelocity.resize(m_numNodes, btVector3(0, 0, 0));
	}

	// need to setZero here as resize only set value for newly allocated items
	for (int i = 0; i < m_numNodes; ++i)
	{
		m_dv[i].setZero();
		m_ddv[i].setZero();
		m_residual[i].setZero();
	}

	if (dt > 0)
	{
		m_dt = dt;
	}
	m_objective->reinitialize(nodeUpdated, dt);

  int N = bodies.size();
	if (nodeUpdated)
	{
		m_staticConstraints.resize(N);
		m_nodeRigidConstraints.resize(N);
		// m_faceRigidConstraints.resize(N);
	}
	for (int i = 0; i < N; ++i)
	{
		m_staticConstraints[i].clear();
		m_nodeRigidConstraints[i].clear();
		// m_faceRigidConstraints[i].clear();
	}

	btDeformableBodySolver::updateSoftBodies();
}

void btReducedSoftBodySolver::predictMotion(btScalar solverdt)
{
  applyExplicitForce(solverdt);

  // predict new mesh location
  predictReduceDeformableMotion(solverdt);

  //TODO: check if there is anything missed from btDeformableBodySolver::predictDeformableMotion
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

    // update tree
    rsb->updateNodeTree(true, true);
    if (!rsb->m_fdbvt.empty())
    {
      rsb->updateFaceTree(true, true);
    }

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
    if (!rsb->isActive())
		{
			continue;
		}

    // set fixed constraints
    for (int j = 0; j < rsb->m_fixedNodes.size(); ++j)
		{
			if (rsb->m_nodes[j].m_im == 0)
			{
				btReducedDeformableStaticConstraint static_constraint(rsb, &rsb->m_nodes[rsb->m_fixedNodes[j]], infoGlobal, m_dt);
				m_staticConstraints[i].push_back(static_constraint);
			}
		}
    btAssert(rsb->m_fixedNodes.size() == m_staticConstraints[i].size());

    // set Deformable Node vs. Rigid constraint
		for (int j = 0; j < rsb->m_nodeRigidContacts.size(); ++j)
		{
			const btSoftBody::DeformableNodeRigidContact& contact = rsb->m_nodeRigidContacts[j];
			// skip fixed points
			if (contact.m_node->m_im == 0)
			{
				continue;
			}
			btReducedDeformableNodeRigidContactConstraint constraint(rsb, contact, infoGlobal, m_dt);
			m_nodeRigidConstraints[i].push_back(constraint);
      rsb->m_contactNodesList.push_back(contact.m_node->index);
		}
    std::cout << "#contact nodes: " << m_nodeRigidConstraints[i].size() << "\n";

    // set Deformable Face vs. Rigid constraint
		// for (int j = 0; j < rsb->m_faceRigidContacts.size(); ++j)
		// {
		// 	const btSoftBody::DeformableFaceRigidContact& contact = rsb->m_faceRigidContacts[j];
		// 	// skip fixed faces
		// 	if (contact.m_c2 == 0)
		// 	{
		// 		continue;
		// 	}
		// 	btDeformableFaceRigidContactConstraint constraint(contact, infoGlobal, m_useStrainLimiting);
		// 	m_faceRigidConstraints[i].push_back(constraint);
		// }

  }
}

btScalar btReducedSoftBodySolver::solveContactConstraints(btCollisionObject** deformableBodies, int numDeformableBodies, const btContactSolverInfo& infoGlobal)
{
  btScalar residualSquare = 0;
	for (int i = 0; i < numDeformableBodies; ++i)
	{
		for (int j = 0; j < m_softBodies.size(); ++j)
		{
			btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(m_softBodies[i]);
			if (rsb != deformableBodies[i])
			{
				continue;
			}
			for (int k = 0; k < m_nodeRigidConstraints[j].size(); ++k)
			{
				btReducedDeformableNodeRigidContactConstraint& constraint = m_nodeRigidConstraints[j][k];
				btScalar localResidualSquare = constraint.solveConstraint(infoGlobal);
				residualSquare = btMax(residualSquare, localResidualSquare);
			}
			// for (int k = 0; k < m_faceRigidConstraints[j].size(); ++k)
			// {
			// 	btReducedDeformableFaceRigidContactConstraint& constraint = m_faceRigidConstraints[j][k];
			// 	btScalar localResidualSquare = constraint.solveConstraint(infoGlobal);
			// 	residualSquare = btMax(residualSquare, localResidualSquare);
			// }
		}
	}
	return residualSquare;
}