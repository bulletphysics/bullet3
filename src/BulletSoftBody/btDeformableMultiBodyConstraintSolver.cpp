/*
 Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>
 
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#include "btDeformableMultiBodyConstraintSolver.h"
#include "BulletReducedDeformableBody/btReducedDeformableBodySolver.h"
#include <iostream>

// override the iterations method to include deformable/multibody contact
btScalar btDeformableMultiBodyConstraintSolver::solveDeformableGroupIterations(btCollisionObject** bodies, int numBodies, btCollisionObject** deformableBodies, int numDeformableBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
{
	{
		// pair deformable body with solver body
		pairDeformableAndSolverBody(bodies, numBodies, numDeformableBodies, infoGlobal);

		///this is a special step to resolve penetrations (just for contacts)
		solveGroupCacheFriendlySplitImpulseIterations(bodies, numBodies, deformableBodies, numDeformableBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);

		int maxIterations = m_maxOverrideNumSolverIterations > infoGlobal.m_numIterations ? m_maxOverrideNumSolverIterations : infoGlobal.m_numIterations;
		for (int iteration = 0; iteration < maxIterations; iteration++)
		{
			// rigid bodies are solved using solver body velocity, but rigid/deformable contact directly uses the velocity of the actual rigid body. So we have to do the following: Solve one iteration of the rigid/rigid contact, get the updated velocity in the solver body and update the velocity of the underlying rigid body. Then solve the rigid/deformable contact. Finally, grab the (once again) updated rigid velocity and update the velocity of the wrapping solver body

			// solve rigid/rigid in solver body
			m_leastSquaresResidual = solveSingleIteration(iteration, bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
			// solver body velocity -> rigid body velocity
			solverBodyWriteBack(infoGlobal);
			btScalar deformableResidual = m_deformableSolver->solveContactConstraints(deformableBodies, numDeformableBodies, infoGlobal);
			// update rigid body velocity in rigid/deformable contact
			m_leastSquaresResidual = btMax(m_leastSquaresResidual, deformableResidual);
			// solver body velocity <- rigid body velocity
			writeToSolverBody(bodies, numBodies, infoGlobal);

			// std::cout << "------------Iteration " << iteration << "------------\n";
			// std::cout << "m_leastSquaresResidual: " << m_leastSquaresResidual << "\n";

			if (m_leastSquaresResidual <= infoGlobal.m_leastSquaresResidualThreshold || (iteration >= (maxIterations - 1)))
			{
#ifdef VERBOSE_RESIDUAL_PRINTF
				if (iteration >= (maxIterations - 1))
					printf("residual = %f at iteration #%d\n", m_leastSquaresResidual, iteration);
#endif
				m_analyticsData.m_numSolverCalls++;
				m_analyticsData.m_numIterationsUsed = iteration + 1;
				m_analyticsData.m_islandId = -2;
				if (numBodies > 0)
					m_analyticsData.m_islandId = bodies[0]->getCompanionId();
				m_analyticsData.m_numBodies = numBodies;
				m_analyticsData.m_numContactManifolds = numManifolds;
				m_analyticsData.m_remainingLeastSquaresResidual = m_leastSquaresResidual;

				m_deformableSolver->deformableBodyInternalWriteBack();
				// std::cout << "[===================Next Step===================]\n";
				break;
			}
		}
	}
	return 0.f;
}

void btDeformableMultiBodyConstraintSolver::solveDeformableBodyGroup(btCollisionObject** bodies, int numBodies, btCollisionObject** deformableBodies, int numDeformableBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, btMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher)
{
	m_tmpMultiBodyConstraints = multiBodyConstraints;
	m_tmpNumMultiBodyConstraints = numMultiBodyConstraints;

	// inherited from MultiBodyConstraintSolver
	solveGroupCacheFriendlySetup(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);

	// overriden
	solveDeformableGroupIterations(bodies, numBodies, deformableBodies, numDeformableBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);

	// inherited from MultiBodyConstraintSolver
	solveGroupCacheFriendlyFinish(bodies, numBodies, info);

	m_tmpMultiBodyConstraints = 0;
	m_tmpNumMultiBodyConstraints = 0;
}

void btDeformableMultiBodyConstraintSolver::synchronizeSolverBodyWithRigidBody(btSolverBody* solverBody, btRigidBody* rigidBody)
{
	// Compute the total velocity change
	btVector3 totalDeltaLinearVelocity = rigidBody->getLinearVelocity() - (solverBody->m_linearVelocity + solverBody->m_deltaLinearVelocity);
	btVector3 totalDeltaAngularVelocity = rigidBody->getAngularVelocity() - (solverBody->m_angularVelocity + solverBody->m_deltaAngularVelocity);

	// Update the delta velocities
	solverBody->m_deltaLinearVelocity += totalDeltaLinearVelocity;
	solverBody->m_deltaAngularVelocity += totalDeltaAngularVelocity;

	// Adjust the solver body's base velocities
	solverBody->m_linearVelocity = rigidBody->getLinearVelocity() - solverBody->m_deltaLinearVelocity;
	solverBody->m_angularVelocity = rigidBody->getAngularVelocity() - solverBody->m_deltaAngularVelocity;
}

void btDeformableMultiBodyConstraintSolver::writeToSolverBody(btCollisionObject** bodies, int numBodies, const btContactSolverInfo& infoGlobal)
{
	// reduced soft body solver directly modifies the solver body
	if (m_deformableSolver->isReducedSolver())
	{
		return;
	}

	for (int i = 0; i < numBodies; i++)
	{
		int bodyId = getOrInitSolverBody(*bodies[i], infoGlobal.m_timeStep);

		btRigidBody* body = btRigidBody::upcast(bodies[i]);
		if (body && body->getInvMass())
		{
			btSolverBody& solverBody = m_tmpSolverBodyPool[bodyId];
			synchronizeSolverBodyWithRigidBody(&solverBody, body);
		}
	}
}

void btDeformableMultiBodyConstraintSolver::solverBodyWriteBack(const btContactSolverInfo& infoGlobal)
{
	// reduced soft body solver directly modifies the solver body
	if (m_deformableSolver->isReducedSolver())
	{
		return;
	}

	for (int i = 0; i < m_tmpSolverBodyPool.size(); i++)
	{
		btRigidBody* body = m_tmpSolverBodyPool[i].m_originalBody;
		if (body)
		{
			m_tmpSolverBodyPool[i].m_originalBody->setLinearVelocity(m_tmpSolverBodyPool[i].m_linearVelocity + m_tmpSolverBodyPool[i].m_deltaLinearVelocity);
			m_tmpSolverBodyPool[i].m_originalBody->setAngularVelocity(m_tmpSolverBodyPool[i].m_angularVelocity + m_tmpSolverBodyPool[i].m_deltaAngularVelocity);
		}
	}
}

void btDeformableMultiBodyConstraintSolver::pairDeformableAndSolverBody(btCollisionObject** bodies, int numBodies, int numDeformableBodies, const btContactSolverInfo& infoGlobal)
{
	if (!m_deformableSolver->isReducedSolver())
	{
		return;
	}

	btReducedDeformableBodySolver* solver = static_cast<btReducedDeformableBodySolver*>(m_deformableSolver);

	for (int i = 0; i < numDeformableBodies; ++i)
	{
		for (int k = 0; k < solver->m_nodeRigidConstraints[i].size(); ++k)
		{
			btReducedDeformableNodeRigidContactConstraint& constraint = solver->m_nodeRigidConstraints[i][k];

			if (!constraint.m_contact->m_cti.m_colObj->isStaticObject())
			{
				btCollisionObject& col_obj = const_cast<btCollisionObject&>(*constraint.m_contact->m_cti.m_colObj);

				// object index in the solver body pool
				int bodyId = getOrInitSolverBody(col_obj, infoGlobal.m_timeStep);

				const btRigidBody* body = btRigidBody::upcast(bodies[bodyId]);
				if (body && body->getInvMass())
				{
					// std::cout << "Node: " << constraint.m_node->index << ", body: " << bodyId << "\n";
					btSolverBody& solverBody = m_tmpSolverBodyPool[bodyId];
					constraint.setSolverBody(bodyId, solverBody);
				}
			}
		}

		// for (int j = 0; j < numBodies; j++)
		// {
		// 	int bodyId = getOrInitSolverBody(*bodies[j], infoGlobal.m_timeStep);

		// 	btRigidBody* body = btRigidBody::upcast(bodies[j]);
		// 	if (body && body->getInvMass())
		// 	{
		// 		btSolverBody& solverBody = m_tmpSolverBodyPool[bodyId];
		// 		m_deformableSolver->pairConstraintWithSolverBody(i, bodyId, solverBody);
		// 	}
		// }
	}
}

void btDeformableMultiBodyConstraintSolver::solveGroupCacheFriendlySplitImpulseIterations(btCollisionObject** bodies, int numBodies, btCollisionObject** deformableBodies, int numDeformableBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
{
	BT_PROFILE("solveGroupCacheFriendlySplitImpulseIterations");
	int iteration;
	if (infoGlobal.m_splitImpulse)
	{
		{
			for (iteration = 0; iteration < infoGlobal.m_numIterations; iteration++)
			{
				btScalar leastSquaresResidual = 0.f;
				{
					int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
					int j;
					for (j = 0; j < numPoolConstraints; j++)
					{
						const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];

						btScalar residual = resolveSplitPenetrationImpulse(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold);
						leastSquaresResidual = btMax(leastSquaresResidual, residual * residual);
					}
					// solve the position correction between deformable and rigid/multibody
					//                    btScalar residual = m_deformableSolver->solveSplitImpulse(infoGlobal);
					btScalar residual = m_deformableSolver->m_objective->m_projection.solveSplitImpulse(deformableBodies, numDeformableBodies, infoGlobal);
					leastSquaresResidual = btMax(leastSquaresResidual, residual * residual);
				}
				if (leastSquaresResidual <= infoGlobal.m_leastSquaresResidualThreshold || iteration >= (infoGlobal.m_numIterations - 1))
				{
#ifdef VERBOSE_RESIDUAL_PRINTF
					if (iteration >= (infoGlobal.m_numIterations - 1))
						printf("split impulse residual = %f at iteration #%d\n", leastSquaresResidual, iteration);
#endif
					break;
				}
			}
		}
	}
}
