/*
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

/* ====== Overview of the Deformable Algorithm ====== */

/*
A single step of the deformable body simulation contains the following main components:
1. Update velocity to a temporary state v_{n+1}^* = v_n + explicit_force * dt / mass, where explicit forces include gravity and elastic forces.
2. Detect collisions between rigid and deformable bodies at position x_{n+1}^* = x_n + dt * v_{n+1}^*.
3. Then velocities of deformable bodies v_{n+1} are solved in
        M(v_{n+1} - v_{n+1}^*) = damping_force * dt / mass,
   by a conjugate gradient solver, where the damping force is implicit and depends on v_{n+1}.
4. Contact constraints are solved as projections as in the paper by Baraff and Witkin https://www.cs.cmu.edu/~baraff/papers/sig98.pdf. Dynamic frictions are treated as a force and added to the rhs of the CG solve, whereas static frictions are treated as constraints similar to contact.
5. Position is updated via x_{n+1} = x_n + dt * v_{n+1}.
6. Apply position correction to prevent numerical drift.

The algorithm also closely resembles the one in http://physbam.stanford.edu/~fedkiw/papers/stanford2008-03.pdf
 */

#include <stdio.h>
#include "btDeformableRigidDynamicsWorld.h"
#include "btDeformableBodySolver.h"
#include "LinearMath/btQuickprof.h"

void btDeformableRigidDynamicsWorld::internalSingleStepSimulation(btScalar timeStep)
{
    BT_PROFILE("internalSingleStepSimulation");
    reinitialize(timeStep);
    // add gravity to velocity of rigid and multi bodys
    applyRigidBodyGravity(timeStep);
    
    ///apply gravity and explicit force to velocity, predict motion
    predictUnconstraintMotion(timeStep);
    
    ///perform collision detection
    btMultiBodyDynamicsWorld::performDiscreteCollisionDetection();
    
    btMultiBodyDynamicsWorld::calculateSimulationIslands();
    
    beforeSolverCallbacks(timeStep);
    
    ///solve deformable bodies constraints
    solveDeformableBodiesConstraints(timeStep);
    
    afterSolverCallbacks(timeStep);
    
    integrateTransforms(timeStep);
    
    ///update vehicle simulation
    btMultiBodyDynamicsWorld::updateActions(timeStep);
    
    btMultiBodyDynamicsWorld::updateActivationState(timeStep);
    // End solver-wise simulation step
    // ///////////////////////////////
}

void btDeformableRigidDynamicsWorld::positionCorrection(btScalar dt)
{
    // perform position correction for all constraints 
    BT_PROFILE("positionCorrection");
    for (int index = 0; index < m_deformableBodySolver->m_objective->projection.m_constraints.size(); ++index)
    {
        btAlignedObjectArray<DeformableFrictionConstraint>& frictions = *m_deformableBodySolver->m_objective->projection.m_frictions[m_deformableBodySolver->m_objective->projection.m_constraints.getKeyAtIndex(index)];
        btAlignedObjectArray<DeformableContactConstraint>& constraints = *m_deformableBodySolver->m_objective->projection.m_constraints.getAtIndex(index);
        for (int i = 0; i < constraints.size(); ++i)
        {
            DeformableContactConstraint& constraint = constraints[i];
            DeformableFrictionConstraint& friction = frictions[i];
            for (int j = 0; j < constraint.m_contact.size(); ++j)
            {
                const btSoftBody::RContact* c = constraint.m_contact[j];
                // skip anchor points
                if (c == NULL || c->m_node->m_im == 0)
                    continue;
                const btSoftBody::sCti& cti = c->m_cti;
                btVector3 va(0, 0, 0);
                
                // grab the velocity of the rigid body
                if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
                {
                    btRigidBody* rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
                    va = rigidCol ? (rigidCol->getVelocityInLocalPoint(c->m_c1)): btVector3(0, 0, 0);
                }
                else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
                {
                    btMultiBodyLinkCollider* multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
                    if (multibodyLinkCol)
                    {
                        const int ndof = multibodyLinkCol->m_multiBody->getNumDofs() + 6;
                        const btScalar* J_n = &c->jacobianData_normal.m_jacobians[0];
                        const btScalar* J_t1 = &c->jacobianData_t1.m_jacobians[0];
                        const btScalar* J_t2 = &c->jacobianData_t2.m_jacobians[0];
                        const btScalar* local_v = multibodyLinkCol->m_multiBody->getVelocityVector();
                        // add in the normal component of the va
                        btScalar vel = 0.0;
                        for (int k = 0; k < ndof; ++k)
                        {
                            vel += local_v[k] * J_n[k];
                        }
                        va = cti.m_normal * vel;
                        
                        vel = 0.0;
                        for (int k = 0; k < ndof; ++k)
                        {
                            vel += local_v[k] * J_t1[k];
                        }
                        va += c->t1 * vel;
                        vel = 0.0;
                        for (int k = 0; k < ndof; ++k)
                        {
                            vel += local_v[k] * J_t2[k];
                        }
                        va += c->t2 * vel;
                    }
                }
                else
                {
                    // The object interacting with deformable node is not supported for position correction
                    btAssert(false);
                }
                
                if (cti.m_colObj->hasContactResponse())
                {
                    btScalar dp = cti.m_offset;
                    
                    // only perform position correction when penetrating
                    if (dp < 0)
                    {
                        if (friction.m_static[j] == true)
                        {
                            c->m_node->m_v = va;
                        }
                        c->m_node->m_v -= dp * cti.m_normal / dt;
                    }
                }
            }
        }
    }
}


void btDeformableRigidDynamicsWorld::integrateTransforms(btScalar dt)
{
    BT_PROFILE("integrateTransforms");
    m_deformableBodySolver->backupVelocity();
    positionCorrection(dt);
    btMultiBodyDynamicsWorld::integrateTransforms(dt);
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            btSoftBody::Node& node = psb->m_nodes[j];
            node.m_x  =  node.m_q + dt * node.m_v;
        }
    }
    m_deformableBodySolver->revertVelocity();
}

void btDeformableRigidDynamicsWorld::solveDeformableBodiesConstraints(btScalar timeStep)
{
    m_deformableBodySolver->solveConstraints(timeStep);
}

void btDeformableRigidDynamicsWorld::addSoftBody(btSoftBody* body, int collisionFilterGroup, int collisionFilterMask)
{
    m_softBodies.push_back(body);
    
    // Set the soft body solver that will deal with this body
    // to be the world's solver
    body->setSoftBodySolver(m_deformableBodySolver);
    
    btCollisionWorld::addCollisionObject(body,
                                         collisionFilterGroup,
                                         collisionFilterMask);
}

void btDeformableRigidDynamicsWorld::predictUnconstraintMotion(btScalar timeStep)
{
    BT_PROFILE("predictUnconstraintMotion");
    btMultiBodyDynamicsWorld::predictUnconstraintMotion(timeStep);
    m_deformableBodySolver->predictMotion(timeStep);
}

void btDeformableRigidDynamicsWorld::reinitialize(btScalar timeStep)
{
    m_internalTime += timeStep;
    m_deformableBodySolver->reinitialize(m_softBodies, timeStep);
    btDispatcherInfo& dispatchInfo = btMultiBodyDynamicsWorld::getDispatchInfo();
    dispatchInfo.m_timeStep = timeStep;
    dispatchInfo.m_stepCount = 0;
    dispatchInfo.m_debugDraw = btMultiBodyDynamicsWorld::getDebugDrawer();
    btMultiBodyDynamicsWorld::getSolverInfo().m_timeStep = timeStep;
}

void btDeformableRigidDynamicsWorld::applyRigidBodyGravity(btScalar timeStep)
{
    // Gravity is applied in stepSimulation and then cleared here and then applied here and then cleared here again
    // so that 1) gravity is applied to velocity before constraint solve and 2) gravity is applied in each substep
    // when there are multiple substeps
    clearForces();
    clearMultiBodyForces();
    btMultiBodyDynamicsWorld::applyGravity();
    // integrate rigid body gravity
    for (int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
    {
        btRigidBody* rb = m_nonStaticRigidBodies[i];
        rb->integrateVelocities(timeStep);
    }
    // integrate multibody gravity
    btMultiBodyDynamicsWorld::solveExternalForces(btMultiBodyDynamicsWorld::getSolverInfo());
    clearForces();
    clearMultiBodyForces();
}

void btDeformableRigidDynamicsWorld::beforeSolverCallbacks(btScalar timeStep)
{
    if (0 != m_internalTickCallback)
    {
        (*m_internalTickCallback)(this, timeStep);
    }
    
    if (0 != m_solverCallback)
    {
        (*m_solverCallback)(m_internalTime, this);
    }
}

void btDeformableRigidDynamicsWorld::afterSolverCallbacks(btScalar timeStep)
{
    if (0 != m_solverCallback)
    {
        (*m_solverCallback)(m_internalTime, this);
    }
}

void btDeformableRigidDynamicsWorld::addForce(btSoftBody* psb, btDeformableLagrangianForce* force)
{
    btAlignedObjectArray<btDeformableLagrangianForce*>& forces = m_deformableBodySolver->m_objective->m_lf;
    bool added = false;
    for (int i = 0; i < forces.size(); ++i)
    {
        if (forces[i]->getForceType() == force->getForceType())
        {
            forces[i]->addSoftBody(psb);
            added = true;
            break;
        }
    }
    if (!added)
    {
        force->addSoftBody(psb);
        force->setIndices(m_deformableBodySolver->m_objective->getIndices());
        forces.push_back(force);
    }
}
