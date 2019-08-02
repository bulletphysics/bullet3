//
//  btDeformableRigidDynamicsWorld.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/1/19.
//

#include <stdio.h>
#include "btDeformableRigidDynamicsWorld.h"
#include "btDeformableBodySolver.h"


void btDeformableRigidDynamicsWorld::internalSingleStepSimulation(btScalar timeStep)
{
    reinitialize(timeStep);
//    beforeSolverCallbacks(timeStep);
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
    for (auto& it : m_deformableBodySolver->m_objective->projection.m_constraints)
    {
        btAlignedObjectArray<DeformableFrictionConstraint>& frictions = m_deformableBodySolver->m_objective->projection.m_frictions[it.first];
        btAlignedObjectArray<DeformableContactConstraint>& constraints = it.second;
        for (int i = 0; i < constraints.size(); ++i)
        {
            DeformableContactConstraint& constraint = constraints[i];
            DeformableFrictionConstraint& friction = frictions[i];
            for (int j = 0; j < constraint.m_contact.size(); ++j)
            {
                const btSoftBody::RContact* c = constraint.m_contact[j];
                // skip anchor points
                if (c == nullptr || c->m_node->m_im == 0)
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
    btMultiBodyDynamicsWorld::predictUnconstraintMotion(timeStep);
    m_deformableBodySolver->predictMotion(float(timeStep));
}

void btDeformableRigidDynamicsWorld::reinitialize(btScalar timeStep)
{
    m_internalTime += timeStep;
    m_deformableBodySolver->reinitialize(m_softBodies);
    btDispatcherInfo& dispatchInfo = btMultiBodyDynamicsWorld::getDispatchInfo();
    dispatchInfo.m_timeStep = timeStep;
    dispatchInfo.m_stepCount = 0;
    dispatchInfo.m_debugDraw = btMultiBodyDynamicsWorld::getDebugDrawer();
    btMultiBodyDynamicsWorld::getSolverInfo().m_timeStep = timeStep;
}

void btDeformableRigidDynamicsWorld::applyRigidBodyGravity(btScalar timeStep)
{
    // TODO: This is an ugly hack to get the desired gravity behavior.
    // gravity is applied in stepSimulation and then cleared here and then applied here and then cleared here again
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
    for (int i = 0; i < m_beforeSolverCallbacks.size(); ++i)
        m_beforeSolverCallbacks[i](m_internalTime, this);
}

void btDeformableRigidDynamicsWorld::afterSolverCallbacks(btScalar timeStep)
{
    for (int i = 0; i < m_beforeSolverCallbacks.size(); ++i)
        m_beforeSolverCallbacks[i](m_internalTime, this);
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
