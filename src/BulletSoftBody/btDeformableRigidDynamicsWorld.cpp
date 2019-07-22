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
    
    positionCorrection();
    
    integrateTransforms(timeStep);
    
    ///update vehicle simulation
    btMultiBodyDynamicsWorld::updateActions(timeStep);
    
    btMultiBodyDynamicsWorld::updateActivationState(timeStep);
    // End solver-wise simulation step
    // ///////////////////////////////
}

void btDeformableRigidDynamicsWorld::positionCorrection()
{
    // perform position correction for all geometric collisions
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        const btScalar mrg = psb->getCollisionShape()->getMargin();
        for (int j = 0; j < psb->m_rcontacts.size(); ++j)
        {
            const btSoftBody::RContact& c = psb->m_rcontacts[j];
            // skip anchor points
            if (c.m_node->m_im == 0)
                continue;
            
            const btSoftBody::sCti& cti = c.m_cti;
            if (cti.m_colObj->hasContactResponse())
            {
                btScalar dp = btMin((btDot(c.m_node->m_x, cti.m_normal) + cti.m_offset), mrg);
                if (dp < 0)
                {
                    // m_c4 is the collision hardness
                    c.m_node->m_q -= dp * cti.m_normal * c.m_c4;
                }
            }
        }
    }
}

void btDeformableRigidDynamicsWorld::integrateTransforms(btScalar dt)
{
    btMultiBodyDynamicsWorld::integrateTransforms(dt);
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            auto& node = psb->m_nodes[j];
            node.m_x  =  node.m_q + dt * node.m_v;
        }
    }
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
    btDiscreteDynamicsWorld::predictUnconstraintMotion(timeStep);
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
