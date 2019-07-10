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
    // Let the solver grab the soft bodies and if necessary optimize for it
    m_deformableBodySolver->optimize(m_softBodies);
    
    if (!m_deformableBodySolver->checkInitialized())
    {
        btAssert("Solver initialization failed\n");
    }

    // from btDiscreteDynamicsWorld singleStepSimulation
    if (0 != m_internalPreTickCallback)
    {
        (*m_internalPreTickCallback)(this, timeStep);
    }
    
    ///apply gravity, predict motion
    predictUnconstraintMotion(timeStep);
    
    btDispatcherInfo& dispatchInfo = btMultiBodyDynamicsWorld::getDispatchInfo();
    
    dispatchInfo.m_timeStep = timeStep;
    dispatchInfo.m_stepCount = 0;
    dispatchInfo.m_debugDraw = btMultiBodyDynamicsWorld::getDebugDrawer();
    
    // only used in CCD
//    createPredictiveContacts(timeStep);
    
    ///perform collision detection
    btMultiBodyDynamicsWorld::performDiscreteCollisionDetection();
    
    btMultiBodyDynamicsWorld::calculateSimulationIslands();
    
    btMultiBodyDynamicsWorld::getSolverInfo().m_timeStep = timeStep;

    if (0 != m_internalTickCallback)
    {
        (*m_internalTickCallback)(this, timeStep);
    }
    
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
    
    ///solve deformable bodies constraints
    solveDeformableBodiesConstraints(timeStep);

    //integrate transforms
    btMultiBodyDynamicsWorld::integrateTransforms(timeStep);
    
    ///update vehicle simulation
    btMultiBodyDynamicsWorld::updateActions(timeStep);
    
    btMultiBodyDynamicsWorld::updateActivationState(timeStep);
    
    ///update soft bodies
    m_deformableBodySolver->updateSoftBodies();

    clearForces();
    // End solver-wise simulation step
    // ///////////////////////////////
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


