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
    m_deformableBodySolver->optimize(getSoftDynamicsWorld()->getSoftBodyArray());
    
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
    
    
    btDispatcherInfo& dispatchInfo = btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::getDispatchInfo();
    
    dispatchInfo.m_timeStep = timeStep;
    dispatchInfo.m_stepCount = 0;
    dispatchInfo.m_debugDraw = btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::getDebugDrawer();
    
    // only used in CCD
//    createPredictiveContacts(timeStep);
    
    ///perform collision detection
    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::performDiscreteCollisionDetection();
    
    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::calculateSimulationIslands();
    
    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::getSolverInfo().m_timeStep = timeStep;

    if (0 != m_internalTickCallback)
    {
        (*m_internalTickCallback)(this, timeStep);
    }
    
    
//    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::internalSingleStepSimulation(timeStep);
    
    ///solve deformable bodies constraints
    solveDeformableBodiesConstraints(timeStep);

//    predictUnconstraintMotion(timeStep);
    //integrate transforms
    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::integrateTransforms(timeStep);
    
    ///update vehicle simulation
    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::updateActions(timeStep);
    
    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::updateActivationState(timeStep);
    
    ///update soft bodies
    m_deformableBodySolver->updateSoftBodies();
    
    for (int i = 0; i < m_nonStaticRigidBodies.size(); i++)
    {
        btRigidBody* body = m_nonStaticRigidBodies[i];
        std::cout << "rb v = " << body->getLinearVelocity().getY() << std::endl;
    }
    // End solver-wise simulation step
    // ///////////////////////////////
}

void btDeformableRigidDynamicsWorld::solveDeformableBodiesConstraints(btScalar timeStep)
{
    m_deformableBodySolver->solveConstraints(timeStep);
}

