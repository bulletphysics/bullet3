//
//  btDeformableRigidDynamicsWorld.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/1/19.
//

#include <stdio.h>
#include "btDeformableRigidDynamicsWorld.h"
#include "btDeformableBodySolver.h"

btDeformableBodySolver::btDeformableBodySolver()
: m_numNodes(0)
, m_solveIterations(1)
, m_impulseIterations(1)
, m_world(nullptr)
{
    m_objective = new btBackwardEulerObjective(m_softBodySet, m_backupVelocity);
}

btDeformableBodySolver::~btDeformableBodySolver()
{
    delete m_objective;
}

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
    
    // incorporate gravity into velocity and clear force
    for (int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
    {
        btRigidBody* rb = m_nonStaticRigidBodies[i];
        rb->integrateVelocities(timeStep);
    }
    clearForces();
    
    ///solve deformable bodies constraints
    solveDeformableBodiesConstraints(timeStep);

    //integrate transforms
    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::integrateTransforms(timeStep);
    
    ///update vehicle simulation
    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::updateActions(timeStep);
    
    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::updateActivationState(timeStep);
    
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
    getSoftDynamicsWorld()->getSoftBodyArray().push_back(body);
    
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


