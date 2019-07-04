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
    
    btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::internalSingleStepSimulation(timeStep);
    
    ///solve deformable bodies constraints
    solveDeformableBodiesConstraints(timeStep);

    
    ///update soft bodies
    m_deformableBodySolver->updateSoftBodies();
    
    // End solver-wise simulation step
    // ///////////////////////////////
}

void btDeformableRigidDynamicsWorld::solveDeformableBodiesConstraints(btScalar timeStep)
{
    m_deformableBodySolver->solveConstraints(timeStep);
}

