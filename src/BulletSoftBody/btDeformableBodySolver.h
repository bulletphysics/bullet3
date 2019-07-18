//
//  btDeformableBodySolver.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/1/19.
//

#ifndef BT_DEFORMABLE_BODY_SOLVERS_H
#define BT_DEFORMABLE_BODY_SOLVERS_H

#include <iostream>
#include "btSoftBodySolvers.h"
#include "btBackwardEulerObjective.h"
#include "btDeformableRigidDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"

struct btCollisionObjectWrapper;
class btBackwardEulerObjective;
class btDeformableRigidDynamicsWorld;

class btDeformableBodySolver : public btSoftBodySolver
{
    using TVStack = btAlignedObjectArray<btVector3>;
protected:
    /** Variable to define whether we need to update solver constants on the next iteration */
    bool m_updateSolverConstants;
    int m_numNodes;
    TVStack m_dv;
    TVStack m_residual;
    btAlignedObjectArray<btSoftBody *> m_softBodySet;
    btBackwardEulerObjective* m_objective;
    int m_solveIterations;
    int m_impulseIterations;
    btDeformableRigidDynamicsWorld* m_world;
    btAlignedObjectArray<btVector3> m_backupVelocity;
    btScalar m_dt;
    
public:
    btDeformableBodySolver();
    
    virtual ~btDeformableBodySolver();
    
    virtual SolverTypes getSolverType() const
    {
        return DEFORMABLE_SOLVER;
    }
    
    virtual bool checkInitialized()
    {
        return true;
    }

    virtual void updateSoftBodies();
    
    virtual void optimize(btAlignedObjectArray<btSoftBody *> &softBodies, bool forceUpdate = false)
    {
        m_softBodySet.copyFromArray(softBodies);
    }

    virtual void copyBackToSoftBodies(bool bMove = true) {}

    virtual void solveConstraints(float solverdt);
    
    void postStabilize();
    
    void reinitialize(bool nodeUpdated);
    
    void setConstraints();
    
    void advect(btScalar dt);
    
    void backupVelocity();
    
    void updateVelocity();
    
    bool updateNodes();
    
    virtual void predictMotion(float solverdt);

    virtual void copySoftBodyToVertexBuffer(const btSoftBody *const softBody, btVertexBufferDescriptor *vertexBuffer) {}

    virtual void processCollision(btSoftBody * softBody, const btCollisionObjectWrapper * collisionObjectWrap)
    {
        softBody->defaultCollisionHandler(collisionObjectWrap);
    }

    virtual void processCollision(btSoftBody * softBody, btSoftBody * otherSoftBody) {
        softBody->defaultCollisionHandler(otherSoftBody);
    }
    
    virtual void setWorld(btDeformableRigidDynamicsWorld* world);
};

#endif /* btDeformableBodySolver_h */
