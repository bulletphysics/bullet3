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

    virtual void updateSoftBodies()
    {
        for (int i = 0; i < m_softBodySet.size(); i++)
        {
            btSoftBody *psb = (btSoftBody *)m_softBodySet[i];
            if (psb->isActive())
            {
                psb->integrateMotion(); // normal is updated here
            }
        }
    }
    
    virtual void optimize(btAlignedObjectArray<btSoftBody *> &softBodies, bool forceUpdate = false)
    {
        m_softBodySet.copyFromArray(softBodies);
    }

    virtual void copyBackToSoftBodies(bool bMove = true) {}

    virtual void solveConstraints(float solverdt);
    
    void postStabilize();
    
    void reinitialize(bool nodeUpdated);
    
    void setConstraintDirections();
    
    void advect(btScalar dt)
    {
        size_t counter = 0;
        for (int i = 0; i < m_softBodySet.size(); ++i)
        {
            btSoftBody* psb = m_softBodySet[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                auto& node = psb->m_nodes[j];
                node.m_x  =  node.m_q + dt * node.m_v;
            }
        }
    }
    
    void backupVelocity()
    {
        // serial implementation
        int counter = 0;
        for (int i = 0; i < m_softBodySet.size(); ++i)
        {
            btSoftBody* psb = m_softBodySet[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                m_backupVelocity[counter++] = psb->m_nodes[j].m_v;
            }
        }
    }
    
    void updateVelocity();
    
    bool updateNodes()
    {
        int numNodes = 0;
        for (int i = 0; i < m_softBodySet.size(); ++i)
            numNodes += m_softBodySet[i]->m_nodes.size();
        if (numNodes != m_numNodes)
        {
            m_numNodes = numNodes;
            m_backupVelocity.resize(numNodes);
            return true;
        }
        return false;
    }
    
    virtual void predictMotion(float solverdt)
    {
        for (int i = 0; i < m_softBodySet.size(); ++i)
        {
            btSoftBody *psb = m_softBodySet[i];
            
            if (psb->isActive())
            {
                psb->predictMotion(solverdt);
            }
        }
    }

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
