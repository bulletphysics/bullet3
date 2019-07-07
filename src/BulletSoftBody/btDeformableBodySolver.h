//
//  btDeformableBodySolver.h
//  BulletSoftBody
//
//  Created by Chuyuan Fu on 7/1/19.
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
    btBackwardEulerObjective m_objective;
    int m_solveIterations;
    int m_impulseIterations;
    btDeformableRigidDynamicsWorld* m_world;
    btAlignedObjectArray<btVector3> m_backupVelocity;
    
public:
    btDeformableBodySolver()
    : m_numNodes(0)
    , m_objective(m_softBodySet, m_backupVelocity)
    , m_solveIterations(1)
    , m_impulseIterations(1)
    , m_world(nullptr)
    {
    }
    
    virtual ~btDeformableBodySolver()
    {
    }
    
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

    virtual void solveConstraints(float solverdt)
    {
        bool nodeUpdated = updateNodes();
        reinitialize(nodeUpdated);
        backupVelocity();
        for (int i = 0; i < m_solveIterations; ++i)
        {
            // only need to advect x here if elastic force is implicit
//            prepareSolve(solverdt);
            m_objective.computeResidual(solverdt, m_residual);
            moveTempVelocity(solverdt, m_residual);
            m_objective.computeStep(m_dv, m_residual, solverdt);
            
            updateVelocity();
        }
        advect(solverdt);
    }
    
    void moveTempVelocity(btScalar dt, const TVStack& f)
    {
        size_t counter = 0;
        for (int i = 0; i < m_softBodySet.size(); ++i)
        {
            btSoftBody* psb = m_softBodySet[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                auto& node = psb->m_nodes[j];
                node.m_v += node.m_im * dt * f[counter++];
            }
        }
    }
    
    void reinitialize(bool nodeUpdated)
    {
        if (nodeUpdated)
        {
            m_dv.resize(m_numNodes);
            m_residual.resize(m_numNodes);
            m_backupVelocity.resize(m_numNodes);
        }
        
        for (int i = 0; i < m_dv.size(); ++i)
        {
            m_dv[i].setZero();
            m_residual[i].setZero();
        }
        m_objective.reinitialize(nodeUpdated);
    }
    
    void prepareSolve(btScalar dt)
    {
        for (int i = 0; i < m_softBodySet.size(); ++i)
        {
            btSoftBody* psb = m_softBodySet[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                auto& node = psb->m_nodes[j];
                node.m_x = node.m_q + dt * node.m_v;
            }
        }
    }
    void advect(btScalar dt)
    {
        size_t counter = 0;
        for (int i = 0; i < m_softBodySet.size(); ++i)
        {
            btSoftBody* psb = m_softBodySet[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                auto& node = psb->m_nodes[j];
//                node.m_x +=  dt * m_dv[counter++];
                node.m_x +=  dt * node.m_v;
                if (j == 4)
                {
                    std::cout << "x  " << psb->m_nodes[j].m_x.getY() << std::endl;
                    std::cout << "v  " << psb->m_nodes[j].m_v.getY() << std::endl;
                }
            }
        }
    }
    
    void updateVelocity()
    {
        // serial implementation
        int counter = 0;
        for (int i = 0; i < m_softBodySet.size(); ++i)
        {
            btSoftBody* psb = m_softBodySet[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                psb->m_nodes[j].m_v = m_backupVelocity[counter] + m_dv[counter];
                
                ++counter;
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
    
    bool updateNodes()
    {
        int numNodes = 0;
        for (int i = 0; i < m_softBodySet.size(); ++i)
            numNodes += m_softBodySet[i]->m_nodes.size();
        if (numNodes != m_numNodes)
        {
            m_numNodes = numNodes;
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

    virtual void processCollision(btSoftBody *, btSoftBody *) {
        // TODO
    }
    
    virtual void setWorld(btDeformableRigidDynamicsWorld* world)
    {
        m_world = world;
        m_objective.setWorld(world);
    }
};

#endif /* btDeformableBodySolver_h */
