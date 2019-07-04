//
//  btDeformableBodySolver.h
//  BulletSoftBody
//
//  Created by Chuyuan Fu on 7/1/19.
//

#ifndef BT_DEFORMABLE_BODY_SOLVERS_H
#define BT_DEFORMABLE_BODY_SOLVERS_H

#include "btSoftBodySolvers.h"
#include "btBackwardEulerObjective.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"

struct btCollisionObjectWrapper;

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
    
public:
    btDeformableBodySolver()
    : m_numNodes(0)
    , m_objective(m_softBodySet)
    , m_solveIterations(1)
    , m_impulseIterations(1)
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
        
        for (int i = 0; i < m_solveIterations; ++i)
        {
            // get the velocity after contact solve
            // TODO: perform contact solve here
            for (int j = 0; j < m_impulseIterations; ++j)
            {
                for (int s = 0; s < m_softBodySet.size(); ++s)
                    VSolve_RContacts(m_softBodySet[s], 0, solverdt);
            }
            
            // advect with v_n+1 ** to update position based states
            // where v_n+1 ** is the velocity after contact response
            
            // only need to advect x here if elastic force is implicit
//            prepareSolve(solverdt);
            
            m_objective.computeResidual(solverdt, m_residual);
            m_objective.computeStep(m_dv, m_residual, solverdt);
            
            updateVelocity();
        }
        advect(solverdt);
    }
    
    void reinitialize(bool nodeUpdated)
    {
        if (nodeUpdated)
        {
            m_dv.resize(m_numNodes);
            m_residual.resize(m_numNodes);
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
                node.m_x = node.m_q + dt * node.m_v * psb->m_dampingCoefficient;
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
                node.m_x +=  dt * m_dv[counter++];
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
                psb->m_nodes[j].m_v += m_dv[counter++];
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
    
    void VSolve_RContacts(btSoftBody* psb, btScalar kst, btScalar dt)
    {
        const btScalar mrg = psb->getCollisionShape()->getMargin();
        btMultiBodyJacobianData jacobianData;
        for (int i = 0, ni = psb->m_rcontacts.size(); i < ni; ++i)
        {
            const btSoftBody::RContact& c = psb->m_rcontacts[i];
            const btSoftBody::sCti& cti = c.m_cti;
            if (cti.m_colObj->hasContactResponse())
            {
                btVector3 va(0, 0, 0);
                btRigidBody* rigidCol = 0;
                btMultiBodyLinkCollider* multibodyLinkCol = 0;
                btScalar* deltaV;
                
                if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
                {
                    rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
                    va = rigidCol ? rigidCol->getVelocityInLocalPoint(c.m_c1) * dt : btVector3(0, 0, 0);
                }
                else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
                {
                    multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
                    if (multibodyLinkCol)
                    {
                        const int ndof = multibodyLinkCol->m_multiBody->getNumDofs() + 6;
                        jacobianData.m_jacobians.resize(ndof);
                        jacobianData.m_deltaVelocitiesUnitImpulse.resize(ndof);
                        btScalar* jac = &jacobianData.m_jacobians[0];
                        
                        multibodyLinkCol->m_multiBody->fillContactJacobianMultiDof(multibodyLinkCol->m_link, c.m_node->m_x, cti.m_normal, jac, jacobianData.scratch_r, jacobianData.scratch_v, jacobianData.scratch_m);
                        deltaV = &jacobianData.m_deltaVelocitiesUnitImpulse[0];
                        multibodyLinkCol->m_multiBody->calcAccelerationDeltasMultiDof(&jacobianData.m_jacobians[0], deltaV, jacobianData.scratch_r, jacobianData.scratch_v);
                        
                        btScalar vel = 0.0;
                        for (int j = 0; j < ndof; ++j)
                        {
                            vel += multibodyLinkCol->m_multiBody->getVelocityVector()[j] * jac[j];
                        }
                        va = cti.m_normal * vel * dt;
                    }
                }
                
                const btVector3 vb = c.m_node->m_x - c.m_node->m_q;
                const btVector3 vr = vb - va;
                const btScalar dn = btDot(vr, cti.m_normal);
                if (dn <= SIMD_EPSILON)
                {
                    const btScalar dp = btMin((btDot(c.m_node->m_x, cti.m_normal) + cti.m_offset), mrg);
                    const btVector3 fv = vr - (cti.m_normal * dn);
                    // c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
                    const btVector3 impulse = c.m_c0 * ((vr - (fv * c.m_c3) + (cti.m_normal * (dp * c.m_c4))) * kst);
                    c.m_node->m_v -= impulse * c.m_c2 / dt;
                    
                    if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
                    {
                        if (rigidCol)
                            rigidCol->applyImpulse(impulse, c.m_c1);
                    }
                    else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
                    {
                        if (multibodyLinkCol)
                        {
                            double multiplier = 0.5;
                            multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof(deltaV, -impulse.length() * multiplier);
                        }
                    }
                }
            }
        }
    }

};

#endif /* btDeformableBodySolver_h */
