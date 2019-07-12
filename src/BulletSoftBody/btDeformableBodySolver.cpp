//
//  btDeformableBodySolver.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/9/19.
//

#include <stdio.h>
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

void btDeformableBodySolver::postStabilize()
{
    for (int i = 0; i < m_softBodySet.size(); ++i)
    {
        btSoftBody* psb = m_softBodySet[i];
        btMultiBodyJacobianData jacobianData;
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
                btVector3 va(0, 0, 0);
                btRigidBody* rigidCol = 0;
                btMultiBodyLinkCollider* multibodyLinkCol = 0;
                btScalar* deltaV;
                
                // grab the velocity of the rigid body
                if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
                {
                    rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
                    va = rigidCol ? (rigidCol->getVelocityInLocalPoint(c.m_c1)) * m_dt : btVector3(0, 0, 0);
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
                        va = cti.m_normal * vel * m_dt;
                    }
                }
                
                const btVector3 vb = c.m_node->m_v * m_dt;
                const btVector3 vr = vb - va;
                const btScalar dn = btDot(vr, cti.m_normal);
                
                const btScalar dp = btMin((btDot(c.m_node->m_x, cti.m_normal) + cti.m_offset), mrg);
                
                // c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
                
                btScalar dvn = dn * c.m_c4;
                const btVector3 impulse = c.m_c0 * ((cti.m_normal * (dn * c.m_c4)));
                // TODO: only contact is considered here, add friction later
                if (dp < 0)
                {
                    c.m_node->m_x -= dp * cti.m_normal * c.m_c4;
//                    c.m_node->m_x -= impulse * c.m_c2;
                    
                    ////
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
}

void btDeformableBodySolver::solveConstraints(float solverdt)
{
    m_dt = solverdt;
    bool nodeUpdated = updateNodes();
    reinitialize(nodeUpdated);
    backupVelocity();
    postStabilize();
    for (int i = 0; i < m_solveIterations; ++i)
    {
        m_objective->computeResidual(solverdt, m_residual);
        m_objective->computeStep(m_dv, m_residual, solverdt);
        updateVelocity();
    }
    advect(solverdt);
//    postStabilize();
}

void btDeformableBodySolver::reinitialize(bool nodeUpdated)
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
    m_objective->reinitialize(nodeUpdated);
    
    // remove contact constraints with separating velocity
    setConstraintDirections();
}

void btDeformableBodySolver::setConstraintDirections()
{
    m_objective->setConstraintDirections();
}

void btDeformableBodySolver::setWorld(btDeformableRigidDynamicsWorld* world)
{
    m_world = world;
    m_objective->setWorld(world);
}

void btDeformableBodySolver::updateVelocity()
{
    // serial implementation
    int counter = 0;
    for (int i = 0; i < m_softBodySet.size(); ++i)
    {
        btSoftBody* psb = m_softBodySet[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
//            psb->m_nodes[j].m_v += m_dv[counter];
            psb->m_nodes[j].m_v = m_backupVelocity[counter]+m_dv[counter];
            ++counter;
        }
    }
}
