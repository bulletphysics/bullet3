//
//  btContactProjection.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/4/19.
//

#include "btContactProjection.h"
#include "btDeformableRigidDynamicsWorld.h"
void btContactProjection::update(const TVStack& dv, const TVStack& backupVelocity)
{
    ///solve rigid body constraints
    m_world->btMultiBodyDynamicsWorld::solveConstraints(m_world->getSolverInfo());

    // loop through contacts to create contact constraints
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        btMultiBodyJacobianData jacobianData;
        for (int i = 0, ni = psb->m_rcontacts.size(); i < ni; ++i)
        {
            const btSoftBody::RContact& c = psb->m_rcontacts[i];
            
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
                if (1) // in the same CG solve, the set of constraits doesn't change
//                if (dn <= SIMD_EPSILON)
                {
                    // c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
                    const btVector3 impulse = c.m_c0 *(cti.m_normal * dn);
                    // TODO: only contact is considered here, add friction later
                    
                    // dv = new_impulse + accumulated velocity change in previous CG iterations
                    // so we have the invariant node->m_v = backupVelocity + dv;
                    btVector3 dv = -impulse * c.m_c2/m_dt + c.m_node->m_v - backupVelocity[m_indices[c.m_node]];
                    btScalar dvn = dv.dot(cti.m_normal);
                    m_constrainedValues[m_indices[c.m_node]][0]=(dvn);
                    
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

void btContactProjection::setConstraintDirections()
{
    // set Dirichlet constraint
    size_t counter = 0;
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        const btSoftBody* psb = m_softBodies[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            if (psb->m_nodes[j].m_im == 0)
            {
                m_constrainedDirections[counter].push_back(btVector3(1,0,0));
                m_constrainedDirections[counter].push_back(btVector3(0,1,0));
                m_constrainedDirections[counter].push_back(btVector3(0,0,1));
                m_constrainedValues[counter].push_back(0);
                m_constrainedValues[counter].push_back(0);
                m_constrainedValues[counter].push_back(0);
            }
            ++counter;
        }
    }

    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        btMultiBodyJacobianData jacobianData;
        
        int j = 0;
        while (j < psb->m_rcontacts.size())
        {
            const btSoftBody::RContact& c = psb->m_rcontacts[j];
            // skip anchor points
            if (c.m_node->m_im == 0)
            {
                psb->m_rcontacts.removeAtIndex(j);
                continue;
            }
            
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
                if (dn < SIMD_EPSILON)
                {
                    ++j;
                    m_constrainedDirections[m_indices[c.m_node]].push_back(cti.m_normal);
                    m_constrainedValues[m_indices[c.m_node]].resize(m_constrainedValues[m_indices[c.m_node]].size()+1);
                    continue;
                }
            }
            psb->m_rcontacts.removeAtIndex(j);
        }
    }
}
