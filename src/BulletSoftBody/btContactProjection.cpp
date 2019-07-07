//
//  btContactProjection.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/4/19.
//

#include "btContactProjection.h"
#include "btDeformableRigidDynamicsWorld.h"
void btContactProjection::update(btScalar dt, const TVStack& dv)
{
    size_t counter = 0;
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            psb->m_nodes[j].m_v = m_backupVelocity[counter] + dv[counter];
            ++counter;
        }
    }
        
    ///solve rigid body constraints
    m_world->btSoftRigidDynamicsWorld::btDiscreteDynamicsWorld::solveConstraints(m_world->getSolverInfo());
    
    // clear the old constraints
    for (int i = 0; i < m_constrainedDirections.size(); ++i)
    {
        m_constrainedDirections[i].clear();
        m_constrainedValues[i].clear();
    }
    
    // Set dirichlet constraints
    counter = 0;
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
    
    // loop through contacts to create contact constraints
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        btMultiBodyJacobianData jacobianData;
        const btScalar mrg = psb->getCollisionShape()->getMargin();
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
                    va = rigidCol ? (rigidCol->getVelocityInLocalPoint(c.m_c1) + btVector3(0,-10,0)*dt) * dt : btVector3(0, 0, 0);
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
                
                // TODO: rethink what the velocity of the soft body node should be
                //                    const btVector3 vb = c.m_node->m_x - c.m_node->m_q;
                const btVector3 vb = c.m_node->m_v * dt;
                const btVector3 vr = vb - va;
                const btScalar dn = btDot(vr, cti.m_normal);
                if (dn <= SIMD_EPSILON)
                {
                    const btScalar dp = btMin((btDot(c.m_node->m_x, cti.m_normal) + cti.m_offset), mrg);
                    const btVector3 fv = vr - (cti.m_normal * dn);
                    // c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
//                    const btVector3 impulse = c.m_c0 * ((vr - (fv * c.m_c3)));
                    const btVector3 impulse = c.m_c0 * ((vr - (fv * c.m_c3))+ (cti.m_normal * (dp * c.m_c4)));
                    
                    //c.m_node->m_v -= impulse * c.m_c2 / dt;
                    // TODO: only contact is considered here, add friction later
                    btVector3 normal = cti.m_normal.normalized();
                    btVector3 diff = c.m_node->m_v - m_backupVelocity[m_indices[c.m_node]];
                    btVector3 dv = -impulse * c.m_c2/dt + diff;
                    btScalar dvn = dv.dot(normal);
                    m_constrainedDirections[m_indices[c.m_node]].push_back(normal);
                    m_constrainedValues[m_indices[c.m_node]].push_back(dvn);
                    
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
