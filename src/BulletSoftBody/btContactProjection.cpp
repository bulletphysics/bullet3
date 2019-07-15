//
//  btContactProjection.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/4/19.
//

#include "btContactProjection.h"
#include "btDeformableRigidDynamicsWorld.h"
#include <algorithm>
void btContactProjection::update(const TVStack& dv, const TVStack& backupVelocity)
{
    ///solve rigid body constraints
    m_world->getSolverInfo().m_numIterations = 5;
    m_world->btMultiBodyDynamicsWorld::solveConstraints(m_world->getSolverInfo());

    // loop through constraints to set constrained values
    for (auto& it : m_constraints)
    {
        Friction& friction = m_frictions[it.first];
        btAlignedObjectArray<Constraint>& constraints = it.second;
        for (int i = 0; i < constraints.size(); ++i)
        {
            Constraint& constraint = constraints[i];
            for (int j = 0; j < constraint.m_contact.size(); ++j)
            {
                if (constraint.m_contact[j] == nullptr)
                {
                    // nothing needs to be done for dirichelet constraints
                    continue;
                }
                const btSoftBody::RContact* c = constraint.m_contact[j];
                const btSoftBody::sCti& cti = c->m_cti;
                btMultiBodyJacobianData jacobianData;
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
                        va = rigidCol ? (rigidCol->getVelocityInLocalPoint(c->m_c1)) * m_dt : btVector3(0, 0, 0);
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
                            
                            multibodyLinkCol->m_multiBody->fillContactJacobianMultiDof(multibodyLinkCol->m_link, c->m_node->m_x, cti.m_normal, jac, jacobianData.scratch_r, jacobianData.scratch_v, jacobianData.scratch_m);
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
                    
                    const btVector3 vb = c->m_node->m_v * m_dt;
                    const btVector3 vr = vb - va;
                    const btScalar dn = btDot(vr, cti.m_normal);
                    btVector3 impulse = c->m_c0 * vr;
                    const btVector3 impulse_normal = c->m_c0 *(cti.m_normal * dn);
                    btVector3 impulse_tangent = impulse - impulse_normal;
                    
                    if (dn < 0 && impulse_tangent.norm() > SIMD_EPSILON)
                    {
                        btScalar impulse_tangent_magnitude = std::min(impulse_normal.norm()*c->m_c3*1000, impulse_tangent.norm());
                        
//                        impulse_tangent_magnitude = 0;
                        
                        const btVector3 tangent_dir = impulse_tangent.normalized();
                        impulse_tangent = impulse_tangent_magnitude * tangent_dir;
                        friction.m_direction = impulse_tangent;
                        friction.m_dv = -impulse_tangent * c->m_c2/m_dt + (c->m_node->m_v - backupVelocity[m_indices[c->m_node]]);
                    }
                    impulse = impulse_normal + impulse_tangent;
                    
    //                if (1) // in the same CG solve, the set of constraits doesn't change
                    if (dn <= SIMD_EPSILON)
                    {
                        // c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
                        
                        // TODO: only contact is considered here, add friction later
                        
                        // dv = new_impulse + accumulated velocity change in previous CG iterations
                        // so we have the invariant node->m_v = backupVelocity + dv;
                        btVector3 dv = -impulse * c->m_c2/m_dt + c->m_node->m_v - backupVelocity[m_indices[c->m_node]];
                        btScalar dvn = dv.dot(cti.m_normal);
                        constraint.m_value[j] = dvn;
                        
                        if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
                        {
                            if (rigidCol)
                                rigidCol->applyImpulse(impulse_normal, c->m_c1);
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
}


void btContactProjection::setConstraintDirections()
{
    // set Dirichlet constraint
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            if (psb->m_nodes[j].m_im == 0)
            {
                btAlignedObjectArray<Constraint> c;
                c.push_back(Constraint(btVector3(1,0,0)));
                c.push_back(Constraint(btVector3(0,1,0)));
                c.push_back(Constraint(btVector3(0,0,1)));
                m_constraints[&(psb->m_nodes[j])] = c;
            }
        }
    }

    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        btMultiBodyJacobianData jacobianData;
        
        for (int j = 0; j < psb->m_rcontacts.size(); ++j)
        {
            const btSoftBody::RContact& c = psb->m_rcontacts[j];
            // skip anchor points
            if (c.m_node->m_im == 0)
            {
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
                    if (m_constraints.find(c.m_node) == m_constraints.end())
                    {
                        btAlignedObjectArray<Constraint> constraints;
                        constraints.push_back(Constraint(c));
                        m_constraints[c.m_node] = constraints;
                        m_frictions[c.m_node] = Friction();
                    }
                    else
                    {
                        // group colinear constraints into one
                        const btScalar angle_epsilon = 0.015192247; // less than 10 degree
                        bool merged = false;
                        btAlignedObjectArray<Constraint>& constraints = m_constraints[c.m_node];
                        for (int j = 0; j < constraints.size(); ++j)
                        {
                            const btAlignedObjectArray<btVector3>& dirs = constraints[j].m_direction;
                            btScalar dot_prod = dirs[0].dot(cti.m_normal);
                            if (std::abs(std::abs(dot_prod) - 1) < angle_epsilon)
                            {
                                constraints[j].m_contact.push_back(&c);
                                constraints[j].m_direction.push_back(cti.m_normal);
                                constraints[j].m_value.push_back(0);
                                merged = true;
                                break;
                            }
                        }
                        const int dim = 3;
                        // hard coded no more than 3 constraint directions
                        if (!merged && constraints.size() < dim)
                            constraints.push_back(Constraint(c));
                    }
                }
            }
        }
    }
}
