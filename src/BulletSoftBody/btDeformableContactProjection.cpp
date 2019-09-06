/*
 Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>
 
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#include "btDeformableContactProjection.h"
#include "btDeformableMultiBodyDynamicsWorld.h"
#include <algorithm>
#include <cmath>
btScalar btDeformableContactProjection::update()
{
    btScalar residualSquare = 0;
    btScalar max_impulse = 0;
    // loop through constraints to set constrained values
    for (int index = 0; index < m_constraints.size(); ++index)
    {
        DeformableContactConstraint& constraint = *m_constraints.getAtIndex(index);
        const btSoftBody::Node* node = constraint.m_node;
        for (int j = 0; j < constraint.m_contact.size(); ++j)
        {
            if (constraint.m_contact[j] == NULL)
            {
                // nothing needs to be done for dirichelet constraints
                continue;
            }
            const btSoftBody::RContact* c = constraint.m_contact[j];
            const btSoftBody::sCti& cti = c->m_cti;
            
            if (cti.m_colObj->hasContactResponse())
            {
                btVector3 va(0, 0, 0);
                btRigidBody* rigidCol = 0;
                btMultiBodyLinkCollider* multibodyLinkCol = 0;
                const btScalar* deltaV_normal;
                
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
                        const btScalar* J_n = &c->jacobianData_normal.m_jacobians[0];
                        const btScalar* J_t1 = &c->jacobianData_t1.m_jacobians[0];
                        const btScalar* J_t2 = &c->jacobianData_t2.m_jacobians[0];
                        const btScalar* local_v = multibodyLinkCol->m_multiBody->getVelocityVector();
                        const btScalar* local_dv = multibodyLinkCol->m_multiBody->getDeltaVelocityVector();
                        deltaV_normal = &c->jacobianData_normal.m_deltaVelocitiesUnitImpulse[0];
                        // add in the normal component of the va
                        btScalar vel = 0.0;
                        for (int k = 0; k < ndof; ++k)
                        {
                            vel += (local_v[k]+local_dv[k]) * J_n[k];
                        }
                        va = cti.m_normal * vel * m_dt;
                        // add in the tangential components of the va
                        vel = 0.0;
                        for (int k = 0; k < ndof; ++k)
                        {
                            vel += (local_v[k]+local_dv[k]) * J_t1[k];
                        }
                        va += c->t1 * vel * m_dt;
                        vel = 0.0;
                        for (int k = 0; k < ndof; ++k)
                        {
                            vel += (local_v[k]+local_dv[k]) * J_t2[k];
                        }
                        va += c->t2 * vel * m_dt;
                    }
                }
                
                const btVector3 vb = c->m_node->m_v * m_dt;
                const btVector3 vr = vb - va;
                const btScalar dn = btDot(vr, cti.m_normal);
                btVector3 impulse = c->m_c0 * vr;
                const btVector3 impulse_normal = c->m_c0 * (cti.m_normal * dn);
                btVector3 impulse_tangent = impulse - impulse_normal;
                
                btVector3 old_total_tangent_dv = constraint.m_total_tangent_dv[j];
                constraint.m_total_normal_dv[j] -= impulse_normal * node->m_im;
                constraint.m_total_tangent_dv[j] -= impulse_tangent * node->m_im;
                
                if (constraint.m_total_normal_dv[j].dot(cti.m_normal) < 0)
                {
                    // separating in the normal direction
                    constraint.m_static[j] = false;
                    constraint.m_can_be_dynamic[j] = false;
                    constraint.m_total_tangent_dv[j] = btVector3(0,0,0);
                    impulse_tangent.setZero();
                }
                else
                {
                    if (constraint.m_can_be_dynamic[j] && constraint.m_total_normal_dv[j].norm() * c->m_c3 < constraint.m_total_tangent_dv[j].norm())
                    {
                        // dynamic friction
                        // with dynamic friction, the impulse are still applied to the two objects colliding, however, it does not pose a constraint in the cg solve, hence the change to dv merely serves to update velocity in the contact iterations.
                        constraint.m_static[j] = false;
                        constraint.m_can_be_dynamic[j] = true;
                        if (constraint.m_total_tangent_dv[j].norm() < SIMD_EPSILON)
                        {
                            constraint.m_total_tangent_dv[j] = btVector3(0,0,0);
                        }
                        else
                        {
                            constraint.m_total_tangent_dv[j] = constraint.m_total_tangent_dv[j].normalized() * constraint.m_total_normal_dv[j].norm() * c->m_c3;
                        }
                        impulse_tangent = -btScalar(1)/node->m_im * (constraint.m_total_tangent_dv[j] - old_total_tangent_dv);
                    }
                    else
                    {
                        // static friction
                        constraint.m_static[j] = true;
                        constraint.m_can_be_dynamic[j] = false;
                    }
                }
                impulse = impulse_normal + impulse_tangent;
                max_impulse = btMax(impulse.length2(), max_impulse);
                
                // dn is the normal component of velocity diffrerence. Approximates the residual.
                residualSquare = btMax(residualSquare, dn*dn);
                if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
                {
                    if (rigidCol)
                    {
                        rigidCol->applyImpulse(impulse, c->m_c1);
                    }
                }
                else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
                {
                    if (multibodyLinkCol)
                    {
                        // apply normal component of the impulse
                        multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_normal, impulse.dot(cti.m_normal));
                        if (impulse_tangent.norm() > SIMD_EPSILON)
                        {
                            // apply tangential component of the impulse
                            const btScalar* deltaV_t1 = &c->jacobianData_t1.m_deltaVelocitiesUnitImpulse[0];
                            multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t1, impulse.dot(c->t1));
                            const btScalar* deltaV_t2 = &c->jacobianData_t2.m_deltaVelocitiesUnitImpulse[0];
                            multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t2, impulse.dot(c->t2));
                        }
                    }
                }
            }
        }
    }
    return residualSquare;
}

void btDeformableContactProjection::setConstraints()
{
    BT_PROFILE("setConstraints");
    // set Dirichlet constraint
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            if (psb->m_nodes[j].m_im == 0)
            {
                m_constraints.insert(psb->m_nodes[j].index, DeformableContactConstraint());
            }
        }
    }
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        btMultiBodyJacobianData jacobianData_normal;
        btMultiBodyJacobianData jacobianData_complementary;
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
                        btScalar vel = 0.0;
                        const btScalar* jac = &c.jacobianData_normal.m_jacobians[0];
                        const int ndof = multibodyLinkCol->m_multiBody->getNumDofs() + 6;
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
                    
                    if (m_constraints.find(c.m_node->index) == NULL)
                    {
                        m_constraints.insert(c.m_node->index, DeformableContactConstraint(c));
                    }
                    else
                    {
                        DeformableContactConstraint& constraints = *m_constraints[c.m_node->index];
                        bool single_contact = true;
                        if (single_contact)
                        {
                            if (constraints.m_contact[0]->m_cti.m_offset > cti.m_offset)
                            {
                                constraints.replace(c);
                            }
                        }
                        else
                        {
                            constraints.append(c);
                        }
                    }
                }
            }
        }
    }
}

void btDeformableContactProjection::enforceConstraint(TVStack& x)
{
    for (int index = 0; index < m_constraints.size(); ++index)
    {
        const DeformableContactConstraint& constraints = *m_constraints.getAtIndex(index);
        size_t i = m_constraints.getKeyAtIndex(index).getUid1();
        x[i].setZero();
        for (int j = 0; j < constraints.m_total_normal_dv.size(); ++j)
        {
            x[i] += constraints.m_total_normal_dv[j];
            x[i] += constraints.m_total_tangent_dv[j];
        }
    }
}

void btDeformableContactProjection::project(TVStack& x)
{
    const int dim = 3;
    for (int index = 0; index < m_constraints.size(); ++index)
    {
        const DeformableContactConstraint& constraints = *m_constraints.getAtIndex(index);
        size_t i = m_constraints.getKeyAtIndex(index).getUid1();
        if (constraints.m_contact[0] == NULL)
        {
            // static node
            x[i].setZero();
            continue;
        }
        bool has_static = false;
        for (int j = 0; j < constraints.m_static.size(); ++j)
        {
            has_static = has_static || constraints.m_static[j];
        }
        // static friction => fully constrained
        if (has_static)
        {
            x[i].setZero();
        }
        else if (constraints.m_total_normal_dv.size() >= dim)
        {
            x[i].setZero();
        }
        else if (constraints.m_total_normal_dv.size() == 2)
        {
            
            btVector3 dir0 = (constraints.m_total_normal_dv[0].norm() > SIMD_EPSILON) ? constraints.m_total_normal_dv[0].normalized() : btVector3(0,0,0);
            btVector3 dir1 = (constraints.m_total_normal_dv[1].norm() > SIMD_EPSILON) ? constraints.m_total_normal_dv[1].normalized() : btVector3(0,0,0);
            btVector3 free_dir = btCross(dir0, dir1);
            if (free_dir.norm() < SIMD_EPSILON)
            {
                x[i] -= x[i].dot(dir0) * dir0;
                x[i] -= x[i].dot(dir1) * dir1;
            }
            else
            {
                free_dir.normalize();
                x[i] = x[i].dot(free_dir) * free_dir;
            }
        }
        else
        {
            btAssert(constraints.m_total_normal_dv.size() == 1);
            btVector3 dir0 = (constraints.m_total_normal_dv[0].norm() > SIMD_EPSILON) ? constraints.m_total_normal_dv[0].normalized() : btVector3(0,0,0);
            x[i] -= x[i].dot(dir0) * dir0;
        }
    }
}

void btDeformableContactProjection::applyDynamicFriction(TVStack& f)
{
    for (int index = 0; index < m_constraints.size(); ++index)
    {
        const DeformableContactConstraint& constraint = *m_constraints.getAtIndex(index);
        const btSoftBody::Node* node = constraint.m_node;
        if (node == NULL)
            continue;
        size_t i = m_constraints.getKeyAtIndex(index).getUid1();
        bool has_static_constraint = false;
        
        // apply dynamic friction force (scaled by dt) if the node does not have static friction constraint
        for (int j = 0; j < constraint.m_static.size(); ++j)
        {
            if (constraint.m_static[j])
            {
                has_static_constraint = true;
                break;
            }
        }
        for (int j = 0; j < constraint.m_total_tangent_dv.size(); ++j)
        {
            btVector3 friction_force =  constraint.m_total_tangent_dv[j] * (1./node->m_im);
            if (!has_static_constraint)
            {
                f[i] += friction_force;
            }
        }
    }
}

void btDeformableContactProjection::reinitialize(bool nodeUpdated)
{
    btCGProjection::reinitialize(nodeUpdated);
    m_constraints.clear();
}



