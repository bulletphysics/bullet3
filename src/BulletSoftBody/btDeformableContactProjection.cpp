/*
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
#include "btDeformableRigidDynamicsWorld.h"
#include <algorithm>
#include <cmath>
static void findJacobian(const btMultiBodyLinkCollider* multibodyLinkCol,
                         btMultiBodyJacobianData& jacobianData,
                         const btVector3& contact_point,
                         const btVector3& dir)
{
    const int ndof = multibodyLinkCol->m_multiBody->getNumDofs() + 6;
    jacobianData.m_jacobians.resize(ndof);
    jacobianData.m_deltaVelocitiesUnitImpulse.resize(ndof);
    btScalar* jac = &jacobianData.m_jacobians[0];
    
    multibodyLinkCol->m_multiBody->fillContactJacobianMultiDof(multibodyLinkCol->m_link, contact_point, dir, jac, jacobianData.scratch_r, jacobianData.scratch_v, jacobianData.scratch_m);
    multibodyLinkCol->m_multiBody->calcAccelerationDeltasMultiDof(&jacobianData.m_jacobians[0], &jacobianData.m_deltaVelocitiesUnitImpulse[0], jacobianData.scratch_r, jacobianData.scratch_v);
}

static btVector3 generateUnitOrthogonalVector(const btVector3& u)
{
    btScalar ux = u.getX();
    btScalar uy = u.getY();
    btScalar uz = u.getZ();
    btScalar ax = std::abs(ux);
    btScalar ay = std::abs(uy);
    btScalar az = std::abs(uz);
    btVector3 v;
    if (ax <= ay && ax <= az)
        v = btVector3(0, -uz, uy);
    else if (ay <= ax && ay <= az)
        v = btVector3(-uz, 0, ux);
    else
        v = btVector3(-uy, ux, 0);
    v.normalize();
    return v;
}

void btDeformableContactProjection::update()
{
    ///solve rigid body constraints
    m_world->getSolverInfo().m_numIterations = 1;
    m_world->btMultiBodyDynamicsWorld::solveInternalConstraints(m_world->getSolverInfo());

    // loop through constraints to set constrained values
    for (int index = 0; index < m_constraints.size(); ++index)
    {
        btAlignedObjectArray<DeformableFrictionConstraint>& frictions = *m_frictions[m_constraints.getKeyAtIndex(index)];
        btAlignedObjectArray<DeformableContactConstraint>& constraints = *m_constraints.getAtIndex(index);
        for (int i = 0; i < constraints.size(); ++i)
        {
            DeformableContactConstraint& constraint = constraints[i];
            DeformableFrictionConstraint& friction = frictions[i];
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
                            deltaV_normal = &c->jacobianData_normal.m_deltaVelocitiesUnitImpulse[0];
                            // add in the normal component of the va
                            btScalar vel = 0.0;
                            for (int k = 0; k < ndof; ++k)
                            {
                                vel += local_v[k] * J_n[k];
                            }
                            va = cti.m_normal * vel * m_dt;

                            // add in the tangential components of the va
                            vel = 0.0;
                            for (int k = 0; k < ndof; ++k)
                            {
                                vel += local_v[k] * J_t1[k];
                            }
                            va += c->t1 * vel * m_dt;
                            vel = 0.0;
                            for (int k = 0; k < ndof; ++k)
                            {
                                vel += local_v[k] * J_t2[k];
                            }
                            va += c->t2 * vel * m_dt;
                        }
                    }
                    
                    const btVector3 vb = c->m_node->m_v * m_dt;
                    const btVector3 vr = vb - va;
                    const btScalar dn = btDot(vr, cti.m_normal);
                    btVector3 impulse = c->m_c0 * vr;
                    const btVector3 impulse_normal = c->m_c0 * (cti.m_normal * dn);
                    const btVector3 impulse_tangent = impulse - impulse_normal;
                    
                    // start friction handling
                    // copy old data
                    friction.m_impulse_prev[j] = friction.m_impulse[j];
                    friction.m_dv_prev[j] = friction.m_dv[j];
                    friction.m_static_prev[j] = friction.m_static[j];
                    friction.m_direction_prev[j] = friction.m_direction[j];
                    
                    // get the current tangent direction
                    btScalar local_tangent_norm = impulse_tangent.norm();
                    btVector3 local_tangent_dir = -friction.m_direction[j];
                    if (local_tangent_norm > SIMD_EPSILON)
                        local_tangent_dir = impulse_tangent.normalized();

                    // accumulated impulse on the rb in this and all prev cg iterations
                    constraint.m_accumulated_normal_impulse[j] += impulse_normal.dot(cti.m_normal);
                    const btScalar& accumulated_normal = constraint.m_accumulated_normal_impulse[j];
                    
                    // the total tangential impulse required to stop sliding
                    btVector3 tangent = friction.m_accumulated_tangent_impulse[j] + impulse_tangent;
                    btScalar tangent_norm = tangent.norm();
  
                    if (accumulated_normal < 0)
                    {
                        friction.m_direction[j] = -local_tangent_dir;
                        // do not allow switching from static friction to dynamic friction
                        // it causes cg to explode
                        if (-accumulated_normal*c->m_c3 < tangent_norm && friction.m_static_prev[j] == false && friction.m_released[j] == false)
                        {
                            friction.m_static[j] = false;
                            friction.m_impulse[j] = -accumulated_normal*c->m_c3;
                        }
                        else
                        {
                            friction.m_static[j] = true;
                            friction.m_impulse[j] = tangent_norm;
                        }
                    }
                    else
                    {
                        friction.m_released[j] = true;
                        friction.m_static[j] = false;
                        friction.m_impulse[j] = 0;
                        friction.m_direction[j] = btVector3(0,0,0);
                    }
                    friction.m_dv[j] = friction.m_impulse[j] * c->m_c2/m_dt;
                    friction.m_accumulated_tangent_impulse[j] = -friction.m_impulse[j] * friction.m_direction[j];
                    
                    // the incremental impulse applied to rb in the tangential direction
                    btVector3 incremental_tangent = (friction.m_impulse_prev[j] * friction.m_direction_prev[j])-(friction.m_impulse[j] * friction.m_direction[j]);
                    

                    // dv = new_impulse + accumulated velocity change in previous CG iterations
                    // so we have the invariant node->m_v = backupVelocity + dv;

                    btScalar dvn = -accumulated_normal * c->m_c2/m_dt;
                    
                    // the following is equivalent
                    /*
                        btVector3 dv = -impulse_normal * c->m_c2/m_dt + c->m_node->m_v - backupVelocity[m_indices->at(c->m_node)];
                        btScalar dvn = dv.dot(cti.m_normal);
                     */
                    
                    constraint.m_value[j] = dvn;
                    
                    // the incremental impulse:
                    // in the normal direction it's the normal component of "impulse"
                    // in the tangent direction it's the difference between the frictional impulse in the iteration and the previous iteration
                    impulse = impulse_normal + incremental_tangent;
                    if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
                    {
                        if (rigidCol)
                            rigidCol->applyImpulse(impulse, c->m_c1);
                    }
                    else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
                    {
                        if (multibodyLinkCol)
                        {
                            // apply normal component of the impulse
                            multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof(deltaV_normal, impulse.dot(cti.m_normal));
                            if (incremental_tangent.norm() > SIMD_EPSILON)
                            {
                                // apply tangential component of the impulse
                               const btScalar* deltaV_t1 = &c->jacobianData_t1.m_deltaVelocitiesUnitImpulse[0];
                                multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof(deltaV_t1, impulse.dot(c->t1));
                                const btScalar* deltaV_t2 = &c->jacobianData_t2.m_deltaVelocitiesUnitImpulse[0];
                                multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof(deltaV_t2, impulse.dot(c->t2));
                            }
                        }
                    }
                }
            }
        }
    }
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
                btAlignedObjectArray<DeformableContactConstraint> c;
                c.reserve(3);
                c.push_back(DeformableContactConstraint(btVector3(1,0,0)));
                c.push_back(DeformableContactConstraint(btVector3(0,1,0)));
                c.push_back(DeformableContactConstraint(btVector3(0,0,1)));
                m_constraints.insert(psb->m_nodes[j].index, c);
                
                btAlignedObjectArray<DeformableFrictionConstraint> f;
                f.reserve(3);
                f.push_back(DeformableFrictionConstraint());
                f.push_back(DeformableFrictionConstraint());
                f.push_back(DeformableFrictionConstraint());
                m_frictions.insert(psb->m_nodes[j].index, f);
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
                        btAlignedObjectArray<DeformableContactConstraint> constraints;
                        constraints.push_back(DeformableContactConstraint(c));
                        m_constraints.insert(c.m_node->index,constraints);
                        btAlignedObjectArray<DeformableFrictionConstraint> frictions;
                        frictions.push_back(DeformableFrictionConstraint());
                        m_frictions.insert(c.m_node->index,frictions);
                    }
                    else
                    {
                        // group colinear constraints into one
                        const btScalar angle_epsilon = 0.015192247; // less than 10 degree
                        bool merged = false;
                        btAlignedObjectArray<DeformableContactConstraint>& constraints = *m_constraints[c.m_node->index];
                        btAlignedObjectArray<DeformableFrictionConstraint>& frictions = *m_frictions[c.m_node->index];
                        for (int j = 0; j < constraints.size(); ++j)
                        {
                            const btAlignedObjectArray<btVector3>& dirs = constraints[j].m_direction;
                            btScalar dot_prod = dirs[0].dot(cti.m_normal);
                            if (std::abs(std::abs(dot_prod) - 1) < angle_epsilon)
                            {
                                // group the constraints
                                constraints[j].append(c);
                                // push in an empty friction
                                frictions[j].append();
                                merged = true;
                                break;
                            }
                        }
                        const int dim = 3;
                        // hard coded no more than 3 constraint directions
                        if (!merged && constraints.size() < dim)
                        {
                            constraints.push_back(DeformableContactConstraint(c));
                            frictions.push_back(DeformableFrictionConstraint());
                        }
                    }
                }
            }
        }
    }
}

void btDeformableContactProjection::enforceConstraint(TVStack& x)
{
    const int dim = 3;
    for (int index = 0; index < m_constraints.size(); ++index)
    {
        const btAlignedObjectArray<DeformableContactConstraint>& constraints = *m_constraints.getAtIndex(index);
        size_t i = m_constraints.getKeyAtIndex(index).getUid1();
        const btAlignedObjectArray<DeformableFrictionConstraint>& frictions = *m_frictions[m_constraints.getKeyAtIndex(index)];
        btAssert(constraints.size() <= dim);
        btAssert(constraints.size() > 0);
        if (constraints.size() == 1)
        {
            x[i] -= x[i].dot(constraints[0].m_direction[0]) * constraints[0].m_direction[0];
            for (int j = 0; j < constraints[0].m_direction.size(); ++j)
                x[i] += constraints[0].m_value[j] * constraints[0].m_direction[j];
        }
        else if (constraints.size() == 2)
        {
            btVector3 free_dir = btCross(constraints[0].m_direction[0], constraints[1].m_direction[0]);
            btAssert(free_dir.norm() > SIMD_EPSILON)
            free_dir.normalize();
            x[i] = x[i].dot(free_dir) * free_dir;
            for (int j = 0; j < constraints.size(); ++j)
            {
                for (int k = 0; k < constraints[j].m_direction.size(); ++k)
                {
                    x[i] += constraints[j].m_value[k] * constraints[j].m_direction[k];
                }
            }
        }
        else
        {
            x[i].setZero();
            for (int j = 0; j < constraints.size(); ++j)
            {
                for (int k = 0; k < constraints[j].m_direction.size(); ++k)
                {
                    x[i] += constraints[j].m_value[k] * constraints[j].m_direction[k];
                }
            }
        }
        
        // apply friction if the node is not constrained in all directions
        if (constraints.size() < 3)
        {
            for (int f = 0; f < frictions.size(); ++f)
            {
                const DeformableFrictionConstraint& friction= frictions[f];
                for (int j = 0; j < friction.m_direction.size(); ++j)
                {
                    // add the friction constraint
                    if (friction.m_static[j] == true)
                    {
//                        if (friction.m_static_prev[j] == true)
//                            x[i] -= friction.m_direction_prev[j] * friction.m_dv_prev[j];
//                        x[i] -= x[i].dot(friction.m_direction[j]) * friction.m_direction[j];
                        x[i] += friction.m_direction[j] * friction.m_dv[j];
                    }
                }
            }
        }
    }
}

void btDeformableContactProjection::project(TVStack& x)
{
    const int dim = 3;
    for (int index = 0; index < m_constraints.size(); ++index)
    {
        const btAlignedObjectArray<DeformableContactConstraint>& constraints = *m_constraints.getAtIndex(index);
        btAlignedObjectArray<DeformableFrictionConstraint>& frictions = *m_frictions[m_constraints.getKeyAtIndex(index)];
        size_t i = m_constraints.getKeyAtIndex(index).getUid1();
        btAssert(constraints.size() <= dim);
        btAssert(constraints.size() > 0);
        if (constraints.size() == 1)
        {
            x[i] -= x[i].dot(constraints[0].m_direction[0]) * constraints[0].m_direction[0];
        }
        else if (constraints.size() == 2)
        {
            btVector3 free_dir = btCross(constraints[0].m_direction[0], constraints[1].m_direction[0]);
            btAssert(free_dir.norm() > SIMD_EPSILON)
            free_dir.normalize();
            x[i] = x[i].dot(free_dir) * free_dir;
        }
        else
            x[i].setZero();
        // apply the friction constraint
        if (constraints.size() < 3)
        {
            for (int f = 0; f < frictions.size(); ++f)
            {
                DeformableFrictionConstraint& friction= frictions[f];
                for (int j = 0; j < friction.m_direction.size(); ++j)
                {
                    if (friction.m_static[j] == true)
                    {
                        x[i] -= x[i].dot(friction.m_direction[j]) * friction.m_direction[j];
                    }
                }
            }
        }
    }
}

void btDeformableContactProjection::projectFriction(TVStack& x)
{
    const int dim = 3;
    for (int index = 0; index < m_constraints.size(); ++index)
    {
        const btAlignedObjectArray<DeformableContactConstraint>& constraints = *m_constraints.getAtIndex(index);
        size_t i = m_constraints.getKeyAtIndex(index).getUid1();
        btAlignedObjectArray<DeformableFrictionConstraint>& frictions = *m_frictions[m_constraints.getKeyAtIndex(index)];
        btAssert(constraints.size() <= dim);
        btAssert(constraints.size() > 0);
        
        // apply friction if the node is not constrained in all directions
        if (constraints.size() < 3)
        {
            bool has_static_constraint = false;
            for (int f = 0; f < frictions.size(); ++f)
            {
                DeformableFrictionConstraint& friction= frictions[f];
                for (int j = 0; j < friction.m_static.size(); ++j)
                    has_static_constraint = has_static_constraint || friction.m_static[j];
            }
            
            for (int f = 0; f < frictions.size(); ++f)
            {
                DeformableFrictionConstraint& friction= frictions[f];
                for (int j = 0; j < friction.m_direction.size(); ++j)
                {
                    // only add to the rhs if there is no static friction constraint on the node
                    if (friction.m_static[j] == false)
                    {
                        if (!has_static_constraint)
                            x[i] += friction.m_direction[j] * friction.m_impulse[j];
                    }
                }
            }
        }
    }
}

void btDeformableContactProjection::reinitialize(bool nodeUpdated)
{
    btCGProjection::reinitialize(nodeUpdated);
    m_constraints.clear();
    m_frictions.clear();
}



