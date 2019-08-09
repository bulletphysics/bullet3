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

#ifndef BT_MASS_SPRING_H
#define BT_MASS_SPRING_H

#include "btDeformableLagrangianForce.h"

class btDeformableMassSpringForce : public btDeformableLagrangianForce
{
public:
//    using TVStack = btDeformableLagrangianForce::TVStack;
    typedef btAlignedObjectArray<btVector3> TVStack;
    btDeformableMassSpringForce()
    {
    }
    
    virtual void addScaledImplicitForce(btScalar scale, TVStack& force)
    {
        addScaledDampingForce(scale, force);
    }
    
    virtual void addScaledExplicitForce(btScalar scale, TVStack& force)
    {
        addScaledElasticForce(scale, force);
    }
    
    virtual void addScaledDampingForce(btScalar scale, TVStack& force)
    {
        int numNodes = getNumNodes();
        btAssert(numNodes <= force.size())
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            const btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_links.size(); ++j)
            {
                const btSoftBody::Link& link = psb->m_links[j];
                btSoftBody::Node* node1 = link.m_n[0];
                btSoftBody::Node* node2 = link.m_n[1];
                size_t id1 = node1->index;
                size_t id2 = node2->index;
                
                // damping force
                btVector3 v_diff = (node2->m_v - node1->m_v);
                btScalar k_damp = psb->m_dampingCoefficient;
                btVector3 scaled_force = scale * v_diff * k_damp;
                force[id1] += scaled_force;
                force[id2] -= scaled_force;
            }
        }
    }
    
    virtual void addScaledElasticForce(btScalar scale, TVStack& force)
    {
        int numNodes = getNumNodes();
        btAssert(numNodes <= force.size())
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            const btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_links.size(); ++j)
            {
                const btSoftBody::Link& link = psb->m_links[j];
                btSoftBody::Node* node1 = link.m_n[0];
                btSoftBody::Node* node2 = link.m_n[1];
                btScalar kLST = link.Feature::m_material->m_kLST;
                btScalar r = link.m_rl;
                size_t id1 = node1->index;
                size_t id2 = node2->index;
                
                // elastic force
                // explicit elastic force
                btVector3 dir = (node2->m_q - node1->m_q);
                btVector3 dir_normalized = dir.normalized();
                btVector3 scaled_force = scale * kLST * (dir - dir_normalized * r);
                force[id1] += scaled_force;
                force[id2] -= scaled_force;
            }
        }
    }
    
    virtual void addScaledForceDifferential(btScalar scale, const TVStack& dv, TVStack& df)
    {
        // implicit damping force differential
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            const btSoftBody* psb = m_softBodies[i];
            btScalar scaled_k_damp = psb->m_dampingCoefficient * scale;
            for (int j = 0; j < psb->m_links.size(); ++j)
            {
                const btSoftBody::Link& link = psb->m_links[j];
                btSoftBody::Node* node1 = link.m_n[0];
                btSoftBody::Node* node2 = link.m_n[1];
                size_t id1 = node1->index;
                size_t id2 = node2->index;
                btVector3 local_scaled_df = scaled_k_damp * (dv[id2] - dv[id1]);
                df[id1] += local_scaled_df;
                df[id2] -= local_scaled_df;
            }
        }
    }
    
    virtual btDeformableLagrangianForceType getForceType()
    {
        return BT_MASSSPRING_FORCE;
    }
    
};

#endif /* btMassSpring_h */
