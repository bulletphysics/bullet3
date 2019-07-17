//
//  btMassSpring.h
//  BulletSoftBody
//
//  Created by Chuyuan Fu on 7/1/19.
//

#ifndef BT_MASS_SPRING_H
#define BT_MASS_SPRING_H

#include "btLagrangianForce.h"

class btMassSpring : public btLagrangianForce
{
public:
    using TVStack = btLagrangianForce::TVStack;
    btMassSpring(const btAlignedObjectArray<btSoftBody *>& softBodies) : btLagrangianForce(softBodies)
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
        btAssert(numNodes == force.size())
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            const btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_links.size(); ++j)
            {
                const auto& link = psb->m_links[j];
                const auto node1 = link.m_n[0];
                const auto node2 = link.m_n[1];
                size_t id1 = m_indices[node1];
                size_t id2 = m_indices[node2];
                
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
        btAssert(numNodes == force.size())
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            const btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_links.size(); ++j)
            {
                const auto& link = psb->m_links[j];
                const auto node1 = link.m_n[0];
                const auto node2 = link.m_n[1];
                btScalar kLST = link.Feature::m_material->m_kLST;
                btScalar r = link.m_rl;
                size_t id1 = m_indices[node1];
                size_t id2 = m_indices[node2];
                
                // elastic force
                
                // fully implicit
//                btVector3 dir = (node2->m_x - node1->m_x);
                
                // explicit elastic force
                btVector3 dir = (node2->m_q - node1->m_q);
                btVector3 dir_normalized = dir.normalized();
                btVector3 scaled_force = scale * kLST * (dir - dir_normalized * r);
                force[id1] += scaled_force;
                force[id2] -= scaled_force;
            }
        }
    }
    
    virtual void addScaledElasticForceDifferential(btScalar scale, const TVStack& dx, TVStack& df)
    {
        int numNodes = getNumNodes();
        btAssert(numNodes == dx.size());
        btAssert(numNodes == df.size());
        
        // implicit elastic force differential
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            const btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_links.size(); ++j)
            {
                const auto& link = psb->m_links[j];
                const auto node1 = link.m_n[0];
                const auto node2 = link.m_n[1];
                btScalar kLST = link.Feature::m_material->m_kLST;
                size_t id1 = m_indices[node1];
                size_t id2 = m_indices[node2];
                btVector3 local_scaled_df = scale * kLST * (dx[id2] - dx[id1]);
                df[id1] += local_scaled_df;
                df[id2] -= local_scaled_df;
            }
        }
    }
    
    
    virtual void addScaledDampingForceDifferential(btScalar scale, const TVStack& dv, TVStack& df)
    {
        // implicit damping force differential
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            const btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_links.size(); ++j)
            {
                const auto& link = psb->m_links[j];
                const auto node1 = link.m_n[0];
                const auto node2 = link.m_n[1];
                btScalar k_damp = psb->m_dampingCoefficient;
                size_t id1 = m_indices[node1];
                size_t id2 = m_indices[node2];
                btVector3 local_scaled_df = scale * k_damp * (dv[id2] - dv[id1]);
                df[id1] += local_scaled_df;
                df[id2] -= local_scaled_df;
            }
        }
    }
    
    int getNumNodes()
    {
        int numNodes = 0;
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            numNodes += m_softBodies[i]->m_nodes.size();
        }
        return numNodes;
    }
};

#endif /* btMassSpring_h */
