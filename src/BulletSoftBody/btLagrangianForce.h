//
//  btLagrangianForce.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/1/19.
//

#ifndef BT_LAGRANGIAN_FORCE_H
#define BT_LAGRANGIAN_FORCE_H

#include "btSoftBody.h"
#include <unordered_map>

class btLagrangianForce
{
public:
    using TVStack = btAlignedObjectArray<btVector3>;
    const btAlignedObjectArray<btSoftBody *>& m_softBodies;
    std::unordered_map<btSoftBody::Node *, size_t> m_indices;
    
    btLagrangianForce(const btAlignedObjectArray<btSoftBody *>& softBodies)
    : m_softBodies(softBodies)
    {
    }
    
    virtual ~btLagrangianForce(){}
    
    virtual void addScaledForce(btScalar scale, TVStack& force) = 0;
    
    virtual void addScaledElasticForceDifferential(btScalar scale, const TVStack& dx, TVStack& df) = 0;
    
    virtual void addScaledDampingForceDifferential(btScalar scale, const TVStack& dv, TVStack& df) = 0;
    
    virtual void reinitialize(bool nodeUpdated)
    {
        if (nodeUpdated)
            updateId();
    }
    
    void updateId()
    {
        size_t index = 0;
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                m_indices[&(psb->m_nodes[j])] = index++;
            }
        }
    }
};
#endif /* btLagrangianForce_h */
