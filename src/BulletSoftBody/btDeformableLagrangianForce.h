//
//  btDeformableLagrangianForce.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/1/19.
//

#ifndef BT_DEFORMABLE_LAGRANGIAN_FORCE_H
#define BT_DEFORMABLE_LAGRANGIAN_FORCE_H

#include "btSoftBody.h"
#include <unordered_map>

class btDeformableLagrangianForce
{
public:
    using TVStack = btAlignedObjectArray<btVector3>;
    const btAlignedObjectArray<btSoftBody *>& m_softBodies;
    std::unordered_map<btSoftBody::Node *, size_t> m_indices;
    
    btDeformableLagrangianForce(const btAlignedObjectArray<btSoftBody *>& softBodies)
    : m_softBodies(softBodies)
    {
    }
    
    virtual ~btDeformableLagrangianForce(){}
    
    virtual void addScaledImplicitForce(btScalar scale, TVStack& force) = 0;
    
    virtual void addScaledForceDifferential(btScalar scale, const TVStack& dv, TVStack& df) = 0;
    
    virtual void addScaledExplicitForce(btScalar scale, TVStack& force) = 0;
    
    virtual void reinitialize(bool nodeUpdated)
    {
        if (nodeUpdated)
            updateId();
    }
    
    virtual void updateId()
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
    
    virtual int getNumNodes()
    {
        int numNodes = 0;
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            numNodes += m_softBodies[i]->m_nodes.size();
        }
        return numNodes;
    }
};
#endif /* BT_DEFORMABLE_LAGRANGIAN_FORCE */
