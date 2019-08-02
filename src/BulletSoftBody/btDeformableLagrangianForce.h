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
enum btDeformableLagrangianForceType
{
    BT_GRAVITY_FORCE = 1,
    BT_MASSSPRING_FORCE = 2
};

class btDeformableLagrangianForce
{
public:
//    using TVStack = btAlignedObjectArray<btVector3>;
    typedef btAlignedObjectArray<btVector3> TVStack;
    btAlignedObjectArray<btSoftBody *> m_softBodies;
    const std::unordered_map<btSoftBody::Node *, size_t>* m_indices;
    
    btDeformableLagrangianForce()
    {
    }
    
    virtual ~btDeformableLagrangianForce(){}
    
    virtual void addScaledImplicitForce(btScalar scale, TVStack& force) = 0;
    
    virtual void addScaledForceDifferential(btScalar scale, const TVStack& dv, TVStack& df) = 0;
    
    virtual void addScaledExplicitForce(btScalar scale, TVStack& force) = 0;
    
    virtual btDeformableLagrangianForceType getForceType() = 0;
    
    virtual void reinitialize(bool nodeUpdated)
    {
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
    
    virtual void addSoftBody(btSoftBody* psb)
    {
        m_softBodies.push_back(psb);
    }
    
    virtual void setIndices(const std::unordered_map<btSoftBody::Node *, size_t>* indices)
    {
        m_indices = indices;
    }
};
#endif /* BT_DEFORMABLE_LAGRANGIAN_FORCE */
