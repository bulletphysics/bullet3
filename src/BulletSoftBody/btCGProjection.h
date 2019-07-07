//  btCGProjection.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/4/19.
//

#ifndef BT_CG_PROJECTION_H
#define BT_CG_PROJECTION_H

#include "btSoftBody.h"
#include <unordered_map>

class btDeformableRigidDynamicsWorld;
class btCGProjection
{
public:
//    static const int dim = 3;
    using TVStack = btAlignedObjectArray<btVector3>;
    using TVArrayStack = btAlignedObjectArray<btAlignedObjectArray<btVector3> >;
    using TArrayStack = btAlignedObjectArray<btAlignedObjectArray<btScalar> >;
    btAlignedObjectArray<btSoftBody *> m_softBodies;
    btDeformableRigidDynamicsWorld* m_world;
    std::unordered_map<btSoftBody::Node *, size_t> m_indices;
    TVArrayStack m_constrainedDirections;
    TArrayStack m_constrainedValues;
    
    btCGProjection(btAlignedObjectArray<btSoftBody *>& softBodies)
    : m_softBodies(softBodies)
    {
        
    }
    
    virtual ~btCGProjection()
    {
        
    }
    
    // apply the constraints
    virtual void operator()(TVStack& x) = 0;
    
    // update the constraints
    virtual void update(btScalar dt, const TVStack& dv) = 0;
    
    virtual void reinitialize(bool nodeUpdated)
    {
        if (nodeUpdated)
            updateId();
        m_constrainedValues.resize(m_indices.size());
        m_constrainedDirections.resize(m_indices.size());
    }
    
    void updateId()
    {
        size_t index = 0;
        m_indices.clear();
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                m_indices[&(psb->m_nodes[j])] = index++;
            }
        }
    }
    
    void setSoftBodies(btAlignedObjectArray<btSoftBody* > softBodies)
    {
        m_softBodies.copyFromArray(softBodies);
    }
    
    virtual void setWorld(btDeformableRigidDynamicsWorld* world)
    {
        m_world = world;
    }
};


#endif /* btCGProjection_h */
