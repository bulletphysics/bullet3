//
//  btDeformableGravityForce.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/21/19.
//

#ifndef BT_DEFORMABLE_GRAVITY_FORCE_H
#define BT_DEFORMABLE_GRAVITY_FORCE_H

#include "btDeformableLagrangianForce.h"

class btDeformableGravityForce : public btDeformableLagrangianForce
{
public:
    using TVStack = btDeformableLagrangianForce::TVStack;
    btVector3 m_gravity;
    
    btDeformableGravityForce(const btVector3& g) : m_gravity(g)
    {
        
    }
    
    virtual void addScaledImplicitForce(btScalar scale, TVStack& force)
    {
//        addScaledGravityForce(scale, force);
    }
    
    virtual void addScaledExplicitForce(btScalar scale, TVStack& force)
    {
        addScaledGravityForce(scale, force);
    }
    
    virtual void addScaledForceDifferential(btScalar scale, const TVStack& dv, TVStack& df)
    {
        
    }
    
    virtual void addScaledGravityForce(btScalar scale, TVStack& force)
    {
        int numNodes = getNumNodes();
        btAssert(numNodes <= force.size())
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                btSoftBody::Node& n = psb->m_nodes[j];
                size_t id = m_indices->at(&n);
                btScalar mass = (n.m_im == 0) ? 0 : 1. / n.m_im;
                btVector3 scaled_force = scale * m_gravity * mass;
                force[id] += scaled_force;
            }
        }
    }
    
    virtual btDeformableLagrangianForceType getForceType()
    {
        return BT_GRAVITY_FORCE;
    }
    
    
};
#endif /* BT_DEFORMABLE_GRAVITY_FORCE_H */
