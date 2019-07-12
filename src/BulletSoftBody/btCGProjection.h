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

struct Constraint
{
    const btSoftBody::RContact* m_contact;
    btVector3 m_direction;
    btScalar m_value;
    
    Constraint(const btSoftBody::RContact& rcontact)
    : m_contact(&rcontact)
    , m_direction(rcontact.m_cti.m_normal)
    , m_value(0)
    {
    }
    
    Constraint(const btVector3 dir)
    : m_contact(nullptr)
    , m_direction(dir)
    , m_value(0)
    {}
    
    Constraint()
    : m_contact(nullptr)
    {
        
    }
};

struct Friction
{
    btVector3 m_dv;
    btVector3 m_direction;
    Friction()
    {
        m_dv.setZero();
        m_direction.setZero();
    }
};

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
    const btScalar& m_dt;
    std::unordered_map<btSoftBody::Node *, btAlignedObjectArray<Constraint> > m_constraints;
    std::unordered_map<btSoftBody::Node *, Friction > m_frictions;
    
    btCGProjection(btAlignedObjectArray<btSoftBody *>& softBodies, const btScalar& dt)
    : m_softBodies(softBodies)
    , m_dt(dt)
    {
    }
    
    virtual ~btCGProjection()
    {
    }
    
    // apply the constraints
    virtual void operator()(TVStack& x) = 0;
    
    virtual void setConstraintDirections() = 0;
    
    // update the constraints
    virtual void update(const TVStack& dv, const TVStack& backup_v) = 0;
    
    virtual void reinitialize(bool nodeUpdated)
    {
        if (nodeUpdated)
            updateId();
        m_constraints.clear();
        m_frictions.clear();
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
