//  btCGProjection.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/4/19.
//

#ifndef BT_CG_PROJECTION_H
#define BT_CG_PROJECTION_H

#include "btSoftBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"
#include <unordered_map>

class btDeformableRigidDynamicsWorld;

struct DeformableContactConstraint
{
    btAlignedObjectArray<const btSoftBody::RContact*> m_contact;
    btAlignedObjectArray<btVector3> m_direction;
    btAlignedObjectArray<btScalar> m_value;
    // the magnitude of the total impulse the node applied to the rb in the normal direction in the cg solve
    btAlignedObjectArray<btScalar> m_accumulated_normal_impulse;
    
    DeformableContactConstraint(const btSoftBody::RContact& rcontact)
    {
        append(rcontact);
    }
    
    DeformableContactConstraint(const btVector3 dir)
    {
        m_contact.push_back(nullptr);
        m_direction.push_back(dir);
        m_value.push_back(0);
        m_accumulated_normal_impulse.push_back(0);
    }
    
    DeformableContactConstraint()
    {
        m_contact.push_back(nullptr);
        m_direction.push_back(btVector3(0,0,0));
        m_value.push_back(0);
        m_accumulated_normal_impulse.push_back(0);
    }
    
    void append(const btSoftBody::RContact& rcontact)
    {
        m_contact.push_back(&rcontact);
        m_direction.push_back(rcontact.m_cti.m_normal);
        m_value.push_back(0);
        m_accumulated_normal_impulse.push_back(0);
    }
    
    ~DeformableContactConstraint()
    {
    }
};

struct DeformableFrictionConstraint
{
    
    btAlignedObjectArray<bool> m_static; // whether the friction is static
    btAlignedObjectArray<btScalar> m_impulse; // the impulse magnitude the node feels
    btAlignedObjectArray<btScalar> m_dv;      // the dv magnitude of the node
    btAlignedObjectArray<btVector3> m_direction; // the direction of the friction for the node
    
    
    btAlignedObjectArray<bool> m_static_prev;
    btAlignedObjectArray<btScalar> m_impulse_prev;
    btAlignedObjectArray<btScalar> m_dv_prev;
    btAlignedObjectArray<btVector3> m_direction_prev;
    
    btAlignedObjectArray<bool> m_released; // whether the contact is released

    
    // the total impulse the node applied to the rb in the tangential direction in the cg solve
    btAlignedObjectArray<btVector3> m_accumulated_tangent_impulse;
    
    DeformableFrictionConstraint()
    {
        append();
    }
    
    void append()
    {
        m_static.push_back(false);
        m_static_prev.push_back(false);
        
        m_direction_prev.push_back(btVector3(0,0,0));
        m_direction.push_back(btVector3(0,0,0));
        
        m_impulse.push_back(0);
        m_impulse_prev.push_back(0);
        
        m_dv.push_back(0);
        m_dv_prev.push_back(0);
        
        m_accumulated_tangent_impulse.push_back(btVector3(0,0,0));
        m_released.push_back(false);
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
    const std::unordered_map<btSoftBody::Node *, size_t>* m_indices;
    const btScalar& m_dt;
    
    btCGProjection(btAlignedObjectArray<btSoftBody *>& softBodies, const btScalar& dt, const std::unordered_map<btSoftBody::Node *, size_t>* indices)
    : m_softBodies(softBodies)
    , m_dt(dt)
    , m_indices(indices)
    {
    }
    
    virtual ~btCGProjection()
    {
    }
    
    // apply the constraints
    virtual void project(TVStack& x) = 0;
    
    virtual void setConstraints() = 0;
    
    // update the constraints
    virtual void update() = 0;
    
    virtual void reinitialize(bool nodeUpdated)
    {
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