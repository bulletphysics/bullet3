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

#ifndef BT_CG_PROJECTION_H
#define BT_CG_PROJECTION_H

#include "btSoftBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"

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
    
    DeformableContactConstraint(const btVector3& dir)
    {
        m_contact.push_back(NULL);
        m_direction.push_back(dir);
        m_value.push_back(0);
        m_accumulated_normal_impulse.push_back(0);
    }
    
    DeformableContactConstraint()
    {
        m_contact.push_back(NULL);
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
    typedef btAlignedObjectArray<btVector3> TVStack;
    typedef btAlignedObjectArray<btAlignedObjectArray<btVector3> > TVArrayStack;
    typedef btAlignedObjectArray<btAlignedObjectArray<btScalar> > TArrayStack;
    btAlignedObjectArray<btSoftBody *>& m_softBodies;
    btDeformableRigidDynamicsWorld* m_world;
//    const btAlignedObjectArray<btSoftBody::Node*>* m_nodes;
    const btScalar& m_dt;
    
    btCGProjection(btAlignedObjectArray<btSoftBody *>& softBodies, const btScalar& dt)
    : m_softBodies(softBodies)
    , m_dt(dt)
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
    
    virtual void setWorld(btDeformableRigidDynamicsWorld* world)
    {
        m_world = world;
    }
};


#endif /* btCGProjection_h */
