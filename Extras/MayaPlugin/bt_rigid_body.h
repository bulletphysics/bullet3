/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Walt Disney Studios
 
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising
from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
 
1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this
software in a product, an acknowledgment in the product documentation
would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 
Written by: Nicola Candussi <nicola@fluidinteractive.com>
*/

//bt_rigid_body.h

#ifndef DYN_BT_RIGID_BODY_H
#define DYN_BT_RIGID_BODY_H

#include "shared_ptr.h"
#include "rigid_body_impl.h"
#include "bt_collision_shape.h"

class bt_rigid_body_t: public rigid_body_impl_t {
public:
    virtual void get_transform(mat4x4f &xform) const
    {
        float m[16];
        m_body->getWorldTransform().getOpenGLMatrix(m);
        xform = trans(cmat<float, 4, 4>(m));
    }

    virtual void get_transform(vec3f &position, quatf &rotation) const
    {
        const btTransform& btxform = m_body->getWorldTransform();
        btQuaternion q = btxform.getRotation();
        btVector3 p = btxform.getOrigin();
        position = vec3f(p.x(), p.y(), p.z());
        rotation = quatf(q.w(), q.x(), q.y(), q.z());
    }

    virtual void set_transform(vec3f const &position, quatf const &rotation)
    {
        vec3f tp = position;
        quatf tr = rotation;
        btTransform xform(btQuaternion(tr[1], tr[2], tr[3], tr[0]),
                          btVector3(tp[0], tp[1], tp[2])); 
        m_body->setWorldTransform(xform);
    }

    virtual void set_kinematic(bool kinematic)
    {
        if(kinematic) {
            m_body->setCollisionFlags(m_body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
            m_body->setActivationState(DISABLE_DEACTIVATION);
            m_body->setMassProps(0, btVector3(0.0,0.0,0.0)); 
            m_body->updateInertiaTensor();
        } else {
            m_body->setCollisionFlags(m_body->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
            m_body->setActivationState(ACTIVE_TAG);
            m_body->setMassProps(m_mass, m_inertia); 
            m_body->updateInertiaTensor();
        }
    }
    
    virtual void set_mass(float mass)                
    {
//        std::cout << "bt_rigid_body::set_mass: " << mass << std::endl;
        m_mass = mass;
        if(m_body->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT) {
            m_body->setMassProps(0, btVector3(0,0,0));
        } else  {
            m_body->setMassProps(m_mass, m_inertia); 
        }
        m_body->updateInertiaTensor();
    }

    virtual void set_inertia(vec3f const& I)
    {
        m_inertia = btVector3(I[0], I[1], I[2]);
        if(m_body->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT) {
            m_body->setMassProps(0, btVector3(0,0,0));
        } else  {
            m_body->setMassProps(m_mass, m_inertia); 
        }
        m_body->updateInertiaTensor();
    }

    virtual void set_restitution(float r)
    {
        m_body->setRestitution(r);
    }

    virtual void set_friction(float f)
    {
        m_body->setFriction(f);
    }

    virtual void set_linear_damping(float d)
    {
        m_linear_damping = d;
        m_body->setDamping(m_linear_damping, m_angular_damping);
    }

    virtual void set_angular_damping(float d)
    {
        m_angular_damping = d;
        m_body->setDamping(m_linear_damping, m_angular_damping);
    }

    virtual void set_linear_velocity(vec3f const& v)
    {
        m_body->setLinearVelocity(btVector3(v[0], v[1], v[2]));
    }

    virtual void get_linear_velocity(vec3f& v) const
    {
        const btVector3 &val = m_body->getLinearVelocity();
        v = vec3f(val.x(), val.y(), val.z()); 
    }

    virtual void set_angular_velocity(vec3f const& v)
    {
        m_body->setAngularVelocity(btVector3(v[0], v[1], v[2]));
    }

    virtual void get_angular_velocity(vec3f& v) const
    {
        const btVector3 &val = m_body->getAngularVelocity();
        v = vec3f(val.x(), val.y(), val.z()); 
    }

    virtual void clear_forces()
    {
        m_body->clearForces();
    }

    virtual void apply_central_force(vec3f const& f)
    {
        m_body->applyCentralForce(btVector3(f[0], f[1], f[2]));
    }

    virtual void apply_torque(vec3f const& t)
    {
        m_body->applyTorque(btVector3(t[0], t[1], t[2]));
    }

    btRigidBody* body() { return m_body.get(); }

protected:
    friend class bt_solver_t;

    bt_rigid_body_t(collision_shape_impl_t* cs):
        m_collision_shape(cs),
        m_mass(1),
        m_inertia(2.0/5.0, 2.0/5.0, 2.0/5.0),
        m_linear_damping(0),
        m_angular_damping(0)
    { 
        bt_collision_shape_t* bt_shape = dynamic_cast<bt_collision_shape_t*>(cs);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(m_mass, NULL, bt_shape->shape(), m_inertia);
        rbInfo.m_restitution = 0;
        rbInfo.m_friction = 0.1;
        rbInfo.m_linearDamping = m_linear_damping;
        rbInfo.m_angularDamping = m_angular_damping;
        m_body.reset(new btRigidBody(rbInfo));
    }

private:
    shared_ptr<btRigidBody> m_body; 
    collision_shape_impl_t* m_collision_shape;
    float m_mass;
    btVector3 m_inertia;
    float m_linear_damping;
    float m_angular_damping;
};

#endif
