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

//rigid_body.h

#ifndef DYN_RIGID_BODY_H
#define DYN_RIGID_BODY_H

#include "shared_ptr.h"
#include "collision_shape.h"
#include "mathUtils.h"

#include "rigid_body_impl.h"

class rigid_body_t
{
public:
    //typedefs
    typedef shared_ptr<rigid_body_t> pointer;

    //
    collision_shape_t::pointer collision_shape() { return m_collision_shape;   }

    //
    void set_kinematic(bool kinematic) {    m_impl->set_kinematic(kinematic);   }

    //
    void set_mass(float mass)               { m_impl->set_mass(mass);   }
    void set_inertia(vec3f const& I)        { m_impl->set_inertia(I);   }
    void set_restitution(float r)           { m_impl->set_restitution(r); }
    void set_friction(float f)              { m_impl->set_friction(f);    }
    void set_linear_damping(float d)         { m_impl->set_linear_damping(d);   }
    void set_angular_damping(float d)        { m_impl->set_angular_damping(d);   }

    void set_transform(vec3f const& position, quatf const& rotation)    { m_impl->set_transform(position, rotation);  }
    void get_transform(vec3f& position, quatf& rotation) const          { m_impl->get_transform(position, rotation);   }
    void get_transform(mat4x4f& xform) const                            { m_impl->get_transform(xform); }

    //
    void set_linear_velocity(vec3f const& v)                            { m_impl->set_linear_velocity(v);    }
    void get_linear_velocity(vec3f& v) const                            { m_impl->get_linear_velocity(v);    }

    void set_angular_velocity(vec3f const& v)                           { m_impl->set_angular_velocity(v);   }
    void get_angular_velocity(vec3f& v) const                           { m_impl->get_angular_velocity(v);   }

    //
    void clear_forces()                                                 { m_impl->clear_forces(); }
    void apply_central_force(vec3f const& f)                            { m_impl->apply_central_force(f);    }
    void apply_torque(vec3f const& t)                                   { m_impl->apply_torque(t);    }

public:
    virtual ~rigid_body_t() {};

protected:
    friend class solver_t;    
    rigid_body_t(rigid_body_impl_t* impl, collision_shape_t::pointer& shape): m_impl(impl), m_collision_shape(shape) 
    {
    }

    rigid_body_impl_t* impl() { return m_impl.get(); }
 //   rigid_body_impl_t const* impl() const { return m_impl.get(); }

private:
    shared_ptr<rigid_body_impl_t>           m_impl;
    collision_shape_t::pointer              m_collision_shape;
};



#endif
