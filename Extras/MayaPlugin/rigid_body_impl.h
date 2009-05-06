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


//rigid_body_impl.h

#ifndef DYN_RIGID_BODY_IMPL_H
#define DYN_RIGID_BODY_IMPL_H

#include "mathUtils.h"
#include "constraint/bt_constraint.h"

class rigid_body_impl_t
{
public:
    virtual void set_kinematic(bool kinematic) = 0;

    //
    virtual void set_mass(float mass) = 0;               
    virtual void set_inertia(vec3f const& I) = 0;      
    virtual void set_restitution(float r) = 0;          
    virtual void set_friction(float f) = 0;              
    virtual void set_linear_damping(float d) = 0;         
    virtual void set_angular_damping(float d) = 0;        

    virtual void set_transform(vec3f const& position, quatf const& rotation) = 0;
    virtual void get_transform(vec3f& position, quatf& rotation) const = 0;
    virtual void get_transform(mat4x4f& xform) const = 0;

    virtual void set_linear_velocity(vec3f const& v) = 0;                       
    virtual void get_linear_velocity(vec3f& v) const = 0;                           

    virtual void set_angular_velocity(vec3f const& v) = 0;                          
    virtual void get_angular_velocity(vec3f& v) const = 0;

    virtual void clear_forces() = 0;
    virtual void apply_central_force(vec3f const& f) = 0;
    virtual void apply_torque(vec3f const& t) = 0;

	virtual void update_constraint() = 0;
	virtual void add_constraint(bt_constraint_t* constraint) = 0;
	virtual void remove_constraint(bt_constraint_t* constraint) = 0;

public:
    virtual ~rigid_body_impl_t() {};
};

#endif
