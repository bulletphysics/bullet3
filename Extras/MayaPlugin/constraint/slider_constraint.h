/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Herbert Law
 
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
 
Written by: Herbert Law <Herbert.Law@gmail.com>
*/

//slider_constraint.h

#ifndef DYN_SLIDER_CONSTRAINT_H
#define DYN_SLIDER_CONSTRAINT_H

#include "shared_ptr.h"
#include "rigid_body.h"
#include "mathUtils.h"

#include "constraint.h"
#include "slider_constraint_impl.h"

class slider_constraint_t: public constraint_t
{
public:
    //typedefs
    typedef shared_ptr<slider_constraint_t> pointer;

    //
    rigid_body_t::pointer rigid_bodyA()  {   return m_rigid_bodyA;   }
    rigid_body_t::pointer rigid_bodyB()  {   return m_rigid_bodyB;   }

    //
    void set_pivot(vec3f const& p) 
    {
        slider_constraint_impl_t* slider_impl = dynamic_cast<slider_constraint_impl_t*>(impl());
        slider_impl->set_pivot(p);
    }

    //local space pivot
    void get_pivot(vec3f& p) const  {    
        slider_constraint_impl_t const* slider_impl = dynamic_cast<slider_constraint_impl_t const*>(impl());
        slider_impl->get_pivot(p);   
    }

    //
    void get_world_pivot(vec3f& p) const  {    
        slider_constraint_impl_t const* slider_impl = dynamic_cast<slider_constraint_impl_t const*>(impl());
        slider_impl->get_world_pivot(p);   
    }

    //
    void set_damping(float d) {
        slider_constraint_impl_t* slider_impl = dynamic_cast<slider_constraint_impl_t*>(impl());
        slider_impl->set_damping(d);
    }

    float damping() const {
        slider_constraint_impl_t const* slider_impl = dynamic_cast<slider_constraint_impl_t const*>(impl());
        return slider_impl->damping();  
    }

	void set_world(vec3f const& p) 
    {
        slider_constraint_impl_t* slider_impl = dynamic_cast<slider_constraint_impl_t*>(impl());
        slider_impl->set_world(p);
    }

    //local space pivot
    void get_world(vec3f& p) const  {    
        slider_constraint_impl_t const* slider_impl = dynamic_cast<slider_constraint_impl_t const*>(impl());
        slider_impl->get_world(p);   
    }

	void set_LinLimit(float lower, float upper) {
        slider_constraint_impl_t* slider_impl = dynamic_cast<slider_constraint_impl_t*>(impl());
        slider_impl->set_LinLimit(lower, upper);
    }

	void set_AngLimit(float lower, float upper) {
        slider_constraint_impl_t* slider_impl = dynamic_cast<slider_constraint_impl_t*>(impl());
        slider_impl->set_AngLimit(lower, upper);
    }

public:
    virtual ~slider_constraint_t() {};

protected:
    friend class solver_t;    
    slider_constraint_t(slider_constraint_impl_t* impl, rigid_body_t::pointer& rigid_bodyA, rigid_body_t::pointer& rigid_bodyB): 
        constraint_t(impl),
        m_rigid_bodyA(rigid_bodyA), 
        m_rigid_bodyB(rigid_bodyB) 
    {
    }

private:
    rigid_body_t::pointer                   m_rigid_bodyA;
    rigid_body_t::pointer                   m_rigid_bodyB;
};



#endif //DYN_SLIDER_CONSTRAINT_H
