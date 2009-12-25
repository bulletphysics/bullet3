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

Modified by Roman Ponomarev <rponom@gmail.com>
12/24/2009 : Nail constraint improvements

*/

//nail_constraint.h

#ifndef DYN_NAIL_CONSTRAINT_H
#define DYN_NAIL_CONSTRAINT_H

#include "shared_ptr.h"
#include "rigid_body.h"
#include "mathUtils.h"

#include "constraint.h"
#include "nail_constraint_impl.h"

class nail_constraint_t: public constraint_t
{
public:
    //typedefs
    typedef shared_ptr<nail_constraint_t> pointer;

    //
    rigid_body_t::pointer rigid_bodyA()  {   return m_rigid_bodyA;   }
    rigid_body_t::pointer rigid_bodyB()  {   return m_rigid_bodyB;   }

    //
    void set_pivotA(vec3f const& p) {
        nail_constraint_impl_t* nail_impl = dynamic_cast<nail_constraint_impl_t*>(impl());
        nail_impl->set_pivotA(p);
    }

    //local space pivot
    void get_pivotA(vec3f& p) const {    
        nail_constraint_impl_t const* nail_impl = dynamic_cast<nail_constraint_impl_t const*>(impl());
        nail_impl->get_pivotA(p);   
    }

    //
    void set_world(vec3f const& p) {    
        nail_constraint_impl_t* nail_impl = dynamic_cast<nail_constraint_impl_t*>(impl());
        nail_impl->set_world(p);   
    }

	//
    void get_world(vec3f& p) const  {    
        nail_constraint_impl_t const* nail_impl = dynamic_cast<nail_constraint_impl_t const*>(impl());
        nail_impl->get_world(p);   
    }

    //
    void set_damping(float d) {
        nail_constraint_impl_t* nail_impl = dynamic_cast<nail_constraint_impl_t*>(impl());
        nail_impl->set_damping(d);
    }

    float damping() const {
        nail_constraint_impl_t const* nail_impl = dynamic_cast<nail_constraint_impl_t const*>(impl());
        return nail_impl->damping();  
    }

public:
    virtual ~nail_constraint_t() {};

protected:
    friend class solver_t;    
    nail_constraint_t(nail_constraint_impl_t* impl, rigid_body_t::pointer& rigid_bodyA): 
        constraint_t(impl),
        m_rigid_bodyA(rigid_bodyA),
		m_rigid_bodyB(NULL)
    {
    }

	nail_constraint_t(nail_constraint_impl_t* impl, rigid_body_t::pointer& rigid_bodyA, rigid_body_t::pointer& rigid_bodyB): 
        constraint_t(impl),
        m_rigid_bodyA(rigid_bodyA),
		m_rigid_bodyB(rigid_bodyB)
    {
    }

private:
    rigid_body_t::pointer                   m_rigid_bodyA;
    rigid_body_t::pointer                   m_rigid_bodyB;
};



#endif
