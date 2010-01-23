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

Modified by Roman Ponomarev <rponom@gmail.com>
01/22/2010 : Constraints reworked
*/

//hinge_constraint.h

#ifndef DYN_HINGE_CONSTRAINT_H
#define DYN_HINGE_CONSTRAINT_H

#include "shared_ptr.h"
#include "rigid_body.h"
#include "mathUtils.h"

#include "constraint.h"
#include "hinge_constraint_impl.h"

class hinge_constraint_t: public constraint_t
{
public:
    //typedefs
    typedef shared_ptr<hinge_constraint_t> pointer;

    //
    rigid_body_t::pointer rigid_bodyA()  {   return m_rigid_bodyA;   }
    rigid_body_t::pointer rigid_bodyB()  {   return m_rigid_bodyB;   }

    //
    void get_frameA(vec3f& p, quatf& r) const {    
        hinge_constraint_impl_t const* hinge_impl = dynamic_cast<hinge_constraint_impl_t const*>(impl());
		hinge_impl->get_frameA(p, r);
	} 
    void get_frameB(vec3f& p, quatf& r) const {    
        hinge_constraint_impl_t const* hinge_impl = dynamic_cast<hinge_constraint_impl_t const*>(impl());
		hinge_impl->get_frameB(p, r);
	} 
    void get_invFrameA(vec3f& p, quatf& r) const {    
        hinge_constraint_impl_t const* hinge_impl = dynamic_cast<hinge_constraint_impl_t const*>(impl());
		hinge_impl->get_invFrameA(p, r);
	} 
    void get_invFrameB(vec3f& p, quatf& r) const {    
        hinge_constraint_impl_t const* hinge_impl = dynamic_cast<hinge_constraint_impl_t const*>(impl());
		hinge_impl->get_invFrameB(p, r);
	} 
    void worldToA(vec3f& w, vec3f& p) const {    
        hinge_constraint_impl_t const* hinge_impl = dynamic_cast<hinge_constraint_impl_t const*>(impl());
		hinge_impl->worldToA(w, p);
	} 
    void worldFromB(vec3f& p, vec3f& w) const {    
        hinge_constraint_impl_t const* hinge_impl = dynamic_cast<hinge_constraint_impl_t const*>(impl());
		hinge_impl->worldFromB(p, w);
	} 

    //
    void set_damping(float d) {
        hinge_constraint_impl_t* hinge_impl = dynamic_cast<hinge_constraint_impl_t*>(impl());
        hinge_impl->set_damping(d);
    }

	void set_limit(float lower, float upper, float softness, float bias_factor, float relaxation_factor) {
        hinge_constraint_impl_t* hinge_impl = dynamic_cast<hinge_constraint_impl_t*>(impl());
        hinge_impl->set_limit(lower, upper, softness, bias_factor, relaxation_factor);
    }

	void set_axis(vec3f const& p) {
        hinge_constraint_impl_t* hinge_impl = dynamic_cast<hinge_constraint_impl_t*>(impl());
        hinge_impl->set_axis(p);
    }

    float damping() const {
        hinge_constraint_impl_t const* hinge_impl = dynamic_cast<hinge_constraint_impl_t const*>(impl());
        return hinge_impl->damping();  
    }

	void set_world(vec3f const& p, quatf const& r) 
    {
        hinge_constraint_impl_t* hinge_impl = dynamic_cast<hinge_constraint_impl_t*>(impl());
        hinge_impl->set_world(p, r);
    }

    //local space pivot
    void get_world(vec3f& p, quatf& r) const  {    
        hinge_constraint_impl_t const* hinge_impl = dynamic_cast<hinge_constraint_impl_t const*>(impl());
        hinge_impl->get_world(p, r);   
    }

	void enable_motor(bool enable, float velocity, float impulse) {
        hinge_constraint_impl_t* hinge_impl = dynamic_cast<hinge_constraint_impl_t*>(impl());
        hinge_impl->enable_motor(enable, velocity, impulse);
    }

public:
    virtual ~hinge_constraint_t() {};

protected:
    friend class solver_t;    
    hinge_constraint_t(hinge_constraint_impl_t* impl, rigid_body_t::pointer& rigid_body): 
        constraint_t(impl),
        m_rigid_bodyA(rigid_body) 
    {
    }
    hinge_constraint_t(hinge_constraint_impl_t* impl, rigid_body_t::pointer& rigid_bodyA, rigid_body_t::pointer& rigid_bodyB): 
        constraint_t(impl),
        m_rigid_bodyA(rigid_bodyA),
        m_rigid_bodyB(rigid_bodyB) 
    {
    }

private:
    rigid_body_t::pointer                   m_rigid_bodyA;
    rigid_body_t::pointer                   m_rigid_bodyB;
};



#endif //DYN_HINGE_CONSTRAINT_H
