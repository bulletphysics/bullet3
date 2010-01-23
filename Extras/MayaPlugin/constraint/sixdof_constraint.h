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

//sixdof_constraint.h

#ifndef DYN_SIXDOF_CONSTRAINT_H
#define DYN_SIXDOF_CONSTRAINT_H

#include "shared_ptr.h"
#include "rigid_body.h"
#include "mathUtils.h"

#include "constraint.h"
#include "sixdof_constraint_impl.h"

class sixdof_constraint_t: public constraint_t
{
public:
    //typedefs
    typedef shared_ptr<sixdof_constraint_t> pointer;

    //
    rigid_body_t::pointer rigid_bodyA()  {   return m_rigid_bodyA;   }
    rigid_body_t::pointer rigid_bodyB()  {   return m_rigid_bodyB;   }

    //
    void set_damping(float d) {
        sixdof_constraint_impl_t* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t*>(impl());
        sixdof_impl->set_damping(d);
    }

    float damping() const {
        sixdof_constraint_impl_t const* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t const*>(impl());
        return sixdof_impl->damping();  
    }

	void set_LinLimit(vec3f& lower, vec3f& upper) {
        sixdof_constraint_impl_t* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t*>(impl());
        sixdof_impl->set_LinLimit(lower, upper);
    }

	void set_AngLimit(vec3f& lower, vec3f& upper) {
        sixdof_constraint_impl_t* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t*>(impl());
        sixdof_impl->set_AngLimit(lower, upper);
    }

    void get_frameA(vec3f& p, quatf& r) const {    
        sixdof_constraint_impl_t const* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t const*>(impl());
		sixdof_impl->get_frameA(p, r);
	} 
    void get_frameB(vec3f& p, quatf& r) const {    
        sixdof_constraint_impl_t const* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t const*>(impl());
		sixdof_impl->get_frameB(p, r);
	} 
    void get_invFrameA(vec3f& p, quatf& r) const {    
        sixdof_constraint_impl_t const* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t const*>(impl());
		sixdof_impl->get_invFrameA(p, r);
	} 
    void get_invFrameB(vec3f& p, quatf& r) const {    
        sixdof_constraint_impl_t const* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t const*>(impl());
		sixdof_impl->get_invFrameB(p, r);
	} 
    void worldToA(vec3f& w, vec3f& p) const {    
        sixdof_constraint_impl_t const* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t const*>(impl());
		sixdof_impl->worldToA(w, p);
	} 
    void worldFromB(vec3f& p, vec3f& w) const {    
        sixdof_constraint_impl_t const* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t const*>(impl());
		sixdof_impl->worldFromB(p, w);
	} 
	void set_world(vec3f const& p, quatf const& r) 
    {
        sixdof_constraint_impl_t* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t*>(impl());
        sixdof_impl->set_world(p, r);
    }

    //local space pivot
    void get_world(vec3f& p, quatf& r) const  {    
        sixdof_constraint_impl_t const* sixdof_impl = dynamic_cast<sixdof_constraint_impl_t const*>(impl());
        sixdof_impl->get_world(p, r);   
    }





public:
    virtual ~sixdof_constraint_t() {};

protected:
    friend class solver_t;    
    sixdof_constraint_t(sixdof_constraint_impl_t* impl, rigid_body_t::pointer& rigid_body): 
        constraint_t(impl),
        m_rigid_bodyA(rigid_body) 
    {
    }
    sixdof_constraint_t(sixdof_constraint_impl_t* impl, rigid_body_t::pointer& rigid_bodyA, rigid_body_t::pointer& rigid_bodyB): 
        constraint_t(impl),
        m_rigid_bodyA(rigid_bodyA),
        m_rigid_bodyB(rigid_bodyB) 
    {
    }

private:
    rigid_body_t::pointer                   m_rigid_bodyA;
    rigid_body_t::pointer                   m_rigid_bodyB;
};



#endif //DYN_SIXDOF_CONSTRAINT_H
