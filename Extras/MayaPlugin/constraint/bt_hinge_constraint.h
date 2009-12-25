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
12/24/2009 : Nail constraint improvements

*/

//bt_hinge_constraint.h

#ifndef DYN_BT_HINGE_CONSTRAINT_H
#define DYN_BT_HINGE_CONSTRAINT_H

#include "bt_constraint.h"
#include "hinge_constraint_impl.h"

class bt_hinge_constraint_t: public bt_constraint_t, public hinge_constraint_impl_t {
public:

    virtual void set_damping(float d) {
//        btHingeConstraint* p2pc = static_cast<btHingeConstraint*>(m_constraint.get());
//        p2pc->m_setting.m_damping = d;
    }

	virtual void set_limit(float lower, float upper, float softness, float bias_factor, float relaxation_factor) {
        btHingeConstraint* hinge = static_cast<btHingeConstraint*>(m_constraint.get());
		hinge->setLimit(lower, upper, softness, bias_factor, relaxation_factor);
    }

	virtual void set_axis(vec3f const &p) {
        btHingeConstraint* hinge = static_cast<btHingeConstraint*>(m_constraint.get());
        btVector3 axis(p[0], p[1], p[2]);
		hinge->setAxis(axis);
    }

    virtual float damping() const {
//        btHingeConstraint const* hc = static_cast<btHingeConstraint const*>(m_constraint.get());
//        return hc->m_setting.m_damping;
		return 0;
    }

    //
    virtual void set_pivot(vec3f const &p) {
/*        btHingeConstraint* p2pc = static_cast<btHingeConstraint*>(m_constraint.get());
        btVector3 bt_pivot(p[0], p[1], p[2]);
        p2pc->setPivotA(bt_pivot); 
        p2pc->setPivotB(m_constraint->getRigidBodyA().getCenterOfMassTransform()(bt_pivot));
       // p2pc->buildJacobian();
*/    }

    virtual void get_pivot(vec3f &p) const {
/*        btHingeConstraint const* hc = static_cast<btHingeConstraint const*>(m_constraint.get());
        p[0] = hc->getPivotInA().x(); 
        p[1] = hc->getPivotInA().y(); 
        p[2] = hc->getPivotInA().z(); 
*/    }

    virtual void get_world_pivot(vec3f &p) const {
/*        btHingeConstraint const* hc = static_cast<btHingeConstraint const*>(m_constraint.get());
        p[0] = hc->getPivotInB().x(); 
        p[1] = hc->getPivotInB().y(); 
        p[2] = hc->getPivotInB().z(); 
*/    }

	virtual void set_world(vec3f const &p) {
        btHingeConstraint* hc = static_cast<btHingeConstraint*>(m_constraint.get());
        btVector3 world(p[0], p[1], p[2]);
		btVector3 pivotA = hc->getRigidBodyA().getWorldTransform().inverse() (world);
        hc->getAFrame().getOrigin() = pivotA; 
        hc->getBFrame().getOrigin() = world;
       // p2pc->buildJacobian();
    }

	virtual void get_world(vec3f &p) const {
        btHingeConstraint const* hc = static_cast<btHingeConstraint const*>(m_constraint.get());
        p[0] = hc->getBFrame().getOrigin().x(); 
        p[1] = hc->getBFrame().getOrigin().y(); 
        p[2] = hc->getBFrame().getOrigin().z(); 
    }

	virtual void enable_motor(bool enable, float velocity, float impulse) {
        btHingeConstraint* hinge = static_cast<btHingeConstraint*>(m_constraint.get());
		hinge->enableAngularMotor(enable, velocity, impulse);
	}

	virtual void update_constraint(rigid_body_impl_t* rb)
	{
        btHingeConstraint* hc = static_cast<btHingeConstraint*>(m_constraint.get());
		btVector3 world = hc->getBFrame().getOrigin();
		btVector3 pivotA = hc->getRigidBodyA().getWorldTransform().inverse() (world);
        hc->getAFrame().getOrigin() = pivotA; 
	}
protected:
    friend class bt_solver_t;

    bt_hinge_constraint_t(rigid_body_impl_t* rb, vec3f const& pivot): 
        hinge_constraint_impl_t()
    {
        btRigidBody& bt_body = *static_cast<bt_rigid_body_t*>(rb)->body();
		btVector3 pivotA = bt_body.getCenterOfMassPosition();
		btVector3 btAxisA( 0.0f, 1.0f, 0.0f ); // pointing upwards, aka Y-axis
		btHingeConstraint * hinge = new btHingeConstraint(bt_body, -pivotA, btAxisA);
        m_constraint.reset(hinge);
		rb->add_constraint(this);
    }

private:

};

#endif //DYN_BT_HINGE_CONSTRAINT_H
