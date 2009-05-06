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

//bt_slider_constraint.h

#ifndef DYN_BT_SLIDER_CONSTRAINT_H
#define DYN_BT_SLIDER_CONSTRAINT_H

#include "bt_constraint.h"
#include "slider_constraint_impl.h"

class bt_slider_constraint_t: public bt_constraint_t, public slider_constraint_impl_t {
public:

    virtual void set_damping(float d) {
//        btSliderConstraint* p2pc = static_cast<btSliderConstraint*>(m_constraint.get());
//        p2pc->m_setting.m_damping = d;
    }

    virtual void set_LinLimit(float lower, float upper) {
		btSliderConstraint* slider = static_cast<btSliderConstraint*>(m_constraint.get());
		slider->setLowerLinLimit(lower);
		slider->setUpperLinLimit(upper);
    }

	virtual void set_AngLimit(float lower, float upper) {
		btSliderConstraint* slider = static_cast<btSliderConstraint*>(m_constraint.get());
		slider->setLowerAngLimit(lower);
		slider->setUpperAngLimit(upper);
    }

	virtual float damping() const {
//        btSliderConstraint const* hc = static_cast<btSliderConstraint const*>(m_constraint.get());
//        return hc->m_setting.m_damping;
		return 0;
    }

    //
    virtual void set_pivot(vec3f const &p) {
/*        btSliderConstraint* p2pc = static_cast<btSliderConstraint*>(m_constraint.get());
        btVector3 bt_pivot(p[0], p[1], p[2]);
        p2pc->setPivotA(bt_pivot); 
        p2pc->setPivotB(m_constraint->getRigidBodyA().getCenterOfMassTransform()(bt_pivot));
       // p2pc->buildJacobian();
*/    }

    virtual void get_pivot(vec3f &p) const {
/*        btSliderConstraint const* hc = static_cast<btSliderConstraint const*>(m_constraint.get());
        p[0] = hc->getPivotInA().x(); 
        p[1] = hc->getPivotInA().y(); 
        p[2] = hc->getPivotInA().z(); 
*/    }

    virtual void get_world_pivot(vec3f &p) const {
/*        btSliderConstraint const* hc = static_cast<btSliderConstraint const*>(m_constraint.get());
        p[0] = hc->getPivotInB().x(); 
        p[1] = hc->getPivotInB().y(); 
        p[2] = hc->getPivotInB().z(); 
*/    }

	virtual void set_world(vec3f const &p) {
/*        btSliderConstraint* hc = static_cast<btSliderConstraint*>(m_constraint.get());
        btVector3 world(p[0], p[1], p[2]);
		btVector3 pivotA = hc->getRigidBodyA().getWorldTransform().inverse() (world);
        hc->getFrameOffsetA().getOrigin() = pivotA; 
        hc->getFrameOffsetB().getOrigin() = world;
 */      // p2pc->buildJacobian();
		world = p;
    }

	virtual void get_world(vec3f &p) const {
/*        btSliderConstraint const* hc = static_cast<btSliderConstraint const*>(m_constraint.get());
        p[0] = hc->getFrameOffsetB().getOrigin().x(); 
        p[1] = hc->getFrameOffsetB().getOrigin().y(); 
        p[2] = hc->getFrameOffsetB().getOrigin().z(); 
*/		p = world;
	}

	virtual void update_constraint()
	{
/*        btRigidBody& bt_bodyA = *static_cast<bt_rigid_body_t*>(rbA)->body();
        btRigidBody& bt_bodyB = *static_cast<bt_rigid_body_t*>(rbB)->body();
		btTransform framInA = btTransform::getIdentity();
		btTransform framInB = btTransform::getIdentity();
		btSliderConstraint* slider = static_cast<btSliderConstraint*>(m_constraint.get());
		slider->getFrameOffsetA();		
*/	}

protected:
    friend class bt_solver_t;
	vec3f world;

    bt_slider_constraint_t(rigid_body_impl_t* rbA, vec3f const& pivotA, rigid_body_impl_t* rbB, vec3f const& pivotB): 
        slider_constraint_impl_t()
    {
        btRigidBody& bt_bodyA = *static_cast<bt_rigid_body_t*>(rbA)->body();
        btRigidBody& bt_bodyB = *static_cast<bt_rigid_body_t*>(rbB)->body();
		btTransform framInA = btTransform::getIdentity();
		btTransform framInB = btTransform::getIdentity();
		btSliderConstraint * slider = new btSliderConstraint(bt_bodyA, bt_bodyB, framInA, framInB, true);
		m_constraint.reset(slider);
		rbA->add_constraint(this);
    }

private:

};

#endif //DYN_BT_SLIDER_CONSTRAINT_H
