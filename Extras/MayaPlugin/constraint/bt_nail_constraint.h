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

//bt_nail_constraint.h

#ifndef DYN_BT_NAIL_CONSTRAINT_H
#define DYN_BT_NAIL_CONSTRAINT_H

#include "bt_constraint.h"
#include "nail_constraint_impl.h"

class bt_nail_constraint_t: public bt_constraint_t, public nail_constraint_impl_t {
public:

    virtual void set_damping(float d) {
        btPoint2PointConstraint* p2pc = static_cast<btPoint2PointConstraint*>(m_constraint.get());
        p2pc->m_setting.m_damping = d;
    }

    virtual float damping() const {
        btPoint2PointConstraint const* p2pc = static_cast<btPoint2PointConstraint const*>(m_constraint.get());
        return p2pc->m_setting.m_damping;
    }

    //
    virtual void set_pivotA(vec3f const &p) {
        btPoint2PointConstraint* p2pc = static_cast<btPoint2PointConstraint*>(m_constraint.get());
        btVector3 bt_pivot(p[0], p[1], p[2]);
        p2pc->setPivotA(bt_pivot); 
        p2pc->setPivotB(m_constraint->getRigidBodyA().getCenterOfMassTransform()(bt_pivot));
       // p2pc->buildJacobian();
    }

    virtual void get_pivotA(vec3f &p) const {
        btPoint2PointConstraint const* p2pc = static_cast<btPoint2PointConstraint const*>(m_constraint.get());
        p[0] = p2pc->getPivotInA().x(); 
        p[1] = p2pc->getPivotInA().y(); 
        p[2] = p2pc->getPivotInA().z(); 
    }

    virtual void get_world_pivot(vec3f &p) const {
        btPoint2PointConstraint const* p2pc = static_cast<btPoint2PointConstraint const*>(m_constraint.get());
        p[0] = p2pc->getPivotInB().x(); 
        p[1] = p2pc->getPivotInB().y(); 
        p[2] = p2pc->getPivotInB().z(); 
    }

	virtual void set_world(vec3f const &p) {
        btPoint2PointConstraint* p2pc = static_cast<btPoint2PointConstraint*>(m_constraint.get());
        btVector3 world(p[0], p[1], p[2]);
		btVector3 pivotA = p2pc->getRigidBodyA().getWorldTransform().inverse() (world);
        p2pc->setPivotA(pivotA); 
        p2pc->setPivotB(world);
		p2pc->buildJacobian();
    }

    virtual void get_world(vec3f &p) const {
        btPoint2PointConstraint const* p2pc = static_cast<btPoint2PointConstraint const*>(m_constraint.get());
        p[0] = p2pc->getPivotInB().x(); 
        p[1] = p2pc->getPivotInB().y(); 
        p[2] = p2pc->getPivotInB().z(); 
    }

	virtual void update_constraint()
	{
        btPoint2PointConstraint* p2pc = static_cast<btPoint2PointConstraint*>(m_constraint.get());
		btVector3 world = p2pc->getPivotInB();
		btVector3 pivotA = p2pc->getRigidBodyA().getWorldTransform().inverse() (world);
        p2pc->setPivotA(pivotA); 
	}
protected:
    friend class bt_solver_t;

    bt_nail_constraint_t(rigid_body_impl_t* rb, vec3f const& pivot): 
        nail_constraint_impl_t()
    {
        btRigidBody& bt_body = *static_cast<bt_rigid_body_t*>(rb)->body();
		btVector3 pivotA = bt_body.getCenterOfMassPosition();
        m_constraint.reset(new btPoint2PointConstraint(bt_body, -pivotA));
		rb->add_constraint(this);
    }

private:

};

#endif
