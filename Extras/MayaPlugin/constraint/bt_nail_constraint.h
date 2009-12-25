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
//        btPoint2PointConstraint const* p2pc = static_cast<btPoint2PointConstraint const*>(m_constraint.get());
//        p[0] = p2pc->getPivotInB().x(); 
//        p[1] = p2pc->getPivotInB().y(); 
//        p[2] = p2pc->getPivotInB().z(); 
        btPoint2PointConstraint const* p2pc = static_cast<btPoint2PointConstraint const*>(m_constraint.get());
		btVector3 pivotAinW = p2pc->getRigidBodyA().getCenterOfMassTransform()* p2pc->getPivotInA();
		p[0] = pivotAinW.x();
		p[1] = pivotAinW.y();
		p[2] = pivotAinW.z();
    }

	virtual void set_world(vec3f const &p) {
        btPoint2PointConstraint* p2pc = static_cast<btPoint2PointConstraint*>(m_constraint.get());
        btVector3 world(p[0], p[1], p[2]);
		btVector3 pivotA = p2pc->getRigidBodyA().getWorldTransform().inverse() (world);
        p2pc->setPivotA(pivotA); 
		btVector3 pivotB = p2pc->getRigidBodyB().getWorldTransform().inverse() (world);
        p2pc->setPivotB(pivotB);
//		p2pc->buildJacobian();
    }

    virtual void get_world(vec3f &p) const {
        btPoint2PointConstraint const* p2pc = static_cast<btPoint2PointConstraint const*>(m_constraint.get());
		btVector3 pivotAinW = p2pc->getRigidBodyA().getCenterOfMassTransform()* p2pc->getPivotInA();
//        p[0] = p2pc->getPivotInB().x(); 
//      p[1] = p2pc->getPivotInB().y(); 
//        p[2] = p2pc->getPivotInB().z(); 
		p[0] = pivotAinW.x();
		p[1] = pivotAinW.y();
		p[2] = pivotAinW.z();
    }

	virtual void update_constraint(rigid_body_impl_t* rb)
	{
        btRigidBody* bt_body = static_cast<bt_rigid_body_t*>(rb)->body();
        btPoint2PointConstraint* p2pc = static_cast<btPoint2PointConstraint*>(m_constraint.get());
		btVector3 world, pivot;
		if(bt_body == &p2pc->getRigidBodyA())
		{
			world = p2pc->getRigidBodyB().getWorldTransform() * p2pc->getPivotInB();
			pivot = p2pc->getRigidBodyA().getWorldTransform().inverse() * world;
			p2pc->setPivotA(pivot);
		}
		else if(bt_body == &p2pc->getRigidBodyB())
		{
			world = p2pc->getRigidBodyA().getWorldTransform() * p2pc->getPivotInA();
			pivot = p2pc->getRigidBodyB().getWorldTransform().inverse() * world;
			p2pc->setPivotB(pivot);
		}
		else
		{
			world.setValue(0.f, 0.f, 0.f);
		}

//		btVector3 world = p2pc->getPivotInB();
//		btVector3 pivotA = p2pc->getRigidBodyA().getWorldTransform().inverse() (world);
//      p2pc->setPivotA(pivotA); 
	}
protected:
    friend class bt_solver_t;

    bt_nail_constraint_t(rigid_body_impl_t* rb, vec3f const& pivot): 
        nail_constraint_impl_t()
    {
		btVector3 pivotW(pivot[0], pivot[1], pivot[2]);
        btRigidBody& bt_body = *static_cast<bt_rigid_body_t*>(rb)->body();
		const btTransform& tr = bt_body.getCenterOfMassTransform();
		btTransform iTr = tr.inverse();
		btVector3 nPivot = iTr * pivotW;
        m_constraint.reset(new btPoint2PointConstraint(bt_body, nPivot));
//		btVector3 pivotA = bt_body.getCenterOfMassPosition();
//        m_constraint.reset(new btPoint2PointConstraint(bt_body, -pivotA));
		rb->add_constraint(this);
    }

		bt_nail_constraint_t(rigid_body_impl_t* rbA, rigid_body_impl_t* rbB, vec3f const& pivot): 
        nail_constraint_impl_t()
    {
		btVector3 pivotW(pivot[0], pivot[1], pivot[2]);
        btRigidBody& bt_bodyA = *static_cast<bt_rigid_body_t*>(rbA)->body();
		const btTransform& trA = bt_bodyA.getCenterOfMassTransform();
		btTransform iTrA = trA.inverse();
		btVector3 nPivotA = iTrA * pivotW;
        btRigidBody& bt_bodyB = *static_cast<bt_rigid_body_t*>(rbB)->body();
		const btTransform& trB = bt_bodyB.getCenterOfMassTransform();
		btTransform iTrB = trB.inverse();
		btVector3 nPivotB = iTrB * pivotW;
        m_constraint.reset(new btPoint2PointConstraint(bt_bodyA, bt_bodyB, nPivotA, nPivotB));
		rbA->add_constraint(this);
		rbB->add_constraint(this);
    }

private:

};

#endif
