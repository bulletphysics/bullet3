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
01/22/2010 : Constraints reworked
01/27/2010 : Replaced COLLADA export with Bullet binary export
*/

//bt_solver.h

#ifndef DYN_BT_SOLVER_H
#define DYN_BT_SOLVER_H

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

#include "solver_impl.h"
#include "bt_rigid_body.h"
#include "bt_sphere_shape.h"
#include "bt_plane_shape.h"
#include "bt_box_shape.h"
#include "bt_convex_hull_shape.h"
#include "bt_mesh_shape.h"
#include "constraint/bt_nail_constraint.h"
#include "constraint/bt_hinge_constraint.h"
#include "constraint/bt_slider_constraint.h"
#include "constraint/bt_sixdof_constraint.h"

class bt_solver_t: public solver_impl_t
{
public:
    virtual rigid_body_impl_t* create_rigid_body(collision_shape_impl_t* cs) {
        return new bt_rigid_body_t(cs);
    }

    virtual collision_shape_impl_t* create_sphere_shape(float radius) {
        return new bt_sphere_shape_t(radius);
    }

    virtual collision_shape_impl_t* create_plane_shape(vec3f const& normal, float d) {
        return new bt_plane_shape_t(normal, d);
    }

    virtual collision_shape_impl_t* create_box_shape(vec3f const& halfExtents) {
        return new bt_box_shape_t(halfExtents);
    }

    virtual collision_shape_impl_t* create_convex_hull_shape(vec3f const* vertices, size_t num_vertices,
                                                             vec3f const* normals,
                                                             unsigned int const *indices, size_t num_indices)
    {
        return new bt_convex_hull_shape_t(vertices, num_vertices, normals, indices, num_indices);
    }

    virtual collision_shape_impl_t* create_mesh_shape(vec3f const* vertices, size_t num_vertices,
                                                      vec3f const* normals,
                                                      unsigned int const *indices, size_t num_indices)
    {
        return new bt_mesh_shape_t(vertices, num_vertices, normals, indices, num_indices);
    }

    virtual nail_constraint_impl_t* create_nail_constraint(rigid_body_impl_t* rb, vec3f const& pivot)
    {
        return new bt_nail_constraint_t(rb, pivot);
    }
    virtual nail_constraint_impl_t* create_nail_constraint(rigid_body_impl_t* rbA, rigid_body_impl_t* rbB, vec3f const& pivotInA, vec3f const& pivotInB)
    {
        return new bt_nail_constraint_t(rbA, rbB, pivotInA, pivotInB);
    }
    virtual hinge_constraint_impl_t* create_hinge_constraint(rigid_body_impl_t* rb, vec3f const& pivot, quatf const& rot)
    {
        return new bt_hinge_constraint_t(rb, pivot, rot);
    }
    virtual hinge_constraint_impl_t* create_hinge_constraint(rigid_body_impl_t* rbA, vec3f const& pivotA, quatf const& rotA, rigid_body_impl_t* rbB, vec3f const& pivotB, quatf const& rotB)
    {
        return new bt_hinge_constraint_t(rbA, pivotA, rotA, rbB, pivotB, rotB);
    }
    virtual slider_constraint_impl_t* create_slider_constraint(rigid_body_impl_t* rb, vec3f const& pivot, quatf const& rot)
    {
        return new bt_slider_constraint_t(rb, pivot, rot);
    }
    virtual slider_constraint_impl_t* create_slider_constraint(rigid_body_impl_t* rbA, vec3f const& pivotA, quatf const& rotA, rigid_body_impl_t* rbB, vec3f const& pivotB, quatf const& rotB)
    {
        return new bt_slider_constraint_t(rbA, pivotA, rotA, rbB, pivotB, rotB);
    }
    virtual sixdof_constraint_impl_t* create_sixdof_constraint(rigid_body_impl_t* rb, vec3f const& pivot, quatf const& rot)
    {
        return new bt_sixdof_constraint_t(rb, pivot, rot);
    }
    virtual sixdof_constraint_impl_t* create_sixdof_constraint(rigid_body_impl_t* rbA, vec3f const& pivotA, quatf const& rotA, rigid_body_impl_t* rbB, vec3f const& pivotB, quatf const& rotB)
    {
        return new bt_sixdof_constraint_t(rbA, pivotA, rotA, rbB, pivotB, rotB);
    }

    virtual void add_rigid_body(rigid_body_impl_t* rb)
    {
        bt_rigid_body_t* bt_body = static_cast<bt_rigid_body_t*>(rb);
		bt_body->body()->setActivationState(DISABLE_DEACTIVATION);
        m_dynamicsWorld->addRigidBody(bt_body->body());
    }

    virtual void remove_rigid_body(rigid_body_impl_t* rb)
    {
        bt_rigid_body_t* bt_body = static_cast<bt_rigid_body_t*>(rb);
        m_dynamicsWorld->removeRigidBody(bt_body->body());
    }

    virtual void add_constraint(constraint_impl_t* c)
    {
        bt_constraint_t* btc = dynamic_cast<bt_constraint_t*>(c);
        m_dynamicsWorld->addConstraint(btc->constraint(), true);
    }

    virtual void remove_constraint(constraint_impl_t* c)
    {
        bt_constraint_t* btc = dynamic_cast<bt_constraint_t*>(c);
        m_dynamicsWorld->removeConstraint(btc->constraint());
    }

    virtual void set_gravity(vec3f const& g)
    {
        m_dynamicsWorld->setGravity(btVector3(g[0], g[1], g[2]));
    }

    virtual void set_split_impulse(bool enabled)
    {
        m_dynamicsWorld->getSolverInfo().m_splitImpulse = enabled;
    }

    virtual void step_simulation(float dt) 
    {
        m_dynamicsWorld->stepSimulation(dt, 1000, 1.0f / 120.0f);
    }

	virtual void debug_draw(int dbgMode);

    virtual void export_bullet_file(const char* fileName);

    virtual void import_bullet_file(const char* filename);


protected:
    friend class solver_t;
    bt_solver_t();

private:

    shared_ptr<btBroadphaseInterface>            m_broadphase;
    shared_ptr<btConstraintSolver>               m_solver;
    shared_ptr<btDefaultCollisionConfiguration>  m_collisionConfiguration;
    shared_ptr<btCollisionDispatcher>            m_dispatcher;
    shared_ptr<btDiscreteDynamicsWorld>          m_dynamicsWorld;
};

#endif
