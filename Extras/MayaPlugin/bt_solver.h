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

//bt_solver.h

#ifndef DYN_BT_SOLVER_H
#define DYN_BT_SOLVER_H

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
#include "GIMPACT/Bullet/btGImpactShape.h"
#include "GIMPACT/Bullet/btGImpactCollisionAlgorithm.h"

#include "solver_impl.h"
#include "bt_rigid_body.h"
#include "bt_sphere_shape.h"
#include "bt_plane_shape.h"
#include "bt_box_shape.h"
#include "bt_convex_hull_shape.h"
#include "bt_mesh_shape.h"

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

    virtual void add_rigid_body(rigid_body_impl_t* rb)
    {
        bt_rigid_body_t* bt_body = static_cast<bt_rigid_body_t*>(rb);
        m_dynamicsWorld->addRigidBody(bt_body->body());
    }

    virtual void remove_rigid_body(rigid_body_impl_t* rb)
    {
        bt_rigid_body_t* bt_body = static_cast<bt_rigid_body_t*>(rb);
        m_dynamicsWorld->removeRigidBody(bt_body->body());
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
        m_dynamicsWorld->stepSimulation(dt, 1000, 1.0 / 120.0);
    }

    virtual void export_collada_file(const char* fileName);

    virtual void import_collada_file(const char* filename);


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
