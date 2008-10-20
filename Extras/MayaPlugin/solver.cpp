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

//solver.cpp

#include "solver.h"
#include "bt_solver.h"

shared_ptr<solver_impl_t> solver_t::m_impl;
std::set<rigid_body_t::pointer> solver_t::m_rigid_bodies;
std::set<constraint_t::pointer> solver_t::m_constraints;

shared_ptr<solver_impl_t> solver_t::get_solver()
{
    return m_impl;
}



void solver_t::initialize()
{
    m_impl.reset(new bt_solver_t);
}

void solver_t::cleanup()
{
    
}

//creation methods
sphere_shape_t::pointer solver_t::create_sphere_shape(float radius)
{
    return sphere_shape_t::pointer(new sphere_shape_t(m_impl->create_sphere_shape(radius)));
}

plane_shape_t::pointer solver_t::create_plane_shape(vec3f const& normal, float d)
{
    return plane_shape_t::pointer(new plane_shape_t(m_impl->create_plane_shape(normal, d)));
}

box_shape_t::pointer solver_t::create_box_shape(vec3f const& halfExtents)
{
    return box_shape_t::pointer(new box_shape_t(m_impl->create_box_shape(halfExtents)));
}

convex_hull_shape_t::pointer solver_t::create_convex_hull_shape(vec3f const* vertices, size_t num_vertices,
                                                                vec3f const* normals,
                                                             unsigned int const *indices, size_t num_indices)
{
    return convex_hull_shape_t::pointer(new convex_hull_shape_t(m_impl->create_convex_hull_shape(vertices, num_vertices,
                                                                                                 normals,
                                                                                                 indices, num_indices)));
}

mesh_shape_t::pointer solver_t::create_mesh_shape(vec3f const* vertices, size_t num_vertices,
                                                  vec3f const* normals,
                                                  unsigned int const *indices, size_t num_indices)
{
    return mesh_shape_t::pointer(new mesh_shape_t(m_impl->create_mesh_shape(vertices, num_vertices,
                                                                            normals,
                                                                            indices, num_indices)));
}

rigid_body_t::pointer solver_t::create_rigid_body(collision_shape_t::pointer& cs)
{
    return rigid_body_t::pointer(new rigid_body_t(m_impl->create_rigid_body(cs->impl()), cs));
}

nail_constraint_t::pointer  solver_t::create_nail_constraint(rigid_body_t::pointer& rb, vec3f const& pivot)
{
    return nail_constraint_t::pointer(new nail_constraint_t(m_impl->create_nail_constraint(rb->impl(), pivot), rb));
}


//add/remove from world
void solver_t::add_rigid_body(rigid_body_t::pointer& rb)
{
    if(rb) {
        if(m_rigid_bodies.find(rb) == m_rigid_bodies.end()) {
            m_rigid_bodies.insert(rb);
            m_impl->add_rigid_body(rb->impl());
        }
    }
}

void solver_t::remove_rigid_body(rigid_body_t::pointer& rb)
{
    if(rb) {
        if(m_rigid_bodies.find(rb) != m_rigid_bodies.end()) {
            m_impl->remove_rigid_body(rb->impl());
            m_rigid_bodies.erase(rb);
        }
    }
}

void solver_t::remove_all_rigid_bodies()
{
    std::set<rigid_body_t::pointer>::iterator it;
    for(it = m_rigid_bodies.begin(); it != m_rigid_bodies.end(); ++it) { 
        m_impl->remove_rigid_body(const_cast<rigid_body_t*>((*it).get())->impl());
    }
    m_rigid_bodies.clear();
}

void solver_t::add_constraint(constraint_t::pointer& c)
{
    if(c) {
        if(m_constraints.find(c) == m_constraints.end()) {
            m_constraints.insert(c);
            m_impl->add_constraint(c->impl());
        }
    }
}

void solver_t::remove_constraint(constraint_t::pointer& c)
{
    if(c) {
        if(m_constraints.find(c) != m_constraints.end()) {
            m_impl->remove_constraint(c->impl());
            m_constraints.erase(c);
        }
    }
}

void solver_t::remove_all_constraints()
{
    std::set<constraint_t::pointer>::iterator it;
    for(it = m_constraints.begin(); it != m_constraints.end(); ++it) { 
        m_impl->remove_constraint(const_cast<constraint_t*>((*it).get())->impl());
    }
    m_constraints.clear();
}

void solver_t::set_gravity(vec3f const& g)
{
    m_impl->set_gravity(g);
}

void solver_t::set_split_impulse(bool enabled)
{
    m_impl->set_split_impulse(enabled);
}

void solver_t::step_simulation(float dt)
{
    m_impl->step_simulation(dt);
}
