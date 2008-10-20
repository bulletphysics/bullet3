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

//solver.h

//basic class for all solvers

#ifndef DYN_SOLVER_H
#define DYN_SOLVER_H

#include <set>

#include "mathUtils.h"
#include "shared_ptr.h"

#include "rigid_body.h"
#include "sphere_shape.h"
#include "plane_shape.h"
#include "box_shape.h"
#include "convex_hull_shape.h"
#include "mesh_shape.h"
#include "nail_constraint.h"
#include "solver_impl.h"

class solver_t
{
public:
    static void initialize();
    static void cleanup();

    //creation methods
    static sphere_shape_t::pointer create_sphere_shape(float radius = 1.0); 

    static plane_shape_t::pointer create_plane_shape(vec3f const& normal = vec3f(0, 1, 0), float d = 0); 

    static box_shape_t::pointer create_box_shape(vec3f const& halfExtents = vec3f(0.5f, 0.5f, 0.5f)); 

    static convex_hull_shape_t::pointer create_convex_hull_shape(vec3f const* vertices, size_t num_vertices,
                                                                 vec3f const* normals,
                                                                 unsigned int const *indices, size_t num_indices); 

    static mesh_shape_t::pointer create_mesh_shape(vec3f const* vertices, size_t num_vertices,
                                                   vec3f const* normals,
                                                   unsigned int const *indices, size_t num_indices); 

    static rigid_body_t::pointer create_rigid_body(collision_shape_t::pointer& cs);

    static nail_constraint_t::pointer create_nail_constraint(rigid_body_t::pointer& rb, vec3f const& pivot = vec3f(0, 0, 0));

    //add/remove from world
    static void add_rigid_body(rigid_body_t::pointer& rb);
    static void remove_rigid_body(rigid_body_t::pointer& rb);
    static void remove_all_rigid_bodies();

    //add/remove from world
    static void add_constraint(constraint_t::pointer& c);
    static void remove_constraint(constraint_t::pointer& c);
    static void remove_all_constraints();

    //
    static void set_gravity(vec3f const& g);

    //
    static void set_split_impulse(bool enabled); 

    //
    static void step_simulation(float dt);

    static shared_ptr<solver_impl_t> get_solver();

private:
    static shared_ptr<solver_impl_t> m_impl;
    static std::set<rigid_body_t::pointer> m_rigid_bodies;
    static std::set<constraint_t::pointer> m_constraints;
};



#endif

