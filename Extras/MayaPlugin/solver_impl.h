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

//solver_impl.h

#ifndef DYN_SOLVER_IMPL_H
#define DYN_SOLVER_IMPL_H

#include "rigid_body_impl.h"
#include "constraint/nail_constraint_impl.h"
#include "collision_shape_impl.h"

#include "constraint/hinge_constraint_impl.h"
#include "constraint/slider_constraint_impl.h"
#include "constraint/sixdof_constraint_impl.h"

class solver_impl_t
{
public:
    virtual collision_shape_impl_t* create_sphere_shape(float radius) = 0;

    virtual collision_shape_impl_t* create_plane_shape(vec3f const& normal, float d) = 0;

    virtual collision_shape_impl_t* create_box_shape(vec3f const& halfExtents) = 0;

    virtual collision_shape_impl_t* create_convex_hull_shape(vec3f const* vertices, size_t num_vertices,
                                                             vec3f const* normals,
                                                             unsigned int const *indices, size_t num_indices) = 0; 

    virtual collision_shape_impl_t* create_mesh_shape(vec3f const* vertices, size_t num_vertices,
                                                             vec3f const* normals,
                                                             unsigned int const *indices, size_t num_indices) = 0; 

    virtual rigid_body_impl_t* create_rigid_body(collision_shape_impl_t* cs) = 0;

    virtual nail_constraint_impl_t* create_nail_constraint(rigid_body_impl_t* rb, vec3f const& pivot) = 0;
    virtual nail_constraint_impl_t* create_nail_constraint(rigid_body_impl_t* rbA, rigid_body_impl_t* rbB, vec3f const& pivot) = 0;
    virtual hinge_constraint_impl_t* create_hinge_constraint(rigid_body_impl_t* rb, vec3f const& pivot) = 0;
    virtual slider_constraint_impl_t* create_slider_constraint(rigid_body_impl_t* rbA, vec3f const& pivotA, rigid_body_impl_t* rbB, vec3f const& pivotB) = 0;
    virtual sixdof_constraint_impl_t* create_sixdof_constraint(rigid_body_impl_t* rbA, vec3f const& pivotA, rigid_body_impl_t* rbB, vec3f const& pivotB) = 0;

    virtual void add_rigid_body(rigid_body_impl_t* rb) = 0;

    virtual void remove_rigid_body(rigid_body_impl_t* rb) = 0;

    virtual void add_constraint(constraint_impl_t* rb) = 0;

    virtual void remove_constraint(constraint_impl_t* rb) = 0;

    virtual void set_gravity(vec3f const& g) = 0;

    virtual void set_split_impulse(bool enabled) = 0;

    virtual void export_collada_file(const char* fileName) = 0;

    virtual void import_collada_file(const char* filename) = 0;

    virtual void step_simulation(float dt) = 0;

public:
    virtual ~solver_impl_t() { }
};

#endif

