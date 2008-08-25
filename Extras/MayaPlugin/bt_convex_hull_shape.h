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

//bt_convex_hull_shape.h

#ifndef DYN_BT_CONVEX_HULL_SHAPE_H
#define DYN_BT_CONVEX_HULL_SHAPE_H

#include <vector>

#include "convex_hull_shape_impl.h"
#include "bt_collision_shape.h"

class bt_convex_hull_shape_t: public bt_collision_shape_t, public convex_hull_shape_impl_t 
{
public:
    virtual void gl_draw(size_t draw_style) {

        if(m_vertices.empty() || m_indices.empty()) return;

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        if(draw_style & collision_shape_t::kDSSolid) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        } else {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }
        glVertexPointer(3, GL_FLOAT, 0, &(m_vertices[0]));
        glNormalPointer(GL_FLOAT, 0, &(m_normals[0]));
        glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, &(m_indices[0]));
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    virtual void set_scale(vec3f const& s) {
        shape()->setLocalScaling(btVector3(s[0], s[1], s[2]));
        update();
    }

    virtual void get_scale(vec3f& s) {
        const btVector3& scale = shape()->getLocalScaling();
        s = vec3f(scale.x(), scale.y(), scale.z());
    }

    virtual float volume()                  { return m_volume;  }
    virtual vec3f const& local_inertia()    { return m_local_inertia;  }
    virtual vec3f const& center()           { return m_center; }
    virtual quatf const& rotation()         { return m_rotation;  }

protected:
    friend class bt_solver_t;

    bt_convex_hull_shape_t(vec3f const* vertices, size_t num_vertices,
                           vec3f const* normals,
                            unsigned int const *indices, size_t num_indices): 
                        bt_collision_shape_t(),
                        m_normals(normals, normals + num_vertices),
                        m_indices(indices, indices + num_indices)
    { 
        m_volume = ::volume(vertices, (int*)indices, num_indices);
        m_center = center_of_mass(vertices, (int*)indices, num_indices);
        mat3x3f I = inertia(vertices, (int*)indices, num_indices, m_center);
        m_rotation = diagonalizer(I);

        mat3x3f Q, Qinv; 
        q_to_mat(m_rotation, Q); 
        q_to_mat(qconj(m_rotation), Qinv);
        
        //D = trans(Q) * I * Q;
        m_local_inertia = diag(prod(trans(Q), mat3x3f(prod(I, Q))));

        m_vertices.resize(num_vertices);
        for(size_t i = 0; i < m_vertices.size(); ++i) {
            m_vertices[i] = prod(Qinv, vertices[i] - m_center);
        }

        set_shape(new btConvexHullShape((float const*)&(m_vertices[0]), num_vertices, sizeof(vec3f)));
    }

    void update()
    {
        btConvexHullShape *cu_shape = static_cast<btConvexHullShape*>(shape());

        //apply the scaling
        btVector3 const& scale = cu_shape->getLocalScaling();
        btPoint3 const* points = cu_shape->getPoints();
        for(int i = 0; i < cu_shape->getNumPoints(); ++i) {
            m_vertices[i] = vec3f(scale.x() * points[i].x(), scale.y() * points[i].y(), scale.z() * points[i].z()); 
        }
        m_volume = ::volume(&(m_vertices[0]), (int*)&(m_indices[0]), m_indices.size());
        mat3x3f I = inertia(&(m_vertices[0]), (int*)&(m_indices[0]), (int)m_indices.size(), vec3f(0, 0, 0));
        //std::cout << I << std::endl;
        //m_rotation = diagonalizer(I);
        //std::cout << rotation << std::endl;
        //the rotation shouldn't change from scaling

        mat3x3f Q, Qinv; 
        q_to_mat(m_rotation, Q); 
        q_to_mat(qconj(m_rotation), Qinv);

        //D = Q * I * trans(Q);
        m_local_inertia = diag(prod(trans(Q), mat3x3f(prod(I, Q))));
    }

private:
    std::vector<vec3f> m_vertices;
    std::vector<vec3f> m_normals;
    std::vector<unsigned int> m_indices; 

    float m_volume;
    vec3f m_center;
    quatf m_rotation;
    vec3f m_local_inertia;


};

#endif
