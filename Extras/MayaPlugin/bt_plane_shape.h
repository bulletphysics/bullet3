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

//bt_plane_shape.h

#ifndef DYN_BT_PLANE_SHAPE_H
#define DYN_BT_PLANE_SHAPE_H

#include "plane_shape_impl.h"
#include "drawUtils.h"

class bt_plane_shape_t: public bt_collision_shape_t, public plane_shape_impl_t 
{
public:
    virtual void gl_draw(size_t draw_style) {
      //  btStaticPlaneShape *plane_shape = static_cast<btStaticPlaneShape*>(shape());
        glPushMatrix();
        glScalef(100.0, 0.001, 100.0); 
        if(draw_style & collision_shape_t::kDSSolid) {
            solid_cube();
        } else {
            wire_cube();
        }
        glPopMatrix();
    }

    virtual void set_scale(vec3f const& s) {
        shape()->setLocalScaling(btVector3(s[0], s[1], s[2]));
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

    bt_plane_shape_t(vec3f const& normal, float d): 
        bt_collision_shape_t(),
        m_volume(0),
        m_local_inertia(1, 1, 1),
        m_center(0, 0, 0),
        m_rotation(qidentity<float>())
    { 
        set_shape(new btStaticPlaneShape(btVector3(normal[0], normal[1], normal[2]), d));
      //  shape()->setMargin(0.1);
    }

private:
    float m_volume;
    vec3f m_center;
    quatf m_rotation;
    vec3f m_local_inertia;

};

#endif
