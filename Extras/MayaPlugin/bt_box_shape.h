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
 
//bt_box_shape.h

#ifndef DYN_BT_BOX_SHAPE_H
#define DYN_BT_BOX_SHAPE_H

#include "box_shape_impl.h"
#include "drawUtils.h"

class bt_box_shape_t: public bt_collision_shape_t, public box_shape_impl_t 
{
public:
    virtual void gl_draw(size_t draw_style) {
        btBoxShape *box_shape = static_cast<btBoxShape*>(shape());
        btVector3 const& e = box_shape->getHalfExtentsWithoutMargin();
        glPushMatrix();

        glScalef(2 * e.x(), 2 * e.y(), 2 * e.z()); 
        if(draw_style & collision_shape_t::kDSSolid) {
            solid_cube();
        } else {
            wire_cube();
        }
        glPopMatrix();
    }

    virtual void set_scale(vec3f const& s) {
        const btVector3& scale = shape()->getLocalScaling();
        if(scale.x() != s[0] || scale.y() != s[1] || scale.z() != s[2]) {
            shape()->setLocalScaling(btVector3(s[0], s[1], s[2]));
            update();
        }
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

    bt_box_shape_t(vec3f const& halfExtents): 
        bt_collision_shape_t()
    { 
        set_shape(new btBoxShape(btVector3(halfExtents[0], halfExtents[0], halfExtents[0])));
        update();
    }

    void update()
    {
        btBoxShape *box_shape = static_cast<btBoxShape*>(shape());
        btVector3 e = 2 * box_shape->getHalfExtentsWithoutMargin();
        m_volume = e.x() * e.y() * e.z();
        m_center = vec3f(0, 0, 0);
        m_rotation = qidentity<float>();
        m_local_inertia = vec3f((e.y() * e.y() + e.z() * e.z()) / 12.0f,
                                (e.x() * e.x() + e.z() * e.z()) / 12.0f,
                                (e.x() * e.x() + e.y() * e.y()) / 12.0f);
    }

private:
    float m_volume;
    vec3f m_center;
    quatf m_rotation;
    vec3f m_local_inertia;
};

#endif
