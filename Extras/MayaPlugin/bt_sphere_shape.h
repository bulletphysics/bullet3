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

//bt_sphere_shape.h

#ifndef DYN_BT_SPHERE_SHAPE_H
#define DYN_BT_SPHERE_SHAPE_H

#ifdef WIN32//for glut.h
#include <windows.h>
#endif

//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#endif


#include "sphere_shape_impl.h"
#include "bt_collision_shape.h"
#include "collision_shape.h"
#include "drawUtils.h"

class bt_sphere_shape_t: public bt_collision_shape_t, public sphere_shape_impl_t 
{
public:
    virtual void gl_draw(size_t draw_style) {
        btSphereShape *sphere_shape = static_cast<btSphereShape*>(shape());
        glPushMatrix();
        glScalef(sphere_shape->getRadius(), sphere_shape->getRadius(), sphere_shape->getRadius());
        if(draw_style & collision_shape_t::kDSSolid) {
            solid_sphere();
        } else {
            wire_sphere();
        }
        glPopMatrix();
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

    bt_sphere_shape_t(float radius): 
        bt_collision_shape_t()
    {
        set_shape(new btSphereShape(radius));
       // shape()->setMargin(0.1);
        update();
    }

    void update()
    {
        btSphereShape *sphere_shape = static_cast<btSphereShape*>(shape());
        float radius = sphere_shape->getRadius();
        m_volume = (4.0f * 3.1415926f * radius * radius * radius) / 3.0f;
        m_center = vec3f(0, 0, 0);
        m_rotation = qidentity<float>();
        m_local_inertia = vec3f(2.0f / 5.0f * radius * radius,
                                2.0f / 5.0f * radius * radius,
                                2.0f / 5.0f * radius * radius);
    }

private:
    float m_volume;
    vec3f m_center;
    quatf m_rotation;
    vec3f m_local_inertia;

};

#endif
