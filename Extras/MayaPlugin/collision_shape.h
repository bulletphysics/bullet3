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

//collision_shape.h

#ifndef DYN_COLLISION_SHAPE_H
#define DYN_COLLISION_SHAPE_H

#include "shared_ptr.h"
#include "collision_shape_impl.h"

class collision_shape_t
{
public:
    //typedefs
    typedef shared_ptr<collision_shape_t> pointer;

    //enums
    enum DrawStyle {
        kDSWireframe = 0x0001,
        kDSSolid = 0x0002
    };

    virtual void gl_draw(size_t draw_style = kDSSolid) { m_impl->gl_draw(draw_style); }  

    virtual void set_scale(vec3f const& s)  { m_impl->set_scale(s);  }
    virtual void get_scale(vec3f& s)        { m_impl->get_scale(s);  }

    virtual float volume()               { return m_impl->volume();  }
    //for the inertia is assumed that the mass is 1.0. just multiply by the mass
    virtual vec3f const& local_inertia() { return m_impl->local_inertia();  }
    virtual vec3f const& center()        { return m_impl->center();   }
    virtual quatf const& rotation()      { return m_impl->rotation();   }

public:
    virtual ~collision_shape_t() {}

protected:
    friend class solver_t;

    collision_shape_t(collision_shape_impl_t* impl): m_impl(impl) { }

    collision_shape_impl_t* impl()              { return m_impl.get(); }
    
private:
    shared_ptr<collision_shape_impl_t> m_impl;
};

#endif
