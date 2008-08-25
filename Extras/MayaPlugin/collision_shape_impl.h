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

//collision_shape_impl.h

#ifndef DYN_COLLISION_SHAPE_IMPL_H
#define DYN_COLLISION_SHAPE_IMPL_H

#include "shared_ptr.h"
#include "mathUtils.h"

class collision_shape_impl_t
{
public:
    //
    virtual void gl_draw(size_t draw_style) = 0;

    virtual void set_scale(vec3f const& s) = 0;
    virtual void get_scale(vec3f& s) = 0;

    virtual float volume() = 0;
    virtual vec3f const& local_inertia() = 0;
    virtual vec3f const& center() = 0;
    virtual quatf const& rotation() = 0;

public:
    virtual ~collision_shape_impl_t() {};
};

#endif
