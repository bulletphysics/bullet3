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

//bt_collision_shape.h

#ifndef DYN_BT_COLLISION_SHAPE_H
#define DYN_BT_COLLISION_SHAPE_H

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"

class bt_collision_shape_t 
{
public:

protected:
    friend class bt_solver_t;

    bt_collision_shape_t() { }

    btCollisionShape* shape()               { return m_shape.get(); }
    void set_shape(btCollisionShape *shape) { return m_shape.reset(shape); }

public:
    friend class bt_rigid_body_t;
    virtual ~bt_collision_shape_t() { }

private:
    shared_ptr<btCollisionShape> m_shape; 
};

#endif
