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
01/22/2010 : Constraints reworked
*/

//bt_constraint.h

#ifndef DYN_BT_CONSTRAINT_H
#define DYN_BT_CONSTRAINT_H

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
#include "shared_ptr.h"

class rigid_body_impl_t;

class bt_constraint_t 
{
public:

protected:
    friend class bt_solver_t;

    bt_constraint_t() { }

    btTypedConstraint* constraint()                     { return m_constraint.get(); }
    void set_constraint(btTypedConstraint *constraint)  { return m_constraint.reset(constraint); }
	virtual void update_constraint(rigid_body_impl_t* rb) = 0;

public:
    friend class bt_rigid_body_t;

    virtual ~bt_constraint_t() { }

protected:
    shared_ptr<btTypedConstraint> m_constraint; 
};

#endif
