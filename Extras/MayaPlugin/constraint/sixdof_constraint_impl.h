/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Herbert Law
 
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
 
Written by: Herbert Law <Herbert.Law@gmail.com>

Modified by Roman Ponomarev <rponom@gmail.com>
01/22/2010 : Constraints reworked
*/

//sixdof_constraint_impl.h

#ifndef DYN_SIXDOF_CONSTRAINT_IMPL_H
#define DYN_SIXDOF_CONSTRAINT_IMPL_H

#include "constraint_impl.h"

class sixdof_constraint_impl_t: public constraint_impl_t
{
public:
    virtual void set_world(vec3f const& p, quatf const& r) = 0; 
    virtual void get_world(vec3f& p, quatf& r) const = 0; 
	virtual void get_frameA(vec3f& p, quatf& r) const = 0; 
	virtual void get_frameB(vec3f& p, quatf& r) const = 0; 
	virtual void get_invFrameA(vec3f& p, quatf& r) const = 0; 
	virtual void get_invFrameB(vec3f& p, quatf& r) const = 0; 
	virtual void worldToA(vec3f& w, vec3f& p) const = 0; 
	virtual void worldFromB(vec3f& p, vec3f& w) const = 0; 

    //
    virtual void set_damping(float d) = 0;
    virtual float damping() const = 0;
    virtual void set_LinLimit(vec3f& lower, vec3f& upper) = 0;
    virtual void set_AngLimit(vec3f& lower, vec3f& upper) = 0;

public:
    virtual ~sixdof_constraint_impl_t() {};
};

#endif //DYN_SIXDOF_CONSTRAINT_IMPL_H
