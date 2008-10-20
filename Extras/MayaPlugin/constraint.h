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

//constraint.h

#ifndef DYN_CONSTRAINT_H
#define DYN_CONSTRAINT_H

#include "shared_ptr.h"
#include "constraint_impl.h"

class constraint_t
{
public:
    //typedefs
    typedef shared_ptr<constraint_t> pointer;

public:
    virtual ~constraint_t() {}

protected:
    friend class solver_t;

    constraint_t(constraint_impl_t* impl): m_impl(impl) { }

    constraint_impl_t* impl()              { return m_impl.get(); }
    constraint_impl_t const* impl() const  { return m_impl.get(); }
    
protected:
    shared_ptr<constraint_impl_t> m_impl;
};

#endif
