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

//util.h

#ifndef MVL_UTIL_H
#define MVL_UTIL_H

#include <cmath>
#include "base.h"
#include "traits.h"
#include "vec.h"

namespace mvl {

//translation
template<typename T>
inline
mat<T, 4, 4> translation(vec<T, 3> const& v)
{
    return mat<T, 4, 4>(1, 0, 0, v(0),
                        0, 1, 0, v(1),
                        0, 0, 1, v(2),
                        0, 0, 0, 1);  
}

//scale
template<typename T>
inline
mat<T, 4, 4> scale(vec<T, 3> const& v)
{
   return mat<T, 4, 4> (v(0), 0, 0, 0,
                        0, v(1), 0, 0,
                        0, 0, v(2), 0,
                        0, 0, 0, 1);  
}

} // namespace mvl



#endif
