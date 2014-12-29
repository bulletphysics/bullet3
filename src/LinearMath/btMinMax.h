/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_GEN_MINMAX_H
#define BT_GEN_MINMAX_H

#include "btScalar.h"


// Only in C++, prefer use above template functions.
// Otherwise, use the common unsafe C macro version here.
// The reason using this macro is unsafe, is because the parameters could be executed more than once.
// However, if all parameters aren't active expression, using this macros is safe.
//
// In conclusion, if target isn't only C++, use this macros with non-active expression parameters;
//  otherwise, if only targeting C++, use the templates version (below).

BT_COMMON_BEGIN

#define btMin_unsafe(a, b) ((a) < (b) ? (a) : (b))

#define btMax_unsafe(a, b) ((a) > (b) ? (a) : (b))

#define btClamped_unsafe(a, lb, ub) ((a) < (lb) ? (lb) : ((ub) < (a) ? (ub) : (a)))

#define btSetMin_unsafe(a, b) \
	if ((b) < (a)) \
	(a) = (b)

#define btSetMax_unsafe(a, b) \
	if ((b) > (a)) \
	(a) = (b)

#define btClamp_unsafe(a, lb, ub) do { \
	if (a < lb) \
	{ \
		a = lb; \
	} \
	else if (ub < a) \
	{ \
		a = ub; \
	} \
} while(0)

BT_COMMON_END


// C++ safe templates version
#ifdef __cplusplus
template <class T>
SIMD_FORCE_INLINE const T& btMin(const T& a, const T& b) 
{
  return btMin_unsafe(a, b);
}

template <class T>
SIMD_FORCE_INLINE const T& btMax(const T& a, const T& b) 
{
  return btMax_unsafe(a, b);
}

template <class T>
SIMD_FORCE_INLINE const T& btClamped(const T& a, const T& lb, const T& ub) 
{
	return btClamped_unsafe(a, lb, ub); 
}

template <class T>
SIMD_FORCE_INLINE void btSetMin(T& a, const T& b) 
{
	btSetMin_unsafe(a, b);
}

template <class T>
SIMD_FORCE_INLINE void btSetMax(T& a, const T& b) 
{
	btSetMax_unsafe(a, b);
}

template <class T>
SIMD_FORCE_INLINE void btClamp(T& a, const T& lb, const T& ub) 
{
	btClamp_unsafe(a, lb, ub);
}
#endif//__cplusplus

#endif //BT_GEN_MINMAX_H
