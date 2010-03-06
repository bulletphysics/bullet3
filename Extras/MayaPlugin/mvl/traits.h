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

//traits.h

#ifndef MVL_TRAITS_H
#define MVL_TRAITS_H

namespace mvl {

//simple promotion for now

//check if a type is a POD
template<typename T> 
struct isPOD { enum { value = false }; };
template<> struct isPOD<char> { enum { value = true }; };
template<> struct isPOD<short> { enum { value = true }; };
template<> struct isPOD<int> { enum { value = true }; };
template<> struct isPOD<float> { enum { value = true }; };
template<> struct isPOD<double> { enum { value = true }; };
template<> struct isPOD<long double> { enum { value = true }; };

//
template<bool Condition, typename T1, typename T2> struct ifThenElse { typedef T2 value_type; };
template<typename T1, typename T2> struct ifThenElse<true, T1, T2> { typedef T1 value_type; };

template<typename T1, typename T2> 
struct promote_traits 
{
	typedef typename ifThenElse<isPOD<T1>::value, T2, T1>::value_type value_type;                  
};

template<typename T>                                  
struct promote_traits<T, T>               
{                                           
    typedef T value_type;                  
};                                          

#define TRAITS_DEFINE_MACRO(T1, T2, TP)     \
template<>                                  \
struct promote_traits<T1, T2>               \
{                                           \
    typedef TP value_type;                  \
};                                          \
template<>                                  \
struct promote_traits<T2, T1>               \
{                                           \
    typedef TP value_type;                  \
};                                      
    
TRAITS_DEFINE_MACRO(int, float, float)
TRAITS_DEFINE_MACRO(int, double, double)
TRAITS_DEFINE_MACRO(int, long double, long double)

TRAITS_DEFINE_MACRO(float, double, double)
TRAITS_DEFINE_MACRO(float, long double, long double)

TRAITS_DEFINE_MACRO(double, long double, long double)

#undef TRAITS_DEFINE_MACRO


} // namespace mvl 

#endif

