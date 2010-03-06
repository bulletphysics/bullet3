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

//vec.h

#ifndef MVL_VEC_H
#define MVL_VEC_H

#include <iostream>
#include <cmath>
#include "base.h"
#include "traits.h"

namespace mvl {

template<typename T, std::size_t Sz> 
class vec
{
public:
    typedef T                                       value_type;
    typedef T&                                      reference;
    typedef T const&                                const_reference;
    typedef T*                                      iterator;
    typedef T const*                                const_iterator;
    typedef std::reverse_iterator<iterator>         reverse_iterator;
    typedef std::reverse_iterator<const_iterator>   const_reverse_iterator;

public:
    enum {
        Size        = Sz,
    };
   
public:
    //constructors
    explicit vec() {}

    template<typename T2>
    vec(vec<T2, Size> const& v)
    {
        *this = v;
    }

    explicit vec(value_type val)
    {
        for(int i = 0; i < Size; ++i) {
            m_data[i] = val;
        }
    }  

    explicit vec(value_type x0, value_type x1)
    {   
        m_data[0] = x0; m_data[1] = x1;
    }

    explicit vec(value_type x0, value_type x1, value_type x2)
    {   
        m_data[0] = x0; m_data[1] = x1; m_data[2] = x2;   
    }

    explicit vec(value_type x0, value_type x1, value_type x2, value_type x3)
    {   
        m_data[0] = x0; m_data[1] = x1; m_data[2] = x2; m_data[3] = x3;  
    }

    explicit vec(value_type x0, value_type x1, value_type x2, value_type x3, value_type x4)
    {   
        m_data[0] = x0; m_data[1] = x1; m_data[2] = x2; m_data[3] = x3; m_data[4] = x4;
    }

    explicit vec(value_type x0, value_type x1, value_type x2, value_type x3, value_type x4, value_type x5)
    {   
        m_data[0] = x0; m_data[1] = x1; m_data[2] = x2; m_data[3] = x3; m_data[4] = x4; m_data[5] = x5; 
    }

public:
    //data access
    value_type operator[](std::size_t i) const  { return m_data[i]; }
    reference operator[](std::size_t i)         { return m_data[i]; }

    value_type operator()(std::size_t i) const  { return m_data[i]; }
    reference operator()(std::size_t i)         { return m_data[i]; }

public:
    //stl
    static std::size_t size()               { return Size; }
    static std::size_t max_size()           { return Size; }
    static bool empty()                     { return false; }

    iterator begin()                        { return m_data; }
    iterator end()                          { return m_data + Size; }
    const_iterator begin() const            { return m_data; }
    const_iterator end() const              { return m_data + Size; }
    reverse_iterator rbegin()               { return reverse_iterator(end()); }
    reverse_iterator rend()                 { return reverse_iterator(begin()); }
    const_reverse_iterator rbegin() const   { return const_reverse_iterator(end()); }
    const_reverse_iterator rend() const     { return const_reverse_iterator(begin()); }

    value_type front()                      { return m_data[0]; }
    value_type back()                       { return m_data[Size - 1]; }
    const_reference front() const           { return m_data[0]; }
    const_reference back() const            { return m_data[Size - 1]; }

    friend std::ostream& operator << (std::ostream& out, vec const& v) {
        out << "("; 
        for(size_t i = 0; i < Size - 1; i++) {
                out << v(i) << ", ";
        }
        out << v(Size - 1) << ")"; 
        return out;
    }

public:
    //assignment 
    vec& operator=(vec const& rhs) {
        for(int i = 0; i < Size; ++i) {
            m_data[i] = rhs[i];
        }
        return *this;
    } 

    template<typename T2>
    vec& operator=(vec<T2, Size> const& rhs) {
        for(int i = 0; i < Size; ++i) {
            m_data[i] = rhs[i];
        }
        return *this;
    } 

private:
    T   m_data[Size];
};


//assignment operators 
// OP(vec<T1>, vec<T2>)
// OP(vec<T>, T)

#define VEC_IMPLEMENT_MACRO(OP)                                                         \
template<typename T1, typename T2, std::size_t Sz>                                      \
inline                                                                                  \
vec<T1, Sz>&                                                                            \
operator OP(vec<T1, Sz>& lhs, vec<T2, Sz> const& rhs) {                                 \
    for(int i = 0; i < Sz; ++i) {                                                       \
        lhs[i] OP rhs[i];                                                               \
    }                                                                                   \
    return lhs;                                                                         \
}                                                                                       \
                                                                                        \
template<typename T, std::size_t Sz>                                                    \
inline                                                                                  \
vec<T, Sz>&                                                                             \
operator OP(vec<T, Sz>& lhs, T const& rhs) {                                            \
    for(int i = 0; i < Sz; ++i) {                                                       \
        lhs[i] OP rhs;                                                                  \
    }                                                                                   \
    return lhs;                                                                         \
}                                                                                       \
  
VEC_IMPLEMENT_MACRO(+=)
VEC_IMPLEMENT_MACRO(-=)
VEC_IMPLEMENT_MACRO(*=)
VEC_IMPLEMENT_MACRO(/=)

#undef VEC_IMPLEMENT_MACRO

//operator + (vec, vec)
template<typename T1, typename T2, std::size_t Sz>                                                  
inline 
vec<typename promote_traits<T1, T2>::value_type, Sz>                                                                                             
operator + (vec<T1, Sz> const& lhs, vec<T2, Sz> const& rhs)
{        
    vec<typename promote_traits<T1, T2>::value_type, Sz> res;
    for(int i = 0; i < Sz; ++i) {                                                       
        res[i] = lhs[i] + rhs[i];
    }
    return res;
}                                                                                           

//operator - (vec, vec)
template<typename T1, typename T2, std::size_t Sz>                                                  
inline 
vec<typename promote_traits<T1, T2>::value_type, Sz>                                                                                             
operator - (vec<T1, Sz> const& lhs, vec<T2, Sz> const& rhs)
{        
    vec<typename promote_traits<T1, T2>::value_type, Sz> res;
    for(int i = 0; i < Sz; ++i) {                                                       
        res[i] = lhs[i] - rhs[i];
    }
    return res;
}                                                                                           

//operator * (vec, POD)
template<typename T1, typename T2, std::size_t Sz>                                                    
inline    
vec<typename promote_traits<T1, T2>::value_type, Sz>                                                                                             
operator * (vec<T1, Sz> const& lhs, T2 const& rhs) {                                            
    vec<typename promote_traits<T1, T2>::value_type, Sz> res;
    for(int i = 0; i < Sz; ++i) {                                                       
        res[i] = lhs[i] * rhs;
    }
    return res;
}                                                                                                   

//operator * (POD, vec)
template<typename T1, typename T2, std::size_t Sz>                                                 
inline
vec<typename promote_traits<T1, T2>::value_type, Sz>                                                                                             
operator * (T1 const& lhs, vec<T2, Sz> const& rhs) {                                          
    vec<typename promote_traits<T1, T2>::value_type, Sz> res;
    for(int i = 0; i < Sz; ++i) {                                                       
        res[i] = lhs * rhs[i];
    }
    return res;
}                                                                                                

//operator / (vec, POD)
template<typename T1, typename T2, std::size_t Sz>                                                    
inline                                                                                              
vec<typename promote_traits<T1, T2>::value_type, Sz>                                                                                             
operator / (vec<T1, Sz> const& lhs, T2 const& rhs) {                                            
    vec<typename promote_traits<T1, T2>::value_type, Sz> res;
    for(int i = 0; i < Sz; ++i) {                                                       
        res[i] = lhs[i] / rhs;
    }
    return res;
}                                                                                                   

//element_prod(vec, vec)
template<typename T1, typename T2, std::size_t Sz>                                                  
inline                                                                                              
vec<typename promote_traits<T1, T2>::value_type, Sz>                                                                                             
element_prod(vec<T1, Sz> const& lhs, vec<T2, Sz> const& rhs) {                            
    vec<typename promote_traits<T1, T2>::value_type, Sz> res;
    for(int i = 0; i < Sz; ++i) {                                                       
        res[i] = lhs[i] * rhs[i];
    }
    return res;
}

//element_div(vec, vec)
template<typename T1, typename T2, std::size_t Sz>                                                  
inline                                                                                              
vec<typename promote_traits<T1, T2>::value_type, Sz>                                                                                             
element_div(vec<T1, Sz> const& lhs, vec<T2, Sz> const& rhs) {                            
    vec<typename promote_traits<T1, T2>::value_type, Sz> res;
    for(int i = 0; i < Sz; ++i) {                                                       
        res[i] = lhs[i] / rhs[i];
    }
    return res;
}

//unary operator -(expr_vec)
template<typename T, std::size_t Sz>                                                  
inline                                                                                              
vec<T, Sz>
operator -(vec<T, Sz> const& rhs) {                            
    vec<T, Sz> res;
    for(int i = 0; i < Sz; ++i) {                                                       
        res[i] = -rhs[i];
    }
    return res;
}                                                                                                   

//dot product
template<typename T1, typename T2, std::size_t Sz>
inline 
typename promote_traits<T1, T2>::value_type
dot(vec<T1, Sz> const& lhs, vec<T2, Sz> const& rhs)
{
    typename promote_traits<T1, T2>::value_type res(0);
    for(int i = 0; i < Sz; ++i) {                                                       
        res += rhs[i] * lhs[i];
    }
    return res;
}


//cross product
template<typename T1, typename T2>
inline 
vec<typename promote_traits<T1, T2>::value_type, 3>
cross(vec<T1, 3> const& lhs, vec<T2, 3> const& rhs) {
   typedef typename promote_traits<T1, T2>::value_type value_type;
   return vec<value_type, 3>(lhs(1)*rhs(2) - rhs(1)*lhs(2),
                             rhs(0)*lhs(2) - lhs(0)*rhs(2),
                             lhs(0)*rhs(1) - rhs(0)*lhs(1));
} 

//length of the vector
template<typename T, std::size_t Sz>
inline T
norm2(vec<T, Sz> const& rhs)
{
    return static_cast<T>(sqrt(dot(rhs, rhs)));
}

//length of the vector squared
template<typename T, std::size_t Sz>
inline T
norm_squared(vec<T, Sz> const& rhs)
{
    return dot(rhs, rhs);
}

//normalize the vector
template<typename T, std::size_t Sz>
inline 
vec<T, Sz>
normalize(vec<T, Sz> const& v) {
    typedef T value_type;
    T tmp = norm2(v);
    if(tmp == value_type(0)) {
        tmp = value_type(0); 
    } else {
        tmp = value_type(1) / tmp; 
    }
    vec<T, Sz> res;
    for(int i = 0; i < Sz; ++i) {                                                       
        res[i] = v[i] * tmp;
    }
    return res;
}    


} //namespace mvl 

#endif

