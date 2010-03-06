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

//mat.h

#ifndef MVL_MAT_H
#define MVL_MAT_H

#include <cmath>
#include "base.h"
#include "traits.h"

namespace mvl {

template<typename T, std::size_t R, std::size_t C>
class mat
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
        Rows        = R,
        Cols        = C,
        Size        = Rows * Cols,
    };
   
public:
    //constructors
    explicit mat() {}

    template<typename T2>
    mat(mat<T2, Rows, Cols> const& m)
    {
        *this = m;
    }  

    explicit mat(value_type val)
    {
        for(int i = 0; i < Size; ++i){
            m_data[i] = val;
        }
    } 

    explicit mat(value_type m00, value_type m01,
                 value_type m10, value_type m11)
    {
        operator()(0, 0) = m00; operator()(0, 1) = m01;
        operator()(1, 0) = m10; operator()(1, 1) = m11;
    }

    explicit mat(value_type m00, value_type m01, value_type m02,
                 value_type m10, value_type m11, value_type m12,
                 value_type m20, value_type m21, value_type m22)
    {
        operator()(0, 0) = m00; operator()(0, 1) = m01; operator()(0, 2) = m02;
        operator()(1, 0) = m10; operator()(1, 1) = m11; operator()(1, 2) = m12;
        operator()(2, 0) = m20; operator()(2, 1) = m21; operator()(2, 2) = m22;
    }

    explicit mat(value_type m00, value_type m01, value_type m02, value_type m03,
                 value_type m10, value_type m11, value_type m12, value_type m13,
                 value_type m20, value_type m21, value_type m22, value_type m23,
                 value_type m30, value_type m31, value_type m32, value_type m33)
    {
        operator()(0, 0) = m00; operator()(0, 1) = m01; operator()(0, 2) = m02; operator()(0, 3) = m03;
        operator()(1, 0) = m10; operator()(1, 1) = m11; operator()(1, 2) = m12; operator()(1, 3) = m13;
        operator()(2, 0) = m20; operator()(2, 1) = m21; operator()(2, 2) = m22; operator()(2, 3) = m23;
        operator()(3, 0) = m30; operator()(3, 1) = m31; operator()(3, 2) = m32; operator()(3, 3) = m33;
    }

public:
    //data access
    value_type operator()(std::size_t i, std::size_t j) const  { return m_data[j * Rows + i]; }
    reference operator()(std::size_t i, std::size_t j)         { return m_data[j * Rows + i]; }

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

    friend std::ostream& operator << (std::ostream& out, mat const& m) {
        out << "("; 
        for(size_t i = 0; i < Rows - 1; i++) {
            for(size_t j = 0; j < Cols; j++) {
                out << m(i, j) << ", ";
            }
            out << std::endl;
        }
        for(size_t j = 0; j < Cols - 1; j++) {
            out << m(Rows - 1, j) << ", ";
        }
        out << m(Rows - 1, Cols - 1) << ")"; 
        return out;
    }

public:
    //
    mat& operator=(mat const& rhs) {
        std::copy(rhs.begin(),rhs.end(), begin());
        return *this;
    }

    template<typename T2> 
    mat& operator=(mat<T2, Rows, Cols> const& rhs) {
        std::copy(rhs.begin(),rhs.end(), begin()); 
        return *this;
    }

private:
    //data is stored in column major order, so the matrix can passed directly to the graphics APIs
    T   m_data[Size];
};

//assignment operators 
// OP(mat<T1>, mat<T2>)
// OP(mat<T>, T)

#define MAT_IMPLEMENT_MACRO(OP)                                                         \
template<typename T1, typename T2, std::size_t R, std::size_t C>                        \
inline                                                                                  \
mat<T1, R, C>&                                                                          \
operator OP(mat<T1, R, C>& lhs, mat<T2, R, C> const& rhs) {                             \
    for(int i = 0; i < C * R; ++i) {                                                    \
        lhs[i] OP rhs[i];                                                               \
    }                                                                                   \
    return lhs;                                                                         \
}                                                                                       \
                                                                                        \
template<typename T, std::size_t R, std::size_t C>                                      \
inline                                                                                  \
mat<T, R, C>&                                                                           \
operator OP(mat<T, R, C>& lhs, typename mat<T, R, C>::value_type const& rhs) {          \
    for(int i = 0; i < C * R; ++i) {                                                    \
        lhs[i] OP rhs[i];                                                               \
    }                                                                                   \
    return lhs;                                                                         \
}                                                                                       \
  
MAT_IMPLEMENT_MACRO(+=)
MAT_IMPLEMENT_MACRO(-=)
MAT_IMPLEMENT_MACRO(*=)
MAT_IMPLEMENT_MACRO(/=)

#undef MAT_IMPLEMENT_MACRO

//operator + (mat, mat)
template<typename T1, typename T2, std::size_t R, std::size_t C>                                                  
inline                                                                                              
mat<typename promote_traits<T1, T2>::value_type, R, C>                                                                              
operator + (mat<T1, R, C> const& lhs, mat<T2, R, C> const& rhs) {                            
    mat<typename promote_traits<T1, T2>::value_type, R, C> res;
    for(int i = 0; i < R ; ++i) {
        for(int j = 0; j < C; ++j) {
            res(i, j) = lhs(i, j) + rhs(i, j);
        }
    }
    return res;         
}                                                                                                   

//operator - (mat, mat)
template<typename T1, typename T2, std::size_t R, std::size_t C>                                                  
inline                                                                                              
mat<typename promote_traits<T1, T2>::value_type, R, C>                                                                              
operator - (mat<T1, R, C> const& lhs, mat<T2, R, C> const& rhs) {                            
    mat<typename promote_traits<T1, T2>::value_type, R, C> res;
    for(int i = 0; i < R ; ++i) {
        for(int j = 0; j < C; ++j) {
            res(i, j) = lhs(i, j) - rhs(i, j);
        }
    }
    return res;         
}                                                                                                   

//operator * (mat, POD)
template<typename T1, typename T2, std::size_t R, std::size_t C>                                                    
inline
mat<typename promote_traits<T1, T2>::value_type, R, C>                                                                              
operator * (mat<T1, R, C> const& lhs, T2 const& rhs) {                                            
    mat<typename promote_traits<T1, T2>::value_type, R, C> res;
    for(int i = 0; i < R ; ++i) {
        for(int j = 0; j < C; ++j) {
            res(i, j) = lhs(i, j) * rhs;
        }
    }
    return res;         
}                                                                                                   

//operator * (POD, mat)
template<typename T1, typename T2, std::size_t R, std::size_t C>                                                 
inline                                                                                           
mat<typename promote_traits<T1, T2>::value_type, R, C>                                                                              
operator * (T1 const& lhs, mat<T2, R, C> const& rhs) {                                          
    mat<typename promote_traits<T1, T2>::value_type, R, C> res;
    for(int i = 0; i < R ; ++i) {
        for(int j = 0; j < C; ++j) {
            res(i, j) = lhs * rhs(i, j);
        }
    }
    return res;         
}                                                                                                

//operator / (mat, POD)
template<typename T1, typename T2, std::size_t R, std::size_t C>                                                    
inline                                                                                              
mat<typename promote_traits<T1, T2>::value_type, R, C>                                                                              
operator / (mat<T1, R, C> const& lhs, T2 const& rhs) {                                            
    mat<typename promote_traits<T1, T2>::value_type, R, C> res;
    for(int i = 0; i < R ; ++i) {
        for(int j = 0; j < C; ++j) {
            res(i, j) = lhs(i, j) / rhs;
        }
    }
    return res;         
}                                                                                                   

//element_prod(mat, mat)
template<typename T1, typename T2, std::size_t R, std::size_t C>                                                  
inline                                                                                              
mat<typename promote_traits<T1, T2>::value_type, R, C>                                                                              
element_prod(mat<T1, R, C> const& lhs, mat<T2, R, C> const& rhs) {                            
    mat<typename promote_traits<T1, T2>::value_type, R, C> res;
    for(int i = 0; i < R ; ++i) {
        for(int j = 0; j < C; ++j) {
            res(i, j) = lhs(i, j) * rhs(i, j);
        }
    }
    return res;         
}

//element_div(mat, mat)
template<typename T1, typename T2, std::size_t R, std::size_t C>                                                  
inline                                                                                              
mat<typename promote_traits<T1, T2>::value_type, R, C>                                                                              
element_div(mat<T1, R, C> const& lhs, mat<T2, R, C> const& rhs) {                            
    mat<typename promote_traits<T1, T2>::value_type, R, C> res;
    for(int i = 0; i < R ; ++i) {
        for(int j = 0; j < C; ++j) {
            res(i, j) = lhs(i, j) / rhs(i, j);
        }
    }
    return res;         
}                                                                                                   

//unary operator -(mat)
template<typename T, std::size_t R, std::size_t C>                                                  
inline                                                                                              
mat<T, R, C>
operator -(mat<T, R, C> const& rhs) {                            
    mat<T, R, C> res;
    for(int i = 0; i < R ; ++i) {
        for(int j = 0; j < C; ++j) {
            res(i, j) = -rhs(i, j);
        }
    }
    return res;         
}

//matrix transpose
template<typename T, std::size_t R, std::size_t C>
inline 
mat<T, R, C>
trans(mat<T, R, C> const& rhs) {
    mat<T, R, C> res;
    for(int i = 0; i < R ; ++i) {
        for(int j = 0; j < C; ++j) {
            res(i, j) = rhs(j, i);
        }
    }
    return res;
}  

//identity matrix
template<typename T, std::size_t Sz>
inline
mat<T, Sz, Sz>
identity() {
    mat<T, Sz, Sz> res;
    for(int i = 0; i < Sz ; ++i) {
        for(int j = 0; j < Sz; ++j) {
            res(i, j) = i == j ? 1 : 0;
        }
    }
    return res;
}

//matrix diagonal as vector (for square matrices)
template<typename T, std::size_t N>
inline 
vec<T, N>
diag(mat<T, N, N> const& rhs) {
    vec<T, N> res;
    for(int i = 0; i < N; ++i) {
        res[i] = rhs(i, i);
    }
    return res;
}

//matrix row as vector
template<typename T, std::size_t R, std::size_t C>
inline 
vec<T, C>
row(mat<T, R, C> const& rhs, std::size_t r) {
    vec<T, C> res;
    for(int i = 0; i < C; ++i) {
        res[i] = rhs(r, i);
    }
    return res;
}

//matrix column as vector 
template<typename T, std::size_t R, std::size_t C>
inline 
vec<T, R>
col(mat<T, R, C> const& rhs, std::size_t c) {
    vec<T, R> res;
    for(int i = 0; i < R; ++i) {
        res[i] = rhs(i, c);
    }
    return res;
}


//matrix-matrix product
template<typename T1, typename T2, std::size_t R1, std::size_t C1, std::size_t C2>
inline 
mat<typename promote_traits<T1, T2>::value_type, R1, C2>
prod(mat<T1, R1, C1> const& lhs, mat<T2, C1, C2> const& rhs) {
    mat<typename promote_traits<T1, T2>::value_type, R1, C2> res;
    for(int i = 0; i < R1; ++i) {
        for(int j = 0; j < C2; ++j) {
            res(i, j) = 0;
            for(int k = 0; k < C1; ++k) {
                res(i, j) += lhs(i, k) * rhs(k, j);
            }
        }
    }
    return res;
}

//matrix - column vector product
template<typename T1, typename T2, std::size_t R, std::size_t C>
inline 
vec<typename promote_traits<T1, T2>::value_type, R>
prod(mat<T1, R, C> const& lhs, vec<T2, C> const& rhs) {
    vec<typename promote_traits<T1, T2>::value_type, R> res;
    for(int i = 0; i < R; ++i) {
        res(i) = 0;
        for(int j = 0; j < C; ++j) {
            res(i) += lhs(i, j) * rhs(j);  
        }
    }
    return res;
}


} // namespace mvl 

#endif
