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

//mathUtils.h

#ifndef DYN_MATH_UTILS_H
#define DYN_MATH_UTILS_H

#include "mvl/vec.h"
#include "mvl/mat.h"
#include "mvl/quat.h"
using namespace mvl;

typedef vec<float, 3> vec3f;
typedef mat<float, 4, 4> mat4x4f;
typedef mat<float, 3, 3> mat3x3f;
typedef vec<float, 4> quatf;

template<typename T>
inline T deg2rad(T x) { return x * T(3.141592654 / 180.0); }

template<typename T>
inline T rad2deg(T x) { return x * T(180.0 / 3.141592654); }


template<typename T> 
inline T clamp(T x, T min, T max)
{
    if(x < min) return min;
    if(x > max) return max;
    return x;
}

template<typename T>
inline T sqr(T x) {
    return x * x;
}


// A must be a symmetric matrix.
// returns quaternion q such that its corresponding matrix Q 
// can be used to Diagonalize A
//Diagonal matrix D = Q^T * A * Q;  and  A = Q*D*Q^T
// The colums of q are the eigenvectors D's diagonal is the eigenvalues
// As per 'column' convention if float3x3 Q = q.getmatrix(); then Q*v = q*v*conj(q)
template<typename T>
vec<T, 4> diagonalizer(const mat<T, 3, 3> &A)
{
    const int maxsteps = 24;  // certainly wont need that many.

    typedef vec<T, 3> vec3_t;
    typedef vec<T, 4> quat_t;
    typedef mat<T, 3, 3> mat3x3_t;
    quat_t q(qidentity<T>());
    for(int i = 0; i < maxsteps; ++i)
    {
        mat3x3_t Q;
        q_to_mat(q, Q); // Q*v == q*v*conj(q)
        mat3x3_t D = prod(trans(Q), mat3x3_t(prod(A, Q)));     // A = Q^T*D*Q
        vec3_t offdiag(D(2, 1), D(2, 0), D(1, 0));  // elements not on the diagonal
        vec3_t om(fabs(offdiag[0]), fabs(offdiag[1]), fabs(offdiag[2])); // mag of each offdiag elem
        int k = (om[0] > om[1] && om[0] > om[2]) ? 0 : (om[1] > om[2]) ? 1 : 2; // index of largest element of offdiag
        int k1 = (k + 1) % 3;
        int k2 = (k + 2) % 3;
        if(offdiag[k] == T()) break;  // diagonal already
        T thet = (D(k2, k2) - D(k1, k1)) / (T(2.0) * offdiag[k]);
        T sgn = (thet > 0.0f) ? T(1.0) : T(-1.0);
        thet *= sgn; // make it positive
        T t = sgn / (thet + ((thet < T(1.E6)) ? sqrtf(sqr(thet) + T(1.0)) : thet)) ; // sign(T)/(|T|+sqrt(T^2+1))
        T c = T(1.0) / sqrtf(sqr(t) + T(1.0)); //  c= 1/(t^2+1) , t=s/c 
        if(c == T(1.0)) break;  // no room for improvement - reached machine precision.
        quat_t jr(0,0,0,0); // jacobi rotation for this iteration.
        jr[1 + k] = sgn * sqrtf((T(1.0) - c) / T(2.0));  // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)  
        jr[1 + k] *= -T(1.0); // since our quat-to-matrix convention was for v*M instead of M*v
        jr[0]  = sqrtf(T(1.0) - sqr(jr[1 + k]));
        if(jr[0] == T(1.0)) break; // reached limits of floating point precision
        q = qprod(q, jr);  
        q = normalize(q);
    } 
    return q;
}    

template<typename T> 
inline T determinant(vec<T, 3> const& v0, vec<T, 3> const& v1, vec<T, 3> const& v2)
{
    return dot(v0, cross(v1, v2));
}

template<typename T>
T volume(vec<T, 3> const* vertices, int const* indices, const int num_indices) 
{
    // count is the number of triangles (tris) 
    T  volume = T();
    for(int i = 0; i < num_indices / 3; i++) { // for each triangle
        volume += determinant(vertices[indices[i * 3 + 0]],
                              vertices[indices[i * 3 + 1]],
                              vertices[indices[i * 3 + 2]]);
    }
    return volume / T(6.0);  // since the determinant give 6 times tetra volume
}

template<typename T> 
vec<T, 3> center_of_mass(vec<T, 3> const* vertices, int const* indices, int num_indices) 
{
    // count is the number of triangles (tris) 
    vec<T, 3> com(0, 0, 0);

    T  volume = 0; // actually accumulates the volume*6
    for(int i = 0; i < num_indices / 3; i++)  // for each triangle
    {
        T vol = determinant(vertices[indices[i * 3 + 0]],
                            vertices[indices[i * 3 + 1]],
                            vertices[indices[i * 3 + 2]]);
        com += vol * (vertices[indices[i * 3 + 0]] + vertices[indices[i * 3 + 1]] + vertices[indices[i * 3 + 2]]); 
        volume += vol;
    }
    com /= volume * 4.0f; 
    return com;
}

template<typename T> 
mat<T, 3, 3> inertia(vec<T, 3> const* vertices, int const* indices, int num_indices, vec<T, 3> com = vec<T, 3>(0, 0, 0)) 
{
    typedef vec<T, 3> vec3_t;
    typedef mat<T, 3, 3> mat3x3_t;
    // count is the number of triangles (tris) 
    // The moments are calculated based on the center of rotation com which is [0,0,0] by default
    // assume mass==1.0  you can multiply by mass later.
    // for improved accuracy the next 3 variables, the determinant d, and its calculation should be changed to double
    T volume = 0;                          // technically this variable accumulates the volume times 6
    vec3_t diag(0,0,0);                       // accumulate matrix main diagonal integrals [x*x, y*y, z*z]
    vec3_t offd(0,0,0);                       // accumulate matrix off-diagonal  integrals [y*z, x*z, x*y]

    for(int i=0; i < num_indices / 3; i++)  // for each triangle
    {
        vec3_t const &v0(vertices[indices[i * 3 + 0]]);
        vec3_t const &v1(vertices[indices[i * 3 + 1]]);
        vec3_t const &v2(vertices[indices[i * 3 + 2]]);
        mat3x3_t A(v0[0] - com[0], v0[1] - com[1], v0[2] - com[2],
                   v1[0] - com[0], v1[1] - com[1], v1[2] - com[2],
                   v2[0] - com[0], v2[1] - com[1], v2[2] - com[2]);
        // vol of tiny parallelapiped= d * dr * ds * dt (the 3 partials of my tetral triple integral equation)
        T d = determinant(vec<T, 3>(A(0, 0), A(1, 0), A(2, 0)),
                          vec<T, 3>(A(0, 1), A(1, 1), A(2, 1)),
                          vec<T, 3>(A(0, 2), A(1, 2), A(2, 2)));  
        volume +=d;             // add vol of current tetra (note it could be negative - that's ok we need that sometimes)
        for(int j = 0; j < 3; ++j) {
            int j1=(j+1)%3;   
            int j2=(j+2)%3;   
            diag[j] += (A(0, j) * A(1, j) + A(1, j) * A(2, j) + A(2, j) * A(0, j) + 
                        A(0, j) * A(0, j) + A(1, j) * A(1, j) + A(2, j) * A(2, j)  ) * d;         // divide by 60.0f later;
            offd[j] += (A(0, j1) * A(1, j2)  + A(1, j1) * A(2, j2)  + A(2, j1) * A(0, j2)  +
                        A(0, j1) * A(2, j2)  + A(1, j1) * A(0, j2)  + A(2, j1) * A(1, j2)  +
                        A(0, j1) * A(0, j2) * T(2) + A(1, j1) * A(1, j2) * T(2) + A(2, j1) * A(2, j2) * T(2)) *d; // divide by 120.0f later
        }
    }
    diag /= volume * (T(60.0) / T(6.0));  // divide by total volume (vol/6) since density=1/volume
    offd /= volume * (T(120.0) / T(6.0));
    return mat3x3_t(diag[1] + diag[2]  , -offd[2]      , -offd[1],
                   -offd[2]        , diag[0] + diag[2], -offd[0],
                   -offd[1]        , -offd[0]      , diag[0] + diag[1] );
}

#endif
