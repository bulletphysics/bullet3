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

//quat.h

#ifndef MVL_QUAT_H
#define MVL_QUAT_H

#include <limits>

#include "vec.h"
#include "mat.h"

//quaternions are vectors of size 4
// it's assumed that the layout is in the form (w, (x, y, z)),
// so that the identity quaternion is (1, 0, 0, 0)

namespace mvl {

//quaternion conjugate
template<typename T>
inline vec<T, 4>
qconj(vec<T, 4> const& rhs) {
    return vec<T, 4>(-rhs[0], rhs[1], rhs[2], rhs[3]);
}

//quaternion identity
template<typename T>
inline 
vec<T, 4>
qidentity() {
    return vec<T, 4>(1, 0, 0, 0);
}

//quaternion - quaternion  product
template<typename T1, typename T2>
inline 
vec<typename promote_traits<T1, T2>::value_type, 4>
qprod(vec<T1, 4> const& lhs, vec<T2, 4> const& rhs) {
   typedef typename promote_traits<T1, T2>::value_type value_type;
   return vec<value_type, 4>((lhs(0)*rhs(0)) - (lhs(1)*rhs(1)) - (lhs(2)*rhs(2)) - (lhs(3)*rhs(3)),
                             (lhs(0)*rhs(1)) + (lhs(1)*rhs(0)) + (lhs(2)*rhs(3)) - (lhs(3)*rhs(2)),
                             (lhs(0)*rhs(2)) - (lhs(1)*rhs(3)) + (lhs(2)*rhs(0)) + (lhs(3)*rhs(1)),
                             (lhs(0)*rhs(3)) + (lhs(1)*rhs(2)) - (lhs(2)*rhs(1)) + (lhs(3)*rhs(0)));
} 
  
//quanternion - vector product (rotation)
template<typename T1, typename T2>
inline 
vec<typename promote_traits<T1, T2>::value_type, 3>
qprod(vec<T1, 4> const& q, vec<T2, 3> const& v) {
    typedef typename promote_traits<T1, T2>::value_type value_type;
    vec<value_type, 4> tmp = qprod(qprod(q, vec<value_type, 4>(0, v[0], v[1], v[2])), qconj(q));
    return vec<value_type, 3>(tmp[0], tmp[1], tmp[2]);
}

//spherical interpolation between q0 and q1
template <typename T1, typename T2, typename T3>
inline 
vec<typename promote_traits<T1, T2>::value_type, 4>
qslerp(vec<T1, 4> const& q1, vec<T2, 4> const& q2, T3 t) {
    typedef typename promote_traits<T1, T2>::value_type value_type;
	value_type omega, cosom, sinom, scale0, scale1;
	vec<value_type, 4> tmp;

	cosom =	dot(q1, q2);

	if (cosom < static_cast<value_type>(0.0)) {
			cosom = -cosom;
			tmp = -q2;
	} else {
		tmp = q2;
	}

	if ((static_cast<value_type>(1.0) - cosom) > std::numeric_limits<value_type>::epsilon()) {
		omega  = (value_type) acos(cosom);
		sinom  = sin(omega);
		scale0 = sin((static_cast<value_type>(1.0) - t) * omega) / sinom;
		scale1 = sin(t * omega) / sinom;
	} else {
		scale0 = static_cast<value_type>(1.0) - t;
		scale1 = t;
	}

    return scale0 * q1 + scale1 * tmp;
}

//init quaternion from axis-angle
template<typename T1, typename T2> 
inline 
vec<typename promote_traits<T1, T2>::value_type, 4>
q_from_axis_angle(vec<T1, 3> const& axis, T2 theta)  {
    typedef typename promote_traits<T1, T2>::value_type value_type;
    value_type sin_theta = sin(static_cast<value_type>(static_cast<value_type>(0.5)) * theta);
    return vec<value_type, 4>(cos(static_cast<value_type>(static_cast<value_type>(0.5)) * theta),
                              sin_theta * axis[0],
                              sin_theta * axis[1],
                              sin_theta * axis[2]);
}                    

//get the axis/angle from quaternion
template<typename T1, typename T2, typename T3> 
inline 
void
q_to_axis_angle(vec<T1, 4> const& q, vec<T2, 3>& axis, T3& theta)
{
    T3 half_theta= acos(q[0]);
	
    if(half_theta > 10 * std::numeric_limits<T3>::epsilon()) {
        T3 oost = 1 / sin(half_theta);
		
	axis[0] = oost * q[1];
	axis[1] = oost * q[2];
	axis[2] = oost * q[3];
	theta = 2 * half_theta;
    } else {
        axis[0] = axis[1] = axis[2] = 0;
        theta = 0;
    }
}

//init quaternion from rotation matrix
template<typename T1> 
inline 
vec<T1, 4>
q_from_mat(mat<T1, 3, 3> const& m)  {
    T1 trace, s, hos;
    trace = m(0, 0) + m(1, 1) + m(2, 2);
    if (trace > static_cast<T1>(0.0)) {
        s = sqrt(trace + static_cast<T1>(1.0));
        hos = static_cast<T1>(0.5) / s;
        return vec<T1, 4>(s * static_cast<T1>(0.5), (m(2, 1) - m(1, 2)) * hos, (m(0, 2) - m(2, 0)) * hos, (m(1, 0) - m(0, 1)) * hos);
    } else {
        int biggest;
        enum {A,T,I};
        if (m(0, 0) > m(1, 1))	{
            if (m(2, 2) > m(0, 0)) biggest = I;	
            else biggest = A;
        } else {
            if (m(2, 2) > m(0, 0)) biggest = I;
            else biggest = T;
        }
        switch (biggest) {
        case A:
            s = sqrt( m(0, 0) - (m(1, 1) + m(2, 2)) + static_cast<T1>(1.0));
            if (s > (100 * std::numeric_limits<T1>::epsilon())) {
                hos = static_cast<T1>(0.5) / s;
                return vec<T1, 4>((m(2, 1) - m(1, 2)) * hos, s * static_cast<T1>(0.5), (m(0, 1) + m(1, 0)) * hos, (m(0, 2) + m(2, 0)) * hos);
            }
            // I
            s = sqrt( m(2, 2) - (m(0, 0) + m(1, 1)) + static_cast<T1>(1.0));
            if (s > (100 * std::numeric_limits<T1>::epsilon())) {
                hos = static_cast<T1>(0.5) / s;
                return vec<T1, 4>((m(1, 0) - m(0, 1)) * hos, (m(2, 0) + m(0, 2)) * hos, (m(2, 1) + m(1, 2)) * hos, s * static_cast<T1>(0.5));
            }
            // T
            s = sqrt( m(1, 1) - (m(2, 2) + m(0, 0)) + static_cast<T1>(1.0));
            if (s > (100 * std::numeric_limits<T1>::epsilon())) {
                hos = static_cast<T1>(0.5) / s;
                return vec<T1, 4>((m(0, 2) - m(2, 0)) * hos, (m(1, 0) + m(0, 1)) * hos, s * static_cast<T1>(0.5), (m(1, 2) + m(2, 1)) * hos);
            }
            break;
        case T:
            s = sqrt( m(1, 1) - (m(2, 2) + m(0, 0)) + static_cast<T1>(1.0));
            if (s > (100 * std::numeric_limits<T1>::epsilon())) {
                hos = static_cast<T1>(0.5) / s;
                return vec<T1, 4>((m(0, 2) - m(2, 0)) * hos, (m(1, 0) + m(0, 1)) * hos, s * static_cast<T1>(0.5), (m(1, 2) + m(2, 1)) * hos);
            }
            // I
            s = sqrt( m(2, 2) - (m(0, 0) + m(1, 1)) + static_cast<T1>(1.0));
            if (s > (100 * std::numeric_limits<T1>::epsilon())) {
                hos = static_cast<T1>(0.5) / s;
                return vec<T1, 4>((m(1, 0) - m(0, 1)) * hos, (m(2, 0) + m(0, 2)) * hos, (m(2, 1) + m(1, 2)) * hos, s * static_cast<T1>(0.5));
            }
            // A
            s = sqrt( m(0, 0) - (m(1, 1) + m(2, 2)) + static_cast<T1>(1.0));
            if (s > (100 * std::numeric_limits<T1>::epsilon())) {
                hos = static_cast<T1>(0.5) / s;
                return vec<T1, 4>((m(2, 1) - m(1, 2)) * hos, s * static_cast<T1>(0.5), (m(0, 1) + m(1, 0)) * hos, (m(0, 2) + m(2, 0)) * hos);
            }
            break;
        case I:
            s = sqrt( m(2, 2) - (m(0, 0) + m(1, 1)) + static_cast<T1>(1.0));
            if (s > (100 * std::numeric_limits<T1>::epsilon())) {
                hos = static_cast<T1>(0.5) / s;
                return vec<T1, 4>((m(1, 0) - m(0, 1)) * hos, (m(2, 0) + m(0, 2)) * hos, (m(2, 1) + m(1, 2)) * hos, s * static_cast<T1>(0.5));
            }
            // A
            s = sqrt( m(0, 0) - (m(1, 1) + m(2, 2)) + static_cast<T1>(1.0));
            if (s > (100 * std::numeric_limits<T1>::epsilon())) {
                hos = static_cast<T1>(0.5) / s;
                return vec<T1, 4>((m(2, 1) - m(1, 2)) * hos, s * static_cast<T1>(0.5), (m(0, 1) + m(1, 0)) * hos, (m(0, 2) + m(2, 0)) * hos);
            }
            // T
            s = sqrt( m(1, 1) - (m(2, 2) + m(0, 0)) + static_cast<T1>(1.0));
            if (s > (100 * std::numeric_limits<T1>::epsilon())) {
                hos = static_cast<T1>(0.5) / s;
                return vec<T1, 4>((m(0, 2) - m(2, 0)) * hos, (m(1, 0) + m(0, 1)) * hos, s * static_cast<T1>(0.5), (m(1, 2) + m(2, 1)) * hos);
            }
            break;
        }
    }	
}                    

//get rotation matrix from quaternion
template<typename T> 
inline 
void
q_to_mat(vec<T, 4> const& q, mat<T, 3, 3>& m) {
	T X2,Y2,Z2;		//2*QX, 2*QY, 2*QZ
	T XX2,YY2,ZZ2;	//2*QX*QX, 2*QY*QY, 2*QZ*QZ
	T XY2,XZ2,XW2;	//2*QX*QY, 2*QX*QZ, 2*QX*QW
	T YZ2,YW2,ZW2;	// ...

	X2  = 2 * q[1];
	XX2 = X2 * q[1];
	XY2 = X2 * q[2];
	XZ2 = X2 * q[3];
	XW2 = X2 * q[0];

	Y2  = 2 * q[2];
	YY2 = Y2 * q[2];
	YZ2 = Y2 * q[3];
	YW2 = Y2 * q[0];
	
	Z2  = 2 * q[3];
	ZZ2 = Z2 * q[3];
	ZW2 = Z2 * q[0];
	
	m(0, 0) = 1 - YY2 - ZZ2;
	m(0, 1) = XY2  - ZW2;
	m(0, 2) = XZ2  + YW2;

	m(1, 0) = XY2  + ZW2;
	m(1, 1) = 1 - XX2 - ZZ2;
	m(1, 2) = YZ2  - XW2;

	m(2, 0) = XZ2  - YW2;
	m(2, 1) = YZ2  + XW2;
	m(2, 2) = 1 - XX2 - YY2;

}

template<typename T, size_t R, size_t C>
mat<T, R, C>
cmat(T const* m)
{
    mat<T, R, C> res;
    for(int i = 0; i < R; ++i) {
        for(int j = 0; j < C; ++j) {
            res(i, j) = m[i * C + j];
        }
    }
    return res;
}

} //namespace mvl


#endif
