#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_B_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_B_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace fem
  {
    namespace detail
    {

      /**
      * Calculate B-matrices (derivatives of shape functions N, i.e B = SN).
      *
      * @param  e10      Given the four corner points p0,p1,p2, and p3. This is p1-p0.
      * @param  e20      Given the four corner points p0,p1,p2, and p3. This is p2-p0.
      * @param  e30      Given the four corner points p0,p1,p2, and p3. This is p3-p0.
      * @param  volume   The volume of the tetrahedron.
      * @param  B        Upon return this array of 4 B-vectors contains the computed values.
      */
      template<typename real_type, typename vector3_type>
      inline void compute_B(
        vector3_type const& e10,
        vector3_type const& e20,
        vector3_type const& e30,
        real_type volume,
        vector3_type * B
        )
      {
        //
        // In summary we want to calculate 
        //
        //   B = S N 
        //
        //  where
        //
        //   B = [ B0 B1 B2 B3 ], N = [ N0 N1 N2 N3 ]
        //
        //  Here S is the operational matrix given by
        //        -                  -
        //        | d/dx  0     0    |
        //        | 0     d/dy  0    |
        //        | 0     0     d/dz |
        //   S =  | d/dy  d/dx  0    |
        //        | d/dz  0     d/dx |
        //        | 0     d/dz  d/dy |
        //        -                  -
        // N_i's are called the shape functions and they are used to interpolate values. For
        // 3D linear elastostatics these can be written as
        //
        //         -           -
        //         | w_i 0  0  |
        //   N_i = | 0  w_i 0  |
        //         | 0  0  w_i |
        //         -           -
        //
        //  The w_i's are functions in the coordinates x,y, and z and will be derived
        //  shortly below of here.
        //
        // The matrix B_i is given by:
        //
        //         | b_i  0    0   |
        //         | 0    c_i  0   |
        //   B_i = | 0    0    d_i |
        //         | c_i  b_i  0   |
        //         | d_i  0    b_i |
        //         | 0    d_i  c_i |
        //
        // The entries are stored as
        //
        //   B[i] = [ b_i c_i d_i ]
        //
        // In the following we go into details of deriving formulas for the interpolating
        // functions N_i. From these formulas it follows how to compute the derivatives.
        // Finally we show a clever way of implementing the computaitons.
        //
        //   Using a linear polynomial for interpolating a u(x,y,z)-value at a given
        //   position x,y,z inside the  tetrahedron
        //
        //          u(x,y,z)  =  a_0  +  a_1 x  + a_2 y + a_3 z  =  P^T A 
        //
        //    where the a's are the polynomial coefficients (to be determed) and
        //
        //              | 1 |            | a0 |
        //         P =  | x |  , and A = | a1 |
        //              | y |            | a2 |
        //              | z |            | a3 |
        //
        //    Say we know the u-values, u0(x0,y0,z0),...,u3(x3,y2,z3), at the four tetrahedron corner
        //    points, P0^T [x0 y0 z0],...,P3^T=[x3 y3 z3], then we can set up the linear system
        //
        //          | u0 |    | 1 x0  y0  z0 | | a0 |
        //          | u1 | =  | 1 x1  y1  z1 | | a1 | = C A
        //          | u2 |    | 1 x2  y2  z2 | | a2 |
        //          | u3 |    | 1 x3  y3  z3 | | a3 |
        //
        //    The C matrix is invertible (as long as the four points are in general postion, meaning
        //    that no point can be written as a linear combination of any of the three other points),
        //    so we can solve the polynomial coefficients by
        //
        //                     | u0 |
        //           A = C^-1  | u1 |
        //                     | u2 |
        //                     | u3 |
        //    Substitution into the polynomial interpolation formula yields
        //
        //                                      | u0 |
        //       u(x,y,z) = P^T A =  P^T C^{-1} | u1 |   (*1)
        //                                      | u2 |
        //                                      | u3 |
        //
        //   Denoting the i'th column of,  P^T C^{-1}, by N_i we have  u(x,y,z) = sum_i N_i u_i
        //
        //
        //   Let us now try to look at the bary-centric coordinates, w0,...,w1 of (x,y,z), these
        //   can also be used for interpolation 
        //
        //                                                                | u0 |       |u0|
        //      u(z,y,z) = w0 u0 + w1 u1 + w2 u2 + w3 u3 =  [w0 w1 w2 w3] | u1 | = W^T |u1|
        //                                                                | u2 |       |u2|
        //                                                                | u3 |       |u3|
        //
        //    From the coordinates of the four points and the condition 1 = w0 + w1 + w2 + w3 we can set
        //    up the linear system
        //
        //       | 1 |    |  1    1   1   1  |  |  w0 |
        //       | x | =  |  x0  x1  x2  x3  |  |  w1 | 
        //       | y |    |  y0  y1  y2  y3  |  |  w2 |
        //       | z |    |  z0  z1  z2  z3  |  |  w3 |
        //
        //
        //         P    =          Q                W
        //
        //   Since the four points are in general postion, Q is invertible and we can solve for W
        //
        //         W = Q^{-1} P
        //
        //   Insertion into the barycentric interpolation formula yields
        //
        //                         |u0|                   |u0|               |u0|
        //          u(x,y,z) = W^T |u1| = ( Q^{-1} P )^T  |u1|  = P^T Q^{-T} |u1|    (*2)
        //                         |u2|                   |u2|               |u2|
        //                         |u3|                   |u3|               |u3|
        //
        //   Comparision with the polynomial interpolation derivation we see that C = Q^T, furthermore
        //   we notice that w_i = N_i. So (not very surprinsingly) bary-centric interpolation is really
        //   just linear polynomial interpolation.
        //
        //   Notice that for the volume, V, of the tetrahedron we have the relation: 6 V = det( C ) = det ( Q^T )
        //
        //   When computing the stiffness matrix we are interested in derivatives of the N_i's with
        //   respect to the x,y and z coordinates.
        //
        //
        //    From (*1) and (*2) we see (recall we use zero-indexing)
        //
        //           d N_i
        //           -----  = C^{-1}_{1,i}   =   Q^{-T}_{1,i}        (*3a)
        //            d x
        //
        //           d N_i
        //           -----  = C^{-1}_{2,i}   =   Q^{-T}_{2,i}        (*3b)
        //            d y
        //
        //           d N_i
        //           -----  = C^{-1}_{3,i}   =   Q^{-T}_{3,i}        (*3c)
        //            d z
        //
        //   Instead of actually computing the inverse of the 4x4 C or Q matrices a computational
        //   more efficient solution can be derived, which only requires us to solve for a 3x3 system. 
        //
        //   By Cramers rule we have
        //     
        //                      (-1)^{i+j} det( C_ji)
        //     C^{-1}_{i,j} =  ----------------------
        //                              det(C)
        //
        //   where det(C_ji) is the determinant of C with the j'th row and i'th column removed.
        //
        //   Now defined 
        //
        //       e10  = P1 - P0
        //       e20  = P2 - P0
        //       e30  = P3 - P0
        //
        //   and the matrix E
        //
        //           |  e10^T |    | (x1-x0)  (y1-y0)  (z1-z0) |
        //       E = |  e20^T |  = | (x2-x0)  (y2-y0)  (z2-z0) | 
        //           |  e30^T |    | (x3-x0)  (y3-y0)  (z3-z0) |
        //
        //   Then
        //
        //         det(C) = det(E)      and   C^{-1}_{i+1,j+1}  =  E^{-1}_{i,j}        (*4)
        // 
        //    This can shown by straightforward computation, immediately is is verified that
        //
        //            {-1}^((i+1)+(j+1)) =  {-1}^(i+j)
        //
        //    So we have to show
        //
        //       det( C_(i+1,j+1))      det( E_(ij))
        //    --------------------- =  --------------
        //         det(C)                   det(E)
        //
        // assume det(C)= det(E) (left for reader as exercise:-) then inorder to prove 
        // second half of (*4) we have to show
        //
        //     det( C_(i+1,j+1)) = det( E_(ij))
        //
        // A total of 9 cases exist, for here we will show a single case and leave
        // the remaining cases for the reader, use i=1 and j= 0, implying 
        //         
        //     det( C_(12)) = det( E_(01))
        //
        // So
        //      -          -
        //      | 1 x0  z0 |         -                -
        //  det | 1 x2  z2 |  =  det | (x2-x0) (z2-z0)|
        //      | 1 x3  z3 |         | (x3-x0) (z3-z0)|
        //      -          -         -                -
        //
        // Which is trivially true.  So we have
        // 
        //              | .   ...   |             | .  ...    |
        //   C^{-1}  =  | .  E^{-1} | ;  Q^{-T} = | .  E^{-T} |
        //
        //  As can be seen from (*3) we do not have all the values needed since
        // the first columns of C^{-1} and Q^{-T} are missing. To remedy this problem we can use
        // the condition of the bary-centric coordinates
        //
        //         w0 =  1 - w1 - w2 - w3
        //
        // Taking the derivate wrt. x,y, and z yields
        //
        //           d N_0 
        //           -----  = 1 - E^{-1}_01 - E^{-1}_02 - E^{-1}_03      (*5a)
        //            d x
        //
        //           d N_0
        //           -----  = 1 - E^{-1}_11 - E^{-1}_12 - E^{-1}_13       (*5b)
        //            d y
        //
        //           d N_0
        //           -----  = 1 - E^{-1}_21 - E^{-1}_22 - E^{-1}_23        (*5c)
        //            d z
        //
        // 
        //  and for i>0 we have
        //
        //           d N_i 
        //           -----  = E^{-1}_0i                (*6a)
        //            d x
        //
        //           d N_i 
        //           -----  = E^{-1}_1i                (*6b)
        //            d y
        //
        //
        //           d N_i 
        //           -----  = E^{-1}_2i                 (*6c)
        //            d z
        //
        // The notation b_i = d N_i / dx, c_i = d N_i / dy, and d_i = d N_i / dz is used. Furthermore
        // all the derivatives are returned as four B-vectors, where
        //
        //          B[i] = [ b_i c_i d_i]
        //
        // 
        // 
        // The above may seem as a mathematical trick, so let us try to derive our
        // equations a litlle differently, but before doing so we must first develop
        // some equations for the volume of a tetrahedron.
        // 
        //                     | e10^T |
        //      6 V = det(E) = | e20^T |  = e10 \cdot (e20 \times e30)
        //                     | e30^T |
        //
        // where e10 = p1-p0, e20 = p2-p0 and so on.
        // In general given the right-hand order i,j,k, and m of the nodes we
        // write vol(i,j,k,m) =   eji \cdot (eki \times emi) / 6
        // 
        // Barycentric coordinates are infact the volume weighted weights coresponding to
        // the four tetrahedra lying inside the tetrahedron and having apex at the
        // point p=[x,y,z]^T and bases equal to the 4 triangular faces of the enclosing
        // tetrahedron. To realize this we will examine bary-centric coordinates
        // in the 2D case, that is the case of the Triangle. In 2D area corresponds
        // to the volume, so given a trianle and a point p inside it
        //
        //                + 2
        //               /|
        //              / |
        //             /  |
        //            /   |
        //           / + p|
        //          /     |
        //         /      |
        //      0 +-------+ 1
        //
        // Let the area weighted weights be defined as 
        //
        //         w_0 =  area(1,2,p) / area(0,1,2) 
        //         w_1 =  area(0,P,2) / area(0,1,2)  =  area(2,0,P) / area(0,1,2)
        //         w_2 =  area(0,1,p) / area(0,1,2)  
        //
        // It is intuitive to see that if p moves towards the i'th corner point then
        // the nominator of w_i goes towards area(0,1,2). This means that w_i -> 1 
        // while w_j ->0 for j\neq i.
        //
        //  Having introduced the barycentric coordinates as the area weighted weights
        // it is quite intuitive to see that we also must have
        // 
        //      1 = \sum_i  w_i
        // 
        // In 3D the barycentric coordinates are the volume weighted weights. That is
        // given the right handed order 0,1,2 and 3 of the corner points, we have by
        // analogy to the 2D case
        // 
        //           vol(0,1,2,p)
        //    w_3 = ------------   =  (p-p0) \codt ( e10 \times \e20 ) / 6V
        //               V 
        // 
        //           vol(0,1,3,p)
        //    w_2 = ------------   =  (p-p0) \codt ( e10 \times \e30 ) / 6V
        //               V 
        // 
        //           vol(0,2,3,p)
        //    w_1 = ------------   =  (p-p0) \codt ( e20 \times \e30 ) / 6V
        //               V 
        // 
        //           vol(1,3,2,p)
        //    w_0 = ------------   =  (p-p0) \codt ( e31 \times \e21 ) / 6V
        //               V 
        // 
        // But since we allready know w_3, w_2 and w_1 it is faster to compute w_0 as
        //
        //           w_0 = 1 - w_1  - w_2   - w_3 
        //
        // Recal from previous that we seek the derivatives of w_i's wrt. the coordinaes when
        // we compute the B functions.   Let us write the [x y z] coordinates as [x_0 x_1 x_2] then
        //
        //   d w_3
        //  -------  = ( e10 \times \e20 )_{x_j} / 6V
        //   d x_j
        //
        // That is the j'th coordinate of the cross product divided by 6 times the volume. Similar for
        //
        //   d w_2
        //  -------  = ( e10 \times \e30 )_{x_j} / 6V
        //   d x_j
        //
        //   d w_1
        //  -------  = ( e20 \times \e30 )_{x_j} / 6V
        //   d x_j
        //
        // and finally
        //
        //   d w_0                    d w_i
        //  -------  = 1 - sum_i>0   -------
        //   d x_j                    d x_j
        //
        // Our formulas may seem differnt from those in (*6), however 
        //
        //   d w_k
        //  -------  = ( eji \times \emi )_{x_j} / 6V   =  E_{-1}_{k,j}
        //   d x_j
        //
        // That is the adjoint of E_(j,k) is given by ( eji \times \emi )_{x_j} / 6.
        //
        //
        //

        real_type div6V = 1/(6.0*volume);

        B[1](0) = (e20(2) * e30(1) - e20(1) * e30(2)) * div6V; // b0 = -det(E_11), (where E_ij is the submatrix of E with row i and column j removed)
        B[2](0) = (e10(1) * e30(2) - e10(2) * e30(1)) * div6V; // b1 =  det(E_12)
        B[3](0) = (e10(2) * e20(1) - e10(1) * e20(2)) * div6V; // b2 = -det(E_13)
        B[0](0) = -B[1](0) - B[2](0) - B[3](0); // b3 = -b0 - b1 - b2

        B[1](1) = (e20(0) * e30(2) - e20(2) * e30(0)) * div6V; // c0 =  det(E_21)
        B[2](1) = (e10(2) * e30(0) - e10(0) * e30(2)) * div6V; // c1 = -det(E_22)
        B[3](1) = (e10(0) * e20(2) - e10(2) * e20(0)) * div6V; // c2 =  det(E_23)
        B[0](1) = -B[1](1) - B[2](1) - B[3](1); // c3 = -c0 - c1 - c2

        B[1](2) = (e20(1) * e30(0) - e20(0) * e30(1)) * div6V; // d0 = -det(E_31)
        B[2](2) = (e10(0) * e30(1) - e10(1) * e30(0)) * div6V; // d1 =  det(E_32)
        B[3](2) = (e10(1) * e20(0) - e10(0) * e20(1)) * div6V; // d2 = -det(E_33)
        B[0](2) = -B[1](2) - B[2](2) - B[3](2); // d3 = -d0 - d1 - d2
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_B_H
#endif
