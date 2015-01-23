#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_KE_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_KE_H
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
      * Compute Stiffness matrix of tetrahedral element.
      *
      * @param p0
      * @param p1
      * @param p2
      * @param p3
      * @param E   Youngs modulus.
      * @param nu  Poisson ratio.
      * @param Ke  Upon return contains the computed stiffness values.
      */
      template<typename real_type, typename vector3_type, typename matrix3x3_type>
      inline void compute_Ke(
        vector3_type const & p0,
        vector3_type const & p1,
        vector3_type const & p2,
        vector3_type const & p3,
        real_type const & E,
        real_type const & nu,
        matrix3x3_type Ke[4][4]
      )
      {
        using std::fabs;

        real_type d = p0(0);
        real_type d4 = p0(1);
        real_type d8 = p0(2);
        real_type d1 = p1(0) - d;
        real_type d5 = p1(1) - d4;
        real_type d9 = p1(2) - d8;
        real_type d2 = p2(0) - d;
        real_type d6 = p2(1) - d4;
        real_type d10 = p2(2) - d8;
        real_type d3 = p3(0) - d;
        real_type d7 = p3(1) - d4;
        real_type d11 = p3(2) - d8;
        real_type d12 = (d1 * d6 * d11 + d2 * d7 * d9 + d3 * d5 * d10) - d1 * d7 * d10 - d2 * d5 * d11 - d3 * d6 * d9;
        real_type d13 = 1.0 / d12;
        real_type d14 = fabs(d12 / 6.0);
        vector3_type B[4];
        B[0].clear();
        B[1].clear();
        B[2].clear();
        B[3].clear();
        B[1](0) = (d10 * d7 - d6 * d11) * d13;
        B[2](0) = (d5 * d11 - d9 * d7) * d13;
        B[3](0) = (d9 * d6 - d5 * d10) * d13;
        B[0](0) = -B[1](0) - B[2](0) - B[3](0);
        B[1](1) = (d2 * d11 - d10 * d3) * d13;
        B[2](1) = (d9 * d3 - d1 * d11) * d13;
        B[3](1) = (d1 * d10 - d9 * d2) * d13;
        B[0](1) = -B[1](1) - B[2](1) - B[3](1);
        B[1](2) = (d6 * d3 - d2 * d7) * d13;
        B[2](2) = (d1 * d7 - d5 * d3) * d13;
        B[3](2) = (d5 * d2 - d1 * d6) * d13;
        B[0](2) = -B[1](2) - B[2](2) - B[3](2);
        real_type d15 = E / (1.0 + nu) / (1.0 - 2 * nu);
        real_type d16 = (1.0 - nu) * d15;
        real_type d17 = nu * d15;
        real_type d18 = E / 2 / (1.0 + nu);
        for(int i = 0; i < 4; ++i)
        {
          for(int j = 0; j < 4; ++j)
          {
            real_type d19 = B[i](0);
            real_type d20 = B[i](1);
            real_type d21 = B[i](2);
            real_type d22 = B[j](0);
            real_type d23 = B[j](1);
            real_type d24 = B[j](2);
            Ke[i][j](0,0) = d16 * d19 * d22 + d18 * (d20 * d23 + d21 * d24);
            Ke[i][j](0,1) = d17 * d19 * d23 + d18 * (d20 * d22);
            Ke[i][j](0,2) = d17 * d19 * d24 + d18 * (d21 * d22);
            Ke[i][j](1,0) = d17 * d20 * d22 + d18 * (d19 * d23);
            Ke[i][j](1,1) = d16 * d20 * d23 + d18 * (d19 * d22 + d21 * d24);
            Ke[i][j](1,2) = d17 * d20 * d24 + d18 * (d21 * d23);
            Ke[i][j](2,0) = d17 * d21 * d22 + d18 * (d19 * d24);
            Ke[i][j](2,1) = d17 * d21 * d23 + d18 * (d20 * d24);
            Ke[i][j](2,2) = d16 * d21 * d24 + d18 * (d20 * d23 + d19 * d22);
            Ke[i][j] *= d14;
          }
        }
      }

      /**
      * Compute Stiffness matrix of tetrahedral element.
      *
      */
      template<typename real_type, typename vector3_type, typename matrix3x3_type>
      inline void compute_Ke(
        vector3_type * B,
        vector3_type const& D,
        real_type const & volume,
        matrix3x3_type Ke[4][4]
      )
      {
        for (unsigned int i=0; i<4; ++i)
          for (unsigned int j=i; j<4; ++j)
            compute_Ke_ij( B[i], D, B[j], volume, Ke[i][j] );

        for (unsigned int i=1; i<4; ++i)
          for (unsigned int j=0; j<i; ++j)
            Ke[i][j] = trans(Ke[j][i]);
      }

      /**
      * Compute i,j sub-block of the element stiffness matrix Ke
      *
      * @param Bi     Sub-block entries of B matrix of i'th node given in vector-form. Ie. Bi = [bi ci di]^T.
      * @param D      The elasticity matrix in vector form D = [D0 D1 D2]^T.
      * @param Bj     Sub-block entries of B matrix of i'th node given in vector-form. Ie. Bj = [bj cj dj]^T.
      * @param V      The volume of the tetrahedron.
      * @param Ke_ij  Upon return, contains the computed value.
      */
      template<typename real_type, typename vector3_type, typename matrix3x3_type>
      inline void compute_Ke_ij(
        vector3_type const& Bi,
        vector3_type const& D,
        vector3_type const& Bj,
        real_type const & volume,
        matrix3x3_type & Ke_ij
        )
      {
        assert(Ke_ij.size1() == 3 || !"compute_Ke_ij(): Incompatible dimension");
        assert(Ke_ij.size2() == 3 || !"compute_Ke_ij(): Incompatible dimension");

        //
        // Calculate Ke_ij = Bi^T D Bj V
        //
        // The matrix Bi is given by:
        //
        //          | bi 0  0  |
        //          | 0  ci 0  |
        //   Bi =   | 0  0  di |
        //          | ci bi 0  |
        //          | di 0  bi |
        //          | 0  di ci |
        //
        //  Similar for Bj. The elasticity matrix D is given by
        //
        //
        //                  -                                             -     -                        -
        //                  | 1-nu  nu   nu      0         0        0     |     | D0  D1  D1  0   0   0  |
        //       young      |  nu  1-nu  nu      0         0        0     |     | D1  D0  D1  0   0   0  |
        // D= ------------- |  nu   nu  1-nu     0         0        0     |     | D1  D1  D0  0   0   0  |
        //    (1+nu)(1-2nu) |  0    0   0    (1-2nu)/2     0        0     |  =  | 0   0   0   D2  0   0  |
        //                  |  0    0   0        0     (1-2nu)/2    0     |     | 0   0   0   0   D2  0  |
        //                  |  0    0   0        0         0    (1-2nu)/2 |     | 0   0   0   0   0   D2 |
        //                  -                                             -     -                        -
        //
        //
        // We have something like
        //
        //  Bi^T D Bj
        //
        // These matrices have a particular pattern, especially the B-matrices:
        //
        //
        //                   |x 0 0|T
        //                   |0 x 0|
        //                   |0 0 x|
        //  | x 0 0 x 0 x|   |x x 0|
        //  | 0 x 0 x x 0| = |0 x x|              Bk(i,j)^T = Bk(j,i)
        //  | 0 0 x 0 x x|   |x 0 x|
        //
        // And the product has the pattern
        //
        //
        //                 | x x x 0 0 0 | |x 0 0|
        //                 | x x x 0 0 0 | |0 x 0|
        //                 | x x x 0 0 0 | |0 0 x|
        //  | x 0 0 x 0 x| | 0 0 0 x 0 0 | |x x 0|
        //  | 0 x 0 x x 0| | 0 0 0 0 x 0 | |0 x x|
        //  | 0 0 x 0 x x| | 0 0 0 0 0 x | |x 0 x|
        //
        //
        // If we compute T = Bi^T E, then we get a matrix with pattern
        //
        //  | x x x x 0 x|
        //  | x x x x x 0|
        //  | x x x 0 x x|
        //
        // Now computing  Kij = T Bj,
        //
        //                            |x 0 0 |
        //                            |0 x 0 |
        //                            |0 0 x |
        //  |x x x |   | x x x x 0 x| |x x 0 |
        //  |x x x | = | x x x x x 0| |0 x x |
        //  |x x x |   | x x x 0 x x| |x 0 x |
        //
        //
        // and exploiting that for isotropic materials we have,
        //
        //  D(0,0) = D(1,1) = D(2,2) = D0
        //  D(0,1) = D(1,2) = D(0,2) = D(1,0) = D(2,1) = D(2,0) = D1
        //  D(3,3) = D(4,4) = D(5,5) = D2
        //
        // Finally we do not need to store the B-matrices, observe that since
        //
        //  b(0) = invC(1,0);  b(1) = invC(1,1);  b(2) = invC(1,2);  b(3) = invC(1,3);
        //  c(0) = invC(2,0);  c(1) = invC(2,1);  c(2) = invC(2,2);  c(3) = invC(2,3);
        //  d(0) = invC(3,0);  d(1) = invC(3,1);  d(2) = invC(3,2);  d(3) = invC(3,3);
        //
        // and
        //
        //        | bi  0  0 |
        //        |  0 ci  0 |
        //   Bi = |  0  0 di |
        //        | ci bi  0 |
        //        |  0 di ci |
        //        | di  0 bi |
        //
        // therefore
        //
        //        | invC(1,i)    0         0      |
        //        |    0      invC(2,i)    0      |
        //   Bi = |    0         0      invC(3,i) |
        //        | invC(2,i) invC(1,i)    0      |
        //        |    0      invC(3,i) invC(2,i) |
        //        | invC(3,i)    0      invC(1,i) |
        //
        // Putting it all together and using matlab, we have,
        //
        //syms bi  ci  di real
        //Bi = [ bi  0  0 ;  0 ci  0 ;  0  0 di ; ci bi  0 ; 0 di ci ; di  0 bi ]
        //syms bj  cj  dj real
        //Bj = [ bj  0  0 ;  0 cj  0 ;  0  0 dj ; cj bj  0 ; 0 dj cj ; dj  0 bj ]
        //syms G H J real
        //D = [ G H H 0 0 0; H G H 0 0 0; H H G 0 0 0; 0 0 0 J 0 0; 0 0 0 0 J 0; 0 0 0 0 0 J]
        //T = Bi'*E
        //Kij = T*Bj
        //syms invC1i invC2i invC3i real;
        //Kij = subs(Kij,bi,invC1i);
        //Kij = subs(Kij,ci,invC2i);
        //Kij = subs(Kij,di,invC3i);
        //syms invC1j invC2j invC3j real;
        //Kij = subs(Kij,bj,invC1j);
        //Kij = subs(Kij,cj,invC2j);
        //Kij = subs(Kij,dj,invC3j);
        //Kij = collect(Kij,G)
        //Kij = collect(Kij,H)
        //Kij = collect(Kij,J)
        //
        //  which yields
        //
        //             -                                                                                             -
        //             | D0 bi bj + D2 (ci cj + di dj)  D1 bi cj + D2 ci bj            D1 bi dj + D2 di bj           |
        //   Ke_ij = V | D1 ci bj + D2 bi cj            D0 ci cj + D2 (bi bj + di dj)  D1 ci dj + D2 di cj           |
        //             | D1 di bj + D2 bi dj            D1 di cj + D2 ci dj            D0 di dj + D2 (bi bj + ci cj) |
        //             -                                                                                             -
        real_type bi = Bi(0);
        real_type ci = Bi(1);
        real_type di = Bi(2);

        real_type bj = Bj(0);
        real_type cj = Bj(1);
        real_type dj = Bj(2);

        real_type D0 = D(0)*volume;
        real_type D1 = D(1)*volume;
        real_type D2 = D(2)*volume;

        Ke_ij(0,0) = D0 * bi * bj   +   D2 * (ci * cj + di * dj);
        Ke_ij(0,1) = D1 * bi * cj   +   D2 * (ci * bj);
        Ke_ij(0,2) = D1 * bi * dj   +   D2 * (di * bj);

        Ke_ij(1,0) = D1 * ci * bj   +   D2 * (bi * cj);
        Ke_ij(1,1) = D0 * ci * cj   +   D2 * (bi * bj + di * dj);
        Ke_ij(1,2) = D1 * ci * dj   +   D2 * (di * cj);

        Ke_ij(2,0) = D1 * di * bj   +   D2 * (bi * dj);
        Ke_ij(2,1) = D1 * di * cj   +   D2 * (ci * dj);
        Ke_ij(2,2) = D0 * di * dj   +   D2 * (ci * cj + bi * bj);
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_KE_H
#endif
