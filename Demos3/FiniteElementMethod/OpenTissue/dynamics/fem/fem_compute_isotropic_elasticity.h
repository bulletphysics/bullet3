#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_ISOTROPIC_ELASTICITY_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_ISOTROPIC_ELASTICITY_H
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
      * Compute Isotropic Elasticity Matrix.
      *
      * @param young     Young Modulus.
      * @param poisson   Poisson ratio.
      * @param D         Upon return holds the elasticity matrix in vector form D = [D0,D1,D2]
      */
      template<typename real_type, typename vector3_type>
      inline void compute_isotropic_elasticity_vector( 
        real_type const & young
        , real_type const & poisson
        , vector3_type & D
        )
      {
        assert(young>0       || !"compute_isotropic_elasticity_vector(): Young modulus must be positive");
        assert(poisson>0     || !"compute_isotropic_elasticity_vector(): Poisson ratio must be positive");
        assert(poisson<0.5   || !"compute_isotropic_elasticity_vector(): Poisson ratio must be less than a half");

        // The isotropic Elasticity matrix D is given by
        //               -                                             -     -                        -
        //               | 1-nu  nu   nu      0         0        0     |     | D0  D1  D1  0   0   0  |
        //    young      |  nu  1-nu  nu      0         0        0     |     | D1  D0  D1  0   0   0  |
        // ------------- |  nu   nu  1-nu     0         0        0     |     | D1  D1  D0  0   0   0  |
        // (1+nu)(1-2nu) |  0    0   0    (1-2nu)/2     0        0     |  =  | 0   0   0   D2  0   0  |
        //               |  0    0   0        0     (1-2nu)/2    0     |     | 0   0   0   0   D2  0  |
        //               |  0    0   0        0         0    (1-2nu)/2 |     | 0   0   0   0   0   D2 |
        //               -                                             -     -                        -

        real_type poisson2 = 2.0*poisson;
        real_type scale = young / ((1.0 + poisson) * (1.0 - poisson2));
        D(0) = (1.0 - poisson) * scale;
        D(1) = poisson * scale;
        D(2) = young / (2.0  + poisson2);
      }

      /**
      * Compute Elasticy Matrix.
      *
      * @param young     Youngs modulus (stiffness)
      * @param poisson   Poissons ratio (compressability)
      * @param D         Upon return holds the isotropoc linear elasticity matrix.
      */
      template<typename real_type,typename matrix_type>
      inline void compute_isotropic_elasticity_matix(
        real_type const & young
        , real_type const & poisson
        , matrix_type & D
        )
      {
        assert(young>0       || !"compute_isotropic_elasticity_matix(): Young modulus must be positive");
        assert(poisson>0     || !"compute_isotropic_elasticity_matix(): Poisson ratio must be positive");
        assert(poisson<0.5   || !"compute_isotropic_elasticity_matix(): Poisson ratio must be less than a half");
        assert(D.size1()==6  || !"compute_isotropic_elasticity_matix(): D-matrix did not have 6 rows");
        assert(D.size2()==6  || !"compute_isotropic_elasticity_matix(): D-matrix did not have 6 columns");

        typedef typename matrix_type::value_type value_type;

        value_type lambda = (poisson*young)/(( 1+poisson )*( 1- (2*poisson) ));    //--- Lame modulus
        value_type mu = young/(2*(1+poisson));                                     //--- Shear Modulus
        value_type tmp = lambda+(2*mu);

        D(0,0) = tmp;     D(0,1) = lambda;  D(0,2) = lambda;  D(0,3) = 0;   D(0,4) = 0;   D(0,5) = 0;
        D(1,0) = lambda;  D(1,1) = tmp;     D(1,2) = lambda;  D(1,3) = 0;   D(1,4) = 0;   D(1,5) = 0;
        D(2,0) = lambda;  D(2,1) = lambda;  D(2,2) = tmp;     D(2,3) = 0;   D(2,4) = 0;   D(2,5) = 0;
        D(3,0) = 0;       D(3,1) = 0;       D(3,2) = 0;       D(3,3) = mu;  D(3,4) = 0;   D(3,4) = 0;
        D(4,0) = 0;       D(4,1) = 0;       D(4,2) = 0;       D(4,3) = 0;   D(4,4) = mu;  D(4,5) = 0;
        D(5,0) = 0;       D(5,1) = 0;       D(5,2) = 0;       D(5,3) = 0;   D(5,4) = 0;   D(5,5) = mu;
      }

      /**
      * Alternative representation of isotropic elasticity matrix.
      *
      *
      * @param young     Youngs modulus (stiffness)
      * @param poisson   Poissons ratio (compressability)
      * @param D00
      * @param D01
      * @param D33
      *
      */
      template<typename real_type>
      inline void compute_isotropic_elasticity_vector(
        real_type const & young
        , real_type const & poisson
        , real_type & D00
        , real_type & D01
        , real_type & D33
        )
      {
        assert(young>0       || !"compute_isotropic_elasticity_vector(): Young modulus must be positive");
        assert(poisson>0     || !"compute_isotropic_elasticity_vector(): Poisson ratio must be positive");
        assert(poisson<0.5   || !"compute_isotropic_elasticity_vector(): Poisson ratio must be less than a half");

        D01 = (poisson*young)/(( 1 + poisson )*( 1- (2*poisson) ));    //--- Lame modulus
        D33 = young/(2*(1+poisson));                                   //--- Shear Modulus
        D00 = D01+(2*D33);
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_ISOTROPIC_ELASTICITY_H
#endif
