#ifndef OPENTISSUE_CORE_MATH_MATH_POLAR_DECOMPOSITION_H
#define OPENTISSUE_CORE_MATH_MATH_POLAR_DECOMPOSITION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/math/math_matrix3x3.h>
#include <OpenTissue/core/math/math_eigen_system_decomposition.h>

#include <cmath>


namespace OpenTissue 
{  

  namespace math
  {

    // TODO: [micky] don't introduce yet another sub namespace to avoid name clashing.
    //               Rename the function 'eigen' instead!!
    namespace polar_decomposition 
    {
      /*
      * Polar Decomposition of matrix A (as described by Etzmuss et. al in ``A Fast Finite Solution for Cloth Modelling'')
      *
      *   A = R S
      *
      * Where R is a special orthogonal matrix (R*R^T = I and det(R)=1)
      *
      *   S^2 = A^T A
      * 
      * let d be vector of eigenvalues and let v_0,v_1, and v_2 be corresponding eigenvectors of (A^T A), then
      *
      *   S = sqrt(d_0) v_0 * v_0^T + ... + sqrt(d_2) v_2 * v_2^T
      *
      * Now compute 
      *
      *  R = A * S^-1
      *
      *
      * @return     If the decompostion is succesfull then the return value is true otherwise it is false.
      */
      template<typename matrix3x3_type>
      inline bool eigen(matrix3x3_type const & A,matrix3x3_type & R,matrix3x3_type & S)
      {
        using std::sqrt;

        // A = U D V^T       // A is square so: thin SVD = SVD
        // A = (U V^T)  (V D V^T)
        //   =    R        S
        //
        //Notice S is symmetric  R should be orthonormal?
        //
        //  proof of
        //      S =  sqrt( A^T A )
        //
        //      start by
        //
        //            S * S = A^T A
        // V D V^T V D V^T  =   V D U^T  U D V^T
        //       V D D V^T  =   V D D V^T
        //Assume
        //A = R S
        //pre-multiply and use assumption then
        //  A^T A = A^T R S
        //  A^T A = S^T S = S S
        //  last step used S is symmetric
        typedef typename matrix3x3_type::vector3_type   vector3_type;
        typedef typename matrix3x3_type::value_traits   value_traits;
        matrix3x3_type V;
        vector3_type d;
        matrix3x3_type S2 = trans(A)*A;
        eigen(S2,V,d);

        //--- Test if all eigenvalues are positive
        if( d(0) <= value_traits::zero() || d(1) <= value_traits::zero() || d(2) <= value_traits::zero() )
          return false;

        vector3_type v0 = vector3_type( V(0,0), V(1,0), V(2,0) );
        vector3_type v1 = vector3_type( V(0,1), V(1,1), V(2,1) );
        vector3_type v2 = vector3_type( V(0,2), V(1,2), V(2,2) );
        S =  outer_prod(v0,v0)* sqrt(d(0)) +  outer_prod(v1,v1)* sqrt(d(1)) + outer_prod(v2,v2)* sqrt(d(2));
        R = A * inverse(S);
        return true;
      }

      /**
      * This method is rather bad, it is iterative and there is no way of telling how good the solution is. thus we recommend the eigen method for polar decomposition.
      *
      * Polar Decomposition as described by Shoemake and Duff in ``Matrix Animation and Polar Decomposition''
      *
      *   A = R S
      *
      * Where R is a special orthogonal matrix (R*R^T = I and det(R)=1)
      *
      * Set R_0 = A
      * Do
      *  R_{i+1} = 1/2( R_i + R^{-T}_i )
      * Until R_{i+1}-R_i = 0
      *
      *
      * @return    If a solution was found within the specified threshold value then the return value is true otherwise it is false.
      */
      template<typename matrix3x3_type>
      inline bool newton(matrix3x3_type const & A, matrix3x3_type & R,unsigned int max_iterations, typename matrix3x3_type::value_type const & threshold)
      {
        typedef typename matrix3x3_type::value_traits  value_traits;
        typedef typename matrix3x3_type::value_type    real_type;

        assert(max_iterations>0 || !"polar_decompostion::newton() max_iterations must be positive");
        assert(threshold>value_traits::zero() || !"polar_decomposition::newton(): theshold must be positive");

        matrix3x3_type Q[2];
        int cur = 0, next = 1;
        Q[cur] = A;

        bool within_threshold = false;

        for(unsigned int iteration=0;iteration< max_iterations;++iteration)
        {
          Q[next] = (   Q[cur] + trans(inverse(Q[cur])) )*.5;       
          real_type test = max_value ( Q[next] - Q[cur] );
          if( test < threshold )
          {
            within_threshold = true;
            break;
          }
          cur = (cur + 1)%2;
          next = (next + 1)%2;
        }
        R = Q[next];
        return within_threshold;
      }

    }// namespace polar_decomposition

  }// namespace math
  
}// namespace OpenTissue

// OPENTISSUE_CORE_MATH_MATH_POLAR_DECOMPOSITION_H
#endif

