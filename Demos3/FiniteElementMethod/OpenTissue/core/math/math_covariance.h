#ifndef OPENTISSUE_CORE_MATH_MATH_COVARIANCE_H
#define OPENTISSUE_CORE_MATH_MATH_COVARIANCE_H
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

  namespace math
  {

    /**
    *
    * @param mean   Resulting mean point.
    * @param C      Resulting covariance matrix.
    */
    template< typename vector3_iterator, typename vector3_type, typename matrix3x3_type>
    inline void
      covariance(
      vector3_iterator begin, vector3_iterator end
      , vector3_type & mean, matrix3x3_type & C
      )
    {
      typedef typename vector3_type::value_traits   value_traits;

      unsigned int N = 0;

      mean.clear();

      for(vector3_iterator v = begin;v!=end;++v,++N)
        mean += (*v);
      mean /= N;

      C.clear();
      for(vector3_iterator v = begin;v!=end;++v,++N)
      {
        C(0,0) += ((*v)(0) - mean(0))*((*v)(0) - mean(0));
        C(1,1) += ((*v)(1) - mean(1))*((*v)(1) - mean(1));
        C(2,2) += ((*v)(2) - mean(2))*((*v)(2) - mean(2));
        C(0,1) += ((*v)(0) - mean(0))*((*v)(1) - mean(1));
        C(0,2) += ((*v)(0) - mean(0))*((*v)(2) - mean(2));
        C(1,2) += ((*v)(1) - mean(1))*((*v)(2) - mean(2));
      }
      C(1,0) = C(0,1);
      C(2,0) = C(0,2);
      C(2,1) = C(1,2);
      C /= N;
    }

    /**
    *
    *
    * @param mean   Resulting mean point.
    * @param C      Resulting covariance matrix.
    */
    template<typename vector3_type, typename matrix3x3_type>
    inline void
      covariance_union(
      vector3_type const & mean1, matrix3x3_type const & C1
      , vector3_type const & mean2, matrix3x3_type const & C2
      , vector3_type       & mean,  matrix3x3_type       & C
      )
    {
      typedef typename vector3_type::value_traits   value_traits;

      mean = (mean1 + mean2)/value_traits::two();
      matrix3x3_type KK = outer_prod<matrix3x3_type,vector3_type>(mean,mean);
      matrix3x3_type NN = outer_prod<matrix3x3_type,vector3_type>(mean1,mean1);
      matrix3x3_type MM = outer_prod<matrix3x3_type,vector3_type>(mean2,mean2);
      C = ((C1 + NN + C2 + MM)/value_traits::two() - KK) ;
    }

  } // namespace math

} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_MATH_COVARIANCE_H
#endif
