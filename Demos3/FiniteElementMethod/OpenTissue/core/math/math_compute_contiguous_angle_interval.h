#ifndef OPENTISSUE_CORE_MATH_COMPUTE_CONTIGUOUS_ANGLE_INTERVAL_H
#define OPENTISSUE_CORE_MATH_COMPUTE_CONTIGUOUS_ANGLE_INTERVAL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_is_number.h>

#include <algorithm>
#include <vector>
#include <stdexcept>
#include <cmath>

namespace OpenTissue
{
  namespace math
  {

    /**
    * Compute Contiguous Angle Interval.
    * This function tries to find a contiguous interval of angle values. As
    * input it takes a sequence of angle values. From this sequence the
    * function tries to compute the starting and ending angle (in terms
    * of counter-clock-wise rotation order) of a contiguous interval.
    * interval containing all the angles.
    *
    * One could interact with the function as follows
    *
    *  std::vector<double> angles;
    *
    *  ... fill angles with values ...
    *
    *  double min_val, max_val;
    *  compute_contiguous_angle_interval( angles.begin(), angles.end(), min_val, max_val );
    *
    * The resulting minimum and maximum values yields the starting and ending points
    * of the contiguous interval. The periodicity of rotation angles is used to adjust
    * the end-points of the interval to always be positive values.
    *
    * The supplied angles can be given in any sort of interval as long as
    * the width of the interval is no longer than 2 pi ( one full rotation). Typically
    * one would supply values from either -pi..pi or 0..2pi intervals. 
    *
    * @param begin      An iterator to the first angle value
    * @param end        An iterator to one past the last angle value
    * @param theta_min  Upon return this value holds the starting value of the contiguous angle interval.
    * @param theta_max  Upon return this value holds the ending value of the contiguous angle interval.
    */
    template<typename iterator_type>
    inline void compute_contiguous_angle_interval( 
      iterator_type const & begin
      , iterator_type const & end
      , typename iterator_type::value_type & theta_min
      , typename iterator_type::value_type & theta_max )
    {
      //using std::cos;
      //using std::sin;
      //using std::sqrt;
      //using std::atan2;

      typedef typename iterator_type::value_type         T;
      typedef          OpenTissue::math::ValueTraits<T>  value_traits;


      T const two_pi = value_traits::pi()*value_traits::two();

      // Determine the number of samples
      size_t const N = std::distance( begin, end);

      // Make a copy of the samples and sort them in ascending order
      std::vector<T> storage;
      storage.resize( N );
      std::copy( begin, end, storage.begin() );
      std::sort( storage.begin(), storage.end() );

      // Find the two theta values with the largest gap inbetween. That
      // is the largest angle difference as measured in a counter-clock-wise manner.
      T      max_delta_theta = value_traits::zero();
      size_t max_i           = N;

      for(size_t i = 0u;i<N;++i)
      {
        T const & theta_i = storage[i];
        T const & theta_j = storage[(i+1)%N];
        T const delta_theta = (theta_j < theta_i) ?  theta_j+two_pi-theta_i : theta_j-theta_i;
        if(delta_theta > max_delta_theta)
        {
          max_i = i;
          max_delta_theta = delta_theta;
          theta_min = theta_j;
          theta_max = theta_i;
        }
      }

      //// Compute the mean angle by converting angles to points on
      //// the unit-circle and calculating the mean point.
      //T mx = value_traits::zero();
      //T my = value_traits::zero();
      //for(iterator_type theta = begin; theta != end; ++ theta)
      //{        
      //  T const c = cos( *theta );
      //  T const s = sin( *theta );
      //  mx += c;
      //  my += s;
      //}

      //T norm = sqrt( mx*mx + my*my);
      //assert( is_number(norm) || !"compute_contiguous_angle_interval(): NaN encountered");

      //if(norm<= value_traits::zero())
      //  throw std::logic_error("insufficient angle samples to determine a mean angle"); 

      //mx /= norm;
      //my /= norm;

      //assert( is_number(mx) || !"compute_contiguous_angle_interval(): NaN encountered");
      //assert( is_number(my) || !"compute_contiguous_angle_interval(): NaN encountered");

      while(theta_min < value_traits::zero()) 
        theta_min += two_pi;

      while(theta_max < theta_min) 
        theta_max += two_pi;

      assert( theta_min <= theta_max || !"compute_contiguous_angle_interval(): invalid interval");
    }


  } // namespace math
} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_COMPUTE_CONTIGUOUS_ANGLE_INTERVAL_H
#endif
