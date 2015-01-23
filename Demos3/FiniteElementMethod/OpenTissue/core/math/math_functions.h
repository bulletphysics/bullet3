#ifndef OPENTISSUE_CORE_MATH_MATH_FUNCTIONS_H
#define OPENTISSUE_CORE_MATH_MATH_FUNCTIONS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>



namespace OpenTissue
{

  /**
   * The following functions are defined below:
   * - clamp (and the variations: clamp_min, clamp_max, clamp_zero_one)
   * - fac (faculty)
   * - sgn (sign function)
   * - sinc
   */	
  namespace math
  {

    /**
     * clamp function to clamp a value to be in the interval (min_value; max_value).
     *
     * @param value      The value to be clamped.
     * @param min_value  The minimum allowed value.
     * @param max_value  The maximum allowed value (ohhh, really?).
     * @return           The clamped value. If value already is in (min_value; max_value) no clamping is performed.
     */
    template<typename T>
    inline T clamp(T const & value, T const & min_value, T const & max_value)
    {
      assert(min_value <= max_value || !"max_value cannot be less than min_value");
      using std::min;
      using std::max;

      return T(min(max_value, max(min_value, value)));
    }


    /**
     * clamp function to clamp a value never to be less than min_value.
     *
     * @param value      The value to be clamped.
     * @param min_value  The minimum allowed value.
     * @return           The clamped value if value is less than min_value, otherwise the original value is returned.
     */
    template<typename T>
    inline T clamp_min(T const & value, T const & min_value)
    {
    	using std::max;
      return clamp(value, min_value, max(value, min_value));
    }


    /**
     * clamp function to clamp a value never to be greater than max_value.
     *
     * @param value      The value to be clamped.
     * @param max_value  The maximum allowed value.
     * @return           The clamped value if value is greater than max_value, otherwise the original value is returned.
     */
    template<typename T>
    inline T clamp_max(T const & value, T const & max_value)
    {
    	using std::min;
      return clamp(value, min(value, max_value), max_value);
    }


    /**
     * clamp function to easily clamp a value between 0 and 1.
     * note: this function is mostly usable for T = some real type.
     *
     * @param value      The value to be clamped.
     * @return           The clamped value. If value already is in (0; 1) no clamping is performed.
     */
    template<typename T>
    inline T clamp_zero_one(T const & value)
    {
      return clamp(value, detail::zero<T>(), detail::one<T>());
    }


    template<typename T>
    inline T fac(unsigned long n)
    {
      // TODO what about implicit type conversions? This could have been done more elegangtly using partial specialization  
      unsigned long val = 1;
      for(; n > 0; val *= n--);
      return T(val);
    }


    template<typename T>
    inline T sgn(T const & val)
    {
      static T const zero = detail::zero<T>();
      static T const one  = detail::one<T>();
      return val > zero ? one : val < zero ? -one : zero;
    }


    /**
    * Compute Sinc Function.
    * The implementation of this method was greatly inspired by the
    * one in Open Dynamics Engine v. 0.039
    *
    * This method returns sin(x)/x. this has a singularity at 0 so special
    * handling is needed for small arguments.
    *
    * @param x
    * @return   The value of sin(x)/x
    */
    template<typename T>
    inline T sinc(T & x)
    {
      using std::fabs;
      using std::sin;

      static T const tiny   = (T)(1.0e-4);
      static T const factor = (T)(0.166666666666666666667);

      //--- if |x| < 1e-4 then use a taylor series expansion. this two term expansion
      //--- is actually accurate to one LS bit within this range if double precision
      //--- is being used - so don't worry!
      return (fabs(x) < tiny) ? (detail::one<T>() - x*x*factor) : (sin(x)/x);
    }


    // use this to convert radians into degrees
    template<typename T>
    inline T to_degrees(T const & radians)
    {
      return radians*detail::radian<T>();
    }


    // use this to convert degrees into radians
    template<typename T>
    inline T to_radians(T const & degrees)
    {
      return degrees*detail::degree<T>();
    }

  }  // namespace math
  
}  // namespace OpenTissue

// #define OPENTISSUE_CORE_MATH_MATH_FUNCTIONS_H
#endif
