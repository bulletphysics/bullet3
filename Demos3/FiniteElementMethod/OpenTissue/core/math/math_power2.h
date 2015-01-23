#ifndef OPENTISSUE_CORE_MATH_MATH_POWER2_H
#define OPENTISSUE_CORE_MATH_MATH_POWER2_H
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
    * This method test whether there exist some positive integer, n, such that
    *
    *  2^n == val
    *
    * In which case it returns true otherwise it returns false.
    */
    template< class T>
    inline bool is_power2( T val )
    {
      T next = 1u;
      for ( unsigned int i = 0u; i < 32u; ++i )
      {
        if ( val == next )
          return true;
        next = next << 1u;
      }
      return false;
    }

    /**
    * This function finds the smallest positive integer, n, such that
    *
    *   val <= 2^n
    *
    * and returns the value 2^n.
    */
    template<class T>
    inline T upper_power2( T val )
    {
      T next = 1u;
      for ( unsigned int i = 0u; i < 32u; ++i )
      {
        if ( next >= val )
          return next;
        next = next << 1u;
      }
      return 0u;
    }

    /**
    * This function finds the largest positive integer, n, such that
    *
    *   2^n <= val
    *
    * and returns the value 2^n.
    */
    template<class T>
    inline T lower_power2( T val )
    {
      T next = 1u << 31u;
      for ( unsigned int i = 0u; i < 32u; ++i )
      {
        if ( next <= val )
          return next;
        next = next >> 1u;
      }
      return 0u;
    }

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_POWER2_H
#endif
