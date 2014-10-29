#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_BISECTION_LINE_SEARCH_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_BISECTION_LINE_SEARCH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_gradient_at_point.h>

namespace OpenTissue
{
  namespace grid
  {
    /**
    * Grid Bisection Line Search
    *
    * @param q_a
    * @param q_b
    * @param phi
    * @param maximize   If true the bisection method tries to find the maximimum value between q_a and q_b otherwise it tries to find the minimum value.
    *
    * @return        The point that maximizes the value of phi on the line between q_a and q_b.
    */
    template<typename vector3_type,typename grid_type>
    inline vector3_type bisection_line_search(vector3_type q_a, vector3_type q_b, grid_type & phi, bool maximize = true)
    {
      using std::fabs;
      typedef typename vector3_type::value_type   real_type;

      real_type const precision           = 10e-5;//OpenTissue::math::working_precision<real_type>(100);
      real_type const too_small_interval  = sqr_length(q_b-q_a)*0.0001; //--- 1/100'th of distance!
      vector3_type n = unit(gradient_at_point(phi,q_a));
      vector3_type r;


      real_type const sign = maximize? 1.0 : -1.0;


      bool forever = true;
      do
      {
        vector3_type q_c = (q_a + q_b)*.5;
        if( sqr_length(q_a - q_b) < too_small_interval )
        {
          r = q_c;
          break;
        }
        vector3_type dir       = unit(gradient_at_point(phi,q_c));
        real_type    n_dot_dir = inner_prod(n , dir)*sign;
        if(fabs(n_dot_dir) < precision)
        {
          r = q_c;
          break;
        }
        if(n_dot_dir > 0)
        {
          q_a = q_c;
        }
        if(n_dot_dir < 0)
        {
          q_b = q_c;
        }
      }
      while (forever);
      return r;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_BISECTION_LINE_SEARCH_H
#endif
