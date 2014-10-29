#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_SECOND_DERIVATVE_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_SECOND_DERIVATVE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_gradient.h>
#include <OpenTissue/core/containers/grid/util/grid_hessian.h>
#include <OpenTissue/core/math/math_vector3.h>   // Needed for vector3.
#include <OpenTissue/core/math/math_matrix3x3.h>   // Needed for matrix3x3.

namespace OpenTissue
{
  namespace grid
  {

    template<typename grid_type, typename real_type>
    inline void second_derivative(
      grid_type const & grid
      , size_t i
      , size_t j
      , size_t k
      , real_type & derivative
      )
    {
      using std::fabs;

      typedef OpenTissue::math::Vector3<real_type>           vector3_type;
      typedef OpenTissue::math::Matrix3x3<real_type>         matrix3x3_type;

      static vector3_type   g;
      static matrix3x3_type H;


      real_type const too_small = 10e-7;

      gradient(grid, i, j, k, g );

      real_type  tmp = g*g;
      if(fabs(tmp) < too_small)
      {
        derivative = 0.0;
        return;
      }

      hessian(grid, i, j, k, H);
      derivative = (1.0)/(tmp)* (g * (H *g));
    }

    template<typename grid_iterator,typename real_type>
    inline void second_derivative (grid_iterator const & iter, real_type &  derivative)
    {
      typedef typename grid_iterator::grid_type                grid_type;

      size_t       i   = iter.i();
      size_t       j   = iter.j();
      size_t       k   = iter.k();

      grid_type const & grid = iter.get_grid();
      second_derivative(grid, i, j, k, derivative);
    }

    template<typename grid_iterator>
    inline typename grid_iterator::math_types::real_type 
      second_derivative( grid_iterator const & iter )
    {
      typedef typename grid_iterator::math_types::real_type  real_type;

      real_type derivative;

      second_derivative(iter, derivative);

      return derivative;
    }

  } // namespace grid

} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_SECOND_DERIVATVE_H
#endif
