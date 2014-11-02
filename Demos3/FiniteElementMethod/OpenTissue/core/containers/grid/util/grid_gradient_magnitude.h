#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRADIENT_MAGNITUDE_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRADIENT_MAGNITUDE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_gradient.h> 
#include <OpenTissue/core/math/math_vector3.h>         

#include <cmath>  

namespace OpenTissue
{
  namespace grid
  {

    template<typename grid_type, typename real_type>
    inline void gradient_magnitude(
      grid_type const & grid
      , size_t i
      , size_t j
      , size_t k
      , real_type & grad_mag
      )
    {
      using std::sqrt;

      typedef OpenTissue::math::Vector3<real_type> vector3_type;
      vector3_type g;
      gradient(map,i,j,k,g);
      grad_mag = sqrt(g*g);
    }

    template<typename grid_iterator, typename real_type>
    inline void gradient_magnitude (grid_iterator const & iter, real_type & grad_mag )
    {
      typedef typename grid_iterator::grid_type               grid_type;

      size_t       i   = iter.i();
      size_t       j   = iter.j();
      size_t       k   = iter.k();

      grid_type const & grid = iter.get_grid();

      gradient_magnitude(grid, i, j, k, grad_mag);
    }

    template<typename grid_iterator>
    inline typename grid_iterator::math_types::real_type gradient_magnitude( grid_iterator const & iter )
    {
      typedef typename grid_iterator::math_types::real_type  real_type;

      real_type grad_mag;

      gradient_magnitude(iter, grad_mag);

      return grad_mag;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRADIENT_MAGNITUDE_H
#endif
