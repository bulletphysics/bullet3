#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_MEAN_CURVATURE_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_MEAN_CURVATURE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>

#include <OpenTissue/core/containers/grid/util/grid_gradient.h>
#include <OpenTissue/core/containers/grid/util/grid_hessian.h>

namespace OpenTissue
{
  namespace grid
  {

    template<typename grid_type, typename real_type>
    inline void mean_curvature(
      grid_type const & grid
      , size_t i
      , size_t j
      , size_t k
      , real_type & K
      )
    {
      using std::min;
      using std::max;
      using std::pow;

      typedef OpenTissue::math::Vector3<real_type>          vector3_type;
      typedef OpenTissue::math::Matrix3x3<real_type>        matrix3x3_type;

      real_type limit_K = boost::numeric_cast<real_type>(   1. / min( grid.dx(), min( grid.dy(), grid.dz() ) )    );

      vector3_type g;
      matrix3x3_type H;

      gradient( grid, i, j, k, g );
      hessian( grid, i, j, k, H );
      real_type h = g * g;

      //--- Test whether the gradient was zero, if so we simply imagine it has norm one, a better
      //--- solution would proberly be to pick a random node and compute the curvature information
      //--- herein (this is suggest by Oscher and Fedkiw).
      if ( h == 0 )
        h = 1;
      //--- Compute Mean curvature, defined as: kappa = \nabla \cdot (\nabla \phi / \norm{\nabla \phi}  )
      const static real_type exponent = boost::numeric_cast<real_type>( 3. / 2. );
      K = ( 1.0 / pow( h, exponent ) ) * (
        g( 0 ) * g( 0 ) * ( H( 1, 1 ) + H( 2, 2 ) ) - 2. * g( 1 ) * g( 2 ) * H( 1, 2 ) +
        g( 1 ) * g( 1 ) * ( H( 0, 0 ) + H( 2, 2 ) ) - 2. * g( 0 ) * g( 2 ) * H( 0, 2 ) +
        g( 2 ) * g( 2 ) * ( H( 0, 0 ) + H( 1, 1 ) ) - 2. * g( 0 ) * g( 1 ) * H( 0, 1 )
        );
      //--- Clamp Curvature, it does not make sense if we compute
      //--- a curvature value that can not be representated with the
      //--- current map resolution.
      K = min( K, limit_K  );
      K = max( K, -limit_K );
    }

    template<typename grid_iterator,typename real_type>
    inline void mean_curvature (grid_iterator const & iter, real_type &  K)
    {
      typedef typename grid_iterator::grid_type                grid_type;

      size_t       i   = iter.i();
      size_t       j   = iter.j();
      size_t       k   = iter.k();

      grid_type const & grid = iter.get_grid();
      mean_curvature(grid, i, j, k, K);
    }

    template<typename grid_iterator>
    inline typename grid_iterator::math_types::real_type mean_curvature( grid_iterator const & iter )
    {
      typedef typename grid_iterator::math_types::real_type  real_type;

      real_type K;
      mean_curvature(iter, K);

      return K;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_MEAN_CURVATURE_H
#endif
