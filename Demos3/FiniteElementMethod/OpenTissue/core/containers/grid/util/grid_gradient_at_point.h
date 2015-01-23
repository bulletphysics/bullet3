#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_GRADIENT_AT_POINT_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_GRADIENT_AT_POINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_trillinear.h>
#include <OpenTissue/core/containers/grid/util/grid_enclosing_indices.h>
#include <OpenTissue/core/containers/grid/util/grid_gradient.h>

namespace OpenTissue
{
  namespace grid
  {

    template<typename grid_type,typename vector3_type>
    inline vector3_type gradient_at_point (grid_type const & grid,  vector3_type const & point )
    {
      typedef typename grid_type::value_type   value_type;

      const static value_type infty = grid.infinity();
      const static value_type unused = grid.unused();

      size_t i0, j0, k0, i1, j1, k1;
      enclosing_indices(grid, point, i0, j0, k0, i1, j1, k1 );

      vector3_type g000,g001,g010,g011,g100,g101,g110,g111;
      gradient( grid, i0, j0, k0, g000 );
      gradient( grid, i1, j0, k0, g001 );
      gradient( grid, i0, j1, k0, g010 );
      gradient( grid, i1, j1, k0, g011 );
      gradient( grid, i0, j0, k1, g100 );
      gradient( grid, i1, j0, k1, g101 );
      gradient( grid, i0, j1, k1, g110 );
      gradient( grid, i1, j1, k1, g111 );

      if ( g000( 0 ) == unused )
        return vector3_type( infty, infty, infty );
      if ( g001( 0 ) == unused )
        return vector3_type( infty, infty, infty );
      if ( g010( 0 ) == unused )
        return vector3_type( infty, infty, infty );
      if ( g011( 0 ) == unused )
        return vector3_type( infty, infty, infty );
      if ( g100( 0 ) == unused )
        return vector3_type( infty, infty, infty );
      if ( g101( 0 ) == unused )
        return vector3_type( infty, infty, infty );
      if ( g110( 0 ) == unused )
        return vector3_type( infty, infty, infty );
      if ( g111( 0 ) == unused )
        return vector3_type( infty, infty, infty );

      typename vector3_type::value_type s = ( point(0) - ( i0*grid.dx() + grid.min_coord(0) ) ) / grid.dx();
      typename vector3_type::value_type t = ( point(1) - ( j0*grid.dy() + grid.min_coord(1) ) ) / grid.dy();
      typename vector3_type::value_type u = ( point(2) - ( k0*grid.dz() + grid.min_coord(2) ) ) / grid.dz();

      return  OpenTissue::math::trillinear(g000, g001, g010, g011, g100, g101, g110, g111, s, t, u ) ;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_GRADIENT_AT_POINT_H
#endif
