#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_VALUE_AT_POINT_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_VALUE_AT_POINT_H
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

namespace OpenTissue
{
  namespace grid
  {
    /**
    *
    *
    * Warning this method do not test whether the point lies inside the grid. If the point is
    * outside the behavior is undefined.
    */
    template<typename grid_type,typename vector3_type>
    inline typename grid_type::value_type value_at_point(grid_type const & grid, vector3_type const & point )
    {
      typedef typename grid_type::value_type      value_type;
      typedef typename vector3_type::value_type  real_type;

      //const static value_type infty  = grid.infinity();
      const static value_type unused = grid.unused();
      const static value_type zero   = value_type();  //--- by standard default constructed is zero!!!

      size_t i0, j0, k0, i1, j1, k1;
      enclosing_indices( grid, point, i0, j0, k0, i1, j1, k1 );

      value_type d000 = grid( i0, j0, k0 );
      value_type d001 = grid( i1, j0, k0 );
      value_type d010 = grid( i0, j1, k0 );
      value_type d011 = grid( i1, j1, k0 );
      value_type d100 = grid( i0, j0, k1 );
      value_type d101 = grid( i1, j0, k1 );
      value_type d110 = grid( i0, j1, k1 );
      value_type d111 = grid( i1, j1, k1 );

      size_t cnt_unused = 0;

#define MAGIC(val) \
  if ( (val) == unused) { \
  (val) = zero; \
  ++cnt_unused; \
  }
      MAGIC( d000 );
      MAGIC( d001 );
      MAGIC( d010 );
      MAGIC( d011 );
      MAGIC( d100 );
      MAGIC( d101 );
      MAGIC( d110 );
      MAGIC( d111 );
#undef MAGIC

      if ( cnt_unused == 8 )
        return unused;

      real_type s = ( point(0) - ( i0*grid.dx() + grid.min_coord(0) ) ) / grid.dx();
      real_type t = ( point(1) - ( j0*grid.dy() + grid.min_coord(1) ) ) / grid.dy();
      real_type u = ( point(2) - ( k0*grid.dz() + grid.min_coord(2) ) ) / grid.dz();
      return OpenTissue::math::trillinear( d000, d001, d010, d011, d100, d101, d110, d111, s, t, u );
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_VALUE_AT_POINT_H
#endif
