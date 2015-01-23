#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_IS_POINT_INSIDE_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_IS_POINT_INSIDE_H
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
  namespace grid
  {
    /**
    * Point Inclusion Test.
    *
    * Checks if a given points is strictly inside the grid.
    * Notice if point lies on boundary of grid, then
    * it is not consider to be inside the grid.
    *
    * @param point     Vector with coordinates for a point.
    * @return          True if point is strictly inside. False otherwise.
    */
    template<typename grid_type,typename vector3_type>
    inline bool is_point_inside( grid_type const & grid, vector3_type const & point )
    {
      for ( size_t i = 0u;i < 3u;++i )
      {
        if ( point( i ) <= grid.min_coord( i ) )
          return false;
        if ( point( i ) >= grid.max_coord( i ) )
          return false;
      }
      return true;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_IS_POINT_INSIDE_H
#endif
