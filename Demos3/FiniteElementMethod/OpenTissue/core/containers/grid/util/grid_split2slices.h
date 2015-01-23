#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_SPLIT2SLICES_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_SPLIT2SLICES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>

namespace OpenTissue
{
  namespace grid
  {

    /**
    * Split a 3D grid into multiple 2D grids along some major axis.
    *
    * @param grid         The grid that should be split inot slices.
    * @param slices      Upon return this container holds grids corresponding to all the slices.
    * @param axis        The axis to split along.
    */
    template <typename grid_type,typename grid_container>
    inline void split2slices(grid_type & grid, grid_container & slices, size_t axis = 2)
    {
      typename grid_type::math_types      math_types;
      typename math_types::vector3_type   vector3_type;

      assert( axis==2 || !"split2slices(): Only splits along k-axis is currently supported");

      slices.clear();

      if (axis != 2) 
        return;

      vector3_type min_coord = grid.min_coord();
      vector3_type max_coord = grid.max_coord();
      min_coord(2) = 0;
      max_coord(2) = 0;
      for (size_t z = 0; z < grid.K(); ++z)
      {

        grid_type slice;
        slice.create(min_coord, max_coord, grid.I(), grid.J(), 2);
        for (size_t y = 0; y < grid.J(); ++y)
          for (size_t x = 0; x < grid.I(); ++x)
          {
            slice(x, y, 0) = grid(x, y, z);
            slice(x, y, 1) = grid(x, y, z); // HACKING!!!
          }
          slices.push_back(slice);
      }
    }

  } // namespace grid

} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_SPLIT2SLICES_H
#endif
