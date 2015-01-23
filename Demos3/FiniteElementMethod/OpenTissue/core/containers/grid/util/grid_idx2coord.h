#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_IDX_TO_COORD_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_IDX_TO_COORD_H
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
    * Index To Coordinate Conversion.
    * This method computes the coordinates of the voxel with indices (i,j,k)
    *
    * @param map     The grid.
    * @param i       The index of the voxel along the I-axe.
    * @param j       The index of the voxel along the J-axe.
    * @param k       The index of the voxel along the K-axe.
    * @param point   Upon return this parameter contains the coordinates.
    */
    template <typename grid_type, typename vector3_type>
    inline void idx2coord( grid_type const & grid, size_t i, size_t j, size_t k, vector3_type & point )
    {
      typedef typename vector3_type::value_type real_type;
      assert(i<grid.I() || !"idx2coord(): i'th index was too big");
      assert(j<grid.J() || !"idx2coord(): j'th index was too big");
      assert(k<grid.K() || !"idx2coord(): k'th index was too big");

      real_type const & dx = grid.dx();
      real_type const & dy = grid.dy();
      real_type const & dz = grid.dz();
      real_type const & mx = grid.min_coord( 0 );
      real_type const & my = grid.min_coord( 1 );
      real_type const & mz = grid.min_coord( 2 );

      point(0) = i * dx + mx;
      point(1) = j * dy + my;
      point(2) = k * dz + mz;
    }

    /**
    * Index To Coordinate Conversion.
    * This method computes the coordinates of the voxel with indices (i,j,k)
    *
    * @param iter    An iterator to the voxel.
    * @param point   Upon return this parameter contains the coordinates.
    */
    template<typename grid_iterator,typename vector3_type>
    inline void idx2coord (grid_iterator const & iter, vector3_type & point )
    {
      typedef typename grid_iterator::grid_type                 grid_type;

      size_t       i   = iter.i();
      size_t       j   = iter.j();
      size_t       k   = iter.k();

      grid_type const & grid = iter.get_grid();

      idx2coord(grid, i, j, k, point);
    }

    /**
    * Index To Coordinate Conversion.
    * This method computes the coordinates of the voxel with indices (i,j,k)
    *
    * @param iter    An iterator to the voxel.
    * @return        Holds the coordinates.
    */
    template<typename grid_iterator>
    inline typename grid_iterator::math_types::vector3_type idx2coord (grid_iterator const & iter )
    {
      typedef typename grid_iterator::math_types::vector3_type  vector3_type;

      vector3_type point;

      idx2coord(iter, point);

      return point;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_IDX_TO_COORD_H
#endif
