#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_COORD_TO_IDX_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_COORD_TO_IDX_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>

namespace OpenTissue
{
  namespace grid
  {
    /**
    * Coordinate 2 Voxel Indices.
    * This method computes the indices (i,j,k) of
    * the voxel containing the specified point.
    *
    * The voxel index of the voxel containing the points is equal to index
    * of the grid node of the containing voxel with lowest coordinates
    * (ie. lower-left-back grid node).
    *
    * @param grid     The grid.
    * @param point   The point.
    * @param i       Upon return this parameter contains the index of the voxel along the I-axe.
    * @param j       Upon return this parameter contains the index of the voxel along the J-axe.
    * @param k       Upon return this parameter contains the index of the voxel along the K-axe.
    */
    template <typename grid_type, typename vector3_type>
    inline void coord2idx( grid_type const & grid, vector3_type const & point, size_t & i, size_t & j, size_t & k )
    {
      using std::floor;

      typedef typename vector3_type::value_type real_type;

      real_type const & dx = grid.dx();
      real_type const & dy = grid.dy();
      real_type const & dz = grid.dz();
      real_type const & mx = grid.min_coord( 0 );
      real_type const & my = grid.min_coord( 1 );
      real_type const & mz = grid.min_coord( 2 );

      real_type xval = ( point( 0 ) - mx ) / dx;
      i = static_cast<size_t>( floor( xval ) );
      if ( ( xval - i ) > .5 )
        ++i;
      real_type yval = ( point( 1 ) - my ) / dy;
      j = static_cast<size_t>( floor( yval ) );
      if ( ( yval - j ) > .5 )
        ++j;
      real_type zval = ( point( 2 ) - mz ) / dz;
      k = static_cast<size_t>( floor( zval ) );
      if ( ( zval - k ) > .5 )
        ++k;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_COORD_TO_IDX_H
#endif
