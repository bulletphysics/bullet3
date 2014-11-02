#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_ENCLOSING_INDICES_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_ENCLOSING_INDICES_H
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
    * Get Enclosing Indices.
    * Finds enclosing node indices for a given point.
    * Assumes that the point is strictly inside the volume.
    *
    * @param grid       The grid.
    * @param point     Vector with coordinates of the point.
    * @param i0        Upon return, this parameter contains the i-index of the lower-left-back node.
    * @param j0        Upon return, this parameter contains the j-index of the lower-left-back node.
    * @param k0        Upon return, this parameter contains the k-index of the lower-left-back node.
    * @param i1        Upon return, this parameter contains the i-index of the upper-right-front node.
    * @param j1        Upon return, this parameter contains the j-index of the upper-right-front node.
    * @param k1        Upon return, this parameter contains the k-index of the upper-right-front node.
    */
    template <typename grid_type, typename vector3_type>
    inline void enclosing_indices( 
      grid_type const & grid
      , vector3_type const & point
      , size_t& i0
      , size_t& j0
      , size_t& k0
      , size_t& i1
      , size_t& j1
      , size_t& k1 
      )
    {
      using std::max;
      using std::min;
      using std::floor;

      typedef typename vector3_type::value_type real_type;

      real_type  const & dx = grid.dx();
      real_type  const & dy = grid.dy();
      real_type  const & dz = grid.dz();
      size_t const & I  = grid.I();
      size_t const & J  = grid.J();
      size_t const & K  = grid.K();

      vector3_type range = grid.max_coord() - grid.min_coord();
      vector3_type diff  = point - grid.min_coord();

      diff = min(  max(diff, vector3_type( real_type() ) ) , range);

      diff(0)  /= dx;
      diff(1)  /= dy;
      diff(2)  /= dz;

      i0 = static_cast<size_t>( floor( diff(0) ) );
      j0 = static_cast<size_t>( floor( diff(1) ) );
      k0 = static_cast<size_t>( floor( diff(2) ) );
      i1 = ( i0 + 1 ) % I;
      j1 = ( j0 + 1 ) % J;
      k1 = ( k0 + 1 ) % K;

    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_ENCLOSING_INDICES_H
#endif
