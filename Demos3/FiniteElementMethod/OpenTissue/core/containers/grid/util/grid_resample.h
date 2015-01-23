#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_RESAMPLE_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_RESAMPLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>

#include <OpenTissue/core/containers/grid/util/grid_value_at_point.h>
#include <OpenTissue/core/containers/grid/util/grid_idx2coord.h>

namespace OpenTissue
{
  namespace grid
  {
    /**
    * Resample a grid to lower dimensions.
    * @param M        Original grid to be resampled.
    * @param m        Destination grid.
    * @param factor_i Scale factor in the I-direction of the grid.
    * @param factor_j Scale factor in the J-direction of the grid.
    * @param factor_k Scale factor in the K-direction of the grid.
    * @return         Upon return the destination grid m contains the resampled grid.
    */
    template <typename grid_type, typename real_type>
    inline void resample(grid_type const & M, grid_type & m, real_type factor_i, real_type factor_j, real_type factor_k)
    {
      using std::floor;

      assert(factor_i > 0 || !"resample(): i scale factor must be positive");
      assert(factor_j > 0 || !"resample(): j scale factor must be positive");
      assert(factor_k > 0 || !"resample(): k scale factor must be positive");

      typedef typename grid_type::math_types      math_types;
      typedef typename math_types::vector3_type   vector3_type;
      typedef typename grid_type::index_iterator  index_iterator;

      // TODO: Add gaussian smoothing with a standard deviation corresponding to factor_n.
      //grid_type S = M;
      //grid_gaussian_convolution(M,S,factor_i/2.0,factor_j/2.0,factor_k/2.0);

      m.create(
        M.min_coord(), M.max_coord()
        , static_cast<size_t>( floor(M.I()/factor_i) )
        , static_cast<size_t>( floor(M.J()/factor_j) )
        , static_cast<size_t>( floor(M.K()/factor_k) )
        );

      vector3_type point;
      for( index_iterator iter = m.begin(); iter != m.end(); ++iter )
      {
        idx2coord( iter,point );
        (*iter) = value_at_point( M, point );
      }
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_RESAMPLE_H
#endif
