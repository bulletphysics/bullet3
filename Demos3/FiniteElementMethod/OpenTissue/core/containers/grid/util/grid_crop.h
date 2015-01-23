#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_CROP_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_CROP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>

#include <OpenTissue/core/containers/grid/util/grid_idx2coord.h>
#include <OpenTissue/core/math/math_vector3.h>

namespace OpenTissue
{
  namespace grid
  {
    /**
    * Crop grid to bounding box that do not include treshold.
    * Can be used to automatically crop emptyness from scanned data.
    * @param M        Original grid to be cropped.
    * @param m        Destination grid.
    * @param treshold Maximum value that needs to be cropped.
    * @return         Upon return the destination grid m contains the cropped grid.
    */
    template < typename grid_type >
    inline void crop(grid_type const & M, grid_type & m, typename grid_type::value_type const & treshold)
    {
      using std::min;
      using std::max;

      typedef typename grid_type::math_types          math_types;
      typedef typename math_types::vector3_type       vector3_type;

      typedef OpenTissue::math::Vector3<size_t>       index_vector;

      index_vector min_idx( M.I(), M.J(), M.K() );
      index_vector max_idx( 0, 0, 0 );

      for(size_t k=0; k<M.K(); ++k )
        for(size_t j=0; j<M.J(); ++j )
          for(size_t i=0; i<M.I(); ++i )
          {
            if ( M(i,j,k) > treshold )
            {
              min_idx = min( min_idx, index_vector(i,j,k) );
              max_idx = max( max_idx, index_vector(i,j,k) );
            }
          }
          index_vector new_dim = (max_idx - min_idx) + index_vector(1);

          vector3_type min_coord,max_coord;      
          idx2coord(M, min_idx(0), min_idx(1), min_idx(2),min_coord);
          idx2coord(M, max_idx(0), max_idx(1), max_idx(2),max_coord);

          m.create( min_coord, max_coord, new_dim(0), new_dim(1), new_dim(2) );

          size_t i_offset = min_idx(0);
          size_t j_offset = min_idx(1);
          size_t k_offset = min_idx(2);
          for(size_t k=0; k<m.K(); ++k )
            for(size_t j=0; j<m.J(); ++j )
              for(size_t i=0; i<m.I(); ++i )
              {
                m(i,j,k) = M( i+i_offset, j+j_offset, k+k_offset );
              }
    }

    /**
    * Crop grid to user specified bounding box.
    * @param M     Original grid to be cropped.
    * @param m     Destination grid.
    * @param min_i Lower i-coord of bounding box.
    * @param min_j Lower j-coord of bounding box.
    * @param min_k Lower k-coord of bounding box.
    * @param max_i Upper i-coord of bounding box.
    * @param max_j Upper j-coord of bounding box.
    * @param max_k Upper k-coord of bounding box.
    * @return      Upon return the destination grid m contains the cropped grid.
    */
    template < typename grid_type >
    inline void crop(
      grid_type const & M
      , grid_type & m
      , size_t min_i
      , size_t min_j
      , size_t min_k
      , size_t max_i
      , size_t max_j
      , size_t max_k
      )
    {
      typedef typename grid_type::math_types          math_types;
      typedef typename math_types::vector3_type       vector3_type;
      typedef typename grid_type::index_iterator      index_iterator;

      vector3_type min_coord,max_coord;
      idx2coord(M, min_i, min_j, min_k, min_coord);
      idx2coord(M, max_i, max_j, max_k, max_coord);
      m.create( min_coord, max_coord, max_i-min_i+1, max_j-min_j+1, max_k-min_k+1 );

      for(size_t k=0; k<m.K(); ++k )
        for(size_t j=0; j<m.J(); ++j )
          for(size_t i=0; i<m.I(); ++i )
          {
            m(i,j,k) = M( i+min_i, j+min_j, k+min_k );
          }
    }
  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_CROP_H
#endif
