#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_VOXEL_PLANE_CLIP_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_VOXEL_PLANE_CLIP_H
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
    * Clip a voxelgrid against a plane.
    * @param voxels Input voxel grid.
    * @param plane  Plane to clip against.
    * @param below  Upon return, contains grid of all voxels below plane.
    * @param above  Upon return, contains grid of all voxels above plane.
    */
    template < typename grid_type, typename plane_type >
    inline void voxel_plane_clip(
      grid_type const& voxels
      , plane_type const& plane
      , grid_type & below
      , grid_type & above
      )
    {
      typedef typename grid_type::value_type value_type;
      typedef typename grid_type::const_index_iterator const_index_iterator;

      const_index_iterator voxel;
      for ( voxel=voxels.begin(); voxel!=voxels.end(); ++voxel )
      {
        if ( plane.signed_distance( voxel.get_coord() ) >= 0 )
        {
          above( voxel.get_index() ) = voxels( voxel.get_index() );
          below( voxel.get_index() ) = value_type(0);
        }
        else
        {
          below( voxel.get_index() ) = voxels( voxel.get_index() );
          above( voxel.get_index() ) = value_type(0);
        }
      }
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_VOXEL_PLANE_CLIP_H
#endif
