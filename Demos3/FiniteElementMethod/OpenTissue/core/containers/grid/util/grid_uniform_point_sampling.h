#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_UNIFORM_POINT_SAMPLING_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_UNIFORM_POINT_SAMPLING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/functions.h> //--- needed for gradient_func and value_func

namespace OpenTissue
{
  namespace grid
  {

    /**
    * Uniform Point Sampling of Grid.
    *
    * @param phi              The (signed distance grid) level set grid.
    * @param points           Upon return this container containts the random points.
    * @param sub_Sample       Optimal argument, allows one to control the density of points.
    */
    template<typename grid_type,typename vector3_container>
    inline void uniform_point_sampling(
      grid_type const & phi
      , vector3_container & points
      , size_t sub_sample = 2u
      )
    {
      typedef typename vector3_container::value_type    vector3_type;
      typedef typename vector3_type::value_type         real_type;

      points.clear();

      real_type min_x = phi.min_coord(0);
      real_type min_y = phi.min_coord(1);
      real_type min_z = phi.min_coord(2);

      real_type max_x = phi.max_coord(0);
      real_type max_y = phi.max_coord(1);
      real_type max_z = phi.max_coord(2);

      real_type dx = phi.dx()*sub_sample;
      real_type dy = phi.dy()*sub_sample;
      real_type dz = phi.dz()*sub_sample;

      for ( real_type x = (min_x+dx);  x < (max_x-dx);  x += dx )
        for ( real_type y = (min_y+dy);  y < (max_y-dy);  y += dy )
          for ( real_type z = (min_z+dz);  z < (max_z-dz);  z += dz )
          {
            points.push_back( vector3_type( x, y, z ) );
          }
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_UNIFORM_POINT_SAMPLING_H
#endif
