#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_IGNORE_REGION_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_IGNORE_REGION_H
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
    * Tag used to specify inside region of a level set (phi<0)
    */
    struct inside_region_tag {};

    /**
    * Tag used to specify outside region of a level set (phi>0)
    */
    struct outside_region_tag {};


    /**
    * Mask out inside region.
    * This function masks out the inside region of level set
    * function by assigning the unused value to all phi<0.
    *
    * @param phi   The level set grid.
    */
    template<typename grid_type>
    inline void ignore_region(grid_type & phi, inside_region_tag const & /*region*/)
    {
      typedef typename grid_type::value_type      value_type;
      typedef typename grid_type::index_iterator  iterator;

      value_type unused  = phi.unused();
      iterator   end     = phi.end();
      iterator   p       = phi.begin();
      for(;p!=end;++p)
        if(*p < 0 )
          *p = unused;
    }

    /**
    * Mask out outside region.
    * This function masks out the inside region of level set
    * function by assigning the unused value to all phi>0.
    *
    * @param phi   The level set grid.
    */
    template<typename grid_type>
    inline void ignore_region(grid_type & phi, outside_region_tag const & /*region*/)
    {
      typedef typename grid_type::value_type      value_type;
      typedef typename grid_type::index_iterator  iterator;

      value_type unused  = phi.unused();
      iterator   end     = phi.end();
      iterator   p       = phi.begin();
      for(;p!=end;++p)
        if(*p > 0 )
          *p = unused;
    }

  } // namespace grid
} // namespace OpenTissue

//  OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_IGNORE_REGION_H
#endif
