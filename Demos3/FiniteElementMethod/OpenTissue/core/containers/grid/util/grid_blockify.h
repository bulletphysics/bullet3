#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_BLOCKIFY_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_BLOCKIFY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>


#include <iostream>
#include <cmath>
#include <algorithm>

namespace OpenTissue
{
  namespace grid
  {
    /**
    * Level Set Blockifier.
    * This function takes a level set grid and re-initializes it into
    * small cubic blocks (-1 inside and +1 outside).
    *
    * @param phi   The level set grid.
    */
    template<typename grid_type>
    inline void blockify(grid_type & phi)
    {
      typedef typename grid_type::value_type      value_type;
      typedef typename grid_type::index_iterator  iterator;

      value_type unused  = phi.unused();
      value_type inside  = value_type(-1.0);
      value_type outside = value_type( 1.0);

      size_t offset  = 7u;
      size_t spacing = 11u;
      size_t block   = offset + spacing;
      iterator   begin   = phi.begin();
      iterator   end     = phi.end();
      iterator   p;
      for(p=begin;p!=end;++p)
      {
        if(*p==unused)
          continue;

        bool inside_i = ((p.i() % block)>offset);
        bool inside_j = ((p.j() % block)>offset);
        bool inside_k = ((p.k() % block)>offset);
        if(inside_i && inside_j && inside_k)
          *p = inside;
        else
          *p = outside;
      }
    }

    /** Initialize a grid with a pattern of boxes of width offset.
    *
    * @param phi     The grid to initialize.
    * @param inside  Value to fill inside blocks
    * @param outside Value to fill outside blocks
    * @param offset  The offset from the borders.
    * @param spacing The width, height, depth and half the spacing of boxes.
    */
    template<typename grid_type,typename value_type>
    inline void blockify(grid_type & phi, value_type inside, value_type outside, size_t offset, size_t spacing)
    {
      typedef typename grid_type::value_type      internal_type;
      typedef typename grid_type::index_iterator  iterator;

      internal_type unused  = phi.unused();
      internal_type inside_ = (internal_type)( inside );
      internal_type outside_ = (internal_type)( outside );

      size_t block   = offset + spacing;
      iterator   begin   = phi.begin();
      iterator   end     = phi.end();
      iterator   p;
      for(p=begin;p!=end;++p)
      {
        if(*p==unused)
          continue;

        bool inside_i = ((p.i() % block)>offset);
        bool inside_j = ((p.j() % block)>offset);
        bool inside_k = ((p.k() % block)>offset);
        if(inside_i && inside_j && inside_k)
          *p = inside_;
        else
          *p = outside_;
      }
    }

    /** Initialize a slice of a grid with a pattern of boxes of width offset.
    *
    * @param phi     The grid to initialize.
    * @param inside  Value to fill inside blocks
    * @param outside Value to fill outside blocks
    * @param offset  The offset from the borders.
    * @param spacing The width, height, and half the spacing of boxes.
    * @param slice   The slice in z-depth that needs to be filled
    */
    template<typename grid_type,typename value_type>
    inline void blockify(grid_type & phi, value_type inside, value_type outside, size_t offset, size_t spacing, size_t slice)
    {
      typedef typename grid_type::value_type      internal_type;
      typedef typename grid_type::index_iterator  iterator;

      internal_type unused  = phi.unused();
      internal_type inside_ = (internal_type)( inside );
      internal_type outside_ = (internal_type)( outside );

      size_t block   = offset + spacing;
      iterator   begin   = phi.begin();
      iterator   end     = phi.end();
      iterator   p;
      for(p=begin;p!=end;++p)
      {
        if (p.k()!=slice)
          continue;
        if(*p==unused)
          continue;
        bool inside_i = ((p.i() % block)>offset);
        bool inside_j = ((p.j() % block)>offset);
        if(inside_i && inside_j)
          *p = inside_;
        else
          *p = outside_;
      }
    }


  } // namespace grid
} // namespace OpenTissue

//  OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_BLOCKIFY_H
#endif
