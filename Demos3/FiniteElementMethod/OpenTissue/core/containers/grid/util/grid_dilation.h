#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_DILATION_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_DILATION_H
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
    * Dilate level set.
    *
    * @param phi      Input level set.
    * @param radius   Radius of spherical structural element.
    * @param dt       Time-step to use in update.
    * @param psi      Output levelset. Note if input is a signed distance map then output may not be a signed distance map, thus you may need to redistance output levelset.
    */
    template<
      typename grid_type_in
      , typename real_type
      , typename grid_type_out
    >
    inline void dilation(
    grid_type_in const & phi
    , real_type const & radius
    , real_type const & dt
    , grid_type_out & psi
    )
    {
      typedef typename grid_type_in::value_type      input_type;
      typedef typename grid_type_out::value_type     output_type;
      typedef typename grid_type_in::const_index_iterator  input_iterator;
      
      assert(radius>0 || !"dilation(): radius must be positive");
      assert(dt>0     || !"dilation(): time-step must be positive");
      
      input_iterator input_begin = phi.begin();
      input_iterator input_end   = phi.end();
      input_iterator input;
      input_type unused = phi.unused();
      for(input=input_begin;input!=input_end;++input)
        if(*input!=unused)
          psi(input.get_index()) = *input - dt*radius;
    }

  } // namespace grid

} // namespace OpenTissue

//  OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_DILATION_H
#endif
