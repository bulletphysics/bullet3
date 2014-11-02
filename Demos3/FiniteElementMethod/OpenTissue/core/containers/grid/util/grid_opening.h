#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_OPENING_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_OPENING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_dilation.h>
#include <OpenTissue/core/containers/grid/util/grid_erosion.h>

namespace OpenTissue
{
  namespace grid
  {

    /**
    * Opening Operation.
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
    inline void opening(
    grid_type_in const & phi
    , real_type const & radius
    , real_type const & dt
    , grid_type_out & psi
    )
    {
      dilation(phi,radius,dt,psi);
      erosion(psi,radius,dt,psi);
    }

  } // namespace grid
} // namespace OpenTissue

//  OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_OPENING_H
#endif
