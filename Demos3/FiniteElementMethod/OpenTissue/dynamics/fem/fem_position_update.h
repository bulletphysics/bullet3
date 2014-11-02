#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_POSITION_UPDATE_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_POSITION_UPDATE_H
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
  namespace fem
  {
    namespace detail
    {
      /**
      * Position Update.
      *
      * Note: Velocities must have been computed prior to invoking this function.
      * @param mesh
      * @param dt
      */
      template < typename fem_mesh, typename real_type >
      inline void position_update(fem_mesh & mesh, real_type const & dt)
      {
        for (int n=0;n<mesh.m_nodes.size();n++)
        {
          if(mesh.m_nodes[n].m_fixed)
            continue;
          mesh.m_nodes[n].m_coord += dt * mesh.m_nodes[n].m_velocity;
        }
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_POSITION_UPDATE_H
#endif
