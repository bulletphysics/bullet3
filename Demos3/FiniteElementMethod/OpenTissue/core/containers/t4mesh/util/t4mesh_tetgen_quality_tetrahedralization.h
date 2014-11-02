#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_TETGEN_QUALITY_TETRAHEDRALIZATION_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_TETGEN_QUALITY_TETRAHEDRALIZATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/t4mesh/util/t4mesh_tetgen_mesh_lofter.h>

namespace OpenTissue
{
  namespace t4mesh
  {
    namespace tetgen
    {

      /**
      * Quality Tetrahedralization Routine.
      * - uses TetGen to create the resulting tetrahedal mesh
      *
      * @param polymesh          A poly mesh, which holds a closed two-manifold.
      * @param t4mesh            A generic t4mesh, which upon return holds the generated tetrahedal mesh.
      *
      * @return                  if succesfull then the return value is true otherwise it is false.
      */
      template<typename t4mesh_type, typename polymesh_type>
      inline bool quality_tetrahedralization(const polymesh_type& polymesh, t4mesh_type & t4mesh)
      {
        OpenTissue::t4mesh::mesh_lofter_settings config;

        config.m_intermediate_file = "tmp";
        config.m_quality_ratio = 2.0;
        config.m_maximum_volume = 0.0;
        config.m_quiet_output = true;

        return OpenTissue::t4mesh::mesh_lofter(t4mesh, polymesh, config);
      }

    } // namespace tetgen
  } // namespace t4mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_TETGEN_QUALITY_TETRAHEDRALIZATION_H
#endif
