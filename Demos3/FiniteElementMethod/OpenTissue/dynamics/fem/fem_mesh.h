#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_MESH_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_MESH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/t4mesh/t4mesh.h>
#include <OpenTissue/dynamics/fem/fem_node_traits.h>
#include <OpenTissue/dynamics/fem/fem_tetrahedron_traits.h>

namespace OpenTissue
{
  namespace fem
  {

    template <typename math_types>
    class Mesh 
      : public OpenTissue::t4mesh::T4Mesh<
                                      math_types
                                      , OpenTissue::fem::detail::NodeTraits<math_types>
                                      , OpenTissue::fem::detail::TetrahedronTraits<math_types> 
                                      >
    {
    public:

      typedef typename math_types::real_type           real_type;
      typedef typename math_types::vector3_type        vector3_type;
      typedef typename math_types::matrix3x3_type      matrix3x3_type;
    };

  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_MESH_H
#endif
