#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_DEFAULT_TRAITS_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_DEFAULT_TRAITS_H
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
  namespace t4mesh
  {

    template<typename math_types>
    class DefaultNodeTraits  
    {
    public:

      typedef typename math_types::vector3_type  vector3_type;
      typedef typename math_types::real_type     real_type;

      vector3_type  m_coord;    ///< Default Coordinate of tetramesh node.
    };

    class DefaultTetrahedronTraits { };

    class DefaultT4EdgeTraits  {  };

    class DefaultT4FaceTraits  {  };

  } // namespace t4mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_DEFAULT_TRAITS_H
#endif
