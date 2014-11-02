#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_T4MESH_DEFAULT_POINT_CONTAINER_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_T4MESH_DEFAULT_POINT_CONTAINER_H
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

    /**
    * Default Point Container.
    * This utility class can be used to make the coordniates of the nodes in
    * a tetrahedra mesh appear as a point container, i.e. as though the coordinates
    * are stored as 
    *
    *   std::vector<vector3_type> coordinates;
    *
    * and accesses as
    *
    *   coordinates[node->idx()]
    *
    * instead of
    *
    *   node->m_coord
    *
    * Many algoritms in OpenTissue have been implemented in such a way that they
    * do not rely on nodes to have a m_coord member. Instead coordinates are passed
    * as point containers. This utility make it convenient to use these algorithms
    * on nodes where coordinates are stored in m_coord member.
    *
    */
    template<typename M>
    struct default_point_container
    {
      typedef          M                          mesh_type;
      typedef typename mesh_type::math_types      math_types;
      typedef typename math_types::vector3_type   value_type;

      mesh_type * m_mesh;

      default_point_container(mesh_type * mesh) : m_mesh(mesh) {}

      value_type & operator[] (unsigned int const & idx)
      {
        return m_mesh->node(idx)->m_coord;
      }

      value_type const & operator[] (unsigned int const & idx)const
      {
        return m_mesh->node(idx)->m_coord;
      }

      void clear(){}
      size_t size() const {return m_mesh->size_nodes();}
      void resize(size_t){}
    };


  } // namespace t4mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_T4MESH_DEFAULT_POINT_CONTAINER_H
#endif
