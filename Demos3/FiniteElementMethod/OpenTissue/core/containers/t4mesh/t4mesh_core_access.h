#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_T4MESH_CORE_ACCESS_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_T4MESH_CORE_ACCESS_H
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
    class t4mesh_core_access
    {
    public:

      template< typename feature_type, typename index_type>
      static void set_index(feature_type & f, index_type idx)
      {
        f.set_index(idx);  
      }

      template< typename feature_type, typename mesh_type>
      static void set_owner(feature_type & f, mesh_type * owner)
      {
        f.set_owner(owner);
      }

      template< typename tetrahedron_type, typename index_type>
      static void set_node0(tetrahedron_type & tetrahedron, index_type idx)      
      {
        tetrahedron.set_node0(idx);
      }

      template< typename tetrahedron_type, typename index_type>
      static void set_node1(tetrahedron_type & tetrahedron, index_type idx)
      {
        tetrahedron.set_node1(idx);
      }
      
      template< typename tetrahedron_type, typename index_type>
      static void set_node2(tetrahedron_type & tetrahedron, index_type idx)
      {
        tetrahedron.set_node2(idx);
      }
      
      template< typename tetrahedron_type, typename index_type>
      static void set_node3(tetrahedron_type & tetrahedron, index_type idx)
      {
        tetrahedron.set_node3(idx);
      }

      template< typename node_type, typename index_type>
      static void tetrahedra_push_back(node_type & node, index_type idx) 
      { 
        node.tetrahedra_push_back(idx); 
      }

      template< typename node_type, typename index_type>
      static void tetrahedra_remove(node_type & node, index_type idx) 
      { 
        node.tetrahedra_remove(idx); 
      }

    };

  } // namespace trimesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_T4MESH_CORE_ACCESS_H
#endif
