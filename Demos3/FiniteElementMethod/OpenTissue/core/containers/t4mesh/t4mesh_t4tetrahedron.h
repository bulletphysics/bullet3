#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_T4TETRAHEDRON_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_T4TETRAHEDRON_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>

namespace OpenTissue
{
  namespace t4mesh
  {

    template< typename mesh_type_>
    class T4Tetrahedron : public mesh_type_::tetrahedron_traits
    {
    public:
      typedef          mesh_type_                        mesh_type;
      typedef typename mesh_type::node_type              node_type;
      typedef typename mesh_type::tetrahedron_type       tetrahedron_type;
      typedef typename mesh_type::index_type             index_type;

      index_type                m_idx;      ///< Global index of tetrahedron
      mesh_type               * m_owner;    ///< Pointer to mesh which the node belongs to.
      index_type m_nodes[4];  ///< Global index of node i,j,k and m

    private:

      friend class t4mesh_core_access;

      void set_index(index_type idx)    { m_idx = idx;      }
      void set_owner(mesh_type * owner) { m_owner = owner;  }
      void set_node0(index_type idx)    { m_nodes[0] = idx; }
      void set_node1(index_type idx)    { m_nodes[1] = idx; }
      void set_node2(index_type idx)    { m_nodes[2] = idx; }
      void set_node3(index_type idx)    { m_nodes[3] = idx; }

    public:
#ifdef TODO_ERWIN
      T4Tetrahedron() 
        : m_idx( mesh_type::undefined() )
        , m_owner(0) 
      { 
        this->m_nodes.assign( mesh_type::undefined() ); 
      }
#endif

    public:

		node_type*	node(index_type idx)
		{
			return &m_owner->m_nodes[m_nodes[idx]];
		}

		const node_type*	node(index_type idx) const
		{
			return &m_owner->m_nodes[m_nodes[idx]];
		}

		node_type*	i()
		{
			return node(0);
		}
		node_type*	j()
		{
			return node(1);
		}
		node_type*	k()
		{
			return node(2);
		}
		node_type*	m()
		{
			return node(3);
		}


      index_type            idx()   const { return m_idx; }

      index_type            node_idx(index_type const & local_idx) const 
      { 
        assert(0<=local_idx);
        assert(local_idx<=3);

        return m_nodes[local_idx]; 
      }

      
      mesh_type       * owner()       { return m_owner; }
      mesh_type const * owner() const { return m_owner; }



    };

  } // namespace t4mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_T4TETRAHEDRON_H
#endif
