#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_T4NODE_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_T4NODE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <list>
#include <cassert>

namespace OpenTissue
{
  namespace t4mesh
  {

    template< typename mesh_type_>
    class T4Node : public mesh_type_::node_traits
    {
    public:

      typedef          mesh_type_                            mesh_type;
      typedef typename mesh_type::node_type                  node_type;
      typedef typename mesh_type::tetrahedron_type           tetrahedron_type;
      typedef typename mesh_type::index_type                 index_type;
      typedef          std::list<index_type>                 tetrahedra_index_container;
      typedef typename tetrahedra_index_container::iterator  tetrahedra_idx_iterator;

    protected:

      index_type                   m_idx;              ///< Global index of node
      mesh_type                  * m_owner;            ///< Pointer to mesh which the node belongs to.
      tetrahedra_index_container   m_tetrahedra;       ///< Indices of tetrahedra this node is part of.

    private:

      friend class t4mesh_core_access;

      void set_index(index_type idx)            { m_idx = idx;                 }
      void set_owner(mesh_type * owner)       { m_owner = owner;             }
      void tetrahedra_push_back(index_type idx) { m_tetrahedra.push_back(idx); }
      void tetrahedra_remove(index_type idx)    { m_tetrahedra.remove(idx);    }

    public:

      class tetrahedron_circulator
      {
      private:

        node_type               * m_node;    ///< A pointer to the node
        tetrahedra_idx_iterator   m_it;      ///< An ``local'' iterator to the tetrahedron index, indicating current tetrahedron of this iterator.

      public:

        tetrahedron_circulator() 
          : m_node( 0 )
          , m_it( 0 ) 
        {}

        tetrahedron_circulator( node_type * node, tetrahedra_idx_iterator pos) 
          : m_node( node )
          , m_it( pos )
        {
          assert(m_node || !"tetrahedron_circulator() : node was null");
          assert(m_node->m_owner || !"tetrahedron_circulator(..) : owner was null");
        }

        bool operator== ( tetrahedron_circulator const & other ) const   
        {   
          return ( m_node == other.m_node && m_it == other.m_it );  
        }

        bool operator!= ( tetrahedron_circulator const & other ) const   
        {
          return !( *this == other);   
        }

        tetrahedron_type & operator*()
        {
          assert(m_node || !"tetrahedron_circulator::* : node was null");
          assert(m_node->m_owner || !"tetrahedron_circulator::* : owner was null");
          return *(m_node->m_owner->tetrahedron(*m_it));
        }

        tetrahedron_type * operator->()
        {
          assert(m_node || !"tetrahedron_circulator::-> : node was null");
          assert(m_node->m_owner || !"tetrahedron_circulator::-> : owner was null");
          return &(*(m_node->m_owner->tetrahedron(*m_it)));
        }

        tetrahedron_circulator & operator++()
        {
          assert(m_node || !"tetrahedron_circulator::++ : node was null");
          assert(m_node->m_owner || !"tetrahedron_circulator::++ : owner was null");
          ++m_it; 
          return *this;
        }

      };

      tetrahedron_circulator begin() { return tetrahedron_circulator( this, m_tetrahedra.begin() ); }
      tetrahedron_circulator end()   { return tetrahedron_circulator( this, m_tetrahedra.end() );   }

      /**
      * Get Number of Tetrahedra. 
      *
      * @return       The number of tetrahedra that this node is part of. If
      *               zero it means the node is an isolated node.
      */
      size_t size_tetrahedra() const { return m_tetrahedra.size(); }

      bool isolated() const { return m_tetrahedra.empty(); }

    public:

      T4Node() 
        : m_idx( mesh_type::undefined() )
        , m_owner(0)
        , m_tetrahedra() 
      {}

    public:

      index_type        idx()   const { return m_idx;   }
      mesh_type       * owner()       { return m_owner; }
      mesh_type const * owner() const { return m_owner; }

    };

  } // namespace t4mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_T4NODE_H
#endif
