#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_T4MESH_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_T4MESH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/t4mesh/t4mesh_t4node.h>
#include <OpenTissue/core/containers/t4mesh/t4mesh_t4tetrahedron.h>
#include <OpenTissue/core/containers/t4mesh/t4mesh_core_access.h>
#include <OpenTissue/core/math/math_constants.h>

#include <vector>
#include <list>
#include <cassert>

namespace OpenTissue
{
  namespace t4mesh
  {
    namespace detail
    {

      /**
      * Basic (Simple) Tetrahedra Mesh.
      *
      * This tetrahedra mesh data structure is designed specially for
      * two purposes: It should maintain a valid topology of the mesh
      * at all times, that is the connectivity of nodes and tetrahedra
      * are always valid.
      *
      * The other purpose is to make sure that the global indexing of
      * nodes (0..N-1) and tetrahedra (0..T-1) always is a compact range
      * starting from zero to the maximum number minus one.
      *
      * Obviously removing entities (nodes or tetrahedra) alters the global
      * index ranges, thus end users can not trust previously stored indices
      * of entities in their own apps.
      *
      * The mesh takes three template arguments. The first specifies the
      * math_types used in the mesh. The following two arguments are node
      * traits and tetrahedron traits respectively.
      */
      template<
        typename M
        , typename N
        , typename T
      >
      class T4Mesh
      {
      public:

        typedef M                        math_types;
        typedef N                        node_traits;
        typedef T                        tetrahedron_traits;
        typedef T4Mesh<M,N,T>            mesh_type;
        typedef T4Node<mesh_type>        node_type;
        typedef T4Tetrahedron<mesh_type> tetrahedron_type;
        typedef size_t                   index_type;

        /**
        * Undefined index value. 
        *
        * @return   An index value that means that the index value is undefined. The
        *          largest possible value of the index type is used for this purpose.
        */
        static index_type const & undefined() 
        {
          static index_type value = FLT_MAX;//math::detail::highest<index_type>();
          return value;
        }

      protected:

        typedef std::vector< node_type >        node_container;
        typedef std::vector< tetrahedron_type > tetrahedra_container;

	  public:

        node_container           m_nodes;            ///< Internal node storage.
        tetrahedra_container     m_tetrahedra;       ///< Internal tetrahedra storage.

      public:

        
      public:

        T4Mesh() 
          : m_nodes()
          , m_tetrahedra() 
        {}


        T4Mesh( T4Mesh const & cpy) 
        {
          *this = cpy;
        }



      public:

        void clear()
        {
          m_nodes.clear();
          m_tetrahedra.clear();
        }


      public:

        /**
        * Add New Node.
        * New node will have index value equal to number of nodes prior to insertion.
        *
        * @return            A iterator to the new node
        */
        void insert()
        {
          m_nodes.push_back( node_type() );
          node_type & nd = m_nodes.back();
          t4mesh_core_access::set_index( nd, this->m_nodes.size()-1 );
          t4mesh_core_access::set_owner( nd, this );
        }

        /**
        * Overloaded insert method for tetrahedron, support index-based insertion.
        *
        * This is a bit slower than using the iterator-based insertion method directly.
        * But it makes it easier to code....
        *
        */
        void	insert(
          index_type i,
          index_type j,
          index_type k,
          index_type m
          )
        {

//          assert(find(i,j,k,m)==tetrahedron_end() || !"T4Mesh::insert(): Tetrahedron already exists in mesh");

          m_tetrahedra.push_back( tetrahedron_type() );
          tetrahedron_type & t = m_tetrahedra.back();

          t4mesh_core_access::set_index( t, m_tetrahedra.size() - 1 );
          t4mesh_core_access::set_owner( t, this );
          t4mesh_core_access::set_node0( t, i );
          t4mesh_core_access::set_node1( t, j );
          t4mesh_core_access::set_node2( t, k );
          t4mesh_core_access::set_node3( t, m );

          t4mesh_core_access::tetrahedra_push_back( m_nodes[i], t.idx() );
          t4mesh_core_access::tetrahedra_push_back( m_nodes[j], t.idx() );
          t4mesh_core_access::tetrahedra_push_back( m_nodes[k], t.idx() );
          t4mesh_core_access::tetrahedra_push_back( m_nodes[m], t.idx() );

          //return tetrahedron_iterator(m_tetrahedra, t.idx());
        }


        /**
        * Erase Tetrahedron at specified Position.
        *
        * @param where
        *
        * @return
        */
        index_type	erase(index_type& where)
        {
			index_type last = m_tetrahedra.size()-1;
			if (where != last)
			{
				this->swap(where,last);
			}

          this->unlink(last);
          //--- This might be a bit stupid, it would
          //--- proberly be better to keep track of last unused
          //--- entry and only resize vector when there is no
          //--- more space
          m_tetrahedra.pop_back();            
          return where;
        }

      protected:

      protected:


      };

    } // namespace detail
  } // namespace t4mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_T4MESH_H
#endif
