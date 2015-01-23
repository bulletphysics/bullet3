#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_T4EDGES_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_T4EDGES_H
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

    /**
    * t4mesh Edge.
    * An edge is uniquely defined by the indices of its two end nodes.
    * The order of indices do not matter.
    */
    template<typename M, typename E>
    class T4Edge : public E
    {
    public:

      typedef M                               mesh_type;
      typedef E                               edge_traits;
      typedef T4Edge<M,E>                     edge_type;
      typedef typename mesh_type::index_type  index_type;

    protected:

      index_type m_idxA;      ///< Index to first node.
      index_type m_idxB;      ///< Index to second node.

    public:

      T4Edge()
        : m_idxA(-1)
        , m_idxB(-1)
      {}

      T4Edge(index_type const & indexA, index_type const & indexB)
        : m_idxA(indexA)
        , m_idxB(indexB)
      {}

    public:

      index_type const & idxA() const { return m_idxA; }
      index_type const & idxB() const { return m_idxB; }

      bool operator==(edge_type const & edge) const
      {
        if(m_idxA==edge.idxA() && m_idxB==edge.idxB())
          return true;
        if(m_idxB==edge.idxA() && m_idxA==edge.idxB())
          return true;
        return false;
      }

      bool operator!=(edge_type const & edge)const{  return !((*this)==edge); }

    };

    /**
    * Tetrahedra Mesh Edges.
    * Edges are not represented explicitly in a t4mesh, only nodes and tetrahedra
    * are representated. Thus this class extracts all unique edges from a t4mesh, by
    * traversing it and generating explicit edges.
    *
    * Note that if subsequent changes are made to the t4mesh, the edge changes are
    * not reflected by the edges stored in this class. Meaning that a new edge-queury
    * must be initiated everytime the t4mesh changes its topology.
    *
    */
    template<class M, typename E >
    class T4Edges
    {
    public:

      typedef M                                        mesh_type;
      typedef E                                        edge_traits;
      typedef T4Edge<M,E>                              edge_type;
      typedef typename mesh_type::index_type           index_type;
      typedef std::list<edge_type>                     edge_container;
      typedef typename edge_container::iterator        edge_iterator;
      typedef typename edge_container::const_iterator  const_edge_iterator;

    protected:

      typedef enum{white,grey,black}                       color_type;
      typedef std::vector<color_type>                      color_container;
      typedef typename mesh_type::node_type                node_type;
      typedef typename node_type::tetrahedron_circulator   tetrahedron_type;
      typedef std::list<node_type*>                        work_queue;

    protected:

      edge_container m_edges;  

    public:

      edge_iterator       begin()       { return m_edges.begin(); }
      edge_iterator       end()         { return m_edges.end();   }
      const_edge_iterator begin() const { return m_edges.begin(); }
      const_edge_iterator end()   const { return m_edges.end();   }


    protected:

      /**
      * Internally used functor, needed for visiting neighboring
      * nodes and extracting un-seen edges.
      */
      struct VisitT4Edge
      {
        void visit(
          index_type & idxA
          , index_type & idxB
          , work_queue & work
          , color_container & colors
          , edge_container & edges
          , mesh_type & mesh
          )
        {
          if(idxA==idxB)//--- self-loop, ignore it
            return;

          //std::cout << "\tvisting:" <<idxB << " : color = " << colors[idxB] << std::endl;

          if(colors[idxB]==white)
          {
            colors[idxB] = grey;
            work.push_back(&(*(mesh.node(idxB))));
          }
          if(colors[idxB]!=black)
          {
            edges.push_back(edge_type(idxA,idxB));
          }
        }
      };

    public:

      T4Edges(){}

      /**
      * Specialized Constructor.
      * This constructor traverses the specified mesh and extracts all edges.
      */
      T4Edges(mesh_type & mesh)
      {
        index_type idxA,idxB;

        color_container colors(mesh.size_nodes());
        std::fill(colors.begin(),colors.end(),white);
        node_type * node = &(*(mesh.node(0)));
        colors[0]=grey;
        work_queue work;
        work.push_back(node);
        while(!work.empty())
        {
          node = work.back();
          work.pop_back();
          idxA = node->idx();

          assert(colors[idxA] == grey || !"T4Edges(mesh): Encounted non-greay node");

          //std::cout << idxA << " : color = " << colors[idxA] << std::endl;

          std::list<index_type> neighbors;
          for(tetrahedron_type T = node->begin();T!=node->end();++T)
          {
            idxB = T->i()->idx();
            if(idxB != idxA)
              neighbors.push_back(idxB);
            idxB = T->j()->idx();
            if(idxB != idxA)
              neighbors.push_back(idxB);
            idxB = T->k()->idx();
            if(idxB != idxA)
              neighbors.push_back(idxB);
            idxB = T->m()->idx();
            if(idxB != idxA)
              neighbors.push_back(idxB);
          }
          neighbors.sort();
          neighbors.unique();

          for(typename std::list<index_type>::iterator n = neighbors.begin();n!=neighbors.end();++n)
            VisitT4Edge().visit(idxA, *n ,work,colors,m_edges,mesh);

          //for(tetrahedron_type T = node->begin();T!=node->end();++T)
          //{
          //  idxB = T->i()->idx();
          //  VisitT4Edge().visit(idxA,idxB,work,colors,m_edges,mesh);
          //  idxB = T->j()->idx();
          //  VisitT4Edge().visit(idxA,idxB,work,colors,m_edges,mesh);
          //  idxB = T->k()->idx();
          //  VisitT4Edge().visit(idxA,idxB,work,colors,m_edges,mesh);
          //  idxB = T->m()->idx();
          //  VisitT4Edge().visit(idxA,idxB,work,colors,m_edges,mesh);
          //}
          colors[idxA] = black;
        }
      };

    };

  } // namespace t4mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_T4EDGES_H
#endif
