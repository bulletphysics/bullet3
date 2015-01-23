#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TETRAHEDRON_SLICER_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TETRAHEDRON_SLICER_H
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
  namespace geometry
  {


    template<typename vector3_type_>
    class TetrahedronSlicer
    {
    public:

      typedef vector3_type_                      vector3_type;
      typedef typename vector3_type::value_type  real_type;


      vector3_type m_nodes[4];   ///< Nodes of tetrahedron
      int m_edges[6][2];         ///< Edge topology of tetrahedron
      ///< m_edges[i][0]  index of the first node of the i'th edge
      ///< m_edges[i][1]  index of the second node of the i'th edge

      vector3_type m_intersections[4];    ///< Intersection points of slice and tetrahedron.
      vector3_type m_triangle1[3];        ///< Vertices of first triangle in slice.
      vector3_type m_triangle2[3];        ///< Vertices of second trinagle in slice.

    public:

      TetrahedronSlicer(
        vector3_type const & p0
        , vector3_type const & p1
        , vector3_type const & p2
        , vector3_type const & p3
        )
      {
        m_nodes[0] = p0;
        m_nodes[1] = p1;
        m_nodes[2] = p2;
        m_nodes[3] = p3;

        //
        //  Node indices and edge labels:
        //
        //                      3
        //                     +
        //                    / \
        //                   /   \
        //                  /  .  \
        //                 /       \
        //                /         \
        //               /     .     \
        //              /             \
        //             /               \
        //            /       .         \
        //           C        F          E
        //          /                     \
        //         /         .             \
        //        /                         \
        //       /           .               \
        //      /          2+                 \
        //     /       .         .             \
        //    /    . B                D.        \
        //   / .                              .  \
        //  +------------------A------------------+
        //0                                         1
        //
        //
        m_edges[ 0 ][ 0 ] = 0; // A
        m_edges[ 0 ][ 1 ] = 1;
        m_edges[ 1 ][ 0 ] = 0; // B
        m_edges[ 1 ][ 1 ] = 2;
        m_edges[ 2 ][ 0 ] = 0; // C
        m_edges[ 2 ][ 1 ] = 3;
        m_edges[ 3 ][ 0 ] = 1; // D
        m_edges[ 3 ][ 1 ] = 2;
        m_edges[ 4 ][ 0 ] = 1; // E
        m_edges[ 4 ][ 1 ] = 3;
        m_edges[ 5 ][ 0 ] = 2; // F
        m_edges[ 5 ][ 1 ] = 3;

      };

    protected:

      template<typename plane_type>
      void compute_slice(plane_type const & plane)
      {
        //--- Loop over all edges and find those that cross the plane
        int count = 0;
        for(int i=0;i<6;++i)
        {
          vector3_type & a = m_nodes[m_edges[i][0]];
          vector3_type & b = m_nodes[m_edges[i][1]];
          real_type dst_a = plane.signed_distance(a);
          real_type dst_b = plane.signed_distance(b);
          if ( (dst_a*dst_b<0) || (dst_a==0 && dst_b>0) || (dst_b==0 && dst_a>0))
          {
            //--- Compute coordinates of the plane intersections
            vector3_type & a = m_nodes[m_edges[ i ][0]];
            vector3_type & b = m_nodes[m_edges[ i ][1]];
            m_intersections[count++] = plane.get_intersection(a,b);
          }
        }
        assert(count==3 || count==4); //--no other possibilities?

        //--- Order plane intersections to a a CCW polygon (as seen from positive side of plane)
        vector3_type & n = plane.n();
        vector3_type e10 = m_intersections[1] - m_intersections[0];
        vector3_type e21 = m_intersections[2] - m_intersections[1];
        if(n*(e10%e20)<0)
        {
          m_triangle1[0] = m_intersections[0];
          m_triangle1[1] = m_intersections[2];
          m_triangle1[2] = m_intersections[1];
        }
        else
        {
          m_triangle1[0] = m_intersections[0];
          m_triangle1[1] = m_intersections[1];
          m_triangle1[2] = m_intersections[2];
        }
        if ( count == 4 )
        {
          for(int i=0;i<3;++i)
          {
            int next = (i+1)%3;
            int prev = i;
            vector3_type e = m_triangle1[next] - m_triangle1[prev];
            vector3_type h = m_intersections[4] - m_triangle1[prev];
            if(n*(h%e)>0)
            {
              m_triangle2[0] = m_triangle1[prev];
              m_triangle2[1] = m_intersections[4];
              m_triangle2[2] = m_triangle1[next];
              break;
            }
          }
        }
      };

    };

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TETRAHEDRON_SLICER_H
#endif
