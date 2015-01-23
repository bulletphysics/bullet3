 #ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TETRAHEDRON_Z_SLICER_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TETRAHEDRON_Z_SLICER_H
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
    /**
    * Specialized Tetrahedron Slicer.
    * Cut a tetrahedron with a specified z-plane and returns the
    * sliced tetrahedron.
    *
    * This code was originally implemented by Andreas Bæentzen, IMM (jab@imm.dtu.dk).
    *
    */
    template<typename vector3_type_>
    class ZTetrahedronSlicer
    {
    public:

      typedef vector3_type_                      vector3_type;
      typedef typename vector3_type::value_type  real_type;

    protected:

      vector3_type  m_p[4];  ///< Nodes of tetrahedron.

    public:

      ZTetrahedronSlicer(
        vector3_type const & p0
        , vector3_type const & p1
        , vector3_type const & p2
        , vector3_type const & p3
        )
      {
        m_p[0] = p0;
        m_p[1] = p1;
        m_p[2] = p2;
        m_p[3] = p3;

        //--- Primitive bubble sort - ensures that tetrahedron vertices
        //--- are sorted according to z value.
        while(swap(0,1)||swap(1,2)||swap(2,3));
      };

    protected:

      /**
      * Swap the order of two nodes if their z-value allows it.
      *
      * @param i   A node index.
      * @param j   A node index.
      *
      * @return   true if swapped otherwise false.
      */
      bool swap(int i,int j)
      {
        if(m_p[j](2)< m_p[i](2))
        {
          vector3_type tmp = m_p[j];
          m_p[j] = m_p[i];
          m_p[i] = tmp;
          return true;
        }
        return false;
      };

      /**
      * Computes intersection point of edge going from node i towards node j
      *
      * @param z   The z value, indicating the z-plane aginst which the edge is tested.
      * @param i   The node index of the starting node (got lowest z-value).
      * @param j   The node index of the ending node (got highest z-value).
      * @param p   Upon return this argument holds the intersection point.
      */
      void intersect(real_type const &  z, int i, int j, vector3_type & p) const
      {
        p = m_p[i];
        vector3_type dir = m_p[j] - m_p[i];
        real_type s = (z-m_p[i](2))/(m_p[j](2)-m_p[i](2));
        p += s * dir;
      };

    public:

      /**
      * Compute Intersection Slice of Tetrahedron and Z-plane.
      *
      * @param z       The z-plane to slice against.
      * @param slice   Upon return holds the intersection points of the sliced tetrahedron.
      *
      * @return        The number of vertices in the sliced tetrahedron.
      */
      unsigned int intersect(real_type const & z, vector3_type slice[4]) const
      {
        if( z < m_p[0](2) || z > m_p[3](2) )
          return 0;
        if( z<m_p[1](2) )
        {
          intersect(z,0,3, slice[0]);
          intersect(z,0,1, slice[1]);
          intersect(z,0,2, slice[2]);
          return 3;
        }
        else if( z<m_p[2](2) )
        {
          intersect(z,0,3, slice[0]);
          intersect(z,1,3, slice[1]);
          intersect(z,1,2, slice[2]);
          intersect(z,0,2, slice[3]);
          return 4;
        }
        else
        {
          intersect(z,0,3, slice[0]);
          intersect(z,1,3, slice[1]);
          intersect(z,2,3, slice[2]);
          return 3;
        }
      }
      //     unsigned int intersect(real_type const & z, vector3_type slice[4]) const
      //     {
      //       unsigned int cnt=0;
      //       if(z > m_p[0](2))
      //       {
      //         if(z < m_p[3](2))
      //         {
      //           intersect(z,0,3,slice[cnt++]);
      //           if(z<m_p[1](2))
      //             intersect(z,0,1,slice[cnt++]);
      //           else
      //           {
      //             intersect(z,1,3,slice[cnt++]);
      //
      //             if(z<m_p[2](2))
      //               intersect(z,1,2,slice[cnt++]);
      //           }
      //           if(z<m_p[2](2))
      //             intersect(z,0,2, slice[cnt++]);
      //           else
      //             intersect(z,2,3,slice[cnt++]);
      //           return cnt;
      //         }
      //         else return 0;
      //       }
      //       else
      //         return 0;
      //     }

    };

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TETRAHEDRON_Z_SLICER_H
#endif
