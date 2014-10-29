#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TETRAHEDRON_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TETRAHEDRON_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_volume_shape.h>
#include <OpenTissue/core/geometry/geometry_compute_tetrahedron_aabb.h>

#include <OpenTissue/utility/utility_class_id.h>
#include <OpenTissue/core/math/math_constants.h>
#include <OpenTissue/core/geometry/geometry_barycentric.h>

#include <cassert>
#include <cmath>
#include <float.h>


namespace OpenTissue
{

  namespace geometry
  {

    template< typename math_types_ >
    class Tetrahedron
      : public VolumeShape< math_types_ >
      , public OpenTissue::utility::ClassID< OpenTissue::geometry::Tetrahedron<math_types_> >
    {
    public:

      typedef          math_types_                   math_types;
      typedef typename math_types::value_traits      value_traits;
      typedef typename math_types::real_type         real_type;
      typedef typename math_types::vector3_type      vector3_type;
      typedef typename math_types::matrix3x3_type    matrix3x3_type;
      typedef typename math_types::quaternion_type   quaternion_type;

    protected:

      vector3_type m_p0;     ///< Tetrahedron vertex number 1
      vector3_type m_p1;     ///< Tetrahedron vertex number 2
      vector3_type m_p2;     ///< Tetrahedron vertex number 3
      vector3_type m_p3;     ///< Tetrahedron vertex number 4

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::Tetrahedron<math_types_> >::class_id(); }

      Tetrahedron()
      {
        m_p0.clear();
        m_p1.clear();
        m_p2.clear();
        m_p3.clear();
      }

      virtual ~Tetrahedron() {}

      Tetrahedron(Tetrahedron const & tetrahedron_)  {    set(tetrahedron_);  }

      Tetrahedron( vector3_type const & p0, vector3_type const & p1, vector3_type const & p2, vector3_type const & p3)
      {
        set(p0,p1,p2,p3);
      }

      Tetrahedron const & operator=(Tetrahedron const & tetrahedron_)
      {
        set(tetrahedron_);
        return *this;
      }

      void set(Tetrahedron const & t)
      {
        m_p0 = t.m_p0;
        m_p1 = t.m_p1;
        m_p2 = t.m_p2;
        m_p3 = t.m_p3;
      }

      void set(vector3_type const & p0, vector3_type const & p1, vector3_type const & p2, vector3_type const & p3)
      {
        m_p0 = p0;
        m_p1 = p1;
        m_p2 = p2;
        m_p3 = p3;
      }

      real_type area() const
      {
        vector3_type u,v,uXv;
        real_type A = value_traits::zero();

        u = m_p1-m_p0;
        v = m_p2-m_p0;
        uXv = (u % v);
        A += length(uXv);

        u = m_p0-m_p3;
        v = m_p1-m_p3;
        uXv = (u % v);
        A += length(uXv);

        u = m_p1-m_p3;
        v = m_p2-m_p3;
        uXv = (u % v);
        A += length(uXv);

        u = m_p2-m_p3;
        v = m_p0-m_p3;
        uXv = (u % v);
        A += length(uXv);

        return A/value_traits::two();
      }

      void compute_surface_points(std::vector<vector3_type> & points) const
      {
        points.push_back(m_p0);
        points.push_back(m_p1);
        points.push_back(m_p2);
        points.push_back(m_p3);
      }

      vector3_type       & p0()        {    return m_p0;  };  
      vector3_type       & p1()        {    return m_p1;  };  
      vector3_type       & p2()        {    return m_p2;  };  
      vector3_type       & p3()        {    return m_p3;  };  
      vector3_type const & p0() const  {    return m_p0;  };
      vector3_type const & p1() const  {    return m_p1;  };
      vector3_type const & p2() const  {    return m_p2;  };
      vector3_type const & p3() const  {    return m_p3;  };

      vector3_type center() const  {    return vector3_type( (m_p0+m_p1+m_p2+m_p3)/4. );  };

      /**
      * Compute Barycentric Coordinates.
      *
      * @param p    The point for which the barycentric coordinates should be computed.
      * @param w1    Upon return this parameter contains the value of the first barycentric coordinate.
      * @param w2    Upon return this parameter contains the value of the second barycentric coordinate.
      * @param w3    Upon return this parameter contains the value of the third barycentric coordinate.
      * @param w4    Upon return this parameter contains the value of the fourth barycentric coordinate.
      */
      void barycentric(vector3_type const & p,real_type & w1,real_type & w2,real_type & w3,real_type & w4) const
      {
        OpenTissue::geometry::barycentric_algebraic(m_p0,m_p1,m_p2,m_p3,p,w1,w2,w3,w4);
      }

      real_type diameter() const
      {
        using std::max;
        using std::sqrt;

        vector3_type x30 = m_p3 - m_p0;
        vector3_type x20 = m_p2 - m_p0;
        vector3_type x10 = m_p1 - m_p0;
        real_type d = max( max(x30*x30, x20*x20), x10*x10);
        return sqrt(d);
      }

      vector3_type get_support_point(vector3_type const & v) const
      {
        vector3_type p;
        real_type tst = -FLT_MAX;//math::detail::lowest<real_type>();
        real_type tmp = v * m_p0;
        if (tmp>tst)
        {
          tst = tmp;
          p = m_p0;
        }
        tmp = v * m_p1;
        if (tmp>tst)
        {
          tst = tmp;
          p = m_p1;
        }
        tmp = v *m_p2;
        if (tmp>tst)
        {
          tst = tmp;
          p = m_p2;
        }
        tmp = v * m_p3;
        if (tmp>tst)
        {
          tst = tmp;
          p = m_p3;
        }
        return p;
      }

      void translate(vector3_type const & T)
      {
        m_p0 += T;
        m_p1 += T;
        m_p2 += T;
        m_p3 += T;
      }

      void rotate(matrix3x3_type const & R)
      {
        vector3_type c = center();
        m_p0 = (R*(m_p0 - c)) + c;
        m_p1 = (R*(m_p1 - c)) + c;
        m_p2 = (R*(m_p2 - c)) + c;
        m_p3 = (R*(m_p3 - c)) + c;
      }

      void scale(real_type const & s)
      {
        vector3_type c = center();
        m_p0 = s*(m_p0 - c) + c;
        m_p1 = s*(m_p1 - c) + c;
        m_p2 = s*(m_p2 - c) + c;
        m_p3 = s*(m_p3 - c) + c;
      }

      real_type volume() const
      {
        /*
        ** From Zienkiewicz & Taylor p.637
        ** V = 1/6*det( 1 x0 y0 z0; 1 x1 y1 z1; 1 x2 y2 z2; 1 x3 y3 z3)
        ** where x0 = n0->i; y0 = n0[1]; ...
        ** Calculated by Mathematica ;-)
        */
        return (m_p0[2]*m_p1[1]*m_p2[0] - m_p0[1]*m_p1[2]*m_p2[0] - m_p0[2]*m_p1[0]*m_p2[1]
        + m_p0[0]*m_p1[2]*m_p2[1] + m_p0[1]*m_p1[0]*m_p2[2] - m_p0[0]*m_p1[1]*m_p2[2]
        - m_p0[2]*m_p1[1]*m_p3[0] + m_p0[1]*m_p1[2]*m_p3[0] + m_p0[2]*m_p2[1]*m_p3[0]
        - m_p1[2]*m_p2[1]*m_p3[0] - m_p0[1]*m_p2[2]*m_p3[0] + m_p1[1]*m_p2[2]*m_p3[0]
        + m_p0[2]*m_p1[0]*m_p3[1] - m_p0[0]*m_p1[2]*m_p3[1] - m_p0[2]*m_p2[0]*m_p3[1]
        + m_p1[2]*m_p2[0]*m_p3[1] + m_p0[0]*m_p2[2]*m_p3[1] - m_p1[0]*m_p2[2]*m_p3[1]
        - m_p0[1]*m_p1[0]*m_p3[2] + m_p0[0]*m_p1[1]*m_p3[2] + m_p0[1]*m_p2[0]*m_p3[2]
        - m_p1[1]*m_p2[0]*m_p3[2] - m_p0[0]*m_p2[1]*m_p3[2] + m_p1[0]*m_p2[1]*m_p3[2])/6.;
      }


      /**
      * Compute Bounding Box.
      * This method computes an axis aligned bounding
      * box (AABB) that encloses the geometry.
      *
      * @param r           The position of the model frame (i.e the coordinate frame the geometry lives in).
      * @param R           The orientation of the model frame (i.e the coordinate frame the geometry lives in).
      * @param min_coord   Upon return holds the minimum corner of the box.
      * @param max_coord   Upon return holds the maximum corner of the box.
      *
      */
      void compute_collision_aabb(
        vector3_type const & r
        , matrix3x3_type const & R
        , vector3_type & min_coord
        , vector3_type & max_coord
        ) const
      {
        compute_tetrahedron_aabb( *this, min_coord, max_coord );
      }


    };

  }  // namespace geometry

} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TETRAHEDRON_H
#endif
