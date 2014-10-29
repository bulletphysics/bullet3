#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_AABB_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_AABB_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_volume_shape.h>
#include <OpenTissue/utility/utility_class_id.h>

#include <algorithm>
#include <cassert>


namespace OpenTissue
{

  namespace geometry
  {

    /**
    * Axed Aligned Bounding Box (AABB).
    */
    template< typename math_types_ >
    class AABB
      : public VolumeShape< math_types_ >
      , public OpenTissue::utility::ClassID< AABB<math_types_> >
    {
    public:

      typedef          math_types_                   math_types;
      typedef typename math_types::value_traits      value_traits;
      typedef typename math_types::real_type         real_type;
      typedef typename math_types::vector3_type      vector3_type;
      typedef typename math_types::matrix3x3_type    matrix3x3_type;
      typedef typename math_types::quaternion_type   quaternion_type;

    public:

      vector3_type m_min;  ///< Coordinates of minimum corner.
      vector3_type m_max;  ///< Coordinates of maximum corner.

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::AABB<math_types_> >::class_id(); }

      AABB()
      {
        m_min.clear();
        m_max.clear();
      }

      explicit AABB( 
        real_type const & xmin
        , real_type const & ymin
        , real_type const & zmin
        , real_type const & xmax
        , real_type const & ymax
        , real_type const & zmax 
        )
      {
        set(xmin,ymin,zmin,xmax,ymax,zmax);
      }

      explicit AABB( vector3_type const & pmin_v, vector3_type const & pmax_v) { set(pmin_v,pmax_v); }

      virtual ~AABB() {}

    public:

      void compute_surface_points(std::vector<vector3_type> & points) const
      {
        vector3_type dia;
        dia = m_max - m_min;

        vector3_type p000 = m_min;
        vector3_type p001 = m_min + vector3_type(              dia[0], value_traits::zero(), value_traits::zero());
        vector3_type p010 = m_min + vector3_type(value_traits::zero(),               dia[1], value_traits::zero());
        vector3_type p011 = m_min + vector3_type(              dia[0],               dia[1], value_traits::zero());
        vector3_type p100 = m_min + vector3_type(value_traits::zero(), value_traits::zero(),               dia[2]);
        vector3_type p101 = m_min + vector3_type(              dia[0], value_traits::zero(),               dia[2]);
        vector3_type p110 = m_min + vector3_type(value_traits::zero(),               dia[1],               dia[2]);
        vector3_type p111 = m_min + vector3_type(              dia[0],               dia[1],               dia[2]);

        points.push_back(p000);
        points.push_back(p001);
        points.push_back(p010);
        points.push_back(p011);
        points.push_back(p100);
        points.push_back(p101);
        points.push_back(p110);
        points.push_back(p111);
      }

      vector3_type center() const { return vector3_type( (m_max+m_min)/value_traits::two() ); }

    public:
      /**
      * Assingment Method.
      * Assigns the size of the specified AABB to this AABB.
      *
      * @param other_aabb   Another AABB
      */
      void set(AABB const & other_aabb)
      {
        this->m_min = other_aabb.m_min;
        this->m_max = other_aabb.m_max;
      }

      void set( vector3_type const & pmin_v, vector3_type const & pmax_v)
      {
        assert(pmin_v[0] <= pmax_v[0] || !"AABB.set(): minimum must be less than or equal to maximum");
        assert(pmin_v[1] <= pmax_v[1] || !"AABB.set(): minimum must be less than or equal to maximum");
        assert(pmin_v[2] <= pmax_v[2] || !"AABB.set(): minimum must be less than or equal to maximum");
        m_min = pmin_v;
        m_max = pmax_v;
      }

      /**
      * Set AABB box.
      *
      *  @param xmin
      *  @param ymin
      *  @param zmin
      *  @param xmax
      *  @param ymax
      *  @param zmax
      */
      void set( 
        real_type const & xmin
        , real_type const & ymin
        , real_type const & zmin
        , real_type const & xmax
        , real_type const & ymax
        , real_type const & zmax
        )
      {
        assert(xmin<=xmax || !"AABB.set(): minimum must be less than or equal to maximum");
        assert(ymin<=ymax || !"AABB.set(): minimum must be less than or equal to maximum");
        assert(zmin<=zmax || !"AABB.set(): minimum must be less than or equal to maximum");
        m_min = vector3_type(xmin,ymin,zmin);
        m_max = vector3_type(xmax,ymax,zmax);
      }

      /**
      *   Set AABB box.
      *
      *  @param xmin
      *  @param ymin
      *  @param zmin
      *  @param width
      *  @param height
      *  @param depth
      */
      void extent( 
        real_type const & xmin
        , real_type const & ymin
        , real_type const & zmin
        , real_type const & width
        , real_type const & height
        , real_type const & depth
        )
      {
        assert(width>=0 || !"AABB.extent(): width must be non-negative");
        assert(height>=0 || !"AABB.extent(): height must be non-negative");
        assert(depth>=0 || !"AABB.extent(): depth must be non-negative");
        m_min = vector3_type(xmin,ymin,zmin);
        m_max = vector3_type(xmin+width,ymin+height,zmin+depth);
      }

    public:

      real_type const & x() const    {      return m_min[0];    }
      real_type const & y() const    {      return m_min[1];    }
      real_type const & z() const    {      return m_min[2];    }
      real_type         w() const    {      return m_max[0] - m_min[0];    }
      real_type         h() const    {      return m_max[1] - m_min[1];    }
      real_type         d() const    {      return m_max[2] - m_min[2];    }

      vector3_type       & min()       { return m_min; }
      vector3_type       & max()       { return m_max; }
      vector3_type const & min() const { return m_min; }
      vector3_type const & max() const { return m_max; }

      /**
      * Update Bounding Box
      * This method updates the size of the AABB such that it encloses the
      * given nodes.
      *
      * @param n0
      * @param n1
      * @param n2
      */
      void update(vector3_type const & n0,vector3_type const & n1, vector3_type const & n2)
      {
        using std::min;
        using std::max;

        m_min[0] = min(n2[0],min(n1[0],n0[0]));
        m_min[1] = min(n2[1],min(n1[1],n0[1]));
        m_min[2] = min(n2[2],min(n1[2],n0[2]));
        m_max[0] = max(n2[0],max(n1[0],n0[0]));
        m_max[1] = max(n2[1],max(n1[1],n0[1]));
        m_max[2] = max(n2[2],max(n1[2],n0[2]));
      }

      /**
      * Update Bounding Box
      * This method updates the size of the AABB such that it encloses the
      * given nodes.
      *
      * @param n0
      * @param n1
      * @param n2
      * @param n3
      */
      void update(vector3_type const & n0,vector3_type const & n1,vector3_type const & n2, vector3_type const & n3)
      {
        using std::min;
        using std::max;

        m_min[0] = min(n3[0],min(n2[0],min(n1[0],n0[0])));
        m_min[1] = min(n3[1],min(n2[1],min(n1[1],n0[1])));
        m_min[2] = min(n3[2],min(n2[2],min(n1[2],n0[2])));
        m_max[0] = max(n3[0],max(n2[0],max(n1[0],n0[0])));
        m_max[1] = max(n3[1],max(n2[1],max(n1[1],n0[1])));
        m_max[2] = max(n3[2],max(n2[2],max(n1[2],n0[2])));
      }

      /**
      * Update Box.
      * This method updates the size of the AABB such that it encloses
      * the given AABBs.
      *
      * @param A
      * @param B
      */
      void update(AABB const & A,AABB const & B)
      {
        using std::min;
        using std::max;

        m_min[0] = min(A.m_min[0],B.m_min[0]);
        m_min[1] = min(A.m_min[1],B.m_min[1]);
        m_min[2] = min(A.m_min[2],B.m_min[2]);
        m_max[0] = max(A.m_max[0],B.m_max[0]);
        m_max[1] = max(A.m_max[1],B.m_max[1]);
        m_max[2] = max(A.m_max[2],B.m_max[2]);
      }

      /**
      * Get Volume.
      *
      * @return      The current value of the volume of the AABB.
      */
      real_type volume() const
      {
        vector3_type vd = m_max-m_min;
        return vd[0]*vd[1]*vd[2];
      }

      real_type area() const
      {
        vector3_type vd = m_max-m_min;
        return 2*vd[0]*vd[1]  + 2*vd[0]*vd[2]  + 2*vd[2]*vd[1];
      }

      real_type diameter() const
      {
        vector3_type vd = m_max-m_min;
        return length(vd);
      }

      void translate(vector3_type const & T)
      {
        m_max += T;
        m_min += T;
      }

      void rotate(matrix3x3_type const & /*R*/)    {    }

      void scale(real_type const & s)
      {
        assert(s>=value_traits::zero() || !"AABB::scale: s must be non-negative");
        vector3_type c = center();
        m_min -= c;
        m_max -= c;
        m_min *= s;
        m_max *= s;
        m_min += c;
        m_max += c;
      }

      vector3_type get_support_point(vector3_type const & v) const
      {
        vector3_type dir = unit(v);                
        vector3_type ext = (m_max - m_min)/2.;
        vector3_type c = center();
        vector3_type p(0,0,0);

        real_type sign = value_traits::zero();
        real_type tst = ext[0] * dir[0];
        if(tst>0)
          sign = value_traits::one();
        else if(tst<0)
          sign = -value_traits::one();
        else //(tst==0)
          sign = value_traits::zero();
        p[0] += sign * ext[0];

        tst = ext[1] * dir[1];
        if(tst>0)
          sign = value_traits::one();
        else if(tst<0)
          sign = -value_traits::one();
        else //(tst==0)
          sign = value_traits::zero();
        p[1] += sign * ext[1];

        tst = ext[2] * dir[2];
        if(tst>0)
          sign = value_traits::one();
        else if(tst<0)
          sign = -value_traits::one();
        else //(tst==0)
          sign = value_traits::zero();
        p[2] += sign * ext[2];
        p += c;
        return p;
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
        , matrix3x3_type const & /*R*/
        , vector3_type & min_coord
        , vector3_type & max_coord
        ) const
      {
        min_coord = r + this->min();
        max_coord = r + this->max();
      }


    };

  }  // namespace geometry

} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_AABB_H
#endif
