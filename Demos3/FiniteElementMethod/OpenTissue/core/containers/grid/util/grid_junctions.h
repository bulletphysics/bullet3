#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_JUNCTIONS_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_JUNCTIONS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>  // Needed for BasicMathTypes

#include <OpenTissue/core/containers/grid/util/grid_gradient.h>
#include <OpenTissue/core/containers/grid/util/grid_coord2idx.h>
#include <OpenTissue/core/containers/grid/util/grid_idx2coord.h>
#include <OpenTissue/core/containers/grid/util/grid_gradient_at_point.h>
#include <OpenTissue/core/containers/grid/util/grid_local_minima.h>

#include <OpenTissue/core/containers/grid/util/grid_aof.h>

#include <OpenTissue/collision/spatial_hashing/spatial_hashing.h>
#include <OpenTissue/collision/intersect/intersect_sphere_triangle.h>

#include <cmath>
#include <list>

namespace OpenTissue
{
  namespace grid
  {

    namespace detail
    {

      template<typename math_types_>
      class JunctionCandidate
      {
      public:

        typedef math_types_                        math_types;
        typedef typename math_types::vector3_type  vector3_type;
        typedef typename math_types::real_type     real_type;

      public:

        vector3_type   m_center;        //--- Estimated center of the largest enclosing sphere.
        real_type      m_radius;        //--- Upper estimate of the radius of the largest enclosed sphere located at the junction point.
        size_t   m_cnt_boundary;  //--- number of boundary that the largest enclosed sphere touches.

      public:

        JunctionCandidate(vector3_type const & center, real_type const & radius)
          : m_center(center)
          , m_radius(radius)
          , m_cnt_boundary()
        {
          assert(m_radius>0 || !"JunctionCandidate(): Non-positive radius");
        }
      };



      template<typename face_type,typename junction_candiate_type>
      class JunctionCollisionPolicy
      {
      public:

        typedef typename junction_candiate_type::math_types     math_types;
        typedef typename math_types::vector3_type               vector3_type;
        typedef typename math_types::real_type                  real_type;
        typedef face_type                                       data_type;
        typedef junction_candiate_type                          query_type;
        typedef bool                                            result_container;
        typedef OpenTissue::spatial_hashing::PrimeNumberHashFunction             hash_function;

        typedef OpenTissue::spatial_hashing::Grid< vector3_type, OpenTissue::math::Vector3<int>, data_type, hash_function>  hash_grid;

      public:

        vector3_type min_coord(face_type const & face) const
        {
          vector3_type coord;
          mesh::compute_face_minimum_coord(face,coord);
          return coord;
        }

        vector3_type max_coord(face_type const & face) const
        {
          vector3_type coord;
          mesh::compute_face_maximum_coord(face,coord);
          return coord;
        }

        vector3_type min_coord(junction_candiate_type const & candidate) const
        {
          return vector3_type(
            candidate.m_center(0) - candidate.m_radius
            , candidate.m_center(1) - candidate.m_radius
            , candidate.m_center(2) - candidate.m_radius
            );
        }

        vector3_type max_coord(junction_candiate_type const & candidate) const
        {
          return vector3_type(
            candidate.m_center(0) + candidate.m_radius
            , candidate.m_center(1) + candidate.m_radius
            , candidate.m_center(2) + candidate.m_radius
            );
        }

        void reset(result_container & /*results*/)      {   }

        void report(data_type const & data, query_type const & query,result_container & /*results*/)
        {
          face_type * face                   = const_cast<face_type*>(&data);
          junction_candiate_type * candidate = const_cast<junction_candiate_type*>(&query);

          if(candidate->m_cnt_boundary >= 4)
            return;


          OpenTissue::geometry::Triangle<math_types> triangle;
          triangle.set( face );

          typedef OpenTissue::math::BasicMathTypes< real_type, size_t> math_types;
          OpenTissue::geometry::Sphere<math_types> sphere(candidate->m_center,candidate->m_radius);

          if(OpenTissue::intersect::sphere_triangle(sphere,triangle))
          {
            ++(candidate->m_cnt_boundary);
          }

        }
      };

    }// namespace detail



    /**
    * Detect MREP junctions.
    *
    * KE 2006-02-15: This is work in progress, please ignore it!
    *
    * @param aof        A the aof grid.
    * @param F          A grid containing non-zero values for junctions.
    */
    template<typename grid_type, typename mesh_type, typename point_container >
    inline void junctions(
      grid_type const & aof
      , grid_type & phi
      , mesh_type /*const*/ & mesh
      , point_container & points
      )
    {
      using std::fabs;
      using std::min;
      using std::max;
      using std::sqrt;

      typedef typename point_container::value_type                                 vector3_type;
      typedef typename vector3_type::value_type                                    real_type;
      typedef typename grid_type::value_type                                        value_type;
      typedef typename grid_type::const_iterator                                    const_iterator;
      typedef typename grid_type::const_index_iterator                              const_index_iterator;
      typedef typename grid_type::index_iterator                                    index_iterator;
      typedef typename mesh_type::face_type                                        face_type;


      //--- vorticity   w = grad cross grad(phi)  Is this any good????     
      //
      //        d gz      d gy
      // wx =  ------ -  ------
      //         dy        dz
      //
      //        d gx      d gz
      // wy =  ------ -  ------
      //         dz        dx
      //
      //        d gy      d gx
      // wy =  ------ -  ------
      //         dx        dy
      //




      //{
      //-------------------------------------------------------
      //--- Thining  based detection, sucks!!! ----------------
      //-------------------------------------------------------
      //size_t I = phi.I();
      //size_t J = phi.J();
      //size_t K = phi.K();
      //value_type   s[27];
      //grid_type tmp = aof;
      //{
      //  index_iterator        t      = tmp.begin();
      //  const_index_iterator  a      = aof.begin();
      //  const_index_iterator  end    = aof.end();
      //  for(;a!=end;++a,++t)
      //  {
      //    (*t) = 0;
      //    size_t i = a.i();
      //    size_t j = a.j();
      //    size_t k = a.k();
      //    if( (*a) > 0)
      //      (*t) = detail::compute_aof_value(phi,i,j,k,0.1);
      //    if( (*t) < 1e-5)
      //    {
      //      (*t) = 0;
      //      continue;
      //    }
      //    if( phi(i,j,k) > 0)
      //    {
      //      (*t) = 0;
      //      continue;
      //    }
      //  }
      //}
      //{
      //  phi = tmp;
      //  index_iterator t      = phi.begin();
      //  index_iterator a      = tmp.begin();
      //  index_iterator end    = tmp.end();
      //  for(;a!=end;++a,++t)
      //  {
      //    size_t i = a.i();
      //    size_t j = a.j();
      //    size_t k = a.k();
      //    if( (*a) < 1e-5)
      //      continue;
      //    size_t im1   = ( i ) ?  i - 1 : 0;
      //    size_t jm1   = ( j ) ?  j - 1 : 0;
      //    size_t km1   = ( k ) ?  k - 1 : 0;
      //    size_t ip1   = min( i + 1u, I - 1u );
      //    size_t jp1   = min( j + 1u, J - 1u );
      //    size_t kp1   = min( k + 1u, K - 1u );
      //    s[0]  = tmp( im1, jm1, km1);
      //    s[1]  = tmp( im1, jm1,   k);
      //    s[2]  = tmp( im1, jm1, kp1);
      //    s[3]  = tmp( im1,   j, km1);
      //    s[4]  = tmp( im1,   j,   k);
      //    s[5]  = tmp( im1,   j, kp1);
      //    s[6]  = tmp( im1, jp1, km1);
      //    s[7]  = tmp( im1, jp1,   k);
      //    s[8]  = tmp( im1, jp1, kp1);
      //    s[9]  = tmp(   i, jm1, km1);
      //    s[10] = tmp(   i, jm1,   k);
      //    s[11] = tmp(   i, jm1, kp1);
      //    s[12] = tmp(   i,   j, km1);
      //    s[13] = tmp(   i,   j,   k);
      //    s[14] = tmp(   i,   j, kp1);
      //    s[15] = tmp(   i, jp1, km1);
      //    s[16] = tmp(   i, jp1,   k);
      //    s[17] = tmp(   i, jp1, kp1);
      //    s[18] = tmp( ip1, jm1, km1);
      //    s[19] = tmp( ip1, jm1,   k);
      //    s[20] = tmp( ip1, jm1, kp1);
      //    s[21] = tmp( ip1,   j, km1);
      //    s[22] = tmp( ip1,   j,   k);
      //    s[23] = tmp( ip1,   j, kp1);
      //    s[24] = tmp( ip1, jp1, km1);
      //    s[25] = tmp( ip1, jp1,   k);
      //    s[26] = tmp( ip1, jp1, kp1);
      //    bool has_zero_neighbor   = false;
      //    bool has_larger_neighbor = false;
      //    bool is_maxima           = true;
      //    for(size_t i=0u;i<27u;++i)
      //    {
      //      if(s[i]<=0)
      //        has_zero_neighbor = true;
      //      if(s[i]>s[13])
      //        has_larger_neighbor = true;
      //      if(i!=13 && s[13]< s[i])
      //        is_maxima = false;
      //    }
      //    if(!is_maxima && has_larger_neighbor && has_zero_neighbor)//--- thining
      //    {
      //      (*t) = 0;
      //      continue;
      //    }
      //    vector3_type p;
      //    idx2coord(aof,   i,   j,   k, p);
      //    points.push_back(p);
      //  }
      //}




      //-------------------------------------------------------
      //--- Angle based detection, sucks!!! -------------------
      //-------------------------------------------------------
      //vector3_type g[27];
      //vector3_type p[27];
      //value_type   s[27];
      //vector3_type d[27];
      //vector3_type q[27];
      //size_t I = phi.I();
      //size_t J = phi.J();
      //size_t K = phi.K();
      //  const_index_iterator  a      = aof.begin();
      //  const_index_iterator  end    = aof.end();
      //  for(;a!=end;++a,++t)
      //  {
      //    size_t i = a.i();
      //    size_t j = a.j();
      //    size_t k = a.k();
      //    if( (*a) < 1e-5) || phi(i,j,k) > 0)
      //    {
      //      continue;
      //    }
      //size_t im1   = ( i ) ?  i - 1 : 0;
      //size_t jm1   = ( j ) ?  j - 1 : 0;
      //size_t km1   = ( k ) ?  k - 1 : 0;
      //size_t ip1   = min( i + 1u, I - 1u );
      //size_t jp1   = min( j + 1u, J - 1u );
      //size_t kp1   = min( k + 1u, K - 1u );
      //grid_gradient(phi, im1, jm1, km1, g[0]);
      //grid_gradient(phi, im1, jm1,   k, g[1]);
      //grid_gradient(phi, im1, jm1, kp1, g[2]);
      //grid_gradient(phi, im1,   j, km1, g[3]);
      //grid_gradient(phi, im1,   j,   k, g[4]);
      //grid_gradient(phi, im1,   j, kp1, g[5]);
      //grid_gradient(phi, im1, jp1, km1, g[6]);
      //grid_gradient(phi, im1, jp1,   k, g[7]);
      //grid_gradient(phi, im1, jp1, kp1, g[8]);
      //grid_gradient(phi,   i, jm1, km1, g[9]);
      //grid_gradient(phi,   i, jm1,   k, g[10]);
      //grid_gradient(phi,   i, jm1, kp1, g[11]);
      //grid_gradient(phi,   i,   j, km1, g[12]);
      //grid_gradient(phi,   i,   j,   k, g[13]);
      //grid_gradient(phi,   i,   j, kp1, g[14]);
      //grid_gradient(phi,   i, jp1, km1, g[15]);
      //grid_gradient(phi,   i, jp1,   k, g[16]);
      //grid_gradient(phi,   i, jp1, kp1, g[17]);
      //grid_gradient(phi, ip1, jm1, km1, g[18]);
      //grid_gradient(phi, ip1, jm1,   k, g[19]);
      //grid_gradient(phi, ip1, jm1, kp1, g[20]);
      //grid_gradient(phi, ip1,   j, km1, g[21]);
      //grid_gradient(phi, ip1,   j,   k, g[22]);
      //grid_gradient(phi, ip1,   j, kp1, g[23]);
      //grid_gradient(phi, ip1, jp1, km1, g[24]);
      //grid_gradient(phi, ip1, jp1,   k, g[25]);
      //grid_gradient(phi, ip1, jp1, kp1, g[26]);
      //grid_idx2coord(phi, im1, jm1, km1, p[0]);
      //grid_idx2coord(phi, im1, jm1,   k, p[1]);
      //grid_idx2coord(phi, im1, jm1, kp1, p[2]);
      //grid_idx2coord(phi, im1,   j, km1, p[3]);
      //grid_idx2coord(phi, im1,   j,   k, p[4]);
      //grid_idx2coord(phi, im1,   j, kp1, p[5]);
      //grid_idx2coord(phi, im1, jp1, km1, p[6]);
      //grid_idx2coord(phi, im1, jp1,   k, p[7]);
      //grid_idx2coord(phi, im1, jp1, kp1, p[8]);
      //grid_idx2coord(phi,   i, jm1, km1, p[9]);
      //grid_idx2coord(phi,   i, jm1,   k, p[10]);
      //grid_idx2coord(phi,   i, jm1, kp1, p[11]);
      //grid_idx2coord(phi,   i,   j, km1, p[12]);
      //grid_idx2coord(phi,   i,   j,   k, p[13]);
      //grid_idx2coord(phi,   i,   j, kp1, p[14]);
      //grid_idx2coord(phi,   i, jp1, km1, p[15]);
      //grid_idx2coord(phi,   i, jp1,   k, p[16]);
      //grid_idx2coord(phi,   i, jp1, kp1, p[17]);
      //grid_idx2coord(phi, ip1, jm1, km1, p[18]);
      //grid_idx2coord(phi, ip1, jm1,   k, p[19]);
      //grid_idx2coord(phi, ip1, jm1, kp1, p[20]);
      //grid_idx2coord(phi, ip1,   j, km1, p[21]);
      //grid_idx2coord(phi, ip1,   j,   k, p[22]);
      //grid_idx2coord(phi, ip1,   j, kp1, p[23]);
      //grid_idx2coord(phi, ip1, jp1, km1, p[24]);
      //grid_idx2coord(phi, ip1, jp1,   k, p[25]);
      //grid_idx2coord(phi, ip1, jp1, kp1, p[26]);
      //s[0] = phi( im1, jm1, km1);
      //s[1] = phi( im1, jm1,   k);
      //s[2] = phi( im1, jm1, kp1);
      //s[3] = phi( im1,   j, km1);
      //s[4] = phi( im1,   j,   k);
      //s[5] = phi( im1,   j, kp1);
      //s[6] = phi( im1, jp1, km1);
      //s[7] = phi( im1, jp1,   k);
      //s[8] = phi( im1, jp1, kp1);
      //s[9] = phi(   i, jm1, km1);
      //s[10] = phi(   i, jm1,   k);
      //s[11] = phi(   i, jm1, kp1);
      //s[12] = phi(   i,   j, km1);
      //s[13] = phi(   i,   j,   k);
      //s[14] = phi(   i,   j, kp1);
      //s[15] = phi(   i, jp1, km1);
      //s[16] = phi(   i, jp1,   k);
      //s[17] = phi(   i, jp1, kp1);
      //s[18] = phi( ip1, jm1, km1);
      //s[19] = phi( ip1, jm1,   k);
      //s[20] = phi( ip1, jm1, kp1);
      //s[21] = phi( ip1,   j, km1);
      //s[22] = phi( ip1,   j,   k);
      //s[23] = phi( ip1,   j, kp1);
      //s[24] = phi( ip1, jp1, km1);
      //s[25] = phi( ip1, jp1,   k);
      //s[26] = phi( ip1, jp1, kp1);
      //for(size_t i=0u;i<27u;++i)
      //{
      //  q[i] = p[i] - g[i]*s[i];
      //  d[i] = q[i] - p[i];
      //}
      //bool is_junction = false;
      //real_type min_cos_theta  = 100000.0;
      //real_type max_cos_theta  = 0.0;
      //for(size_t i=0u;i<27u;++i)
      //{
      //  if(i==13)
      //    continue;
      //  for(size_t j=i+1;j<27u;++j)
      //  {
      //    if(j==13)
      //      continue;
      //    real_type cos_theta =  fabs( g[i] * g[j] );
      //    cos_theta = max(0.0, min(cos_theta, 1.0));
      //    min_cos_theta = min(cos_theta,min_cos_theta);
      //    max_cos_theta = max(cos_theta,max_cos_theta);
      //  }
      //}
      //std::cout << min_cos_theta << " " << max_cos_theta << std::endl;
      //if(min_cos_theta >0.01 && min_cos_theta< 0.99)
      //  is_junction = true;
      //if(is_junction)
      //points.push_back(p[13]);
      //  }
      //-------------------------------------------------------
      //-------------------------------------------------------
      //-------------------------------------------------------

      std::cout << "junctions(): junction candiates = " << points.size() << std::endl;
    }


    template<typename grid_type, typename mesh_type, typename point_container >
    inline void junctions2(
      grid_type const & aof
      , grid_type const & phi
      , mesh_type /*const*/ & mesh
      , point_container & points
      )
    {
      //--- For each point in the clamped AOF take a sphere with radius
      //--- equal to the distance value plus some small threshold (voxel size!), test
      //--- the sphere for intersection with the mesh-faces
      //---
      //---  If at least four intersection points are found, then the AOF point is a potential junction... no! Think of a pyramid
      //---
      //---  At least four points is necessary but not a sufficient condition...
      //---
      //---  For each intersection take a vector from the AOF point to the face intersection point, if all such vectors
      //---  contrain the sphere then we have a junction point. That is the vectors must span the entire R^3 (to remove
      //---  3DOF of translation motion of the sphere along the mrep).
      //---
      //--- Do we need sub-pixel precision of AOF points?......
      //---
      //---
      //---
      //---  culver  :  exact polyhedra
      //---  foskey  :  theta-SMA
      //---  hoffmann:  voronoi-region/distance map
      //---
      //---
      //---
      //---

      using std::fabs;
      using std::min;
      using std::sqrt;

      typedef typename point_container::value_type                                 vector3_type;
      typedef typename vector3_type::value_type                                    real_type;
      typedef typename grid_type::const_iterator                                    const_iterator;
      typedef typename grid_type::const_index_iterator                              const_index_iterator;

      typedef typename grid_type::math_types                                        math_types;
      typedef typename mesh_type::face_type                                        face_type;
      typedef          detail::JunctionCandidate<math_types>                      candidate_type;

      typedef          std::list<candidate_type>                          candidate_container;
      typedef          detail::JunctionCollisionPolicy<face_type,candidate_type>   collision_policy;
      typedef          AABBDataQuery< typename collision_policy::hash_grid, collision_policy >      query_algorithm;

      bool results = false;
      query_algorithm     query;
      candidate_container candidates;

      {
        real_type          dx  = phi.dx()*0.5;
        real_type          dy  = phi.dy()*0.5;
        real_type          dz  = phi.dz()*0.5;
        real_type          rho = sqrt(dx*dx+dy*dy+dz*dz)*.5;

        const_iterator        p       = phi.begin();
        const_index_iterator  a       = aof.begin();
        const_index_iterator  end    = aof.end();
        for(;a!=end;++a,++p)
        {
          if( (*a) < 1e-5  || (*p) >= 0)
            continue;
          vector3_type center;
          idx2coord(a,center);
          real_type radius = fabs( *p );
          candidate_type candidate(center,radius + rho);
          candidates.push_back(candidate);
        }
      }

      query.init(candidates.begin(),candidates.end());
      query(mesh.face_begin(), mesh.face_end(), candidates.begin(), candidates.end(), results, typename query_algorithm::all_tag() );

      size_t cnt = 0;
      for(typename candidate_container::iterator c = candidates.begin(); c!=candidates.end();++c)
      {
        if(c->m_cnt_boundary >= 4)
        {
          points.push_back(c->m_center );
          ++cnt;
        }
      }
      std::cout << "junctions2(): junction candiates = " << cnt << std::endl;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_JUNCTIONS_H
#endif
