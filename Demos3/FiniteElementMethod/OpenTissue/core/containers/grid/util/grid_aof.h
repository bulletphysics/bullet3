#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_AOF_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_AOF_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>


#include <OpenTissue/core/containers/grid/util/grid_gradient.h>
#include <OpenTissue/core/containers/grid/util/grid_coord2idx.h>
#include <OpenTissue/core/containers/grid/util/grid_idx2coord.h>
#include <OpenTissue/core/containers/grid/util/grid_gradient_at_point.h>

#include <cmath>

namespace OpenTissue
{
  namespace grid
  {

    namespace detail
    {

      /**
      * Compute Average Outward Flux at designated nodal position of a signed distance grid.
      *
      *
      * @param phi    The signed distance grid.
      * @param i      The i'th index of the node.
      * @param j      The j'th index of the node.
      * @param k      The k'th index of the node.
      * @param scale  Default value is 0.5, this determines the scale on which the flux is computed. 0.5 correponds to ``voxel-based''.
      *
      * @return       The flux value.
      */
      template<typename grid_type>
      inline typename grid_type::value_type
        compute_aof_value(
        grid_type const & phi
        , size_t const & i
        , size_t const & j
        , size_t const & k
        , double scale = 0.5
        )
      {
        using std::min;
        using std::sqrt;

        typedef typename grid_type::iterator              iterator;
        typedef typename grid_type::const_index_iterator  const_index_iterator;
        typedef typename grid_type::value_type            value_type;
        typedef typename grid_type::math_types            math_types;
        typedef typename math_types::vector3_type         vector3_type;
        typedef typename math_types::real_type            real_type;

        real_type          dx  = phi.dx()*scale;
        real_type          dy  = phi.dy()*scale;
        real_type          dz  = phi.dz()*scale;

        vector3_type n[26];
        vector3_type dp[26];

        dp[0]  = vector3_type( dx,  0,  0);
        dp[1]  = vector3_type(-dx,  0,  0);
        dp[2]  = vector3_type(  0, dy,  0);
        dp[3]  = vector3_type(  0,-dy,  0);
        dp[4]  = vector3_type(  0,  0, dz);
        dp[5]  = vector3_type(  0,  0,-dz);
        dp[6]  = vector3_type( dx, dy, dz);
        dp[7]  = vector3_type( dx, dy,-dz);
        dp[8]  = vector3_type( dx,-dy, dz);
        dp[9]  = vector3_type( dx,-dy,-dz);
        dp[10] = vector3_type(-dx, dy, dz);
        dp[11] = vector3_type(-dx, dy,-dz);
        dp[12] = vector3_type(-dx,-dy, dz);
        dp[13] = vector3_type(-dx,-dy,-dz);
        dp[14] = vector3_type( dx, dy,  0);
        dp[15] = vector3_type( dx,-dy,  0);
        dp[16] = vector3_type(-dx, dy,  0);
        dp[17] = vector3_type(-dx,-dy,  0);
        dp[18] = vector3_type( dx,  0, dz);
        dp[19] = vector3_type( dx,  0,-dz);
        dp[20] = vector3_type(-dx,  0, dz);
        dp[21] = vector3_type(-dx,  0,-dz);
        dp[22] = vector3_type(  0, dy, dz);
        dp[23] = vector3_type(  0, dy,-dz);
        dp[24] = vector3_type(  0,-dy, dz);
        dp[25] = vector3_type(  0,-dy,-dz);
        for(size_t cnt=0;cnt<26u;++cnt)
          n[cnt] = unit(dp[cnt]);

        vector3_type p;
        idx2coord(phi,i,j,k,p);
        real_type flux = real_type();
        for(size_t cnt=0;cnt<26u;++cnt)
        {
          vector3_type q = p + dp[cnt];
          vector3_type g = gradient_at_point(phi,q);
          flux += unit(g)*n[cnt];
        }
        flux /= 26.0;
        return static_cast<value_type>(flux);
      }

    } //namespace detail


    /**
    * Average Outward Flux.
    *
    * @param phi        A the distance grid.
    * @param F          A grid containing the values of the average outward flux.
    */
    template<typename grid_type>
    inline void aof(
      grid_type const & phi
      , grid_type & F
      )
    {
      using std::min;
      using std::sqrt;

      typedef typename grid_type::iterator              iterator;
      typedef typename grid_type::const_index_iterator  const_index_iterator;
      typedef typename grid_type::value_type            value_type;
      typedef typename grid_type::math_types            math_types;
      typedef typename math_types::vector3_type         vector3_type;
      typedef typename math_types::real_type            real_type;

      value_type const unused = phi.unused();
      size_t const & I   = phi.I();
      size_t const & J   = phi.J();
      size_t const & K   = phi.K();
      F.create(phi.min_coord(),phi.max_coord(),I,J,K);

      real_type          dx  = phi.dx()*0.5;
      real_type          dy  = phi.dy()*0.5;
      real_type          dz  = phi.dz()*0.5;
      //real_type          rho = sqrt(dx*dx+dy*dy+dz*dz);
      vector3_type n[26];
      vector3_type dp[26];
      dp[0]  = vector3_type( dx,  0,  0);
      dp[1]  = vector3_type(-dx,  0,  0);
      dp[2]  = vector3_type(  0, dy,  0);
      dp[3]  = vector3_type(  0,-dy,  0);
      dp[4]  = vector3_type(  0,  0, dz);
      dp[5]  = vector3_type(  0,  0,-dz);
      dp[6]  = vector3_type( dx, dy, dz);
      dp[7]  = vector3_type( dx, dy,-dz);
      dp[8]  = vector3_type( dx,-dy, dz);
      dp[9]  = vector3_type( dx,-dy,-dz);
      dp[10] = vector3_type(-dx, dy, dz);
      dp[11] = vector3_type(-dx, dy,-dz);
      dp[12] = vector3_type(-dx,-dy, dz);
      dp[13] = vector3_type(-dx,-dy,-dz);
      dp[14] = vector3_type( dx, dy,  0);
      dp[15] = vector3_type( dx,-dy,  0);
      dp[16] = vector3_type(-dx, dy,  0);
      dp[17] = vector3_type(-dx,-dy,  0);
      dp[18] = vector3_type( dx,  0, dz);
      dp[19] = vector3_type( dx,  0,-dz);
      dp[20] = vector3_type(-dx,  0, dz);
      dp[21] = vector3_type(-dx,  0,-dz);
      dp[22] = vector3_type(  0, dy, dz);
      dp[23] = vector3_type(  0, dy,-dz);
      dp[24] = vector3_type(  0,-dy, dz);
      dp[25] = vector3_type(  0,-dy,-dz);
      for(size_t i=0;i<26u;++i)
        n[i] = unit(dp[i]);
      //n[0]  = unit(vector3_type( 1, 0, 0));
      //n[1]  = unit(vector3_type(-1, 0, 0));
      //n[2]  = unit(vector3_type( 0, 1, 0));
      //n[3]  = unit(vector3_type( 0,-1, 0));
      //n[4]  = unit(vector3_type( 0, 0, 1));
      //n[5]  = unit(vector3_type( 0, 0,-1));
      //n[6]  = unit(vector3_type( 1, 1, 1));
      //n[7]  = unit(vector3_type( 1, 1,-1));
      //n[8]  = unit(vector3_type( 1,-1, 1));
      //n[9]  = unit(vector3_type( 1,-1,-1));
      //n[10] = unit(vector3_type(-1, 1, 1));
      //n[11] = unit(vector3_type(-1, 1,-1));
      //n[12] = unit(vector3_type(-1,-1, 1));
      //n[13] = unit(vector3_type(-1,-1,-1));
      //n[14] = unit(vector3_type( 1, 1, 0));
      //n[15] = unit(vector3_type( 1,-1, 0));
      //n[16] = unit(vector3_type(-1, 1, 0));
      //n[17] = unit(vector3_type(-1,-1, 0));
      //n[18] = unit(vector3_type( 1, 0, 1));
      //n[19] = unit(vector3_type( 1, 0,-1));
      //n[20] = unit(vector3_type(-1, 0, 1));
      //n[21] = unit(vector3_type(-1, 0,-1));
      //n[22] = unit(vector3_type( 0, 1, 1));
      //n[23] = unit(vector3_type( 0, 1,-1));
      //n[24] = unit(vector3_type( 0,-1, 1));
      //n[25] = unit(vector3_type( 0,-1,-1));
      //for(size_t i=0;i<26u;++i)
      //  dp[i] = n[i]*rho;
      iterator              f       = F.begin();
      const_index_iterator  v       = phi.begin();
      const_index_iterator  vend    = phi.end();
      for(;v!=vend;++v,++f)
      {
        if((*v)==unused)
        {
          (*f) = value_type(); //--- should default to zero!!!
          continue;
        }
        size_t i     = v.i();
        size_t j     = v.j();
        size_t k     = v.k();
        vector3_type p;
        idx2coord(phi,i,j,k,p);
        real_type flux = real_type();
        for(size_t i=0;i<26u;++i)
        {
          vector3_type q = p + dp[i];
          vector3_type g = gradient_at_point(phi,q);
          flux += unit(g)*n[i];
        }
        flux /= 26.0;
        (*f) = static_cast<value_type>(flux);
      }

      //vector3_type n_mpp = unit(vector3_type( -1,  1,  1));
      //vector3_type n_mp0 = unit(vector3_type( -1,  1,  0));
      //vector3_type n_mpm = unit(vector3_type( -1,  1, -1));
      //vector3_type n_m0p = unit(vector3_type( -1,  0,  1));
      //vector3_type n_m00 = unit(vector3_type( -1,  0,  0));
      //vector3_type n_m0m = unit(vector3_type( -1,  0, -1));
      //vector3_type n_mmp = unit(vector3_type( -1, -1,  1));
      //vector3_type n_mm0 = unit(vector3_type( -1, -1,  0));
      //vector3_type n_mmm = unit(vector3_type( -1, -1, -1));
      //vector3_type n_0pp = unit(vector3_type(  0,  1,  1));
      //vector3_type n_0p0 = unit(vector3_type(  0,  1,  0));
      //vector3_type n_0pm = unit(vector3_type(  0,  1, -1));
      //vector3_type n_00p = unit(vector3_type(  0,  0,  1));
      //vector3_type n_000 = unit(vector3_type(  0,  0,  0));
      //vector3_type n_00m = unit(vector3_type(  0,  0, -1));
      //vector3_type n_0mp = unit(vector3_type(  0, -1,  1));
      //vector3_type n_0m0 = unit(vector3_type(  0, -1,  0));
      //vector3_type n_0mm = unit(vector3_type(  0, -1, -1));
      //vector3_type n_ppp = unit(vector3_type(  1,  1,  1));
      //vector3_type n_pp0 = unit(vector3_type(  1,  1,  0));
      //vector3_type n_ppm = unit(vector3_type(  1,  1, -1));
      //vector3_type n_p0p = unit(vector3_type(  1,  0,  1));
      //vector3_type n_p00 = unit(vector3_type(  1,  0,  0));
      //vector3_type n_p0m = unit(vector3_type(  1,  0, -1));
      //vector3_type n_pmp = unit(vector3_type(  1, -1,  1));
      //vector3_type n_pm0 = unit(vector3_type(  1, -1,  0));
      //vector3_type n_pmm = unit(vector3_type(  1, -1, -1));
      //vector3_type g_mpp;
      //vector3_type g_mp0;
      //vector3_type g_mpm;
      //vector3_type g_m0p;
      //vector3_type g_m00;
      //vector3_type g_m0m;
      //vector3_type g_mmp;
      //vector3_type g_mm0;
      //vector3_type g_mmm;
      //vector3_type g_0pp;
      //vector3_type g_0p0;
      //vector3_type g_0pm;
      //vector3_type g_00p;
      //vector3_type g_000;
      //vector3_type g_00m;
      //vector3_type g_0mp;
      //vector3_type g_0m0;
      //vector3_type g_0mm;
      //vector3_type g_ppp;
      //vector3_type g_pp0;
      //vector3_type g_ppm;
      //vector3_type g_p0p;
      //vector3_type g_p00;
      //vector3_type g_p0m;
      //vector3_type g_pmp;
      //vector3_type g_pm0;
      //vector3_type g_pmm;
      //iterator              f       = F.begin();
      //const_index_iterator  v       = phi.begin();
      //const_index_iterator  vend    = phi.end();
      //for(;v!=vend;++v,++f)
      //{
      //  if((*v)==unused)
      //  {
      //    (*f) = value_type(); //--- should default to zero!!!
      //    continue;
      //  }
      //  size_t i     = v.i();
      //  size_t j     = v.j();
      //  size_t k     = v.k();
      //  size_t im1   = ( i ) ?  i - 1 : 0;
      //  size_t jm1   = ( j ) ?  j - 1 : 0;
      //  size_t km1   = ( k ) ?  k - 1 : 0;
      //  size_t ip1   = min( i + 1u, I - 1u );
      //  size_t jp1   = min( j + 1u, J - 1u );
      //  size_t kp1   = min( k + 1u, K - 1u );
      //  gradient(phi, im1, jp1, kp1, g_mpp );
      //  gradient(phi, im1, jp1,   k, g_mp0 );
      //  gradient(phi, im1, jp1, km1, g_mpm );
      //  gradient(phi, im1,   j, kp1, g_m0p );
      //  gradient(phi, im1,   j,   k, g_m00 );
      //  gradient(phi, im1,   j, km1, g_m0m );
      //  gradient(phi, im1, jm1, kp1, g_mmp );
      //  gradient(phi, im1, jm1,   k, g_mm0 );
      //  gradient(phi, im1, jm1, km1, g_mmm );
      //  gradient(phi,   i, jp1, kp1, g_0pp );
      //  gradient(phi,   i, jp1,   k, g_0p0 );
      //  gradient(phi,   i, jp1, km1, g_0pm );
      //  gradient(phi,   i,   j, kp1, g_00p );
      //  gradient(phi,   i,   j,   k, g_000 );
      //  gradient(phi,   i,   j, km1, g_00m );
      //  gradient(phi,   i, jm1, kp1, g_0mp );
      //  gradient(phi,   i, jm1,   k, g_0m0 );
      //  gradient(phi,   i, jm1, km1, g_0mm );
      //  gradient(phi, ip1, jp1, kp1, g_ppp );
      //  gradient(phi, ip1, jp1,   k, g_pp0 );
      //  gradient(phi, ip1, jp1, km1, g_ppm );
      //  gradient(phi, ip1,   j, kp1, g_p0p );
      //  gradient(phi, ip1,   j,   k, g_p00 );
      //  gradient(phi, ip1,   j, km1, g_p0m );
      //  gradient(phi, ip1, jm1, kp1, g_pmp );
      //  gradient(phi, ip1, jm1,   k, g_pm0 );
      //  gradient(phi, ip1, jm1, km1, g_pmm );
      //  real_type flux = real_type();
      //  flux += unit(g_mpp) * n_mpp;
      //  flux += unit(g_mp0) * n_mp0;
      //  flux += unit(g_mpm) * n_mpm;
      //  flux += unit(g_m0p) * n_m0p;
      //  flux += unit(g_m00) * n_m00;
      //  flux += unit(g_m0m) * n_m0m;
      //  flux += unit(g_mmp) * n_mmp;
      //  flux += unit(g_mm0) * n_mm0;
      //  flux += unit(g_mmm) * n_mmm;
      //  flux += unit(g_0pp) * n_0pp;
      //  flux += unit(g_0p0) * n_0p0;
      //  flux += unit(g_0pm) * n_0pm;
      //  flux += unit(g_00p) * n_00p;
      //  flux += unit(g_000) * n_000;
      //  flux += unit(g_00m) * n_00m;
      //  flux += unit(g_0mp) * n_0mp;
      //  flux += unit(g_0m0) * n_0m0;
      //  flux += unit(g_0mm) * n_0mm;
      //  flux += unit(g_ppp) * n_ppp;
      //  flux += unit(g_pp0) * n_pp0;
      //  flux += unit(g_ppm) * n_ppm;
      //  flux += unit(g_p0p) * n_p0p;
      //  flux += unit(g_p00) * n_p00;
      //  flux += unit(g_p0m) * n_p0m;
      //  flux += unit(g_pmp) * n_pmp;
      //  flux += unit(g_pm0) * n_pm0;
      //  flux += unit(g_pmm) * n_pmm;
      //  flux /= 26.0;
      //  (*f) = static_cast<value_type>(flux);
      //}

    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_AOF_H
#endif
