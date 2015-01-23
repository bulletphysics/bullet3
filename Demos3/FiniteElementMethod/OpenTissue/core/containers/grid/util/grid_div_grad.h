#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_DIV_GRAD_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_DIV_GRAD_H
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
#include <cmath>

namespace OpenTissue
{
  namespace grid
  {

    /**
    * Div-Grad.
    * Computes the divergence of the gradient of a specified field. Ie. the flux of the gradient field!
    *
    * @param phi        A the grid.
    * @param M          A grid containing the values of the divergence of the gradient field of phi.
    */
    template<typename grid_type>
    inline void div_grad(
      grid_type const & phi
      , grid_type & M
      )
    {
      using std::min;

      typedef OpenTissue::math::Vector3< typename grid_type::value_type>  vector3_type;

      typedef typename grid_type::iterator              iterator;
      typedef typename grid_type::const_index_iterator  const_index_iterator;
      typedef typename grid_type::value_type            value_type;
      typedef typename grid_type::math_types            math_types;
      typedef typename math_types::real_type            real_type;

      size_t const & I = phi.I();
      size_t const & J = phi.J();
      size_t const & K = phi.K();
      M.create(phi.min_coord(),phi.max_coord(),I,J,K);
      static value_type unused = phi.unused();

      vector3_type /*g000,*/gp00,gm00,g0p0,g0m0,g00p,g00m;
      vector3_type gppp,gppm,gpmp,gpmm,gmpp,gmpm,gmmp,gmmm;
      vector3_type gpp0, gpm0, gmp0, gmm0, gp0p, gp0m, gm0p, gm0m, g0pp, g0pm, g0mp, g0mm;

      static vector3_type np00 = unit(vector3_type( 1, 0, 0));
      static vector3_type nm00 = unit(vector3_type(-1, 0, 0));
      static vector3_type n0p0 = unit(vector3_type( 0, 1, 0));
      static vector3_type n0m0 = unit(vector3_type( 0,-1, 0));
      static vector3_type n00p = unit(vector3_type( 0, 0, 1));
      static vector3_type n00m = unit(vector3_type( 0, 0,-1));
      static vector3_type nppp = unit(vector3_type( 1, 1, 1));
      static vector3_type nppm = unit(vector3_type( 1, 1,-1));
      static vector3_type npmp = unit(vector3_type( 1,-1, 1));
      static vector3_type npmm = unit(vector3_type( 1,-1,-1));
      static vector3_type nmpp = unit(vector3_type(-1, 1, 1));
      static vector3_type nmpm = unit(vector3_type(-1, 1,-1));
      static vector3_type nmmp = unit(vector3_type(-1,-1, 1));
      static vector3_type nmmm = unit(vector3_type(-1,-1,-1));
      static vector3_type npp0 = unit(vector3_type( 1, 1, 0));
      static vector3_type npm0 = unit(vector3_type( 1,-1, 0));
      static vector3_type nmp0 = unit(vector3_type(-1, 1, 0));
      static vector3_type nmm0 = unit(vector3_type(-1,-1, 0));
      static vector3_type np0p = unit(vector3_type( 1, 0, 1));
      static vector3_type np0m = unit(vector3_type( 1, 0,-1));
      static vector3_type nm0p = unit(vector3_type(-1, 0, 1));
      static vector3_type nm0m = unit(vector3_type(-1, 0,-1));
      static vector3_type n0pp = unit(vector3_type( 0, 1, 1));
      static vector3_type n0pm = unit(vector3_type( 0, 1,-1));
      static vector3_type n0mp = unit(vector3_type( 0,-1, 1));
      static vector3_type n0mm = unit(vector3_type( 0,-1,-1));


      iterator              m       = M.begin();
      const_index_iterator  pbegin  = phi.begin();
      const_index_iterator  pend    = phi.end();
      const_index_iterator  p       = pbegin;
      for(;p!=pend;++p,++m)
      {
        if((*p)==unused)
        {
          *m = value_type(); //--- should default to zero!!!
          continue;
        }

        size_t i     = p.i();
        size_t j     = p.j();
        size_t k     = p.k();
        size_t im1   = ( i ) ?  i - 1 : 0;
        size_t jm1   = ( j ) ?  j - 1 : 0;
        size_t km1   = ( k ) ?  k - 1 : 0;
        size_t ip1   = min( i + 1u, I - 1u );
        size_t jp1   = min( j + 1u, J - 1u );
        size_t kp1   = min( k + 1u, K - 1u );

        gradient(phi, ip1,    j,    k, gp00);
        gradient(phi, im1,    j,    k, gm00);
        gradient(phi,   i,  jp1,    k, g0p0);
        gradient(phi,   i,  jm1,    k, g0m0);
        gradient(phi,   i,    j,  kp1, g00p);
        gradient(phi,   i,    j,  km1, g00m);
        gradient(phi, ip1,  jp1,    k, gpp0);
        gradient(phi, ip1,  jm1,    k, gpm0);
        gradient(phi, im1,  jp1,    k, gmp0);
        gradient(phi, im1,  jm1,    k, gmm0);
        gradient(phi, ip1,    j,  kp1, gp0p);
        gradient(phi, ip1,    j,  km1, gp0m);
        gradient(phi, im1,    j,  kp1, gm0p);
        gradient(phi, im1,    j,  km1, gm0m);
        gradient(phi,   i,  jp1,  kp1, g0pp);
        gradient(phi,   i,  jp1,  km1, g0pm);
        gradient(phi,   i,  jm1,  kp1, g0mp);
        gradient(phi,   i,  jm1,  km1, g0mm);
        gradient(phi, ip1,  jp1,  kp1, gppp);
        gradient(phi, ip1,  jp1,  km1, gppm);
        gradient(phi, ip1,  jm1,  kp1, gpmp);
        gradient(phi, ip1,  jm1,  km1, gpmm);
        gradient(phi, im1,  jp1,  kp1, gmpp);
        gradient(phi, im1,  jp1,  km1, gmpm);
        gradient(phi, ip1,  jm1,  kp1, gmmp);
        gradient(phi, im1,  jm1,  km1, gmmm);

        real_type flux = real_type(); //--- should default to zero!!!
        flux += gp00 * np00;
        flux += gm00 * nm00;
        flux += g0p0 * n0p0;
        flux += g0m0 * n0m0;
        flux += g00p * n00p;
        flux += g00m * n00m;
        flux += gpp0 * npp0;
        flux += gpm0 * npm0;
        flux += gmp0 * nmp0;
        flux += gmm0 * nmm0;
        flux += gp0p * np0p;
        flux += gp0m * np0m;
        flux += gm0p * nm0p;
        flux += gm0m * nm0m;
        flux += g0pp * n0pp;
        flux += g0pm * n0pm;
        flux += g0mp * n0mp;
        flux += g0mm * n0mm;
        flux += gppp * nppp;
        flux += gppm * nppm;
        flux += gpmp * npmp;
        flux += gpmm * npmm;
        flux += gmpp * nmpp;
        flux += gmpm * nmpm;
        flux += gmmp * nmmp;
        flux += gmmm * nmmm;

        (*m) = static_cast<value_type>(flux);
      }
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_DIV_GRAD_H
#endif
