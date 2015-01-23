#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_UPWIND_GRADIENT_FIELD_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_UPWIND_GRADIENT_FIELD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>

namespace OpenTissue
{
  namespace grid
  {

    /**
    * Picks upwind direction of gradient according to the speed function (F).
    */
    template < typename grid_type >
    inline void upwind_gradient_field(
      grid_type const & phi
      , grid_type const & F
      , grid_type & Nx
      , grid_type & Ny
      , grid_type & Nz
      )
    {
      using std::min;

      typedef OpenTissue::math::Vector3< typename grid_type::value_type>  vector3_type;

      typedef typename grid_type::value_type            value_type;
      typedef typename grid_type::iterator              iterator;
      typedef typename grid_type::const_iterator        const_iterator;
      typedef typename grid_type::const_index_iterator  const_index_iterator;
      typedef typename grid_type::math_types            math_types;
      typedef typename math_types::real_type            real_type;

      size_t I = phi.I();
      size_t J = phi.J();
      size_t K = phi.K();
      real_type inv_dx = 1.0/phi.dx();
      real_type inv_dy = 1.0/phi.dy();
      real_type inv_dz = 1.0/phi.dz();

      real_type inv_2dx = 0.5/phi.dx();
      real_type inv_2dy = 0.5/phi.dy();
      real_type inv_2dz = 0.5/phi.dz();

      iterator              nx      = Nx.begin();
      iterator              ny      = Ny.begin();
      iterator              nz      = Nz.begin();
      const_iterator        f       = F.begin();
      const_index_iterator  pbegin  = phi.begin();
      const_index_iterator  pend    = phi.end();
      const_index_iterator  p       = pbegin;

      for(;p!=pend; ++f,++p,++nx,++ny,++nz)
      {
        size_t i = p.i();
        size_t j = p.j();
        size_t k = p.k();

        size_t im1   = ( i ) ?  i - 1 : 0;
        size_t jm1   = ( j ) ?  j - 1 : 0;
        size_t km1   = ( k ) ?  k - 1 : 0;
        size_t ip1   = min( i + 1u, I - 1u );
        size_t jp1   = min( j + 1u, J - 1u );
        size_t kp1   = min( k + 1u, K - 1u );
        size_t i000  = ( k   * J + j )   * I + i;
        size_t im00  = ( k   * J + j )   * I + im1;
        size_t ip00  = ( k   * J + j )   * I + ip1;
        size_t i0m0  = ( k   * J + jm1 ) * I + i;
        size_t i0p0  = ( k   * J + jp1 ) * I + i;
        size_t i00m  = ( km1 * J + j )   * I + i;
        size_t i00p  = ( kp1 * J + j )   * I + i;
        real_type  v000  = phi(i000);
        real_type  vp00  = phi(ip00);
        real_type  vm00  = phi(im00);
        real_type  v0p0  = phi(i0p0);
        real_type  v0m0  = phi(i0m0);
        real_type  v00p  = phi(i00p);
        real_type  v00m  = phi(i00m);
        // phi^t+1  =   phi_t + dt*F
        //  F > 0 use forward diffs
        //  F < 0 use backward diffs
        if((*f)>0)
        {
          *nx = (vp00 - v000 ) * inv_dx;
          *ny = (v0p0 - v000 ) * inv_dy;
          *nz = (v00p - v000 ) * inv_dz;
        }
        else if((*f)<0)
        {
          *nx = (v000 - vm00) * inv_dx;
          *ny = (v000 - v0m0) * inv_dy;
          *nz = (v000 - v00m) * inv_dz;
        }
        else
        {
          *nx = (vp00 - vm00) * inv_2dx;
          *ny = (v0p0 - v0m0) * inv_2dy;
          *nz = (v00p - v00m) * inv_2dz;
        }
      }
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_UPWIND_GRADIENT_FIELD_H
#endif
