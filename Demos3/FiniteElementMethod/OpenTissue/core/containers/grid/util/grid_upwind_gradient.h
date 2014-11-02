#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_UPWIND_GRADIENT_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_UPWIND_GRADIENT_H
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
    * Compute Upwind Gradient at Specified Node location.
    *
    * @param phi        A the map in which we want to know the gradient.
    * @param F          A map containing the values of the speed function.
    * @param i          The i'th index of the node at which we want to compute the gradient.
    * @param j          The j'th index of the node at which we want to compute the gradient.
    * @param k          The k'th index of the node at which we want to compute the gradient.
    * @param gradient   Upon completion contains the upwind gradient
    */
    template<typename grid_type, typename vector3_type>
    inline void upwind_gradient(
      grid_type const & phi
      , grid_type const & F
      , size_t i
      , size_t j
      , size_t k
      , vector3_type & gradient 
      )
    {
      using std::min;

      typedef typename grid_type::value_type            value_type;
      typedef typename grid_type::math_types            math_types;
      typedef typename math_types::real_type            real_type;

      size_t I       = phi.I();
      size_t J       = phi.J();
      size_t K       = phi.K();
      real_type  inv_dx  = 1.0/phi.dx();
      real_type  inv_dy  = 1.0/phi.dy();
      real_type  inv_dz  = 1.0/phi.dz();
      real_type  inv_2dx = 0.5/phi.dx();
      real_type  inv_2dy = 0.5/phi.dy();
      real_type  inv_2dz = 0.5/phi.dz();

      size_t im1     = ( i ) ?  i - 1 : 0;
      size_t jm1     = ( j ) ?  j - 1 : 0;
      size_t km1     = ( k ) ?  k - 1 : 0;
      size_t ip1     = min( i + 1u, I - 1u );
      size_t jp1     = min( j + 1u, J - 1u );
      size_t kp1     = min( k + 1u, K - 1u );
      size_t i000    = ( k   * J + j )   * I + i;
      size_t im00    = ( k   * J + j )   * I + im1;
      size_t ip00    = ( k   * J + j )   * I + ip1;
      size_t i0m0    = ( k   * J + jm1 ) * I + i;
      size_t i0p0    = ( k   * J + jp1 ) * I + i;
      size_t i00m    = ( km1 * J + j )   * I + i;
      size_t i00p    = ( kp1 * J + j )   * I + i;
      real_type  f       = F(i000);
      real_type  v000    = phi(i000);
      real_type  vp00    = phi(ip00);
      real_type  vm00    = phi(im00);
      real_type  v0p0    = phi(i0p0);
      real_type  v0m0    = phi(i0m0);
      real_type  v00p    = phi(i00p);
      real_type  v00m    = phi(i00m);
      // phi^t+1  =   phi_t + dt*F
      //  F > 0 use forward diffs
      //  F < 0 use backward diffs
      real_type  nx      = 0;
      real_type  ny      = 0;
      real_type  nz      = 0;
      if(f>0)
      {
        nx = (vp00 - v000 ) * inv_dx;
        ny = (v0p0 - v000 ) * inv_dy;
        nz = (v00p - v000 ) * inv_dz;
      }
      else if(f<0)
      {
        nx = (v000 - vm00) * inv_dx;
        ny = (v000 - v0m0) * inv_dy;
        nz = (v000 - v00m) * inv_dz;
      }
      else
      {
        nx = (vp00 - vm00) * inv_2dx;
        ny = (v0p0 - v0m0) * inv_2dy;
        nz = (v00p - v00m) * inv_2dz;
      }
      gradient = vector3_type(nx,ny,nz);
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_UPWIND_GRADIENT_H
#endif
