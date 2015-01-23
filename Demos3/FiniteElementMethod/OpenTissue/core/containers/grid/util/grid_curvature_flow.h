#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_CURVATURE_FLOW_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_CURVATURE_FLOW_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_mean_curvature.h>

namespace OpenTissue
{
namespace grid
{

  /**
  * Calculate curvature flow.
  *
  * WARNING: Unexpected behavior if input and output level sets are the same.
  *
  *
  * @param phi      Input level set.
  * @param mu       Mean Curvature coefficient.
  * @param dt       Time-step to use in update.
  * @param psi      Output levelset. Note if input is a signed distance map
  *                 then output may not be a signed distance map, thus
  *                 you may need to redistance output levelset.
  */
  template<
      typename grid_type
    , typename real_type
  >
  inline void curvature_flow(
    grid_type const & phi
  , real_type const & mu
  , real_type const & dt
  , grid_type & psi
  )
  {
    using std::min;
    using std::max;
    using std::sqrt;

    typedef typename grid_type::value_type            value_type;
    typedef typename grid_type::iterator              iterator;
    typedef typename grid_type::index_iterator        index_iterator;
    typedef typename grid_type::const_index_iterator  const_index_iterator;

    assert(mu>0 || !"curvature_flow(): curvature coefficient must be positive");
    assert(dt>0 || !"curvature_flow(): time-step must be positive");

    size_t I    = phi.I();
    size_t J    = phi.J();
    size_t K    = phi.K();
    real_type dx    = static_cast<real_type> ( phi.dx() );
    real_type dy    = static_cast<real_type> ( phi.dy() );
    real_type dz    = static_cast<real_type> ( phi.dz() );
    real_type  m_inv_dx2 = static_cast<real_type> ( 1.0 / (dx*dx) );
    real_type  m_inv_dy2 = static_cast<real_type> ( 1.0 / (dy*dy) );
    real_type  m_inv_dz2 = static_cast<real_type> ( 1.0 / (dz*dz) );
    real_type  m_inv_4dxy = static_cast<real_type> ( 0.25 / (dx*dy) );
    real_type  m_inv_4dxz = static_cast<real_type> ( 0.25 / (dx*dz) );
    real_type  m_inv_4dyz = static_cast<real_type> ( 0.25 / (dy*dz) );

    real_type min_delta     = static_cast<real_type> (min(dx,min(dy,dz) ));

    real_type kappa_limit = static_cast<real_type> ( 1.0 / ( max(dx, max(dy,dz) ) ) );

    value_type zero = static_cast<value_type>(0.0);
    grid_type F = phi;          //--- speed function

    if(&psi != &phi)
      psi = phi;

    iterator       fbegin = F.begin();
    index_iterator sbegin = psi.begin();
    index_iterator send   = psi.end();

    real_type threshold = static_cast<real_type>(10e-15);  //--- small number used to avoid division by zero!!!

    real_type time = static_cast<real_type>(0.0);
    while (time < dt)
    {
      value_type  max_F = zero; //--- maximum speed, used to setup CFL condition
      iterator              f      = fbegin;
      index_iterator        s      = sbegin;
      for(;s!=send; ++s,++f)
      {
        size_t i = s.i();
        size_t j = s.j();
        size_t k = s.k();

        size_t im1         = ( i ) ?  i - 1 : 0;
        size_t jm1         = ( j ) ?  j - 1 : 0;
        size_t km1         = ( k ) ?  k - 1 : 0;
        size_t ip1         = min( i + 1, I - 1 );
        size_t jp1         = min( j + 1, J - 1 );
        size_t kp1         = min( k + 1, K - 1 );

        size_t idx_000 = ( k   * J + j )   * I + i;
        size_t idx_m00 = ( k   * J + j )   * I + im1;
        size_t idx_p00 = ( k   * J + j )   * I + ip1;
        size_t idx_0m0 = ( k   * J + jm1 ) * I + i;
        size_t idx_0p0 = ( k   * J + jp1 ) * I + i;
        size_t idx_00m = ( km1 * J + j )   * I + i;
        size_t idx_00p = ( kp1 * J + j )   * I + i;
        size_t idx_pp0 = ( k   * J + jp1 ) * I + ip1;
        size_t idx_mp0 = ( k   * J + jp1 ) * I + im1;
        size_t idx_pm0 = ( k   * J + jm1 ) * I + ip1;
        size_t idx_mm0 = ( k   * J + jm1 ) * I + im1;
        size_t idx_p0p = ( kp1 * J + j )   * I + ip1;
        size_t idx_m0p = ( kp1 * J + j )   * I + im1;
        size_t idx_p0m = ( km1 * J + j )   * I + ip1;
        size_t idx_m0m = ( km1 * J + j )   * I + im1;
        size_t idx_0pp = ( kp1 * J + jp1 ) * I + i;
        size_t idx_0pm = ( km1 * J + jp1 ) * I + i;
        size_t idx_0mp = ( kp1 * J + jm1 ) * I + i;
        size_t idx_0mm = ( km1 * J + jm1 ) * I + i;

        real_type d000 = 2.0 * psi( idx_000 );
        real_type dp00 = psi( idx_p00 );
        real_type dm00 = psi( idx_m00 );
        real_type d0p0 = psi( idx_0p0 );
        real_type d0m0 = psi( idx_0m0 );
        real_type d00p = psi( idx_00p );
        real_type d00m = psi( idx_00m );
        real_type dpp0 = psi( idx_pp0 );
        real_type dmp0 = psi( idx_mp0 );
        real_type dpm0 = psi( idx_pm0 );
        real_type dmm0 = psi( idx_mm0 );
        real_type dp0p = psi( idx_p0p );
        real_type dm0p = psi( idx_m0p );
        real_type dp0m = psi( idx_p0m );
        real_type dm0m = psi( idx_m0m );
        real_type d0pp = psi( idx_0pp );
        real_type d0pm = psi( idx_0pm );
        real_type d0mp = psi( idx_0mp );
        real_type d0mm = psi( idx_0mm );
        //---- Hessian matrix is defines as
        //
        //         | d/(dx*dx)   d(/(dx*dy)  d/(dx*dz) |
        //    H =  | d/(dy*dx)   d(/(dy*dy)  d/(dy*dz) |  phi
        //         | d/(dz*dx)   d(/(dz*dy)  d/(dz*dz) |
        //
        //
        //--- Following is a central diff approximation
        real_type   psi_x  = (dp00 - dm00) * m_inv_dx2;
        real_type   psi_y  = (d0p0 - d0m0) * m_inv_dy2;
        real_type   psi_z  = (d00p - d00m) * m_inv_dz2;     
        real_type   psi_xx = ( dp00 + dm00 - d000 ) * m_inv_dx2;
        real_type   psi_yy = ( d0p0 + d0m0 - d000 ) * m_inv_dy2;
        real_type   psi_zz = ( d00p + d00m - d000 ) * m_inv_dz2;
        real_type   psi_xy = ( dpp0 - dmp0 - dpm0 + dmm0 ) * m_inv_4dxy;
        real_type   psi_xz = ( dp0p - dm0p - dp0m + dm0m ) * m_inv_4dxz;
        real_type   psi_yz = ( d0pp - d0pm - d0mp + d0mm ) * m_inv_4dyz;
        real_type norm_grad_psi = sqrt( (psi_x*psi_x) + (psi_y*psi_y) + (psi_z*psi_z) );
        real_type kappa =  
          (
             (psi_x*psi_x)*psi_yy
          +  (psi_x*psi_x)*psi_zz
          +  (psi_y*psi_y)*psi_xx
          +  (psi_y*psi_y)*psi_zz
          +  (psi_z*psi_z)*psi_xx
          +  (psi_z*psi_z)*psi_yy
          -  2*(psi_x*psi_y)*psi_xy
          -  2*(psi_x*psi_z)*psi_xz
          -  2*(psi_y*psi_z)*psi_yz 
          )
          / 
          (norm_grad_psi*norm_grad_psi*norm_grad_psi  + threshold );

        kappa = max( kappa, -kappa_limit );
        kappa = min( kappa, kappa_limit );

        *f =  static_cast<value_type>(mu * kappa);
        max_F = max(max_F, std::fabs(*f));
      }
      real_type time_left = max( dt - time, 0.0 );
      if(time_left <= 0)
        return;

      value_type  time_step =  static_cast<value_type>( min( time_left,  min_delta / (max_F)  )  );

      std::cout << "\tcurvature_flow() : timestep = " << time_step << std::endl;

      for(s=sbegin, f=fbegin;s!=send;++s,++f)
        (*s) = (*s) + time_step * (*f);
      time += time_step;
    }
  }

} // namespace grid
} // namespace OpenTissue

//  OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_CURVATURE_FLOW_H
#endif
