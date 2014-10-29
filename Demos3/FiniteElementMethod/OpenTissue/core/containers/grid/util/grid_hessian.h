#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_HESSIAN_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_HESSIAN_H
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
  namespace grid
  {

    template<typename grid_type, typename matrix3x3_type>
    inline void hessian(
      grid_type const & grid
      , size_t i
      , size_t j
      , size_t k
      , matrix3x3_type & H 
      )
    {
      using std::min;

      typedef typename matrix3x3_type::value_type real_type;

      static grid_type * old_grid = 0;
      static real_type m_inv_dx2 = 0;        ///< Pre-computed value of 1./grid.dx()*grid.dx()
      static real_type m_inv_dy2 = 0;        ///< Pre-computed value of 1./grid.dy()*grid.dy()
      static real_type m_inv_dz2 = 0;        ///< Pre-computed value of 1./grid.dz()*grid.dz()
      static real_type m_inv_4dxy = 0;       ///< Pre-computed value of 1./4*grid.dx()*grid.dy()
      static real_type m_inv_4dxz = 0;       ///< Pre-computed value of 1./4*grid.dx()*grid.dz()
      static real_type m_inv_4dyz = 0;       ///< Pre-computed value of 1./4*grid.dy()*grid.dz()

      if(old_grid!=&grid)
      {
        old_grid = const_cast<grid_type*>(&grid);
        m_inv_dx2 = 1.0 / grid.dx() * grid.dx();
        m_inv_dy2 = 1.0 / grid.dy() * grid.dy();
        m_inv_dz2 = 1.0 / grid.dz() * grid.dz();
        m_inv_4dxy = 0.25 / grid.dx() * grid.dy();
        m_inv_4dxz = 0.25 / grid.dx() * grid.dz();
        m_inv_4dyz = 0.25 / grid.dy() * grid.dz();
      }

      size_t I = grid.I();
      size_t J = grid.J();
      size_t K = grid.K();

      size_t im1 = 0, jm1 = 0, km1 = 0;
      if ( i )
        im1 = i - 1;
      if ( j )
        jm1 = j - 1;
      if ( k )
        km1 = k - 1;
      size_t ip1 = min( i + 1u, I - 1u );
      size_t jp1 = min( j + 1u, J - 1u );
      size_t kp1 = min( k + 1u, K - 1u );

      size_t idx_000 = ( k   * J + j )   * I + i;
      size_t idx_m00 = ( k   * J + j )   * I + im1;
      size_t idx_p00 = ( k   * J + j )   * I + ip1;
      size_t idx_0m0 = ( k   * J + jm1 ) * I + i;
      size_t idx_0p0 = ( k   * J + jp1 ) * I + i;
      size_t idx_00m = ( km1 * J + j )   * I + i;
      size_t idx_00p = ( kp1 * J + j )   * I + i;

      real_type d000 = 2.0 * grid( idx_000 );
      real_type dp00 = grid( idx_p00 );
      real_type dm00 = grid( idx_m00 );
      real_type d0p0 = grid( idx_0p0 );
      real_type d0m0 = grid( idx_0m0 );
      real_type d00p = grid( idx_00p );
      real_type d00m = grid( idx_00m );

      size_t idx_pp0 = ( k   * J + jp1 ) * I + ip1;
      size_t idx_mp0 = ( k   * J + jp1 ) * I + im1;
      size_t idx_pm0 = ( k   * J + jm1 ) * I + ip1;
      size_t idx_mm0 = ( k   * J + jm1 ) * I + im1;
      size_t idx_p0p = ( kp1 * J + j )   * I + ip1;
      size_t idx_m0p = ( kp1 * J + j )   * I + im1;

      real_type dpp0 = grid( idx_pp0 );
      real_type dmp0 = grid( idx_mp0 );
      real_type dpm0 = grid( idx_pm0 );
      real_type dmm0 = grid( idx_mm0 );
      real_type dp0p = grid( idx_p0p );
      real_type dm0p = grid( idx_m0p );

      size_t idx_p0m = ( km1 * J + j )   * I + ip1;
      size_t idx_m0m = ( km1 * J + j )   * I + im1;
      size_t idx_0pp = ( kp1 * J + jp1 ) * I + i;
      size_t idx_0pm = ( km1 * J + jp1 ) * I + i;
      size_t idx_0mp = ( kp1 * J + jm1 ) * I + i;
      size_t idx_0mm = ( km1 * J + jm1 ) * I + i;

      real_type dp0m = grid( idx_p0m );
      real_type dm0m = grid( idx_m0m );
      real_type d0pp = grid( idx_0pp );
      real_type d0pm = grid( idx_0pm );
      real_type d0mp = grid( idx_0mp );
      real_type d0mm = grid( idx_0mm );
      //---- Hessian matrix is defines as
      //
      //         | d/(dx*dx)   d(/(dx*dy)  d/(dx*dz) |
      //    H =  | d/(dy*dx)   d(/(dy*dy)  d/(dy*dz) |  phi
      //         | d/(dz*dx)   d(/(dz*dy)  d/(dz*dz) |
      //
      //
      //--- Following is a central diff approximation
      H( 0, 0 ) = ( dp00 + dm00 - d000 ) * m_inv_dx2;
      H( 1, 1 ) = ( d0p0 + d0m0 - d000 ) * m_inv_dy2;
      H( 2, 2 ) = ( d00p + d00m - d000 ) * m_inv_dz2;
      H( 1, 0 ) = H( 0, 1 ) = ( dpp0 - dmp0 - dpm0 + dmm0 ) * m_inv_4dxy;
      H( 0, 2 ) = H( 2, 0 ) = ( dp0p - dm0p - dp0m + dm0m ) * m_inv_4dxz;
      H( 1, 2 ) = H( 2, 1 ) = ( d0pp - d0pm - d0mp + d0mm ) * m_inv_4dyz;
    }

    template<typename grid_iterator,typename matrix3x3_type>
    inline void hessian (grid_iterator const & iter, matrix3x3_type & H )
    {
      typedef typename grid_iterator::grid_type                 grid_type;

      size_t       i   = iter.i();
      size_t       j   = iter.j();
      size_t       k   = iter.k();

      grid_type const & grid = iter.get_grid();

      hessian(grid, i, j, k, H);
    }


    template<typename grid_iterator>
    inline OpenTissue::math::Matrix3x3< typename grid_iterator::math_types::real_type >    
      hessian (grid_iterator const & iter )
    {
      typedef typename grid_iterator::math_types::real_type  real_type;
      typedef OpenTissue::math::Matrix3x3< real_type >      matrix3x3_type;

      matrix3x3_type H;

      hessian(iter, H);

      return H;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_HESSIAN_H
#endif
