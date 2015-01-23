#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_GRADIENT_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_GRADIENT_H
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

    template<typename grid_type, typename vector3_type>
    inline void gradient(
      grid_type const & grid
      , size_t i
      , size_t j
      , size_t k
      , vector3_type & gradient 
      )
    {
      using std::min;

      typedef typename grid_type::value_type            value_type;
      typedef typename vector3_type::value_type         real_type;

      static value_type const unused = grid.unused();

      size_t const & I = grid.I();
      size_t const & J = grid.J();
      size_t const & K = grid.K();

      size_t im1 = 0, jm1 = 0, km1 = 0;
      if ( i>0 )
        im1 = i - 1;
      if ( j>0 )
        jm1 = j - 1;
      if ( k>0 )
        km1 = k - 1;

      size_t ip1 = min( i + 1u, I - 1u );
      size_t jp1 = min( j + 1u, J - 1u );
      size_t kp1 = min( k + 1u, K - 1u );

      size_t idx     = ( k   * J + j )   * I + i;
      size_t idx_im1 = ( k   * J + j )   * I + im1;
      size_t idx_ip1 = ( k   * J + j )   * I + ip1;
      size_t idx_jm1 = ( k   * J + jm1 ) * I + i;
      size_t idx_jp1 = ( k   * J + jp1 ) * I + i;
      size_t idx_km1 = ( km1 * J + j )   * I + i;
      size_t idx_kp1 = ( kp1 * J + j )   * I + i;

      //--- Return Unused-vector if any of the values in the map is unused.
      gradient = vector3_type(unused,unused,unused);

      value_type s_idx = grid( idx );    
      if ( s_idx == unused )
        return;

      value_type s_im1 = grid( idx_im1 );
      if ( s_im1 == unused )
        return;

      value_type s_ip1 = grid( idx_ip1 );
      if ( s_ip1 == unused )
        return;

      value_type s_jm1 = grid( idx_jm1 );
      if ( s_jm1 == unused )
        return;

      value_type s_jp1 = grid( idx_jp1 );
      if ( s_jp1 == unused )
        return;

      value_type s_km1 = grid( idx_km1 );
      if ( s_km1 == unused )
        return;

      value_type s_kp1 = grid( idx_kp1 );
      if ( s_kp1 == unused )
        return;

      real_type r_idx = static_cast<real_type>( s_idx );
      real_type r_im1 = static_cast<real_type>( s_im1 );
      real_type r_ip1 = static_cast<real_type>( s_ip1 );
      real_type r_jm1 = static_cast<real_type>( s_jm1 );
      real_type r_jp1 = static_cast<real_type>( s_jp1 );
      real_type r_km1 = static_cast<real_type>( s_km1 );
      real_type r_kp1 = static_cast<real_type>( s_kp1 );

      if ( i == 0 )
      {
        gradient( 0 ) = ( r_ip1 - r_idx );
        gradient( 0 ) /= grid.dx();
      }
      else if ( i == ( grid.I() - 1 ) )
      {
        gradient( 0 ) = ( r_idx - r_im1 );
        gradient( 0 ) /= grid.dx();
      }
      else
      {
        gradient( 0 ) = ( r_ip1 - r_im1 );
        gradient( 0 ) /= ( 2 * grid.dx() );
      }
      if ( j == 0 )
      {
        gradient( 1 ) = ( r_jp1 - r_idx );
        gradient( 1 ) /= grid.dy();
      }
      else if ( j == ( grid.J() - 1 ) )
      {
        gradient( 1 ) = ( r_idx - r_jm1 );
        gradient( 1 ) /= grid.dy();
      }
      else
      {
        gradient( 1 ) = ( r_jp1 - r_jm1 );
        gradient( 1 ) /= ( 2 * grid.dy() );
      }

      if ( k == 0 )
      {
        gradient( 2 ) = ( r_kp1 - r_idx );
        gradient( 2 ) /= grid.dz();
      }
      else if ( k == ( grid.K() - 1 ) )
      {
        gradient( 2 ) = ( r_idx - r_km1 );
        gradient( 2 ) /= grid.dz();
      }
      else
      {
        gradient( 2 ) = ( r_kp1 - r_km1 );
        gradient( 2 ) /= ( 2 * grid.dz() );
      }
    }

    template<typename grid_iterator,typename vector3_type>
    inline void gradient (grid_iterator const & iter, vector3_type & gradient )
    {
      typedef typename grid_iterator::grid_type               grid_type;

      size_t       i   = iter.i();
      size_t       j   = iter.j();
      size_t       k   = iter.k();
      grid_type const & grid = iter.get_grid();
      gradient(grid, i, j, k, gradient);
    }

    template<typename grid_iterator>
    inline typename grid_iterator::math_types::vector3_type gradient (grid_iterator const & iter )
    {
      typedef typename grid_iterator::math_types::vector3_type  vector3_type;

      vector3_type gradient;

      gradient(iter, gradient);
      return gradient;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_GRADIENT_H
#endif
