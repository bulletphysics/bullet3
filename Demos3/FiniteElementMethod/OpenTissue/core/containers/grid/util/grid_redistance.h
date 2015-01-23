#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_REDISTANCE_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_REDISTANCE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>

#include <OpenTissue/core/math/math_is_finite.h>
#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/containers/grid/util/grid_compute_sign_function.h>

namespace OpenTissue
{
  namespace grid
  {

    namespace detail
    {
      class FullRedistance
      {
      protected:

        template < typename grid_type >
        typename grid_type::value_type compute_speed(
          grid_type const & phi
          , grid_type const & S0
          , grid_type & speed
          )
        {
          using std::min;
          using std::max;
          using std::sqrt;
          using std::fabs;

          typedef typename grid_type::const_index_iterator   const_index_iterator;
          typedef typename grid_type::const_iterator         const_iterator;
          typedef typename grid_type::iterator               iterator;
          typedef typename grid_type::math_types             math_types;
          typedef typename math_types::value_type            real_type;

          assert(phi.I()==S0.I() || !"compute_speed(): incompatible grid dimensions");
          assert(phi.J()==S0.J() || !"compute_speed(): incompatible grid dimensions");
          assert(phi.K()==S0.K() || !"compute_speed(): incompatible grid dimensions");
          assert(phi.min_coord()==S0.min_coord() || !"compute_speed(): incompatible grid side lengths");
          assert(phi.max_coord()==S0.max_coord() || !"compute_speed(): incompatible grid side lengths");

          speed.create(phi.min_coord(),phi.max_coord(),phi.I(),phi.J(),phi.K());

          real_type inv_dx = static_cast<real_type> ( 1.0 / phi.dx());
          real_type inv_dy = static_cast<real_type> ( 1.0 / phi.dy());
          real_type inv_dz = static_cast<real_type> ( 1.0 / phi.dz());

          real_type inv_dx2 = inv_dx*inv_dx;
          real_type inv_dy2 = inv_dx*inv_dy;
          real_type inv_dz2 = inv_dx*inv_dz;

          real_type cfl_condition = real_type(0.0);
          real_type zero = static_cast<real_type>(0.0);

          iterator             f   = speed.begin();
          const_index_iterator p   = phi.begin();
          const_index_iterator end = phi.end();
          const_iterator       s0  = S0.begin();

          size_t I = phi.I();
          size_t J = phi.J();
          size_t K = phi.K();

          for ( ; p != end; ++p, ++f,++s0)
          {
            size_t i = p.i();
            size_t j = p.j();
            size_t k = p.k();
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
            real_type d000 = phi( idx_000 );
            real_type dp00 = phi( idx_p00 );
            real_type dm00 = phi( idx_m00 );
            real_type d0p0 = phi( idx_0p0 );
            real_type d0m0 = phi( idx_0m0 );
            real_type d00p = phi( idx_00p );
            real_type d00m = phi( idx_00m );
            assert( is_finite( d000 ) || !"compute_speed(): NaN encountered");
            assert( is_finite( dp00 ) || !"compute_speed(): NaN encountered");
            assert( is_finite( dm00 ) || !"compute_speed(): NaN encountered");
            assert( is_finite( d0p0 ) || !"compute_speed(): NaN encountered");
            assert( is_finite( d0m0 ) || !"compute_speed(): NaN encountered");
            assert( is_finite( d00p ) || !"compute_speed(): NaN encountered");
            assert( is_finite( d00m ) || !"compute_speed(): NaN encountered");
            real_type dxp = (dp00 - d000)*inv_dx;
            real_type dxm = (d000 - dm00)*inv_dx;
            real_type dyp = (d0p0 - d000)*inv_dy;
            real_type dym = (d000 - d0m0)*inv_dy;
            real_type dzp = (d00p - d000)*inv_dz;
            real_type dzm = (d000 - d00m)*inv_dz;
            assert( is_finite( dxp ) || !"compute_speed(): NaN encountered");
            assert( is_finite( dxm ) || !"compute_speed(): NaN encountered");
            assert( is_finite( dyp ) || !"compute_speed(): NaN encountered");
            assert( is_finite( dym ) || !"compute_speed(): NaN encountered");
            assert( is_finite( dzp ) || !"compute_speed(): NaN encountered");
            assert( is_finite( dzm ) || !"compute_speed(): NaN encountered");
            real_type dxp_max = max( dxp, zero );
            real_type dxm_max = max( dxm, zero );
            real_type dyp_max = max( dyp, zero );
            real_type dym_max = max( dym, zero );
            real_type dzp_max = max( dzp, zero );
            real_type dzm_max = max( dzm, zero );
            real_type dxp_min = min( dxp, zero );
            real_type dxm_min = min( dxm, zero );
            real_type dyp_min = min( dyp, zero );
            real_type dym_min = min( dym, zero );
            real_type dzp_min = min( dzp, zero );
            real_type dzm_min = min( dzm, zero );
            real_type dxp_max_2 = dxp_max * dxp_max;
            real_type dxm_max_2 = dxm_max * dxm_max;
            real_type dyp_max_2 = dyp_max * dyp_max;
            real_type dym_max_2 = dym_max * dym_max;
            real_type dzp_max_2 = dzp_max * dzp_max;
            real_type dzm_max_2 = dzm_max * dzm_max;
            real_type dxp_min_2 = dxp_min * dxp_min;
            real_type dxm_min_2 = dxm_min * dxm_min;
            real_type dyp_min_2 = dyp_min * dyp_min;
            real_type dym_min_2 = dym_min * dym_min;
            real_type dzp_min_2 = dzp_min * dzp_min;
            real_type dzm_min_2 = dzm_min * dzm_min;
            // yellowbook p58
            real_type phi_x_2_p = max( dxm_max_2, dxp_min_2 );
            real_type phi_x_2_m = max( dxm_min_2, dxp_max_2 );
            real_type phi_y_2_p = max( dym_max_2, dyp_min_2 );
            real_type phi_y_2_m = max( dym_min_2, dyp_max_2 );
            real_type phi_z_2_p = max( dzm_max_2, dzp_min_2 );
            real_type phi_z_2_m = max( dzm_min_2, dzp_max_2 );
            // peng99 eq40
            real_type norm_grad_phi_p = sqrt( phi_x_2_p + phi_y_2_p + phi_z_2_p );
            real_type norm_grad_phi_m = sqrt( phi_x_2_m + phi_y_2_m + phi_z_2_m );
            // Godunov scheme (yellowbook p58 eq6.3 and 6.4)
            if ( (*s0) > 0 )
            {
              (*f) = (*s0) * ( norm_grad_phi_p - 1.0 );
            }
            else if ( (*s0) < 0 )
            {
              (*f) = (*s0) * ( norm_grad_phi_m - 1.0 );
            }
            else
            {
              (*f) = 0;
            }
            real_type phi_x = fabs((dp00 - dm00)*inv_dx*.5);
            real_type phi_y = fabs((d0p0 - d0m0)*inv_dy*.5);
            real_type phi_z = fabs((d00p - d00m)*inv_dz*.5);
            real_type norm_grad_phi = sqrt(phi_x*phi_x + phi_y*phi_y + phi_z*phi_z);

            real_type tmp =  fabs(  (*s0)*phi_x*inv_dx2/norm_grad_phi + (*s0)*phi_y*inv_dy2/norm_grad_phi + (*s0)*phi_z*inv_dz2/norm_grad_phi  );
            cfl_condition = max( cfl_condition,  tmp );
          }
          return 1.0/cfl_condition;
        }

        template <       typename grid_type   , typename real_type   >
        real_type update(
          grid_type const & phi
          , grid_type const & speed
          , real_type const & time_step
          , grid_type & psi
          , real_type const & gamma = 10e30 
          )
        {
          typedef typename grid_type::const_index_iterator   const_iterator;
          typedef typename grid_type::iterator               iterator;

          assert(phi.I()==psi.I()                 || !"update(): incompatible grid dimensions");
          assert(phi.J()==psi.J()                 || !"update(): incompatible grid dimensions");
          assert(phi.K()==psi.K()                 || !"update(): incompatible grid dimensions");
          assert(phi.min_coord()==psi.min_coord() || !"update(): incompatible grid side lengths");
          assert(phi.max_coord()==psi.max_coord() || !"update(): incompatible grid side lengths");
          assert(time_step>0                      || !"update(): time-step must be positive");
          
          real_type steady = real_type(0.0);
          const_iterator i   = phi.begin();
          const_iterator f   = speed.begin();
          iterator       o   = psi.begin();
          const_iterator end = phi.end();
          for ( ; i!=end;++i, ++o, ++f)
          {
            real_type diff  = time_step * (*f);
            (*o) = (*i) - diff;
            diff = std::fabs(diff);
            if ( (*o) < gamma &&    steady < diff )
              steady = diff;
          }
          return steady;
        }

      public:

        /**
        * Signed Distance Map Reinitialization.
        *
        * Brute-force implementation of the reinitialization of the distance function.
        * The following Hamilton-Jacobi equation is solved:
        *   d_tau + S(d) * ( |grad phi| - 1 ) = 0
        *   d(x,0) = d_0(x) = phi(x,t)
        * to steady state, with S(d) approximated by:
        *   s = d / ( sqrt( d^2 + |Dd|^2 * delta_x^2) )
        *
        * Calculates gradient magnitude of phi using upwind-scheme.
        * Performs PDE update using forward Euler time discretization.
        *
        * @param phi              Input level set that should be redistanced into a signed distance grid.
        * @param psi              Output level set. That is the redistaned phi.
        * @param max_iterations   The maximum number of iterations allowed to do re-initialization.
        * @param stead_threshold  The threshold value used to test for steady state.
        */
        template < typename grid_type >
        void operator()(
          grid_type const & phi
          , grid_type & psi
          , size_t max_iterations = 10
          , double steady_threshold = 0.05
          )
        {
          using std::min;
          using std::max;

          typedef typename grid_type::value_type       real_type;

          assert(phi.I()==psi.I() || !"operator(): incompatible grid dimensions");
          assert(phi.J()==psi.J() || !"operator(): incompatible grid dimensions");
          assert(phi.K()==psi.K() || !"operator(): incompatible grid dimensions");
          assert(phi.min_coord()==psi.min_coord() || !"operator(): incompatible grid side lengths");
          assert(phi.max_coord()==psi.max_coord() || !"operator(): incompatible grid side lengths");

          static grid_type S0;       // TODO: Hmm, these temporaries take up a lot of space!!!
          static grid_type speed;

          compute_sign_function(phi,S0);

          real_type alpha = static_cast<real_type> ( 0.9 );  //--- CFL number 
          //real_type min_delta = static_cast<real_type> ( (min( phi.dx(), min( phi.dy(), phi.dz() ) ) ) );
          real_type max_delta = static_cast<real_type> ( (max( phi.dx(), max( phi.dy(), phi.dz() ) ) ) );

          grid_type *  phi_in = const_cast<grid_type*>(&phi);
          grid_type * phi_out = &psi;

          real_type gamma = static_cast<real_type>( max_delta * max_iterations );
          real_type steady = static_cast<real_type>(0.0);

          for(size_t iterations=0;iterations<max_iterations;++iterations)
          {
            for ( size_t flipflop = 2; flipflop; --flipflop )
            {
              std::swap( phi_in, phi_out );
              real_type cfl_condition = compute_speed( (*phi_in) ,S0,speed);

              //--- CFL condition (yellowbook p50)
              //---
              //--- Given PDE:
              //---
              //---  phi_t + H(\nabla \phi) = 0
              //---
              //---  CFL condition is
              //---
              //---   \delta t \max \left{   
              //---                    \frac{|H_{\phi_x}|}{delta_x^2}
              //---                    +
              //---                    \frac{|H_{\phi_y}|}{delta_y^2}
              //---                    + 
              //---                    \frac{|H_{\phi_z}|}{delta_z^2}
              //---               \right} < alpha
              //---
              //---  where 0 < alpha < 1 is the CFL number (see pp 30 eq. 3.9). From this we pick the time step \delta t as 
              //---
              //---   delta_t =  alpha / 
              //---                  \left{   
              //---                    \frac{|H_{\phi_x}|}{delta_x^2}
              //---                    +
              //---                    \frac{|H_{\phi_y}|}{delta_y^2}
              //---                    + 
              //---                    \frac{|H_{\phi_z}|}{delta_z^2}
              //---               \right}            
              //---
              //---  In our case
              //---
              //---     H = s(|\nabla \phi| -1 )
              //---
              //---  So
              //---
              //---     H_{\phi_x} = \frac{ s \phi_x }{ |\nabla \phi| }
              //---
              //---  And we find
              //---
              //---   delta_t =  alpha / 
              //---                  \left{   
              //---                    \frac{|s \phi_x}|}{|\nabla \phi| delta_x^2}
              //---                    +
              //---                    \frac{|s \phi_y}|}{|\nabla \phi| delta_y^2}
              //---                    + 
              //---                    \frac{|s \phi_z}|}{|\nabla \phi| delta_z^2}
              //---               \right}            
              //---
              //---
              real_type time_step = alpha*cfl_condition;
              std::cout << "\ttime step = " << time_step << std::endl;

              steady = update( (*phi_in), speed, time_step, (*phi_out), gamma);
            }
            std::cout << "\tredistance(): iteration = " << iterations << " steady = " << steady <<std::endl;
            if(steady < steady_threshold)
              break;
          }
        }
      };

    }// namespace detail

    /**
    * Signed Distance Map Reinitialization.
    *
    * Brute-force implementation of the reinitialization of the distance function.
    * The following Hamilton-Jacobi equation is solved:
    *   d_tau + S(d) * ( |grad phi| - 1 ) = 0
    *   d(x,0) = d_0(x) = phi(x,t)
    * to steady state, with S(d) approximated by:
    *   s = d / ( sqrt( d^2 + |Dd|^2 * delta_x^2) )
    *
    * Calculates gradient magnitude of phi using upwind-scheme.
    * Performs PDE update using forward Euler time discretization.
    *
    * @param phi              Input level set that should be redistanced into a signed distance grid.
    * @param psi              Output level set. That is the redistanced phi.
    * @param max_iterations   The maximum number of iterations allowed to do re-initialization.
    * @param stead_threshold  The threshold value used to test for steady state.
    */
    template < typename grid_type >
    inline void redistance(
      grid_type const & phi
      , grid_type & psi
      , size_t max_iterations = 10
      , double steady_threshold = 0.05
      )
    {
      static detail::FullRedistance redistance_class;
      redistance_class(phi,psi,max_iterations,steady_threshold);
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_REDISTANCE_H
#endif
