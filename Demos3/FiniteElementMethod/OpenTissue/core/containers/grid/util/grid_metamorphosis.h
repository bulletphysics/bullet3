#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_METAMORPHOSIS_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_METAMORPHOSIS_H
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

    /**
    * Auxilliary function used by metamorphosis().
    *
    * This function compute the normal velocity flow.
    *
    *
    *
    * @param phi    The current surface
    * @param gamma  The inside-outside function of target surface
    * @param F      Upon return contains the normal velocity flow field.
    */
    template<
      typename grid_type
    >
    inline void metamorphosis_speed_function(
    grid_type const & phi
    , grid_type const & gamma
    , grid_type  & F
    )
    {
      using std::min;
      using std::max;
      using std::sqrt;

      typedef typename grid_type::value_type            value_type;
      typedef typename grid_type::iterator              iterator;
      typedef typename grid_type::index_iterator        index_iterator;
      typedef typename grid_type::const_index_iterator  const_index_iterator;
      typedef typename grid_type::math_types            math_types;
      typedef typename math_types::real_type            real_type;

      size_t I = phi.I();
      size_t J = phi.J();
      size_t K = phi.K();
      real_type dx = phi.dx();
      real_type dy = phi.dy();
      real_type dz = phi.dz();

      const_index_iterator end = phi.end();
      const_index_iterator   u = phi.begin();
      index_iterator         f = F.begin();
      const_index_iterator   g = gamma.begin();

      for(;u!=end; ++u,++g,++f)
      {
        size_t i = u.i();
        size_t j = u.j();
        size_t k = u.k();

        size_t im1         = ( i ) ?  i - 1 : 0;
        size_t jm1         = ( j ) ?  j - 1 : 0;
        size_t km1         = ( k ) ?  k - 1 : 0;
        size_t ip1         = min( i + 1u, I - 1u );
        size_t jp1         = min( j + 1u, J - 1u );
        size_t kp1         = min( k + 1u, K - 1u );
        size_t idx         = ( k   * J + j )   * I + i;
        size_t idx_im1     = ( k   * J + j )   * I + im1;
        size_t idx_ip1     = ( k   * J + j )   * I + ip1;
        size_t idx_jm1     = ( k   * J + jm1 ) * I + i;
        size_t idx_jp1     = ( k   * J + jp1 ) * I + i;
        size_t idx_km1     = ( km1 * J + j )   * I + i;
        size_t idx_kp1     = ( kp1 * J + j )   * I + i;
        real_type  u_idx      = phi(idx);
        real_type  u_idx_im1  = phi(idx_im1);
        real_type  u_idx_ip1  = phi(idx_ip1);
        real_type  u_idx_jm1  = phi(idx_jm1);
        real_type  u_idx_jp1  = phi(idx_jp1);
        real_type  u_idx_km1  = phi(idx_km1);
        real_type  u_idx_kp1  = phi(idx_kp1);

        real_type  Upx        = (u_idx_ip1 - u_idx    )/ dx;
        real_type  Upy        = (u_idx_jp1 - u_idx    )/ dy;
        real_type  Upz        = (u_idx_kp1 - u_idx    )/ dz;
        real_type  Umx        = (u_idx     - u_idx_im1)/ dx;
        real_type  Umy        = (u_idx     - u_idx_jm1)/ dy;
        real_type  Umz        = (u_idx     - u_idx_km1)/ dz;

        real_type Upx_min = min( Upx, 0.0);
        real_type Upx_max = max( Upx, 0.0);
        real_type Umx_min = min( Umx, 0.0);
        real_type Umx_max = max( Umx, 0.0);

        real_type Upy_min = min( Upy, 0.0);
        real_type Upy_max = max( Upy, 0.0);
        real_type Umy_min = min( Umy, 0.0);
        real_type Umy_max = max( Umy, 0.0);

        real_type Upz_min = min( Upz, 0.0);
        real_type Upz_max = max( Upz, 0.0);
        real_type Umz_min = min( Umz, 0.0);
        real_type Umz_max = max( Umz, 0.0);

        if(-(*g)>=0)
          *f = (*g)*sqrt(
          Upx_min*Upx_min
          + Umx_max*Umx_max
          + Upy_min*Upy_min
          + Umy_max*Umy_max
          + Upz_min*Upz_min
          + Umz_max*Umz_max
          );
        else
          *f = (*g)*sqrt(
          Upx_max*Upx_max
          + Umx_min*Umx_min
          + Upy_max*Upy_max
          + Umy_min*Umy_min
          + Upz_max*Upz_max
          + Umz_min*Umz_min
          );
      }
      //std::cout << "metamorphosis_speed_function(): done..." << std::endl;
    }


    /**
    * Solid Shape Metamorphosis.
    *
    *  Solve the PDE:
    *
    *    \frac{\partial \phi(x) }{  \partial t}  = | \nabla \phi(x)| \gamma(x)
    *
    * where \gamma usually is an inside-outside function of the target
    * object. Usually the signed distance map of the target is used as
    * \gamma. That is
    *
    *    \gamma (x)  = \phi_target (x)
    *
    * It could be defined more genrally as
    *
    *   \gamma (x)  = f ( \phi_target (x) )
    *
    * where the function f is used to control the speed of the moving
    * interface of phi.
    *
    *
    * The process is controlled by the user by defining the initial overlapping
    * regions of source and target.
    *
    *
    * The final metamorphosis strategy is
    *
    *    \frac{\partial \phi(x) }{  \partial t}  = | \nabla \phi(x)| \gamma( T(x,alpha) )
    *
    * Given the coordinate transform:
    *
    *   y = T(x,alpha)
    *
    * where x is a point in source and y is corresponding point in
    * target, and 0 \leq alpha \leq 1 is a parameterization such that
    *
    *   T(x,0) = x
    *   T(x,1) = y
    *
    * The transform T is for instance useful for alignment of the objects.
    *
    *
    * The flow is implemented using a upwind (Godonov) scheme. Currently
    * implementation do not support the T-transform. The user do have the
    * option of applying an f-function outside the function, since only
    * the gamma-field is surplied to this function.
    *
    *
    * @param phi      Input/output level set.
    * @param gamma    The inside/outside function of the target object.
    * @param dt       The time-step size.
    */
    template<
      typename grid_type
    >
    inline void metamorphosis(
    grid_type & phi
    , grid_type const & gamma
    , typename grid_type::math_types::real_type const & dt
    )
    {
      typedef typename grid_type::value_type            value_type;
      typedef typename grid_type::iterator              iterator;
      typedef typename grid_type::index_iterator        index_iterator;
      typedef typename grid_type::const_index_iterator  const_index_iterator;
      typedef typename grid_type::math_types            math_types;
      typedef typename math_types::real_type            real_type;

      using std::min;
      using std::max;
      using std::fabs;

      real_type dx = phi.dx();
      real_type dy = phi.dy();
      real_type dz = phi.dz();

      grid_type F = phi;

      real_type min_delta  = min ( dx, min( dy, dz));
      assert(min_delta>0 || !"metamorphosis(): minimum spacing was non-positive!");
      real_type max_gamma  = max ( fabs( max(gamma)), fabs( min(gamma) ) );
      assert(max_gamma>0 || !"metamorphosis(): maximum absolute speed was non-positive!");
      real_type time       = static_cast<real_type>(0.0);

      while (time < dt)
      {
        metamorphosis_speed_function(phi,gamma,F);

        real_type  time_step = min( dt,  min_delta / (3.0*max_gamma) );
        assert(time_step>0 || !"metamorphosis(): time_step was non-positive!");
        std::cout << "\ttime step = " << time_step << std::endl;

        index_iterator       end = phi.end();
        index_iterator         u = phi.begin();
        index_iterator         f = F.begin();
        for(;u!=end; ++u,++f)
          (*u) = (*u) + time_step *(*f);

        time += time_step;
      }
      std::cout << "metamorphosis(): done..." << std::endl;
    }

  } // namespace grid
} // namespace OpenTissue

//  OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_METAMORPHOSIS_H
#endif
