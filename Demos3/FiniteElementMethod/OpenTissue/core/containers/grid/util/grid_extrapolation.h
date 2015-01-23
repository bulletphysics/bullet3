#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_EXTRAPOLATION_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_EXTRAPOLATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>

#include <OpenTissue/core/containers/grid/util/gradient.h>
#include <OpenTissue/core/containers/grid/util/grid_compute_sign_function.h>
#include <OpenTissue/core/math/math_vector3.h>

namespace OpenTissue
{
  namespace grid
  {

    /**
    *
    * @param max_iterations   The maximum number of iterations allowed to do extrapolation.
    * @param stead_threshold  The threshold value used to test for steady state.
    */
    template < typename grid_type >
    inline void extrapolation(
      grid_type & S
      , grid_type const & phi
      , size_t max_iterations = 10
      //    , double steady_threshold = 0.05
      )
    {
      using std::min;
      using std::max;
      using std::fabs;

      typedef OpenTissue::math::Vector3< typename grid_type::value_type>  vector3_type;

      typedef typename grid_type::iterator              iterator;
      typedef typename grid_type::index_iterator        index_iterator;
      typedef typename grid_type::const_index_iterator  const_index_iterator;
      typedef typename grid_type::value_type            value_type;
      typedef typename grid_type::math_types            math_types;
      typedef typename math_types::real_type            real_type;

      Gradient<vector3_type> gradient;

      size_t I = S.I();
      size_t J = S.J();
      size_t K = S.K();

      real_type dx = S.dx();
      real_type dy = S.dy();
      real_type dz = S.dz();

      value_type zero = static_cast<value_type>(0.0);

      grid_type F = S;          //--- speed function
      grid_type sign_phi;       //--- sign function
      compute_sign_function(phi,sign_phi);

      iterator       fbegin = F.begin();
      index_iterator sbegin = S.begin();
      index_iterator send   = S.end();

      size_t iteration = 0;
      bool steady_state = false;

      while (!steady_state)
      {
        value_type  max_F = zero; //--- maximum speed, used to setup CFL condition
        const_index_iterator  p      = phi.begin();
        iterator              s_phi  = sign_phi.begin();
        iterator              f      = fbegin;
        index_iterator        s      = sbegin;
        for(;s!=send; ++s,++f,++s_phi,++p)
        {
          size_t i = s.i();
          size_t j = s.j();
          size_t k = s.k();

          vector3_type n = gradient(p);

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
          value_type  s_idx      = S(idx);
          value_type  s_idx_im1  = S(idx_im1);
          value_type  s_idx_ip1  = S(idx_ip1);
          value_type  s_idx_jm1  = S(idx_jm1);
          value_type  s_idx_jp1  = S(idx_jp1);
          value_type  s_idx_km1  = S(idx_km1);
          value_type  s_idx_kp1  = S(idx_kp1);
          value_type  Spx        = (s_idx_ip1 - s_idx    )/ dx;
          value_type  Spy        = (s_idx_jp1 - s_idx    )/ dy;
          value_type  Spz        = (s_idx_kp1 - s_idx    )/ dz;
          value_type  Smx        = (s_idx     - s_idx_im1)/ dx;
          value_type  Smy        = (s_idx     - s_idx_jm1)/ dy;
          value_type  Smz        = (s_idx     - s_idx_km1)/ dz;

          *f =  max( (*s_phi)*n(0) , zero)*Smx 
            +  min( (*s_phi)*n(0) , zero)*Spx 
            +  max( (*s_phi)*n(1) , zero)*Smy 
            +  min( (*s_phi)*n(1) , zero)*Spy
            +  max( (*s_phi)*n(2) , zero)*Smz 
            +  min( (*s_phi)*n(2) , zero)*Spz;

          max_F = max(max_F, fabs(*f));
        }
        value_type  time_step = static_cast<value_type>(  1.0 / (max_F + 1.0)  );
        for(s=sbegin, f=fbegin;s!=send;++s,++f)
          (*s) = (*s) - time_step * (*f);
        ++iteration;
        if(iteration> max_iterations)
          steady_state = true;
      }
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_EXTRAPOLATION_H
#endif
