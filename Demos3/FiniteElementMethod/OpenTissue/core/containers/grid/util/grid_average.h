#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_AVERAGE_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_AVERAGE_H
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

    template < typename grid_type >
    inline void average( grid_type & phi )
    {
      using std::min;

      typedef typename grid_type::index_iterator        iterator;
      typedef typename grid_type::value_type            value_type;

      grid_type tmp = phi;

      size_t I = phi.I();
      size_t J = phi.J();
      size_t K = phi.K();

      iterator  pend    = phi.end();
      iterator  p       = phi.begin();
      iterator  t       = tmp.begin();

      for(;p!=pend;++p,++t)
      {
        size_t i = p.i();
        size_t j = p.j();
        size_t k = p.k();

        static size_t idx[27];
        size_t im1   = ( i ) ?  i - 1 : 0;
        size_t jm1   = ( j ) ?  j - 1 : 0;
        size_t km1   = ( k ) ?  k - 1 : 0;
        size_t ip1   = min( i + 1u, I - 1u );
        size_t jp1   = min( j + 1u, J - 1u );
        size_t kp1   = min( k + 1u, K - 1u );
        idx[0]  = ( kp1 * J + jp1 )   * I + im1;
        idx[1]  = ( k   * J + jp1 )   * I + im1;
        idx[2]  = ( km1 * J + jp1 )   * I + im1;
        idx[3]  = ( kp1 * J + j   )   * I + im1;
        idx[4]  = ( k   * J + j   )   * I + im1;
        idx[5]  = ( km1 * J + j   )   * I + im1;
        idx[6]  = ( kp1 * J + jm1 )   * I + im1;
        idx[7]  = ( k   * J + jm1 )   * I + im1;
        idx[8]  = ( km1 * J + jm1 )   * I + im1;
        idx[9]  = ( kp1 * J + jp1 )   * I + i;
        idx[10] = ( k   * J + jp1 )   * I + i;
        idx[11] = ( km1 * J + jp1 )   * I + i;
        idx[12] = ( kp1 * J + j   )   * I + i;
        idx[13] = ( k   * J + j   )   * I + i;
        idx[14] = ( km1 * J + j   )   * I + i;
        idx[15] = ( kp1 * J + jm1 )   * I + i;
        idx[16] = ( k   * J + jm1 )   * I + i;
        idx[17] = ( km1 * J + jm1 )   * I + i;
        idx[18] = ( kp1 * J + jp1 )   * I + ip1;
        idx[19] = ( k   * J + jp1 )   * I + ip1;
        idx[20] = ( km1 * J + jp1 )   * I + ip1;
        idx[21] = ( kp1 * J + j   )   * I + ip1;
        idx[22] = ( k   * J + j   )   * I + ip1;
        idx[23] = ( km1 * J + j   )   * I + ip1;
        idx[24] = ( kp1 * J + jm1 )   * I + ip1;
        idx[25] = ( k   * J + jm1 )   * I + ip1;
        idx[26] = ( km1 * J + jm1 )   * I + ip1;
        value_type avg = value_type(); //--- default constructed zero by standard!!!
        for(size_t i=0;i<27u;++i)
          avg += phi(idx[i]);
        avg /= value_type(25);
        *t = avg;
      }
      phi = tmp;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_AVERAGE_H
#endif
