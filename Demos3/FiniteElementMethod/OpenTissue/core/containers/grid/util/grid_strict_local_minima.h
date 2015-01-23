#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_STRICT_GRID_LOCAL_MINIMA_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_STRICT_GRID_LOCAL_MINIMA_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_idx2coord.h>
#include <cmath>

namespace OpenTissue
{
  namespace grid
  {

    namespace detail
    {

      /**
      * Test if a specified node is a strict local minima.
      *
      * @param i
      * @param j
      * @param k
      * @param phi
      *
      * @return   True if node i,j,k is a local minima, otherwise false.
      */
      template < typename grid_type >
      inline bool is_strict_local_minima(
        size_t i
        , size_t j
        , size_t k
        , grid_type const & phi
        )
      {
        using std::min;
        static size_t idx[27];
        size_t I = phi.I();
        size_t J = phi.J();
        size_t K = phi.K();
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
        for(size_t i=0;i<27u;++i)
          if(  phi(idx[13]) >= phi(idx[i]) )
            return false;
        return true;
      }

    } // namespace detail

    /**
    * Extract local minima nodes.
    * Note that local minima may exist at non-nodal locations. This function
    * only considers nodal positions. Thus if you are looking for local minima
    * at sub-pixel accuracy, then you need another approach.
    *
    * @param phi   The map from where local minima nodes should be extracted from.
    * @param points   Upon return this container contains all the coordinates of all nodes that where local minima.
    */
    template < typename grid_type,typename point_container >
    inline void strict_local_minima_as_points(
      grid_type const & phi
      , point_container & points
      )
    {
      typedef typename point_container::value_type     vector3_type;
      typedef typename grid_type::const_index_iterator  const_index_iterator;
      typedef typename vector3_type::value_type        real_type;
      typedef typename grid_type::value_type            value_type;

      const_index_iterator  pend    = phi.end();
      const_index_iterator  p       = phi.begin();
      for(;p!=pend;++p)
      {
        if(detail::is_strict_local_minima(p.i(),p.j(),p.k(),phi))
        {
          vector3_type point;
          idx2coord(phi,p.i(),p.j(),p.k(),point);
          points.push_back(point);
        }
      }
    }

    /**
    * Extract Strict Local Minima as Points.
    *
    * @param phi
    * @param mask         A mask that can be used to mask-out nodes in phi, which is not
    *                     of interest. Positive values correspond to nodes that are allowed
    *                     to be classified as local minima.
    * @param points
    */
    template < typename grid_type,typename point_container >
    inline void strict_local_minima_as_points(
      grid_type const & phi
      , grid_type const & mask
      , point_container & points
      )
    {
      typedef typename point_container::value_type     vector3_type;
      typedef typename grid_type::const_index_iterator  const_index_iterator;
      typedef typename vector3_type::value_type        real_type;
      typedef typename grid_type::value_type            value_type;
      const_index_iterator  end     = phi.end();
      const_index_iterator  p       = phi.begin();
      const_index_iterator  m       = mask.begin();
      for(;p!=end;++p,++m)
      {
        if( (*m) <= 0  )
          continue;
        if(detail::is_strict_local_minima(p.i(),p.j(),p.k(),phi))
        {
          vector3_type point;
          idx2coord(phi,p.i(),p.j(),p.k(),point);
          points.push_back(point);
        }
      }
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_STRICT_GRID_LOCAL_MINIMA_H
#endif
