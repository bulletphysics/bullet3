#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_COMPUTE_SIGN_FUNCTION_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_COMPUTE_SIGN_FUNCTION_H
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

    template < typename grid_type  >
    inline void compute_sign_function( grid_type const & phi, grid_type & S0 )
    {
      using std::sqrt;

      typedef typename grid_type::value_type             real_type;
      typedef typename grid_type::const_index_iterator   const_iterator;
      typedef typename grid_type::index_iterator         iterator;

      S0.create(phi.min_coord(),phi.max_coord(),phi.I(),phi.J(),phi.K());
      // yellowbook p67.

      real_type delta = phi.dx()*phi.dx()  + phi.dy()*phi.dy() + phi.dz()*phi.dz();
      const_iterator begin = phi.begin();
      const_iterator end   = phi.end();

      for ( const_iterator idx = begin;idx!=end;++idx)
      {
        size_t i = idx.i();
        size_t j = idx.j();
        size_t k = idx.k();
        real_type c = phi( i,j,k );
        S0( i,j,k ) = c / ( sqrt( c * c + delta  ) );//---TODO: Verify this formula!!!
      }
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_COMPUTE_SIGN_FUNCTION_H
#endif
