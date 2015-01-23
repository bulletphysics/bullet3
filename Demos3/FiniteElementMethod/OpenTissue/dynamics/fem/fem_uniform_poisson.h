#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_UNIFORM_POISSON_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_UNIFORM_POISSON_H
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
  namespace fem
  {
    namespace detail
    {

      /**
      *
      * @param begin
      * @param end
      * @param poisson
      */
      template<typename real_type, typename tetrahedron_iterator>
      inline void uniform_poisson(
        tetrahedron_iterator const & begin
        , tetrahedron_iterator const & end
        , real_type const & poisson
        )
      {
        assert(poisson>=0   || !"uniform_poisson() : Poisson ratio must be non-negative");
        assert(poisson<=0.5 || !"uniform_poisson() : Poisson ratio must not be larger than a half");

        for (tetrahedron_iterator T = begin;T!=end;++T)
          T->m_poisson = poisson;
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_UNIFORM_POISSON_H
#endif
