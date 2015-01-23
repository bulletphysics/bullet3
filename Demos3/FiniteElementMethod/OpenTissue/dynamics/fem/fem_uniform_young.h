#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_UNIFORM_YOUNG_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_UNIFORM_YOUNG_H
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
      * @param young
      */
      template<typename real_type, typename tetrahedron_iterator>
      inline void uniform_young(
        tetrahedron_iterator const & begin
        , tetrahedron_iterator const & end
        ,real_type const & young
        )
      {
        assert(young>=0 || !"uniform_young(): Young modulus must not be negative");

        for (tetrahedron_iterator T = begin;T!=end;++T)
          T->m_young = young;
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_UNIFORM_YOUNG_H
#endif
