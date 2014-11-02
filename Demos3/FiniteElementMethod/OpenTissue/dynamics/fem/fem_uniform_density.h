#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_UNIFORM_DENSITY_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_UNIFORM_DENSITY_H
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
      * Set Uniform Density.
      *
      * @param begin
      * @param end
      * @param density
      */
      template<typename real_type, typename tetrahedron_iterator>
      inline void uniform_density(tetrahedron_iterator const & begin, tetrahedron_iterator const & end,real_type const & density)
      {
        assert(density>0 || !"uniform_density(): density must be positive");
        for (tetrahedron_iterator T = begin;T!=end;++T)
          T->m_density = density;
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_UNIFORM_DENSITY_H
#endif
