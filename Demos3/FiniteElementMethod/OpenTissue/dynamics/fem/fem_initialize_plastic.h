#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_INITIALIZE_PLASTIC_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_INITIALIZE_PLASTIC_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/fem/fem_compute_b.h>
#include <OpenTissue/dynamics/fem/fem_compute_isotropic_elasticity.h>

namespace OpenTissue
{
  namespace fem
  {
    namespace detail
    {
      /**
      * Initialize plasticity material parameters.
      *
      * Note: Initialize_stiffness_elements must have been invoked prior
      * to calling this function.
      *
      * @param begin      Iterator to first tetrahedron.
      * @param end        Iterator to one past last tetrahedron.
      * @param c_yield    Plastic yield.
      * @param c_creep    Plastic creep.
      * @param c_max      Plastic max.
      */
      template < typename tetrahedron_type,typename real_type >
      inline void initialize_plastic_single(
        tetrahedron_type* T
        , real_type const & c_yield
        , real_type const & c_creep
        , real_type const & c_max
        )
      {
        assert(c_yield>=0 || !"initialize_plastic(): yield must be non-negative");
        assert(c_creep>=0 || !"initialize_plastic(): creep must be non-negative");
        assert(c_max>=0   || !"initialize_plastic(): max must be non-negative");

      T->m_yield = c_yield;
      T->m_creep = c_creep;
      T->m_max = c_max;
      for(int i=0;i<6;++i)
        T->m_plastic[i] = 0;

      compute_B(T->m_e10, T->m_e20, T->m_e30, T->m_V, T->m_B);
      compute_isotropic_elasticity_vector (T->m_young, T->m_poisson, T->m_D);
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_INITIALIZE_PLASTIC_H
#endif
