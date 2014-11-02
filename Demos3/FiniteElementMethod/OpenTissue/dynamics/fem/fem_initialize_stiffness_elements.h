#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_INITIALIZE_STIFFNESS_ELEMENTS_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_INITIALIZE_STIFFNESS_ELEMENTS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/fem/fem_compute_ke.h>
#include <OpenTissue/dynamics/fem/fem_compute_volume.h>

namespace OpenTissue
{
  namespace fem
  {
    namespace detail
    {
      /**
      *
      *
      * NOTE: Material parameters (young modulus and poisson ratio) must have
      * been set prior to invoking this function.
      *
      * @param begin
      * @param end
      */
      template < typename tetrahedron_type >
      inline void initialize_stiffness_elements_single(tetrahedron_type* T)
      {
        {
          T->m_e10 = T->j()->m_model_coord - T->i()->m_model_coord;
          T->m_e20 = T->k()->m_model_coord - T->i()->m_model_coord;
          T->m_e30 = T->m()->m_model_coord - T->i()->m_model_coord;

          T->m_V = compute_volume(T->m_e10, T->m_e20, T->m_e30);

          //T->m_V = compute_volume(
          //  T->i()->m_model_coord,
          //  T->j()->m_model_coord,
          //  T->k()->m_model_coord,
          //  T->m()->m_model_coord
          //  );

          assert(T->m_V>0 || !"initialize_stiffness_elements(): Element with negative volume is encountered! Maybe you got a bad mesh?");

          T->m_Re = math::diag(1.0);

          //compute_Ke(T->m_B, T->m_D, T->m_V, T->m_Ke);
          compute_Ke(
            T->node(0)->m_model_coord,
            T->node(1)->m_model_coord,
            T->node(2)->m_model_coord,
            T->node(3)->m_model_coord,
            T->m_young, T->m_poisson,
            T->m_Ke
            );
        }
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_INITIALIZE_STIFFNESS_ELEMENTS_H
#endif
