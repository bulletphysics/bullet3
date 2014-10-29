#ifndef OPENTISSUE_DYNAMICS_DEFORMATION_FEM_STIFFNESSWARPING_H
#define OPENTISSUE_DYNAMICS_DEFORMATION_FEM_STIFFNESSWARPING_H
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
      template <typename math_types>
      class TetrahedronTraits
      {
      public:

        typedef typename math_types::real_type         real_type;
        typedef typename math_types::vector3_type      vector3_type;
        typedef typename math_types::matrix3x3_type    matrix3x3_type;

      public:

        real_type m_young;
        real_type m_poisson;
        real_type m_density;

        matrix3x3_type m_Ke[4][4];  ///< Stiffness element matrix
        matrix3x3_type m_Re;        ///< Rotational warp of tetrahedron.
        real_type      m_V;         ///< Volume of tetrahedron

        vector3_type m_e10;         ///< edge from p0 to p1
        vector3_type m_e20;         ///< edge from p0 to p2
        vector3_type m_e30;         ///< edge from p0 to p3

        //--- Stuff used exclusive by plastic effects

        vector3_type m_B[4];        ///< placeholders for Jacobian of shapefunctions: B = SN.
        vector3_type m_D;           ///< Elasticity Matrix in vector from
        real_type    m_plastic[6];  ///< Plastic strain tensor.
        real_type    m_yield;
        real_type    m_creep;
        real_type    m_max;
      };

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_DEFORMATION_FEM_STIFFNESSWARPING_H
#endif
