#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_CLEAR_STIFFNESS_ASSEMBLY_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_CLEAR_STIFFNESS_ASSEMBLY_H
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
      */
      template < typename node_type >
      inline void clear_stiffness_assembly_single(node_type* node)
      {
		  typedef typename node_type::matrix_iterator matrix_iterator;
          node->m_f0.clear();
          for (matrix_iterator Kij = node->Kbegin() ; Kij != node->Kend(); ++Kij )
            Kij->second.clear();
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_CLEAR_STIFFNESS_ASSEMBLY_H
#endif
