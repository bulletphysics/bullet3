#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_NODE_TRAITS_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_NODE_TRAITS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <map>

namespace OpenTissue
{
  namespace fem
  {
    namespace detail
    {

      template <typename math_types>
      class NodeTraits
      {
      public:

        typedef typename math_types::real_type              real_type;
        typedef typename math_types::vector3_type           vector3_type;
        typedef typename math_types::matrix3x3_type         matrix3x3_type;

        typedef typename std::map<int,matrix3x3_type>  matrix_container;
        typedef typename matrix_container::iterator    matrix_iterator;

      public:

	  NodeTraits(int i,int j)
	  {
	  }
        matrix_container m_K_row;      ///< Currently stored in a map container, key correspond to column and mapped value to 3-by-3 sub block.
        matrix_container m_A_row;
        vector3_type     m_f0;
        vector3_type     m_b;

        bool m_fixed;    ///< If the node is in-moveable this flag is set to true otherwise it is false.

        vector3_type m_model_coord;
        vector3_type m_coord;            ///< World coord

        vector3_type m_velocity;
        real_type    m_mass;
        vector3_type m_f_external;

        // Needed by the ConjugateGradient method
        vector3_type m_update;
        vector3_type m_prev;
        vector3_type m_residual;

      public:

        matrix_iterator Kbegin() 
		{ 
			return m_K_row.begin(); 
		}
        matrix_iterator Kend()   { return m_K_row.end();   }
        matrix_iterator Abegin() { return m_A_row.begin(); }
        matrix_iterator Aend()   { return m_A_row.end();   }

        matrix3x3_type & K(int const & column_idx) { 
			return m_K_row[column_idx]; 
		}
        matrix3x3_type & A(int const & column_idx) { return m_A_row[column_idx]; }

      public:

        NodeTraits()
          : m_fixed(false)
        {
		}

      };

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_NODE_TRAITS_H
#endif
