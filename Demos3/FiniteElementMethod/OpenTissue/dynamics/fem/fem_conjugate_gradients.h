#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_CONJUGATE_GRADIENTS_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_CONJUGATE_GRADIENTS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#define n_i (&mesh.m_nodes[n])
#define n_j (&mesh.m_nodes[j])


namespace OpenTissue
{
  namespace fem
  {
    namespace detail
    {
      /**
      * Conjugate Gradient Solver.
      * This solver has been hard-wired for a WarpT4Mesh to solve the equation
      *
      *   A v = b
      *
      */
      template < typename fem_mesh >
      inline void conjugate_gradients(
        fem_mesh & mesh
        , unsigned int min_iterations
        , unsigned int max_iterations
        )
      {
        using std::fabs;

        typedef typename fem_mesh::real_type                     real_type;
        typedef typename fem_mesh::vector3_type                  vector3_type;
        typedef typename fem_mesh::matrix3x3_type                matrix3x3_type;
        typedef typename fem_mesh::node_type::matrix_iterator    matrix_iterator;


        real_type tiny      = 1e-010;       // TODO: Should be user controllable
        real_type tolerence = 0.001;        // TODO: Should be user controllable

        //---  r = b - A v
        //---  p = r
        for(int n=0;n<mesh.m_nodes.size();n++)
        {

			if(n_i->m_fixed)
            continue;

          n_i->m_residual = n_i->m_b;

          matrix_iterator Abegin = n_i->Abegin();
          matrix_iterator Aend   = n_i->Aend();
          for (matrix_iterator A = Abegin; A != Aend;++A)
          {
            unsigned int     j    = A->first;
            matrix3x3_type & A_ij = A->second;
            vector3_type &   v_j  = n_j->m_velocity;

            n_i->m_residual -= A_ij * v_j;
          }
          n_i->m_prev = n_i->m_residual;
        }

        for(unsigned int iteration = 0; iteration < max_iterations; ++iteration)
        {
          real_type d = 0.0;
          real_type d2 = 0.0;

          //--- u = A p
          //--- d = r*r
          //--- d2 = p*u

          for(int n=0;n<mesh.m_nodes.size();n++)
          {
            if(n_i->m_fixed)
              continue;

            n_i->m_update.clear();

            matrix_iterator Abegin = n_i->Abegin();
            matrix_iterator Aend   = n_i->Aend();
            for (matrix_iterator A = Abegin; A != Aend;++A)
            {

              unsigned int     j    = A->first;
              matrix3x3_type & A_ij = A->second;

              n_i->m_update += A_ij * n_j->m_prev;

            }
            d  += n_i->m_residual * n_i->m_residual;
            d2 += n_i->m_prev     * n_i->m_update;
          }

          if(fabs(d2) < tiny)
            d2 = tiny;

          real_type d3 = d / d2;
          real_type d1 = 0.0;
          //--- v += p * d3
          //--- r -= u * d3
          //--- d1 = r*r

          for(int n=0;n<mesh.m_nodes.size();n++)
          {
            if(n_i->m_fixed)
              continue;
            n_i->m_velocity +=  n_i->m_prev     * d3;
            n_i->m_residual -=  n_i->m_update   * d3;
            d1 +=  n_i->m_residual * n_i->m_residual;
          }
          if(iteration >= min_iterations && d1 < tolerence)
            break;

          if(fabs(d) < tiny)
            d = tiny;

          real_type d4 = d1 / d;
          //--- p = r + d4 * p
		  for(int n=0;n<mesh.m_nodes.size();n++)
          {

		    if(n_i->m_fixed)
              continue;
            n_i->m_prev = n_i->m_residual + n_i->m_prev * d4;
          }
        }
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_CONJUGATE_GRADIENTS_H
#endif
