#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_DYNAMICS_ASSEMBLY_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_DYNAMICS_ASSEMBLY_H
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
      * Setup dynamic equation.
      * This is the equation of motion given as:
      *
      *
      *   A v^{i+1} = b
      *
      *
      * where
      *

      *   A = M + \delta t C + \delta t^2 K
      *   b = M v^i - \delta t (K x^i + f_0 + f_plas - f_ext)
      *
      * In this implementation, the following holds:
      *
      *    M is a diagonal matrix with diagonal elements given by m_mass.
      *    C is a diagonal matrix given by massCoef*M, which is Raleigh damping with stiffness coefficient zero.
      *
      * Also plastic forces, f_plas, is ignored.
      *
      * @param dt                The time step, \delta t, which is about to be taken.
      * @param mass_damping      Coefficient for mass damping in the Raleigh damping equation.
      *                          The coefficient \alpha in C = \alpha M + \beta K. In this implementation \beta = 0.
      *
      *
      *
      *
      */
      template < typename fem_mesh, typename real_type >
      inline void dynamics_assembly(
        fem_mesh & mesh,
        real_type const & mass_damping,
        real_type const & dt
        )
      {
        typedef typename fem_mesh::vector3_type                  vector3_type;
        typedef typename fem_mesh::matrix3x3_type                matrix3x3_type;
        typedef typename fem_mesh::node_type::matrix_iterator    matrix_iterator;

		for (int n = 0;n<mesh.m_nodes.size();n++)
		{
		  //fem_mesh::node_type& ni = mesh.m_nodes[n];
		  //if (ni.m_mass)
		  {
			  unsigned int i     =  n_i->idx();
			  vector3_type & b_i =  n_i->m_b;
			  real_type & m_i    =  n_i->m_mass;

			  b_i.clear();

			  matrix_iterator Kbegin = n_i->Kbegin();
			  matrix_iterator Kend   = n_i->Kend();
			  for (matrix_iterator K = Kbegin; K != Kend;++K)
			  {
				unsigned int     j    = K->first;
				matrix3x3_type & K_ij = K->second;
				vector3_type &   x_j  = n_j->m_coord;
				matrix3x3_type & A_ij = n_i->A(j);

				A_ij = K_ij * (dt*dt);
				b_i -= K_ij * x_j;
				if (i == j)
				{
				  real_type c_i = mass_damping*m_i;
				  real_type tmp = m_i + dt*c_i;
				  A_ij(0,0) += tmp; A_ij(1,1) += tmp;  A_ij(2,2) += tmp;
				}
			  }
			  b_i -= n_i->m_f0;
			  b_i += n_i->m_f_external;
			  b_i *= dt;
			  b_i += n_i->m_velocity * m_i;
		  }
        }
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_DYNAMICS_ASSEMBLY_H
#endif
