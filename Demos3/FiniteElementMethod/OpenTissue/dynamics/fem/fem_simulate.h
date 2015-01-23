#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_SIMULATE_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_SIMULATE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/fem/fem_clear_stiffness_assembly.h>
#include <OpenTissue/dynamics/fem/fem_update_orientation.h>
#include <OpenTissue/dynamics/fem/fem_reset_orientation.h>
#include <OpenTissue/dynamics/fem/fem_stiffness_assembly.h>
#include <OpenTissue/dynamics/fem/fem_add_plasticity_force.h>
#include <OpenTissue/dynamics/fem/fem_dynamics_assembly.h>
#include <OpenTissue/dynamics/fem/fem_conjugate_gradients.h>
#include <OpenTissue/dynamics/fem/fem_position_update.h>

namespace OpenTissue
{
  namespace fem
  {
    /**
    * Simulate.
    * Note external forces must have been computed prior to invocation.
    *
    * The dynamic equation has the following form (where u=x-x_0, and x' is derivative wrt. time)
    *
    *   M x'' + Cx' + K (x-x_0) = f_ext
    *
    * This can be transformed to a system of 2x3n equations of first order derivative:
    *
    *     x' = v
    *   M v' = - C v - K (x-x_0) + f_ext
    *
    * The semi-implicit Euler scheme approximates the above with:
    *
    *     x^(i+1) = x^i + \delta t * v^(i+1)
    *   M v^(i+1) = M v^i + \delta t ( - C v^(i+1)  - K ( x^i - x_0 ) + f^i_ext
    *
    * This is solved using implicit integration.
    *
    * @param mesh
    * @param time_step
    * @param use_stiffness_warping
    */
    template < typename fem_mesh, typename real_type >
    inline void simulate(
      fem_mesh & mesh
      , real_type const & time_step
      , bool use_stiffness_warping,
	  real_type mass_damping = 2.0,
	  unsigned int min_iterations=20,
	  unsigned int max_iterations=20
      )
    {
      // Some theory:
      //
      //
      // Implicit discretization of
      //
      //   M d^2x/dt^2 + C dx/dt + K (x-x0) = f_ext
      //
      // Evaluate at (i+1), and use d^2x/dt^2 = (v^{i+1} - v^i)/timestep and  dx/dt = v^{i+1}
      //
      //     M (v^{i+1} - v^i)/timestep + C v^{i+1} + K (x^{i+1}-x0) = f_ext
      //
      // and    x^{i+1} = x^i + v^{i+1}*timestep
      //
      //     M (v^{i+1} - v^i)/timestep + C v^{i+1} + K ( (x^i + v^{i+1}*timestep) -x0) = f_ext
      //
      //     M (v^{i+1} - v^i)/timestep + C v^{i+1} + K*x^i + timestep*K*v^{i+1} - K x0 = f_ext
      //
      //     M v^{i+1} - M v^i + timestep*C v^{i+1} + timestep*timestep*K*v^{i+1}  = timestep * (f_ext - K*x^i  + K x0)
      //
      //     (M  + timestep*C  + timestep*timestep*K) v^{i+1}  =  M v^i + timestep * (f_ext - K*x^i  + K x0)
      //
      // let f0 = -K x0
      //
      //     (M  + timestep*C  + timestep*timestep*K) v^{i+1}  =  M v^i + timestep * (f_ext - K*x^i  -f0)
      //
      //     (M  + timestep*C  + timestep*timestep*K) v^{i+1}  =  M v^i - timestep * (K*x^i  + f0 - f_ext)
      //
      // so we need to solve A v^{i+1} = b for v^{i+1}, where
      //
      //     A = (M  + timestep*C  + timestep*timestep*K)
      //     b =  M v^i - timestep * (K*x^i  + f0 - f_ext)
      //
      // afterwards we do a position update
      //
      //     x^{i+1} = x^i + v^{i+1}*timestep
      //
      // Notice that a fully implicit scheme requres K^{i+1}, however for linear elastic materials K is constant.

	for (int n=0;n<mesh.m_nodes.size();n++)
	{
      detail::clear_stiffness_assembly_single(&mesh.m_nodes[n]);
	}
	for (int t=0;t<mesh.m_tetrahedra.size();t++)
	{
      if(use_stiffness_warping)
        detail::update_orientation_single(&mesh.m_tetrahedra[t]);
      else
        detail::reset_orientation_single(&mesh.m_tetrahedra[t]);
	}

	for (int t=0;t<mesh.m_tetrahedra.size();t++)
	{
      detail::stiffness_assembly_single1(&mesh.m_tetrahedra[t]);
	}
	  for (int i=0;i<mesh.m_tetrahedra.size();i++)
	  {
	      detail::add_plasticity_force_single1(mesh.m_tetrahedra[i],time_step);
	  }

      detail::dynamics_assembly(mesh,mass_damping,time_step);


      detail::conjugate_gradients(mesh, min_iterations, max_iterations);

	  detail::position_update(mesh,time_step);

    }

  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_SIMULATE_H
#endif
