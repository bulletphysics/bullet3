#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_INIT_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_INIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/fem/fem_uniform_young.h>
#include <OpenTissue/dynamics/fem/fem_uniform_poisson.h>
#include <OpenTissue/dynamics/fem/fem_uniform_density.h>
#include <OpenTissue/dynamics/fem/fem_compute_mass.h>
#include <OpenTissue/dynamics/fem/fem_initialize_stiffness_elements.h>
#include <OpenTissue/dynamics/fem/fem_clear_stiffness_assembly.h>
#include <OpenTissue/dynamics/fem/fem_initialize_plastic.h>

namespace OpenTissue
{
  namespace fem
  {
    /**
    * Initialize Data Structres.
    * Should be invoked at start-up before any calls to animate_warping.
    *
    * This function assumes that mesh geometry have been set up prior
    * to invokation (i.e. both world coord and orignial coord are initlized
    * and the same).
    *
    * This method initialized the model with the same material parameters. End
    * user could write his or her own initilization method replacing
    * the steps where young modules, poisson ratio and densities are
    * assigned to the model.
    *
    * @param mesh
    * @param young    Young Moduls, a value of 500000 seems work quite well.
    * @param poisson  Poisson ration, a value of 0.33 seems work quite well.
    * @param density    Mass density, a value of 1000 seems to work quite well.
    * @param c_yield    Plastic yield.
    * @param c_creep    Plastic creep.
    * @param c_max      Plastic max.
    */
    template < typename fem_mesh,typename real_type >
    inline void init(
      fem_mesh & mesh,
      real_type const & young,
      real_type const & poisson,
      real_type const & density,
      real_type const & c_yield,
      real_type const & c_creep,
      real_type const & c_max
      )
    {
        assert(poisson>=0   || !"uniform_poisson() : Poisson ratio must be non-negative");
        assert(poisson<=0.5 || !"uniform_poisson() : Poisson ratio must not be larger than a half");
        assert(density>0 || !"uniform_density(): density must be positive");

		//--- Assign material parameters
		for (int t=0;t<mesh.m_tetrahedra.size();t++)
		{
			mesh.m_tetrahedra[t].m_young = young;
			mesh.m_tetrahedra[t].m_poisson = poisson;
			mesh.m_tetrahedra[t].m_density = density;
			detail::initialize_stiffness_elements_single(&mesh.m_tetrahedra[t]);
		}
      //--- Compute stiffness and mass matrices
		for (int m=0;m<mesh.m_nodes.size();m++)
		{
			detail::clear_stiffness_assembly_single(&mesh.m_nodes[m]);
		}
      detail::compute_mass(mesh);
	  for (int t=0;t<mesh.m_tetrahedra.size();t++)
	  {
	      detail::initialize_plastic_single(&mesh.m_tetrahedra[t],c_yield,c_creep,c_max);
	  }
    }

  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_INIT_H
#endif
