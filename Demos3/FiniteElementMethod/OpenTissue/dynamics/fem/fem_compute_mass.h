#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_MASS_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_MASS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <limits>

namespace OpenTissue
{
  namespace fem
  {
    namespace detail
    {

      /**
      * Compute Mass.
      *
      * Note: Volume should have been computed prior to invoking this
      * function. Thus make sure that initialize_stiffness_elements have
      * been invoked prior to this function.
      *
      * @param mesh
      */
      template<typename fem_mesh>
      inline void compute_mass(fem_mesh & mesh)
      {
        typedef typename fem_mesh::real_type                real_type;

        //---
        //--- A mass matrix is a discrete representation of a continuous mass distribution.
        //--- To compute our mass matrix for a tetrahedral element with linear shape
        //--- functions we need the formula (pp. 266 in Cook)
        //---
        //---                                                  a!b!c!d!
        //---  \int_V N_1^a  N_2^b N_3^c N_4^d dV = 6V --------------------------     (**)
        //---                                             (3 + a + b +c + d)!
        //---
        //--- A consistent element mass matrix (pp. 376 Cook) is defined as
        //---
        //---   m = \int_V \rho N^T N dV           (***)
        //---
        //--- This equation can be derived from work balance, the details of which is unimportant
        //--- here (look Cook pp. 375-376 for details).
        //--- Assumping \rho is constant over each tetrahedral element and using the linear shape
        //--- functions the above definition (***) results in
        //---
        //---                    |N_1|
        //--- m =  \rho \int_V   |N_2|  |N_1 N_2 N_3 N_4| dV
        //---                    |N_3|
        //---                    |N_4|
        //---
        //---                       |(N_1 N_1)   (N_1 N_2)   (N_1 N_3)   (N_1 N_4)|
        //--- m =   \rho    \int_V  |(N_2 N_1)   (N_2 N_2)   (N_2 N_3)   (N_2 N_4)| dV
        //---                       |(N_3 N_1)   (N_3 N_2)   (N_3 N_3)   (N_3 N_4)|
        //---                       |(N_4 N_1)   (N_4 N_2)   (N_4 N_3)   (N_4 N_4)|
        //---
        //--- by (**)
        //---
        //---                 | 2 1 1 1|
        //--- m = \rho  V/20  | 1 2 1 1|          (****)
        //---                 | 1 1 2 1|
        //---                 | 1 1 1 2|
        //---
        //---               V
        //--- m_ij =  \rho --- (1+delta_ij)
        //---               20
        //---
        //--- in 3D this means that for the tetrahedral element
        //---
        //---                | 2 2 2  1 1 1  1 1 1  1 1 1 |
        //---                | 2 2 2  1 1 1  1 1 1  1 1 1 |
        //---                | 2 2 2  1 1 1  1 1 1  1 1 1 |
        //---                |                            |
        //---                | 1 1 1  2 2 2  1 1 1  1 1 1 |
        //---                | 1 1 1  2 2 2  1 1 1  1 1 1 |
        //---            V   | 1 1 1  2 2 2  1 1 1  1 1 1 |
        //--- Me = \rho ---  |                            |
        //---            20  | 1 1 1  1 1 1  2 2 2  1 1 1 |
        //---                | 1 1 1  1 1 1  2 2 2  1 1 1 |
        //---                | 1 1 1  1 1 1  2 2 2  1 1 1 |
        //---                |                            |
        //---                | 1 1 1  1 1 1  1 1 1  2 2 2 |
        //---                | 1 1 1  1 1 1  1 1 1  2 2 2 |
        //---                | 1 1 1  1 1 1  1 1 1  2 2 2 |
        //---
        //--- Notice that in order to obtain the global/system mass matrix an assembly similar to the
        //--- stiffnees matrix assembly must be carried out. Further, the global M matrix will
        //--- have the same sub-block pattern as the global K matrix.
        //---
        //--- A consistent mass matrix is often not used in computer graphics. Instead and
        //--- ad-hoc approach named ``lumped'' mass matrix is applied.
        //--- The lumped mass matrix is obtained by placing particle masses at the nodes.
        //--- This corresponds to shifting all the masses in the rows of (****) onto the
        //--- diagonal. In 3D this yields the element mass matrix
        //---
        //---                | 1 0 0  0 0 0  0 0 0  0 0 0 |
        //---                | 0 1 0  0 0 0  0 0 0  0 0 0 |
        //---                | 0 0 1  0 0 0  0 0 0  0 0 0 |
        //---                |                            |
        //---                | 0 0 0  1 0 0  0 0 0  0 0 0 |
        //---                | 0 0 0  0 1 0  0 0 0  0 0 0 |
        //---            V   | 0 0 0  0 0 1  0 0 0  0 0 0 |
        //--- Me = \rho ---  |                            |
        //---            4   | 0 0 0  0 0 0  1 0 0  0 0 0 |
        //---                | 0 0 0  0 0 0  0 1 0  0 0 0 |
        //---                | 0 0 0  0 0 0  0 0 1  0 0 0 |
        //---                |                            |
        //---                | 0 0 0  0 0 0  0 0 0  1 0 0 |
        //---                | 0 0 0  0 0 0  0 0 0  0 1 0 |
        //---                | 0 0 0  0 0 0  0 0 0  0 0 1 |
        //---
        //--- Thus a lumped mass matrix is diagonal whereas a consistent mass matrix
        //--- is not. Observe that the global mass matrix would also diagonal and the
        //--- assembly is simplified to an iteration over all tetrahedra, while
        //--- incementing the nodal mass by one fourth of the tetrahedral mass.
        //---
        //---  for each node n
        //---    mass(n) = 0
        //---  next n
        //---  for each tetrahedron e
        //---    mass(n_i) += \rho_e Ve / 4
        //---    mass(n_j) += \rho_e Ve / 4
        //---    mass(n_k) += \rho_e Ve / 4
        //---    mass(n_m) += \rho_e Ve / 4
        //---  next e
        //---
        //--- where n_i,n_j,n_k and n_m are the four nodes of the e'th tetrahedron.
        //---
        //--- The advantage of lumping is less storage and higher performace. On the downside
        //--- lumping introduces a discontinouty in the displacement field.
        //---
        //--- Obrien.shen state that the errors in lumping is negligeble for small-size course
        //--- meshes used in computer graphics. However, for finer meshes the errors becomes
        //--- noticeable.
        //---
        //--- There do exist other approaches for computing mass matrices, even mehtods which
        //--- combine other methods. We refer the interested reader to Cook for more details. Here
        //--- we have limited our selfes to the two most common methods.
        //---
        //--- It is worthwhile to notice that under the reasonable assumptions that V and \rho are
        //--- positive for all elements both the element mass matrices and the global mass matrices
        //--- are symmetric positive definite matrices.

        for (int n=0;n<mesh.m_nodes.size();n++)
        {

			if(mesh.m_nodes[n].m_fixed)
            mesh.m_nodes[n].m_mass = std::numeric_limits<real_type>::max();
          else
            mesh.m_nodes[n].m_mass = 0;
        }

        for (int t=0;t<mesh.m_tetrahedra.size();t++)
        {

          real_type amount = mesh.m_tetrahedra[t].m_density * mesh.m_tetrahedra[t].m_V *.25;
          mesh.m_tetrahedra[t].i()->m_mass += amount;
          mesh.m_tetrahedra[t].j()->m_mass += amount;
          mesh.m_tetrahedra[t].k()->m_mass += amount;
          mesh.m_tetrahedra[t].m()->m_mass += amount;
        }
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_MASS_H
#endif
