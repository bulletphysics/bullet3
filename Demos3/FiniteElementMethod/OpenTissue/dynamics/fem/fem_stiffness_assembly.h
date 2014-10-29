#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_STIFFNESS_ASSEMBLY_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_STIFFNESS_ASSEMBLY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#define p_i (&T->m_owner->m_nodes[T->m_nodes[i]])
#define p_j (&T->m_owner->m_nodes[T->m_nodes[j]])

namespace OpenTissue
{
  namespace fem
  {
    namespace detail
    {
      /**
      * Stiffness Matrix Assembly.
      * Observe that prior to invocations of this method the rotations of all tetrahedra
      * must have been setted. Also all stiffness assembly data should have been cleared
      * prior to invocations by using the clear_stiffness_assembly method.
      *
      *
      * From linear elastostatics we have
      *
      *   K u = f
      *
      * where u is vertex displacements and f is the resulting nodal forces and K is the stiffness matrix.
      * Using u = x - x0, where x is current position and x0 original position we have
      *
      *   K (x - x0)  = f
      *   K x  - K x0 = f
      *
      * Introducing  f0 = - K x0 as the force offset vectors we have
      *
      *   f = K x + f0
      *
      * The idea of stiffness warping is to rotate the x-position back into the
      * original coordinate system in which K were original computed, then compute
      * nodal forces in this frame and finally rotate teh nodal forces back to
      * the current frame.
      *
      * Let R denote the current orientation, then R^{-1} rotates back to the
      * orignal frame which means we have
      *
      *   f =   R K ( R^{-1} x  - x0 )
      *   f =   R K R^{-1} x  - R K x0
      *
      * So with stiffness warping we have to compute
      *
      *   K'  =  R K R^{-1}
      *   f0' =  - R K x0
      *
      *  For n nodes the system stiffness matrix K' is a 3n X 3n symmetric and sparse matrix.
      *  It would be insane to actual allocated such a matrix instead the matrix is
      *  stored inside the nodes of the volume mesh.
      *
      *  Each node stores a row of K' and the correspond entries of the f0' vector.
      *
      *  That is the i'th node stores all non-zero 3-by-3 sub-block of the i'th row
      *  of the stiffness matrix and it also stores the i'th 3-dimensional subvector of f0
      *
      *        K'              f0'
      *             j
      *      -      .    -      - -
      *      |      .    |      | |
      *      |      .    |      | |
      * i->  |......X....|      |x|
      *      |      .    |      | |
      *      |      .    |      | |
      *      -      .    -      - -
      *
      *
      * Let M denote the set of all tetrahedral elements then the assembly
      *  of the warped stiffness matrix can be written as
      *
      *    K'_ij  =      sum    Re Ke_ij Re^T
      *                e \in M
      *                 and
      *               i, j \in e
      *
      * And the assembly of the force offset vector
      *
      *    f0'_i  =      sum    - Re Ke_ij x0_j
      *                e \in M
      *                 and
      *               i, j \in e
      *
      *
      * Notice that this can be optimized if node i as a meber of the e'th element
      *  then the contribution to the above summation can be written as (recal
      * the indices j,k and m denote the three ofther nodes of e):
      *
      *    -Re Ke_ii x0_i - Re Ke_ij x0_j -Re Ke_im x0_m - Re Ke_ik x0_k
      *    -Re   ( Ke_ii x0_i + Ke_ij x0_j + Ke_im x0_m + Ke_ik x0_k)
      *
      * which saves us a few matrix multiplications and we then finally have
      *
      *    f0'_i  =      sum  -Re ( Ke_ii x0_i + Ke_ij x0_j + Ke_im x0_m + Ke_ik x0_k)
      *                e \in M
      *                  and
      *                i \in e
      *
      * Assuming that f0'_i is initially cleared to zero for all i. This result in the
      * implementation strategy:
      *
      *   for each element e do
      *     for each node i of e do
      *       tmp = 0
      *       for each node j of e do
      *         tmp += Ke_ij * xo_j
      *        next j
      *        f0'_i -= Re*tmp
      *      next i
      *    next e
      *
      * Also the stiffness matrix can be optimized slightly by exploiting the symmetry property.
      * The symmetry indicates that it is sufficient to only compute the upper triangular and
      * diagonal parts. The lower triangular parts can be found simply be taking the transpose
      * of the upper triangular parts.
      *
      * So the e'th tetrahedral element contributes with
      *
      *     K'_ij +=  Re Ke_ij Re^{T}
      *     K'_ji +=  Re Ke_ji Re^{T} = ( Re Ke_ij Re^{T} )^T
      *
      * because Ke_ji = Ke_ij^T. Assuming that all K'_ij is initially cleared to zero this results
      * in the implementation strategy:
      *
      *   for each element e do
      *     for each node i of e do
      *       for each node j of e do
      *         if i >= j then
      *            tmp = Re Ke_ij Re^{T}
      *            K'_ij +=  tmp
      *            if j > i then
      *              K'_ji +=  trans(tmp)
      *            end if
      *          end if
      *        next j
      *      next i
      *    next e
      *
      * The two implementation strategies can now be combined into one, which
      * is what we have implemented below.
      *
      * Note that if Re is initially set to the identity matrix, then the stiffness
      * warping reduces to the traditional assembly of stiffness matrix (as it is
      * used in linear elastostatics).
      *
      *
      * If the i'th node is set to be fixed, this actually corresponds to
      * letting K'_ii = identity and letting K'_ij and K'_ji be zero for all j not
      * equal to i. This is known as a Direchlet boundary condition. However, we do
      * not this during our assembly. Instead we assemble the K' matrix as though
      * there were no fixed nodes. Later on when we use the K' matrix in computations
      * such as K' x, we simply test whether x_j is fixed when it is multilpied by
      * the j'th column of K' i.e. K'_*j. If so we simply do nothing!!! This is
      * computatonally more tracjktable and it also allow us to more easily turn
      * nodes fixed and un-fixed dynamically during animation.
      *
      *
      *
      *
      *
      * @param begin
      * @param end
      *
      */
      template<typename tetrahedron_type>
      inline void stiffness_assembly_single1(tetrahedron_type* T)
      {
		typedef typename tetrahedron_type::real_type        real_type;
        typedef typename tetrahedron_type::vector3_type     vector3_type;
        typedef typename tetrahedron_type::matrix3x3_type   matrix3x3_type;

        {
          matrix3x3_type & Re = T->m_Re;
          for (int i = 0; i < 4; ++i)
          {
            vector3_type f;
            f.clear();
            for (int j = 0; j < 4; ++j)
            {
              matrix3x3_type & Ke_ij = T->m_Ke[i][j];
              vector3_type   & x0_j  = p_j->m_model_coord;

              f += Ke_ij * x0_j;
              if (j >= i)
              {

                matrix3x3_type tmp = Re * Ke_ij * trans(Re);

                p_i->K(p_j->idx()) += tmp;
                if (j > i)
                  p_j->K(p_i->idx()) += trans(tmp);
              }
            }

            p_i->m_f0 -= Re*f;

          }
        }
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_STIFFNESS_ASSEMBLY_H
#endif
