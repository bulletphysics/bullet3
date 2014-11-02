#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_UPDATE_ORIENTATION_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_UPDATE_ORIENTATION_H
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
      */
      template<typename tetrahedron_type>
      inline void update_orientation_single(tetrahedron_type* T)
      {
        typedef typename tetrahedron_type::real_type        real_type;
        typedef typename tetrahedron_type::vector3_type     vector3_type;
        typedef typename tetrahedron_type::matrix3x3_type   matrix3x3_type;

        {
          real_type div6V = 1.0 / T->m_V*6.0;
          //--- The derivation in the orignal paper on stiffness warping were stated as
          //---
          //---         | n1^T |
          //---    N =  | n2^T |   : WCS -> U
          //---         | n3^T |
          //---
          //---         | n1'^T |
          //---   N' =  | n2'^T |   : WCS -> D
          //---         | n3'^T |
          //---
          //--- From which we have
          //---
          //---    R = N' N^T
          //---
          //--- This is valid under the assumption that the n-vectors form a orthonormal basis. In that
          //--- paper the n-vectors were determined using a heuristic approach. Besides
          //--- the rotation were computed on a per vertex basis not on a per-tetrahedron
          //--- bases as we have outline above.
          //---
          //---
          //--- Later Muller et. al. used barycentric coordinates to find the transform. We
          //--- will here go into details on this method. Let the deformed corners be q0, q1,
          //--- q2, and q3 and the undeformed corners p0, p1, p2,and p3. Looking at some
          //--- point p = [x y z 1]^T inside the undeformed tetrahedron this can be written
          //---
          //---                          | w0 |
          //---      p = | p0 p1 p2 p3 | | w1 | = P w  (*1)
          //---          | 1   1  1  1 | | w2 |
          //---                          | w3 |
          //---
          //---  The same point in the deformed tetrahedron has the same barycentric coordinates
          //---  which mean
          //---
          //---                          | w0 |
          //---      q = | q0 q1 q2 q3 | | w1 | = Q w   (*2)
          //---          | 1   1  1  1 | | w2 |
          //---                          | w3 |
          //---
          //---  We can now use (*1) to solve for w and insert this into (*2), this yields
          //---
          //---      q = Q P^{-1} p
          //---
          //---  The matirx Q P^{-1} transforms p into q. Due to P and Q having their fourth rows
          //---  equal to 1 it can be shown that this matrix have the block structure
          //---
          //---       Q P^{-1}  = | R  t  |        (*3)
          //---                   | 0^T 1 |
          //---
          //---  Which we recognize as a transformation matrix of homegeneous coordinates. The t vector
          //---  gives the translation, the R matrix includes, scaling, shearing and rotation.
          //---
          //---  We thus need to extract the rotational information from this R matrix. In their
          //---  paper Mueller et. al. suggest using Polar Decompostions (cite Shoemake and Duff; Etzmuss).
          //---  However, a simpler although more imprecise approach would simply be to apply a Grahram-Schimdt
          //---  orthonormalization to transform R into an orthonormal matrix. This seems to work quite well
          //---  in practice. We have not observed any visual difference in using polar decomposition or
          //---  orthonormalization.
          //---
          //---  Now for some optimizations. Since we are only interested in the R part of (*3) we can
          //---  compute this more efficiently exploiting the fact that barycentric coordinates sum
          //---  up to one. If we substitute w0 = 1 - w1 - w2 - w3 in (*1) and (*2) we can throw away
          //---  the fourth rows since they simply state 1=1, which is trivially true.
          //---
          //---                           | 1-w1-w2-w3 |
          //---      p = | p0 p1 p2 p3 |  |     w1     |
          //---                           |     w2     |
          //---                           |     w3     |
          //---
          //---  Which is
          //---
          //---                                          | 1  |
          //---      p = | p0 (p1-p0) (p2-p0) (p3-p0) |  | w1 |
          //---                                          | w2 |
          //---                                          | w3 |
          //---
          //--- We can move the first column over on the left hand side
          //---
          //---                                            | w1 |
          //---      (p-p0) = | (p1-p0) (p2-p0) (p3-p0) |  | w2 |
          //---                                            | w3 |
          //---
          //---  Introducing e10 = p1-p0, e20 = p2-p0, e30 = p3-p0, and E = [e10 e20 e30 ] we have
          //---
          //---                  w1
          //---      (p-p0) = E  w2    (*5)
          //---                  w3
          //---
          //---  Similar for *(2) we have
          //---
          //---                   w1
          //---      (q-q0) = E'  w2  (*6)
          //---                   w3
          //---
          //--- Where E' = [e10' e20' e3'] and e10' = q1-q0, e20' = q2-q0, e30' = q3-q0. Now
          //--- inverting (*5) and insertion into (*6) yields
          //---
          //---   (q-q0) = E'  E^{-1} (p-p0)
          //---
          //--- By comparison with (*3) we see that
          //---
          //---           R = E' E^{-1}
          //---
          //--- Using Cramers rule the inverse of E can be written as
          //---
          //---                 | (e2 x e3)^T |
          //---   E^{-1} = 1/6V | (e3 x e1)^T |
          //---                 | (e1 x e2)^T |
          //---
          //--- This can easily be confirmed by straigthforward computation
          //---
          //---                     | e1 \cdot (e2 x e3)   e2 \cdot (e2 x e3)   e3 \cdot (e2 x e3) |
          //---   E^{-1} E  =  1/6v | e1 \cdot (e3 x e1)   e2 \cdot (e3 x e1)   e3 \cdot (e3 x e1) |
          //---                     | e1 \cdot (e1 x e2)   e2 \cdot (e1 x e2)   e3 \cdot (e1 x e2) |
          //---
          //---                     | 6V   0  0  |
          //---              = 1/6V | 0   6V  0  |  = I
          //---                     | 0    0  6V |
          //---
          //--- Using the notation
          //---
          //---   n1 = e2 x e3 / 6V
          //---   n2 = e3 x e1 / 6V
          //---   n3 = e1 x e2 / 6V
          //---
          //--- we write
          //---
          //---             | n1^T |
          //---   E^{-1} =  | n2^T |
          //---             | n3^T |
          //---
          //---  And we end up with
          //---
          //---                          | n1^T |
          //---   R = [e10'  e20'  e30'] | n2^T |
          //---                          | n3^T |
          //---
          //--- Observe that E' is very in-expensive to compute and all non-primed quantities (n1, n2, n3) can
          //--- be precomputed and stored on a per tetrahedron basis if there is enough memory available. Even
          //--- in case where memory is not available n1, n2 and n3 are quite cheap to compute.
          //---

          real_type e1x = T->m_e10(0);
          real_type e1y = T->m_e10(1);
          real_type e1z = T->m_e10(2);
          real_type e2x = T->m_e20(0);
          real_type e2y = T->m_e20(1);
          real_type e2z = T->m_e20(2);
          real_type e3x = T->m_e30(0);
          real_type e3y = T->m_e30(1);
          real_type e3z = T->m_e30(2);
          real_type n1x = (e2y * e3z - e3y * e2z) * div6V;
          real_type n1y = (e3x * e2z - e2x * e3z) * div6V;
          real_type n1z = (e2x * e3y - e3x * e2y) * div6V;
          real_type n2x = (e1z * e3y - e1y * e3z) * div6V;
          real_type n2y = (e1x * e3z - e1z * e3x) * div6V;
          real_type n2z = (e1y * e3x - e1x * e3y) * div6V;
          real_type n3x = (e1y * e2z - e1z * e2y) * div6V;
          real_type n3y = (e1z * e2x - e1x * e2z) * div6V;
          real_type n3z = (e1x * e2y - e1y * e2x) * div6V;
		  const vector3_type & p0 = T->m_owner->m_nodes[T->m_nodes[0]].m_coord;
		  const vector3_type & p1 = T->m_owner->m_nodes[T->m_nodes[1]].m_coord;
		  const vector3_type & p2 = T->m_owner->m_nodes[T->m_nodes[2]].m_coord;
		  const vector3_type & p3 = T->m_owner->m_nodes[T->m_nodes[3]].m_coord;

          e1x = p1(0) - p0(0);
          e1y = p1(1) - p0(1);
          e1z = p1(2) - p0(2);
          e2x = p2(0) - p0(0);
          e2y = p2(1) - p0(1);
          e2z = p2(2) - p0(2);
          e3x = p3(0) - p0(0);
          e3y = p3(1) - p0(1);
          e3z = p3(2) - p0(2);
          T->m_Re(0,0) = e1x * n1x + e2x * n2x + e3x * n3x;   T->m_Re(0,1) = e1x * n1y + e2x * n2y + e3x * n3y;   T->m_Re(0,2) = e1x * n1z + e2x * n2z + e3x * n3z;
          T->m_Re(1,0) = e1y * n1x + e2y * n2x + e3y * n3x;   T->m_Re(1,1) = e1y * n1y + e2y * n2y + e3y * n3y;   T->m_Re(1,2) = e1y * n1z + e2y * n2z + e3y * n3z;
          T->m_Re(2,0) = e1z * n1x + e2z * n2x + e3z * n3x;   T->m_Re(2,1) = e1z * n1y + e2z * n2y + e3z * n3y;   T->m_Re(2,2) = e1z * n1z + e2z * n2z + e3z * n3z;

          T->m_Re = ortonormalize(T->m_Re);

          //matrix3x3_type M = T->m_Re,S;
          //OpenTissue::PolarDecomposition3x3 decomp;
          //decomp.eigen(M, T->m_Re, S);//--- Etzmuss-style
          //decomp.newton(M, T->m_Re );//--- Shoemake-Duff-style
        }
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_UPDATE_ORIENTATION_H
#endif
