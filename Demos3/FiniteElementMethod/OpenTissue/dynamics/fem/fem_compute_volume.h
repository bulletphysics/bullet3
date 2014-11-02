#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_VOLUME_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_VOLUME_H
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
      * Compute volume of tetrahedron.
      *
      * @param e10  edge vector from p0 to p1, i.e. e10 = p1-p0.
      * @param e20  edge vector from p0 to p2, i.e. e20 = p2-p0.
      * @param e30  edge vector from p0 to p3, i.e. e30 = p3-p0.
      *
      * @return     The signed volume of the tetrahedra with nodes p0,p1,p2 and p3.
      */
      template<typename vector3_type>
      inline typename vector3_type::value_type compute_volume(vector3_type const& e10, vector3_type const& e20, vector3_type const& e30)
      {
        typedef typename vector3_type::value_type real_type;
        // Calculate the volume given by:
        //                                                      |  1   1   1   1  |
        //                                 | e1x e2x e3x |      | p0x p1x p2x p3x |
        //   6V = | e1 \cdot (e2 x e3) | = | e1y e2y e3y | = 6V | p0y p1y p2y p3y |
        //                                 | e1z e2z e3z |      | p0z p1z p2z p3z |
        //
        real_type sixV = e10*(e20 % e30);
        return sixV / 6.0;
      }

      /**
      * Compute Tetrahedron Volume
      *
      * @param p0
      * @param p1
      * @param p2
      * @param p3
      *
      * @return     The signed volume of the tetrahedra with nodes p0,p1,p2 and p3.
      */
      template<typename vector3_type>
      inline typename vector3_type::value_type compute_volume(vector3_type const & p0,vector3_type const & p1,vector3_type const & p2,vector3_type const & p3)
      {
        typedef typename vector3_type::value_type  real_type;

        real_type d = p0(0);
        real_type d4 = p0(1);
        real_type d8 = p0(2);
        real_type d1 = p1(0) - d;
        real_type d5 = p1(1) - d4;
        real_type d9 = p1(2) - d8;
        real_type d2 = p2(0) - d;
        real_type d6 = p2(1) - d4;
        real_type d10 = p2(2) - d8;
        real_type d3 = p3(0) - d;
        real_type d7 = p3(1) - d4;
        real_type d11 = p3(2) - d8;
        real_type d12 = (d1 * d6 * d11 + d2 * d7 * d9 + d3 * d5 * d10) - d1 * d7 * d10 - d2 * d5 * d11 - d3 * d6 * d9;
        return d12 / 6.0;
        /*
        ** From Zienkiewicz & Taylor p.637
        ** V = 1/6*det( 1 x0 y0 z0; 1 x1 y1 z1; 1 x2 y2 z2; 1 x3 y3 z3)
        ** where x0 = n0->i; y0 = n0(1); ...
        ** Calculated by Mathematica ;-)
        */
        /*
        return (p0(2)*p1(1)*p2(0) - p0(1)*p1(2)*p2(0) - p0(2)*p1(0)*p2(1)
        + p0(0)*p1(2)*p2(1) + p0(1)*p1(0)*p2(2) - p0(0)*p1(1)*p2(2)
        - p0(2)*p1(1)*p3(0) + p0(1)*p1(2)*p3(0) + p0(2)*p2(1)*p3(0)
        - p1(2)*p2(1)*p3(0) - p0(1)*p2(2)*p3(0) + p1(1)*p2(2)*p3(0)
        + p0(2)*p1(0)*p3(1) - p0(0)*p1(2)*p3(1) - p0(2)*p2(0)*p3(1)
        + p1(2)*p2(0)*p3(1) + p0(0)*p2(2)*p3(1) - p1(0)*p2(2)*p3(1)
        - p0(1)*p1(0)*p3(2) + p0(0)*p1(1)*p3(2) + p0(1)*p2(0)*p3(2)
        - p1(1)*p2(0)*p3(2) - p0(0)*p2(1)*p3(2) + p1(0)*p2(1)*p3(2))/6.;
        */
      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_COMPUTE_VOLUME_H
#endif
