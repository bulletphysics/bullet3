#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_ADD_PLASTICITY_FORCE_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_ADD_PLASTICITY_FORCE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>
#include <algorithm> //std::min
namespace OpenTissue
{
  namespace fem
  {
    namespace detail
    {


		template <typename tetrahedron_type, typename real_type >
		void add_plasticity_force_single1(tetrahedron_type& tet, real_type const & dt)
		{

			        using std::min;
					using std::sqrt;

        typedef typename tetrahedron_type::vector3_type     vector3_type;


			          assert(tet.m_yield>=0        || !"add_plasticity_force(): yield must be non-negative");
          assert(tet.m_creep>=0        || !"add_plasticity_force(): creep must be non-negative");
          assert(tet.m_creep<=(1.0/dt) || !"add_plasticity_force(): creep must be less that reciprocal time-step");
          assert(tet.m_max>=0          || !"add_plasticity_force(): max must be non-negative");

          //--- Storage for total and elastic strains (plastic strains are stored in the tetrahedra)
          real_type e_total[6];
          real_type e_elastic[6];

          for(int i=0;i<6;++i)
            e_elastic[i] = e_total[i] = 0;

          //--- Compute total strain: e_total  = Be (Re^{-1} x - x0)
          for(unsigned int j=0;j<4;++j)
          {
            vector3_type & x_j = tet.node(j)->m_coord;
            vector3_type & x0_j = tet.node(j)->m_model_coord;

            vector3_type tmp = (trans(tet.m_Re)*x_j) - x0_j;
            real_type bj = tet.m_B[j](0);
            real_type cj = tet.m_B[j](1);
            real_type dj = tet.m_B[j](2);
            e_total[0] +=  bj*tmp(0);
            e_total[1] +=             cj*tmp(1);
            e_total[2] +=                         dj*tmp(2);
            e_total[3] += cj*tmp(0) + bj*tmp(1);
            e_total[4] += dj*tmp(0)             + bj*tmp(2);
            e_total[5] +=             dj*tmp(1) + cj*tmp(2);
          }

          //--- Compute elastic strain
          for(int i=0;i<6;++i)
            e_elastic[i] = e_total[i] - tet.m_plastic[i];

          //--- if elastic strain exceeds c_yield then it is added to plastic strain by c_creep
          real_type norm_elastic = 0;
          for(int i=0;i<6;++i)
            norm_elastic += e_elastic[i]*e_elastic[i];
          norm_elastic = sqrt(norm_elastic);
          //max_elastic = max(max_elastic,norm_elastic);

          if(norm_elastic > tet.m_yield)
          {
            real_type amount = dt*min(tet.m_creep,(1.0/dt));  //--- make sure creep do not exceed 1/dt
            for(int i=0;i<6;++i)
              tet.m_plastic[i] += amount*e_elastic[i];
          }

          //--- if plastic strain exceeds c_max then it is clamped to maximum magnitude
          real_type norm_plastic = 0;
          for(int i=0;i<6;++i)
            norm_plastic += tet.m_plastic[i]*tet.m_plastic[i];
          norm_plastic = sqrt(norm_plastic);
          //max_plastic = max(max_plastic,norm_plastic);

          if(norm_plastic > tet.m_max)
          {
            real_type scale = tet.m_max/norm_plastic;
            for(int i=0;i<6;++i)
              tet.m_plastic[i] *= scale;
          }
          //--- Compute plastic forces:  f_plastic = Re Pe e_plastic; where Pe = Ve Be^T E
          for(unsigned int j=0;j<4;++j)
          {
            real_type * plastic = tet.m_plastic;
            real_type bj = tet.m_B[j](0);
            real_type cj = tet.m_B[j](1);
            real_type dj = tet.m_B[j](2);
            real_type E0 = tet.m_D(0);
            real_type E1 = tet.m_D(1);
            real_type E2 = tet.m_D(2);
            vector3_type f;

            //---
            //---
            //---  Recall the structure of the B and E matrices
            //---
            //---         | bj  0    0 |       | E0  E1  E1           |
            //---   B_j = | 0   cj   0 |       | E1  E0  E0           |
            //---         | 0   0   dj |   E = | E1  E1  E0           |
            //---         | cj  bj   0 |       |             E2       |
            //---         | dj  0   bj |       |                E2    |
            //---         |  0  dj  cj |       |                   E2 |
            //---
            //---   This implyies that the product B_j^T E is
            //---
            //---         | bj E0    bj E1    bj E1    cj E2    dj E2      0   |
            //---         | cj E1    cj E0    cj E1    bj E2      0      dj E2 |
            //---         | dj E1    dj E1    dj E0      0      bj E2    cj E2 |
            //---
            //---  Notice that eventhough this is a 3X6 matrix with 18-3=15 nonzero elements
            //---  it actually only contains 9 different value.
            //---
            //---  In fact these values could be precomputed and stored in each tetrahedron.
            //---  Furthermore they could be pre-multiplied by the volume of the tetrahedron
            //---  to yield the matrix,
            //---
            //---         P_j = Ve B_j^T E
            //---
            //---  The plastic force that should be subtracted from node j is then computed
            //---  in each iteration as
            //---
            //---      f_j = Re P_j e_plastic
            //---
            //---
            real_type  bjE0 = bj*E0;
            real_type  bjE1 = bj*E1;
            real_type  bjE2 = bj*E2;
            real_type  cjE0 = cj*E0;
            real_type  cjE1 = cj*E1;
            real_type  cjE2 = cj*E2;
            real_type  djE0 = dj*E0;
            real_type  djE1 = dj*E1;
            real_type  djE2 = dj*E2;

            f(0) = bjE0*plastic[0] + bjE1*plastic[1] + bjE1*plastic[2] + cjE2*plastic[3] + djE2*plastic[4];
            f(1) = cjE1*plastic[0] + cjE0*plastic[1] + cjE1*plastic[2] + bjE2*plastic[3] +                  + djE2*plastic[5];
            f(2) = djE1*plastic[0] + djE1*plastic[1] + djE0*plastic[2] +                    bjE2*plastic[4] + cjE2*plastic[5];

            f *= tet.m_V;
            tet.node(j)->m_f_external += tet.m_Re*f;
		  }
		}
      /**
      * Add plasticity forces.
      *
      * @param begin      Iterator to first tetrahedron.
      * @param end        Iterator to one past last tetrahedron.
      * @param dt         Simulation time step
      *
      */
      template < typename tetrahedron_iterator,typename real_type >
      inline void add_plasticity_force2(
        tetrahedron_iterator begin
        , tetrahedron_iterator end
        , real_type const & dt
        )
      {
        //typedef typename tetrahedron_iterator::value_type::matrix3x3_type   matrix3x3_type;

        //real_type max_elastic = 0;
        //real_type max_plastic = 0;

        assert(dt>0 || !"add_plasticity_force(): time-step must be positive");
        //---
        //--- The total strain is given by
        //---
        //---    e_total = Be u
        //---
        //--- Where u is the nodal displacement, ie u = x - x0. Applying
        //--- the idea of stiffness warping we have
        //---
        //---   e_total = Be ( Re^{-1} x - x0)
        //---
        //--- where R_e is the rotational deformation of the e'th
        //--- tetrahedral element.
        //---
        //--- The elastic strain is given as the total strain minus the plastic strain
        //---
        //---   e_elastic = e_total - e_plastic
        //---
        //--- e_plastic is initial initialized to zero. Plastic strain causes plastic forces in the material
        //---
        //---    f_plastic = Re Ke u_plastic
        //---
        //--- Using  e_plastic = Be u_plastic
        //---
        //---    f_plastic = Re Ke Be^{-1} e_plastic
        //---    f_plastic = Re (Ve Be^T E Be ) Be^{-1} e_plastic
        //---    f_plastic = Re (Ve Be^T E)  e_plastic
        //---
        //--- Introducing the plasticity matrix Pe = (Ve Be^T E) we have
        //---
        //---    f_plastic = Re Pe  e_plastic
        //---

        for (tetrahedron_iterator T = begin; T != end; ++T)
        {
			add_plasticity_force_single(T,dt);
          
        }
        //std::cout << "max elastic = " << max_elastic << std::endl;
        //std::cout << "max plastic = " << max_plastic << std::endl;

      }

    } // namespace detail
  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_ADD_PLASTICITY_FORCE_H
#endif
