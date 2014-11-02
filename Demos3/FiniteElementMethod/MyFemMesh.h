/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011-2014 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///the Finite Element Method is extracted from the OpenTissue library,
///under the zlib license: http://www.opentissue.org/mediawiki/index.php/Main_Page


#ifndef OPENTISSUE_DYNAMICS_FEM_FEM_MESH_H
#define OPENTISSUE_DYNAMICS_FEM_FEM_MESH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

//#include <OpenTissue/core/containers/t4mesh/t4mesh.h>
#include <OpenTissue/dynamics/fem/fem_node_traits.h>
#include <OpenTissue/dynamics/fem/fem_tetrahedron_traits.h>
#include <vector>

namespace OpenTissue
{
  namespace fem
  {

    template <typename math_types>
    class Mesh 
/*      : public OpenTissue::t4mesh::T4Mesh<
                                      math_types
                                      , OpenTissue::fem::detail::NodeTraits<math_types>
                                      , OpenTissue::fem::detail::TetrahedronTraits<math_types> 
                                      >
									  */
    {
    public:



      typedef typename math_types::real_type           real_type;
      typedef typename math_types::vector3_type        vector3_type;
      typedef typename math_types::matrix3x3_type      matrix3x3_type;

	class MyNodeType
    {
    public:

		MyNodeType()
			:m_fixed(false)
		{

		}

		typedef typename std::map<int,matrix3x3_type>  matrix_container;
   
		typedef typename matrix_container::iterator    matrix_iterator;

      typedef typename math_types::vector3_type  vector3_type;
      typedef typename math_types::real_type     real_type;

      vector3_type  m_coord;    ///< Default Coordinate of tetramesh node.
	  vector3_type m_model_coord;
	  vector3_type	m_f_external;
	  vector3_type	m_velocity;
	  real_type	m_mass;
	  bool m_fixed;
	          // Needed by the ConjugateGradient method
        vector3_type m_update;
        vector3_type m_prev;
        vector3_type m_residual;
        matrix_container m_K_row;      ///< Currently stored in a map container, key correspond to column and mapped value to 3-by-3 sub block.
        matrix_container m_A_row;

        vector3_type     m_f0;
        vector3_type     m_b;
        matrix_iterator Kbegin() { return m_K_row.begin(); }
        matrix_iterator Kend()   { return m_K_row.end();   }
        matrix_iterator Abegin() { return m_A_row.begin(); }
        matrix_iterator Aend()   { return m_A_row.end();   }

        matrix3x3_type & K(int const & column_idx) { return m_K_row[column_idx]; }
        matrix3x3_type & A(int const & column_idx) { return m_A_row[column_idx]; }

		int        m_idx;
		int   idx()   const { 
			return m_idx;   
		}

		

    };

	class MyTetrahedronType
	{
	public:

		typedef typename math_types::matrix3x3_type      matrix3x3_type;
        typedef typename math_types::real_type        real_type;
  typedef typename math_types::vector3_type  vector3_type;
    
		real_type	m_young;
		real_type	m_poisson;
		real_type	m_density;
		int	m_nodes[4];


		Mesh* m_owner;
		
        matrix3x3_type m_Ke[4][4];  ///< Stiffness element matrix
        matrix3x3_type m_Re;        ///< Rotational warp of tetrahedron.
        real_type      m_V;         ///< Volume of tetrahedron

        vector3_type m_e10;         ///< edge from p0 to p1
        vector3_type m_e20;         ///< edge from p0 to p2
        vector3_type m_e30;         ///< edge from p0 to p3

        //--- Stuff used exclusive by plastic effects

        vector3_type m_B[4];        ///< placeholders for Jacobian of shapefunctions: B = SN.
        vector3_type m_D;           ///< Elasticity Matrix in vector from
        real_type    m_plastic[6];  ///< Plastic strain tensor.
        real_type    m_yield;
        real_type    m_creep;
        real_type    m_max;

	MyNodeType*	node(int i) 
	{
		return &m_owner->m_nodes[m_nodes[i]];
	}
		
	MyNodeType* i() 
		{
			return &m_owner->m_nodes[m_nodes[0]];
		}
		MyNodeType* j() 
		{
			return &m_owner->m_nodes[m_nodes[1]];
		}

		MyNodeType* k() 
		{
			return &m_owner->m_nodes[m_nodes[2]];
		}
		MyNodeType* m() 
		{
			return &m_owner->m_nodes[m_nodes[3]];
		}

		MyTetrahedronType(int bla, int bla2)
		{

		}

	};


	    typedef std::vector< MyNodeType>        node_container;
        typedef std::vector< MyTetrahedronType > tetrahedra_container;
		typedef MyNodeType	node_type;

	  public:

        node_container           m_nodes;            ///< Internal node storage.

		void	insert(int a,int b,int c,int d)
		{
			MyTetrahedronType t(1,1);
			t.m_owner = this;
			t.m_nodes[0] = a;
			t.m_nodes[1] = b;
			t.m_nodes[2] = c;
			t.m_nodes[3] = d;
			m_tetrahedra.push_back(t);


		}
		void insert()
		{
			MyNodeType n;
			n.m_idx = m_nodes.size();
			m_nodes.push_back(n);

		}
		void	clear() 
		{
			m_nodes.clear();
			m_tetrahedra.clear();
		
		}

        tetrahedra_container     m_tetrahedra;       ///< Internal tetrahedra storage.


    };

  } // namespace fem
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_FEM_FEM_MESH_H
#endif
