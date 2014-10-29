#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_BLOCK_GENERATOR_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_BLOCK_GENERATOR_H
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
  namespace t4mesh
  {


    /**
    * t4mesh Block Generator.
    *
    * @param I               The number of blocks along x axis.
    * @param J               The number of blocks along y axis.
    * @param K               The number of blocks along z axis.
    * @param block_width     The edgelength of the blocks along x-axis.
    * @param block_height    The edgelength of the blocks along x-axis.
    * @param block_depth     The edgelength of the blocks along x-axis.
    * @param mesh            A generic t4mesh, which upon return holds the generated mesh.
    */
    template < typename real_type, typename t4mesh_type >
    void generate_blocks(
      unsigned int const & I
      , unsigned int const & J
      , unsigned int const & K
      , real_type const & block_width
      , real_type const & block_height
      , real_type const & block_depth
      , t4mesh_type & mesh
      )
    {

      mesh.clear();

      unsigned int numVertices = (I + 1) * (J + 1) * (K + 1);
      for (unsigned int i=0; i<numVertices; ++i)
        mesh.insert();

	  int node_it = 0;
      for (unsigned int x = 0; x <= I; ++x)
      {
        for (unsigned int y = 0; y <= J; ++y)
        {
          for (unsigned int z = 0; z <= K; ++z)
          {
			  
            mesh.m_nodes[node_it].m_coord(0) = block_width*x;
            mesh.m_nodes[node_it].m_coord(2) = block_depth*y;
            mesh.m_nodes[node_it].m_coord(1) = block_height*z;
            ++node_it;
          }
        }
      }
      for (unsigned int i = 0; i < I; ++i)
      {
        for (unsigned int j = 0; j < J; ++j)
        {
          for (unsigned int k = 0; k < K; ++k)
          {
            // For each block, the 8 corners are numerated as:
            //     4*-----*7
            //     /|    /|
            //    / |   / |
            //  5*-----*6 |
            //   | 0*--|--*3
            //   | /   | /
            //   |/    |/
            //  1*-----*2
            int p0 = (i * (J + 1) + j) * (K + 1) + k;
            int p1 = p0 + 1;
            int p3 = ((i + 1) * (J + 1) + j) * (K + 1) + k;
            int p2 = p3 + 1;
            int p7 = ((i + 1) * (J + 1) + (j + 1)) * (K + 1) + k;
            int p6 = p7 + 1;
            int p4 = (i * (J + 1) + (j + 1)) * (K + 1) + k;
            int p5 = p4 + 1;
            
            // Ensure that neighboring tetras are sharing faces
            if ((i + j + k) % 2 == 1)
            {
              mesh.insert(p1,p2,p6,p3);
              mesh.insert(p3,p6,p4,p7);
              mesh.insert(p1,p4,p6,p5);
              mesh.insert(p1,p3,p4,p0);
              mesh.insert(p1,p6,p4,p3);
            }
            else
            {
              mesh.insert(p2,p0,p5,p1);
              mesh.insert(p2,p7,p0,p3);
              mesh.insert(p2,p5,p7,p6);
              mesh.insert(p0,p7,p5,p4);
              mesh.insert(p2,p0,p7,p5);
            }
          }
        }
      }
    };


  } // namespace t4mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_BLOCK_GENERATOR_H
#endif
