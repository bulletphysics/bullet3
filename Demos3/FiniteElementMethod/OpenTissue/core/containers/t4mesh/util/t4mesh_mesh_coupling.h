#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_MESH_COUPLING_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_MESH_COUPLING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>


#include <OpenTissue/collision/spatial_hashing/spatial_hashing.h>

namespace OpenTissue
{

  namespace t4mesh
  {

    namespace mesh_coupling
    {

      template<typename surface_mesh,typename volume_mesh>
      class collision_policy
      {
      public:

        typedef typename surface_mesh::vertex_type                  vertex_type;
        typedef typename volume_mesh::tetrahedron_type              tetrahedron_type;

        typedef double                                              real_type;
        typedef math::Vector3<real_type>                            point_type;

        typedef vertex_type*               data_type;
        typedef tetrahedron_type           query_type;

        typedef OpenTissue::spatial_hashing::GridHashFunction                        hash_function;
        typedef OpenTissue::spatial_hashing::Grid< point_type, math::Vector3<int>, data_type, hash_function>  hash_grid;

        class result_type
        {
        public:
          vertex_type * m_data;
          tetrahedron_type * m_query;
          real_type  m_w0;
          real_type  m_w1;
          real_type  m_w2;
          real_type  m_w3;
        };

        typedef std::list<result_type>  result_container;

      public:

        point_type position(data_type const & data) const 
        {
          return data->m_coord;
        }

        point_type min_coord(query_type const & query) const 
        {
          using std::min;

          point_type & p0 = query.i()->m_coord;
          point_type & p1 = query.j()->m_coord;
          point_type & p2 = query.k()->m_coord;
          point_type & p3 = query.m()->m_coord;
          return min( p0, min( p1 , min( p2, p3) ) );
        }

        point_type max_coord(query_type const & query) const 
        {
          using std::max;

          point_type & p0 = query.i()->m_coord;
          point_type & p1 = query.j()->m_coord;
          point_type & p2 = query.k()->m_coord;
          point_type & p3 = query.m()->m_coord;
          return max( p0, max( p1 , max( p2, p3) ) );
        }

        void reset(result_container & results)  {    results.clear();  };
        void report(data_type const & data, query_type const & query,result_container & results)
        {
          //--- First we do a quick rejection test. If the vertex is allready reported then simply ignore it!!!
          if(data->m_tag == 1)
            return;

          point_type & pi = query.i()->m_coord;
          point_type & pj = query.j()->m_coord;
          point_type & pk = query.k()->m_coord;
          point_type & pm = query.m()->m_coord;
          point_type & p  = data->m_coord;

          real_type delta = 10e-5;
          real_type lower = - delta;
          real_type upper = 1.+ delta;
          result_type result;
          OpenTissue::geometry::barycentric_algebraic(pi,pj,pk,pm,p,result.m_w0,result.m_w1,result.m_w2,result.m_w3);
          if(
            (result.m_w0>lower)&&(result.m_w0<upper)
            &&
            (result.m_w1>lower)&&(result.m_w1<upper)
            &&
            (result.m_w2>lower)&&(result.m_w2<upper)
            &&
            (result.m_w3>lower)&&(result.m_w3<upper)
            )
          {
            data->m_tag = 1;
            result.m_data = const_cast<vertex_type*>( data );
            result.m_query = const_cast<tetrahedron_type*>( &query );
            results.push_back( result );
            return;
          }
        }
      };

      /**
      * Bind Mesh Surface to t4mesh. 
      *
      *
      * @param M   A surface mesh that should be bound to a volume mesh.
      * @param V   A T4Mesh it is implicitly assumed that node traits have a
      *            real_type and vector3_type (these are defined in the
      *            default node traits).
      */
      template<typename surface_mesh, typename volume_mesh, typename point_container,typename index_container>
      void bind_surface (surface_mesh & M,volume_mesh & V,point_container & barycentric,index_container & bind_indices)
      {

        //---
        //--- The main idea behind mesh coupling is to create a multiresolution for the
        //--- animataion. The idea is to separate the visual geometry from the geometry
        //--- used to compute the dynamics. This allows one to use highly detailed visual
        //--- representation of objects, while using a computational low cost coarse volume
        //--- mesh for the computing the dynamcis.
        //---
        //--- A technique for doing this is callled mesh coupling or cartoon meshing. Below
        //--- we describe how it is used together with tetrahedral meshes. It is however a
        //--- general approach and can be used with other types of geometries, FFD lattices
        //--- are probably anohter very common example on mesh coupling.
        //---
        //--- When using Mesh Coulping the first step is to bind the vertices of the surface
        //--- mesh to the tetrahedral elements of the volume mesh.
        //---
        //--- Here we use a spatial hashing algorithm to find vertex tetrahedron pairs, where
        //--- the vertex is embedded inside the tetrahedron. The actual test is done by first
        //--- computing the barycentric coordinates of the vertex with respect to a tetrahedron
        //--- in question.
        //---
        //--- If each barycentric coordinate is greather than equal to zero and less than equal
        //--- to one then the vertex is embedded in the tetrahedron. In pratice we need to apply
        //--- treshold testing to counter numerical precision problems. It may therefor happen
        //--- that vertices lying close to a face of a tetrahedron gets reported twice. Once of
        //--- the tetrahedron embedding it and once for the neighboring tetetrahedron. The same
        //--- happens if the vertex lies exactly on a face.
        //---
        //--- Therefore, we firstdo a quick rejection test. If the vertex is allready reported
        //--- then simply ignore it!!!
        //---
        //--- Before rendering each frame, we must update the vertex positions to reflect the
        //--- underlying deformation of the tetrahedral mesh. This is done using the barycentric
        //--- coordinates, such that the new vertex position is given by
        //---
        //---    c = w0 p0 + w1 p1 + w2 p2 + w3 p3
        //---
        //---  Where p0, p1 ,p2, and p3 are the nodal coordinates of the tetrahedron which
        //--- the vertex was bounded to.
        //---
        //--- If stiffness warping is used, the element rotation, Re, can be
        //--- used to update the undeformed vertex normal, n0, into the deformed
        //--- vertex normal, n, by,
        //---
        //---    n = Re n0
        //---
        //--- Often a tetrahedra mesh is used with a conservative coverage of the surface mesh.
        //--- That means one is guaranteed that all vertices of the surface mesh are embedded
        //--- inside one unique tetrahedron. However, mesh coupling can be used in cases where
        //--- one only have a partial coverage. The solution is to bind a vertex to the
        //--- closest tetrahedron. Eventhough the vertex lies outside the tetrahedra mesh, the
        //--- barycentric coordinates extend the deformation of the tetrahedra mesh beyond
        //--- its surface.
        //---
        typedef collision_policy<surface_mesh,volume_mesh> policy;
        typedef OpenTissue::spatial_hashing::PointDataQuery<typename policy::hash_grid, policy> point_query_type;
        typename policy::result_container   results;
        point_query_type point_query;
        point_query.auto_init_settings(V.tetrahedron_begin(),V.tetrahedron_end());
        //--- perform query
        mesh::clear_vertex_tags(M);

        std::vector<typename surface_mesh::vertex_type*>  vertex_ptr_container(M.size_vertices());
        unsigned int i = 0;
        for(typename surface_mesh::vertex_iterator v= M.vertex_begin();v!=M.vertex_end();++v,++i)
        {
          vertex_ptr_container[i] = &(*v);
        }

        point_query(
          vertex_ptr_container.begin()
          , vertex_ptr_container.end()
          , V.tetrahedron_begin()
          , V.tetrahedron_end()
          , results
          , typename point_query_type::all_tag()
          );

        unsigned int size = static_cast<unsigned int>( results.size() );
        barycentric.resize(size);
        bind_indices.resize(size);
        typename policy::result_container::iterator Rbegin = results.begin();
        typename policy::result_container::iterator Rend   = results.end();
        for(typename policy::result_container::iterator R = Rbegin;R!=Rend;++R)
        {
          unsigned int i = static_cast<unsigned int>( R->m_data->get_handle().get_idx() );

          barycentric[i](0) = R->m_w1;
          barycentric[i](1) = R->m_w2;
          barycentric[i](2) = R->m_w3;
          bind_indices[i] = R->m_query->idx();
        }
      };

      /**
      * Update Surface Mesh.
      *
      *
      * @param M               A surface mesh that is bound to a volume mesh.
      * @param V               The volume mesh that the surface is bound to. It
      *                        is implicitly assumed that the node traits define
      *                        a real_type and vector3_type ((these are defined in
      *                        the default node traits).
      * @param barycentric
      * @param bind_info
      */
      template<typename surface_mesh, typename volume_mesh, typename point_container,typename index_container>
      void update_surface (
        surface_mesh & M,
        volume_mesh  & V,
        point_container const & barycentric,
        index_container const & bind_info
        )
      {
        typename surface_mesh::vertex_iterator begin = M.vertex_begin();
        typename surface_mesh::vertex_iterator end   = M.vertex_end();

        typedef typename volume_mesh::tetrahedron_iterator tetrahedron_iterator;
        typedef typename volume_mesh::node_type            node_type;
        typedef typename node_type::real_type              real_type;
        typedef typename node_type::vector3_type           vector3_type;

        for( typename surface_mesh::vertex_iterator v = begin; v!=end; ++v)
        {
          unsigned int i = static_cast<unsigned int>( v->get_handle().get_idx() );
          tetrahedron_iterator T = V.tetrahedron( bind_info[i] );

          real_type w1 = barycentric[i](0);
          real_type w2 = barycentric[i](1);
          real_type w3 = barycentric[i](2);
          real_type w0 = 1 - w1 - w2 - w3;

          vector3_type & p0 = T->i()->m_coord;
          vector3_type & p1 = T->j()->m_coord;
          vector3_type & p2 = T->k()->m_coord;
          vector3_type & p3 = T->m()->m_coord;
          v->m_coord = p0*w0 + p1*w1 + p2*w2 + p3*w3;

        }
      };

    } // namespace mesh_coupling

  } // namespace t4mesh

} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_MESH_COUPLING_H
#endif
