#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_T4BOUNDARY_FACES_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_T4BOUNDARY_FACES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <list>
#include <cassert>

namespace OpenTissue
{
  namespace t4mesh
  {

    /**
    * t4mesh Face.
    */
    template<typename M, typename F>
    class T4Face : public F
    {
    public:

      typedef M                               mesh_type;
      typedef F                               face_traits;
      typedef T4Face<M,F>                     face_type;
      typedef typename mesh_type::index_type  index_type;

    protected:

      index_type m_idx0;         ///< First node index.
      index_type m_idx1;         ///< Second node index.
      index_type m_idx2;         ///< Third node index.

    public:

      T4Face()
        : m_idx0(-1)
        , m_idx1(-1)
        , m_idx2(-1)
      {}

      T4Face(index_type const & index0, index_type const & index1, index_type const & index2)
        : m_idx0(index0)
        , m_idx1(index1)
        , m_idx2(index2)
      {}

    public:

      const index_type & idx0() const { return m_idx0; }
      const index_type & idx1() const { return m_idx1; }
      const index_type & idx2() const { return m_idx2; }

    };

    /**
    * Tetrahedra Mesh Boundary Faces.
    * Note that if subsequent changes are made to the t4mesh are not reflected in
    * the faces stored in this class. Meaning that a new face-queury
    * must be initiated everytime the t4mesh changes its topology.
    *
    * Example of usage:
    *
    *  typedef t4mesh<...>  MyMeshType;
    *  MyMeshType mymesh;
    *  ...
    *  class MyFaceTraits : public DefaultFaceTraits
    *  {
    *    public:
    *       Color m_color;
    *       ...
    *  };
    *  typedef T4BoundaryFaces<MyMeshType,MyFaceTraits> MyBoundaryFaces;
    *  MyBoundaryFaces bounday(mymesh);
    *  for(MyBoundaryFaces::face_iterator face=boundary.begin();face!=boundary.end();++face)
    *  {
    *     face->m_color = Color::white;
    *     std::cout << face->idx0() << std::endl;
    *  }
    */
    template<class M,typename F>
    class T4BoundaryFaces
    {
    public:

      typedef M                                        mesh_type;
      typedef F                                        face_traits;
      typedef T4Face<M,F>                              face_type;
      typedef std::list<face_type>                     face_container;
      typedef typename face_container::iterator        face_iterator;
      typedef typename face_container::const_iterator  const_face_iterator;

    protected:

      face_container m_faces;           ///< Container of extrated boundary faces.

    public:

      face_iterator       begin()       { return m_faces.begin(); }
      face_iterator       end()         { return m_faces.end(); }
      const_face_iterator begin() const { return m_faces.begin(); }
      const_face_iterator end()   const { return m_faces.end(); }

    public:

      /**
      * Default Constructor.
      * Constructs an empty set of boundary faces.
      */
      T4BoundaryFaces() 
        : m_faces() 
      {}

      /**
      * Specialezed Constructor
      * Traverses the tetrahedral mesh, and extracts all boundary faces. A boundary
      * face is a face that only have one neighboring tetrahedron. A face inside
      * the tetrahedral mesh will have exactly two neighboring tetrahedra.
      *
      * Face node indices are given in CCW order.
      *
      * @param mesh       The tetrahedral mesh from which boundary
      *                   faces are extracted.
      */
      T4BoundaryFaces(mesh_type & mesh) 
        : m_faces()
      {
        typename mesh_type::tetrahedron_iterator tetrahedron;
        for( tetrahedron = mesh.tetrahedron_begin(); tetrahedron != mesh.tetrahedron_end(); ++tetrahedron)
        {
          if(tetrahedron->jkm()==mesh.tetrahedron_end())
            m_faces.push_back(face_type(tetrahedron->j()->idx(),tetrahedron->k()->idx(),tetrahedron->m()->idx()));
          if(tetrahedron->ijm()==mesh.tetrahedron_end())
            m_faces.push_back(face_type(tetrahedron->i()->idx(),tetrahedron->j()->idx(),tetrahedron->m()->idx()));
          if(tetrahedron->kim()==mesh.tetrahedron_end())
            m_faces.push_back(face_type(tetrahedron->k()->idx(),tetrahedron->i()->idx(),tetrahedron->m()->idx()));
          if(tetrahedron->ikj()==mesh.tetrahedron_end())
            m_faces.push_back(face_type(tetrahedron->i()->idx(),tetrahedron->k()->idx(),tetrahedron->j()->idx()));
        }
      }

    };

  } // namespace t4mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_T4BOUNDARY_FACES_H
#endif
