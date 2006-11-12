#ifndef GIM_TRIMESH_DATA_H_INCLUDED
#define GIM_TRIMESH_DATA_H_INCLUDED
/*! \file gim_trimesh_data.h
\author Francisco León Nájera
*/
/*
-----------------------------------------------------------------------------
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com

 This library is free software; you can redistribute it and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.
   (3) The zlib/libpng license that is included with this library in
       the file GIMPACT-LICENSE-ZLIB.TXT.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-ZLIB.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/

#include "GIMPACT/gim_boxpruning.h"

/*! \addtogroup TRIMESH_DATA
\brief


*/
//! @{

//! Class for managing Mesh data
struct GIM_TRIMESH_DATA
{
    GUINT m_vertex_scalar_type;//!< G_STYPE_REAL or G_STYPE_REAL2
    GUINT m_index_scalar_type;//!< G_STYPE_INT,G_STYPE_UINT, G_STYPE_SHORT or G_STYPE_USHORT

    GBUFFER_ARRAY m_source_vertex_buffer;//!< Buffer of vec3f coordinates

    //! vec3ui,vec3i,vec3s or vec3us Indices of triangles,groups of three elements.
    /*!
    Array of Triangle indices. Each triple contains indices of the vertices for each triangle.
    Array size = Triangle count
    */
    GBUFFER_ARRAY m_tri_index_buffer;

    GIM_AABB_TREE m_bv_tree;

    GUINT  m_ref_count;//!< Refvertex_scalar_typeerence counting
    GUINT m_handle;
};

//Functions for trimesh data
//Trimesh data manager

void gim_trimesh_data_manager_init();
void gim_trimesh_data_manager_end();
int gim_trimesh_data_is_valid_manager();

//! Test if the trimesh data has bounding volume tree
int gim_trimesh_data_has_bv_tree(GUINT trimesh_data_handle);

/*!
\param trimesh_data_handle Must be superior to 0
*/
void gim_trimesh_data_get(GUINT trimesh_data_handle,GIM_TRIMESH_DATA ** trimesh_data);


//! Creates  an empty trimesh
void gim_trimesh_data_create_empty(GUINT * trimesh_data_handle);


//! Create a trimesh from vertex array and an index array
/*!
\param trimesh_data_handle A pointer to receive the trimesh handle
\param vertex_array A buffer to a vec3f array
\param vertex_count
\param triindex_array
\param index_count
\param copy_vertices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
\param copy_indices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
*/
void gim_trimesh_data_create_from_arrays(GUINT * trimesh_data_handle, GBUFFER_ARRAY * vertex_array,GUINT vertex_scalar_type, GBUFFER_ARRAY * triindex_array,GUINT index_scalar_type);

//! Create a trimesh from vertex array and an index array
/*!
\param trimesh_data_handle A pointer to receive the trimesh handle
\param vertex_array A buffer to a vec3f array
\param vertex_count
\param triindex_array
\param index_count
\param copy_vertices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
\param copy_indices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
*/
void gim_trimesh_data_create_from_array_data(GUINT * trimesh_data_handle, vec3f * vertex_array, GUINT vertex_count,char copy_vertices, GUINT * triindex_array,  GUINT triangle_count,char copy_indices);

//! Increase the reference to a GIM_TRIMESH_DATA object
/*!
Use this function for prevents destroying the trimesh data when it stills
is being used. If you have a singleton scene manager, increase the references to all trimesh data resources.
*/
void gim_trimesh_data_inc_ref(GUINT trimesh_data_handle);

//! Decrease the reference to a GIM_TRIMESH_DATA object, and attempts to destroy it
/*!
Use this function for safe destruction of the trimesh data. If the reference of the trimesh is bigger than 1, then
it doen't destroy the trimesh data.
*/
void gim_trimesh_data_dec_ref(GUINT trimesh_data_handle);


void gim_trimesh_data_destroy(GUINT trimesh_data_handle);

/*!
Locks m_source_vertex_buffer and m_tri_index_buffer
*/
void gim_trimesh_data_lock(GUINT trimesh_data_handle, int access,GIM_TRIMESH_DATA ** trimesh_data);

/*!
Unlocks m_source_vertex_buffer and m_tri_index_buffer
*/
void gim_trimesh_data_unlock(GUINT trimesh_data_handle);

GUINT gim_trimesh_data_get_vertex_count(GUINT trimesh_data_handle);
GUINT gim_trimesh_data_get_triangle_count(GUINT trimesh_data_handle);

//! Get triangle indices
/*!
\pre trimesh_data must be locked
*/
void gim_trimesh_data_get_triangle_indices(GIM_TRIMESH_DATA * trimesh_data, GUINT tri_index, GUINT * indices);

//! Get trimesh vertex
/*!
\pre trimesh_data must be locked
*/
void gim_trimesh_data_get_vertex(GIM_TRIMESH_DATA * trimesh_data, GUINT vertex_index, vec3f vec);

//! Get triangle vertices
/*!
\pre trimesh_data must be locked
*/
void gim_trimesh_data_get_triangle_vertices(GIM_TRIMESH_DATA * trimesh_data, GUINT tri_index, vec3f v1, vec3f v2, vec3f v3);

//! Builds a Bounding Volume tree from the Trimesh data
void gim_trimesh_data_build_aabbtree(GUINT trimesh_data_handle);

//! Copies two meshes
/*!
\param source_trimesh_data
\param dest_trimesh_data
\param copy_by_reference If 1, it attach a reference to the source vertices, else it copies the vertices
\param transformed_reply If 1, transformed vertices are reply of source vertives. 1 Is recommended
*/
void gim_trimesh_data_copy(GUINT source_trimesh_data,GUINT dest_trimesh_data, char copy_by_reference);

//! Copies two meshes
/*!
\param source_trimesh_data
\param dest_trimesh_data
\param copy_by_reference If 1, it attach a reference to the source vertices, else it copies the vertices
\param transformed_reply If 1, transformed vertices are reply of source vertives. 1 Is recommended
*/
void gim_trimesh_data_create_copy(GUINT source_trimesh_data,GUINT * dest_trimesh_data, char copy_by_reference);

//! @}



#endif // GIM_TRIMESH_DATA_H_INCLUDED
