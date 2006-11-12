
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

#include "GIMPACT/gim_trimesh_data.h"

GDYNAMIC_ARRAY g_trimesh_data_array;
GDYNAMIC_ARRAY g_trimesh_data_free_positions;


void gim_trimesh_data_manager_init()
{
    GIM_DYNARRAY_CREATE(GIM_TRIMESH_DATA,g_trimesh_data_array,G_ARRAY_GROW_SIZE);
    GIM_DYNARRAY_CREATE(GUINT,g_trimesh_data_free_positions,G_ARRAY_GROW_SIZE);
}

void gim_trimesh_data_manager_end()
{
    //Destroy all trimeshes
    GUINT i,tricount;
    GIM_TRIMESH_DATA * trimeshes = GIM_DYNARRAY_POINTER(GIM_TRIMESH_DATA,g_trimesh_data_array);
    tricount = g_trimesh_data_array.m_size;

    for (i=0;i<tricount;i++)
    {
    	if(GIM_IS_VALID_BUFFER_ID(trimeshes[i].m_source_vertex_buffer.m_buffer_id))
    	{
    	    trimeshes[i].m_ref_count = 0;
    	    gim_trimesh_data_destroy(i);
    	}
    }

    GIM_DYNARRAY_DESTROY(g_trimesh_data_array);
    GIM_DYNARRAY_DESTROY(g_trimesh_data_free_positions);
}

int gim_trimesh_data_is_valid_manager()
{
    if(g_trimesh_data_array.m_pdata == 0 ) return 0;
    return 1;
}

//! Returns the free position (1 based) on the g_trimesh_data_array
GUINT _gim_trimesh_data_get_avaliable_pos()
{
    if(gim_trimesh_data_is_valid_manager()==0) return 0;
    if(g_trimesh_data_free_positions.m_size>0)
    {
        GUINT freeposition = *GIM_DYNARRAY_POINTER_LAST(GUINT,g_trimesh_data_free_positions);
        GIM_DYNARRAY_POP_ITEM(g_trimesh_data_free_positions);
        return freeposition+1;
    }
    GIM_DYNARRAY_PUSH_EMPTY(GIM_TRIMESH_DATA,g_trimesh_data_array);
    return g_trimesh_data_array.m_size;
}

void _gim_trimesh_data_free_handle(GUINT handle)
{
    if(gim_trimesh_data_is_valid_manager()==0) return;
    GUINT freeposition = handle-1;
    GIM_DYNARRAY_PUSH_ITEM(GUINT,g_trimesh_data_free_positions,freeposition);
}



void gim_trimesh_data_get(GUINT trimesh_data_handle,GIM_TRIMESH_DATA ** trimesh_data)
{
    if(gim_trimesh_data_is_valid_manager()==0)
    {
        *trimesh_data = 0;
         return;
    }
    GIM_TRIMESH_DATA * trimeshes = GIM_DYNARRAY_POINTER(GIM_TRIMESH_DATA,g_trimesh_data_array);
    *trimesh_data = trimeshes+(trimesh_data_handle-1);
}

void _gim_gim_trimesh_data_create_empty(GIM_TRIMESH_DATA * tridata)
{
    tridata->m_vertex_scalar_type = G_STYPE_REAL;
    tridata->m_index_scalar_type = G_STYPE_UINT;
    tridata->m_ref_count = 0;
    //Init bvh
    tridata->m_bv_tree.m_node_array = 0;
    tridata->m_bv_tree.m_num_nodes = 0;
    tridata->m_bv_tree.m_root_node = 0;
    tridata->m_bv_tree.m_nodes_array_size = 0;

    //Reference buffer arrays
    tridata->m_source_vertex_buffer.m_buffer_data = 0;
    tridata->m_source_vertex_buffer.m_buffer_id.m_buffer_id = G_UINT_INFINITY;
    tridata->m_source_vertex_buffer.m_element_count = 0;

    tridata->m_tri_index_buffer.m_buffer_data = 0;
    tridata->m_tri_index_buffer.m_buffer_id.m_buffer_id = G_UINT_INFINITY;
    tridata->m_tri_index_buffer.m_element_count = 0;
}



void _gim_gim_trimesh_data_build_from_arrays(GIM_TRIMESH_DATA * tridata,GBUFFER_ARRAY * vertex_array,GUINT vertex_scalar_type, GBUFFER_ARRAY * triindex_array,GUINT index_scalar_type)
{
    tridata->m_vertex_scalar_type = vertex_scalar_type;
    tridata->m_index_scalar_type = index_scalar_type;
    tridata->m_ref_count = 0;
    //Init bvh
    tridata->m_bv_tree.m_node_array = 0;
    tridata->m_bv_tree.m_num_nodes = 0;
    tridata->m_bv_tree.m_root_node = 0;
    tridata->m_bv_tree.m_nodes_array_size = 0;

    //Reference buffer arrays
    gim_buffer_array_copy_ref(vertex_array,&tridata->m_source_vertex_buffer);
    gim_buffer_array_copy_ref(triindex_array,&tridata->m_tri_index_buffer);
}

void _gim_gim_trimesh_data_destroy_data(GIM_TRIMESH_DATA * tridata)
{
    GIM_BUFFER_ARRAY_DESTROY(tridata->m_source_vertex_buffer);
    GIM_BUFFER_ARRAY_DESTROY(tridata->m_tri_index_buffer);
    gim_aabbtree_destroy(&tridata->m_bv_tree);
}

void gim_trimesh_data_create_empty(GUINT * trimesh_data_handle)
{
    *trimesh_data_handle = _gim_trimesh_data_get_avaliable_pos();
    if(*trimesh_data_handle == 0) return ; //an error ocurred
    GIM_TRIMESH_DATA * tridata;

    gim_trimesh_data_get(*trimesh_data_handle,&tridata);

    //init trimesh
    tridata->m_handle = *trimesh_data_handle;

    _gim_gim_trimesh_data_create_empty(tridata);
}

void gim_trimesh_data_create_from_arrays(GUINT * trimesh_data_handle, GBUFFER_ARRAY * vertex_array,GUINT vertex_scalar_type, GBUFFER_ARRAY * triindex_array,GUINT index_scalar_type)
{
    *trimesh_data_handle = _gim_trimesh_data_get_avaliable_pos();
    if(*trimesh_data_handle == 0) return ; //an error ocurred
    GIM_TRIMESH_DATA * tridata;

    gim_trimesh_data_get(*trimesh_data_handle,&tridata);

    //init trimesh
    tridata->m_handle = *trimesh_data_handle;
    //construct
    _gim_gim_trimesh_data_build_from_arrays(tridata,vertex_array,vertex_scalar_type,triindex_array,index_scalar_type);
}

void gim_trimesh_data_create_from_array_data(GUINT * trimesh_data_handle, vec3f * vertex_array, GUINT vertex_count,char copy_vertices, GUINT * triindex_array, GUINT triangle_count,char copy_indices)
{
    if(gim_trimesh_data_is_valid_manager()==0)
    {
        *trimesh_data_handle = 0;
         return;
    }

    GBUFFER_ARRAY buffer_vertex_array;
    GBUFFER_ARRAY buffer_triindex_array;

    //Create vertices
    if(copy_vertices == 1)
    {
        gim_create_common_buffer_from_data(vertex_array, vertex_count*sizeof(vec3f), &buffer_vertex_array.m_buffer_id);
    }
    else//Create a shared buffer
    {
        gim_create_shared_buffer_from_data(vertex_array, vertex_count*sizeof(vec3f), &buffer_vertex_array.m_buffer_id);
    }
    GIM_BUFFER_ARRAY_INIT_TYPE(vec3f,buffer_vertex_array,buffer_vertex_array.m_buffer_id,vertex_count);


    //Create vertices
    if(copy_indices == 1)
    {
        gim_create_common_buffer_from_data(triindex_array, triangle_count*sizeof(vec3ui), &buffer_triindex_array.m_buffer_id);
    }
    else//Create a shared buffer
    {
        gim_create_shared_buffer_from_data(triindex_array, triangle_count*sizeof(vec3ui), &buffer_triindex_array.m_buffer_id);
    }
    GIM_BUFFER_ARRAY_INIT_TYPE(vec3ui,buffer_triindex_array,buffer_triindex_array.m_buffer_id,triangle_count);

    gim_trimesh_data_create_from_arrays(trimesh_data_handle, &buffer_vertex_array,G_STYPE_REAL, &buffer_triindex_array,G_STYPE_UINT);

    ///always call this after create a buffer_array
    GIM_BUFFER_ARRAY_DESTROY(buffer_vertex_array);
    GIM_BUFFER_ARRAY_DESTROY(buffer_triindex_array);
}

void gim_trimesh_data_inc_ref(GUINT trimesh_data_handle)
{
    if(gim_trimesh_data_is_valid_manager()==0)  return;
    GIM_TRIMESH_DATA * tridata;
    gim_trimesh_data_get(trimesh_data_handle,&tridata);
    tridata->m_ref_count++;
}

void gim_trimesh_data_dec_ref(GUINT trimesh_data_handle)
{
    if(gim_trimesh_data_is_valid_manager()==0)  return;
    GIM_TRIMESH_DATA * tridata;
    gim_trimesh_data_get(trimesh_data_handle,&tridata);
    if(tridata->m_ref_count==0)
    {
        //Attemps to destroy the trimesh
        gim_trimesh_data_destroy(trimesh_data_handle);
        return;
    }
    tridata->m_ref_count--;

    if(tridata->m_ref_count==0)
    {
        //Attemps to destroy the trimesh
        gim_trimesh_data_destroy(trimesh_data_handle);
    }
}

void gim_trimesh_data_destroy(GUINT trimesh_data_handle)
{
    if(gim_trimesh_data_is_valid_manager()==0)  return;
    GIM_TRIMESH_DATA * tridata;
    gim_trimesh_data_get(trimesh_data_handle,&tridata);
    if(tridata->m_ref_count>0) return; //Can't destroy it now

    _gim_gim_trimesh_data_destroy_data(tridata);

    //Get a free position
    _gim_trimesh_data_free_handle(trimesh_data_handle);
}

void gim_trimesh_data_lock(GUINT trimesh_data_handle, int access,GIM_TRIMESH_DATA ** trimesh_data)
{
    if(gim_trimesh_data_is_valid_manager()==0)  return;
    gim_trimesh_data_get(trimesh_data_handle,trimesh_data);
    gim_buffer_array_lock(&(*trimesh_data)->m_source_vertex_buffer,access);
    gim_buffer_array_lock(&(*trimesh_data)->m_tri_index_buffer,access);
}



void gim_trimesh_data_unlock(GUINT trimesh_data_handle)
{
    if(gim_trimesh_data_is_valid_manager()==0)  return;
    GIM_TRIMESH_DATA * tridata;
    gim_trimesh_data_get(trimesh_data_handle,&tridata);
    gim_buffer_array_unlock(&tridata->m_source_vertex_buffer);
    gim_buffer_array_unlock(&tridata->m_tri_index_buffer);
}

GUINT gim_trimesh_data_get_vertex_count(GUINT trimesh_data_handle)
{
    GIM_TRIMESH_DATA * tridata;
    gim_trimesh_data_get(trimesh_data_handle,&tridata);
    return tridata->m_source_vertex_buffer.m_element_count;
}

GUINT gim_trimesh_data_get_triangle_count(GUINT trimesh_data_handle)
{
    GIM_TRIMESH_DATA * tridata;
    gim_trimesh_data_get(trimesh_data_handle,&tridata);
    return tridata->m_tri_index_buffer.m_element_count;
}

void gim_trimesh_data_get_triangle_indices(GIM_TRIMESH_DATA * trimesh_data, GUINT tri_index, GUINT * indices)
{
    if(trimesh_data->m_index_scalar_type == G_STYPE_UINT)
    {
        GUINT * uipindices = GIM_BUFFER_ARRAY_POINTER(GUINT,trimesh_data->m_tri_index_buffer,tri_index);
        indices[0] = uipindices[0];
        indices[1] = uipindices[1];
        indices[2] = uipindices[2];
    }
    else if(trimesh_data->m_index_scalar_type == G_STYPE_INT)
    {
        GINT * ipindices = GIM_BUFFER_ARRAY_POINTER(GINT,trimesh_data->m_tri_index_buffer,tri_index);
        indices[0] = ipindices[0];
        indices[1] = ipindices[1];
        indices[2] = ipindices[2];
    }
    else if(trimesh_data->m_index_scalar_type == G_STYPE_USHORT)
    {
        GUSHORT * uspindices = GIM_BUFFER_ARRAY_POINTER(GUSHORT,trimesh_data->m_tri_index_buffer,tri_index);
        indices[0] = uspindices[0];
        indices[1] = uspindices[1];
        indices[2] = uspindices[2];
    }
    else if(trimesh_data->m_index_scalar_type == G_STYPE_SHORT)
    {
        GSHORT * spindices = GIM_BUFFER_ARRAY_POINTER(GSHORT,trimesh_data->m_tri_index_buffer,tri_index);
        indices[0] = spindices[0];
        indices[1] = spindices[1];
        indices[2] = spindices[2];
    }
}

void gim_trimesh_data_get_vertex(GIM_TRIMESH_DATA * trimesh_data, GUINT vertex_index, vec3f vec)
{
    if(trimesh_data->m_vertex_scalar_type == G_STYPE_REAL)
    {
        GREAL * rvalues = GIM_BUFFER_ARRAY_POINTER(GREAL,trimesh_data->m_source_vertex_buffer,vertex_index);
        vec[0] = rvalues[0];
        vec[1] = rvalues[1];
        vec[2] = rvalues[2];
    }
    else if(trimesh_data->m_vertex_scalar_type == G_STYPE_REAL2)
    {
        GREAL2 * r2values = GIM_BUFFER_ARRAY_POINTER(GREAL2,trimesh_data->m_source_vertex_buffer,vertex_index);
        vec[0] = r2values[0];
        vec[1] = r2values[1];
        vec[2] = r2values[2];
    }
}

void gim_trimesh_data_get_triangle_vertices(GIM_TRIMESH_DATA * trimesh_data, GUINT tri_index, vec3f v1, vec3f v2, vec3f v3)
{
    GUINT tri_indices[3];
    gim_trimesh_data_get_triangle_indices(trimesh_data,tri_index,tri_indices);

    if(trimesh_data->m_vertex_scalar_type == G_STYPE_REAL)
    {
        GREAL * rvalues = GIM_BUFFER_ARRAY_POINTER(GREAL,trimesh_data->m_source_vertex_buffer,tri_indices[0]);
        VEC_COPY(v1,rvalues);

        rvalues = GIM_BUFFER_ARRAY_POINTER(GREAL,trimesh_data->m_source_vertex_buffer,tri_indices[1]);
        VEC_COPY(v2,rvalues);

        rvalues = GIM_BUFFER_ARRAY_POINTER(GREAL,trimesh_data->m_source_vertex_buffer,tri_indices[2]);
        VEC_COPY(v3,rvalues);
    }
    else if(trimesh_data->m_vertex_scalar_type == G_STYPE_REAL2)
    {
        GREAL2 * r2values = GIM_BUFFER_ARRAY_POINTER(GREAL2,trimesh_data->m_source_vertex_buffer,tri_indices[0]);
        VEC_COPY(v1,r2values);

        r2values = GIM_BUFFER_ARRAY_POINTER(GREAL2,trimesh_data->m_source_vertex_buffer,tri_indices[1]);
        VEC_COPY(v2,r2values);

        r2values = GIM_BUFFER_ARRAY_POINTER(GREAL2,trimesh_data->m_source_vertex_buffer,tri_indices[2]);
        VEC_COPY(v3,r2values);
    }
}

void gim_trimesh_data_build_aabbtree(GUINT trimesh_data_handle)
{
    if(gim_trimesh_data_is_valid_manager()==0)  return;
    GIM_TRIMESH_DATA * tridata;
    gim_trimesh_data_lock(trimesh_data_handle, G_MA_READ_ONLY,&tridata);

    GUINT i,tricount;
    tricount = gim_trimesh_data_get_triangle_count(trimesh_data_handle);
    GIM_AABB_DATA * boxarray = (GIM_AABB_DATA *)gim_alloc(tricount*sizeof(GIM_AABB_DATA));

    vec3f trivertices[3];

    for (i=0;i<tricount ;i++ )
    {
        gim_trimesh_data_get_triangle_vertices(tridata,i,trivertices[0],trivertices[1],trivertices[2]);
        COMPUTEAABB_FOR_TRIANGLE(boxarray[i].m_aabb,trivertices[0],trivertices[1],trivertices[2]);
        boxarray[i].m_data = i;
    }
    gim_trimesh_data_unlock(trimesh_data_handle);

    gim_aabbtree_create(&tridata->m_bv_tree,boxarray,tricount);
    gim_free(boxarray,0);
}

int gim_trimesh_data_has_bv_tree(GUINT trimesh_data_handle)
{
    GIM_TRIMESH_DATA * tridata;
    gim_trimesh_data_get(trimesh_data_handle,&tridata);

    if(tridata->m_bv_tree.m_node_array == 0) return 0;
    return 1;
}


void gim_trimesh_data_copy(GUINT source_trimesh_data,GUINT dest_trimesh_data, char copy_by_reference)
{
    if(gim_trimesh_data_is_valid_manager()==0)  return;
    GIM_TRIMESH_DATA * sourcetridata;
    gim_trimesh_data_get(source_trimesh_data,&sourcetridata);

    GIM_TRIMESH_DATA * desttridata;
    gim_trimesh_data_get(dest_trimesh_data,&desttridata);

    _gim_gim_trimesh_data_destroy_data(desttridata);//Clear previus data

    if(copy_by_reference==1)
    {
        _gim_gim_trimesh_data_build_from_arrays(desttridata, &sourcetridata->m_source_vertex_buffer,
        sourcetridata->m_vertex_scalar_type, &sourcetridata->m_tri_index_buffer,
        sourcetridata->m_index_scalar_type);
    }
    else
    {
        GBUFFER_ARRAY buffer_vertex_array;
        GBUFFER_ARRAY buffer_triindex_array;

        gim_buffer_array_copy_value(&sourcetridata->m_source_vertex_buffer,&buffer_vertex_array,G_BUFFER_MANAGER_SYSTEM,G_MU_DYNAMIC_READ_WRITE);

        gim_buffer_array_copy_value(&sourcetridata->m_tri_index_buffer,&buffer_triindex_array,G_BUFFER_MANAGER_SYSTEM,G_MU_DYNAMIC_READ_WRITE);

        _gim_gim_trimesh_data_build_from_arrays(desttridata, &buffer_vertex_array,
        sourcetridata->m_vertex_scalar_type, &buffer_triindex_array,
        sourcetridata->m_index_scalar_type);

        ///always call this after create a buffer_array
        GIM_BUFFER_ARRAY_DESTROY(buffer_vertex_array);
        GIM_BUFFER_ARRAY_DESTROY(buffer_triindex_array);
    }

    //If it has a Bounding volume tree, construct it

    if(gim_trimesh_data_has_bv_tree(source_trimesh_data))
    {
        gim_trimesh_data_build_aabbtree(dest_trimesh_data);
    }
}


void gim_trimesh_data_create_copy(GUINT source_trimesh_data,GUINT * dest_trimesh_data, char copy_by_reference)
{
    if(gim_trimesh_data_is_valid_manager()==0)  return;
    gim_trimesh_data_create_empty(dest_trimesh_data);
    gim_trimesh_data_copy(source_trimesh_data,*dest_trimesh_data, copy_by_reference);
}
