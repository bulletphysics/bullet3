
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


#include "GIMPACT/gim_trimesh.h"
#include "assert.h"

GUINT gim_trimesh_get_triangle_count(GIM_TRIMESH * trimesh)
{
    return gim_trimesh_data_get_triangle_count(trimesh->m_trimesh_data_handle);
}

GUINT gim_trimesh_get_vertex_count(GIM_TRIMESH * trimesh)
{
    return gim_trimesh_data_get_vertex_count(trimesh->m_trimesh_data_handle);
}

GUINT gim_trimesh_get_bound_method(GIM_TRIMESH * trimesh)
{
    if(gim_trimesh_data_has_bv_tree(trimesh->m_trimesh_data_handle) != 0)  return G_TRIMESH_BOUND_AABB_TREE;
    else if(trimesh->m_aabbset != 0)   return G_TRIMESH_BOUND_AABB_SET;
    return G_TRIMESH_BOUND_NONE;
}


#define LAZY_TRANSFORM_VERTEX(bitset,vec_index,transform,source_trimesh_data,dest_vert,temp_vert)\
{\
    GUINT bit_eval;\
    if(GIM_DYNARRAY_IS_VALID(bitset))\
    {\
        GIM_BITSET_GET(bitset,vec_index,bit_eval);\
        if(bit_eval == 0)\
        {\
            gim_trimesh_data_get_vertex(source_trimesh_data,vec_index,vec);\
            MAT_DOT_VEC_3X4(dest_vert,transform,temp_vert);\
        }\
    }\
}\




//! Fetch a single transformed vertex
/*!
\pre gim_trimesh_locks_work_data must be called before
*/
void gim_trimesh_get_vertex(GIM_TRIMESH * trimesh, GUINT vertex_index, vec3f vec)
{
    if(trimesh->m_cache)
    {
        if(GIM_BUFFER_ARRAY_IS_VALID(trimesh->m_cache->m_transformed_vertex_buffer))
        {
			GREAL * transvecf = GIM_BUFFER_ARRAY_POINTER(GREAL,trimesh->m_cache->m_transformed_vertex_buffer,vertex_index);
            LAZY_TRANSFORM_VERTEX(trimesh->m_cache->m_transformed_vertices_cache_bitset,vertex_index,trimesh->m_transform,trimesh->m_unlocked_trimesh_data,transvecf,vec);
            VEC_COPY(vec,transvecf);
            return;
        }
    }

    if(gim_trimesh_has_tranform(trimesh))
    {
        vec3f vectemp;
        gim_trimesh_data_get_vertex(trimesh->m_unlocked_trimesh_data,vertex_index,vectemp);
        MAT_DOT_VEC_3X4(vec,trimesh->m_transform,vectemp);
        return;
    }
    gim_trimesh_data_get_vertex(trimesh->m_unlocked_trimesh_data,vertex_index,vec);
}

void gim_trimesh_get_triangle_vertices_local(GIM_TRIMESH * trimesh, GUINT triangle_index,
											 vec3f v1,vec3f v2,vec3f v3)
{
	vec3ui triangle_indices;
    gim_trimesh_data_get_triangle_indices(
    trimesh->m_unlocked_trimesh_data,triangle_index,triangle_indices);

	gim_trimesh_data_get_vertex(trimesh->m_unlocked_trimesh_data,triangle_indices[0],v1);
	gim_trimesh_data_get_vertex(trimesh->m_unlocked_trimesh_data,triangle_indices[1],v2);
	gim_trimesh_data_get_vertex(trimesh->m_unlocked_trimesh_data,triangle_indices[2],v3);
}

void gim_trimesh_get_vertex_local(GIM_TRIMESH * trimesh, GUINT vertex_index, vec3f vec)
{
	gim_trimesh_data_get_vertex(trimesh->m_unlocked_trimesh_data,vertex_index,vec);
}


void gim_trimesh_get_triangle_vertices(GIM_TRIMESH * trimesh, GUINT triangle_index, vec3f v1,vec3f v2,vec3f v3)
{
	if(gim_trimesh_has_tranform(trimesh)==0)
	{
		gim_trimesh_get_triangle_vertices_local(trimesh,triangle_index,v1,v2,v3);
		return;
	}

    vec3ui triangle_indices;
    gim_trimesh_data_get_triangle_indices(
    trimesh->m_unlocked_trimesh_data,triangle_index,triangle_indices);
	if(trimesh->m_aabbset)
	{
		vec3f * vertices = GIM_BUFFER_ARRAY_POINTER(vec3f,trimesh->m_cache->m_transformed_vertex_buffer,0);
		VEC_COPY(v1,vertices[triangle_indices[0]]);
		VEC_COPY(v2,vertices[triangle_indices[1]]);
		VEC_COPY(v3,vertices[triangle_indices[2]]);
		return;
	}
    gim_trimesh_get_vertex(trimesh, triangle_indices[0], v1);
    gim_trimesh_get_vertex(trimesh, triangle_indices[1], v2);
    gim_trimesh_get_vertex(trimesh, triangle_indices[2], v3);
}

void gim_trimesh_initialize_cache(GIM_TRIMESH * trimesh)
{
    trimesh->m_cache = (GIM_TRIMESH_CACHE *)gim_alloc(sizeof(GIM_TRIMESH_CACHE));

    //Initializes the transformed vertices
    if(gim_trimesh_has_tranform(trimesh))
    {
        gim_trimesh_data_get(trimesh->m_trimesh_data_handle,&trimesh->m_unlocked_trimesh_data );
        //Create vertex buffer
        gim_create_common_vertex_buffer(&trimesh->m_cache->m_transformed_vertex_buffer,
        trimesh->m_unlocked_trimesh_data->m_source_vertex_buffer.m_element_count);
        //Copy the vertex buffer
        gim_copy_vertex_buffers(&trimesh->m_unlocked_trimesh_data->m_source_vertex_buffer,
        trimesh->m_unlocked_trimesh_data->m_vertex_scalar_type,
        &trimesh->m_cache->m_transformed_vertex_buffer,G_STYPE_REAL);

        trimesh->m_cache->m_tranformed_scalar_type = G_STYPE_REAL;

        trimesh->m_unlocked_trimesh_data = 0;

        //Create vertices bitset
        if(gim_trimesh_get_bound_method(trimesh)==G_TRIMESH_BOUND_AABB_TREE)
        {
            //Create the bitset
            GIM_BITSET_CREATE_SIZED(trimesh->m_cache->m_transformed_vertices_cache_bitset,
            trimesh->m_cache->m_transformed_vertex_buffer.m_element_count);
        }
        else
        {
            GIM_DYNARRAY_INVALIDATE(trimesh->m_cache->m_transformed_vertices_cache_bitset);
        }
    }
    else
    {
        GIM_BUFFER_ARRAY_INVALIDATE(trimesh->m_cache->m_transformed_vertex_buffer);
        GIM_DYNARRAY_INVALIDATE(trimesh->m_cache->m_transformed_vertices_cache_bitset);
    }

    GUINT facecount = gim_trimesh_get_triangle_count(trimesh);
    //create the planes cache
    GIM_DYNARRAY_CREATE_SIZED(GIM_TRIPLANES_CACHE,trimesh->m_cache->m_planes_cache_buffer,facecount);
    //Create the bitset
    GIM_BITSET_CREATE_SIZED(trimesh->m_cache->m_planes_cache_bitset,facecount);
}

void gim_trimesh_destroy_cache(GIM_TRIMESH * trimesh)
{
    if(trimesh->m_cache==0) return;
    if(trimesh->m_unlocked_trimesh_data != 0)
    {
        gim_trimesh_unlocks_work_data(trimesh);
    }
    GIM_DYNARRAY_DESTROY(trimesh->m_cache->m_planes_cache_buffer);
    GIM_DYNARRAY_DESTROY(trimesh->m_cache->m_planes_cache_bitset);

    if(GIM_BUFFER_ARRAY_IS_VALID(trimesh->m_cache->m_transformed_vertex_buffer))
    {
        GIM_BUFFER_ARRAY_DESTROY(trimesh->m_cache->m_transformed_vertex_buffer);
        if(GIM_DYNARRAY_IS_VALID(trimesh->m_cache->m_transformed_vertices_cache_bitset))
        {
            GIM_DYNARRAY_DESTROY(trimesh->m_cache->m_transformed_vertices_cache_bitset);
        }
    }

    gim_free(trimesh->m_cache,0);
    trimesh->m_cache = 0;
}

void gim_trimesh_initialize_bound(GIM_TRIMESH * trimesh)
{
    if(gim_trimesh_get_bound_method(trimesh)==G_TRIMESH_BOUND_NONE)
    {
        GUINT facecount = gim_trimesh_get_triangle_count(trimesh);
        trimesh->m_aabbset = (GIM_AABB_SET *)gim_alloc(sizeof(GIM_AABB_SET));
        gim_aabbset_alloc(trimesh->m_aabbset,facecount);
    }
}

void gim_trimesh_destroy_bound(GIM_TRIMESH * trimesh)
{
    if(trimesh->m_aabbset != 0)
    {
        gim_aabbset_destroy(trimesh->m_aabbset);
        gim_free(trimesh->m_aabbset,0);
    }
    trimesh->m_aabbset = 0;
}


void gim_trimesh_create(GIM_TRIMESH * trimesh, GUINT trimesh_data_handle,
char apply_transform,char create_cache)
{
    trimesh->m_mask = GIM_TRIMESH_NEED_UPDATE;//needs update

    trimesh->m_trimesh_data_handle = trimesh_data_handle;
    trimesh->m_unlocked_trimesh_data = 0;
    gim_trimesh_data_inc_ref(trimesh->m_trimesh_data_handle);

    trimesh->m_aabbset = 0;
    trimesh->m_cache = 0;

    if(apply_transform==1) trimesh->m_mask |= GIM_TRIMESH_APPLY_TRANSFORMATION;

    gim_trimesh_initialize_bound(trimesh);

    if(create_cache == 1 || gim_trimesh_get_bound_method(trimesh)==G_TRIMESH_BOUND_AABB_SET  )
    {
         gim_trimesh_initialize_cache(trimesh);
    }
    //Callback is 0
    trimesh->m_update_callback = 0;
    //set to identity
    IDENTIFY_MATRIX_4X4(trimesh->m_transform);
    IDENTIFY_MATRIX_4X4(trimesh->m_inv_transform);
}

void gim_trimesh_create_from_arrays(GIM_TRIMESH * trimesh, GBUFFER_ARRAY * vertex_array,GUINT vertex_scalar_type, GBUFFER_ARRAY * triindex_array,GUINT index_scalar_type,
char apply_transform,char create_cache)
{
    //Create mesh data
    gim_trimesh_data_create_from_arrays(
    &trimesh->m_trimesh_data_handle,vertex_array,vertex_scalar_type,triindex_array,index_scalar_type);
    //Create
    gim_trimesh_create(trimesh,trimesh->m_trimesh_data_handle,apply_transform,create_cache);
}



//! Create a trimesh from vertex array and an index array
/*!

\param trimesh An uninitialized GIM_TRIMESH  structure
\param vertex_array A buffer to a vec3f array
\param vertex_count
\param triindex_array
\param index_count
\param copy_vertices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
\param copy_indices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
\param transformed_reply If , then the m_transformed_vertices is a reply of the source vertices. Else it just be a reference to the original array.
*/
void gim_trimesh_create_from_data(GIM_TRIMESH * trimesh, vec3f * vertex_array, GUINT vertex_count,char copy_vertices, GUINT * triindex_array, GUINT triangle_count,char copy_indices,
char apply_transform,char create_cache)
{

    //Create mesh data
    gim_trimesh_data_create_from_array_data(
    &trimesh->m_trimesh_data_handle,vertex_array,vertex_count,copy_vertices,
     triindex_array, triangle_count,copy_indices);
    //Create
    gim_trimesh_create(trimesh,trimesh->m_trimesh_data_handle,
    apply_transform,create_cache);
}

//! Clears auxiliary data and releases buffer arrays
void gim_trimesh_destroy(GIM_TRIMESH * trimesh)
{
    gim_trimesh_destroy_cache(trimesh);
    gim_trimesh_destroy_bound(trimesh);
    //Release trimesh data
    gim_trimesh_data_dec_ref(trimesh->m_trimesh_data_handle);
    trimesh->m_trimesh_data_handle = 0;
}

//! Copies two meshes
/*!
\pre dest_trimesh shouldn't be created
\post dest_trimesh will be created
\param source_trimesh
\param dest_trimesh
\param copy_by_reference If 1, it attach a reference to the source vertices, else it copies the vertices
\param transformed_reply IF 1, then it forces the m_trasnformed_vertices to be  a reply of the source vertices
*/
void gim_trimesh_copy(GIM_TRIMESH * source_trimesh,GIM_TRIMESH * dest_trimesh, char copy_by_reference,
char apply_transform,char create_cache)
{
    if(copy_by_reference==1)
    {

        gim_trimesh_create(dest_trimesh,source_trimesh->m_trimesh_data_handle,
    apply_transform,create_cache);
    }
    else
    {
        gim_trimesh_data_create_copy(source_trimesh->m_trimesh_data_handle,&dest_trimesh->m_trimesh_data_handle,0);

        gim_trimesh_create(dest_trimesh,dest_trimesh->m_trimesh_data_handle,
    apply_transform,create_cache);
    }
}


void gim_trimesh_locks_work_data(GIM_TRIMESH * trimesh)
{
    gim_trimesh_data_lock(trimesh->m_trimesh_data_handle,G_MA_READ_ONLY,&trimesh->m_unlocked_trimesh_data);
    if(trimesh->m_cache)
    {
        if(GIM_BUFFER_ARRAY_IS_VALID(trimesh->m_cache->m_transformed_vertex_buffer))
        {
            gim_buffer_array_lock(&trimesh->m_cache->m_transformed_vertex_buffer,G_MA_READ_ONLY);
        }
    }
}

//! unlocks the trimesh
/*!
\post unlocks m_tri_index_buffer and m_transformed_vertex_buffer.
\param trimesh
*/
void gim_trimesh_unlocks_work_data(GIM_TRIMESH * trimesh)
{
    trimesh->m_unlocked_trimesh_data = 0;
    gim_trimesh_data_unlock(trimesh->m_trimesh_data_handle);

    if(trimesh->m_cache)
    {
        if(GIM_BUFFER_ARRAY_IS_VALID(trimesh->m_cache->m_transformed_vertex_buffer))
        {
            gim_buffer_array_unlock(&trimesh->m_cache->m_transformed_vertex_buffer);
        }
    }
}


//! Returns 1 if the m_transformed_vertex_buffer is a reply of m_source_vertex_buffer
char gim_trimesh_has_tranform(GIM_TRIMESH * trimesh)
{
    if(trimesh->m_mask&GIM_TRIMESH_APPLY_TRANSFORMATION) return 1;
    return 0;
}

//! Returns 1 if the trimesh needs to update their aabbset and the planes cache.
char gim_trimesh_needs_update(GIM_TRIMESH * trimesh)
{
    if(trimesh->m_mask&GIM_TRIMESH_NEED_UPDATE) return 1;
    return 0;
}

//! Change the state of the trimesh for force it to update
/*!
Call it after made changes to the trimesh.
\post gim_trimesh_need_update(trimesh) will return 1
*/
void gim_trimesh_post_update(GIM_TRIMESH * trimesh)
{
    trimesh->m_mask |= GIM_TRIMESH_NEED_UPDATE;
}



//! Updates m_transformed_vertex_buffer
/*!
\pre m_transformed_vertex_buffer must be unlocked
*/
void gim_trimesh_update_vertices(GIM_TRIMESH * trimesh)
{
    if(trimesh->m_cache == 0) return;
    if(gim_trimesh_has_tranform(trimesh) == 0) return; //Don't perform transformation

    if(GIM_DYNARRAY_IS_VALID( trimesh->m_cache->m_transformed_vertices_cache_bitset))
    {
        GIM_BITSET_CLEAR_ALL(trimesh->m_cache->m_transformed_vertices_cache_bitset);
        return;
    }


//transform vertices
    mat4f transform;
    COPY_MATRIX_4X4(transform,trimesh->m_transform);

    gim_trimesh_data_get(trimesh->m_trimesh_data_handle,&trimesh->m_unlocked_trimesh_data );

    gim_transform_vertex_buffers(
    &trimesh->m_unlocked_trimesh_data->m_source_vertex_buffer,
    trimesh->m_unlocked_trimesh_data->m_vertex_scalar_type,
    &trimesh->m_cache->m_transformed_vertex_buffer,trimesh->m_cache->m_tranformed_scalar_type,transform);

    trimesh->m_unlocked_trimesh_data = 0;
}

void gim_trimesh_update_cache(GIM_TRIMESH * trimesh)
{
    if(trimesh->m_cache == 0) return;
    //Clear planes cache
    GIM_BITSET_CLEAR_ALL(trimesh->m_cache->m_planes_cache_bitset);
    gim_trimesh_update_vertices(trimesh);
}

//! Updates m_aabbset and m_planes_cache_bitset
/*!
\pre gim_trimesh_locks_work_data must be called before
*/
void gim_trimesh_update_aabbset(GIM_TRIMESH * trimesh)
{
    if(trimesh->m_aabbset == 0) return;
    assert(trimesh->m_cache);
    assert(GIM_BUFFER_ARRAY_IS_VALID(trimesh->m_cache->m_transformed_vertex_buffer));

    vec3f * vertices = GIM_BUFFER_ARRAY_POINTER(vec3f,trimesh->m_cache->m_transformed_vertex_buffer,0);

    // box set
    aabb3f * paabb = trimesh->m_aabbset->m_boxes;
    GUINT triangle_count = gim_trimesh_get_triangle_count(trimesh);
    vec3ui triangle_indices;
    GUINT i;
    for (i=0; i<triangle_count;i++)
    {
        gim_trimesh_data_get_triangle_indices(
        trimesh->m_unlocked_trimesh_data,i,triangle_indices);
        COMPUTEAABB_FOR_TRIANGLE((*paabb),
        vertices[triangle_indices[0]],
        vertices[triangle_indices[1]],vertices[triangle_indices[2]]);
        paabb++;
    }

    //Sorts set
    gim_aabbset_update(trimesh->m_aabbset);
}

//! Updates the trimesh if needed
/*!
\post If gim_trimesh_needs_update returns 1, then it calls  gim_trimesh_update_vertices and gim_trimesh_update_aabbset
*/
void gim_trimesh_update(GIM_TRIMESH * trimesh)
{
    if(gim_trimesh_needs_update(trimesh)==0) return;

    gim_trimesh_update_cache(trimesh);

    if(trimesh->m_aabbset!=0)
    {
        gim_trimesh_locks_work_data(trimesh);
        gim_trimesh_update_aabbset(trimesh);
        gim_trimesh_unlocks_work_data(trimesh);
    }

    //Clear update flag
     trimesh->m_mask &= ~GIM_TRIMESH_NEED_UPDATE;
}

void gim_trimesh_set_tranform(GIM_TRIMESH * trimesh, mat4f transform)
{
    GREAL diff = 0.0f;
    float * originaltrans = &trimesh->m_transform[0][0];
    float * newtrans = &transform[0][0];
    GUINT i;
    for (i=0;i<16;i++)
    {
    	diff += fabs(originaltrans[i]-newtrans[i]);
    }

    if(GIM_IS_ZERO(diff)) return ;///don't need to update
    //if(diff< 0.0006f) return ;///don't need to update

    COPY_MATRIX_4X4(trimesh->m_transform,transform);

    GREAL det;
    INVERT_4X4(trimesh->m_inv_transform,det,transform);

    gim_trimesh_post_update(trimesh);
}

void gim_trimesh_get_triangle_data(GIM_TRIMESH * trimesh, GUINT triangle_index, GIM_TRIANGLE_DATA * tri_data)
{
    gim_trimesh_get_triangle_vertices(trimesh,triangle_index,
    tri_data->m_vertices[0],
    tri_data->m_vertices[1],
    tri_data->m_vertices[2]);

    if(trimesh->m_cache)
    {
        //Get the planes
        GIM_TRIPLANES_CACHE * planes = GIM_DYNARRAY_POINTER(GIM_TRIPLANES_CACHE,trimesh->m_cache->m_planes_cache_buffer);
        planes += triangle_index;

        //verify planes cache
        char bit_eval;
        GIM_BITSET_GET(trimesh->m_cache->m_planes_cache_bitset,triangle_index,bit_eval);
        if(bit_eval == 0)// Needs to calc the planes
        {
            //Calc the face plane
            TRIANGLE_PLANE(tri_data->m_vertices[0],tri_data->m_vertices[1],tri_data->m_vertices[2],planes->m_planes[0]);
            //Calc the edge 1
            EDGE_PLANE(tri_data->m_vertices[0],tri_data->m_vertices[1],(planes->m_planes[0]),(planes->m_planes[1]));

            //Calc the edge 2
            EDGE_PLANE(tri_data->m_vertices[1],tri_data->m_vertices[2],(planes->m_planes[0]),(planes->m_planes[2]));

            //Calc the edge 3
            EDGE_PLANE(tri_data->m_vertices[2],tri_data->m_vertices[0],(planes->m_planes[0]), (planes->m_planes[3]));

            //mark
            GIM_BITSET_SET(trimesh->m_cache->m_planes_cache_bitset,triangle_index);
        }


        VEC_COPY_4((tri_data->m_planes.m_planes[0]),(planes->m_planes[0]));//face plane
        VEC_COPY_4((tri_data->m_planes.m_planes[1]),(planes->m_planes[1]));//edge1
        VEC_COPY_4((tri_data->m_planes.m_planes[2]),(planes->m_planes[2]));//edge2
        VEC_COPY_4((tri_data->m_planes.m_planes[3]),(planes->m_planes[3]));//edge3
		tri_data->m_has_planes = 1;
        return;
    }
    GIM_CALC_TRIANGLE_DATA_PLANES((*tri_data));
	tri_data->m_has_planes = 1;
}

void gim_trimesh_get_triangle_data_lazy(GIM_TRIMESH * trimesh, GUINT triangle_index, GIM_TRIANGLE_DATA * tri_data)
{
	   gim_trimesh_get_triangle_vertices(trimesh,triangle_index,
    tri_data->m_vertices[0],
    tri_data->m_vertices[1],
    tri_data->m_vertices[2]);

    if(trimesh->m_cache)
    {
        //Get the planes
        GIM_TRIPLANES_CACHE * planes = GIM_DYNARRAY_POINTER(GIM_TRIPLANES_CACHE,trimesh->m_cache->m_planes_cache_buffer);
        planes += triangle_index;

        //verify planes cache
        char bit_eval;
        GIM_BITSET_GET(trimesh->m_cache->m_planes_cache_bitset,triangle_index,bit_eval);
        if(bit_eval == 0)// Needs to calc the planes
        {
            //Calc the face plane
            TRIANGLE_PLANE(tri_data->m_vertices[0],tri_data->m_vertices[1],tri_data->m_vertices[2],planes->m_planes[0]);
            //Calc the edge 1
            EDGE_PLANE(tri_data->m_vertices[0],tri_data->m_vertices[1],(planes->m_planes[0]),(planes->m_planes[1]));

            //Calc the edge 2
            EDGE_PLANE(tri_data->m_vertices[1],tri_data->m_vertices[2],(planes->m_planes[0]),(planes->m_planes[2]));

            //Calc the edge 3
            EDGE_PLANE(tri_data->m_vertices[2],tri_data->m_vertices[0],(planes->m_planes[0]), (planes->m_planes[3]));

            //mark
            GIM_BITSET_SET(trimesh->m_cache->m_planes_cache_bitset,triangle_index);
        }


        VEC_COPY_4((tri_data->m_planes.m_planes[0]),(planes->m_planes[0]));//face plane
        VEC_COPY_4((tri_data->m_planes.m_planes[1]),(planes->m_planes[1]));//edge1
        VEC_COPY_4((tri_data->m_planes.m_planes[2]),(planes->m_planes[2]));//edge2
        VEC_COPY_4((tri_data->m_planes.m_planes[3]),(planes->m_planes[3]));//edge3
		tri_data->m_has_planes = 1;
        return;
    }
    tri_data->m_has_planes = 0;

}

void gim_trimesh_get_aabb(GIM_TRIMESH * trimesh,aabb3f * bound)
{
    if(trimesh->m_aabbset)
    {
        AABB_COPY((*bound),trimesh->m_aabbset->m_global_bound);
        return;
    }

    if(gim_trimesh_data_has_bv_tree(trimesh->m_trimesh_data_handle))
    {
        gim_trimesh_data_get(trimesh->m_trimesh_data_handle,&trimesh->m_unlocked_trimesh_data);
        if(gim_trimesh_has_tranform(trimesh))
        {
            AABB_TRANSFORM((*bound),
            trimesh->m_unlocked_trimesh_data->m_bv_tree.m_root_node->m_data.m_aabb,
            trimesh->m_transform);
        }
        else
        {
            AABB_COPY((*bound),trimesh->m_unlocked_trimesh_data->m_bv_tree.m_root_node->m_data.m_aabb);
        }
		trimesh->m_unlocked_trimesh_data  = 0;
        return;
    }
}


void gim_trimesh_midphase_box_collision(GIM_TRIMESH * trimesh,aabb3f *test_aabb, GDYNAMIC_ARRAY * collided)
{
    collided->m_size = 0;
    if(gim_trimesh_get_bound_method(trimesh)==G_TRIMESH_BOUND_AABB_TREE)
    {
        GIM_TRIMESH_DATA  * tridata;
        gim_trimesh_data_get(trimesh->m_trimesh_data_handle,&tridata);
        aabb3f transbound;
        AABB_TRANSFORM(transbound,(*test_aabb),trimesh->m_inv_transform);
        gim_aabbtree_box_collision(&transbound,&tridata->m_bv_tree,collided);
    }
    else if(gim_trimesh_get_bound_method(trimesh)==G_TRIMESH_BOUND_AABB_SET)
    {
        gim_aabbset_box_collision(test_aabb,trimesh->m_aabbset,collided);
    }
}

void gim_trimesh_midphase_box_collision_local(GIM_TRIMESH * trimesh,aabb3f *test_aabb, GDYNAMIC_ARRAY * collided)
{
	collided->m_size = 0;
    if(gim_trimesh_get_bound_method(trimesh)==G_TRIMESH_BOUND_AABB_TREE)
    {
        GIM_TRIMESH_DATA  * tridata;
        gim_trimesh_data_get(trimesh->m_trimesh_data_handle,&tridata);        
        gim_aabbtree_box_collision(test_aabb,&tridata->m_bv_tree,collided);
    }
    else if(gim_trimesh_get_bound_method(trimesh)==G_TRIMESH_BOUND_AABB_SET)
    {
        gim_aabbset_box_collision(test_aabb,trimesh->m_aabbset,collided);
    }
}

void gim_trimesh_midphase_trimesh_collision(GIM_TRIMESH * trimesh1,
GIM_TRIMESH * trimesh2, GDYNAMIC_ARRAY * collision_pairs, char * swaped)
{
    collision_pairs->m_size = 0;
    GIM_TRIMESH_DATA  * tridata1;
    GIM_TRIMESH_DATA  * tridata2;
    if(gim_trimesh_get_bound_method(trimesh1)==G_TRIMESH_BOUND_AABB_TREE)
    {
        gim_trimesh_data_get(trimesh1->m_trimesh_data_handle,&tridata1);
        if(gim_trimesh_get_bound_method(trimesh2)==G_TRIMESH_BOUND_AABB_TREE)
        {
            *swaped = 0;
            gim_trimesh_data_get(trimesh2->m_trimesh_data_handle,&tridata2);

            gim_aabbtree_bipartite_intersections_trans(
            &tridata1->m_bv_tree,
            &tridata2->m_bv_tree,
            trimesh1->m_transform,
            trimesh1->m_inv_transform,
            trimesh2->m_transform,
            trimesh2->m_inv_transform,
            collision_pairs);
        }
        else if(gim_trimesh_get_bound_method(trimesh2)==G_TRIMESH_BOUND_AABB_SET)
        {
            *swaped = 1;

            gim_aabbset_aabbtree_intersections_trans(
            trimesh2->m_aabbset,
            &tridata1->m_bv_tree,
            trimesh1->m_transform,
            trimesh1->m_inv_transform,
            collision_pairs);
        }
    }
    else if(gim_trimesh_get_bound_method(trimesh1)==G_TRIMESH_BOUND_AABB_SET)
    {
        *swaped = 0;
        if(gim_trimesh_get_bound_method(trimesh2)==G_TRIMESH_BOUND_AABB_TREE)
        {
            gim_trimesh_data_get(trimesh2->m_trimesh_data_handle,&tridata2);

            gim_aabbset_aabbtree_intersections_trans(
            trimesh1->m_aabbset,
            &tridata2->m_bv_tree,
            trimesh2->m_transform,
            trimesh2->m_inv_transform,
            collision_pairs);
        }
        else if(gim_trimesh_get_bound_method(trimesh2)==G_TRIMESH_BOUND_AABB_SET)
        {
            gim_aabbset_bipartite_intersections(
            trimesh1->m_aabbset,
            trimesh2->m_aabbset,
            collision_pairs);
        }
    }
}

