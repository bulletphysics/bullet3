#ifndef GIM_TRIMESH_H_INCLUDED
#define GIM_TRIMESH_H_INCLUDED
/*! \file gim_trimesh.h
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

#include "GIMPACT/gim_trimesh_data.h"
#include "GIMPACT/gim_vertex_buffer_util.h"
#include "GIMPACT/gim_contact.h"


/*! \addtogroup TRIMESH
\brief
A Trimesh is the basic geometric structure for representing solid objects.
<p><strong>CREATING TRIMESHES</strong></p>
<ul>
<li> For creating trimeshes, you must initialize Buffer managers by calling \ref gimpact_init
<li> Then you must define the vertex and index sources by creating them with \ref BUFFER_ARRAYS routines, and then call  \ref gim_trimesh_create_from_arrays.
<li> An alternative way for creaing trimesh objects is calling  \ref gim_trimesh_create_from_data.
<li> For access to the trimesh data (vertices, triangle indices), you must call  \ref gim_trimesh_locks_work_data , and  \ref gim_trimesh_unlocks_work_data for finish the access.
<li> Each time when the trimesh data is modified, you must call  \ref gim_trimesh_update after.
<li> When a trimesh is no longer needed, you must call \ref gim_trimesh_destroy.
</ul>

<p>This is an example of how to create a deformable trimesh that shares vertices with the user application:</p>
\code
//Declaration of vertices
vec3f trimeshvertices[200];
//Declaration of indices
GUINT trimeshindices[100];

... Initializing vertices and triangle indices at beginning

//Then create trimesh
GIM_TRIMESH mytrimesh;

//Calling trimesh create function

gim_trimesh_create_from_data(
&mytrimesh,
trimeshvertices,200,
0 ,//copy_vertices is 0
trimeshindices,
100,
0, //copy_indices is 0
0 //transformed_reply is 0
);
\endcode
<p>Note that parameter transformed_reply is 0, that means that m_transformed_vertex_buffer is a reference to m_source_vertex on the trimesh, and transformations are not avaliable. Use that configuration if you have to simulate a deformable trimesh like cloth or elastic bodies.</p>
<p>When the trimesh is no longer needed, destroy it safely with gim_trimesh_destroy()</p>
<p><strong>UPDATING TRIMESHES</strong></p>
<p>On simulation loops, is needed to update trimeshes every time for update vertices althought updating triangle boxes and planes cache. There is two ways for update trimeshes: </p>
<ul>
<li> Updating vertices directly. You need to access to the \ref GIM_TRIMESH.m_source_vertex_buffer member; a vertex buffer which has access to the source vertices.
\code
// Access to the source vertices
gim_buffer_array_lock(&mytrimesh.m_source_vertex_buffer, G_MA_READ_WRITE);

//Get a pointer to the vertex buffer
vec3f * vertexpointer = GIM_BUFFER_ARRAY_POINTER(vec3f,mytrimesh.m_source_vertex_buffer,0);

//Get the amount of vertices
int veccount = mytrimesh.m_source_vertex_buffer.m_element_count;

//Modify vertices
for (int i=0;i<veccount ;i++ )
{
    .....
    .....
    processing vertices
    .....
	.....
}

// Don't forget to unlock the source vertex array
gim_buffer_array_unlock(&mytrimesh.m_source_vertex_buffer);

// Notify that the state of the trimesh is changed
gim_trimesh_post_update(&mytrimesh.m_source_vertex_buffer);

\endcode
For making trimeshes that allow to update their vertices, use \ref gim_trimesh_create_from_data with parameter <strong>transformed_reply</strong> = 0.
</ul>
<ul>
<li> Aplying a transformation. Simply use \ref gim_trimesh_set_tranform . Remember that with this method trimeshes must be created with \ref gim_trimesh_create_from_data with parameter <strong>transformed_reply</strong> = 1.
</ul>
<p> After updating vertices, you must call \ref gim_trimesh_update()</p>
<p><strong>TRIMESHES COLLISION</strong></p>
<p>Before collide trimeshes, you need to update them first.</p>
<p>Then you must use \ref gim_trimesh_trimesh_collision().</p>

*/
//! @{

//! Prototype for updating vertices
typedef void * gim_update_trimesh_function(struct GIM_TRIMESH *);

struct GIM_TRIMESH_CACHE
{
    GDYNAMIC_ARRAY m_planes_cache_buffer;//! Allocated GIM_TRIPLANES_CACHE
    GDYNAMIC_ARRAY m_planes_cache_bitset;

    //! Allocated transformed vertices vec3f
    /*!
    \invariant must be aligned
    */
    GBUFFER_ARRAY m_transformed_vertex_buffer;
    GDYNAMIC_ARRAY m_transformed_vertices_cache_bitset;
    GUINT m_tranformed_scalar_type;

};

//! Constant which defines the Bounding volume method
#define G_TRIMESH_BOUND_NONE 0
//! Constant which defines the Bounding volume method
#define G_TRIMESH_BOUND_AABB_TREE 1
//! Constant which defines the Bounding volume method
#define G_TRIMESH_BOUND_AABB_SET 2

///MAsk defines
#define GIM_TRIMESH_APPLY_TRANSFORMATION 1
#define GIM_TRIMESH_NEED_UPDATE 2

//! Trimesh
struct GIM_TRIMESH
{
    GUINT m_trimesh_data_handle;//!< Handle to a trimesh data object
    GIM_TRIMESH_DATA * m_unlocked_trimesh_data; //!< temporal pointer
    char m_mask;//!< Don't use directly
    GIM_AABB_SET * m_aabbset;
    GIM_TRIMESH_CACHE * m_cache;
    gim_update_trimesh_function * m_update_callback;//! If null, then m_transform is applied.
    mat4f m_transform;
    mat4f m_inv_transform;
};
//typedef struct _GIM_TRIMESH GIM_TRIMESH;

//! Return the bound method
/*!
\ return G_TRIMESH_BOUND_AABB_TREE or G_TRIMESH_BOUND_AABB_SET
*/
GUINT gim_trimesh_get_bound_method(GIM_TRIMESH * trimesh);

/// Info about mesh
//! Return the trimesh triangle count
GUINT gim_trimesh_get_triangle_count(GIM_TRIMESH * trimesh);

GUINT gim_trimesh_get_vertex_count(GIM_TRIMESH * trimesh);

//! Returns 1 if the m_transformed_vertex_buffer is a reply of m_source_vertex_buffer
char gim_trimesh_has_tranform(GIM_TRIMESH * trimesh);

//! Get the global bounding box
void gim_trimesh_get_aabb(GIM_TRIMESH * trimesh,aabb3f * bound);

//! Returns 1 if the trimesh needs to update their aabbset and the planes cache.
char gim_trimesh_needs_update(GIM_TRIMESH * trimesh);

//! Change the state of the trimesh for force it to update
/*!
Call it after made changes to the trimesh.
\post gim_trimesh_need_update(trimesh) will return 1
\sa gim_trimesh_needs_update
*/
void gim_trimesh_post_update(GIM_TRIMESH * trimesh);



//! Creates cache information for this mesh
/*!
*/
void gim_trimesh_initialize_cache(GIM_TRIMESH * trimesh);

//! Destroys the cache
/*!
*/
void gim_trimesh_destroy_cache(GIM_TRIMESH * trimesh);

//! Creates an AABB set for triangles.
/*!
*/
void gim_trimesh_initialize_bound(GIM_TRIMESH * trimesh);

//! Destroys the AABB set
/*!
*/
void gim_trimesh_destroy_bound(GIM_TRIMESH * trimesh);

//! Creates a trimesh
/*!

\param trimesh for be created
\param trimesh_data_handle Handle to a trimesh data
\param apply_transform If 1, then it transforms the source vertices.
\param create_Cache If 1, then it allocates a cache for transformed vertices and planes.
\post Adds a reference to the trimesh_data_handle, and creates the auxiliary data (m_aabbset,m_planes_cache_buffer)
*/
void gim_trimesh_create(GIM_TRIMESH * trimesh, GUINT trimesh_data_handle,char apply_transform,char create_cache);

//! Creates the aabb set and the triangles cache
/*!

\param trimesh
\param vertex_array
\param triindex_array
\param apply_transform If 1, then it transforms the source vertices.
\param create_Cache If 1, then it allocates a cache for transformed vertices and planes.
\param vertex_scalar_type G_STYPE_REAL or G_STYPE_REAL2
\param index_scalar_type G_STYPE_SHORT,G_STYPE_USHORT,G_STYPE_INT or G_STYPE_UINT
\post it copies the arrays by reference, and creates the auxiliary data (m_aabbset,m_planes_cache_buffer)
*/
void gim_trimesh_create_from_arrays(GIM_TRIMESH * trimesh, GBUFFER_ARRAY * vertex_array,GUINT vertex_scalar_type, GBUFFER_ARRAY * triindex_array,GUINT index_scalar_type,
char apply_transform,char create_cache);



//! Create a trimesh from vertex array and an index array
/*!
\param trimesh An uninitialized GIM_TRIMESH  structure
\param vertex_array A buffer to a vec3f array
\param vertex_count
\param triindex_array
\param triangle_count
\param copy_vertices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
\param copy_indices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
\param apply_transform If 1, then it transforms the source vertices.
\param create_Cache If 1, then it allocates a cache for transformed vertices and planes.
*/
void gim_trimesh_create_from_data(GIM_TRIMESH * trimesh, vec3f * vertex_array, GUINT vertex_count,char copy_vertices, GUINT * triindex_array, GUINT triangle_count,char copy_indices,
char apply_transform,char create_cache);

//! Clears auxiliary data and releases buffer arrays
void gim_trimesh_destroy(GIM_TRIMESH * trimesh);

//! Copies two meshes
/*!
\param source_trimesh
\param dest_trimesh
\param copy_by_reference If 1, it attach a reference to the source vertices, else it copies the vertices
\param apply_transform If 1, then it transforms the source vertices.
\param create_Cache If 1, then it allocates a cache for transformed vertices and planes.
*/
void gim_trimesh_copy(GIM_TRIMESH * source_trimesh,GIM_TRIMESH * dest_trimesh, char copy_by_reference,
char apply_transform,char create_cache);


//! Locks the trimesh for working with it
/*!
\post locks m_trimesh_data_handle in G_MA_READ_ONLY and m_transformed_vertex_buffer in G_MA_READ_WRITE.
\param trimesh
*/
void gim_trimesh_locks_work_data(GIM_TRIMESH * trimesh);


//! unlocks the trimesh
/*!
\post unlocks m_tri_index_buffer and m_transformed_vertex_buffer.
\param trimesh
*/
void gim_trimesh_unlocks_work_data(GIM_TRIMESH * trimesh);

//! Updates m_transformed_vertex_buffer
/*!
\pre m_transformed_vertex_buffer must be unlocked
*/
void gim_trimesh_update_vertices(GIM_TRIMESH * trimesh);


//! Updates the cache variables
void gim_trimesh_update_cache(GIM_TRIMESH * trimesh);

//! Updates m_aabbset and m_planes_cache_bitset
/*!
\pre gim_trimesh_locks_work_data must be called before
*/
void gim_trimesh_update_aabbset(GIM_TRIMESH * trimesh);

//! Calls before perfom collisions. Updates the trimesh if needed
/*!
\post If gim_trimesh_needs_update returns 1, then it calls  gim_trimesh_update_vertices and gim_trimesh_update_aabbset
*/
void gim_trimesh_update(GIM_TRIMESH * trimesh);

//! Set the transform of a trimesh
/*!
\post This function calls to gim_trimesh_post_update
*/
void gim_trimesh_set_tranform(GIM_TRIMESH * trimesh, mat4f transform);

//! Fetch triangle data
/*!
\pre gim_trimesh_locks_work_data must be called before
*/
void gim_trimesh_get_triangle_data(GIM_TRIMESH * trimesh, GUINT triangle_index, GIM_TRIANGLE_DATA * tri_data);

//! Fetch triangle data
/*!
\pre gim_trimesh_locks_work_data must be called before
If Planes cache is not avaliable, it doesn't calc the planes
*/
void gim_trimesh_get_triangle_data_lazy(GIM_TRIMESH * trimesh, GUINT triangle_index, GIM_TRIANGLE_DATA * tri_data);


//! Fetch triangle vertices, transformed to global space
/*!
\pre gim_trimesh_locks_work_data must be called before
*/
void gim_trimesh_get_triangle_vertices(GIM_TRIMESH * trimesh, GUINT triangle_index, vec3f v1,vec3f v2,vec3f v3);

//! Fetch a single transformed vertex
/*!
\pre gim_trimesh_locks_work_data must be called before
*/
void gim_trimesh_get_vertex(GIM_TRIMESH * trimesh, GUINT vertex_index, vec3f vec);

//! Fetch triangle vertices, transformed to global space
/*!
\pre gim_trimesh_locks_work_data must be called before
*/
void gim_trimesh_get_triangle_vertices_local(GIM_TRIMESH * trimesh, GUINT triangle_index, vec3f v1,vec3f v2,vec3f v3);

//! Fetch a single transformed vertex
/*!
\pre gim_trimesh_locks_work_data must be called before
*/
void gim_trimesh_get_vertex_local(GIM_TRIMESH * trimesh, GUINT vertex_index, vec3f vec);


//! Finds Intersected boxes
/*!
\param trimesh
\param test_aabb Box for collision query, in global space
\param collided Array of GUINT elements, indices of trinagles. Must be initialized before (Reserve size ~ 100)
*/
void gim_trimesh_midphase_box_collision(GIM_TRIMESH * trimesh,aabb3f *test_aabb, GDYNAMIC_ARRAY * collided);



//! Finds Intersected boxes in local space
/*!
\param trimesh
\param test_aabb Box for collision query, in global space
\param collided Array of GUINT elements, indices of trinagles. Must be initialized before (Reserve size ~ 100)
*/
void gim_trimesh_midphase_box_collision_local(GIM_TRIMESH * trimesh,aabb3f *test_aabb, GDYNAMIC_ARRAY * collided);


//! Finds overlapping boxes between sets
/*!
\param trimesh1
\param trimesh2
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
\param swaped If 1, result pairs are swaped and that would means that element 1 belongs to trimesh2 and element 2 belongs to trimesh1. 0 is normal.
*/
void gim_trimesh_midphase_trimesh_collision(GIM_TRIMESH * trimesh1,
GIM_TRIMESH * trimesh2, GDYNAMIC_ARRAY * collision_pairs, char * swaped);




//! Trimesh Trimesh Collisions
/*!
Before use this function you must update each trimesh:
\code
gim_trimesh_update(TriMesh1);
gim_trimesh_update(TriMesh2);
\endcode
Then you must use the trimesh collision in this way:
\code
int collide_trimeshes(GIM_TRIMESH * TriMesh1, GIM_TRIMESH * TriMesh2)
{
    //Create contact list
    GDYNAMIC_ARRAY trimeshcontacts;
    GIM_CREATE_CONTACT_LIST(trimeshcontacts);

    //Collide trimeshes
    gim_trimesh_trimesh_collision(TriMesh1,TriMesh2,&trimeshcontacts);

    if(trimeshcontacts.m_size == 0) //do  nothing
    {
        GIM_DYNARRAY_DESTROY(trimeshcontacts);//clean contact array
        return 0;
    }

    //Getting a pointer to the contact array
    GIM_CONTACT * ptrimeshcontacts = GIM_DYNARRAY_POINTER(GIM_CONTACT,trimeshcontacts);

    int contactcount = trimeshcontacts.m_size;
    int i;
    //Process contacts
    for (i=0;i<contactcount ;i++)
    {
        //Do something with the contact (ptrimeshcontacts)
        ......
        ......
        // Like creating joints or anything else
        ......
        ......
        ptrimeshcontacts++;
    }
    GIM_DYNARRAY_DESTROY(trimeshcontacts);
    return contactcount;
}
\endcode
In each contact
<ul>
<li> m_handle1 points to trimesh1.
<li> m_handle2 points to trimesh2.
<li> m_feature1 Is a triangle index of trimesh1.
<li> m_feature2 Is a triangle index of trimesh2.
</ul>

\param trimesh1 Collider
\param trimesh2 Collidee
\param contacts A GIM_CONTACT array. Must be initialized
*/
void gim_trimesh_trimesh_collision(GIM_TRIMESH * trimesh1, GIM_TRIMESH * trimesh2, GDYNAMIC_ARRAY * contacts);


//! Trimesh Sphere Collisions
/*!
Before use this function you must update the trimesh:
\code
gim_trimesh_update(trimesh);
\endcode
Then you must use this function in this way:
\code
int collide_trimesh_sphere(GIM_TRIMESH * trimesh, vec3f center,GREAL radius)
{
    //Create contact list
    GDYNAMIC_ARRAY trimeshcontacts;
    GIM_CREATE_CONTACT_LIST(trimeshcontacts);

    //Collide trimeshes
    gim_trimesh_sphere_collision(trimesh,center,radius,&trimeshcontacts);

    if(trimeshcontacts.m_size == 0) //do  nothing
    {
        GIM_DYNARRAY_DESTROY(trimeshcontacts);//clean contact array
        return 0;
    }

    //Getting a pointer to the contact array
    GIM_CONTACT * ptrimeshcontacts = GIM_DYNARRAY_POINTER(GIM_CONTACT,trimeshcontacts);

    int contactcount = trimeshcontacts.m_size;
    int i;
    //Process contacts
    for (i=0;i<contactcount ;i++)
    {
        //Do something with the contact (ptrimeshcontacts)
        ......
        ......
        // Like creating joints or anything else
        ......
        ......
        ptrimeshcontacts++;
    }
    GIM_DYNARRAY_DESTROY(trimeshcontacts);
    return contactcount;
}
\endcode

In each contact
<ul>
<li> m_handle1 points to trimesh.
<li> m_handle2 points to NULL.
<li> m_feature1 Is a triangle index of trimesh.
</ul>

\param trimesh
\param center
\param radius
\param contacts A GIM_CONTACT array. Must be initialized
*/
void gim_trimesh_sphere_collision(GIM_TRIMESH * trimesh,vec3f center,GREAL radius, GDYNAMIC_ARRAY * contacts);


//! Trimesh Capsule collision
/*!
Find the closest primitive collided by the ray.

Before use this function you must update the trimesh:
\code
gim_trimesh_update(trimesh);
\endcode
Then you must use this function in this way:
\code
int collide_trimesh_capsule(GIM_TRIMESH * trimesh, GIM_CAPSULE_DATA * capsule)
{
    //Create contact list
    GDYNAMIC_ARRAY trimeshcontacts;
    GIM_CREATE_CONTACT_LIST(trimeshcontacts);

    //Collide trimeshes
    gim_trimesh_capsule_collision(trimesh,capsule,&trimeshcontacts);

    if(trimeshcontacts.m_size == 0) //do  nothing
    {
        GIM_DYNARRAY_DESTROY(trimeshcontacts);//clean contact array
        return 0;
    }

    //Getting a pointer to the contact array
    GIM_CONTACT * ptrimeshcontacts = GIM_DYNARRAY_POINTER(GIM_CONTACT,trimeshcontacts);

    int contactcount = trimeshcontacts.m_size;
    int i;
    //Process contacts
    for (i=0;i<contactcount ;i++)
    {
        //Do something with the contact (ptrimeshcontacts)
        ......
        ......
        // Like creating joints or anything else
        ......
        ......
        ptrimeshcontacts++;
    }
    GIM_DYNARRAY_DESTROY(trimeshcontacts);
    return contactcount;
}
\endcode

In each contact
<ul>
<li> m_handle1 points to trimesh.
<li> m_handle2 points to NULL.
<li> m_feature1 Is a triangle index of trimesh.
</ul>

\param trimesh
\param capsule
\param contacts A GIM_CONTACT array. Must be initialized
*/
void gim_trimesh_capsule_collision(GIM_TRIMESH * trimesh, GIM_CAPSULE_DATA * capsule, GDYNAMIC_ARRAY * contacts);


///Function for create Trimesh Plane  collision result
#define GIM_CREATE_TRIMESHPLANE_CONTACTS(dynarray) GIM_DYNARRAY_CREATE(vec4f,dynarray,G_ARRAY_GROW_SIZE)

//! Trimesh Plane Collisions
/*!

Before use this function you must update the trimesh:
\code
gim_trimesh_update(trimesh);
\endcode
Then you must use this function in this way:
\code
int collide_trimesh_plane(GIM_TRIMESH * trimesh, vec4f plane)
{
    //Create contact list
    GDYNAMIC_ARRAY tri_plane_contacts;
    GIM_CREATE_TRIMESHPLANE_CONTACTS(tri_plane_contacts);

    //Collide trimeshes
    gim_trimesh_plane_collision(trimesh,plane,&tri_plane_contacts);

    if(tri_plane_contacts.m_size == 0) //do  nothing
    {
        GIM_DYNARRAY_DESTROY(tri_plane_contacts);//clean contact array
        return 0;
    }

    //Getting a pointer to the contact array
    vec4f * planecontacts = GIM_DYNARRAY_POINTER(vec4f,tri_plane_contacts);

    int contactcount = tri_plane_contacts.m_size;
    int i;
    //Process contacts
    for (i=0;i<contactcount ;i++)
    {
        vec3f contactpoint;
        GREAL contactdis;

        VEC_COPY(contactpoint,planecontacts[i]); //Get contact point
        contactdis = planecontacts[i][3]; // Get distance depth

        //Do something with the contact
        ......
        ......
        // Like creating joints or anything else
        ......
        ......
    }
    GIM_DYNARRAY_DESTROY(tri_plane_contacts);
    return contactcount;
}
\endcode

In each contact the 3 first coordinates refers to the contact point, the fourth refers to the distance depth and the normal is the normal of the plane.

\param trimesh
\param plane vec4f plane
\param contacts A vec4f array. Must be initialized (~100). Each element have the coordinate point in the first 3 elements, and vec4f[3] has the penetration depth.
*/
void gim_trimesh_plane_collision(GIM_TRIMESH * trimesh,vec4f plane, GDYNAMIC_ARRAY * contacts);


//! Trimesh Ray Collisions
/*!
\param trimesh
\param origin
\param dir
\param tmax
\param contact
\return 1 if the ray collides, else 0
*/
int gim_trimesh_ray_collision(GIM_TRIMESH * trimesh,vec3f origin,vec3f dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA * contact);


//! Trimesh Ray Collisions closest
/*!
Find the closest primitive collided by the ray
\param trimesh
\param origin
\param dir
\param tmax
\param contact
\return 1 if the ray collides, else 0
*/
int gim_trimesh_ray_closest_collision(GIM_TRIMESH * trimesh,vec3f origin,vec3f dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA * contact);

//! @}



#endif // GIM_TRIMESH_H_INCLUDED
