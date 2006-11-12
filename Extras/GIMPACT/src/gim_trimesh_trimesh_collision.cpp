
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

#define CLASSIFY_TRI_BY_FACE(v1,v2,v3,faceplane,out_of_face)\
{   \
    _distances[0] = DISTANCE_PLANE_POINT(faceplane,v1);\
    _distances[1] =  _distances[0] * DISTANCE_PLANE_POINT(faceplane,v2);\
    _distances[2] =  _distances[0] * DISTANCE_PLANE_POINT(faceplane,v3); \
	if(_distances[1]<=0.0f || _distances[2]<=0.0f)\
	{\
	    out_of_face = 0;\
	}\
	else\
	{\
	    out_of_face = 1;\
	}\
}\

#define CLASSIFY_TRI_BY_PLANE(v1,v2,v3,faceplane,out_of_face)\
{   \
	if(DISTANCE_PLANE_POINT(faceplane,v1)<=0.0f)\
	{\
	    out_of_face = 0;\
	}\
	else if(DISTANCE_PLANE_POINT(faceplane,v2)<=0.0f)\
	{\
	    out_of_face = 0;\
	}\
	else if(DISTANCE_PLANE_POINT(faceplane,v3)<=0.0f)\
	{\
	    out_of_face = 0;\
	}\
	else\
	{\
	    out_of_face = 1;\
	}\
}\


//! Receives the 3 edge planes
#define MOST_DEEP_POINTS(plane,points,point_count,deep_points,deep_points_count,maxdeep)\
{\
    maxdeep=-1000.0f;\
    GUINT _k;\
	GREAL _dist;\
	deep_points_count = 0;\
	for(_k=0;_k<point_count;_k++)\
	{\
	    _dist = -DISTANCE_PLANE_POINT(plane,points[_k]);\
		if(_dist>maxdeep)\
		{\
			maxdeep = _dist;\
			_max_candidates[0] = _k;\
			deep_points_count=1;\
		}\
		else if((_dist+G_EPSILON)>=maxdeep)\
		{\
		    _max_candidates[deep_points_count] = _k;\
			deep_points_count++;\
		}\
	}\
	if(maxdeep<0.0f)\
    {\
        deep_points_count = 0;\
    }\
    else\
    {\
        for(_k=0;_k<deep_points_count;_k++)\
        {\
            VEC_COPY(deep_points[_k],points[_max_candidates[_k]]);\
        }\
    }\
}\

//! Receives the 3 edge planes
#define CLIP_TRI_POINTS_BY_TRI_EDGE_PLANES(tri_points,tri_edge_planes, clipped_points, clipped_point_count)\
{\
    clipped_point_count = 0;    \
    _temp_clip_count = 0;\
    PLANE_CLIP_POLYGON(tri_edge_planes[0],tri_points,3,_temp_clip,_temp_clip_count,MAX_TRI_CLIPPING);\
    if(_temp_clip_count>0)\
    {\
        _temp_clip_count2 = 0;\
        PLANE_CLIP_POLYGON(tri_edge_planes[1],_temp_clip,_temp_clip_count,_temp_clip2,_temp_clip_count2,MAX_TRI_CLIPPING);\
        if(_temp_clip_count2>0)\
        {\
            PLANE_CLIP_POLYGON(tri_edge_planes[2],_temp_clip2,_temp_clip_count2,clipped_points,clipped_point_count,MAX_TRI_CLIPPING);\
        }\
    }\
}\


static int _gim_triangle_triangle_collision(
							GIM_TRIANGLE_DATA *tri1,
							GIM_TRIANGLE_DATA *tri2,
							GIM_TRIANGLE_CONTACT_DATA * contact_data)
{


//Cache variables for triangle intersection
    GUINT _max_candidates[MAX_TRI_CLIPPING];
    vec3f _temp_clip[MAX_TRI_CLIPPING];
    GUINT _temp_clip_count = 0;
    vec3f _temp_clip2[MAX_TRI_CLIPPING];
    GUINT _temp_clip_count2 = 0;
    vec3f clipped_points2[MAX_TRI_CLIPPING];
    vec3f deep_points2[MAX_TRI_CLIPPING];
    vec3f clipped_points1[MAX_TRI_CLIPPING];
    vec3f deep_points1[MAX_TRI_CLIPPING];


    //State variabnles
	GUINT mostdir=0;
	GUINT clipped2_count=0;

	//Clip tri2 by tri1 edges

	CLIP_TRI_POINTS_BY_TRI_EDGE_PLANES(tri2->m_vertices,(&tri1->m_planes.m_planes[1]), clipped_points2, clipped2_count);

	if(clipped2_count == 0 )
	{
	     return 0;//Reject
	}

	//find most deep interval face1
	GUINT deep2_count=0;

	GREAL maxdeep;

	MOST_DEEP_POINTS((tri1->m_planes.m_planes[0]), clipped_points2, clipped2_count, deep_points2, deep2_count, maxdeep);
	if(deep2_count==0)
	{
//	    *perror = 0.0f;
	     return 0;//Reject
	}

	//Normal pointing to triangle1
	VEC_SCALE(contact_data->m_separating_normal,-1.0f,(tri1->m_planes.m_planes[0]));


	//Clip tri1 by tri2 edges

	GUINT clipped1_count=0;

	CLIP_TRI_POINTS_BY_TRI_EDGE_PLANES(tri1->m_vertices,(&tri2->m_planes.m_planes[1]), clipped_points1, clipped1_count);

	if(clipped2_count == 0 )
	{
//	    *perror = 0.0f;
	     return 0;//Reject
	}


	//find interval face2
	GUINT deep1_count=0;

	GREAL dist;

	MOST_DEEP_POINTS((tri2->m_planes.m_planes[0]), clipped_points1, clipped1_count, deep_points1, deep1_count, dist);

	if(deep1_count==0)
	{
//	    *perror = 0.0f;
	    return 0;
	}

	if(dist<maxdeep)
	{
		maxdeep = dist;
		mostdir = 1;
		VEC_COPY(contact_data->m_separating_normal,(tri2->m_planes.m_planes[0]));
	}
	//set deep
	contact_data->m_penetration_depth = maxdeep;

	////check most dir for contacts
	if(mostdir==0)
	{
	    contact_data->m_point_count = deep2_count;
	    for(mostdir=0;mostdir<deep2_count;mostdir++)
	    {
	        VEC_COPY(contact_data->m_points[mostdir] ,deep_points2[mostdir]);
	    }
	}
	else
	{
		contact_data->m_point_count = deep1_count;
	    for(mostdir=0;mostdir<deep1_count;mostdir++)
	    {
	        VEC_COPY(contact_data->m_points[mostdir] ,deep_points1[mostdir]);
	    }
	}
	return 1;
}



//! Finds the contact points from a collision of two triangles
/*!
Returns the contact points, the penetration depth and the separating normal of the collision
between two triangles. The normal is pointing toward triangle 1 from triangle 2
*/
int gim_triangle_triangle_collision(
							GIM_TRIANGLE_DATA *tri1,
							GIM_TRIANGLE_DATA *tri2,
							GIM_TRIANGLE_CONTACT_DATA * contact_data)
{
    vec3f _distances;
    char out_of_face=0;

	if(tri2->m_has_planes==0)
	{
		 TRIANGLE_PLANE(tri2->m_vertices[0],
			 tri2->m_vertices[1],tri2->m_vertices[2],tri2->m_planes.m_planes[0]);
	}
	//Fast overlapp
    CLASSIFY_TRI_BY_FACE(tri1->m_vertices[0],tri1->m_vertices[1],tri1->m_vertices[2],tri2->m_planes.m_planes[0],out_of_face);
    if(out_of_face==1) return 0;

	if(tri1->m_has_planes==0)
	{
		 TRIANGLE_PLANE(tri1->m_vertices[0],tri1->m_vertices[1],
			 tri1->m_vertices[2],tri1->m_planes.m_planes[0]);
	}
	//Fast overlapp
    CLASSIFY_TRI_BY_FACE(tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2],tri1->m_planes.m_planes[0],out_of_face);
    if(out_of_face==1) return 0;

	/*

	if(tri1->m_has_planes==0)
	{
		 EDGE_PLANE(tri1->m_vertices[0],tri1->m_vertices[1],
			 (tri1->m_planes.m_planes[0]),(tri1->m_planes.m_planes[1]));

	}


	//Fast overlapp
	CLASSIFY_TRI_BY_PLANE(tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2],tri1->m_planes.m_planes[1],out_of_face);
    if(out_of_face==1) return 0;

	if(tri1->m_has_planes==0)
	{
		 EDGE_PLANE(tri1->m_vertices[1],tri1->m_vertices[2],
			 (tri1->m_planes.m_planes[0]),(tri1->m_planes.m_planes[2]));

	}

	//Fast overlapp
	CLASSIFY_TRI_BY_PLANE(tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2],tri1->m_planes.m_planes[2],out_of_face);
    if(out_of_face==1) return 0;

	if(tri1->m_has_planes==0)
	{

		 EDGE_PLANE(tri1->m_vertices[2],tri1->m_vertices[0],
			 (tri1->m_planes.m_planes[0]),(tri1->m_planes.m_planes[3]));

	}

	//Fast overlapp
	CLASSIFY_TRI_BY_PLANE(tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2],tri1->m_planes.m_planes[3],out_of_face);
    if(out_of_face==1) return 0;

	if(tri2->m_has_planes==0)
	{
		 EDGE_PLANE(tri2->m_vertices[0],tri2->m_vertices[1],
			 (tri2->m_planes.m_planes[0]),(tri2->m_planes.m_planes[1]));
		 EDGE_PLANE(tri2->m_vertices[1],tri2->m_vertices[2],
			 (tri2->m_planes.m_planes[0]),(tri2->m_planes.m_planes[2]));
		 EDGE_PLANE(tri2->m_vertices[2],tri2->m_vertices[0],
			 (tri2->m_planes.m_planes[0]),(tri2->m_planes.m_planes[3]));
	}*/

	if(tri1->m_has_planes==0)
	{
		EDGE_PLANE(tri1->m_vertices[0],tri1->m_vertices[1],
			 (tri1->m_planes.m_planes[0]),(tri1->m_planes.m_planes[1]));
		 EDGE_PLANE(tri1->m_vertices[1],tri1->m_vertices[2],
			 (tri1->m_planes.m_planes[0]),(tri1->m_planes.m_planes[2]));
		 EDGE_PLANE(tri1->m_vertices[2],tri1->m_vertices[0],
			 (tri1->m_planes.m_planes[0]),(tri1->m_planes.m_planes[3]));
	}

	if(tri2->m_has_planes==0)
	{
		EDGE_PLANE(tri2->m_vertices[0],tri2->m_vertices[1],
			 (tri2->m_planes.m_planes[0]),(tri2->m_planes.m_planes[1]));
		 EDGE_PLANE(tri2->m_vertices[1],tri2->m_vertices[2],
			 (tri2->m_planes.m_planes[0]),(tri2->m_planes.m_planes[2]));
		 EDGE_PLANE(tri2->m_vertices[2],tri2->m_vertices[0],
			 (tri2->m_planes.m_planes[0]),(tri2->m_planes.m_planes[3]));
	}

    return _gim_triangle_triangle_collision(tri1,tri2,contact_data);
}


//! Trimesh Trimesh Collisions
/*!

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
void gim_trimesh_trimesh_collision(GIM_TRIMESH * trimesh1, GIM_TRIMESH * trimesh2, GDYNAMIC_ARRAY * contacts)
{
    contacts->m_size = 0;
    GDYNAMIC_ARRAY collision_pairs;
    GIM_CREATE_PAIR_SET(collision_pairs);

    char swaped;
    gim_trimesh_midphase_trimesh_collision(trimesh1,trimesh2,&collision_pairs,&swaped);

    if(collision_pairs.m_size==0)
    {
        GIM_DYNARRAY_DESTROY(collision_pairs);
         return; //no collisioin
    }

    //Locks meshes
    gim_trimesh_locks_work_data(trimesh1);
    gim_trimesh_locks_work_data(trimesh2);


    //pair pointer
    GUINT  *pairs = GIM_DYNARRAY_POINTER(GUINT,collision_pairs);
    //dummy contacts
    GDYNAMIC_ARRAY dummycontacts;
    GIM_CREATE_CONTACT_LIST(dummycontacts);

    //Auxiliary triangle data
    GIM_TRIANGLE_CONTACT_DATA tri_contact_data;
    GIM_TRIANGLE_DATA tri1data,tri2data;


    GUINT i, ti1,ti2,ci;
    int colresult;
    for (i=0;i<collision_pairs.m_size; i++)
    {
        ti1 = pairs[i*2 + swaped]; // m_index1
        ti2 = pairs[i*2 + (!swaped) ]; // m_index2
        //Get triangles data
        gim_trimesh_get_triangle_data_lazy(trimesh1,ti1,&tri1data);
        gim_trimesh_get_triangle_data_lazy(trimesh2,ti2,&tri2data);

        //collide triangles
        colresult = gim_triangle_triangle_collision(&tri1data,&tri2data,&tri_contact_data);
        if(colresult == 1)
        {
            //Add contacts
            for (ci=0;ci<tri_contact_data.m_point_count ;ci++ )
            {
                GIM_PUSH_CONTACT(dummycontacts, tri_contact_data.m_points[ci],tri_contact_data.m_separating_normal ,tri_contact_data.m_penetration_depth,trimesh1, trimesh2, ti1, ti2);
            }
        }
    }

    if(dummycontacts.m_size == 0) //reject
    {
		//Unlocks meshes
		gim_trimesh_unlocks_work_data(trimesh1);
		gim_trimesh_unlocks_work_data(trimesh2);
        GIM_DYNARRAY_DESTROY(dummycontacts);
        GIM_DYNARRAY_DESTROY(collision_pairs);
        return;
    }
    //merge contacts
    gim_merge_contacts(&dummycontacts,contacts);

    //Terminate
    GIM_DYNARRAY_DESTROY(dummycontacts);
    GIM_DYNARRAY_DESTROY(collision_pairs);

    //Unlocks meshes
    gim_trimesh_unlocks_work_data(trimesh1);
    gim_trimesh_unlocks_work_data(trimesh2);
}


//! Trimesh Plane Collisions
/*!

\param trimesh
\param plane vec4f plane
\param contacts A vec4f array. Must be initialized (~100). Each element have the coordinate point in the first 3 elements, and vec4f[3] has the penetration depth.
*/
void gim_trimesh_plane_collision(GIM_TRIMESH * trimesh,vec4f plane, GDYNAMIC_ARRAY * contacts)
{
    contacts->m_size = 0;
    char classify;
    aabb3f bound;
    gim_trimesh_get_aabb(trimesh,&bound);
    PLANE_CLASSIFY_BOX(plane,bound,classify);
    if(classify>1) return; // in front of plane

    //Locks mesh
    gim_trimesh_locks_work_data(trimesh);
    //Get vertices
    GUINT i, vertcount = gim_trimesh_get_vertex_count(trimesh);
    vec3f vec;

    GREAL dist;
    vec4f * result_contact;

    for (i=0;i<vertcount;i++)
    {
        gim_trimesh_get_vertex(trimesh,i,vec);
        dist = DISTANCE_PLANE_POINT(plane,vec);
        if(dist<=0.0f)
        {
             GIM_DYNARRAY_PUSH_EMPTY(vec4f,(*contacts));
             result_contact = GIM_DYNARRAY_POINTER_LAST(vec4f,(*contacts));
             VEC_COPY((*result_contact),vec);
             (*result_contact)[3] = -dist;
        }
    }
    gim_trimesh_unlocks_work_data(trimesh);
}
