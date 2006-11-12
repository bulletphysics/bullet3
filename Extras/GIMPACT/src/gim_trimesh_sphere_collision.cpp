
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

int gim_triangle_sphere_collision(
							GIM_TRIANGLE_DATA *tri,
							vec3f center, GREAL radius,
							GIM_TRIANGLE_CONTACT_DATA * contact_data)
{
    contact_data->m_point_count = 0;

    //Find Face plane distance
    GREAL  dis = DISTANCE_PLANE_POINT(tri->m_planes.m_planes[0],center);
    if(dis>radius) return 0; //out
    if(dis<-radius) return 0;//Out of triangle
    contact_data->m_penetration_depth = dis;

    //Find the most edge
    GUINT most_edge = 4;//no edge
    GREAL max_dis = 0.0f;
    dis = DISTANCE_PLANE_POINT(tri->m_planes.m_planes[1],center);
    if(dis>radius) return 0;//Out of triangle
    if(dis>0.0f)
    {
        max_dis = dis;
        most_edge = 0;
    }

    dis = DISTANCE_PLANE_POINT(tri->m_planes.m_planes[2],center);
    if(dis>radius) return 0;//Out of triangle
    if(dis>max_dis)// && dis>0.0f)
    {
        max_dis = dis;
        most_edge = 1;
    }

    dis = DISTANCE_PLANE_POINT(tri->m_planes.m_planes[3],center);
    if(dis>radius) return 0;//Out of triangle
    if(dis>max_dis)// && dis>0.0f)
    {
        max_dis = dis;
        most_edge = 2;
    }

    if(most_edge == 4) //Box is into triangle
    {
        //contact_data->m_penetration_depth = dis is set above
        //Find Face plane point
        VEC_COPY(contact_data->m_separating_normal,tri->m_planes.m_planes[0]);
        //Find point projection on plane
        if(contact_data->m_penetration_depth>=0.0f)
        {
            VEC_SCALE(contact_data->m_points[0],-radius,contact_data->m_separating_normal);
        }
        else
        {
            VEC_SCALE(contact_data->m_points[0],radius,contact_data->m_separating_normal);
        }
        contact_data->m_penetration_depth = radius - contact_data->m_penetration_depth;

        VEC_SUM(contact_data->m_points[0],contact_data->m_points[0],center);
        //Scale normal for pointing to triangle
        VEC_SCALE(contact_data->m_separating_normal,-1.0f,contact_data->m_separating_normal);
        contact_data->m_point_count = 1;
        return 1;
    }
    //find the edge
    vec3f e1,e2;
    VEC_COPY(e1,tri->m_vertices[most_edge]);
    VEC_COPY(e2,tri->m_vertices[(most_edge+1)%3]);

    CLOSEST_POINT_ON_SEGMENT(contact_data->m_points[0],center,e1,e2);
    //find distance
    VEC_DIFF(e1,center,contact_data->m_points[0]);
    VEC_LENGTH(e1,dis);
    if(dis>radius) return 0;

    contact_data->m_penetration_depth = radius - dis;

    if(GIM_IS_ZERO(dis))
    {
        VEC_COPY(contact_data->m_separating_normal,tri->m_planes.m_planes[most_edge+1]);
        VEC_SCALE(contact_data->m_points[0],-radius,contact_data->m_separating_normal);
        VEC_SUM(contact_data->m_points[0],contact_data->m_points[0],center);
    }
    else
    {
        VEC_SCALE(contact_data->m_separating_normal,1.0f/dis,e1);
        VEC_SCALE(contact_data->m_points[0],-radius,contact_data->m_separating_normal);
        VEC_SUM(contact_data->m_points[0],contact_data->m_points[0],center);
    }

    //Scale normal for pointing to triangle
    VEC_SCALE(contact_data->m_separating_normal,-1.0f,contact_data->m_separating_normal);

    contact_data->m_point_count = 1;
    return 1;

}

//! Trimesh Sphere Collisions
/*!
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
void gim_trimesh_sphere_collision(GIM_TRIMESH * trimesh,vec3f center,GREAL radius, GDYNAMIC_ARRAY * contacts)
{
    contacts->m_size = 0;

    aabb3f test_aabb;
	test_aabb.minX = center[0]-radius;
	test_aabb.maxX = center[0]+radius;
	test_aabb.minY = center[1]-radius;
	test_aabb.maxY = center[1]+radius;
	test_aabb.minZ = center[2]-radius;
	test_aabb.maxZ = center[2]+radius;

	GDYNAMIC_ARRAY collision_result;
	GIM_CREATE_BOXQUERY_LIST(collision_result);

	gim_trimesh_midphase_box_collision(trimesh,&test_aabb,&collision_result);

	if(collision_result.m_size==0)
	{
	    GIM_DYNARRAY_DESTROY(collision_result);
	}

	//collide triangles
	//Locks trimesh
	gim_trimesh_locks_work_data(trimesh);
	 //dummy contacts
    GDYNAMIC_ARRAY dummycontacts;
    GIM_CREATE_CONTACT_LIST(dummycontacts);

	int cresult;
	unsigned int i;
	GUINT * boxesresult = GIM_DYNARRAY_POINTER(GUINT,collision_result);
	GIM_TRIANGLE_CONTACT_DATA tri_contact_data;
	GIM_TRIANGLE_DATA tri_data;

	for(i=0;i<collision_result.m_size;i++)
	{
		gim_trimesh_get_triangle_data(trimesh,boxesresult[i],&tri_data);
		cresult = gim_triangle_sphere_collision(&tri_data,center,radius,&tri_contact_data);
		if(cresult!=0)
		{
		    GIM_PUSH_CONTACT(dummycontacts, tri_contact_data.m_points[0],tri_contact_data.m_separating_normal ,tri_contact_data.m_penetration_depth,trimesh, 0, boxesresult[i],0);
		}
	}
	///unlocks
	gim_trimesh_unlocks_work_data(trimesh);
	///Destroy box result
	GIM_DYNARRAY_DESTROY(collision_result);

	 //merge contacts
    gim_merge_contacts(&dummycontacts,contacts);

    //Destroy dummy
    GIM_DYNARRAY_DESTROY(dummycontacts);
}

