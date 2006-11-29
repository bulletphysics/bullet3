
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

//! Utility function for find the closest point between a segment and a triangle
/*!

\param triangle
\param s1
\param s2
\param contacts Contains the closest points on the segment (1,2), and the normal points to segment, and m_depth contains the distance

\post The contacts array is not set to 0. It adds aditional contacts
*/
void gim_closest_point_triangle_segment(GIM_TRIANGLE_DATA * triangle, vec3f s1,vec3f s2, GDYNAMIC_ARRAY * contacts)
{
    vec3f segment_points[4];
    vec3f closest_points[2];
    GUINT intersection_type, out_edge= 10;
    GREAL dis, dis_temp,perpend;
    vec4f sdiff;

    dis = DISTANCE_PLANE_POINT(triangle->m_planes.m_planes[0],s1);
    dis_temp = DISTANCE_PLANE_POINT(triangle->m_planes.m_planes[0],s2);

    if(dis<=0.0f && dis_temp<=0.0f) return;

    VEC_DIFF(sdiff,s2,s1);
    perpend = VEC_DOT(sdiff,triangle->m_planes.m_planes[0]);

    if(!GIM_IS_ZERO(perpend)) // Not perpendicular
    {
        if(dis<dis_temp)
        {
            VEC_COPY(closest_points[0],s1);
        }
        else
        {
            dis = dis_temp;
            VEC_COPY(closest_points[0],s2);
        }

        //Testing segment vertices over triangle
        if(dis>=0.0f && dis_temp>=0.0f)
        {
            POINT_IN_HULL(closest_points[0],(&triangle->m_planes.m_planes[1]),3,out_edge);

            if(out_edge==0)//Point over face
            {
                GIM_PUSH_CONTACT((*contacts),closest_points[0] ,triangle->m_planes.m_planes[0] ,dis,0, 0, 0,0);
                return;
            }
        }
        else
        {

            PLANE_CLIP_SEGMENT(s1,s2,triangle->m_planes.m_planes[0],closest_points[1]);

            POINT_IN_HULL(closest_points[1],(&triangle->m_planes.m_planes[1]),3,out_edge);

            if(out_edge==0)//Point over face
            {
                GIM_PUSH_CONTACT((*contacts),closest_points[0] ,triangle->m_planes.m_planes[0] ,dis,0, 0, 0,0);
                return;
            }
        }

    }
    else // Perpendicular Face
    {
        //out_edge=10
        //Clip segment by triangle
    //    Edge1
        PLANE_CLIP_SEGMENT_CLOSEST(s1,s2,triangle->m_planes.m_planes[1],segment_points[0],segment_points[1],intersection_type);
        if(intersection_type==0||intersection_type==1)
        {
            out_edge = 0;
            VEC_COPY(closest_points[0],segment_points[0]);
        }
        else
        {
            //Edge2
            PLANE_CLIP_SEGMENT_CLOSEST(segment_points[0],segment_points[1],triangle->m_planes.m_planes[2],segment_points[2],segment_points[3],intersection_type);
            if(intersection_type==0||intersection_type==1)
            {
                out_edge = 1;
                VEC_COPY(closest_points[0],segment_points[3]);
            }
            else
            {
                //Edge3
                PLANE_CLIP_SEGMENT_CLOSEST(segment_points[2],segment_points[3],triangle->m_planes.m_planes[3],closest_points[0],closest_points[1],intersection_type);
                if(intersection_type==0||intersection_type==1)
                {
                    out_edge = 2;
                }
            }
        }
        //POST closest_points[0] and closest_points[1] are inside the triangle, if out_edge>2
        if(out_edge>2) // Over triangle
        {
            dis = VEC_DOT(closest_points[0],triangle->m_planes.m_planes[0]);
            GIM_PUSH_CONTACT((*contacts),closest_points[0] ,triangle->m_planes.m_planes[0] ,dis,0, 0, 0,0);
            GIM_PUSH_CONTACT((*contacts),closest_points[1] ,triangle->m_planes.m_planes[0] ,dis,0, 0, 0,0);
            return;
        }
    }

    //Find closest edges
    out_edge = 10;
    dis = G_REAL_INFINITY;
    GUINT i;
    for(i=0;i<3;i++)
    {
        SEGMENT_COLLISION(s1,s2,triangle->m_vertices[i],triangle->m_vertices[(i+1)%3],segment_points[0],segment_points[1]);
        VEC_DIFF(sdiff,segment_points[0],segment_points[1]);
        dis_temp = VEC_DOT(sdiff,sdiff);
        if(dis_temp< dis)
        {
            dis = dis_temp;
            out_edge = i;
            VEC_COPY(closest_points[0],segment_points[0]);
            VEC_COPY(closest_points[1],sdiff);//normal
        }
    }
    if(out_edge>2) return ;// ???? btAssert this please

    if(GIM_IS_ZERO(dis))
    {
        //Set face plane
        GIM_PUSH_CONTACT((*contacts),closest_points[0] ,triangle->m_planes.m_planes[0] ,0.0f,0, 0, 0,0);

    }
    else
    {
        GIM_SQRT(dis,dis);
        VEC_SCALE(closest_points[1],(1.0f/dis),closest_points[1]);//normal
        GIM_PUSH_CONTACT((*contacts),closest_points[0] ,closest_points[1],dis,0, 0, 0,0);
    }
}


//! Utility function for find the closest point between a capsule and a triangle
/*!

\param triangle
\param capsule
\param contacts Contains the closest points on the capsule, and the normal points to triangle

\post The contacts array is not set to 0. It adds aditional contacts
*/
int gim_triangle_capsule_collision(GIM_TRIANGLE_DATA * triangle, GIM_CAPSULE_DATA * capsule, GDYNAMIC_ARRAY * contacts)
{
    GUINT old_contact_size = contacts->m_size;
    gim_closest_point_triangle_segment(triangle,capsule->m_point1,capsule->m_point2,contacts);
    GIM_CONTACT * pcontact = GIM_DYNARRAY_POINTER(GIM_CONTACT ,(*contacts));
    pcontact+= old_contact_size;

    if(pcontact->m_depth > capsule->m_radius)
    {
        contacts->m_size = old_contact_size;
        return 0;
    }

    vec3f vec;
    while(old_contact_size<contacts->m_size)
    {
        //Scale the normal for pointing to triangle
        VEC_SCALE(pcontact->m_normal,-1.0f,pcontact->m_normal);
        //Fix the contact point
        VEC_SCALE(vec,capsule->m_radius,pcontact->m_normal);
        VEC_SUM(pcontact->m_point,vec,pcontact->m_point);
        //Fix the depth
        pcontact->m_depth = capsule->m_radius - pcontact->m_depth;

        pcontact++;
        old_contact_size++;
    }

    return 1;
}


//! Trimesh Capsule collision
/*!
Find the closest primitive collided by the ray
\param trimesh
\param capsule
\param contact
\param contacts A GIM_CONTACT array. Must be initialized
*/
void gim_trimesh_capsule_collision(GIM_TRIMESH * trimesh, GIM_CAPSULE_DATA * capsule, GDYNAMIC_ARRAY * contacts)
{
    contacts->m_size = 0;

    aabb3f test_aabb;
    CALC_CAPSULE_AABB((*capsule),test_aabb);

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
	GIM_TRIANGLE_DATA tri_data;
	GUINT old_contact_size;
	GIM_CONTACT * pcontact;

	for(i=0;i<collision_result.m_size;i++)
	{
	    old_contact_size = dummycontacts.m_size;
		gim_trimesh_get_triangle_data(trimesh,boxesresult[i],&tri_data);
		cresult = gim_triangle_capsule_collision(&tri_data, capsule, &dummycontacts);
		if(cresult!=0)
		{
		    pcontact = GIM_DYNARRAY_POINTER(GIM_CONTACT ,dummycontacts);
            pcontact+= old_contact_size;
		    while(old_contact_size<dummycontacts.m_size)
            {
                pcontact->m_handle1 = trimesh;
                pcontact->m_handle2 = capsule;
                pcontact->m_feature1 = boxesresult[i];
                pcontact->m_feature2 = 0;
                pcontact++;
                old_contact_size++;
            }
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
