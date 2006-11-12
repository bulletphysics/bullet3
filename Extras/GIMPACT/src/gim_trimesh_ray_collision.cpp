
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


//! Trimesh Ray Collisions
/*!

\param trimesh
\param contact
\return 1 if the ray collides, else 0
*/
int gim_trimesh_ray_collision(GIM_TRIMESH * trimesh,vec3f origin,vec3f dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA * contact)
{
    GDYNAMIC_ARRAY collision_result;
	GIM_CREATE_BOXQUERY_LIST(collision_result);

	if(trimesh->m_aabbset)
	{
		gim_aabbset_ray_collision(origin,dir,tmax,trimesh->m_aabbset,
			&collision_result);
	}
	else if(gim_trimesh_get_bound_method(trimesh)==G_TRIMESH_BOUND_AABB_TREE)
	{
		vec3f torigin, tdir;
		gim_trimesh_data_get(trimesh->m_trimesh_data_handle,
			&trimesh->m_unlocked_trimesh_data);

		MAT_DOT_VEC_3X4(torigin,trimesh->m_inv_transform,dir);
		MAT_DOT_VEC_3X3(tdir,trimesh->m_inv_transform,dir);
		gim_aabbtree_ray_collision(
			torigin,tdir, tmax,
			&trimesh->m_unlocked_trimesh_data->m_bv_tree, &collision_result);

		trimesh->m_unlocked_trimesh_data = 0;

	}

	if(collision_result.m_size==0)
	{
	    GIM_DYNARRAY_DESTROY(collision_result);
	    return 0;
	}

	//collide triangles

	GUINT * boxesresult = GIM_DYNARRAY_POINTER(GUINT,collision_result);
	GIM_TRIANGLE_DATA  tridata;
	vec3f pout;
	GREAL tparam,u,v;
	char does_intersect;

	gim_trimesh_locks_work_data(trimesh);

	for(unsigned int i=0;i<collision_result.m_size;i++)
	{
		gim_trimesh_get_triangle_data(trimesh,boxesresult[i],&tridata);

		RAY_TRIANGLE_INTERSECTION(origin,dir,tridata.m_vertices[0],tridata.m_vertices[1],tridata.m_vertices[2],tridata.m_planes.m_planes[0],pout,u,v,tparam,tmax,does_intersect);
		if(does_intersect)
		{
		    contact->tparam = tparam;
		    contact->u = u;
		    contact->v = v;
		    contact->m_face_id = boxesresult[i];
		    VEC_COPY(contact->m_point,pout);
		    VEC_COPY(contact->m_normal,tridata.m_planes.m_planes[0]);

		    gim_trimesh_unlocks_work_data(trimesh);
            GIM_DYNARRAY_DESTROY(collision_result);
		    return 1;
		}
	}

	gim_trimesh_unlocks_work_data(trimesh);
	GIM_DYNARRAY_DESTROY(collision_result);
	return 0;//no collisiion
}


//! Trimesh Ray Collisions closest
/*!
Find the closest primitive collided by the ray
\param trimesh
\param contact
\return 1 if the ray collides, else 0
*/
int gim_trimesh_ray_closest_collision(GIM_TRIMESH * trimesh,vec3f origin,vec3f dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA * contact)
{
    GDYNAMIC_ARRAY collision_result;
	GIM_CREATE_BOXQUERY_LIST(collision_result);

	if(trimesh->m_aabbset)
	{
		gim_aabbset_ray_collision(origin,dir,tmax,trimesh->m_aabbset,
			&collision_result);
	}
	else if(gim_trimesh_get_bound_method(trimesh)==G_TRIMESH_BOUND_AABB_TREE)
	{
		vec3f torigin, tdir;
		gim_trimesh_data_get(trimesh->m_trimesh_data_handle,
			&trimesh->m_unlocked_trimesh_data);

		MAT_DOT_VEC_3X4(torigin,trimesh->m_inv_transform,dir);
		MAT_DOT_VEC_3X3(tdir,trimesh->m_inv_transform,dir);
		gim_aabbtree_ray_collision(
			torigin,tdir, tmax,
			&trimesh->m_unlocked_trimesh_data->m_bv_tree, &collision_result);

		trimesh->m_unlocked_trimesh_data = 0;

	}

	if(collision_result.m_size==0)
	{
	    GIM_DYNARRAY_DESTROY(collision_result);
	    return 0;
	}

	//collide triangles

	GUINT * boxesresult = GIM_DYNARRAY_POINTER(GUINT,collision_result);
	GIM_TRIANGLE_DATA  tridata;
	vec3f pout;
	GREAL tparam,u,v;
	char does_intersect;
	contact->tparam = tmax + 0.1f;


	gim_trimesh_locks_work_data(trimesh);

	for(unsigned int i=0;i<collision_result.m_size;i++)
	{
		gim_trimesh_get_triangle_data(trimesh,boxesresult[i],&tridata);

		RAY_TRIANGLE_INTERSECTION(origin,dir,tridata.m_vertices[0],tridata.m_vertices[1],tridata.m_vertices[2],tridata.m_planes.m_planes[0],pout,u,v,tparam,tmax,does_intersect);
		if(does_intersect && (tparam < contact->tparam))
		{
            contact->tparam = tparam;
		    contact->u = u;
		    contact->v = v;
		    contact->m_face_id = boxesresult[i];
		    VEC_COPY(contact->m_point,pout);
		    VEC_COPY(contact->m_normal,tridata.m_planes.m_planes[0]);
		}
	}

	gim_trimesh_unlocks_work_data(trimesh);
	GIM_DYNARRAY_DESTROY(collision_result);
	if(contact->tparam > tmax) return 0;
	return 1;
}
