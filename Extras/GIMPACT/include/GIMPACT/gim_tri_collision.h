#ifndef GIM_TRI_COLLISION_H_INCLUDED
#define GIM_TRI_COLLISION_H_INCLUDED

/*! \file gim_tri_collision.h
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

/*! \addtogroup GEOMETRIC_OPERATIONS
*/
//! @{


#define MAX_TRI_CLIPPING 8

//! Clips a polygon by a plane
#define PLANE_CLIP_POLYGON(plane,polygon_points,polygon_point_count,clipped,clipped_count,max_clipped) \
{ \
    clipped_count = 0; \
    GUINT  _i, _vi, _prevclassif=32000, _classif; \
	GREAL _d; \
	for(_i=0;_i<=polygon_point_count;_i++) \
	{ \
		_vi = _i%polygon_point_count; \
		_d = DISTANCE_PLANE_POINT(plane,polygon_points[_vi]); \
		_classif = _d>G_EPSILON ?1:0; \
		if(_classif == 0) \
		{ \
			if(_prevclassif==1) \
			{\
				if(clipped_count<max_clipped) \
				{\
                    PLANE_CLIP_SEGMENT(polygon_points[_i-1],polygon_points[_vi],plane,clipped[clipped_count]); \
                    clipped_count++; \
                } \
            } \
			if(clipped_count<max_clipped&&_i<polygon_point_count) \
			{ \
			    VEC_COPY(clipped[clipped_count],polygon_points[_vi]); \
				clipped_count++; \
			} \
		} \
		else \
		{ \
            if(_prevclassif==0) \
            { \
                if(clipped_count<max_clipped) \
                { \
                    PLANE_CLIP_SEGMENT(polygon_points[_i-1],polygon_points[_vi],plane,clipped[clipped_count]); \
                    clipped_count++; \
                } \
            } \
		} \
		_prevclassif = _classif; \
	} \
}\


struct GIM_TRIPLANES_CACHE
{
    /*!
    Planes are:
    0 : Face normal plane (0,3)
    1 : Edge 1 plane (4,7)
    2 : Edge 2 plane (8,11)
    3 : Edge 3 plane (12,15)
    */
    vec4f m_planes[4];
};
//typedef struct _GIM_TRIPLANES_CACHE GIM_TRIPLANES_CACHE;


struct GIM_TRIANGLE_DATA
{
    vec3f m_vertices[3];
    GIM_TRIPLANES_CACHE m_planes;
	int m_has_planes;
};
//typedef struct _GIM_TRIANGLE_DATA GIM_TRIANGLE_DATA;

//! tri_data is a GIM_TRIANGLE_DATA
#define GIM_CALC_TRIANGLE_DATA_PLANES(tri_data)\
{\
        TRIANGLE_PLANE((tri_data).m_vertices[0],(tri_data).m_vertices[1],(tri_data).m_vertices[2],(tri_data).m_planes.m_planes[0]);\
        EDGE_PLANE((tri_data).m_vertices[0],(tri_data).m_vertices[1],((tri_data).m_planes.m_planes[0]),((tri_data).m_planes.m_planes[1]));\
        EDGE_PLANE((tri_data).m_vertices[1],(tri_data).m_vertices[2],((tri_data).m_planes.m_planes[0]),((tri_data).m_planes.m_planes[2]));\
        EDGE_PLANE((tri_data).m_vertices[2],(tri_data).m_vertices[0],((tri_data).m_planes.m_planes[0]), ((tri_data).m_planes.m_planes[3]));\
}\

//Structure for collision

struct GIM_TRIANGLE_CONTACT_DATA
{
    GREAL m_penetration_depth;
    GUINT m_point_count;
    vec3f m_separating_normal;
    vec3f m_points[MAX_TRI_CLIPPING];
};
//typedef struct _GIM_TRIANGLE_CONTACT_DATA GIM_TRIANGLE_CONTACT_DATA;

struct GIM_TRIANGLE_RAY_CONTACT_DATA
{
    GREAL u;
    GREAL v;
    GREAL tparam;
    GUINT m_face_id;
    vec3f m_point;
    vec3f m_normal;
};
//typedef struct _GIM_TRIANGLE_RAY_CONTACT_DATA GIM_TRIANGLE_RAY_CONTACT_DATA;

//! Fast Triangle Triangle overlapping test
int gim_triangle_triangle_overlap(
							GIM_TRIANGLE_DATA *tri1,
							GIM_TRIANGLE_DATA *tri2);


//! Fast but inacurate conservative Triangle Triangle overlapping test
int gim_triangle_triangle_overlap_fast(
							GIM_TRIANGLE_DATA *tri1,
							GIM_TRIANGLE_DATA *tri2);


//! Finds the contact points from a collision of two triangles
/*!
Returns the contact points, the penetration depth and the separating normal of the collision
between two triangles. The normal is pointing toward triangle 1 from triangle 2
*/
int gim_triangle_triangle_collision(
							GIM_TRIANGLE_DATA *tri1,
							GIM_TRIANGLE_DATA *tri2,
							GIM_TRIANGLE_CONTACT_DATA * contact_data);

//Ray triangle


/*!
	Solve the System for u,v parameters:

	u*axe1[i1] + v*axe2[i1] = vecproj[i1]
	u*axe1[i2] + v*axe2[i2] = vecproj[i2]

	sustitute:
	v = (vecproj[i2] - u*axe1[i2])/axe2[i2]

	then the first equation in terms of 'u':

	--> u*axe1[i1] + ((vecproj[i2] - u*axe1[i2])/axe2[i2])*axe2[i1] = vecproj[i1]

	--> u*axe1[i1] + vecproj[i2]*axe2[i1]/axe2[i2] - u*axe1[i2]*axe2[i1]/axe2[i2] = vecproj[i1]

	--> u*(axe1[i1]  - axe1[i2]*axe2[i1]/axe2[i2]) = vecproj[i1] - vecproj[i2]*axe2[i1]/axe2[i2]

	--> u*((axe1[i1]*axe2[i2]  - axe1[i2]*axe2[i1])/axe2[i2]) = (vecproj[i1]*axe2[i2] - vecproj[i2]*axe2[i1])/axe2[i2]

	--> u*(axe1[i1]*axe2[i2]  - axe1[i2]*axe2[i1]) = vecproj[i1]*axe2[i2] - vecproj[i2]*axe2[i1]

	--> u = (vecproj[i1]*axe2[i2] - vecproj[i2]*axe2[i1]) /(axe1[i1]*axe2[i2]  - axe1[i2]*axe2[i1])

if 0.0<= u+v <=1.0 then they are inside of triangle

	*/
#define TRIANGLE_GET_UVPARAMETERS(point,vec1,vec2,vec3,tri_plane,u,v,outside)\
{\
	vec3f _axe1, _axe2, _vecproj;\
	VEC_DIFF(_axe1,vec2,vec1);\
	VEC_DIFF(_axe2,vec3,vec1);\
	VEC_DIFF(_vecproj,point,vec1);\
	GUINT _i1,_i2;\
	PLANE_MINOR_AXES(tri_plane, _i1, _i2);\
	if(fabsf(_axe2[_i2])<G_EPSILON)\
	{\
		u = (_vecproj[_i2]*_axe2[_i1] - _vecproj[_i1]*_axe2[_i2]) /(_axe1[_i2]*_axe2[_i1]  - _axe1[_i1]*_axe2[_i2]);\
		v = (_vecproj[_i1] - u*_axe1[_i1])/_axe2[_i1];\
	}\
	else\
	{\
		u = (_vecproj[_i1]*_axe2[_i2] - _vecproj[_i2]*_axe2[_i1]) /(_axe1[_i1]*_axe2[_i2]  - _axe1[_i2]*_axe2[_i1]);\
		v = (_vecproj[_i2] - u*_axe1[_i2])/_axe2[_i2];\
	}\
	if(u<-G_EPSILON)\
	{\
		outside = 1;\
	}\
	else if(v<-G_EPSILON)\
	{\
		outside = 1;\
	}\
	else\
	{\
		float sumuv;\
		sumuv = u+v;\
		if(sumuv<-G_EPSILON)\
		{\
			outside = 1;\
		}\
		else if(sumuv-1.0f>G_EPSILON)\
		{\
			outside = 1;\
		}\
		else\
		{\
			outside = 0;\
		}\
	}\
}\

//! Finds the collision of a ray and a triangle.
#define RAY_TRIANGLE_INTERSECTION(vOrigin,vDir,vec1,vec2,vec3,tri_plane,pout,u,v,tparam,tmax,does_intersect)\
{\
	RAY_PLANE_COLLISION(tri_plane,vDir,vOrigin,pout,tparam,does_intersect);\
	if(does_intersect != 0)\
	{\
        if(tparam<-G_EPSILON||tparam>tmax+G_EPSILON)\
        {\
            does_intersect = 0;\
        }\
        else\
        {\
            TRIANGLE_GET_UVPARAMETERS(pout,vec1,vec2,vec3,tri_plane,u,v,does_intersect);\
            does_intersect = !does_intersect;\
        }\
	}\
}\


//! @}

#endif // GIM_TRI_COLLISION_H_INCLUDED
