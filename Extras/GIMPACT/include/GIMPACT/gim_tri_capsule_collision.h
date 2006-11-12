#ifndef GIM_TRI_CAPSULE_COLLISION_H_INCLUDED
#define GIM_TRI_CAPSULE_COLLISION_H_INCLUDED

/*! \file gim_tri_capsule_collision.h
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

#include "GIMPACT/gim_memory.h"

/*! \addtogroup GEOMETRIC_OPERATIONS
*/
//! @{

//! Capsule struct
struct GIM_CAPSULE_DATA
{
    GREAL m_radius;
    vec3f m_point1;
    vec3f m_point2;
};
//typedef struct _GIM_CAPSULE_DATA GIM_CAPSULE_DATA;

#define CALC_CAPSULE_AABB(capsule,aabb)\
{\
    if(capsule.m_point1[0]<capsule.m_point2[0])\
    {\
        aabb.minX = capsule.m_point1[0] - capsule.m_radius;\
        aabb.maxX = capsule.m_point2[0] + capsule.m_radius;\
    }\
    else\
    {\
        aabb.minX = capsule.m_point2[0] - capsule.m_radius;\
        aabb.maxX = capsule.m_point1[0] + capsule.m_radius;\
    }\
    if(capsule.m_point1[1]<capsule.m_point2[1])\
    {\
        aabb.minY = capsule.m_point1[1] - capsule.m_radius;\
        aabb.maxY = capsule.m_point2[1] + capsule.m_radius;\
    }\
    else\
    {\
        aabb.minY = capsule.m_point2[1] - capsule.m_radius;\
        aabb.maxY = capsule.m_point1[1] + capsule.m_radius;\
    }\
    if(capsule.m_point1[2]<capsule.m_point2[2])\
    {\
        aabb.minZ = capsule.m_point1[2] - capsule.m_radius;\
        aabb.maxZ = capsule.m_point2[2] + capsule.m_radius;\
    }\
    else\
    {\
        aabb.minZ = capsule.m_point2[2] - capsule.m_radius;\
        aabb.maxZ = capsule.m_point1[2] + capsule.m_radius;\
    }\
}\

//! Utility function for find the closest point between a segment and a triangle
/*!

\param triangle
\param s1
\param s2
\param contacts Contains the closest points on the segment (1,2), and the normal points to segment, and m_depth contains the distance

\post The contacts array is not set to 0. It adds aditional contacts
*/
void gim_closest_point_triangle_segment(GIM_TRIANGLE_DATA * triangle, vec3f s1,vec3f s2, GDYNAMIC_ARRAY * contacts);





//! Utility function for find the closest point between a capsule and a triangle
/*!

\param triangle
\param capsule
\param contacts Contains the closest points on the capsule, and the normal points to triangle
\return 1 if the triangle collides the capsule
\post The contacts array is not set to 0. It adds aditional contacts
*/
int gim_triangle_capsule_collision(GIM_TRIANGLE_DATA * triangle, GIM_CAPSULE_DATA * capsule, GDYNAMIC_ARRAY * contacts);
//! @}

#endif // GIM_TRI_CAPSULE_COLLISION_H_INCLUDED
