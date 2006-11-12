#ifndef GIM_TRI_SPHERE_COLLISION_H_INCLUDED
#define GIM_TRI_SPHERE_COLLISION_H_INCLUDED

/*! \file gim_tri_sphere_collision.h
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

//! Finds the contact points from a collision of a triangle and a sphere
/*!
\param tri
\param center
\param radius
\param contact_data Contains the closest points on the Sphere, and the normal is pointing to triangle
*/
int gim_triangle_sphere_collision(
							GIM_TRIANGLE_DATA *tri,
							vec3f center, GREAL radius,
							GIM_TRIANGLE_CONTACT_DATA * contact_data);

//! @}
#endif // GIM_TRI_SPHERE_COLLISION_H_INCLUDED
