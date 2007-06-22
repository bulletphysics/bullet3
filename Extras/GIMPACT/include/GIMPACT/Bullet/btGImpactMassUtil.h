/*! \file btGImpactMassUtil.h
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


#ifndef GIMPACT_MASS_UTIL_H
#define GIMPACT_MASS_UTIL_H

#include "LinearMath/btTransform.h"



SIMD_FORCE_INLINE btVector3 gim_inertia_add_transformed(
	const btVector3 & source_inertia, const btVector3 & added_inertia, const btTransform & transform)
{
	btMatrix3x3  rotatedTensor = transform.getBasis().scaled(added_inertia) * transform.getBasis().transpose();

	btScalar x2 = transform.getOrigin()[0];
	x2*= x2;
	btScalar y2 = transform.getOrigin()[1];
	y2*= y2;
	btScalar z2 = transform.getOrigin()[2];
	z2*= z2;

	btScalar ix = rotatedTensor[0][0]*(y2+z2);
	btScalar iy = rotatedTensor[1][1]*(x2+z2);
	btScalar iz = rotatedTensor[2][2]*(x2+y2);

	return btVector3(source_inertia[0]+ix,source_inertia[1]+iy,source_inertia[2] + iz);
}

SIMD_FORCE_INLINE btVector3 gim_get_point_inertia(const btVector3 & point, btScalar mass)
{
	btScalar x2 = point[0]*point[0];
	btScalar y2 = point[1]*point[1];
	btScalar z2 = point[2]*point[2];
	return btVector3(mass*(y2+z2),mass*(x2+z2),mass*(x2+y2));
}


#endif //GIMPACT_MESH_SHAPE_H
