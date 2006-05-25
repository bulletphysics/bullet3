/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*

the standard ODE geometry primitives.

*/

#ifndef _ODE_COLLISION_STD_H_
#define _ODE_COLLISION_STD_H_

#include <ode/common.h>
#include "collision_kernel.h"


// primitive collision functions - these have the dColliderFn interface, i.e.
// the same interface as dCollide(). the first and second geom arguments must
// have the specified types.


int dCollideSphereSphere (dxGeom *o1, dxGeom *o2, int flags,
			  dContactGeom *contact, int skip);
int dCollideSphereBox (dxGeom *o1, dxGeom *o2, int flags,
		       dContactGeom *contact, int skip);
int dCollideSpherePlane (dxGeom *o1, dxGeom *o2, int flags,
			 dContactGeom *contact, int skip);
int dCollideBoxBox (dxGeom *o1, dxGeom *o2, int flags,
		    dContactGeom *contact, int skip);
int dCollideBoxPlane (dxGeom *o1, dxGeom *o2,
		      int flags, dContactGeom *contact, int skip);
int dCollideCCylinderSphere (dxGeom *o1, dxGeom *o2, int flags,
			     dContactGeom *contact, int skip);
int dCollideCCylinderBox (dxGeom *o1, dxGeom *o2, int flags,
			  dContactGeom *contact, int skip);
int dCollideCCylinderCCylinder (dxGeom *o1, dxGeom *o2,
				int flags, dContactGeom *contact, int skip);
int dCollideCCylinderPlane (dxGeom *o1, dxGeom *o2, int flags,
			    dContactGeom *contact, int skip);
int dCollideRaySphere (dxGeom *o1, dxGeom *o2, int flags,
		       dContactGeom *contact, int skip);
int dCollideRayBox (dxGeom *o1, dxGeom *o2, int flags,
		    dContactGeom *contact, int skip);
int dCollideRayCCylinder (dxGeom *o1, dxGeom *o2,
			  int flags, dContactGeom *contact, int skip);
int dCollideRayPlane (dxGeom *o1, dxGeom *o2, int flags,
		      dContactGeom *contact, int skip);
///dCollideConvexConvex: see collision_convex.cpp for details
int dCollideConvexConvex(dxGeom *o1, dxGeom *o2, int flags,
			  dContactGeom *contact, int skip);


#endif
