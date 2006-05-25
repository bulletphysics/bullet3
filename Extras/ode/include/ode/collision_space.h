/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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

#ifndef _ODE_COLLISION_SPACE_H_
#define _ODE_COLLISION_SPACE_H_

#include <ode/common.h>

#ifdef __cplusplus
extern "C" {
#endif

struct dContactGeom;

typedef void dNearCallback (void *data, dGeomID o1, dGeomID o2);


dSpaceID dSimpleSpaceCreate (dSpaceID space);
dSpaceID dHashSpaceCreate (dSpaceID space);
dSpaceID dQuadTreeSpaceCreate (dSpaceID space, dVector3 Center, dVector3 Extents, int Depth);

void dSpaceDestroy (dSpaceID);

void dHashSpaceSetLevels (dSpaceID space, int minlevel, int maxlevel);
void dHashSpaceGetLevels (dSpaceID space, int *minlevel, int *maxlevel);

void dSpaceSetCleanup (dSpaceID space, int mode);
int dSpaceGetCleanup (dSpaceID space);

void dSpaceAdd (dSpaceID, dGeomID);
void dSpaceRemove (dSpaceID, dGeomID);
int dSpaceQuery (dSpaceID, dGeomID);
void dSpaceClean (dSpaceID);
int dSpaceGetNumGeoms (dSpaceID);
dGeomID dSpaceGetGeom (dSpaceID, int i);


#ifdef __cplusplus
}
#endif

#endif
