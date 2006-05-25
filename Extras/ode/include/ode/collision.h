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

#ifndef _ODE_COLLISION_H_
#define _ODE_COLLISION_H_

#include <ode/common.h>
#include <ode/collision_space.h>
#include <ode/contact.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ************************************************************************ */
/* general functions */

void dGeomDestroy (dGeomID);
void dGeomSetData (dGeomID, void *);
void *dGeomGetData (dGeomID);
void dGeomSetBody (dGeomID, dBodyID);
dBodyID dGeomGetBody (dGeomID);
void dGeomSetPosition (dGeomID, dReal x, dReal y, dReal z);
void dGeomSetRotation (dGeomID, const dMatrix3 R);
void dGeomSetQuaternion (dGeomID, const dQuaternion);
const dReal * dGeomGetPosition (dGeomID);
const dReal * dGeomGetRotation (dGeomID);
void dGeomGetQuaternion (dGeomID, dQuaternion result);
void dGeomGetAABB (dGeomID, dReal aabb[6]);
int dGeomIsSpace (dGeomID);
dSpaceID dGeomGetSpace (dGeomID);
int dGeomGetClass (dGeomID);
void dGeomSetCategoryBits (dGeomID, unsigned long bits);
void dGeomSetCollideBits (dGeomID, unsigned long bits);
unsigned long dGeomGetCategoryBits (dGeomID);
unsigned long dGeomGetCollideBits (dGeomID);
void dGeomEnable (dGeomID);
void dGeomDisable (dGeomID);
int dGeomIsEnabled (dGeomID);

/* ************************************************************************ */
/* collision detection */

int dCollide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact,
	      int skip);
void dSpaceCollide (dSpaceID space, void *data, dNearCallback *callback);
void dSpaceCollide2 (dGeomID o1, dGeomID o2, void *data,
		     dNearCallback *callback);

/* ************************************************************************ */
/* standard classes */

/* the maximum number of user classes that are supported */
enum {
  dMaxUserClasses = 4
};

/* class numbers - each geometry object needs a unique number */
enum {
  dSphereClass = 0,
  dBoxClass,
  dCCylinderClass,
  dCylinderClass,
  dPlaneClass,
  dRayClass,
  dGeomTransformClass,
  dTriMeshClass,
  dConvexClass,

  dFirstSpaceClass,
  dSimpleSpaceClass = dFirstSpaceClass,
  dHashSpaceClass,
  dQuadTreeSpaceClass,
  dLastSpaceClass = dQuadTreeSpaceClass,

  dFirstUserClass,
  dLastUserClass = dFirstUserClass + dMaxUserClasses - 1,
  dGeomNumClasses
};


dGeomID dCreateSphere (dSpaceID space, dReal radius);
void dGeomSphereSetRadius (dGeomID sphere, dReal radius);
dReal dGeomSphereGetRadius (dGeomID sphere);
dReal dGeomSpherePointDepth (dGeomID sphere, dReal x, dReal y, dReal z);

dGeomID dCreateBox (dSpaceID space, dReal lx, dReal ly, dReal lz);
void dGeomBoxSetLengths (dGeomID box, dReal lx, dReal ly, dReal lz);
void dGeomBoxGetLengths (dGeomID box, dVector3 result);
dReal dGeomBoxPointDepth (dGeomID box, dReal x, dReal y, dReal z);

dGeomID dCreatePlane (dSpaceID space, dReal a, dReal b, dReal c, dReal d);
void dGeomPlaneSetParams (dGeomID plane, dReal a, dReal b, dReal c, dReal d);
void dGeomPlaneGetParams (dGeomID plane, dVector4 result);
dReal dGeomPlanePointDepth (dGeomID plane, dReal x, dReal y, dReal z);

dGeomID dCreateCCylinder (dSpaceID space, dReal radius, dReal length);
void dGeomCCylinderSetParams (dGeomID ccylinder, dReal radius, dReal length);
void dGeomCCylinderGetParams (dGeomID ccylinder, dReal *radius, dReal *length);
dReal dGeomCCylinderPointDepth (dGeomID ccylinder, dReal x, dReal y, dReal z);

dGeomID dCreateRay (dSpaceID space, dReal length);
void dGeomRaySetLength (dGeomID ray, dReal length);
dReal dGeomRayGetLength (dGeomID ray);
void dGeomRaySet (dGeomID ray, dReal px, dReal py, dReal pz,
		  dReal dx, dReal dy, dReal dz);
void dGeomRayGet (dGeomID ray, dVector3 start, dVector3 dir);

/*
 * Set/get ray flags that influence ray collision detection.
 * These flags are currently only noticed by the trimesh collider, because
 * they can make a major differences there.
 */
void dGeomRaySetParams (dGeomID g, int FirstContact, int BackfaceCull);
void dGeomRayGetParams (dGeomID g, int *FirstContact, int *BackfaceCull);
void dGeomRaySetClosestHit (dGeomID g, int closestHit);
int dGeomRayGetClosestHit (dGeomID g);

#include "collision_trimesh.h"

dGeomID dCreateGeomTransform (dSpaceID space);
void dGeomTransformSetGeom (dGeomID g, dGeomID obj);
dGeomID dGeomTransformGetGeom (dGeomID g);
void dGeomTransformSetCleanup (dGeomID g, int mode);
int dGeomTransformGetCleanup (dGeomID g);
void dGeomTransformSetInfo (dGeomID g, int mode);
int dGeomTransformGetInfo (dGeomID g);

/* ************************************************************************ */
/* utility functions */

void dClosestLineSegmentPoints (const dVector3 a1, const dVector3 a2,
				const dVector3 b1, const dVector3 b2,
				dVector3 cp1, dVector3 cp2);

int dBoxTouchesBox (const dVector3 _p1, const dMatrix3 R1,
		    const dVector3 side1, const dVector3 _p2,
		    const dMatrix3 R2, const dVector3 side2);

void dInfiniteAABB (dGeomID geom, dReal aabb[6]);
void dCloseODE(void);

/* ************************************************************************ */
/* custom classes */

typedef void dGetAABBFn (dGeomID, dReal aabb[6]);
typedef int dColliderFn (dGeomID o1, dGeomID o2,
			 int flags, dContactGeom *contact, int skip);
typedef dColliderFn * dGetColliderFnFn (int num);
typedef void dGeomDtorFn (dGeomID o);
typedef int dAABBTestFn (dGeomID o1, dGeomID o2, dReal aabb[6]);

typedef struct dGeomClass {
  int bytes;
  dGetColliderFnFn *collider;
  dGetAABBFn *aabb;
  dAABBTestFn *aabb_test;
  dGeomDtorFn *dtor;
} dGeomClass;

int dCreateGeomClass (const dGeomClass *classptr);
void * dGeomGetClassData (dGeomID);
dGeomID dCreateGeom (int classnum);

/* ************************************************************************ */

#ifdef __cplusplus
}
#endif

#endif
