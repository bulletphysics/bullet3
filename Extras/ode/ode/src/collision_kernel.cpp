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

/*

core collision functions and data structures, plus part of the public API
for geometry objects

*/

#include <ode/common.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/objects.h>
#include "collision_kernel.h"
#include "collision_util.h"
#include "collision_std.h"
#include "collision_transform.h"
#include "collision_trimesh_internal.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

//****************************************************************************
// helper functions for dCollide()ing a space with another geom

// this struct records the parameters passed to dCollideSpaceGeom()

struct SpaceGeomColliderData {
  int flags;			// space left in contacts array
  dContactGeom *contact;
  int skip;
};


static void space_geom_collider (void *data, dxGeom *o1, dxGeom *o2)
{
  SpaceGeomColliderData *d = (SpaceGeomColliderData*) data;
  if (d->flags & NUMC_MASK) {
    int n = dCollide (o1,o2,d->flags,d->contact,d->skip);
    d->contact = CONTACT (d->contact,d->skip*n);
    d->flags -= n;
  }
}


static int dCollideSpaceGeom (dxGeom *o1, dxGeom *o2, int flags,
			      dContactGeom *contact, int skip)
{
  SpaceGeomColliderData data;
  data.flags = flags;
  data.contact = contact;
  data.skip = skip;
  dSpaceCollide2 (o1,o2,&data,&space_geom_collider);
  return (flags & NUMC_MASK) - (data.flags & NUMC_MASK);
}

//****************************************************************************
// dispatcher for the N^2 collider functions

// function pointers and modes for n^2 class collider functions

struct dColliderEntry {
  dColliderFn *fn;	// collider function, 0 = no function available
  int reverse;		// 1 = reverse o1 and o2
};
static dColliderEntry colliders[dGeomNumClasses][dGeomNumClasses];
static int colliders_initialized = 0;


// setCollider() will refuse to write over a collider entry once it has
// been written.

void setCollider (int i, int j, dColliderFn *fn)
{
  if (colliders[i][j].fn == 0) {
    colliders[i][j].fn = fn;
    colliders[i][j].reverse = 0;
  }
  if (colliders[j][i].fn == 0) {
    colliders[j][i].fn = fn;
    colliders[j][i].reverse = 1;
  }
}


static void setAllColliders (int i, dColliderFn *fn)
{
  for (int j=0; j<dGeomNumClasses; j++) setCollider (i,j,fn);
}


static void initColliders()
{
  int i,j;

  if (colliders_initialized) return;
  colliders_initialized = 1;

  memset (colliders,0,sizeof(colliders));

  // setup space colliders
  for (i=dFirstSpaceClass; i <= dLastSpaceClass; i++) {
    for (j=0; j < dGeomNumClasses; j++) {
      setCollider (i,j,&dCollideSpaceGeom);
    }
  }

  setCollider (dSphereClass,dSphereClass,&dCollideSphereSphere);
  setCollider (dSphereClass,dBoxClass,&dCollideSphereBox);
  setCollider (dSphereClass,dPlaneClass,&dCollideSpherePlane);
  setCollider (dBoxClass,dBoxClass,&dCollideBoxBox);
  setCollider (dBoxClass,dPlaneClass,&dCollideBoxPlane);
  setCollider (dCCylinderClass,dSphereClass,&dCollideCCylinderSphere);
  setCollider (dCCylinderClass,dBoxClass,&dCollideCCylinderBox);
  setCollider (dCCylinderClass,dCCylinderClass,&dCollideCCylinderCCylinder);
  setCollider (dCCylinderClass,dPlaneClass,&dCollideCCylinderPlane);
  setCollider (dRayClass,dSphereClass,&dCollideRaySphere);
  setCollider (dRayClass,dBoxClass,&dCollideRayBox);
  setCollider (dRayClass,dCCylinderClass,&dCollideRayCCylinder);
  setCollider (dRayClass,dPlaneClass,&dCollideRayPlane);
#ifdef dTRIMESH_ENABLED
  setCollider (dTriMeshClass,dSphereClass,&dCollideSTL);
  setCollider (dTriMeshClass,dBoxClass,&dCollideBTL);
  setCollider (dTriMeshClass,dRayClass,&dCollideRTL);
  setCollider (dTriMeshClass,dTriMeshClass,&dCollideTTL);
  setCollider (dTriMeshClass,dCCylinderClass,&dCollideCCTL);
#endif



  setAllColliders (dGeomTransformClass,&dCollideTransform);
}


int dCollide (dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact,
	      int skip)
{
  dAASSERT(o1 && o2 && contact);
  dUASSERT(colliders_initialized,"colliders array not initialized");
  dUASSERT(o1->type >= 0 && o1->type < dGeomNumClasses,"bad o1 class number");
  dUASSERT(o2->type >= 0 && o2->type < dGeomNumClasses,"bad o2 class number");

  // no contacts if both geoms are the same
  if (o1 == o2) return 0;

  // no contacts if both geoms on the same body, and the body is not 0
  if (o1->body == o2->body && o1->body) return 0;

  dColliderEntry *ce = &colliders[o1->type][o2->type];
  int count = 0;
  if (ce->fn) {
    if (ce->reverse) {
      count = (*ce->fn) (o2,o1,flags,contact,skip);
      for (int i=0; i<count; i++) {
	dContactGeom *c = CONTACT(contact,skip*i);
	c->normal[0] = -c->normal[0];
	c->normal[1] = -c->normal[1];
	c->normal[2] = -c->normal[2];
	dxGeom *tmp = c->g1;
	c->g1 = c->g2;
	c->g2 = tmp;
      }
    }
    else {
      count = (*ce->fn) (o1,o2,flags,contact,skip);
    }
  }
  return count;
}

//****************************************************************************
// dxGeom

dxGeom::dxGeom (dSpaceID _space, int is_placeable)
{
  initColliders();

  // setup body vars. invalid type of -1 must be changed by the constructor.
  type = -1;
  gflags = GEOM_DIRTY | GEOM_AABB_BAD | GEOM_ENABLED;
  if (is_placeable) gflags |= GEOM_PLACEABLE;
  data = 0;
  body = 0;
  body_next = 0;
  if (is_placeable) {
    dxPosR *pr = (dxPosR*) dAlloc (sizeof(dxPosR));
    pos = pr->pos;
    R = pr->R;
    dSetZero (pos,4);
    dRSetIdentity (R);
  }
  else {
    pos = 0;
    R = 0;
  }

  // setup space vars
  next = 0;
  tome = 0;
  parent_space = 0;
  dSetZero (aabb,6);
  category_bits = ~0;
  collide_bits = ~0;

  // put this geom in a space if required
  if (_space) dSpaceAdd (_space,this);
}


dxGeom::~dxGeom()
{
  if (parent_space) dSpaceRemove (parent_space,this);
  if ((gflags & GEOM_PLACEABLE) && !body) dFree (pos,sizeof(dxPosR));
  bodyRemove();
}


int dxGeom::AABBTest (dxGeom *o, dReal aabb[6])
{
  return 1;
}


void dxGeom::bodyRemove()
{
  if (body) {
    // delete this geom from body list
    dxGeom **last = &body->geom, *g = body->geom;
    while (g) {
      if (g == this) {
	*last = g->body_next;
	break;
      }
      last = &g->body_next;
      g = g->body_next;
    }
    body = 0;
    body_next = 0;
  }
}

//****************************************************************************
// misc

dxGeom *dGeomGetBodyNext (dxGeom *geom)
{
  return geom->body_next;
}

//****************************************************************************
// public API for geometry objects

#define CHECK_NOT_LOCKED(space) \
  dUASSERT (!(space && space->lock_count), \
	    "invalid operation for geom in locked space");


void dGeomDestroy (dxGeom *g)
{
  dAASSERT (g);
  delete g;
}


void dGeomSetData (dxGeom *g, void *data)
{
  dAASSERT (g);
  g->data = data;
}


void *dGeomGetData (dxGeom *g)
{
  dAASSERT (g);
  return g->data;
}


void dGeomSetBody (dxGeom *g, dxBody *b)
{
  dAASSERT (g);
  dUASSERT (g->gflags & GEOM_PLACEABLE,"geom must be placeable");
  CHECK_NOT_LOCKED (g->parent_space);

  if (b) {
    if (!g->body) dFree (g->pos,sizeof(dxPosR));
    g->pos = b->pos;
    g->R = b->R;
    dGeomMoved (g);
    if (g->body != b) {
      g->bodyRemove();
      g->bodyAdd (b);
    }
  }
  else {
    if (g->body) {
      dxPosR *pr = (dxPosR*) dAlloc (sizeof(dxPosR));
      g->pos = pr->pos;
      g->R = pr->R;
      memcpy (g->pos,g->body->pos,sizeof(dVector3));
      memcpy (g->R,g->body->R,sizeof(dMatrix3));
      g->bodyRemove();
    }
    // dGeomMoved() should not be called if the body is being set to 0, as the
    // new position of the geom is set to the old position of the body, so the
    // effective position of the geom remains unchanged.
  }
}


dBodyID dGeomGetBody (dxGeom *g)
{
  dAASSERT (g);
  return g->body;
}


void dGeomSetPosition (dxGeom *g, dReal x, dReal y, dReal z)
{
  dAASSERT (g);
  dUASSERT (g->gflags & GEOM_PLACEABLE,"geom must be placeable");
  CHECK_NOT_LOCKED (g->parent_space);
  if (g->body) {
    // this will call dGeomMoved (g), so we don't have to
    dBodySetPosition (g->body,x,y,z);
  }
  else {
    g->pos[0] = x;
    g->pos[1] = y;
    g->pos[2] = z;
    dGeomMoved (g);
  }
}


void dGeomSetRotation (dxGeom *g, const dMatrix3 R)
{
  dAASSERT (g && R);
  dUASSERT (g->gflags & GEOM_PLACEABLE,"geom must be placeable");
  CHECK_NOT_LOCKED (g->parent_space);
  if (g->body) {
    // this will call dGeomMoved (g), so we don't have to
    dBodySetRotation (g->body,R);
  }
  else {
    memcpy (g->R,R,sizeof(dMatrix3));
    dGeomMoved (g);
  }
}


void dGeomSetQuaternion (dxGeom *g, const dQuaternion quat)
{
  dAASSERT (g && quat);
  dUASSERT (g->gflags & GEOM_PLACEABLE,"geom must be placeable");
  CHECK_NOT_LOCKED (g->parent_space);
  if (g->body) {
    // this will call dGeomMoved (g), so we don't have to
    dBodySetQuaternion (g->body,quat);
  }
  else {
    dQtoR (quat, g->R);
    dGeomMoved (g);
  }
}


const dReal * dGeomGetPosition (dxGeom *g)
{
  dAASSERT (g);
  dUASSERT (g->gflags & GEOM_PLACEABLE,"geom must be placeable");
  return g->pos;
}


const dReal * dGeomGetRotation (dxGeom *g)
{
  dAASSERT (g);
  dUASSERT (g->gflags & GEOM_PLACEABLE,"geom must be placeable");
  return g->R;
}


void dGeomGetQuaternion (dxGeom *g, dQuaternion quat)
{
  dAASSERT (g);
  dUASSERT (g->gflags & GEOM_PLACEABLE,"geom must be placeable");
  if (g->body) {
    const dReal * body_quat = dBodyGetQuaternion (g->body);
    quat[0] = body_quat[0];
    quat[1] = body_quat[1];
    quat[2] = body_quat[2];
    quat[3] = body_quat[3];
  }
  else {
    dRtoQ (g->R, quat);
  }
}


void dGeomGetAABB (dxGeom *g, dReal aabb[6])
{
  dAASSERT (g);
  dAASSERT (aabb);
  g->recomputeAABB();
  memcpy (aabb,g->aabb,6 * sizeof(dReal));
}


int dGeomIsSpace (dxGeom *g)
{
  dAASSERT (g);
  return IS_SPACE(g);
}


dSpaceID dGeomGetSpace (dxGeom *g)
{
  dAASSERT (g);
  return g->parent_space;
}


int dGeomGetClass (dxGeom *g)
{
  dAASSERT (g);
  return g->type;
}


void dGeomSetCategoryBits (dxGeom *g, unsigned long bits)
{
  dAASSERT (g);
  CHECK_NOT_LOCKED (g->parent_space);
  g->category_bits = bits;
}


void dGeomSetCollideBits (dxGeom *g, unsigned long bits)
{
  dAASSERT (g);
  CHECK_NOT_LOCKED (g->parent_space);
  g->collide_bits = bits;
}


unsigned long dGeomGetCategoryBits (dxGeom *g)
{
  dAASSERT (g);
  return g->category_bits;
}


unsigned long dGeomGetCollideBits (dxGeom *g)
{
  dAASSERT (g);
  return g->collide_bits;
}


void dGeomEnable (dxGeom *g)
{
	dAASSERT (g);
	g->gflags |= GEOM_ENABLED;
}

void dGeomDisable (dxGeom *g)
{
	dAASSERT (g);
	g->gflags &= ~GEOM_ENABLED;
}

int dGeomIsEnabled (dxGeom *g)
{
	dAASSERT (g);
	return (g->gflags & GEOM_ENABLED) != 0;
}


//****************************************************************************
// C interface that lets the user make new classes. this interface is a lot
// more cumbersome than C++ subclassing, which is what is used internally
// in ODE. this API is mainly to support legacy code.

static int num_user_classes = 0;
static dGeomClass user_classes [dMaxUserClasses];


struct dxUserGeom : public dxGeom {
  void *user_data;

  dxUserGeom (int class_num);
  ~dxUserGeom();
  void computeAABB();
  int AABBTest (dxGeom *o, dReal aabb[6]);
};


dxUserGeom::dxUserGeom (int class_num) : dxGeom (0,1)
{
  type = class_num;
  int size = user_classes[type-dFirstUserClass].bytes;
  user_data = dAlloc (size);
  memset (user_data,0,size);
}


dxUserGeom::~dxUserGeom()
{
  dGeomClass *c = &user_classes[type-dFirstUserClass];
  if (c->dtor) c->dtor (this);
  dFree (user_data,c->bytes);
}


void dxUserGeom::computeAABB()
{
  user_classes[type-dFirstUserClass].aabb (this,aabb);
}


int dxUserGeom::AABBTest (dxGeom *o, dReal aabb[6])
{
  dGeomClass *c = &user_classes[type-dFirstUserClass];
  if (c->aabb_test) return c->aabb_test (this,o,aabb);
  else return 1;
}


static int dCollideUserGeomWithGeom (dxGeom *o1, dxGeom *o2, int flags,
				     dContactGeom *contact, int skip)
{
  // this generic collider function is called the first time that a user class
  // tries to collide against something. it will find out the correct collider
  // function and then set the colliders array so that the correct function is
  // called directly the next time around.

  int t1 = o1->type;	// note that o1 is a user geom
  int t2 = o2->type;	// o2 *may* be a user geom

  // find the collider function to use. if o1 does not know how to collide with
  // o2, then o2 might know how to collide with o1 (provided that it is a user
  // geom).
  dColliderFn *fn = user_classes[t1-dFirstUserClass].collider (t2);
  int reverse = 0;
  if (!fn && t2 >= dFirstUserClass && t2 <= dLastUserClass) {
    fn = user_classes[t2-dFirstUserClass].collider (t1);
    reverse = 1;
  }

  // set the colliders array so that the correct function is called directly
  // the next time around. note that fn can be 0 here if no collider was found,
  // which means that dCollide() will always return 0 for this case.
  colliders[t1][t2].fn = fn;
  colliders[t1][t2].reverse = reverse;
  colliders[t2][t1].fn = fn;
  colliders[t2][t1].reverse = !reverse;

  // now call the collider function indirectly through dCollide(), so that
  // contact reversing is properly handled.
  return dCollide (o1,o2,flags,contact,skip);
}


int dCreateGeomClass (const dGeomClass *c)
{
  dUASSERT(c && c->bytes >= 0 && c->collider && c->aabb,"bad geom class");

  if (num_user_classes >= dMaxUserClasses) {
    dDebug (0,"too many user classes, you must increase the limit and "
	      "recompile ODE");
  }
  user_classes[num_user_classes] = *c;
  int class_number = num_user_classes + dFirstUserClass;
  initColliders();
  setAllColliders (class_number,&dCollideUserGeomWithGeom);

  num_user_classes++;
  return class_number;
}


void * dGeomGetClassData (dxGeom *g)
{
  dUASSERT (g && g->type >= dFirstUserClass &&
	    g->type <= dLastUserClass,"not a custom class");
  dxUserGeom *user = (dxUserGeom*) g;
  return user->user_data;
}


dGeomID dCreateGeom (int classnum)
{
  dUASSERT (classnum >= dFirstUserClass &&
	    classnum <= dLastUserClass,"not a custom class");
  return new dxUserGeom (classnum);
}

//****************************************************************************
// here is where we deallocate any memory that has been globally
// allocated, or free other global resources.

void dCloseODE()
{
  colliders_initialized = 0;
  num_user_classes = 0;
}
