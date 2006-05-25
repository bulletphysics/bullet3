/*************************************************************************
 *									 *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.	 *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org 	 *
 *									 *
 * This library is free software; you can redistribute it and/or	 *
 * modify it under the terms of EITHER: 				 *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *	 Software Foundation; either version 2.1 of the License, or (at  *
 *	 your option) any later version. The text of the GNU Lesser	 *
 *	 General Public License is included with this library in the	 *
 *	 file LICENSE.TXT.						 *
 *   (2) The BSD-style license that is included with this library in	 *
 *	 the file LICENSE-BSD.TXT.					 *
 *									 *
 * This library is distributed in the hope that it will be useful,	 *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of	 *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files	 *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.			 *
 *									 *
 *************************************************************************/

/* C++ interface for new collision API */


#ifndef _ODE_ODECPP_COLLISION_H_
#define _ODE_ODECPP_COLLISION_H_
#ifdef __cplusplus

#include <ode/error.h>


class dGeom {
  // intentionally undefined, don't use these
  dGeom (dGeom &);
  void operator= (dGeom &);

protected:
  dGeomID _id;

public:
  dGeom()
    { _id = 0; }
  ~dGeom()
    { if (_id) dGeomDestroy (_id); }

  dGeomID id() const
    { return _id; }
  operator dGeomID() const
    { return _id; }

  void destroy() {
    if (_id) dGeomDestroy (_id);
    _id = 0;
  }

  int getClass() const
    { return dGeomGetClass (_id); }

  dSpaceID getSpace() const
    { return dGeomGetSpace (_id); }

  void setData (void *data)
    { dGeomSetData (_id,data); }
  void *getData() const
    { return dGeomGetData (_id); }

  void setBody (dBodyID b)
    { dGeomSetBody (_id,b); }
  dBodyID getBody() const
    { return dGeomGetBody (_id); }

  void setPosition (dReal x, dReal y, dReal z)
    { dGeomSetPosition (_id,x,y,z); }
  const dReal * getPosition() const
    { return dGeomGetPosition (_id); }

  void setRotation (const dMatrix3 R)
    { dGeomSetRotation (_id,R); }
  const dReal * getRotation() const
    { return dGeomGetRotation (_id); }
    
  void setQuaternion (const dQuaternion quat)
    { dGeomSetQuaternion (_id,quat); }
  void getQuaternion (dQuaternion quat) const
    { dGeomGetQuaternion (_id,quat); }

  void getAABB (dReal aabb[6]) const
    { dGeomGetAABB (_id, aabb); }

  int isSpace()
    { return dGeomIsSpace (_id); }

  void setCategoryBits (unsigned long bits)
    { dGeomSetCategoryBits (_id, bits); }
  void setCollideBits (unsigned long bits)
    { dGeomSetCollideBits (_id, bits); }
  unsigned long getCategoryBits()
    { return dGeomGetCategoryBits (_id); }
  unsigned long getCollideBits()
    { return dGeomGetCollideBits (_id); }

  void enable()
    { dGeomEnable (_id); }
  void disable()
    { dGeomDisable (_id); }
  int isEnabled()
    { return dGeomIsEnabled (_id); }

  void collide2 (dGeomID g, void *data, dNearCallback *callback)
    { dSpaceCollide2 (_id,g,data,callback); }
};


class dSpace : public dGeom {
  // intentionally undefined, don't use these
  dSpace (dSpace &);
  void operator= (dSpace &);

protected:
  // the default constructor is protected so that you
  // can't instance this class. you must instance one
  // of its subclasses instead.
  dSpace () { _id = 0; }

public:
  dSpaceID id() const
    { return (dSpaceID) _id; }
  operator dSpaceID() const
    { return (dSpaceID) _id; }

  void setCleanup (int mode)
    { dSpaceSetCleanup (id(), mode); }
  int getCleanup()
    { return dSpaceGetCleanup (id()); }

  void add (dGeomID x)
    { dSpaceAdd (id(), x); }
  void remove (dGeomID x)
    { dSpaceRemove (id(), x); }
  int query (dGeomID x)
    { return dSpaceQuery (id(),x); }

  int getNumGeoms()
    { return dSpaceGetNumGeoms (id()); }
  dGeomID getGeom (int i)
    { return dSpaceGetGeom (id(),i); }

  void collide (void *data, dNearCallback *callback)
    { dSpaceCollide (id(),data,callback); }
};


class dSimpleSpace : public dSpace {
  // intentionally undefined, don't use these
  dSimpleSpace (dSimpleSpace &);
  void operator= (dSimpleSpace &);

public:
  dSimpleSpace (dSpaceID space)
    { _id = (dGeomID) dSimpleSpaceCreate (space); }
};


class dHashSpace : public dSpace {
  // intentionally undefined, don't use these
  dHashSpace (dHashSpace &);
  void operator= (dHashSpace &);

public:
  dHashSpace (dSpaceID space)
    { _id = (dGeomID) dHashSpaceCreate (space); }
  void setLevels (int minlevel, int maxlevel)
    { dHashSpaceSetLevels (id(),minlevel,maxlevel); }
};


class dQuadTreeSpace : public dSpace {
  // intentionally undefined, don't use these
  dQuadTreeSpace (dQuadTreeSpace &);
  void operator= (dQuadTreeSpace &);

public:
  dQuadTreeSpace (dSpaceID space, dVector3 center, dVector3 extents, int depth)
    { _id = (dGeomID) dQuadTreeSpaceCreate (space,center,extents,depth); }
};


class dSphere : public dGeom {
  // intentionally undefined, don't use these
  dSphere (dSphere &);
  void operator= (dSphere &);

public:
  dSphere () { }
  dSphere (dSpaceID space, dReal radius)
    { _id = dCreateSphere (space, radius); }

  void create (dSpaceID space, dReal radius) {
    if (_id) dGeomDestroy (_id);
    _id = dCreateSphere (space, radius);
  }

  void setRadius (dReal radius)
    { dGeomSphereSetRadius (_id, radius); }
  dReal getRadius() const
    { return dGeomSphereGetRadius (_id); }
};


class dBox : public dGeom {
  // intentionally undefined, don't use these
  dBox (dBox &);
  void operator= (dBox &);

public:
  dBox () { }
  dBox (dSpaceID space, dReal lx, dReal ly, dReal lz)
    { _id = dCreateBox (space,lx,ly,lz); }

  void create (dSpaceID space, dReal lx, dReal ly, dReal lz) {
    if (_id) dGeomDestroy (_id);
    _id = dCreateBox (space,lx,ly,lz);
  }

  void setLengths (dReal lx, dReal ly, dReal lz)
    { dGeomBoxSetLengths (_id, lx, ly, lz); }
  void getLengths (dVector3 result) const
    { dGeomBoxGetLengths (_id,result); }
};


class dPlane : public dGeom {
  // intentionally undefined, don't use these
  dPlane (dPlane &);
  void operator= (dPlane &);

public:
  dPlane() { }
  dPlane (dSpaceID space, dReal a, dReal b, dReal c, dReal d)
    { _id = dCreatePlane (space,a,b,c,d); }

  void create (dSpaceID space, dReal a, dReal b, dReal c, dReal d) {
    if (_id) dGeomDestroy (_id);
    _id = dCreatePlane (space,a,b,c,d);
  }

  void setParams (dReal a, dReal b, dReal c, dReal d)
    { dGeomPlaneSetParams (_id, a, b, c, d); }
  void getParams (dVector4 result) const
    { dGeomPlaneGetParams (_id,result); }
};


class dCCylinder : public dGeom {
  // intentionally undefined, don't use these
  dCCylinder (dCCylinder &);
  void operator= (dCCylinder &);

public:
  dCCylinder() { }
  dCCylinder (dSpaceID space, dReal radius, dReal length)
    { _id = dCreateCCylinder (space,radius,length); }

  void create (dSpaceID space, dReal radius, dReal length) {
    if (_id) dGeomDestroy (_id);
    _id = dCreateCCylinder (space,radius,length);
  }

  void setParams (dReal radius, dReal length)
    { dGeomCCylinderSetParams (_id, radius, length); }
  void getParams (dReal *radius, dReal *length) const
    { dGeomCCylinderGetParams (_id,radius,length); }
};


class dRay : public dGeom {
  // intentionally undefined, don't use these
  dRay (dRay &);
  void operator= (dRay &);

public:
  dRay() { }
  dRay (dSpaceID space, dReal length)
    { _id = dCreateRay (space,length); }

  void create (dSpaceID space, dReal length) {
    if (_id) dGeomDestroy (_id);
    _id = dCreateRay (space,length);
  }

  void setLength (dReal length)
    { dGeomRaySetLength (_id, length); }
  dReal getLength()
    { return dGeomRayGetLength (_id); }

  void set (dReal px, dReal py, dReal pz, dReal dx, dReal dy, dReal dz)
    { dGeomRaySet (_id, px, py, pz, dx, dy, dz); }
  void get (dVector3 start, dVector3 dir)
    { dGeomRayGet (_id, start, dir); }

  void setParams (int firstContact, int backfaceCull)
    { dGeomRaySetParams (_id, firstContact, backfaceCull); }
  void getParams (int *firstContact, int *backfaceCull)
    { dGeomRayGetParams (_id, firstContact, backfaceCull); }
  void setClosestHit (int closestHit)
    { dGeomRaySetClosestHit (_id, closestHit); }
  int getClosestHit()
    { return dGeomRayGetClosestHit (_id); }
};


class dGeomTransform : public dGeom {
  // intentionally undefined, don't use these
  dGeomTransform (dGeomTransform &);
  void operator= (dGeomTransform &);

public:
  dGeomTransform() { }
  dGeomTransform (dSpaceID space)
    { _id = dCreateGeomTransform (space); }

  void create (dSpaceID space=0) {
    if (_id) dGeomDestroy (_id);
    _id = dCreateGeomTransform (space);
  }

  void setGeom (dGeomID geom)
    { dGeomTransformSetGeom (_id, geom); }
  dGeomID getGeom() const
    { return dGeomTransformGetGeom (_id); }

  void setCleanup (int mode)
    { dGeomTransformSetCleanup (_id,mode); }
  int getCleanup ()
    { return dGeomTransformGetCleanup (_id); }

  void setInfo (int mode)
    { dGeomTransformSetInfo (_id,mode); }
  int getInfo()
    { return dGeomTransformGetInfo (_id); }
};


#endif
#endif
