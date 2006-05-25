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

// Convex collision detection (Bullet GJK) by Erwin Coumans, http://www.continuousphysics.com/Bullet/
//
// Convex Integration is work in progress, so most users can ignore this file!
// Report suggestions/bugs in Bullet forum:
// http://www.continuousphysics.com/Bullet/phpBB2/
//

#ifdef BULLET_CONVEX_SUPPORT

#include "collision_convex_internal.h"
#include <CollisionShapes/CollisionShape.h>
#include "BulletOdeTransformConvert.h"
#include "BulletOdeCollide.h"

//****************************************************************************
// convex object public API

int dCollideConvexConvex(dxGeom *o1, dxGeom *o2, int flags,
			  dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  
  contact->g1 = o1;
  contact->g2 = o2;

  return BulletOdeCollide(o1,o2,contact,flags & NUMC_MASK,skip);
}

dxConvex::dxConvex (dSpaceID space, class CollisionShape* shape) : dxGeom (space,1)
{
  type = dConvexClass;
  m_bulletCollisionShape = shape;
}


void	GetAabbFromConvex(dGeomID geom,dVector3 halfExtents,dVector3 aabbCenter)
{

	CollisionShape* shape = GetCollisionShapeFromConvex(geom);
	SimdVector3 aabbMin,aabbMax;

	SimdTransform tr;
	tr.setIdentity();

	shape->GetAabb(tr,aabbMin,aabbMax);
	SimdVector3 aabbHalfExtents = (aabbMax-aabbMin)*0.5f;
	SimdVector3 center = (aabbMax+aabbMin)*0.5f;

	halfExtents[0] = aabbHalfExtents[0];
	halfExtents[1] = aabbHalfExtents[1];
	halfExtents[2] = aabbHalfExtents[2];

	aabbCenter[0] = center [0];
	aabbCenter[1] = center [1];
	aabbCenter[2] = center [2];

}


void dxConvex::computeAABB()
{
  dReal xrange = 1e30;
  dReal yrange = 1e30f;
  dReal zrange = 1e30f;
	CollisionShape* shape = GetCollisionShapeFromConvex(this);

  SimdVector3 aabbMin,aabbMax;
  SimdTransform tr = GetTransformFromGeom(this);
  shape->GetAabb(tr,aabbMin,aabbMax);

  aabb[0] = aabbMin[0];
  aabb[1] = aabbMax[0];
  aabb[2] = aabbMin[1];
  aabb[3] = aabbMax[1];
  aabb[4] = aabbMin[2];
  aabb[5] = aabbMax[2];
}

void	dGeomConvexGetLengths(dGeomID convex, dVector3 result)
{
  dUASSERT (convex && convex->type == dConvexClass,"argument not a convex");
  dxConvex* cnvx = (dxConvex*) convex;
  
  dVector3 center;
  GetAabbFromConvex(convex,result,center);
  result[0]*=2.f;
  result[1]*=2.f;
  result[2]*=2.f;

}



dGeomID dCreateConvex(dSpaceID space, class CollisionShape* shape)
{
  return new dxConvex(space,shape);
}


#endif //BULLET_CONVEX_SUPPORT
