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

// Convex (Bullet) GJK code by Erwin Coumans, http://www.continuousphysics.com/Bullet/

#ifndef COLLISION_CONVEX_INTERNAL_H
#define COLLISION_CONVEX_INTERNAL_H


#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_util.h"
#include "collision_kernel.h"

//Bullet class
class CollisionShape;

/// General Convex shape support using Bullet GJK
struct dxConvex : public dxGeom {
  class CollisionShape* m_bulletCollisionShape;
  dxConvex (dSpaceID space, class CollisionShape* shape);
  void computeAABB();
};

void	GetAabbFromConvex(dGeomID geom,dVector3 halfExtents,dVector3 aabbCenter);

#endif //COLLISION_CONVEX_INTERNAL_H
