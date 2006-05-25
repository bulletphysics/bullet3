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

stuff common to all spaces

*/

#ifndef _ODE_COLLISION_SPACE_INTERNAL_H_
#define _ODE_COLLISION_SPACE_INTERNAL_H_

#define ALLOCA(x) dALLOCA16(x)

#define CHECK_NOT_LOCKED(space) \
  dUASSERT ((space)==0 || (space)->lock_count==0, \
	    "invalid operation for locked space");


// collide two geoms together. for the hash table space, this is
// called if the two AABBs inhabit the same hash table cells.
// this only calls the callback function if the AABBs actually
// intersect. if a geom has an AABB test function, that is called to
// provide a further refinement of the intersection.
//
// NOTE: this assumes that the geom AABBs are valid on entry
// and that both geoms are enabled.

static void collideAABBs (dxGeom *g1, dxGeom *g2,
			  void *data, dNearCallback *callback)
{
  dIASSERT((g1->gflags & GEOM_AABB_BAD)==0);
  dIASSERT((g2->gflags & GEOM_AABB_BAD)==0);

  // no contacts if both geoms on the same body, and the body is not 0
  if (g1->body == g2->body && g1->body) return;

  // test if the category and collide bitfields match
  if ( ((g1->category_bits & g2->collide_bits) ||
	(g2->category_bits & g1->collide_bits)) == 0) {
    return;
  }

  // if the bounding boxes are disjoint then don't do anything
  dReal *bounds1 = g1->aabb;
  dReal *bounds2 = g2->aabb;
  if (bounds1[0] > bounds2[1] ||
      bounds1[1] < bounds2[0] ||
      bounds1[2] > bounds2[3] ||
      bounds1[3] < bounds2[2] ||
      bounds1[4] > bounds2[5] ||
      bounds1[5] < bounds2[4]) {
    return;
  }

  // check if either object is able to prove that it doesn't intersect the
  // AABB of the other
  if (g1->AABBTest (g2,bounds2) == 0) return;
  if (g2->AABBTest (g1,bounds1) == 0) return;

  // the objects might actually intersect - call the space callback function
  callback (data,g1,g2);
}

#endif
