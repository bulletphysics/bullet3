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

some useful collision utility stuff.

*/

#ifndef _ODE_COLLISION_UTIL_H_
#define _ODE_COLLISION_UTIL_H_

#include <ode/common.h>
#include <ode/contact.h>


// given a pointer `p' to a dContactGeom, return the dContactGeom at
// p + skip bytes.
#define CONTACT(p,skip) ((dContactGeom*) (((char*)p) + (skip)))


// if the spheres (p1,r1) and (p2,r2) collide, set the contact `c' and
// return 1, else return 0.

int dCollideSpheres (dVector3 p1, dReal r1,
		     dVector3 p2, dReal r2, dContactGeom *c);


// given two lines
//    qa = pa + alpha* ua
//    qb = pb + beta * ub
// where pa,pb are two points, ua,ub are two unit length vectors, and alpha,
// beta go from [-inf,inf], return alpha and beta such that qa and qb are
// as close as possible

void dLineClosestApproach (const dVector3 pa, const dVector3 ua,
			   const dVector3 pb, const dVector3 ub,
			   dReal *alpha, dReal *beta);


// given a line segment p1-p2 and a box (center 'c', rotation 'R', side length
// vector 'side'), compute the points of closest approach between the box
// and the line. return these points in 'lret' (the point on the line) and
// 'bret' (the point on the box). if the line actually penetrates the box
// then the solution is not unique, but only one solution will be returned.
// in this case the solution points will coincide.

void dClosestLineBoxPoints (const dVector3 p1, const dVector3 p2,
			    const dVector3 c, const dMatrix3 R,
			    const dVector3 side,
			    dVector3 lret, dVector3 bret);

#endif
