
/*

this is code that was once useful but has now been obseleted.

this file should not be compiled as part of ODE!

*/

//***************************************************************************
// intersect a line segment with a plane

extern "C" int dClipLineToBox (const dVector3 p1, const dVector3 p2,
			       const dVector3 p, const dMatrix3 R,
			       const dVector3 side)
{
  // compute the start and end of the line (p1 and p2) relative to the box.
  // we will do all subsequent computations in this box-relative coordinate
  // system. we have to do a translation and rotation for each point.
  dVector3 tmp,s,e;
  tmp[0] = p1[0] - p[0];
  tmp[1] = p1[1] - p[1];
  tmp[2] = p1[2] - p[2];
  dMULTIPLY1_331 (s,R,tmp);
  tmp[0] = p2[0] - p[0];
  tmp[1] = p2[1] - p[1];
  tmp[2] = p2[2] - p[2];
  dMULTIPLY1_331 (e,R,tmp);

  // compute the vector 'v' from the start point to the end point
  dVector3 v;
  v[0] = e[0] - s[0];
  v[1] = e[1] - s[1];
  v[2] = e[2] - s[2];

  // a point on the line is defined by the parameter 't'. t=0 corresponds
  // to the start of the line, t=1 corresponds to the end of the line.
  // we will clip the line to the box by finding the range of t where a
  // point on the line is inside the box. the currently known bounds for
  // t and tlo..thi.
  dReal tlo=0,thi=1;

  // clip in the X/Y/Z direction
  for (int i=0; i<3; i++) {
    // first adjust s,e for the current t range. this is redundant for the
    // first iteration, but never mind.
    e[i] = s[i] + thi*v[i];
    s[i] = s[i] + tlo*v[i];
    // compute where t intersects the positive and negative sides.
    dReal tp = ( side[i] - s[i])/v[i];	// @@@ handle case where denom=0
    dReal tm = (-side[i] - s[i])/v[i];
    // handle 9 intersection cases
    if (s[i] <= -side[i]) {
      tlo = tm;
      if (e[i] <= -side[i]) return 0;
      else if (e[i] >= side[i]) thi = tp;
    }
    else if (s[i] <= side[i]) {
      if (e[i] <= -side[i]) thi = tm;
      else if (e[i] >= side[i]) thi = tp;
    }
    else {
      tlo = tp;
      if (e[i] <= -side[i]) thi = tm;
      else if (e[i] >= side[i]) return 0;
    }
  }

  //... @@@ AT HERE @@@

  return 1;
}


//***************************************************************************
// a nice try at C-B collision. unfortunately it doesn't work. the logic
// for testing for line-box intersection is correct, but unfortunately the
// closest-point distance estimates are often too large. as a result contact
// points are placed incorrectly.


int dCollideCB (const dxGeom *o1, const dxGeom *o2, int flags,
		dContactGeom *contact, int skip)
{
  int i;

  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->_class->num == dCCylinderClass);
  dIASSERT (o2->_class->num == dBoxClass);
  contact->g1 = const_cast<dxGeom*> (o1);
  contact->g2 = const_cast<dxGeom*> (o2);
  dxCCylinder *cyl = (dxCCylinder*) CLASSDATA(o1);
  dxBox *box = (dxBox*) CLASSDATA(o2);

  // get p1,p2 = cylinder axis endpoints, get radius
  dVector3 p1,p2;
  dReal clen = cyl->lz * REAL(0.5);
  p1[0] = o1->pos[0] + clen * o1->R[2];
  p1[1] = o1->pos[1] + clen * o1->R[6];
  p1[2] = o1->pos[2] + clen * o1->R[10];
  p2[0] = o1->pos[0] - clen * o1->R[2];
  p2[1] = o1->pos[1] - clen * o1->R[6];
  p2[2] = o1->pos[2] - clen * o1->R[10];
  dReal radius = cyl->radius;

  // copy out box center, rotation matrix, and side array
  dReal *c = o2->pos;
  dReal *R = o2->R;
  dReal *side = box->side;

  // compute the start and end of the line (p1 and p2) relative to the box.
  // we will do all subsequent computations in this box-relative coordinate
  // system. we have to do a translation and rotation for each point.
  dVector3 tmp3,s,e;
  tmp3[0] = p1[0] - c[0];
  tmp3[1] = p1[1] - c[1];
  tmp3[2] = p1[2] - c[2];
  dMULTIPLY1_331 (s,R,tmp3);
  tmp3[0] = p2[0] - c[0];
  tmp3[1] = p2[1] - c[1];
  tmp3[2] = p2[2] - c[2];
  dMULTIPLY1_331 (e,R,tmp3);

  // compute the vector 'v' from the start point to the end point
  dVector3 v;
  v[0] = e[0] - s[0];
  v[1] = e[1] - s[1];
  v[2] = e[2] - s[2];

  // compute the half-sides of the box
  dReal S0 = side[0] * REAL(0.5);
  dReal S1 = side[1] * REAL(0.5);
  dReal S2 = side[2] * REAL(0.5);

  // compute the size of the bounding box around the line segment
  dReal B0 = dFabs (v[0]);
  dReal B1 = dFabs (v[1]);
  dReal B2 = dFabs (v[2]);

  // for all 6 separation axes, measure the penetration depth. if any depth is
  // less than 0 then the objects don't penetrate at all so we can just
  // return 0. find the axis with the smallest depth, and record its normal.

  // note: normalR is set to point to a column of R if that is the smallest
  // depth normal so far. otherwise normalR is 0 and normalC is set to a
  // vector relative to the box. invert_normal is 1 if the sign of the normal
  // should be flipped.

  dReal depth,trial_depth,tmp,length;
  const dReal *normalR=0;
  dVector3 normalC;
  int invert_normal = 0;
  int code = 0;		// 0=no contact, 1-3=face contact, 4-6=edge contact

  depth = dInfinity;

  // look at face-normal axes

#undef TEST
#define TEST(center,depth_expr,norm,contact_code) \
  tmp = (center); \
  trial_depth = radius + REAL(0.5) * ((depth_expr) - dFabs(tmp)); \
  if (trial_depth < 0) return 0; \
  if (trial_depth < depth) { \
    depth = trial_depth; \
    normalR = (norm); \
    invert_normal = (tmp < 0); \
    code = contact_code; \
  }

  TEST (s[0]+e[0], side[0] + B0, R+0, 1);
  TEST (s[1]+e[1], side[1] + B1, R+1, 2);
  TEST (s[2]+e[2], side[2] + B2, R+2, 3);

  // look at v x box-edge axes

#undef TEST
#define TEST(box_radius,line_offset,nx,ny,nz,contact_code) \
  tmp = (line_offset); \
  trial_depth = (box_radius) - dFabs(tmp); \
  length = dSqrt ((nx)*(nx) + (ny)*(ny) + (nz)*(nz)); \
  if (length > 0) { \
    length = dRecip(length); \
    trial_depth = trial_depth * length + radius; \
    if (trial_depth < 0) return 0; \
    if (trial_depth < depth) { \
      depth = trial_depth; \
      normalR = 0; \
      normalC[0] = (nx)*length; \
      normalC[1] = (ny)*length; \
      normalC[2] = (nz)*length; \
      invert_normal = (tmp < 0); \
      code = contact_code; \
    } \
  }

  TEST (B2*S1+B1*S2,v[1]*s[2]-v[2]*s[1], 0,-v[2],v[1], 4);
  TEST (B2*S0+B0*S2,v[2]*s[0]-v[0]*s[2], v[2],0,-v[0], 5);
  TEST (B1*S0+B0*S1,v[0]*s[1]-v[1]*s[0], -v[1],v[0],0, 6);

#undef TEST

  // if we get to this point, the box and ccylinder interpenetrate.
  // compute the normal in global coordinates.
  dReal *normal = contact[0].normal;
  if (normalR) {
    normal[0] = normalR[0];
    normal[1] = normalR[4];
    normal[2] = normalR[8];
  }
  else {
    dMULTIPLY0_331 (normal,R,normalC);
  }
  if (invert_normal) {
    normal[0] = -normal[0];
    normal[1] = -normal[1];
    normal[2] = -normal[2];
  }

  // set the depth
  contact[0].depth = depth;

  if (code == 0) {
    return 0;		// should never get here
  }
  else if (code >= 4) {
    // handle edge contacts
    // find an endpoint q1 on the intersecting edge of the box
    dVector3 q1;
    dReal sign[3];
    for (i=0; i<3; i++) q1[i] = c[i];
    sign[0] = (dDOT14(normal,R+0) > 0) ? REAL(1.0) : REAL(-1.0);
    for (i=0; i<3; i++) q1[i] += sign[0] * S0 * R[i*4];
    sign[1] = (dDOT14(normal,R+1) > 0) ? REAL(1.0) : REAL(-1.0);
    for (i=0; i<3; i++) q1[i] += sign[1] * S1 * R[i*4+1];
    sign[2] = (dDOT14(normal,R+2) > 0) ? REAL(1.0) : REAL(-1.0);
    for (i=0; i<3; i++) q1[i] += sign[2] * S2 * R[i*4+2];

    // find the other endpoint q2 of the intersecting edge
    dVector3 q2;
    for (i=0; i<3; i++)
      q2[i] = q1[i] - R[code-4 + i*4] * (sign[code-4] * side[code-4]);

    // determine the closest point between the box edge and the line segment
    dVector3 cp1,cp2;
    dClosestLineSegmentPoints (q1,q2, p1,p2, cp1,cp2);
    for (i=0; i<3; i++) contact[0].pos[i] = cp1[i] - REAL(0.5)*normal[i]*depth;
    return 1;
  }
  else {
    // handle face contacts.
    // @@@ temporary: make deepest vertex on the line the contact point.
    // @@@ this kind of works, but we sometimes need two contact points for
    // @@@ stability.

    // compute 'v' in global coordinates
    dVector3 gv;
    for (i=0; i<3; i++) gv[i] = p2[i] - p1[i];

    if (dDOT (normal,gv) > 0) {
      for (i=0; i<3; i++)
	contact[0].pos[i] = p1[i] + (depth*REAL(0.5)-radius)*normal[i];
    }
    else {
      for (i=0; i<3; i++)
	contact[0].pos[i] = p2[i] + (depth*REAL(0.5)-radius)*normal[i];
    }
    return 1;
  }
}

//***************************************************************************
// this function works, it's just not being used for anything at the moment:

// given a box (R,side), `R' is the rotation matrix for the box, and `side'
// is a vector of x/y/z side lengths, return the size of the interval of the
// box projected along the given axis. if the axis has unit length then the
// return value will be the actual diameter, otherwise the result will be
// scaled by the axis length.

static inline dReal boxDiameter (const dMatrix3 R, const dVector3 side,
				 const dVector3 axis)
{
  dVector3 q;
  dMULTIPLY1_331 (q,R,axis);	// transform axis to body-relative
  return dFabs(q[0])*side[0] + dFabs(q[1])*side[1] + dFabs(q[2])*side[2];
}

//***************************************************************************
// the old capped cylinder to capped cylinder collision code. this fails to
// detect cap-to-cap contact points when the cylinder axis are aligned, but
// other that that it is pretty robust.

// this returns at most one contact point when the two cylinder's axes are not
// aligned, and at most two (for stability) when they are aligned.
// the algorithm minimizes the distance between two "sample spheres" that are
// positioned along the cylinder axes according to:
//    sphere1 = pos1 + alpha1 * axis1
//    sphere2 = pos2 + alpha2 * axis2
// alpha1 and alpha2 are limited to +/- half the length of the cylinders.
// the algorithm works by finding a solution that has both alphas free, or
// a solution that has one or both alphas fixed to the ends of the cylinder.

int dCollideCCylinderCCylinder (dxGeom *o1, dxGeom *o2,
				int flags, dContactGeom *contact, int skip)
{
  int i;
  const dReal tolerance = REAL(1e-5);

  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dCCylinderClass);
  dIASSERT (o2->type == dCCylinderClass);
  dxCCylinder *cyl1 = (dxCCylinder*) o1;
  dxCCylinder *cyl2 = (dxCCylinder*) o2;

  contact->g1 = o1;
  contact->g2 = o2;

  // copy out some variables, for convenience
  dReal lz1 = cyl1->lz * REAL(0.5);
  dReal lz2 = cyl2->lz * REAL(0.5);
  dReal *pos1 = o1->pos;
  dReal *pos2 = o2->pos;
  dReal axis1[3],axis2[3];
  axis1[0] = o1->R[2];
  axis1[1] = o1->R[6];
  axis1[2] = o1->R[10];
  axis2[0] = o2->R[2];
  axis2[1] = o2->R[6];
  axis2[2] = o2->R[10];

  dReal alpha1,alpha2,sphere1[3],sphere2[3];
  int fix1 = 0;		// 0 if alpha1 is free, +/-1 to fix at +/- lz1
  int fix2 = 0;		// 0 if alpha2 is free, +/-1 to fix at +/- lz2

  for (int count=0; count<9; count++) {
    // find a trial solution by fixing or not fixing the alphas
    if (fix1) {
      if (fix2) {
	// alpha1 and alpha2 are fixed, so the solution is easy
	if (fix1 > 0) alpha1 = lz1; else alpha1 = -lz1;
	if (fix2 > 0) alpha2 = lz2; else alpha2 = -lz2;
	for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
	for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
      }
      else {
	// fix alpha1 but let alpha2 be free
	if (fix1 > 0) alpha1 = lz1; else alpha1 = -lz1;
	for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
	alpha2 = (axis2[0]*(sphere1[0]-pos2[0]) +
		  axis2[1]*(sphere1[1]-pos2[1]) +
		  axis2[2]*(sphere1[2]-pos2[2]));
	for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
      }
    }
    else {
      if (fix2) {
	// fix alpha2 but let alpha1 be free
	if (fix2 > 0) alpha2 = lz2; else alpha2 = -lz2;
	for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
	alpha1 = (axis1[0]*(sphere2[0]-pos1[0]) +
		  axis1[1]*(sphere2[1]-pos1[1]) +
		  axis1[2]*(sphere2[2]-pos1[2]));
	for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
      }
      else {
	// let alpha1 and alpha2 be free
	// compute determinant of d(d^2)\d(alpha) jacobian
	dReal a1a2 = dDOT (axis1,axis2);
	dReal det = REAL(1.0)-a1a2*a1a2;
	if (det < tolerance) {
	  // the cylinder axes (almost) parallel, so we will generate up to two
	  // contacts. the solution matrix is rank deficient so alpha1 and
	  // alpha2 are related by:
	  //       alpha2 =   alpha1 + (pos1-pos2)'*axis1   (if axis1==axis2)
	  //    or alpha2 = -(alpha1 + (pos1-pos2)'*axis1)  (if axis1==-axis2)
	  // first compute where the two cylinders overlap in alpha1 space:
	  if (a1a2 < 0) {
	    axis2[0] = -axis2[0];
	    axis2[1] = -axis2[1];
	    axis2[2] = -axis2[2];
	  }
	  dReal q[3];
	  for (i=0; i<3; i++) q[i] = pos1[i]-pos2[i];
	  dReal k = dDOT (axis1,q);
	  dReal a1lo = -lz1;
	  dReal a1hi = lz1;
	  dReal a2lo = -lz2 - k;
	  dReal a2hi = lz2 - k;
	  dReal lo = (a1lo > a2lo) ? a1lo : a2lo;
	  dReal hi = (a1hi < a2hi) ? a1hi : a2hi;
	  if (lo <= hi) {
	    int num_contacts = flags & NUMC_MASK;
	    if (num_contacts >= 2 && lo < hi) {
	      // generate up to two contacts. if one of those contacts is
	      // not made, fall back on the one-contact strategy.
	      for (i=0; i<3; i++) sphere1[i] = pos1[i] + lo*axis1[i];
	      for (i=0; i<3; i++) sphere2[i] = pos2[i] + (lo+k)*axis2[i];
	      int n1 = dCollideSpheres (sphere1,cyl1->radius,
					sphere2,cyl2->radius,contact);
	      if (n1) {
		for (i=0; i<3; i++) sphere1[i] = pos1[i] + hi*axis1[i];
		for (i=0; i<3; i++) sphere2[i] = pos2[i] + (hi+k)*axis2[i];
		dContactGeom *c2 = CONTACT(contact,skip);
		int n2 = dCollideSpheres (sphere1,cyl1->radius,
					  sphere2,cyl2->radius, c2);
		if (n2) {
		  c2->g1 = o1;
		  c2->g2 = o2;
		  return 2;
		}
	      }
	    }

	    // just one contact to generate, so put it in the middle of
	    // the range
	    alpha1 = (lo + hi) * REAL(0.5);
	    alpha2 = alpha1 + k;
	    for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
	    for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
	    return dCollideSpheres (sphere1,cyl1->radius,
				    sphere2,cyl2->radius,contact);
	  }
	  else return 0;
	}
	det = REAL(1.0)/det;
	dReal delta[3];
	for (i=0; i<3; i++) delta[i] = pos1[i] - pos2[i];
	dReal q1 = dDOT (delta,axis1);
	dReal q2 = dDOT (delta,axis2);
	alpha1 = det*(a1a2*q2-q1);
	alpha2 = det*(q2-a1a2*q1);
	for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
	for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
      }
    }

    // if the alphas are outside their allowed ranges then fix them and
    // try again
    if (fix1==0) {
      if (alpha1 < -lz1) {
	fix1 = -1;
	continue;
      }
      if (alpha1 > lz1) {
	fix1 = 1;
	continue;
      }
    }
    if (fix2==0) {
      if (alpha2 < -lz2) {
	fix2 = -1;
	continue;
      }
      if (alpha2 > lz2) {
	fix2 = 1;
	continue;
      }
    }

    // unfix the alpha variables if the local distance gradient indicates
    // that we are not yet at the minimum
    dReal tmp[3];
    for (i=0; i<3; i++) tmp[i] = sphere1[i] - sphere2[i];
    if (fix1) {
      dReal gradient = dDOT (tmp,axis1);
      if ((fix1 > 0 && gradient > 0) || (fix1 < 0 && gradient < 0)) {
	fix1 = 0;
	continue;
      }
    }
    if (fix2) {
      dReal gradient = -dDOT (tmp,axis2);
      if ((fix2 > 0 && gradient > 0) || (fix2 < 0 && gradient < 0)) {
	fix2 = 0;
	continue;
      }
    }
    return dCollideSpheres (sphere1,cyl1->radius,sphere2,cyl2->radius,contact);
  }
  // if we go through the loop too much, then give up. we should NEVER get to
  // this point (i hope).
  dMessage (0,"dCollideCC(): too many iterations");
  return 0;
}
