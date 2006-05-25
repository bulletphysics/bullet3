/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * Fast iterative solver, David Whittaker. Email: david@csworkbench.com  *
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

// This is the StepFast code by David Whittaker. This code is faster, but
// sometimes less stable than, the original "big matrix" code.
// Refer to the user's manual for more information.
// Note that this source file duplicates a lot of stuff from step.cpp,
// eventually we should move the common code to a third file.

#include "objects.h"
#include "joint.h"
#include <ode/config.h>
#include <ode/objects.h>
#include <ode/odemath.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include <ode/matrix.h>
#include <ode/misc.h>
#include "lcp.h"
#include "step.h"
#include "util.h"


// misc defines

#define ALLOCA dALLOCA16

#define RANDOM_JOINT_ORDER
//#define FAST_FACTOR	//use a factorization approximation to the LCP solver (fast, theoretically less accurate)
#define SLOW_LCP      //use the old LCP solver
//#define NO_ISLANDS    //does not perform island creation code (3~4% of simulation time), body disabling doesn't work
//#define TIMING


static int autoEnableDepth = 2;

void dWorldSetAutoEnableDepthSF1 (dxWorld *world, int autodepth)
{
	if (autodepth > 0)
		autoEnableDepth = autodepth;
	else
		autoEnableDepth = 0;
}

int dWorldGetAutoEnableDepthSF1 (dxWorld *world)
{
	return autoEnableDepth;
}

//little bit of math.... the _sym_ functions assume the return matrix will be symmetric
static void
Multiply2_sym_p8p (dReal * A, dReal * B, dReal * C, int p, int Askip)
{
	int i, j;
	dReal sum, *aa, *ad, *bb, *cc;
	dIASSERT (p > 0 && A && B && C);
	bb = B;
	for (i = 0; i < p; i++)
	{
		//aa is going accross the matrix, ad down
		aa = ad = A;
		cc = C;
		for (j = i; j < p; j++)
		{
			sum = bb[0] * cc[0];
			sum += bb[1] * cc[1];
			sum += bb[2] * cc[2];
			sum += bb[4] * cc[4];
			sum += bb[5] * cc[5];
			sum += bb[6] * cc[6];
			*(aa++) = *ad = sum;
			ad += Askip;
			cc += 8;
		}
		bb += 8;
		A += Askip + 1;
		C += 8;
	}
}

static void
MultiplyAdd2_sym_p8p (dReal * A, dReal * B, dReal * C, int p, int Askip)
{
	int i, j;
	dReal sum, *aa, *ad, *bb, *cc;
	dIASSERT (p > 0 && A && B && C);
	bb = B;
	for (i = 0; i < p; i++)
	{
		//aa is going accross the matrix, ad down
		aa = ad = A;
		cc = C;
		for (j = i; j < p; j++)
		{
			sum = bb[0] * cc[0];
			sum += bb[1] * cc[1];
			sum += bb[2] * cc[2];
			sum += bb[4] * cc[4];
			sum += bb[5] * cc[5];
			sum += bb[6] * cc[6];
			*(aa++) += sum;
			*ad += sum;
			ad += Askip;
			cc += 8;
		}
		bb += 8;
		A += Askip + 1;
		C += 8;
	}
}


// this assumes the 4th and 8th rows of B are zero.

static void
Multiply0_p81 (dReal * A, dReal * B, dReal * C, int p)
{
	int i;
	dIASSERT (p > 0 && A && B && C);
	dReal sum;
	for (i = p; i; i--)
	{
		sum = B[0] * C[0];
		sum += B[1] * C[1];
		sum += B[2] * C[2];
		sum += B[4] * C[4];
		sum += B[5] * C[5];
		sum += B[6] * C[6];
		*(A++) = sum;
		B += 8;
	}
}


// this assumes the 4th and 8th rows of B are zero.

static void
MultiplyAdd0_p81 (dReal * A, dReal * B, dReal * C, int p)
{
	int i;
	dIASSERT (p > 0 && A && B && C);
	dReal sum;
	for (i = p; i; i--)
	{
		sum = B[0] * C[0];
		sum += B[1] * C[1];
		sum += B[2] * C[2];
		sum += B[4] * C[4];
		sum += B[5] * C[5];
		sum += B[6] * C[6];
		*(A++) += sum;
		B += 8;
	}
}


// this assumes the 4th and 8th rows of B are zero.

static void
Multiply1_8q1 (dReal * A, dReal * B, dReal * C, int q)
{
	int k;
	dReal sum;
	dIASSERT (q > 0 && A && B && C);
	sum = 0;
	for (k = 0; k < q; k++)
		sum += B[k * 8] * C[k];
	A[0] = sum;
	sum = 0;
	for (k = 0; k < q; k++)
		sum += B[1 + k * 8] * C[k];
	A[1] = sum;
	sum = 0;
	for (k = 0; k < q; k++)
		sum += B[2 + k * 8] * C[k];
	A[2] = sum;
	sum = 0;
	for (k = 0; k < q; k++)
		sum += B[4 + k * 8] * C[k];
	A[4] = sum;
	sum = 0;
	for (k = 0; k < q; k++)
		sum += B[5 + k * 8] * C[k];
	A[5] = sum;
	sum = 0;
	for (k = 0; k < q; k++)
		sum += B[6 + k * 8] * C[k];
	A[6] = sum;
}

//****************************************************************************
// body rotation

// return sin(x)/x. this has a singularity at 0 so special handling is needed
// for small arguments.

static inline dReal
sinc (dReal x)
{
	// if |x| < 1e-4 then use a taylor series expansion. this two term expansion
	// is actually accurate to one LS bit within this range if double precision
	// is being used - so don't worry!
	if (dFabs (x) < 1.0e-4)
		return REAL (1.0) - x * x * REAL (0.166666666666666666667);
	else
		return dSin (x) / x;
}


// given a body b, apply its linear and angular rotation over the time
// interval h, thereby adjusting its position and orientation.

static inline void
moveAndRotateBody (dxBody * b, dReal h)
{
	int j;

	// handle linear velocity
	for (j = 0; j < 3; j++)
		b->pos[j] += h * b->lvel[j];

	if (b->flags & dxBodyFlagFiniteRotation)
	{
		dVector3 irv;			// infitesimal rotation vector
		dQuaternion q;			// quaternion for finite rotation

		if (b->flags & dxBodyFlagFiniteRotationAxis)
		{
			// split the angular velocity vector into a component along the finite
			// rotation axis, and a component orthogonal to it.
			dVector3 frv, irv;	// finite rotation vector
			dReal k = dDOT (b->finite_rot_axis, b->avel);
			frv[0] = b->finite_rot_axis[0] * k;
			frv[1] = b->finite_rot_axis[1] * k;
			frv[2] = b->finite_rot_axis[2] * k;
			irv[0] = b->avel[0] - frv[0];
			irv[1] = b->avel[1] - frv[1];
			irv[2] = b->avel[2] - frv[2];

			// make a rotation quaternion q that corresponds to frv * h.
			// compare this with the full-finite-rotation case below.
			h *= REAL (0.5);
			dReal theta = k * h;
			q[0] = dCos (theta);
			dReal s = sinc (theta) * h;
			q[1] = frv[0] * s;
			q[2] = frv[1] * s;
			q[3] = frv[2] * s;
		}
		else
		{
			// make a rotation quaternion q that corresponds to w * h
			dReal wlen = dSqrt (b->avel[0] * b->avel[0] + b->avel[1] * b->avel[1] + b->avel[2] * b->avel[2]);
			h *= REAL (0.5);
			dReal theta = wlen * h;
			q[0] = dCos (theta);
			dReal s = sinc (theta) * h;
			q[1] = b->avel[0] * s;
			q[2] = b->avel[1] * s;
			q[3] = b->avel[2] * s;
		}

		// do the finite rotation
		dQuaternion q2;
		dQMultiply0 (q2, q, b->q);
		for (j = 0; j < 4; j++)
			b->q[j] = q2[j];

		// do the infitesimal rotation if required
		if (b->flags & dxBodyFlagFiniteRotationAxis)
		{
			dReal dq[4];
			dWtoDQ (irv, b->q, dq);
			for (j = 0; j < 4; j++)
				b->q[j] += h * dq[j];
		}
	}
	else
	{
		// the normal way - do an infitesimal rotation
		dReal dq[4];
		dWtoDQ (b->avel, b->q, dq);
		for (j = 0; j < 4; j++)
			b->q[j] += h * dq[j];
	}

	// normalize the quaternion and convert it to a rotation matrix
	dNormalize4 (b->q);
	dQtoR (b->q, b->R);

	// notify all attached geoms that this body has moved
	for (dxGeom * geom = b->geom; geom; geom = dGeomGetBodyNext (geom))
		dGeomMoved (geom);
}

//****************************************************************************
//This is an implementation of the iterated/relaxation algorithm.
//Here is a quick overview of the algorithm per Sergi Valverde's posts to the
//mailing list:
//
//  for i=0..N-1 do
//      for c = 0..C-1 do
//          Solve constraint c-th
//          Apply forces to constraint bodies
//      next
//  next
//  Integrate bodies

void
dInternalStepFast (dxWorld * world, dxBody * body[2], dReal * GI[2], dReal * GinvI[2], dxJoint * joint, dxJoint::Info1 info, dxJoint::Info2 Jinfo, dReal stepsize)
{
	int i, j, k;
# ifdef TIMING
	dTimerNow ("constraint preprocessing");
# endif

	dReal stepsize1 = dRecip (stepsize);

	int m = info.m;
	// nothing to do if no constraints.
	if (m <= 0)
		return;

	int nub = 0;
	if (info.nub == info.m)
		nub = m;

	// compute A = J*invM*J'. first compute JinvM = J*invM. this has the same
	// format as J so we just go through the constraints in J multiplying by
	// the appropriate scalars and matrices.
#   ifdef TIMING
	dTimerNow ("compute A");
#   endif
	dReal JinvM[2 * 6 * 8];
	//dSetZero (JinvM, 2 * m * 8);

	dReal *Jsrc = Jinfo.J1l;
	dReal *Jdst = JinvM;
	if (body[0])
	{
		for (j = m - 1; j >= 0; j--)
		{
			for (k = 0; k < 3; k++)
				Jdst[k] = Jsrc[k] * body[0]->invMass;
			dMULTIPLY0_133 (Jdst + 4, Jsrc + 4, GinvI[0]);
			Jsrc += 8;
			Jdst += 8;
		}
	}
	if (body[1])
	{
		Jsrc = Jinfo.J2l;
		Jdst = JinvM + 8 * m;
		for (j = m - 1; j >= 0; j--)
		{
			for (k = 0; k < 3; k++)
				Jdst[k] = Jsrc[k] * body[1]->invMass;
			dMULTIPLY0_133 (Jdst + 4, Jsrc + 4, GinvI[1]);
			Jsrc += 8;
			Jdst += 8;
		}
	}


	// now compute A = JinvM * J'.
	int mskip = dPAD (m);
	dReal A[6 * 8];
	//dSetZero (A, 6 * 8);

	if (body[0]) {
		Multiply2_sym_p8p (A, JinvM, Jinfo.J1l, m, mskip);
		if (body[1])
			MultiplyAdd2_sym_p8p (A, JinvM + 8 * m, Jinfo.J2l,
                                              m, mskip);
	} else {
		if (body[1])
			Multiply2_sym_p8p (A, JinvM + 8 * m, Jinfo.J2l,
                                           m, mskip);
	}

	// add cfm to the diagonal of A
	for (i = 0; i < m; i++)
		A[i * mskip + i] += Jinfo.cfm[i] * stepsize1;

	// compute the right hand side `rhs'
#   ifdef TIMING
	dTimerNow ("compute rhs");
#   endif
	dReal tmp1[16];
	//dSetZero (tmp1, 16);
	// put v/h + invM*fe into tmp1
	for (i = 0; i < 2; i++)
	{
		if (!body[i])
			continue;
		for (j = 0; j < 3; j++)
			tmp1[i * 8 + j] = body[i]->facc[j] * body[i]->invMass + body[i]->lvel[j] * stepsize1;
		dMULTIPLY0_331 (tmp1 + i * 8 + 4, GinvI[i], body[i]->tacc);
		for (j = 0; j < 3; j++)
			tmp1[i * 8 + 4 + j] += body[i]->avel[j] * stepsize1;
	}
	// put J*tmp1 into rhs
	dReal rhs[6];
	//dSetZero (rhs, 6);

	if (body[0]) {
		Multiply0_p81 (rhs, Jinfo.J1l, tmp1, m);
		if (body[1])
			MultiplyAdd0_p81 (rhs, Jinfo.J2l, tmp1 + 8, m);
	} else {
		if (body[1])
			Multiply0_p81 (rhs, Jinfo.J2l, tmp1 + 8, m);
	}

	// complete rhs
	for (i = 0; i < m; i++)
		rhs[i] = Jinfo.c[i] * stepsize1 - rhs[i];

#ifdef SLOW_LCP
	// solve the LCP problem and get lambda.
	// this will destroy A but that's okay
#	ifdef TIMING
	dTimerNow ("solving LCP problem");
#	endif
	dReal *lambda = (dReal *) ALLOCA (m * sizeof (dReal));
	dReal *residual = (dReal *) ALLOCA (m * sizeof (dReal));
	dReal lo[6], hi[6];
	memcpy (lo, Jinfo.lo, m * sizeof (dReal));
	memcpy (hi, Jinfo.hi, m * sizeof (dReal));
	dSolveLCP (m, A, lambda, rhs, residual, nub, lo, hi, Jinfo.findex);
#endif

	// LCP Solver replacement:
	// This algorithm goes like this:
	// Do a straightforward LDLT factorization of the matrix A, solving for
	// A*x = rhs
	// For each x[i] that is outside of the bounds of lo[i] and hi[i],
	//    clamp x[i] into that range.
	//    Substitute into A the now known x's
	//    subtract the residual away from the rhs.
	//    Remove row and column i from L, updating the factorization
	//    place the known x's at the end of the array, keeping up with location in p
	// Repeat until all constraints have been clamped or all are within bounds
	//
	// This is probably only faster in the single joint case where only one repeat is
	// the norm.

#ifdef FAST_FACTOR
	// factorize A (L*D*L'=A)
#	ifdef TIMING
	dTimerNow ("factorize A");
#	endif
	dReal d[6];
	dReal L[6 * 8];
	memcpy (L, A, m * mskip * sizeof (dReal));
	dFactorLDLT (L, d, m, mskip);

	// compute lambda
#	ifdef TIMING
	dTimerNow ("compute lambda");
#	endif

	int left = m;				//constraints left to solve.
	int remove[6];
	dReal lambda[6];
	dReal x[6];
	int p[6];
	for (i = 0; i < 6; i++)
		p[i] = i;
	while (true)
	{
		memcpy (x, rhs, left * sizeof (dReal));
		dSolveLDLT (L, d, x, left, mskip);

		int fixed = 0;
		for (i = 0; i < left; i++)
		{
			j = p[i];
			remove[i] = false;
			// This isn't the exact same use of findex as dSolveLCP.... since x[findex]
			// may change after I've already clamped x[i], but it should be close
			if (Jinfo.findex[j] > -1)
			{
				dReal f = fabs (Jinfo.hi[j] * x[p[Jinfo.findex[j]]]);
				if (x[i] > f)
					x[i] = f;
				else if (x[i] < -f)
					x[i] = -f;
				else
					continue;
			}
			else
			{
				if (x[i] > Jinfo.hi[j])
					x[i] = Jinfo.hi[j];
				else if (x[i] < Jinfo.lo[j])
					x[i] = Jinfo.lo[j];
				else
					continue;
			}
			remove[i] = true;
			fixed++;
		}
		if (fixed == 0 || fixed == left)	//no change or all constraints solved
			break;

		for (i = 0; i < left; i++)	//sub in to right hand side.
			if (remove[i])
				for (j = 0; j < left; j++)
					if (!remove[j])
						rhs[j] -= A[j * mskip + i] * x[i];

		for (int r = left - 1; r >= 0; r--)	//eliminate row/col for fixed variables
		{
			if (remove[r])
			{
				//dRemoveLDLT adapted for use without row pointers.
				if (r == left - 1)
				{
					left--;
					continue;	// deleting last row/col is easy
				}
				else if (r == 0)
				{
					dReal a[6];
					for (i = 0; i < left; i++)
						a[i] = -A[i * mskip];
					a[0] += REAL (1.0);
					dLDLTAddTL (L, d, a, left, mskip);
				}
				else
				{
					dReal t[6];
					dReal a[6];
					for (i = 0; i < r; i++)
						t[i] = L[r * mskip + i] / d[i];
					for (i = 0; i < left - r; i++)
						a[i] = dDot (L + (r + i) * mskip, t, r) - A[(r + i) * mskip + r];
					a[0] += REAL (1.0);
					dLDLTAddTL (L + r * mskip + r, d + r, a, left - r, mskip);
				}

				dRemoveRowCol (L, left, mskip, r);
				//end dRemoveLDLT

				left--;
				if (r < (left - 1))
				{
					dReal tx = x[r];
					memmove (d + r, d + r + 1, (left - r) * sizeof (dReal));
					memmove (rhs + r, rhs + r + 1, (left - r) * sizeof (dReal));
					//x will get written over by rhs anyway, no need to move it around
					//just store the fixed value we just discovered in it.
					x[left] = tx;
					for (i = 0; i < m; i++)
						if (p[i] > r && p[i] <= left)
							p[i]--;
					p[r] = left;
				}
			}
		}
	}

	for (i = 0; i < m; i++)
		lambda[i] = x[p[i]];
#	endif
	// compute the constraint force `cforce'
#	ifdef TIMING
	dTimerNow ("compute constraint force");
#endif

	// compute cforce = J'*lambda
	dJointFeedback *fb = joint->feedback;
	dReal cforce[16];
	//dSetZero (cforce, 16);

	if (fb)
	{
		// the user has requested feedback on the amount of force that this
		// joint is applying to the bodies. we use a slightly slower
		// computation that splits out the force components and puts them
		// in the feedback structure.
		dReal data1[8], data2[8];
		if (body[0])
		{
			Multiply1_8q1 (data1, Jinfo.J1l, lambda, m);
			dReal *cf1 = cforce;
			cf1[0] = (fb->f1[0] = data1[0]);
			cf1[1] = (fb->f1[1] = data1[1]);
			cf1[2] = (fb->f1[2] = data1[2]);
			cf1[4] = (fb->t1[0] = data1[4]);
			cf1[5] = (fb->t1[1] = data1[5]);
			cf1[6] = (fb->t1[2] = data1[6]);
		}
		if (body[1])
		{
			Multiply1_8q1 (data2, Jinfo.J2l, lambda, m);
			dReal *cf2 = cforce + 8;
			cf2[0] = (fb->f2[0] = data2[0]);
			cf2[1] = (fb->f2[1] = data2[1]);
			cf2[2] = (fb->f2[2] = data2[2]);
			cf2[4] = (fb->t2[0] = data2[4]);
			cf2[5] = (fb->t2[1] = data2[5]);
			cf2[6] = (fb->t2[2] = data2[6]);
		}
	}
	else
	{
		// no feedback is required, let's compute cforce the faster way
		if (body[0])
			Multiply1_8q1 (cforce, Jinfo.J1l, lambda, m);
		if (body[1])
			Multiply1_8q1 (cforce + 8, Jinfo.J2l, lambda, m);
	}

	for (i = 0; i < 2; i++)
	{
		if (!body[i])
			continue;
		for (j = 0; j < 3; j++)
		{
			body[i]->facc[j] += cforce[i * 8 + j];
			body[i]->tacc[j] += cforce[i * 8 + 4 + j];
		}
	}
}

void
dInternalStepIslandFast (dxWorld * world, dxBody * const *bodies, int nb, dxJoint * const *_joints, int nj, dReal stepsize, int maxiterations)
{
#   ifdef TIMING
	dTimerNow ("preprocessing");
#   endif
	dxBody *bodyPair[2], *body;
	dReal *GIPair[2], *GinvIPair[2];
	dxJoint *joint;
	int iter, b, j, i;
	dReal ministep = stepsize / maxiterations;

	// make a local copy of the joint array, because we might want to modify it.
	// (the "dxJoint *const*" declaration says we're allowed to modify the joints
	// but not the joint array, because the caller might need it unchanged).
	dxJoint **joints = (dxJoint **) ALLOCA (nj * sizeof (dxJoint *));
	memcpy (joints, _joints, nj * sizeof (dxJoint *));

	// get m = total constraint dimension, nub = number of unbounded variables.
	// create constraint offset array and number-of-rows array for all joints.
	// the constraints are re-ordered as follows: the purely unbounded
	// constraints, the mixed unbounded + LCP constraints, and last the purely
	// LCP constraints. this assists the LCP solver to put all unbounded
	// variables at the start for a quick factorization.
	//
	// joints with m=0 are inactive and are removed from the joints array
	// entirely, so that the code that follows does not consider them.
	// also number all active joints in the joint list (set their tag values).
	// inactive joints receive a tag value of -1.

	int m = 0;
	dxJoint::Info1 * info = (dxJoint::Info1 *) ALLOCA (nj * sizeof (dxJoint::Info1));
	int *ofs = (int *) ALLOCA (nj * sizeof (int));
	for (i = 0, j = 0; j < nj; j++)
	{	// i=dest, j=src
		joints[j]->vtable->getInfo1 (joints[j], info + i);
		dIASSERT (info[i].m >= 0 && info[i].m <= 6 && info[i].nub >= 0 && info[i].nub <= info[i].m);
		if (info[i].m > 0)
		{
			joints[i] = joints[j];
			joints[i]->tag = i;
			i++;
		}
		else
		{
			joints[j]->tag = -1;
		}
	}
	nj = i;

	// the purely unbounded constraints
	for (i = 0; i < nj; i++)
	{
		ofs[i] = m;
		m += info[i].m;
	}
	dReal *c = NULL;
	dReal *cfm = NULL;
	dReal *lo = NULL;
	dReal *hi = NULL;
	int *findex = NULL;

	dReal *J = NULL;
	dxJoint::Info2 * Jinfo = NULL;

	if (m)
	{
	// create a constraint equation right hand side vector `c', a constraint
	// force mixing vector `cfm', and LCP low and high bound vectors, and an
	// 'findex' vector.
		c = (dReal *) ALLOCA (m * sizeof (dReal));
		cfm = (dReal *) ALLOCA (m * sizeof (dReal));
		lo = (dReal *) ALLOCA (m * sizeof (dReal));
		hi = (dReal *) ALLOCA (m * sizeof (dReal));
		findex = (int *) ALLOCA (m * sizeof (int));
	dSetZero (c, m);
	dSetValue (cfm, m, world->global_cfm);
	dSetValue (lo, m, -dInfinity);
	dSetValue (hi, m, dInfinity);
	for (i = 0; i < m; i++)
		findex[i] = -1;

	// get jacobian data from constraints. a (2*m)x8 matrix will be created
	// to store the two jacobian blocks from each constraint. it has this
	// format:
	//
	//   l l l 0 a a a 0  \    .
	//   l l l 0 a a a 0   }-- jacobian body 1 block for joint 0 (3 rows)
	//   l l l 0 a a a 0  /
	//   l l l 0 a a a 0  \    .
	//   l l l 0 a a a 0   }-- jacobian body 2 block for joint 0 (3 rows)
	//   l l l 0 a a a 0  /
	//   l l l 0 a a a 0  }--- jacobian body 1 block for joint 1 (1 row)
	//   l l l 0 a a a 0  }--- jacobian body 2 block for joint 1 (1 row)
	//   etc...
	//
	//   (lll) = linear jacobian data
	//   (aaa) = angular jacobian data
	//
#   ifdef TIMING
	dTimerNow ("create J");
#   endif
		J = (dReal *) ALLOCA (2 * m * 8 * sizeof (dReal));
		dSetZero (J, 2 * m * 8);
		Jinfo = (dxJoint::Info2 *) ALLOCA (nj * sizeof (dxJoint::Info2));
	for (i = 0; i < nj; i++)
	{
		Jinfo[i].rowskip = 8;
		Jinfo[i].fps = dRecip (stepsize);
		Jinfo[i].erp = world->global_erp;
		Jinfo[i].J1l = J + 2 * 8 * ofs[i];
		Jinfo[i].J1a = Jinfo[i].J1l + 4;
		Jinfo[i].J2l = Jinfo[i].J1l + 8 * info[i].m;
		Jinfo[i].J2a = Jinfo[i].J2l + 4;
		Jinfo[i].c = c + ofs[i];
		Jinfo[i].cfm = cfm + ofs[i];
		Jinfo[i].lo = lo + ofs[i];
		Jinfo[i].hi = hi + ofs[i];
		Jinfo[i].findex = findex + ofs[i];
		//joints[i]->vtable->getInfo2 (joints[i], Jinfo+i);
	}

	}

	dReal *saveFacc = (dReal *) ALLOCA (nb * 4 * sizeof (dReal));
	dReal *saveTacc = (dReal *) ALLOCA (nb * 4 * sizeof (dReal));
	dReal *globalI = (dReal *) ALLOCA (nb * 12 * sizeof (dReal));
	dReal *globalInvI = (dReal *) ALLOCA (nb * 12 * sizeof (dReal));
	for (b = 0; b < nb; b++)
	{
		for (i = 0; i < 4; i++)
		{
			saveFacc[b * 4 + i] = bodies[b]->facc[i];
			saveTacc[b * 4 + i] = bodies[b]->tacc[i];
		}
                bodies[b]->tag = b;
	}

	for (iter = 0; iter < maxiterations; iter++)
	{
#	ifdef TIMING
		dTimerNow ("applying inertia and gravity");
#	endif
		dReal tmp[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		for (b = 0; b < nb; b++)
		{
			body = bodies[b];

			// for all bodies, compute the inertia tensor and its inverse in the global
			// frame, and compute the rotational force and add it to the torque
			// accumulator. I and invI are vertically stacked 3x4 matrices, one per body.
			// @@@ check computation of rotational force.

			// compute inertia tensor in global frame
			dMULTIPLY2_333 (tmp, body->mass.I, body->R);
			dMULTIPLY0_333 (globalI + b * 12, body->R, tmp);
			// compute inverse inertia tensor in global frame
			dMULTIPLY2_333 (tmp, body->invI, body->R);
			dMULTIPLY0_333 (globalInvI + b * 12, body->R, tmp);

			for (i = 0; i < 4; i++)
				body->tacc[i] = saveTacc[b * 4 + i];
			// compute rotational force
			dMULTIPLY0_331 (tmp, globalI + b * 12, body->avel);
			dCROSS (body->tacc, -=, body->avel, tmp);

			// add the gravity force to all bodies
			if ((body->flags & dxBodyNoGravity) == 0)
			{
				body->facc[0] = saveFacc[b * 4 + 0] + body->mass.mass * world->gravity[0];
				body->facc[1] = saveFacc[b * 4 + 1] + body->mass.mass * world->gravity[1];
				body->facc[2] = saveFacc[b * 4 + 2] + body->mass.mass * world->gravity[2];
				body->facc[3] = 0;
			} else {
                                body->facc[0] = saveFacc[b * 4 + 0];
                                body->facc[1] = saveFacc[b * 4 + 1];
                                body->facc[2] = saveFacc[b * 4 + 2];
				body->facc[3] = 0;
                        }

		}

#ifdef RANDOM_JOINT_ORDER
#ifdef TIMING
		dTimerNow ("randomizing joint order");
#endif
		//randomize the order of the joints by looping through the array
		//and swapping the current joint pointer with a random one before it.
		for (j = 0; j < nj; j++)
		{
			joint = joints[j];
			dxJoint::Info1 i1 = info[j];
			dxJoint::Info2 i2 = Jinfo[j];
                        const int r = dRandInt(j+1);
			joints[j] = joints[r];
			info[j] = info[r];
			Jinfo[j] = Jinfo[r];
			joints[r] = joint;
			info[r] = i1;
			Jinfo[r] = i2;
		}
#endif

		//now iterate through the random ordered joint array we created.
		for (j = 0; j < nj; j++)
		{
#ifdef TIMING
			dTimerNow ("setting up joint");
#endif
			joint = joints[j];
			bodyPair[0] = joint->node[0].body;
			bodyPair[1] = joint->node[1].body;

			if (bodyPair[0] && (bodyPair[0]->flags & dxBodyDisabled))
				bodyPair[0] = 0;
			if (bodyPair[1] && (bodyPair[1]->flags & dxBodyDisabled))
				bodyPair[1] = 0;
			
			//if this joint is not connected to any enabled bodies, skip it.
			if (!bodyPair[0] && !bodyPair[1])
				continue;
			
			if (bodyPair[0])
			{
				GIPair[0] = globalI + bodyPair[0]->tag * 12;
				GinvIPair[0] = globalInvI + bodyPair[0]->tag * 12;
			}
			if (bodyPair[1])
			{
				GIPair[1] = globalI + bodyPair[1]->tag * 12;
				GinvIPair[1] = globalInvI + bodyPair[1]->tag * 12;
			}

			joints[j]->vtable->getInfo2 (joints[j], Jinfo + j);

			//dInternalStepIslandFast is an exact copy of the old routine with one
			//modification: the calculated forces are added back to the facc and tacc
			//vectors instead of applying them to the bodies and moving them.
			if (info[j].m > 0)
			{
			dInternalStepFast (world, bodyPair, GIPair, GinvIPair, joint, info[j], Jinfo[j], ministep);
			}		
		}
		//  }
#	ifdef TIMING
		dTimerNow ("moving bodies");
#	endif
		//Now we can simulate all the free floating bodies, and move them.
		for (b = 0; b < nb; b++)
		{
			body = bodies[b];

			for (i = 0; i < 4; i++)
			{
				body->facc[i] *= ministep;
				body->tacc[i] *= ministep;
			}

			//apply torque
			dMULTIPLYADD0_331 (body->avel, globalInvI + b * 12, body->tacc);

			//apply force
			for (i = 0; i < 3; i++)
				body->lvel[i] += body->invMass * body->facc[i];

			//move It!
			moveAndRotateBody (body, ministep);
		}
	}
	for (b = 0; b < nb; b++)
		for (j = 0; j < 4; j++)
			bodies[b]->facc[j] = bodies[b]->tacc[j] = 0;
}


#ifdef NO_ISLANDS

// Since the iterative algorithm doesn't care about islands of bodies, this is a
// faster algorithm that just sends it all the joints and bodies in one array.
// It's downfall is it's inability to handle disabled bodies as well as the old one.
static void
processIslandsFast (dxWorld * world, dReal stepsize, int maxiterations)
{
	// nothing to do if no bodies
	if (world->nb <= 0)
		return;

	dInternalHandleAutoDisabling (world,stepsize);

#	ifdef TIMING
	dTimerStart ("creating joint and body arrays");
#	endif
	dxBody **bodies, *body;
	dxJoint **joints, *joint;
	joints = (dxJoint **) ALLOCA (world->nj * sizeof (dxJoint *));
	bodies = (dxBody **) ALLOCA (world->nb * sizeof (dxBody *));

	int nj = 0;
	for (joint = world->firstjoint; joint; joint = (dxJoint *) joint->next)
		joints[nj++] = joint;

	int nb = 0;
	for (body = world->firstbody; body; body = (dxBody *) body->next)
		bodies[nb++] = body;

	dInternalStepIslandFast (world, bodies, nb, joints, nj, stepsize, maxiterations);
#	ifdef TIMING
	dTimerEnd ();
	dTimerReport (stdout, 1);
#	endif
}

#else

//****************************************************************************
// island processing

// this groups all joints and bodies in a world into islands. all objects
// in an island are reachable by going through connected bodies and joints.
// each island can be simulated separately.
// note that joints that are not attached to anything will not be included
// in any island, an so they do not affect the simulation.
//
// this function starts new island from unvisited bodies. however, it will
// never start a new islands from a disabled body. thus islands of disabled
// bodies will not be included in the simulation. disabled bodies are
// re-enabled if they are found to be part of an active island.

static void
processIslandsFast (dxWorld * world, dReal stepsize, int maxiterations)
{
#ifdef TIMING
	dTimerStart ("Island Setup");
#endif
	dxBody *b, *bb, **body;
	dxJoint *j, **joint;

	// nothing to do if no bodies
	if (world->nb <= 0)
		return;


	dInternalHandleAutoDisabling (world,stepsize);

	// make arrays for body and joint lists (for a single island) to go into
	body = (dxBody **) ALLOCA (world->nb * sizeof (dxBody *));
	joint = (dxJoint **) ALLOCA (world->nj * sizeof (dxJoint *));
	int bcount = 0;				// number of bodies in `body'
	int jcount = 0;				// number of joints in `joint'
	int tbcount = 0;
	int tjcount = 0;
	
	// set all body/joint tags to 0
	for (b = world->firstbody; b; b = (dxBody *) b->next)
		b->tag = 0;
	for (j = world->firstjoint; j; j = (dxJoint *) j->next)
		j->tag = 0;

	// allocate a stack of unvisited bodies in the island. the maximum size of
	// the stack can be the lesser of the number of bodies or joints, because
	// new bodies are only ever added to the stack by going through untagged
	// joints. all the bodies in the stack must be tagged!
	int stackalloc = (world->nj < world->nb) ? world->nj : world->nb;
	dxBody **stack = (dxBody **) ALLOCA (stackalloc * sizeof (dxBody *));
	int *autostack = (int *) ALLOCA (stackalloc * sizeof (int));

	for (bb = world->firstbody; bb; bb = (dxBody *) bb->next)
	{
#ifdef TIMING
		dTimerNow ("Island Processing");
#endif
		// get bb = the next enabled, untagged body, and tag it
		if (bb->tag || (bb->flags & dxBodyDisabled))
			continue;
		bb->tag = 1;

		// tag all bodies and joints starting from bb.
		int stacksize = 0;
		int autoDepth = autoEnableDepth;
		b = bb;
		body[0] = bb;
		bcount = 1;
		jcount = 0;
		goto quickstart;
		while (stacksize > 0)
		{
			b = stack[--stacksize];	// pop body off stack
			autoDepth = autostack[stacksize];
			body[bcount++] = b;	// put body on body list
		  quickstart:

			// traverse and tag all body's joints, add untagged connected bodies
			// to stack
			for (dxJointNode * n = b->firstjoint; n; n = n->next)
			{
				if (!n->joint->tag)
				{
					int thisDepth = autoEnableDepth;
					n->joint->tag = 1;
					joint[jcount++] = n->joint;
					if (n->body && !n->body->tag)
					{
						if (n->body->flags & dxBodyDisabled)
							thisDepth = autoDepth - 1;
						if (thisDepth < 0)
							continue;
						n->body->flags &= ~dxBodyDisabled;
						n->body->tag = 1;
						autostack[stacksize] = thisDepth;
						stack[stacksize++] = n->body;
					}
				}
			}
			dIASSERT (stacksize <= world->nb);
			dIASSERT (stacksize <= world->nj);
		}

		// now do something with body and joint lists
		dInternalStepIslandFast (world, body, bcount, joint, jcount, stepsize, maxiterations);

		// what we've just done may have altered the body/joint tag values.
		// we must make sure that these tags are nonzero.
		// also make sure all bodies are in the enabled state.
		int i;
		for (i = 0; i < bcount; i++)
		{
			body[i]->tag = 1;
			body[i]->flags &= ~dxBodyDisabled;
		}
		for (i = 0; i < jcount; i++)
			joint[i]->tag = 1;
		
		tbcount += bcount;
		tjcount += jcount;
	}
	
#ifdef TIMING
	dMessage(0, "Total joints processed: %i, bodies: %i", tjcount, tbcount);
#endif

	// if debugging, check that all objects (except for disabled bodies,
	// unconnected joints, and joints that are connected to disabled bodies)
	// were tagged.
# ifndef dNODEBUG
	for (b = world->firstbody; b; b = (dxBody *) b->next)
	{
		if (b->flags & dxBodyDisabled)
		{
			if (b->tag)
				dDebug (0, "disabled body tagged");
		}
		else
		{
			if (!b->tag)
				dDebug (0, "enabled body not tagged");
		}
	}
	for (j = world->firstjoint; j; j = (dxJoint *) j->next)
	{
		if ((j->node[0].body && (j->node[0].body->flags & dxBodyDisabled) == 0) || (j->node[1].body && (j->node[1].body->flags & dxBodyDisabled) == 0))
		{
			if (!j->tag)
				dDebug (0, "attached enabled joint not tagged");
		}
		else
		{
			if (j->tag)
				dDebug (0, "unattached or disabled joint tagged");
		}
	}
# endif

#	ifdef TIMING
	dTimerEnd ();
	dTimerReport (stdout, 1);
#	endif
}

#endif


void dWorldStepFast1 (dWorldID w, dReal stepsize, int maxiterations)
{
	dUASSERT (w, "bad world argument");
	dUASSERT (stepsize > 0, "stepsize must be > 0");
	processIslandsFast (w, stepsize, maxiterations);
}
