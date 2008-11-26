/*
 * Quickstep constraint solver re-distributed under the ZLib license with permission from Russell L. Smith
 * Original version is from Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org
 Bullet Continuous Collision Detection and Physics Library
 Bullet is Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btSorLcp.h"
#include "btOdeSolverBody.h"
#include "LinearMath/btQuickProf.h"

#ifdef USE_SOR_SOLVER

// SOR LCP taken from ode quickstep, for comparisons to Bullet sequential impulse solver.
#include "LinearMath/btScalar.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include <math.h>
#include <float.h>//FLT_MAX
#ifdef WIN32
#include <memory.h>
#endif
#include <string.h>
#include <stdio.h>

#if defined (WIN32)
#include <malloc.h>
#else
#if defined (__FreeBSD__)
#include <stdlib.h>
#else
#include <alloca.h>
#endif
#endif

#include "btOdeJoint.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
////////////////////////////////////////////////////////////////////
//math stuff
#include "btOdeMacros.h"

//***************************************************************************
// configuration

// for the SOR and CG methods:
// uncomment the following line to use warm starting. this definitely
// help for motor-driven joints. unfortunately it appears to hurt
// with high-friction contacts using the SOR method. use with care

//#define WARM_STARTING 1

// for the SOR method:
// uncomment the following line to randomly reorder constraint rows
// during the solution. depending on the situation, this can help a lot
// or hardly at all, but it doesn't seem to hurt.

//#define RANDOMLY_REORDER_CONSTRAINTS 1

//***************************************************************************
// various common computations involving the matrix JconstraintAxis
// compute iMJ = inv(M)*JconstraintAxis'
inline void compute_invM_JT (int numConstraintRows, dRealMutablePtr JconstraintAxis, dRealMutablePtr iMJ, int *jb,
	//OdeSolverBody* const *body,
	 const btAlignedObjectArray<btOdeSolverBody*> &body,
	dRealPtr inverseInertiaWorld)
{
	int i,j;
	dRealMutablePtr iMJ_ptr = iMJ;
	dRealMutablePtr J_ptr = JconstraintAxis;
	for (i=0; i<numConstraintRows; i++) {
		int b1 = jb[i*2];
		int b2 = jb[i*2+1];
		for (j=0; j<3; j++) 
			iMJ_ptr[j] = body[b1]->m_invMass*J_ptr[j];
		dMULTIPLY0_331 (iMJ_ptr + 3, inverseInertiaWorld + 12*b1, J_ptr + 3);

		if (b2 >= 0) {
			for (j=0; j<3; j++) 
				iMJ_ptr[j+6] = body[b2]->m_invMass*J_ptr[j+6];//inv mass * constraint (normal) axis
			dMULTIPLY0_331 (iMJ_ptr + 9, inverseInertiaWorld + 12*b2, J_ptr + 9);//inverse inertia world * constraint (normal) axis
		}
		J_ptr += 12;
		iMJ_ptr += 12;
	}
}

#if 0
static void multiply_invM_JTSpecial (int numConstraintRows, int nb, dRealMutablePtr iMJ, int *jb,
	dRealMutablePtr in, dRealMutablePtr out,int onlyBody1,int onlyBody2)
{
	int i,j;



	dRealMutablePtr out_ptr1 = out + onlyBody1*6;

	for (j=0; j<6; j++)
		out_ptr1[j] = 0;

	if (onlyBody2 >= 0)
	{
		out_ptr1 = out + onlyBody2*6;

		for (j=0; j<6; j++)
			out_ptr1[j] = 0;
	}

	dRealPtr iMJ_ptr = iMJ;
	for (i=0; i<numConstraintRows; i++) {

		int b1 = jb[i*2];

		dRealMutablePtr out_ptr = out + b1*6;
		if ((b1 == onlyBody1) || (b1 == onlyBody2))
		{
			for (j=0; j<6; j++)
				out_ptr[j] += iMJ_ptr[j] * in[i] ;
		}

		iMJ_ptr += 6;

		int b2 = jb[i*2+1];
		if ((b2 == onlyBody1) || (b2 == onlyBody2))
		{
			if (b2 >= 0)
			{
					out_ptr = out + b2*6;
					for (j=0; j<6; j++)
						out_ptr[j] += iMJ_ptr[j] * in[i];
			}
		}

		iMJ_ptr += 6;

	}
}
#endif


// compute out = inv(M)*JconstraintAxis'*in.

#if 0
static void multiply_invM_JT (int numConstraintRows, int nb, dRealMutablePtr iMJ, int *jb,
	dRealMutablePtr in, dRealMutablePtr out)
{
	int i,j;
	dSetZero1 (out,6*nb);
	dRealPtr iMJ_ptr = iMJ;
	for (i=0; i<numConstraintRows; i++) {
		int b1 = jb[i*2];
		int b2 = jb[i*2+1];
		dRealMutablePtr out_ptr = out + b1*6;
		for (j=0; j<6; j++)
			out_ptr[j] += iMJ_ptr[j] * in[i];
		iMJ_ptr += 6;
		if (b2 >= 0) {
			out_ptr = out + b2*6;
			for (j=0; j<6; j++) out_ptr[j] += iMJ_ptr[j] * in[i];
		}
		iMJ_ptr += 6;
	}
}
#endif


// compute out = JconstraintAxis*in.
inline void multiply_J (int numConstraintRows, dRealMutablePtr JconstraintAxis, int *jb,
	dRealMutablePtr in, dRealMutablePtr out)
{
	int i,j;
	dRealPtr J_ptr = JconstraintAxis;
	for (i=0; i<numConstraintRows; i++) {
		int b1 = jb[i*2];
		int b2 = jb[i*2+1];
		btScalar sum = 0;
		dRealMutablePtr in_ptr = in + b1*6;
		for (j=0; j<6; j++) sum += J_ptr[j] * in_ptr[j];
		J_ptr += 6;
		if (b2 >= 0) {
			in_ptr = in + b2*6;
			for (j=0; j<6; j++) sum += J_ptr[j] * in_ptr[j];
		}
		J_ptr += 6;
		out[i] = sum;
	}
}

//***************************************************************************
// SOR-LCP method

// nb is the number of bodies in the body array.
// JconstraintAxis is an numConstraintRows*12 matrix of constraint rows
// jb is an array of first and second body numbers for each constraint row
// inverseInertiaWorld is the global frame inverse inertia for each body (stacked 3x3 matrices)
//
// this returns lambdaAccumulatedImpulse and fc (the constraint force).
// note: fc is returned as inv(M)*JconstraintAxis'*lambdaAccumulatedImpulse, the constraint force is actually JconstraintAxis'*lambdaAccumulatedImpulse
//
// b, lowerLimit and higherLimit are modified on exit

//------------------------------------------------------------------------------
ATTRIBUTE_ALIGNED16(struct) IndexError {
	btScalar error;		// error to sort on
	int findex;
	int index;		// row index
};

//------------------------------------------------------------------------------
void btSorLcpSolver::SOR_LCP(int numConstraintRows, int nb, dRealMutablePtr JconstraintAxis, int *jb, 
	const btAlignedObjectArray<btOdeSolverBody*> &body,
	dRealPtr inverseInertiaWorld, dRealMutablePtr lambdaAccumulatedImpulse, dRealMutablePtr invMforce, dRealMutablePtr rhs,
	dRealMutablePtr lowerLimit, dRealMutablePtr higherLimit, dRealPtr cfm, int *findex,
	int numiter,float overRelax,
	btStackAlloc* stackAlloc
	)
{
	BT_PROFILE("btSorLcpSolver::SOR_LCP");

	//btBlock* saBlock = stackAlloc->beginBlock();//Remo: 10.10.2007
	AutoBlockSa asaBlock(stackAlloc);

	const int num_iterations = numiter;
	const float sor_w = overRelax;		// SOR over-relaxation parameter

	int i,j;

#ifdef WARM_STARTING
	// for warm starting, this seems to be necessary to prevent
	// jerkiness in motor-driven joints. i have no idea why this works.
	for (i=0; i<numConstraintRows; i++) lambdaAccumulatedImpulse[i] *= 0.9;
#else
	dSetZero1 (lambdaAccumulatedImpulse,numConstraintRows);
#endif

	// the lambdaAccumulatedImpulse computed at the previous iteration.
	// this is used to measure error for when we are reordering the indexes.
	dRealAllocaArray (last_lambda,numConstraintRows);

	// a copy of the 'higherLimit' vector in case findex[] is being used
	dRealAllocaArray (hicopy,numConstraintRows);
	memcpy (hicopy,higherLimit,numConstraintRows*sizeof(float));

	// precompute iMJ = inv(M)*JconstraintAxis'
	dRealAllocaArray (iMJ,numConstraintRows*12);
	compute_invM_JT (numConstraintRows,JconstraintAxis,iMJ,jb,body,inverseInertiaWorld);

	// compute fc=(inv(M)*JconstraintAxis')*lambdaAccumulatedImpulse. we will incrementally maintain fc
	// as we change lambdaAccumulatedImpulse.
#ifdef WARM_STARTING
	multiply_invM_JT (numConstraintRows,nb,iMJ,jb,lambdaAccumulatedImpulse,fc);
#else
	dSetZero1 (invMforce,nb*6);
#endif

	// precompute 1 / diagonals of A
	dRealAllocaArray (Ad,numConstraintRows);
	dRealPtr iMJ_ptr = iMJ;
	dRealMutablePtr J_ptr = JconstraintAxis;
	for (i=0; i<numConstraintRows; i++) {
		float sum = 0;
		for (j=0; j<6; j++) 
			sum += iMJ_ptr[j] * J_ptr[j];
		if (jb[i*2+1] >= 0) {
			for (j=6; j<12; j++) 
				sum += iMJ_ptr[j] * J_ptr[j];
		}
		iMJ_ptr += 12;
		J_ptr += 12;
		Ad[i] = sor_w / sum;//(sum + cfm[i]);
	}

	// scale JconstraintAxis and b by Ad
	J_ptr = JconstraintAxis;
	for (i=0; i<numConstraintRows; i++) {
		for (j=0; j<12; j++) {
			J_ptr[0] *= Ad[i];
			J_ptr++;
		}
		rhs[i] *= Ad[i];
	}

	// scale Ad by CFM
	for (i=0; i<numConstraintRows; i++)
		Ad[i] *= cfm[i];

	// order to solve constraint rows in
	//IndexError *order = (IndexError*) alloca (numConstraintRows*sizeof(IndexError));
	IndexError *order = (IndexError*) ALLOCA (numConstraintRows*sizeof(IndexError));
	

#ifndef REORDER_CONSTRAINTS
	// make sure constraints with findex < 0 come first.
	j=0;
	for (i=0; i<numConstraintRows; i++)
		if (findex[i] < 0)
			order[j++].index = i;
	for (i=0; i<numConstraintRows; i++)
		if (findex[i] >= 0)
			order[j++].index = i;
	dIASSERT (j==numConstraintRows);
#endif

	for (int iteration=0; iteration < num_iterations; iteration++) {

#ifdef REORDER_CONSTRAINTS
		// constraints with findex < 0 always come first.
		if (iteration < 2) {
			// for the first two iterations, solve the constraints in
			// the given order
			for (i=0; i<numConstraintRows; i++) {
				order[i].error = i;
				order[i].findex = findex[i];
				order[i].index = i;
			}
		}
		else {
			// sort the constraints so that the ones converging slowest
			// get solved last. use the absolute (not relative) error.
			for (i=0; i<numConstraintRows; i++) {
				float v1 = dFabs (lambdaAccumulatedImpulse[i]);
				float v2 = dFabs (last_lambda[i]);
				float max = (v1 > v2) ? v1 : v2;
				if (max > 0) {
					//@@@ relative error: order[i].error = dFabs(lambdaAccumulatedImpulse[i]-last_lambda[i])/max;
					order[i].error = dFabs(lambdaAccumulatedImpulse[i]-last_lambda[i]);
				}
				else {
					order[i].error = dInfinity;
				}
				order[i].findex = findex[i];
				order[i].index = i;
			}
		}
		qsort (order,numConstraintRows,sizeof(IndexError),&compare_index_error);
#endif
#ifdef RANDOMLY_REORDER_CONSTRAINTS
                if ((iteration & 7) == 0) {
			for (i=1; i<numConstraintRows; ++i) {
				IndexError tmp = order[i];
				int swapi = dRandInt2(i+1);
				order[i] = order[swapi];
				order[swapi] = tmp;
			}
                }
#endif

		//@@@ potential optimization: swap lambdaAccumulatedImpulse and last_lambda pointers rather
		//    than copying the data. we must make sure lambdaAccumulatedImpulse is properly
		//    returned to the caller
		memcpy (last_lambda,lambdaAccumulatedImpulse,numConstraintRows*sizeof(float));

		for (int i=0; i<numConstraintRows; i++) {
			// @@@ potential optimization: we could pre-sort JconstraintAxis and iMJ, thereby
			//     linearizing access to those arrays. hmmm, this does not seem
			//     like a win, but we should think carefully about our memory
			//     access pattern.

			int index = order[i].index;
			J_ptr = JconstraintAxis + index*12;
			iMJ_ptr = iMJ + index*12;

			// set the limits for this constraint. note that 'hicopy' is used.
			// this is the place where the QuickStep method differs from the
			// direct LCP solving method, since that method only performs this
			// limit adjustment once per time step, whereas this method performs
			// once per iteration per constraint row.
			// the constraints are ordered so that all lambdaAccumulatedImpulse[] values needed have
			// already been computed.
			if (findex[index] >= 0) {
				higherLimit[index] = btFabs (hicopy[index] * lambdaAccumulatedImpulse[findex[index]]);
				lowerLimit[index] = -higherLimit[index];
			}

			int b1 = jb[index*2];
			int b2 = jb[index*2+1];
			
			dRealMutablePtr deltaVelocity = invMforce + 6*b1;

			float deltaAppliedImpulse = rhs[index] - lambdaAccumulatedImpulse[index]*Ad[index];
			
			// @@@ potential optimization: SIMD-ize this and the b2 >= 0 case
			deltaAppliedImpulse -=deltaVelocity[0] * J_ptr[0] + deltaVelocity[1] * J_ptr[1] +
				deltaVelocity[2] * J_ptr[2] + deltaVelocity[3] * J_ptr[3] +
				deltaVelocity[4] * J_ptr[4] + deltaVelocity[5] * J_ptr[5];
			// @@@ potential optimization: handle 1-body constraints in a separate
			//     loop to avoid the cost of test & jump?
			if (b2 >= 0) {
				deltaVelocity = invMforce + 6*b2;
				deltaAppliedImpulse -=deltaVelocity[0] * J_ptr[6] + deltaVelocity[1] * J_ptr[7] +
					deltaVelocity[2] * J_ptr[8] + deltaVelocity[3] * J_ptr[9] +
					deltaVelocity[4] * J_ptr[10] + deltaVelocity[5] * J_ptr[11];
			}

			// compute lambdaAccumulatedImpulse and clamp it to [lowerLimit,higherLimit].
			// @@@ potential optimization: does SSE have clamping instructions
			//     to save test+jump penalties here?
			float sum = lambdaAccumulatedImpulse[index] + deltaAppliedImpulse;
			if (sum < lowerLimit[index]) {
				deltaAppliedImpulse = lowerLimit[index]-lambdaAccumulatedImpulse[index];
				lambdaAccumulatedImpulse[index] = lowerLimit[index];
			}
			else if (sum > higherLimit[index]) {
				deltaAppliedImpulse = higherLimit[index]-lambdaAccumulatedImpulse[index];
				lambdaAccumulatedImpulse[index] = higherLimit[index];
			}
			else {
				lambdaAccumulatedImpulse[index] = sum;
			}

			//@@@ a trick that may or may not help
			//float ramp = (1-((float)(iteration+1)/(float)num_iterations));
			//deltaAppliedImpulse *= ramp;

			// update invMforce.
			// @@@ potential optimization: SIMD for this and the b2 >= 0 case
			deltaVelocity = invMforce + 6*b1;
			deltaVelocity[0] += deltaAppliedImpulse * iMJ_ptr[0];
			deltaVelocity[1] += deltaAppliedImpulse * iMJ_ptr[1];
			deltaVelocity[2] += deltaAppliedImpulse * iMJ_ptr[2];
			deltaVelocity[3] += deltaAppliedImpulse * iMJ_ptr[3];
			deltaVelocity[4] += deltaAppliedImpulse * iMJ_ptr[4];
			deltaVelocity[5] += deltaAppliedImpulse * iMJ_ptr[5];
			// @@@ potential optimization: handle 1-body constraints in a separate
			//     loop to avoid the cost of test & jump?
			if (b2 >= 0) {
				deltaVelocity = invMforce + 6*b2;
				deltaVelocity[0] += deltaAppliedImpulse * iMJ_ptr[6];
				deltaVelocity[1] += deltaAppliedImpulse * iMJ_ptr[7];
				deltaVelocity[2] += deltaAppliedImpulse * iMJ_ptr[8];
				deltaVelocity[3] += deltaAppliedImpulse * iMJ_ptr[9];
				deltaVelocity[4] += deltaAppliedImpulse * iMJ_ptr[10];
				deltaVelocity[5] += deltaAppliedImpulse * iMJ_ptr[11];
			}
		}
	}
	//stackAlloc->endBlock(saBlock);//Remo: 10.10.2007
}

//------------------------------------------------------------------------------
void btSorLcpSolver::SolveInternal1 (const btAlignedObjectArray<btOdeSolverBody*> &body, int nb,
			btAlignedObjectArray<btOdeJoint*> &joint, 
			int nj, const btContactSolverInfo& solverInfo,
			btStackAlloc* stackAlloc)
{
	BT_PROFILE("btSorLcpSolver::SolveInternal1");
	//btBlock* saBlock = stackAlloc->beginBlock();//Remo: 10.10.2007
	AutoBlockSa asaBlock(stackAlloc);

	int numIter = solverInfo.m_numIterations;
	float sOr = solverInfo.m_sor;

	int i,j;

	btScalar stepsize1 = dRecip(solverInfo.m_timeStep);

	// number all bodies in the body list - set their tag values
	for (i=0; i<nb; i++)
		body[i]->m_odeTag = i;

	// make a local copy of the joint array, because we might want to modify it.
	// (the "btOdeJoint *const*" declaration says we're allowed to modify the joints
	// but not the joint array, because the caller might need it unchanged).
	//@@@ do we really need to do this? we'll be sorting constraint rows individually, not joints
	//btOdeJoint **joint = (btOdeJoint**) alloca (nj * sizeof(btOdeJoint*));
	//memcpy (joint,_joint,nj * sizeof(btOdeJoint*));

	// for all bodies, compute the inertia tensor and its inverse in the global
	// frame, and compute the rotational force and add it to the torque
	// accumulator. I and inverseInertiaWorld are a vertical stack of 3x4 matrices, one per body.
	dRealAllocaArray (I,3*4*nb);
	dRealAllocaArray (inverseInertiaWorld,3*4*nb);
/*	for (i=0; i<nb; i++) {
		dMatrix3 tmp;
		// compute inertia tensor in global frame
		dMULTIPLY2_333 (tmp,body[i]->m_I,body[i]->m_R);
		// compute inverse inertia tensor in global frame
		dMULTIPLY2_333 (tmp,body[i]->m_invI,body[i]->m_R);
		dMULTIPLY0_333 (inverseInertiaWorld+i*12,body[i]->m_R,tmp);
		// compute rotational force
		dCROSS (body[i]->m_tacc,-=,body[i]->getAngularVelocity(),tmp);
	}
*/
	for (i=0; i<nb; i++) {
		dMatrix3 tmp;
		// compute inertia tensor in global frame
		dMULTIPLY2_333 (tmp,body[i]->m_I,body[i]->m_R);
		dMULTIPLY0_333 (I+i*12,body[i]->m_R,tmp);

		// compute inverse inertia tensor in global frame
		dMULTIPLY2_333 (tmp,body[i]->m_invI,body[i]->m_R);
		dMULTIPLY0_333 (inverseInertiaWorld+i*12,body[i]->m_R,tmp);
		// compute rotational force
//		dMULTIPLY0_331 (tmp,I+i*12,body[i]->m_angularVelocity);
//		dCROSS (body[i]->m_tacc,-=,body[i]->m_angularVelocity,tmp);
	}




	// get joint information (numConstraintRows = total constraint dimension, nub = number of unbounded variables).
	// joints with numConstraintRows=0 are inactive and are removed from the joints array
	// entirely, so that the code that follows does not consider them.
	//@@@ do we really need to save all the info1's
	btOdeJoint::Info1 *info = (btOdeJoint::Info1*) ALLOCA (nj*sizeof(btOdeJoint::Info1));
	
	for (i=0, j=0; j<nj; j++) {	// i=dest, j=src
		joint[j]->GetInfo1 (info+i);
		dIASSERT (info[i].m_numConstraintRows >= 0 && info[i].m_numConstraintRows <= 6 && info[i].nub >= 0 && info[i].nub <= info[i].m_numConstraintRows);
		if (info[i].m_numConstraintRows > 0) {
			joint[i] = joint[j];
			i++;
		}
	}
	nj = i;

	// create the row offset array
	int numConstraintRows = 0;
	int *constraintRowOffsets = (int*) ALLOCA (nj*sizeof(int));
	for (i=0; i<nj; i++) {
		constraintRowOffsets[i] = numConstraintRows;
		numConstraintRows += info[i].m_numConstraintRows;
	}

	// if there are constraints, compute the constraint force
	dRealAllocaArray (JconstraintAxis,numConstraintRows*12);
	int *jb = (int*) ALLOCA (numConstraintRows*2*sizeof(int));
	if (numConstraintRows > 0) {
		// create a constraint equation right hand side vector `c', a constraint
		// force mixing vector `cfm', and LCP low and high bound vectors, and an
		// 'findex' vector.
		dRealAllocaArray (c_rhs,numConstraintRows);
		dRealAllocaArray (cfm,numConstraintRows);
		dRealAllocaArray (lowerLimit,numConstraintRows);
		dRealAllocaArray (higherLimit,numConstraintRows);

		int *findex = (int*) ALLOCA (numConstraintRows*sizeof(int));

		dSetZero1 (c_rhs,numConstraintRows);
		dSetValue1 (cfm,numConstraintRows,solverInfo.m_globalCfm);
		dSetValue1 (lowerLimit,numConstraintRows,-dInfinity);
		dSetValue1 (higherLimit,numConstraintRows, dInfinity);
		for (i=0; i<numConstraintRows; i++) findex[i] = -1;

		// get jacobian data from constraints. an numConstraintRows*12 matrix will be created
		// to store the two jacobian blocks from each constraint. it has this
		// format:
		//
		//   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 \    .
		//   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2  }-- jacobian for joint 0, body 1 and body 2 (3 rows)
		//   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 /
		//   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 }--- jacobian for joint 1, body 1 and body 2 (3 rows)
		//   etc...
		//
		//   (lll) = linear jacobian data
		//   (aaa) = angular jacobian data
		//
		dSetZero1 (JconstraintAxis,numConstraintRows*12);
		btOdeJoint::Info2 Jinfo;
		Jinfo.rowskip = 12;
		Jinfo.fps = stepsize1;
		Jinfo.erp = solverInfo.m_erp;
		for (i=0; i<nj; i++) {
			Jinfo.m_J1linearAxis = JconstraintAxis + constraintRowOffsets[i]*12;
			Jinfo.m_J1angularAxis = Jinfo.m_J1linearAxis + 3;
			Jinfo.m_J2linearAxis = Jinfo.m_J1linearAxis + 6;
			Jinfo.m_J2angularAxis = Jinfo.m_J1linearAxis + 9;
			Jinfo.m_constraintError = c_rhs + constraintRowOffsets[i];
			Jinfo.cfm = cfm + constraintRowOffsets[i];
			Jinfo.m_lowerLimit = lowerLimit + constraintRowOffsets[i];
			Jinfo.m_higherLimit = higherLimit + constraintRowOffsets[i];
			Jinfo.findex = findex + constraintRowOffsets[i];
			joint[i]->GetInfo2 (&Jinfo);

			if (Jinfo.m_constraintError[0] > solverInfo.m_maxErrorReduction)
				Jinfo.m_constraintError[0] = solverInfo.m_maxErrorReduction;

			// adjust returned findex values for global index numbering
			for (j=0; j<info[i].m_numConstraintRows; j++) {
				if (findex[constraintRowOffsets[i] + j] >= 0)
					findex[constraintRowOffsets[i] + j] += constraintRowOffsets[i];
			}
		}

		// create an array of body numbers for each joint row
		int *jb_ptr = jb;
		for (i=0; i<nj; i++) {
			int b1 = (joint[i]->node[0].body) ? (joint[i]->node[0].body->m_odeTag) : -1;
			int b2 = (joint[i]->node[1].body) ? (joint[i]->node[1].body->m_odeTag) : -1;
			for (j=0; j<info[i].m_numConstraintRows; j++) {
				jb_ptr[0] = b1;
				jb_ptr[1] = b2;
				jb_ptr += 2;
			}
		}
		dIASSERT (jb_ptr == jb+2*numConstraintRows);

		// compute the right hand side `rhs'
		dRealAllocaArray (tmp1,nb*6);
		// put v/h + invM*fe into tmp1
		for (i=0; i<nb; i++) {
			btScalar body_invMass = body[i]->m_invMass;
			for (j=0; j<3; j++)
				tmp1[i*6+j] = body[i]->m_facc[j] * body_invMass + body[i]->m_linearVelocity[j] * stepsize1;
			dMULTIPLY0_331NEW (tmp1 + i*6 + 3,=,inverseInertiaWorld + i*12,body[i]->m_tacc);
			for (j=0; j<3; j++)
				tmp1[i*6+3+j] += body[i]->m_angularVelocity[j] * stepsize1;
		}

		// put JconstraintAxis*tmp1 into rhs
		dRealAllocaArray (rhs,numConstraintRows);
		multiply_J (numConstraintRows,JconstraintAxis,jb,tmp1,rhs);

		// complete rhs
		for (i=0; i<numConstraintRows; i++) 
			rhs[i] = c_rhs[i]*stepsize1 - rhs[i];

		// scale CFM
		for (i=0; i<numConstraintRows; i++)
			cfm[i] *= stepsize1;

		// load lambdaAccumulatedImpulse from the value saved on the previous iteration
		dRealAllocaArray (lambdaAccumulatedImpulse,numConstraintRows);
#ifdef WARM_STARTING
		dSetZero1 (lambdaAccumulatedImpulse,numConstraintRows);	//@@@ shouldn't be necessary
		for (i=0; i<nj; i++) {
			memcpy (lambdaAccumulatedImpulse+constraintRowOffsets[i],joint[i]->lambdaAccumulatedImpulse,info[i].numConstraintRows * sizeof(btScalar));
		}
#endif

		// solve the LCP problem and get lambdaAccumulatedImpulse and invM*constraint_force
		dRealAllocaArray (cforce,nb*6);

		/// SOR_LCP
		SOR_LCP (numConstraintRows,nb,JconstraintAxis,jb,body,inverseInertiaWorld,lambdaAccumulatedImpulse,cforce,rhs,lowerLimit,higherLimit,cfm,findex,numIter,sOr,stackAlloc);

#ifdef WARM_STARTING
		// save lambdaAccumulatedImpulse for the next iteration
		//@@@ note that this doesn't work for contact joints yet, as they are
		// recreated every iteration
		for (i=0; i<nj; i++) {
			memcpy (joint[i]->lambdaAccumulatedImpulse,lambdaAccumulatedImpulse+constraintRowOffsets[i],info[i].numConstraintRows * sizeof(btScalar));
		}
#endif

		// note that the SOR method overwrites rhs and JconstraintAxis at this point, so
		// they should not be used again.
		// add stepsize * cforce to the body velocity
		for (i=0; i<nb; i++) {
			for (j=0; j<3; j++)
				body[i]->m_linearVelocity[j] += solverInfo.m_timeStep* cforce[i*6+j];
			for (j=0; j<3; j++)
				body[i]->m_angularVelocity[j] += solverInfo.m_timeStep* cforce[i*6+3+j];

		}
	}

	// compute the velocity update:
	// add stepsize * invM * fe to the body velocity
	for (i=0; i<nb; i++) {
		btScalar body_invMass = body[i]->m_invMass;
		btVector3 linvel = body[i]->m_linearVelocity;
		btVector3 angvel = body[i]->m_angularVelocity;

		for (j=0; j<3; j++)
		{
			linvel[j] += solverInfo.m_timeStep * body_invMass * body[i]->m_facc[j];
		}
		for (j=0; j<3; j++)
		{
			body[i]->m_tacc[j] *= solverInfo.m_timeStep;
		}
		dMULTIPLY0_331NEW(angvel,+=,inverseInertiaWorld + i*12,body[i]->m_tacc);
		body[i]->m_angularVelocity = angvel;
	}
	//stackAlloc->endBlock(saBlock);//Remo: 10.10.2007
}


#endif //USE_SOR_SOLVER
