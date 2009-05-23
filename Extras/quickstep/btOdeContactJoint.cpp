/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "btOdeContactJoint.h"
#include "btOdeSolverBody.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"


//this constant needs to be set up so different solvers give 'similar' results
#define FRICTION_CONSTANT 120.f


btOdeContactJoint::btOdeContactJoint(btPersistentManifold* manifold,int index,bool swap,btOdeSolverBody* body0,btOdeSolverBody* body1)
:m_manifold(manifold),
m_index(index),
m_swapBodies(swap),
m_body0(body0),
m_body1(body1)
{
}

int m_numRows = 3;


void btOdeContactJoint::GetInfo1(Info1 *info)
{
	info->m_numConstraintRows = m_numRows;
	//friction adds another 2...
	
	info->nub = 0;
}

#define dCROSS(a,op,b,c) \
  (a)[0] op ((b)[1]*(c)[2] - (b)[2]*(c)[1]); \
  (a)[1] op ((b)[2]*(c)[0] - (b)[0]*(c)[2]); \
  (a)[2] op ((b)[0]*(c)[1] - (b)[1]*(c)[0]);

#define M_SQRT12 btScalar(0.7071067811865475244008443621048490)

#define dRecipSqrt(x) ((float)(1.0f/btSqrt(float(x))))		/* reciprocal square root */



void dPlaneSpace1 (const dVector3 n, dVector3 p, dVector3 q);
void dPlaneSpace1 (const dVector3 n, dVector3 p, dVector3 q)
{
  if (btFabs(n[2]) > M_SQRT12) {
    // choose p in y-z plane
    btScalar a = n[1]*n[1] + n[2]*n[2];
    btScalar k = dRecipSqrt (a);
    p[0] = 0;
    p[1] = -n[2]*k;
    p[2] = n[1]*k;
    // set q = n x p
    q[0] = a*k;
    q[1] = -n[0]*p[2];
    q[2] = n[0]*p[1];
  }
  else {
    // choose p in x-y plane
    btScalar a = n[0]*n[0] + n[1]*n[1];
    btScalar k = dRecipSqrt (a);
    p[0] = -n[1]*k;
    p[1] = n[0]*k;
    p[2] = 0;
    // set q = n x p
    q[0] = -n[2]*p[1];
    q[1] = n[2]*p[0];
    q[2] = a*k;
  }
}



void btOdeContactJoint::GetInfo2(Info2 *info)
{
	
	int s = info->rowskip;
	int s2 = 2*s;
	
	float swapFactor = m_swapBodies ? -1.f : 1.f;
	
	// get normal, with sign adjusted for body1/body2 polarity
	dVector3 normal;
	
	
	btManifoldPoint& point = m_manifold->getContactPoint(m_index);
	
	normal[0] = swapFactor*point.m_normalWorldOnB.x();
	normal[1] = swapFactor*point.m_normalWorldOnB.y();
	normal[2] = swapFactor*point.m_normalWorldOnB.z();
	normal[3] = 0;	// @@@ hmmm
	
	assert(m_body0);
	//	if (GetBody0())
	btVector3 relativePositionA;
	{
		relativePositionA = point.getPositionWorldOnA() - m_body0->m_centerOfMassPosition;
		
		// set jacobian for normal
		info->m_J1linearAxis[0] = normal[0];
		info->m_J1linearAxis[1] = normal[1];
		info->m_J1linearAxis[2] = normal[2];
		dCROSS (info->m_J1angularAxis,=,relativePositionA,normal);
		
	}

	btVector3 relativePositionB(0,0,0);
	if (m_body1)
	{
		//		if (GetBody1())
		
		{
			btVector3	posBody1 = m_body1 ? m_body1->m_centerOfMassPosition : btVector3(0,0,0);
			relativePositionB = point.getPositionWorldOnB() - posBody1;
			
			//			for (i=0; i<3; i++) c2[i] = j->contact.geom.pos[i] -
			//					  j->node[1].body->pos[i];
			
			info->m_J2linearAxis[0] = -normal[0];
			info->m_J2linearAxis[1] = -normal[1];
			info->m_J2linearAxis[2] = -normal[2];
			dCROSS (info->m_J2angularAxis,= -,relativePositionB,normal);
		}
	}
	
	float depth = -point.getDistance();
//	if (depth < 0.f)
//		depth = 0.f;
	
	info->m_constraintError[0] = depth * info->fps * info->erp ;
	//float maxvel = .2f;

//	if (info->c[0] > maxvel)
//		info->c[0] = maxvel;


	//can override it, not necessary
//	info->cfm[0] = 0.f;
//	info->cfm[1] = 0.f;
//	info->cfm[2] = 0.f;
	
	
	
	// set LCP limits for normal
	info->m_lowerLimit[0] = 0;
	info->m_higherLimit[0] = BT_LARGE_FLOAT;//dInfinity;
	info->m_lowerLimit[1] = 0;
	info->m_higherLimit[1] = 0.f;
	info->m_lowerLimit[2] = 0.f;
	info->m_higherLimit[2] = 0.f;

#define DO_THE_FRICTION_2
#ifdef DO_THE_FRICTION_2
	// now do jacobian for tangential forces
	dVector3 t1,t2;	// two vectors tangential to normal
	
	dVector3 c1;
	c1[0] = relativePositionA.x();
	c1[1] = relativePositionA.y();
	c1[2] = relativePositionA.z();
	
	dVector3 c2;
	c2[0] = relativePositionB.x();
	c2[1] = relativePositionB.y();
	c2[2] = relativePositionB.z();
	
	//combined friction is available in the contact point
	//float friction = 0.25;//FRICTION_CONSTANT ;//* m_body0->m_friction * m_body1->m_friction;
	float friction = 1.0f;
	if (m_body0 != 0) 
		friction *= m_body0->m_friction;
	if (m_body1 != 0) 
		friction *= m_body1->m_friction;
	
	// first friction direction
	if (m_numRows >= 2) 
	{
		
		
		
		dPlaneSpace1 (normal,t1,t2);
		
		info->m_J1linearAxis[s+0] = t1[0];
		info->m_J1linearAxis[s+1] = t1[1];
		info->m_J1linearAxis[s+2] = t1[2];
		dCROSS (info->m_J1angularAxis+s,=,c1,t1);
//		if (1) { //j->node[1].body) {
			info->m_J2linearAxis[s+0] = -t1[0];
			info->m_J2linearAxis[s+1] = -t1[1];
			info->m_J2linearAxis[s+2] = -t1[2];
			dCROSS (info->m_J2angularAxis+s,= -,c2,t1);
//		}
		// set right hand side
//		if (0) {//j->contact.surface.mode & dContactMotion1) {
			//info->c[1] = j->contact.surface.motion1;
//		}
		// set LCP bounds and friction index. this depends on the approximation
		// mode
		//BT_LARGE_FLOAT
		
		
		info->m_lowerLimit[1] = -friction;//-j->contact.surface.mu;
		info->m_higherLimit[1] = friction;//j->contact.surface.mu;
//		if (1)//j->contact.surface.mode & dContactApprox1_1) 
			info->findex[1] = 0;
		
		// set slip (constraint force mixing)
//		if (0)//j->contact.surface.mode & dContactSlip1)
//		{
//			//	info->cfm[1] = j->contact.surface.slip1;
//		} else
//		{
//			//info->cfm[1] = 0.f;
//		}
	}
	
	// second friction direction
	if (m_numRows >= 3) {
		info->m_J1linearAxis[s2+0] = t2[0];
		info->m_J1linearAxis[s2+1] = t2[1];
		info->m_J1linearAxis[s2+2] = t2[2];
		dCROSS (info->m_J1angularAxis+s2,=,c1,t2);
//		if (1) { //j->node[1].body) {
			info->m_J2linearAxis[s2+0] = -t2[0];
			info->m_J2linearAxis[s2+1] = -t2[1];
			info->m_J2linearAxis[s2+2] = -t2[2];
			dCROSS (info->m_J2angularAxis+s2,= -,c2,t2);
//		}

		// set right hand side
//		if (0){//j->contact.surface.mode & dContactMotion2) {
			//info->c[2] = j->contact.surface.motion2;
//		}
		// set LCP bounds and friction index. this depends on the approximation
		// mode
//		if (0){//j->contact.surface.mode & dContactMu2) {
			//info->m_lowerLimit[2] = -j->contact.surface.mu2;
			//info->m_higherLimit[2] = j->contact.surface.mu2;
//		}
//		else {
			info->m_lowerLimit[2] = -friction;
			info->m_higherLimit[2] = friction;
//		}
		if (1)//j->contact.surface.mode & dContactApprox1_2) 
			
		{
			info->findex[2] = 0;
		}
		// set slip (constraint force mixing)
//		if (0) //j->contact.surface.mode & dContactSlip2)
			
//		{
			//info->cfm[2] = j->contact.surface.slip2;
			
//		}
	}
	
#endif //DO_THE_FRICTION_2
	
}

