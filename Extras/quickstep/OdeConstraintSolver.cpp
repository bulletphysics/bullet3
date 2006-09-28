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



#include "OdeConstraintSolver.h"

#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btContactConstraint.h"
#include "btSolve2LinearConstraint.h"
#include "btContactSolverInfo.h"
#include "Dynamics/BU_Joint.h"
#include "Dynamics/ContactJoint.h"

#include "LinearMath/btIDebugDraw.h"

#define USE_SOR_SOLVER

#include "SorLcp.h"

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

class BU_Joint;

//see below

//to bridge with ODE quickstep, we make a temp copy of the rigidbodies in each simultion island, stored in an array of size MAX_RIGIDBODIES
#define MAX_QUICKSTEP_RIGIDBODIES 8192

OdeConstraintSolver::OdeConstraintSolver():
m_cfm(0.f),//1e-5f),
m_erp(0.4f)
{
}







//iterative lcp and penalty method
float OdeConstraintSolver::solveGroup(btPersistentManifold** manifoldPtr, int numManifolds,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer)
{
	m_CurBody = 0;
	m_CurJoint = 0;


	btRigidBody* bodies [MAX_QUICKSTEP_RIGIDBODIES];

	int numBodies = 0;
	BU_Joint* joints [MAX_QUICKSTEP_RIGIDBODIES*4];
	int numJoints = 0;

	for (int j=0;j<numManifolds;j++)
	{

		int body0=-1,body1=-1;

		btPersistentManifold* manifold = manifoldPtr[j];
		if (manifold->getNumContacts() > 0)
		{
			body0 = ConvertBody((btRigidBody*)manifold->getBody0(),bodies,numBodies);
			body1 = ConvertBody((btRigidBody*)manifold->getBody1(),bodies,numBodies);
			ConvertConstraint(manifold,joints,numJoints,bodies,body0,body1,debugDrawer);
		}
	}

	SolveInternal1(m_cfm,m_erp,bodies,numBodies,joints,numJoints,infoGlobal);

	return 0.f;

}

/////////////////////////////////////////////////////////////////////////////////


typedef btScalar dQuaternion[4];
#define _R(i,j) R[(i)*4+(j)]

void dRfromQ1 (dMatrix3 R, const dQuaternion q)
{
  // q = (s,vx,vy,vz)
  btScalar qq1 = 2.f*q[1]*q[1];
  btScalar qq2 = 2.f*q[2]*q[2];
  btScalar qq3 = 2.f*q[3]*q[3];
  _R(0,0) = 1.f - qq2 - qq3;
  _R(0,1) = 2*(q[1]*q[2] - q[0]*q[3]);
  _R(0,2) = 2*(q[1]*q[3] + q[0]*q[2]);
  _R(0,3) = 0.f;
	
  _R(1,0) = 2*(q[1]*q[2] + q[0]*q[3]);
  _R(1,1) = 1.f - qq1 - qq3;
  _R(1,2) = 2*(q[2]*q[3] - q[0]*q[1]);
  _R(1,3) = 0.f;

  _R(2,0) = 2*(q[1]*q[3] - q[0]*q[2]);
  _R(2,1) = 2*(q[2]*q[3] + q[0]*q[1]);
  _R(2,2) = 1.f - qq1 - qq2;
  _R(2,3) = 0.f;

}



int OdeConstraintSolver::ConvertBody(btRigidBody* body,btRigidBody** bodies,int& numBodies)
{
	if (!body || (body->getInvMass() == 0.f) )
	{
		return -1;
	}
	//first try to find
	int i,j;
	for (i=0;i<numBodies;i++)
	{
		if (bodies[i] == body)
			return i;
	}
	//if not found, create a new body
	bodies[numBodies++] = body;
	//convert data


	body->m_facc.setValue(0,0,0,0);
	body->m_tacc.setValue(0,0,0,0);
	
	//are the indices the same ?
	for (i=0;i<4;i++)
	{
		for ( j=0;j<3;j++)
		{
			body->m_invI[i+4*j] = 0.f;
			body->m_I[i+4*j] = 0.f;
		}
	}
	body->m_invI[0+4*0] = 	body->getInvInertiaDiagLocal()[0];
	body->m_invI[1+4*1] = 	body->getInvInertiaDiagLocal()[1];
	body->m_invI[2+4*2] = 	body->getInvInertiaDiagLocal()[2];

	body->m_I[0+0*4] = 1.f/body->getInvInertiaDiagLocal()[0];
	body->m_I[1+1*4] = 1.f/body->getInvInertiaDiagLocal()[1];
	body->m_I[2+2*4] = 1.f/body->getInvInertiaDiagLocal()[2];
	

	
	
	dQuaternion q;

	q[1] = body->getOrientation()[0];
	q[2] = body->getOrientation()[1];
	q[3] = body->getOrientation()[2];
	q[0] = body->getOrientation()[3];
	
	dRfromQ1(body->m_R,q);
	
	return numBodies-1;
}




	
#define MAX_JOINTS_1 65535

static ContactJoint gJointArray[MAX_JOINTS_1];


void OdeConstraintSolver::ConvertConstraint(btPersistentManifold* manifold,BU_Joint** joints,int& numJoints,
					   btRigidBody** bodies,int _bodyId0,int _bodyId1,btIDebugDraw* debugDrawer)
{


	manifold->refreshContactPoints(((btRigidBody*)manifold->getBody0())->getCenterOfMassTransform(),
		((btRigidBody*)manifold->getBody1())->getCenterOfMassTransform());

	int bodyId0 = _bodyId0,bodyId1 = _bodyId1;

	int i,numContacts = manifold->getNumContacts();
	
	bool swapBodies = (bodyId0 < 0);

	
	btRigidBody* body0,*body1;

	if (swapBodies)
	{
		bodyId0 = _bodyId1;
		bodyId1 = _bodyId0;

		body0 = (btRigidBody*)manifold->getBody1();
		body1 = (btRigidBody*)manifold->getBody0();

	} else
	{
		body0 = (btRigidBody*)manifold->getBody0();
		body1 = (btRigidBody*)manifold->getBody1();
	}

	assert(bodyId0 >= 0);

	btVector3 color(0,1,0);
	for (i=0;i<numContacts;i++)
	{

		if (debugDrawer)
		{
			const btManifoldPoint& cp = manifold->getContactPoint(i);

			debugDrawer->drawContactPoint(
				cp.m_positionWorldOnB,
				cp.m_normalWorldOnB,
				cp.getDistance(),
				cp.getLifeTime(),
				color);

		}
		assert (m_CurJoint < MAX_JOINTS_1);

//		if (manifold->getContactPoint(i).getDistance() < 0.0f)
		{
			ContactJoint* cont = new (&gJointArray[m_CurJoint++]) ContactJoint( manifold ,i, swapBodies,body0,body1);

			cont->node[0].joint = cont;
			cont->node[0].body = bodyId0 >= 0 ? bodies[bodyId0] : 0;
			
			cont->node[1].joint = cont;
			cont->node[1].body = bodyId1 >= 0 ? bodies[bodyId1] : 0;
			
			joints[numJoints++] = cont;
			for (int i=0;i<6;i++)
				cont->lambda[i] = 0.f;

			cont->flags = 0;
		}
	}

	//create a new contact constraint
};

