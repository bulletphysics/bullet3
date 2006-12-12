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



#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"

#include "BMF_Api.h"
#include <stdio.h> //printf debugging

#include "ConstraintDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"

const int numObjects = 3;

#define CUBE_HALF_EXTENTS 1.f

GLDebugDrawer debugDrawer;

int main(int argc,char** argv)
{

	ConstraintDemo* constraintDemo = new ConstraintDemo();

	constraintDemo->initPhysics();	

	constraintDemo->setCameraDistance(26.f);

	return glutmain(argc, argv,640,480,"Constraint Demo. http://www.continuousphysics.com/Bullet/phpBB2/",constraintDemo);
}

btTransform sliderTransform;
btVector3 lowerSliderLimit = btVector3(-10,0,0);
btVector3 hiSliderLimit = btVector3(10,0,0);

btRigidBody* d6body0 =0;

void	drawLimit()
{
		btVector3 from = sliderTransform*lowerSliderLimit;
		btVector3 to = sliderTransform*hiSliderLimit;
		btVector3 color(255,0,0);
		glBegin(GL_LINES);
		glColor3f(color.getX(), color.getY(), color.getZ());
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
		if (d6body0)
		{
			from = d6body0->getWorldTransform().getOrigin();
			to = from + d6body0->getWorldTransform().getBasis() * btVector3(0,0,10);
			glVertex3d(from.getX(), from.getY(), from.getZ());
			glVertex3d(to.getX(), to.getY(), to.getZ());
		}
		glEnd();
}



void	ConstraintDemo::initPhysics()
{
	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher();
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	btOverlappingPairCache* pairCache = new btAxisSweep3(worldMin,worldMax);
	//btOverlappingPairCache* broadphase = new btSimpleBroadphase();
	btConstraintSolver* constraintSolver = new btSequentialImpulseConstraintSolver();
	//ConstraintSolver* solver = new OdeConstraintSolver;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver);

	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	
	m_dynamicsWorld->setDebugDrawer(&debugDrawer);

	btCollisionShape* shape = new btBoxShape(btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
	btTransform trans;
	trans.setIdentity();
	trans.setOrigin(btVector3(0,20,0));

	float mass = 1.f;
	//point to point constraint (ball socket)
	{
		btRigidBody* body0 = localCreateRigidBody( mass,trans,shape);
		trans.setOrigin(btVector3(2*CUBE_HALF_EXTENTS,20,0));

		mass = 1.f;
		btRigidBody* body1 = 0;//localCreateRigidBody( mass,trans,shape);
		//body1->setActivationState(DISABLE_DEACTIVATION);
		//body1->setDamping(0.3,0.3);

		btVector3 pivotInA(CUBE_HALF_EXTENTS,-CUBE_HALF_EXTENTS,-CUBE_HALF_EXTENTS);
		btVector3 axisInA(0,0,1);

		btVector3 pivotInB = body1 ? body1->getCenterOfMassTransform().inverse()(body0->getCenterOfMassTransform()(pivotInA)) : pivotInA;
		btVector3 axisInB = body1? 
			(body1->getCenterOfMassTransform().getBasis().inverse()*(body1->getCenterOfMassTransform().getBasis() * axisInA)) : 
		body0->getCenterOfMassTransform().getBasis() * axisInA;

		//btTypedConstraint* p2p = new btPoint2PointConstraint(*body0,*body1,pivotInA,pivotInB);
		//btTypedConstraint* hinge = new btHingeConstraint(*body0,*body1,pivotInA,pivotInB,axisInA,axisInB);
		btHingeConstraint* hinge = new btHingeConstraint(*body0,pivotInA,axisInA);
		
		//use zero targetVelocity and a small maxMotorImpulse to simulate joint friction
		//float	targetVelocity = 0.f;
		//float	maxMotorImpulse = 0.01;
		float	targetVelocity = 1.f;
		float	maxMotorImpulse = 1.0f;
		hinge->enableAngularMotor(true,targetVelocity,maxMotorImpulse);

		m_dynamicsWorld->addConstraint(hinge);//p2p);

	}

	

	//create a slider, using the generic D6 constraint
	{
		mass = 1.f;
		btVector3 sliderWorldPos(0,10,0);
		btVector3 sliderAxis(1,0,0);
		btScalar angle=0.f;//SIMD_RADS_PER_DEG * 10.f;
		btMatrix3x3 sliderOrientation(btQuaternion(sliderAxis ,angle));
		trans.setIdentity();
		trans.setOrigin(sliderWorldPos);
		//trans.setBasis(sliderOrientation);
		sliderTransform = trans;

		d6body0 = localCreateRigidBody( mass,trans,shape);
		d6body0->setActivationState(DISABLE_DEACTIVATION);
		btRigidBody* fixedBody1 = localCreateRigidBody(0,trans,0);

		btTransform frameInA, frameInB;
		frameInA = btTransform::getIdentity();
		frameInB = btTransform::getIdentity();
		
		btGeneric6DofConstraint* slider = new btGeneric6DofConstraint(*d6body0,*fixedBody1,frameInA,frameInB);
		slider->setLinearLowerLimit(lowerSliderLimit);
		slider->setLinearUpperLimit(hiSliderLimit);

		//range should be small, otherwise singularities will 'explode' the constraint
		slider->setAngularLowerLimit(btVector3(10,0,0));
		slider->setAngularUpperLimit(btVector3(0,0,0));

		m_dynamicsWorld->addConstraint(slider);

	}

}


void ConstraintDemo::clientMoveAndDisplay()
{
	
 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

 	float dt = float(m_clock.getTimeMicroseconds()) * 0.000001f;
	m_clock.reset();

	//printf("dt = %f: ",dt);
	
 {
	 	//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			dt = 1.0/420.f;

		int numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
		bool verbose = false;
		if (verbose)
		{
			if (!numSimSteps)
				printf("Interpolated transforms\n");
			else
			{
				if (numSimSteps > maxSimSubSteps)
				{
					//detect dropping frames
					printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
				} else
				{
					printf("Simulated (%i) steps\n",numSimSteps);
				}
			}
		}
 }
	renderme();

	drawLimit();

    glFlush();
    glutSwapBuffers();
}




void ConstraintDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	
	m_dynamicsWorld->updateAabbs();
	
	drawLimit();

	renderme();

    glFlush();
    glutSwapBuffers();
}


