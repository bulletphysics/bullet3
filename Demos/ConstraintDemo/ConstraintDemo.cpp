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

	constraintDemo->setCameraDistance(46.f);

	return glutmain(argc, argv,640,480,"Constraint Demo. http://www.continuousphysics.com/Bullet/phpBB2/",constraintDemo);
}




void	ConstraintDemo::initPhysics()
{
	//ConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	//ConstraintSolver* solver = new OdeConstraintSolver;
	//btCollisionDispatcher* dispatcher = new btCollisionDispatcher();
	//btOverlappingPairCache* broadphase = new btSimpleBroadphase();

	m_dynamicsWorld = new btDiscreteDynamicsWorld();

	btCollisionShape* shape = new btBoxShape(btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
	btTransform trans;
	trans.setIdentity();
	trans.setOrigin(btVector3(0,20,0));

	float mass = 0.f;
	btRigidBody* body0 = localCreateRigidBody( mass,trans,shape);
	trans.setOrigin(btVector3(2*CUBE_HALF_EXTENTS,20,0));

	mass = 1.f;
	btRigidBody* body1 = localCreateRigidBody( mass,trans,shape);
	body1->setDamping(0.3,0.3);

	
	clientResetScene();

	{
		btVector3 pivotInA(CUBE_HALF_EXTENTS,-CUBE_HALF_EXTENTS,-CUBE_HALF_EXTENTS);
		btVector3 axisInA(0,0,1);

		btVector3 pivotInB = body1 ? body1->getCenterOfMassTransform().inverse()(body0->getCenterOfMassTransform()(pivotInA)) : pivotInA;
		btVector3 axisInB = body1? 
			(body1->getCenterOfMassTransform().getBasis().inverse()*(body1->getCenterOfMassTransform().getBasis() * axisInA)) : 
		body0->getCenterOfMassTransform().getBasis() * axisInA;

		btTypedConstraint* p2p = new btPoint2PointConstraint(*body0,*body1,pivotInA,pivotInB);

		m_dynamicsWorld->addConstraint(p2p);

	}
}


void ConstraintDemo::clientMoveAndDisplay()
{
	
	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float deltaTime = 1.f/60.f;

	m_dynamicsWorld->stepSimulation(deltaTime);
	renderme();

    glFlush();
    glutSwapBuffers();
}




void ConstraintDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

    glFlush();
    glutSwapBuffers();
}


