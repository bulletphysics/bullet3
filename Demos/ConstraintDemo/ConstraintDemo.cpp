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

#include "CcdPhysicsEnvironment.h"
#include "CcdPhysicsController.h"
#include "MyMotionState.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"

#include "PHY_Pro.h"
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

	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher();
		
	btOverlappingPairCache* broadphase = new btSimpleBroadphase();


	m_physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
	m_physicsEnvironmentPtr->setDeactivationTime(0.f);
	m_physicsEnvironmentPtr->setGravity(0,-10,0);


	btCollisionShape* shape = new btBoxShape(btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
	btTransform trans;
	trans.setIdentity();
	trans.setOrigin(btVector3(0,20,0));

	bool isDynamic = false;
	float mass = 1.f;

	CcdPhysicsController* ctrl0 = localCreatePhysicsObject( isDynamic,mass,trans,shape);
	trans.setOrigin(btVector3(2*CUBE_HALF_EXTENTS,20,0));
	isDynamic = true;
	CcdPhysicsController* ctrl1 = localCreatePhysicsObject( isDynamic,mass,trans,shape);
	
	
	clientResetScene();

	{
		int constraintId;

			float pivotX=CUBE_HALF_EXTENTS,
				pivotY=-CUBE_HALF_EXTENTS,
				pivotZ=-CUBE_HALF_EXTENTS;
			float axisX=0,axisY=0,axisZ=1;


		constraintId =m_physicsEnvironmentPtr->createConstraint(
		ctrl0,
		ctrl1,
			PHY_POINT2POINT_CONSTRAINT,
			//PHY_GENERIC_6DOF_CONSTRAINT,//can leave any of the 6 degree of freedom 'free' or 'locked'
			//PHY_LINEHINGE_CONSTRAINT,
			pivotX,pivotY,pivotZ,
			axisX,axisY,axisZ
			);

	}
}


void ConstraintDemo::clientMoveAndDisplay()
{
	
	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float deltaTime = 1.f/60.f;

	m_physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);
	
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


