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


///
/// CollisionInterfaceDemo shows high level usage of the Collision Detection.
///
#define TEST_NOT_ADDING_OBJECTS_TO_WORLD

#include "GL_Simplex1to4.h"

//include common Bullet Collision Detection headerfiles
#include "btBulletCollisionCommon.h"

#include "LinearMath/btIDebugDraw.h"
#include "GL_ShapeDrawer.h"
#include "CollisionInterfaceDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"

btScalar yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;

btCollisionObject	objects[maxNumObjects];
btCollisionWorld*	collisionWorld = 0;

GLDebugDrawer debugDrawer;


void	CollisionInterfaceDemo::initPhysics()
{
			
	m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
	
	btMatrix3x3 basisA;
	basisA.setIdentity();

	btMatrix3x3 basisB;
	basisB.setIdentity();

	objects[0].getWorldTransform().setBasis(basisA);
	objects[1].getWorldTransform().setBasis(basisB);

	btBoxShape* boxA = new btBoxShape(btVector3(1,1,1));
	boxA->setMargin(0.f);

	btBoxShape* boxB = new btBoxShape(btVector3(0.5,0.5,0.5));
	boxB->setMargin(0.f);
	//ConvexHullShape	hullA(points0,3);
	//hullA.setLocalScaling(btVector3(3,3,3));
	//ConvexHullShape	hullB(points1,4);
	//hullB.setLocalScaling(btVector3(4,4,4));

	objects[0].setCollisionShape(boxA);//&hullA;
	objects[1].setCollisionShape(boxB);//&hullB;

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btVector3	worldAabbMin(-1000,-1000,-1000);
	btVector3	worldAabbMax(1000,1000,1000);

	btAxisSweep3*	broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);
	
	//SimpleBroadphase is a brute force alternative, performing N^2 aabb overlap tests
	//SimpleBroadphase*	broadphase = new btSimpleBroadphase;

	collisionWorld = new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);
	collisionWorld->setDebugDrawer(&debugDrawer);
		
#ifdef TEST_NOT_ADDING_OBJECTS_TO_WORLD
//	collisionWorld->addCollisionObject(&objects[0]);
	collisionWorld->addCollisionObject(&objects[1]);
#endif //TEST_NOT_ADDING_OBJECTS_TO_WORLD

}


//to be implemented by the demo

void CollisionInterfaceDemo::clientMoveAndDisplay()
{
	
	displayCallback();
}


static btVoronoiSimplexSolver sGjkSimplexSolver;
btSimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;

struct btDrawingResult : public btCollisionWorld::ContactResultCallback
{
	virtual	btScalar	addSingleResult(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
	{

		glBegin(GL_LINES);
		glColor3f(0, 0, 0);

		btVector3 ptA = cp.getPositionWorldOnA();
		btVector3 ptB = cp.getPositionWorldOnB();

		glVertex3d(ptA.x(),ptA.y(),ptA.z());
		glVertex3d(ptB.x(),ptB.y(),ptB.z());
		glEnd();

		return 0;
	}
};

void CollisionInterfaceDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);

		btScalar m[16];
	
	btVector3	worldBoundsMin,worldBoundsMax;
	collisionWorld->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);


	int i;
	for (i=0;i<numObjects;i++)
	{
		
		objects[i].getWorldTransform().getOpenGLMatrix( m );
		m_shapeDrawer->drawOpenGL(m,objects[i].getCollisionShape(),btVector3(1,1,1),getDebugMode(),worldBoundsMin,worldBoundsMax);
	}

	collisionWorld->getDispatchInfo().m_debugDraw = &debugDrawer;
	
	if (collisionWorld)
		collisionWorld->performDiscreteCollisionDetection();

	
	


#ifndef TEST_NOT_ADDING_OBJECTS_TO_WORLD
	
	collisionWorld->debugDrawWorld();
	///one way to draw all the contact points is iterating over contact manifolds in the dispatcher:

	int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();
	for (i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
	
		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);

			glBegin(GL_LINES);
			glColor3f(0, 0, 0);
			
			btVector3 ptA = pt.getPositionWorldOnA();
			btVector3 ptB = pt.getPositionWorldOnB();

			glVertex3d(ptA.x(),ptA.y(),ptA.z());
			glVertex3d(ptB.x(),ptB.y(),ptB.z());
			glEnd();
		}

		//you can un-comment out this line, and then all points are removed
		//contactManifold->clearManifold();	
	}
#else


	glDisable(GL_TEXTURE_2D);
	for (i=0;i<numObjects;i++)
	{
		collisionWorld->debugDrawObject(objects[i].getWorldTransform(),objects[i].getCollisionShape(), btVector3(1,1,0));
	}

	btDrawingResult renderCallback;

	//collisionWorld->contactPairTest(&objects[0],&objects[1], renderCallback);
	collisionWorld->contactTest(&objects[0],renderCallback);
	
#if 0

	//another way is to directly query the dispatcher for both objects. The objects don't need to be inserted into the world

	btCollisionAlgorithm* algo = collisionWorld->getDispatcher()->findAlgorithm(&objects[0],&objects[1]);
	btManifoldResult contactPointResult(&objects[0],&objects[1]);
	algo->processCollision(&objects[0],&objects[1],collisionWorld->getDispatchInfo(),&contactPointResult);
	
	btManifoldArray manifoldArray;
	algo->getAllContactManifolds(manifoldArray);

	int numManifolds = manifoldArray.size();
	for (i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = manifoldArray[i];
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
	//	btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
	
		glDisable(GL_DEPTH_TEST);
		int numContacts = contactManifold->getNumContacts();
		bool swap = obA == &objects[0];

		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
		
			glBegin(GL_LINES);
			glColor3f(0, 0, 0);

			btVector3 ptA = swap ?pt.getPositionWorldOnA():pt.getPositionWorldOnB();
			btVector3 ptB = swap ? pt.getPositionWorldOnB():pt.getPositionWorldOnA();

			glVertex3d(ptA.x(),ptA.y(),ptA.z());
			glVertex3d(ptB.x(),ptB.y(),ptB.z());
			glEnd();
		}

		//you can un-comment out this line, and then all points are removed
		//contactManifold->clearManifold();	
	}
#endif


#endif
	




	//GL_ShapeDrawer::drawCoordSystem();


	btQuaternion qA = objects[0].getWorldTransform().getRotation();
	btQuaternion qB = objects[1].getWorldTransform().getRotation();


	if (!m_idle)
	{

		
		btScalar timeInSeconds = getDeltaTimeMicroseconds()/1000.f;

		btQuaternion orn;

		objects[0].getWorldTransform().getBasis().getEulerYPR(yaw,pitch,roll);
		pitch += 0.00005f*timeInSeconds;
		yaw += 0.0001f*timeInSeconds;
		objects[0].getWorldTransform().getBasis().setEulerYPR(yaw,pitch,roll);

		orn.setEuler(yaw,pitch,roll);
		objects[1].getWorldTransform().setOrigin(objects[1].getWorldTransform().getOrigin()+btVector3(0,-0.00001*timeInSeconds,0));

		//objects[0].getWorldTransform().setRotation(orn);

		
		
	}

	glFlush();
    swapBuffers();
}

void CollisionInterfaceDemo::clientResetScene()
{
	objects[0].getWorldTransform().setOrigin(btVector3(0.0f,3.f,0.f));
	
	btQuaternion rotA(0.739f,-0.204f,0.587f,0.257f);
	rotA.normalize();

	objects[0].getWorldTransform().setRotation(rotA);

	objects[1].getWorldTransform().setOrigin(btVector3(0.0f,4.248f,0.f));
	
}


