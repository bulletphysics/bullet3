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


//#define USE_GROUND_BOX 1
//#define PRINT_CONTACT_STATISTICS 1
//#define CHECK_MEMORY_LEAKS 1

int gNumObjects = 120;
#define HALF_EXTENTS 1.f
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging
float deltaTime = 1.f/60.f;
float	gCollisionMargin = 0.05f;
#include "BasicDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"
////////////////////////////////////

GLDebugDrawer debugDrawer;

int main(int argc,char** argv)
{

	BasicDemo ccdDemo;
	ccdDemo.initPhysics();
	ccdDemo.setCameraDistance(50.f);

#ifdef CHECK_MEMORY_LEAKS
	ccdDemo.exitPhysics();
#else
	return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bullet.sf.net",&ccdDemo);
#endif
	
	//default glut doesn't return from mainloop
	return 0;
}



extern int gNumManifold;
extern int gOverlappingPairs;
extern int gTotalContactPoints;

void BasicDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = m_clock.getTimeMicroseconds();
	m_clock.reset();
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		
	renderme(); 

	glFlush();
	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num manifolds: %i\n",gNumManifold);
	printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	printf("num gTotalContactPoints : %i\n",gTotalContactPoints );
#endif //PRINT_CONTACT_STATISTICS

	gTotalContactPoints = 0;
	glutSwapBuffers();

}



void BasicDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	if (m_dynamicsWorld)
		m_dynamicsWorld->updateAabbs();
	
	renderme();

	glFlush();
	glutSwapBuffers();
}




void	BasicDemo::initPhysics()
{


	m_dispatcher = new	btCollisionDispatcher(true);
	
#ifdef USE_SWEEP_AND_PRUNE
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_overlappingPairCache = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
#else
	m_overlappingPairCache = new btSimpleBroadphase;
#endif //USE_SWEEP_AND_PRUNE

	m_sphereSphereCF = new btSphereSphereCollisionAlgorithm::CreateFunc;
	m_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,m_sphereSphereCF);


	m_sphereBoxCF = new btSphereBoxCollisionAlgorithm::CreateFunc;
	m_boxSphereCF = new btSphereBoxCollisionAlgorithm::CreateFunc;
	m_boxSphereCF->m_swapped = true;
	m_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,m_sphereBoxCF);
	m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,m_boxSphereCF);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btSimpleDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_solver);
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	m_dynamicsWorld->setDebugDrawer(&debugDrawer);

	///create a few basic rigid bodies


	//static ground
#ifdef USE_GROUND_BOX
	btCollisionShape* groundShape = new btBoxShape(btVector3(50.f,50.f,50.f));
#else
	btCollisionShape* groundShape = new btSphereShape(50.f);
#endif//USE_GROUND_BOX

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));
	localCreateRigidBody(0.f,groundTransform,groundShape);

	//create a few dynamic sphere rigidbodies (re-using the same sphere shape)
	btCollisionShape* sphereShape = new btSphereShape(1.f);
	m_collisionShapes.push_back(sphereShape);

	int i;
	for (i=0;i<gNumObjects;i++)
	{
		
		sphereShape->setMargin(gCollisionMargin);
		btTransform trans;
		trans.setIdentity();
		//stack them
		int colsize = 2;
		int row = (int)((i*HALF_EXTENTS*2)/(colsize*2*HALF_EXTENTS));
		int row2 = row;
		int col = (i)%(colsize)-colsize/2;
		btVector3 pos(col*2*HALF_EXTENTS + (row2%2)*HALF_EXTENTS,
			row*2*HALF_EXTENTS+HALF_EXTENTS,0);

		trans.setOrigin(pos);
		//btRigidBody* body = localCreateRigidBody(1.f,trans,sphereShape);
		localCreateRigidBody(1.f,trans,sphereShape);
	}

	clientResetScene();
}
	

void	BasicDemo::exitPhysics()
{


	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (unsigned int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete collision algorithms creation functions
	delete m_sphereSphereCF;
	

	delete m_sphereBoxCF;
	delete m_boxSphereCF;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	
}




