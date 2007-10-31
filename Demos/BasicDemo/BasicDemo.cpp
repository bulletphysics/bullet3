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


#define USE_GROUND_BOX 1
//#define PRINT_CONTACT_STATISTICS 1
//#define USE_PARALLEL_DISPATCHER 1

#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3


///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5


//#define USE_SIMPLE_DYNAMICS_WORLD 1

int gNumObjects = 120;
#define HALF_EXTENTS btScalar(1.)
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/BroadphaseCollision/btMultiSapBroadphase.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging
btScalar deltaTime = btScalar(1./60.);
#include "BasicDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"

#ifdef USE_PARALLEL_DISPATCHER
#include "../../Extras/BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "../../Extras/BulletMultiThreaded/Win32ThreadSupport.h"
#include "../../Extras/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif//USE_PARALLEL_DISPATCHER


#include <LinearMath/btAlignedObjectArray.h>

////////////////////////////////////


class myTest
{

};







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

	setCameraDistance(btScalar(50.));


	m_collisionConfiguration = new btDefaultCollisionConfiguration();

#ifdef USE_PARALLEL_DISPATCHER

#ifdef USE_WIN32_THREADING

	int maxNumOutstandingTasks = 4;//number of maximum outstanding tasks
	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#else
///todo other platform threading
///Playstation 3 SPU (SPURS)  version is available through PS3 Devnet
///Libspe2 SPU support will be available soon
///pthreads version
///you can hook it up to your custom task scheduler by deriving from btThreadSupportInterface
#endif


	m_dispatcher = new	SpuGatheringCollisionDispatcher(threadSupport,maxNumOutstandingTasks,collisionConfiguration);
#else
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER

#define USE_SWEEP_AND_PRUNE 1
#ifdef USE_SWEEP_AND_PRUNE
#define maxProxies 8192
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_overlappingPairCache = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);



#else
	m_overlappingPairCache = new btSimpleBroadphase;
#endif //USE_SWEEP_AND_PRUNE



	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;
	

#ifdef USE_SIMPLE_DYNAMICS_WORLD
	//btSimpleDynamicsWorld doesn't support 'cache friendly' optimization, so disable this
	sol->setSolverMode(btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
	m_dynamicsWorld = new btSimpleDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_solver,m_collisionConfiguration);
#else
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_solver,m_collisionConfiguration);
#endif //USE_SIMPLE_DYNAMICS_WORLD
	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));


	///create a few basic rigid bodies


	//static ground
#ifdef USE_GROUND_BOX
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
#else
	btCollisionShape* groundShape = new btSphereShape(btScalar(50.));
#endif//USE_GROUND_BOX

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-56,0));
	localCreateRigidBody(btScalar(0.),groundTransform,groundShape);

	//create a few dynamic sphere rigidbodies (re-using the same sphere shape)
	//btCollisionShape* sphereShape = new btBoxShape(btVector3(1,1,1));
	btCollisionShape* colShape = new btSphereShape(btScalar(1.));
	m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	{

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(btVector3(
										2.0*i + start_x,
										2.0*k + start_y,
										2.0*j + start_z));

					localCreateRigidBody(1, startTransform,colShape);
				}
			}
		}
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
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




