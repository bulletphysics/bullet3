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
#define PRINT_CONTACT_STATISTICS 1
#define USE_PARALLEL_DISPATCHER 1


//#define USE_SIMPLE_DYNAMICS_WORLD 1

int gNumObjects = 5;
#define HALF_EXTENTS btScalar(1.)
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/BroadphaseCollision/btMultiSapBroadphase.h"
#include "LinearMath/btIDebugDraw.h"
#include <stdio.h> //printf debugging
btScalar deltaTime = btScalar(1./60.);
btScalar gCollisionMargin = btScalar(0.05);
#include "BasicDemo2.h"

#ifdef USE_PARALLEL_DISPATCHER
#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#include "BulletMultiThreaded/SpuLibspe2Support.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif//USE_PARALLEL_DISPATCHER


#include <LinearMath/btAlignedObjectArray.h>

////////////////////////////////////




int main(int argc,char** argv)
{

	BasicDemo ccdDemo;
	ccdDemo.initPhysics();

	int i;
	for (i=0;i<5;i++)
		ccdDemo.clientMoveAndDisplay();
	ccdDemo.exitPhysics();
	
	return 0;
}



extern int gNumManifold;

void BasicDemo::clientMoveAndDisplay()
{

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = m_clock.getTimeMicroseconds();
	m_clock.reset();
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);

	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num contact manifolds: %i\n",gNumManifold);
	 int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
        for (int i=0;i<numManifolds;i++)
        {
                btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
                btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
                btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

                int numContacts = contactManifold->getNumContacts();
                for (int j=0;j<numContacts;j++)
                {
			  btManifoldPoint& pt = contactManifold->getContactPoint(j);
                        btVector3 ptA = pt.getPositionWorldOnA();
                        btVector3 ptB = pt.getPositionWorldOnB();
			printf("contact manifold[%d],pointA[%d]=(%f,%f,%f)\n",i,j,ptA[0],ptA[1],ptA[2]);
		}
	}
#endif //PRINT_CONTACT_STATISTICS


}





void	BasicDemo::initPhysics()
{

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

#ifdef USE_PARALLEL_DISPATCHER

	int maxNumOutstandingTasks = 1;//number of maximum outstanding tasks
#ifdef USE_WIN32_THREADING

	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#else
	
	spe_program_handle_t * program_handle;
#ifndef USE_CESOF
		char* spuFileName = "../../../src/BulletMultiThreaded/out/spuCollision.elf";

			program_handle = spe_image_open (spuFileName);
			if (program_handle == NULL)
		    {
				printf( "SPU OPEN IMAGE ERROR:%s\n",spuFileName);
				exit(0);
		    }
			else
			{
				printf( "IMAGE OPENED:%s\n",spuFileName);
			}
#else
			extern spe_program_handle_t spu_program;
			program_handle = &spu_program;
#endif			
	SpuLibspe2Support* threadSupport = new SpuLibspe2Support( program_handle, maxNumOutstandingTasks);	
	
#endif // WIN32


	m_dispatcher = new	SpuGatheringCollisionDispatcher(threadSupport,maxNumOutstandingTasks,collisionConfiguration);
#else
	m_dispatcher = new	btCollisionDispatcher(collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER
	m_collisionConfiguration = new btDefaultCollisionConfiguration();


#define USE_SWEEP_AND_PRUNE 1
#ifdef USE_SWEEP_AND_PRUNE
#define maxProxies 8192
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_overlappingPairCache = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
	//m_overlappingPairCache = new btMultiSapBroadphase();


#else
	m_overlappingPairCache = new btSimpleBroadphase;
#endif //USE_SWEEP_AND_PRUNE



	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;
	

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_solver,m_collisionConfiguration);
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
	groundTransform.setOrigin(btVector3(0,-50,0));
	localCreateRigidBody(btScalar(0.),groundTransform,groundShape);

	//create a few dynamic sphere rigidbodies (re-using the same sphere shape)
	//btCollisionShape* sphereShape = new btBoxShape(btVector3(1,1,1));
	btCollisionShape* sphereShape = new btSphereShape(btScalar(1.));
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
		//btRigidBody* body = localCreateRigidBody(btScalar(1.),trans,sphereShape);
		localCreateRigidBody(btScalar(1.),trans,sphereShape);
	}

	//clientResetScene();
}
	
btRigidBody* BasicDemo::localCreateRigidBody(btScalar mass,const btTransform& startTrans,btCollisionShape* colShape)
{
btVector3 inertia(0,0,0);
if (mass)
	colShape->calculateLocalInertia(mass,inertia);
	btRigidBody::btRigidBodyConstructionInfo rbci(mass,0,colShape,inertia);
	rbci.m_startWorldTransform = startTrans;

btRigidBody* body = new btRigidBody(rbci);
	m_dynamicsWorld->addRigidBody(body);
return body;

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




