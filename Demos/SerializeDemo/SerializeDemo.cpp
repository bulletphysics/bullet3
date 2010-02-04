/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#define TEST_SERIALIZATION 1

#ifdef BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES
#define CREATE_NEW_BULLETFILE 1
#endif //BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "SerializeDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#ifdef TEST_SERIALIZATION
#include "LinearMath/btSerializer.h"
#include "btBulletFile.h"
#include "btBulletWorldImporter.h"




#endif //TEST_SERIALIZATION

#include <stdio.h> //printf debugging


void SerializeDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

	glFlush();

	swapBuffers();

}



void SerializeDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}



void	SerializeDemo::setupEmptyDynamicsWorld()
{
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

}

void	SerializeDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*50.));

	setupEmptyDynamicsWorld();
	
	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	btCollisionObject* groundObject = 0;

	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
		groundObject = body;
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		int numSpheres = 2;
		btVector3 positions[2] = {btVector3(0.1f,0.2f,0.3f),btVector3(0.4f,0.5f,0.6f)};
		btScalar	radii[2] = {0.3f,0.4f};

		btMultiSphereShape* colShape = new btMultiSphereShape(positions,radii,numSpheres);

		//btCollisionShape* colShape = new btCapsuleShapeZ(SCALING*1,SCALING*1);
		//btCollisionShape* colShape = new btCylinderShapeZ(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
										btScalar(2.0*i + start_x),
										btScalar(20+2.0*k + start_y),
										btScalar(2.0*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					
					body->setActivationState(ISLAND_SLEEPING);

					m_dynamicsWorld->addRigidBody(body);
					body->setActivationState(ISLAND_SLEEPING);
				}
			}
		}
	}


	clientResetScene();


#ifdef TEST_SERIALIZATION
	//test serializing this 

#ifdef CREATE_NEW_BULLETFILE
	int maxSerializeBufferSize = 1024*1024*5;

	btDefaultSerializer*	serializer = new btDefaultSerializer(maxSerializeBufferSize);

	static char* groundName = "GroundName";
	serializer->registerNameForPointer(groundObject, groundName);

	for (int i=0;i<m_collisionShapes.size();i++)
	{
		char* name = new char[20];
		
		sprintf(name,"name%d",i);
		serializer->registerNameForPointer(m_collisionShapes[i],name);
	}

	btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*(btRigidBody*)getDynamicsWorld()->getCollisionObjectArray()[2],btVector3(0,1,0));
	m_dynamicsWorld->addConstraint(p2p);
	
	const char* name = "constraintje";
	serializer->registerNameForPointer(p2p,name);

	m_dynamicsWorld->serialize(serializer);
	
	FILE* f2 = fopen("testFile.bullet","wb");
	fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1,f2);
	fclose(f2);
#endif //CREATE_NEW_BULLETFILE

	exitPhysics();

	//now try again from the loaded file
	setupEmptyDynamicsWorld();

	printf("loading file\n");
	btBulletWorldImporter* fileLoader = new btBulletWorldImporter(m_dynamicsWorld);
	//fileLoader->setVerboseMode(true);
	
	if (fileLoader->loadFile("testFile.bullet"))
	{
		printf("loaded fine\n");
	}



#endif //TEST_SERIALIZATION

}
	

void	SerializeDemo::exitPhysics()
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

	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




