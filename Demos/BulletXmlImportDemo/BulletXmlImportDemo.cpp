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



#include "BulletXmlImportDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btSerializer.h"
#include "btBulletFile.h"
#include "btBulletWorldImporter.h"
#include "btBulletXmlWorldImporter.h"

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include <stdio.h> //printf debugging



#include "GLDebugDrawer.h"
GLDebugDrawer	gDebugDrawer;


void BulletXmlImportDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	

	setupEmptyDynamicsWorld();

	
	m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);


	btBulletXmlWorldImporter* importer = new btBulletXmlWorldImporter(m_dynamicsWorld);
	importer->loadFile("bullet_basic.xml");
//	importer->loadFile("bulletser.xml");
//	importer->loadFile("bullet_constraints.xml");

}

void BulletXmlImportDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
	

		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

	glFlush();

	swapBuffers();

}



void BulletXmlImportDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}


void	BulletXmlImportDemo::setupEmptyDynamicsWorld()
{
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

	m_broadphase = new btDbvtBroadphase();
	
	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	//btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher*)m_dynamicsWorld->getDispatcher());

	

}




BulletXmlImportDemo::~BulletXmlImportDemo()
{
	m_fileLoader->deleteAllData();
	delete m_fileLoader;	
	exitPhysics();
}

void BulletXmlImportDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}


void	BulletXmlImportDemo::exitPhysics()
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




