/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2011 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///FractureDemo shows how to break objects.
///It assumes a btCompoundShaps (where the childshapes are the pre-fractured pieces)
///The btFractureBody is a class derived from btRigidBody, dealing with the collision impacts.
///Press the F key to toggle between fracture and glue mode
///This is preliminary work


#define CUBE_HALF_EXTENTS 1.f
#define EXTRA_HEIGHT 1.f
///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "FractureDemo.h"
#include "GlutStuff.h"
#include "GLDebugFont.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"


#include <stdio.h> //printf debugging


int sFrameNumber = 0;

#include "btFractureBody.h"
#include "btFractureDynamicsWorld.h"





void	FractureDemo::initPhysics()
{

	setTexturing(true);
	setShadows(true);

	setDebugMode(btIDebugDraw::DBG_DrawText|btIDebugDraw::DBG_NoHelpText);

	setCameraDistance(btScalar(SCALING*20.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	//m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	btFractureDynamicsWorld* fractureWorld = new btFractureDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = fractureWorld;

	m_ShootBoxInitialSpeed=100; 

	//m_splitImpulse removes the penetration resolution from the applied impulse, otherwise objects might fracture due to deep penetrations.
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;

	{
		///create a few basic rigid bodies
		btCollisionShape* groundShape = new btBoxShape(btVector3(50,1,50));
	///	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),0);
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,0,0));
		localCreateRigidBody(0.f,groundTransform,groundShape);
	}

	{
		///create a few basic rigid bodies
		btCollisionShape* shape = new btBoxShape(btVector3(1,1,1));
		m_collisionShapes.push_back(shape);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(5,2,0));
		localCreateRigidBody(0.f,tr,shape);
	}



	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btCapsuleShape(SCALING*0.4,SCALING*1);
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


		int gNumObjects = 10;

		for (int i=0;i<gNumObjects;i++)
		{
			btTransform trans;
			trans.setIdentity();

			btVector3 pos(i*2*CUBE_HALF_EXTENTS ,20,0);
			trans.setOrigin(pos);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
			btFractureBody* body = new btFractureBody(rbInfo, m_dynamicsWorld);
			body->setLinearVelocity(btVector3(0,-10,0));

			m_dynamicsWorld->addRigidBody(body);


		}

	}



	fractureWorld->stepSimulation(1./60.,0);
	fractureWorld->glueCallback();



}

void	FractureDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}


void FractureDemo::clientMoveAndDisplay()
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

	showMessage();

	glFlush();

	swapBuffers();

}

void FractureDemo::showMessage()
{
	if((getDebugMode() & btIDebugDraw::DBG_DrawText))
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];

		int lineWidth=380;
		int xStart = m_glutScreenWidth - lineWidth;
		int yStart = 20;

		btFractureDynamicsWorld* world = (btFractureDynamicsWorld*)m_dynamicsWorld;
		if (world->getFractureMode())
		{
			sprintf(buf,"Fracture mode");
		} else
		{
			sprintf(buf,"Glue mode");
		}
		GLDebugDrawString(xStart,yStart,buf);
		sprintf(buf,"f to toggle fracture/glue mode");		
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		sprintf(buf,"space to restart, mouse to pick/shoot");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);

		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}

}


void FractureDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	showMessage();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}


void FractureDemo::keyboardUpCallback(unsigned char key, int x, int y)
{
	if (key=='f')
	{
		btFractureDynamicsWorld* world = (btFractureDynamicsWorld*)m_dynamicsWorld;
		world->setFractureMode(!world->getFractureMode());
	}

	PlatformDemoApplication::keyboardUpCallback(key,x,y);

}


void	FractureDemo::shootBox(const btVector3& destination)
{

	if (m_dynamicsWorld)
	{
		btScalar mass = 1.f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);

		setShootBoxShape ();

		btAssert((!m_shootBoxShape || m_shootBoxShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			m_shootBoxShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btFractureBody* body = new btFractureBody(mass,0,m_shootBoxShape,localInertia,&mass,1,m_dynamicsWorld);

		body->setWorldTransform(startTransform);

		m_dynamicsWorld->addRigidBody(body);


		body->setLinearFactor(btVector3(1,1,1));
		//body->setRestitution(1);

		btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=m_ShootBoxInitialSpeed;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
		body->setCcdMotionThreshold(1.);
		body->setCcdSweptSphereRadius(0.2f);

	}
}






void	FractureDemo::exitPhysics()
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
	m_dynamicsWorld=0;

	delete m_solver;
	m_solver=0;

	delete m_broadphase;
	m_broadphase=0;

	delete m_dispatcher;
	m_dispatcher=0;

	delete m_collisionConfiguration;
	m_collisionConfiguration=0;

}




