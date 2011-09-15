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



#define CUBE_HALF_EXTENTS 0.5
#define EXTRA_HEIGHT 1.f

#include "CcdPhysicsDemo.h"
#include "GlutStuff.h"
#include "GLDebugFont.h"

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"

#if 0
extern btAlignedObjectArray<btVector3> debugContacts;
extern btAlignedObjectArray<btVector3> debugNormals;
#endif 

static GLDebugDrawer	sDebugDrawer;


CcdPhysicsDemo::CcdPhysicsDemo()
:m_ccdMode(USE_CCD)
{
	setDebugMode(btIDebugDraw::DBG_DrawText+btIDebugDraw::DBG_NoHelpText);
	setCameraDistance(btScalar(20.));
}


void CcdPhysicsDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(1./60.);//ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

	displayText();
#if 0
	for (int i=0;i<debugContacts.size();i++)
	{
		getDynamicsWorld()->getDebugDrawer()->drawContactPoint(debugContacts[i],debugNormals[i],0,0,btVector3(1,0,0));
	}
#endif

	glFlush();

	swapBuffers();

}


void CcdPhysicsDemo::displayText()
{
	int lineWidth=440;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if((getDebugMode() & btIDebugDraw::DBG_DrawText)!=0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		glRasterPos3f(xStart, yStart, 0);
		switch (m_ccdMode)
		{
		case USE_CCD:
			{
				sprintf(buf,"Predictive contacts and motion clamping");
				break;
			}
		case USE_NO_CCD:
			{
				sprintf(buf,"CCD handling disabled");
				break;
			}
		default:
			{
				sprintf(buf,"unknown CCD setting");
			};
		};

		GLDebugDrawString(xStart,20,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"Press 'p' to change CCD mode");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"Press '.' or right mouse to shoot bullets");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"space to restart, h(elp), t(ext), w(ire)");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		
		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}	

}



void CcdPhysicsDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	displayText();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->debugDrawWorld();
	}
#if 0
	for (int i=0;i<debugContacts.size();i++)
	{
		getDynamicsWorld()->getDebugDrawer()->drawContactPoint(debugContacts[i],debugNormals[i],0,0,btVector3(1,0,0));
	}
#endif

	glFlush();
	swapBuffers();
}





void	CcdPhysicsDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);
	
	m_ShootBoxInitialSpeed = 4000.f;


	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
//	m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,m_collisionConfiguration->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE,CONVEX_SHAPE_PROXYTYPE));

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld ->setDebugDrawer(&sDebugDrawer);
	m_dynamicsWorld->getSolverInfo().m_splitImpulse=true;
	m_dynamicsWorld->getSolverInfo().m_numIterations = 20;


	if (m_ccdMode==USE_CCD)
	{
		m_dynamicsWorld->getDispatchInfo().m_useContinuous=true;
	} else
	{
		m_dynamicsWorld->getDispatchInfo().m_useContinuous=false;
	}

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btBoxShape* box = new btBoxShape(btVector3(btScalar(110.),btScalar(1.),btScalar(110.)));
//	box->initializePolyhedralFeatures();
	btCollisionShape* groundShape = box;

//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);
	m_collisionShapes.push_back(new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));

	btTransform groundTransform;
	groundTransform.setIdentity();
	//groundTransform.setOrigin(btVector3(5,5,5));

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
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));

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

		int gNumObjects = 120;
		int i;
		for (i=0;i<gNumObjects;i++)
		{
			btCollisionShape* shape = m_collisionShapes[1];
			
			btTransform trans;
			trans.setIdentity();

			//stack them
			int colsize = 10;
			int row = (i*CUBE_HALF_EXTENTS*2)/(colsize*2*CUBE_HALF_EXTENTS);
			int row2 = row;
			int col = (i)%(colsize)-colsize/2;


			if (col>3)
			{
				col=11;
				row2 |=1;
			}

			btVector3 pos(col*2*CUBE_HALF_EXTENTS + (row2%2)*CUBE_HALF_EXTENTS,
				row*2*CUBE_HALF_EXTENTS+CUBE_HALF_EXTENTS+EXTRA_HEIGHT,0);

			trans.setOrigin(pos);
	
			float mass = 1.f;

			btRigidBody* body = localCreateRigidBody(mass,trans,shape);

	
			///when using m_ccdMode
			if (m_ccdMode==USE_CCD)
			{
				body->setCcdMotionThreshold(CUBE_HALF_EXTENTS);
				body->setCcdSweptSphereRadius(0.9*CUBE_HALF_EXTENTS);
			}
		}
	}

}

void	CcdPhysicsDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void CcdPhysicsDemo::keyboardCallback(unsigned char key, int x, int y)
{
	if (key=='p')
	{
		switch (m_ccdMode)
		{
			case USE_CCD:
			{
				m_ccdMode = USE_NO_CCD;
				break;
			}
			case USE_NO_CCD:
			default:
			{
				m_ccdMode = USE_CCD;
			}
		};
		clientResetScene();
	} else
	{
		DemoApplication::keyboardCallback(key,x,y);
	}
}


void	CcdPhysicsDemo::shootBox(const btVector3& destination)
{

	if (m_dynamicsWorld)
	{
		float mass = 1.f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);

		setShootBoxShape ();


		btRigidBody* body = this->localCreateRigidBody(mass, startTransform,m_shootBoxShape);
		body->setLinearFactor(btVector3(1,1,1));
		//body->setRestitution(1);

		btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=m_ShootBoxInitialSpeed;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
		body->setContactProcessingThreshold(1e30);

		///when using m_ccdMode, disable regular CCD
		if (m_ccdMode==USE_CCD)
		{
			body->setCcdMotionThreshold(CUBE_HALF_EXTENTS);
			body->setCcdSweptSphereRadius(0.4f);
		}
		
	}
}




void	CcdPhysicsDemo::exitPhysics()
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




