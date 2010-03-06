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

#include "BulletCollision/CollisionShapes/btBox2dShape.h"
#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.h"
#include "GL_DialogDynamicsWorld.h"
#include "GL_DialogWindow.h"


#include "BulletCollision/CollisionShapes/btBox2dShape.h"
#include "BulletCollision/CollisionShapes/btConvex2dShape.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5 
#define ARRAY_SIZE_Z 1

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "Box2dDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include <stdio.h> //printf debugging


void Box2dDemo::clientMoveAndDisplay()
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

	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->draw(ms / 1000000.f);

	glFlush();

	swapBuffers();

}



void Box2dDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->draw(0.f);

	glFlush();
	swapBuffers();
}



void Box2dDemo::reshape(int w, int h)
{
	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->setScreenSize(w,h);
	PlatformDemoApplication::reshape(w,h);
}

void	Box2dDemo::initPhysics()
{
	
	m_dialogDynamicsWorld = new GL_DialogDynamicsWorld();

	//m_dialogDynamicsWorld->createDialog(100,110,200,50);
	//m_dialogDynamicsWorld->createDialog(100,00,100,100);
	//m_dialogDynamicsWorld->createDialog(0,0,100,100);
	GL_DialogWindow* settings = m_dialogDynamicsWorld->createDialog(50,0,200,120,"Settings");
	GL_ToggleControl* toggle = m_dialogDynamicsWorld->createToggle(settings,"Toggle 1");
	toggle = m_dialogDynamicsWorld->createToggle(settings,"Toggle 2");
	toggle ->m_active = true;
	toggle = m_dialogDynamicsWorld->createToggle(settings,"Toggle 3");
	GL_SliderControl* slider = m_dialogDynamicsWorld->createSlider(settings,"Slider");

	GL_DialogWindow* dialog = m_dialogDynamicsWorld->createDialog(0,200,420,300,"Help");
	GL_TextControl* txt = new GL_TextControl;
	dialog->addControl(txt);
	txt->m_textLines.push_back("Mouse to move");
	txt->m_textLines.push_back("Test 2");
	txt->m_textLines.push_back("mouse to interact");
	txt->m_textLines.push_back("ALT + mouse to move camera");
	txt->m_textLines.push_back("space to reset");
	txt->m_textLines.push_back("cursor keys and z,x to navigate");
	txt->m_textLines.push_back("i to toggle simulation, s single step");
	txt->m_textLines.push_back("q to quit");
	txt->m_textLines.push_back(". to shoot box");
	txt->m_textLines.push_back("d to toggle deactivation");
	txt->m_textLines.push_back("g to toggle mesh animation (ConcaveDemo)");
	txt->m_textLines.push_back("h to toggle help text");
	txt->m_textLines.push_back("o to toggle orthogonal/perspective view");
	//txt->m_textLines.push_back("+- shooting speed = %10.2f",m_ShootBoxInitialSpeed);

	
	
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*50.));
	m_cameraTargetPosition.setValue(0,0,0);//0, ARRAY_SIZE_Y, 0);

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	btVoronoiSimplexSolver* simplex = new btVoronoiSimplexSolver();
	btMinkowskiPenetrationDepthSolver* pdSolver = new btMinkowskiPenetrationDepthSolver();


	btConvex2dConvex2dAlgorithm::CreateFunc* convexAlgo2d = new btConvex2dConvex2dAlgorithm::CreateFunc(simplex,pdSolver);
	
	m_dispatcher->registerCollisionCreateFunc(CONVEX_2D_SHAPE_PROXYTYPE,CONVEX_2D_SHAPE_PROXYTYPE,convexAlgo2d);
	m_dispatcher->registerCollisionCreateFunc(BOX_2D_SHAPE_PROXYTYPE,CONVEX_2D_SHAPE_PROXYTYPE,convexAlgo2d);
	m_dispatcher->registerCollisionCreateFunc(CONVEX_2D_SHAPE_PROXYTYPE,BOX_2D_SHAPE_PROXYTYPE,convexAlgo2d);
	m_dispatcher->registerCollisionCreateFunc(BOX_2D_SHAPE_PROXYTYPE,BOX_2D_SHAPE_PROXYTYPE,new btBox2dBox2dCollisionAlgorithm::CreateFunc());

	m_broadphase = new btDbvtBroadphase();
	//m_broadphase = new btSimpleBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getSolverInfo().m_erp = 1.f;
	//m_dynamicsWorld->getSolverInfo().m_numIterations = 4;
	


	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.),btScalar(50.),btScalar(150.)));
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-43,0));

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

		btScalar u= btScalar(1*SCALING-0.04);
		btVector3 points[3] = {btVector3(0,u,0),btVector3(-u,-u,0),btVector3(u,-u,0)};
		btConvexShape* colShape= new btConvex2dShape(new btBoxShape(btVector3(btScalar(SCALING*1),btScalar(SCALING*1),btScalar(0.04))));
		//btCollisionShape* colShape = new btBox2dShape(btVector3(SCALING*1,SCALING*1,0.04));

		btConvexShape* colShape2= new btConvex2dShape(new btConvexHullShape(&points[0].getX(),3));
		btConvexShape* colShape3= new btConvex2dShape(new btCylinderShapeZ(btVector3(btScalar(SCALING*1),btScalar(SCALING*1),btScalar(0.04))));
		
		

		

		//btUniformScalingShape* colShape = new btUniformScalingShape(convexColShape,1.f);
		colShape->setMargin(btScalar(0.03));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);
		m_collisionShapes.push_back(colShape2);
		

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

//		float start_x = START_POS_X - ARRAY_SIZE_X/2;
//		float start_y = START_POS_Y;
//		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		btVector3 x(-ARRAY_SIZE_X, 8.0f,-20.f);
		btVector3 y;
		btVector3 deltaX(SCALING*1, SCALING*2,0.f);
		btVector3 deltaY(SCALING*2, 0.0f,0.f);

		for (int i = 0; i < ARRAY_SIZE_X; ++i)
		{
			y = x;

			for (int  j = i; j < ARRAY_SIZE_Y; ++j)
			{
					startTransform.setOrigin(y-btVector3(-10,0,0));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(0,0,0);
					switch (j%3)
					{
#if 1
					case 0:
						rbInfo = btRigidBody::btRigidBodyConstructionInfo(mass,myMotionState,colShape,localInertia);
						break;
					case 1:
						rbInfo = btRigidBody::btRigidBodyConstructionInfo(mass,myMotionState,colShape3,localInertia);
						break;
#endif
					default:
						rbInfo = btRigidBody::btRigidBodyConstructionInfo(mass,myMotionState,colShape2,localInertia);
					}
					btRigidBody* body = new btRigidBody(rbInfo);
					//body->setContactProcessingThreshold(colShape->getContactBreakingThreshold());
					body->setActivationState(ISLAND_SLEEPING);
					body->setLinearFactor(btVector3(1,1,0));
					body->setAngularFactor(btVector3(0,0,1));

					m_dynamicsWorld->addRigidBody(body);
					body->setActivationState(ISLAND_SLEEPING);


			//	y += -0.8*deltaY;
				y += deltaY;
			}

			x += deltaX;
		}

	}


	clientResetScene();
}
	

void	Box2dDemo::exitPhysics()
{
	delete m_dialogDynamicsWorld;
	m_dialogDynamicsWorld = 0;

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

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}

void Box2dDemo::mouseFunc(int button, int state, int x, int y)
{

	if (!m_dialogDynamicsWorld->mouseFunc(button,state,x,y))
	{
		DemoApplication::mouseFunc(button,state,x,y);
	}
}

void	Box2dDemo::mouseMotionFunc(int x,int y)
{
	m_dialogDynamicsWorld->mouseMotionFunc(x,y);
	DemoApplication::mouseMotionFunc(x,y);
}

