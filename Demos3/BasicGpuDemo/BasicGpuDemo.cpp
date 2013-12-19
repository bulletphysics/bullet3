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


///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 20
#define ARRAY_SIZE_Y 20
#define ARRAY_SIZE_Z 20

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "BasicGpuDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "b3GpuDynamicsWorld.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"



#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"


#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"



#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"

static GLDebugDrawer gDebugDraw;


void BasicGpuDemo::clientMoveAndDisplay()
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



void BasicGpuDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}

struct btInternalData
{
	cl_context	m_clContext;
	cl_device_id	m_clDevice;
	cl_command_queue	m_clQueue;
	const char* m_clDeviceName;
	bool	m_clInitialized;

	btInternalData()
	{
		m_clContext = 0;
		m_clDevice = 0;
		m_clQueue = 0;
		m_clDeviceName = 0;
		m_clInitialized =false;
	}
};

void BasicGpuDemo::initCL(int preferredDeviceIndex, int preferredPlatformIndex)
{
	void* glCtx=0;
	void* glDC = 0;
	
	
    
	int ciErrNum = 0;
	//#ifdef CL_PLATFORM_INTEL
	//cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
	//#else
	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
	//#endif
	
	cl_platform_id platformId;
	
	//	if (useInterop)
	//	{
	//		m_data->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	//	} else
	{
		m_clData->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0,0,preferredDeviceIndex, preferredPlatformIndex,&platformId);
		b3OpenCLUtils::printPlatformInfo(platformId);
	}
	
	
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	
	int numDev = b3OpenCLUtils::getNumDevices(m_clData->m_clContext);
	
	if (numDev>0)
	{
		m_clData->m_clDevice= b3OpenCLUtils::getDevice(m_clData->m_clContext,0);
		m_clData->m_clQueue = clCreateCommandQueue(m_clData->m_clContext, m_clData->m_clDevice, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
        
        b3OpenCLUtils::printDeviceInfo(m_clData->m_clDevice);
		b3OpenCLDeviceInfo info;
		b3OpenCLUtils::getDeviceInfo(m_clData->m_clDevice,&info);
		m_clData->m_clDeviceName = info.m_deviceName;
		m_clData->m_clInitialized = true;
		
	}
	
}

void BasicGpuDemo::exitCL()
{

}

BasicGpuDemo::BasicGpuDemo()
{
	m_np=0;
	m_bp=0;
	m_clData = new btInternalData;
	setCameraDistance(btScalar(SCALING*60.));
	this->setAzi(45);
	this->setEle(45);

}

BasicGpuDemo::~BasicGpuDemo()
{
	exitPhysics();
	exitCL();
	delete m_clData;
}


extern bool gUseLargeBatches;

void	BasicGpuDemo::initPhysics()
{
	gUseLargeBatches = true;//for testing, this option is faster on NVIDIA GPUs
	//use the Bullet 2.x btQuickprof for profiling of Bullet 3.x
	b3SetCustomEnterProfileZoneFunc(CProfileManager::Start_Profile);
	b3SetCustomLeaveProfileZoneFunc(CProfileManager::Stop_Profile);

	setTexturing(true);
	setShadows(false);//too slow with many objects
	
	
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = 0;
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = 0;

	m_broadphase = 0;

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	
	m_solver = 0;

	initCL(-1,-1);


	if (!m_clData->m_clInitialized)
	{
		printf("Error: cannot initialize OpenCL\n");
		exit(0);
	}

	b3Config config;
	m_np = new b3GpuNarrowPhase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue,config);
	m_bp = new b3GpuSapBroadphase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);
	
	b3DynamicBvhBroadphase* broadphaseDbvt = new b3DynamicBvhBroadphase(config.m_maxConvexBodies);
	
	m_rbp = new b3GpuRigidBodyPipeline(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue, m_np, m_bp,broadphaseDbvt,config);

	m_dynamicsWorld = new b3GpuDynamicsWorld(m_bp,m_np,m_rbp);
	
	m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));


	createObjects();
}

void BasicGpuDemo::createObjects()
{
		///create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(150.),btScalar(50.),btScalar(150.)));
	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);
	if (0)
	{
		btTransform tr;
		tr.setIdentity();
		btVector3 faraway(-1e30,-1e30,-1e30);

		tr.setOrigin(faraway);
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
		btSphereShape* dummyShape = new btSphereShape(0.f);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,dummyShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);


	}
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
		//btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setWorldTransform(groundTransform);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
	
	
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(SCALING*1.f));
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
										btScalar(2.*i + start_x),
										btScalar(6+2.0*k + start_y),
										btScalar(2.*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					body->setWorldTransform(startTransform);
					

					m_dynamicsWorld->addRigidBody(body);
				}
			}
		}
	}

	


	m_np->writeAllBodiesToGpu();
	m_bp->writeAabbsToGpu();
	m_rbp->writeAllInstancesToGpu();

}
void	BasicGpuDemo::clientResetScene()
{
	/*
	exitPhysics();
	initPhysics();
	*/


	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );//reset will take care of this
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	((b3GpuDynamicsWorld*)m_dynamicsWorld)->reset();

	createObjects();
}
	

void	BasicGpuDemo::exitPhysics()
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

	delete m_np;
	
	delete m_bp;

	delete m_rbp;
}




