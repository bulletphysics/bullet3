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

///The 3 following lines include the CPU implementation of the kernels, keep them in this order.
#include "BulletMultiThreaded/btGpuDefines.h"
#include "BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#include "BulletMultiThreaded/btGpuUtilsSharedCode.h"



#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"

#include "btGpuDemoPairCache.h"

#include "btGpuDemoDynamicsWorld.h"
#include "GLDebugFont.h"

#define USE_CUDA_DEMO_PAIR_CASHE 0

#define SPEC_TEST 0
#define OECAKE_LOADER 1

#ifdef _DEBUG
//	#define LARGE_DEMO 0
	#define LARGE_DEMO 1
#else
	#define LARGE_DEMO 1
#endif

#if LARGE_DEMO
	///create 512 (8x8x8) dynamic object
//	#define ARRAY_SIZE_X 116
//	#define ARRAY_SIZE_Y 116

//	#define ARRAY_SIZE_X 228
//	#define ARRAY_SIZE_Y 228
//	#define ARRAY_SIZE_X 256
//	#define ARRAY_SIZE_Y 156
	#define ARRAY_SIZE_X 50
	#define ARRAY_SIZE_Y 100
	#define ARRAY_SIZE_Z 1
#else
	///create 125 (5x5x5) dynamic object
	#define ARRAY_SIZE_X 5
	#define ARRAY_SIZE_Y 5
//	#define ARRAY_SIZE_Z 5
	#define ARRAY_SIZE_Z 1
#endif


//maximum number of objects (and allow user to shoot additional boxes)
#define NUM_SMALL_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z)
#define MAX_PROXIES (NUM_SMALL_PROXIES + 1024)
#define MAX_LARGE_PROXIES 0
#define MAX_SMALL_PROXIES (MAX_PROXIES - MAX_LARGE_PROXIES)

///scaling of the objects (0.1 = 20 centimeter boxes )
//#define SCALING 0.1
#define SCALING 1
#define START_POS_X 0
#define START_POS_Y 0
#define START_POS_Z 0

#include "BasicDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include <stdio.h> //printf debugging

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#ifdef BT_USE_CUDA
#include "../Extras/CUDA/btCudaBroadphase.h"
#else
#include "BulletMultiThreaded/btGpu3DGridBroadphase.h"
#endif

btScalar gTimeStep = btScalar(1./60.);

bool gbDrawBatches = false;
int gSelectedBatch = CUDA_DEMO_DYNAMICS_WORLD_MAX_BATCHES;
#ifdef BT_USE_CUDA
bool gUseCPUSolver = false;
#else
bool gUseCPUSolver = true;
#endif //BT_USE_CUDA

bool gUseBulletNarrowphase = false;

#include "oecakeLoader.h"


class	BasicDemoOecakeLoader : public BasicOECakeReader
{

	BasicDemo* m_demo;

public:

	BasicDemoOecakeLoader(BasicDemo* demo)
	:m_demo(demo)
	{

	}

	virtual void createBodyForCompoundShape(btCompoundShape* compoundTmpShape,bool addConstraint, const btTransform& worldTransform, btScalar mass)
	{

		btDefaultMotionState* myMotionState= 0;
		
		btVector3 aabbMin,aabbMax;
		compoundTmpShape->getAabb(btTransform::getIdentity(),aabbMin,aabbMax);
		int numSpheres = compoundTmpShape->getNumChildShapes();
		btAssert(numSpheres>0);
		if (numSpheres>8)
		{
			printf("error: exceeded 8 spheres\n");
			return;
		}
			
		btVector3* positions = new btVector3[numSpheres];
		btScalar* radii = new btScalar[numSpheres];

		for (int i=0;i<numSpheres;i++)
		{
			btAssert(compoundTmpShape->getChildShape(i)->getShapeType()== SPHERE_SHAPE_PROXYTYPE);
			btSphereShape* sphereShape = (btSphereShape*)compoundTmpShape->getChildShape(i);
			radii[i]=sphereShape->getRadius();
			positions[i] = compoundTmpShape->getChildTransform(i).getOrigin();
		}

		btMultiSphereShape* multiSphere = new btMultiSphereShape(positions,radii,numSpheres);
		m_demo->addCollisionShape(multiSphere);
		
			btVector3 localInertia(0,0,0);
			if (mass)
			{
				myMotionState = new btDefaultMotionState(worldTransform);
				multiSphere->calculateLocalInertia(mass,localInertia);
			}

				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btRigidBody* body = new btRigidBody(mass,myMotionState,multiSphere,localInertia);	
			body->setLinearFactor(btVector3(1,1,0));
			body->setAngularFactor(btVector3(0,0,1));

			body->setWorldTransform(worldTransform);


			m_demo->getDynamicsWorld()->addRigidBody(body);

			if (addConstraint)
			{
				btVector3 pivotInA(0,0,0);
				btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,pivotInA);
				m_demo->getDynamicsWorld()->addConstraint(p2p);
			}
	}

};


void BasicDemo::clientMoveAndDisplay()
{
	updateCamera();
	glDisable(GL_LIGHTING);
	glColor3f(1.f, 1.f, 1.f);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	glDisable(GL_TEXTURE_2D); // we always draw wireframe in this demo

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	///step the simulation
	if (m_dynamicsWorld)
	{
#if USE_CUDA_DEMO_PAIR_CASHE
		btGpuDemoPairCache* pc = (btGpuDemoPairCache*)m_dynamicsWorld->getPairCache();
		pc->m_numSmallProxies = m_dynamicsWorld->getNumCollisionObjects(); //  - 1; // exclude floor
#endif
		m_dynamicsWorld->stepSimulation(gTimeStep,0);//ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
	renderme(); 

	ms = getDeltaTimeMicroseconds();

	glFlush();

	glutSwapBuffers();

}



void BasicDemo::displayCallback(void) {

	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}


#define POS_OFFS_X (ARRAY_SIZE_X * SCALING + 20)
#define POS_OFFS_Y (ARRAY_SIZE_Y * SCALING + 10)
#define POS_OFFS_Z (ARRAY_SIZE_Z * SCALING)

#if OECAKE_LOADER
	btVector3 gWorldMin(-200,   0, 0);
	btVector3 gWorldMax( 200, 200, 0);
#else
	btVector3 gWorldMin(-POS_OFFS_X, -POS_OFFS_Y, -POS_OFFS_Z);
	btVector3 gWorldMax( POS_OFFS_X,  POS_OFFS_Y,  POS_OFFS_Z);
#endif

//btGpuDemoPairCache* gPairCache;
btOverlappingPairCache* gPairCache;


static btScalar fRandMinMax(btScalar fMin, btScalar fMax)
{
	btScalar fr = btScalar(rand()) / btScalar(RAND_MAX);
	return fMax - (fMax - fMin) * fr;
}


void	BasicDemo::initPhysics()
{
	setTexturing(false);
	setShadows(false);

#if OECAKE_LOADER
	setCameraDistance(80.);
	m_cameraTargetPosition.setValue(50, 10, 0);
#else
	#if LARGE_DEMO
		setCameraDistance(btScalar(SCALING*100.));
	#else
		setCameraDistance(btScalar(SCALING*20.));
	#endif
	m_cameraTargetPosition.setValue(START_POS_X, -START_POS_Y-20, START_POS_Z);
#endif
	m_azi = btScalar(0.f);
	m_ele = btScalar(0.f);

	///collision configuration contains default setup for memory, collision setup

	btDefaultCollisionConstructionInfo dci;
	dci.m_defaultMaxPersistentManifoldPoolSize=50000;
	dci.m_defaultMaxCollisionAlgorithmPoolSize=50000;

	m_collisionConfiguration = new btDefaultCollisionConfiguration(dci);

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,new btEmptyAlgorithm::CreateFunc);

	m_dispatcher->setNearCallback(cudaDemoNearCallback);

	
#if USE_CUDA_DEMO_PAIR_CASHE
	gPairCache = new (btAlignedAlloc(sizeof(btGpuDemoPairCache),16)) btGpuDemoPairCache(MAX_PROXIES, 24, MAX_SMALL_PROXIES); 
#else
	gPairCache = new (btAlignedAlloc(sizeof(btHashedOverlappingPairCache),16))btHashedOverlappingPairCache(); 
#endif


	btVector3 numOfCells = (gWorldMax - gWorldMin) / (2. * SCALING);
	int numOfCellsX = (int)numOfCells[0];
	int numOfCellsY = (int)numOfCells[1];
	int numOfCellsZ = (int)numOfCells[2];

//	m_broadphase = new btAxisSweep3(gWorldMin, gWorldMax, MAX_PROXIES,gPairCache);
	m_broadphase = new btDbvtBroadphase(gPairCache);
//	m_broadphase = new btGpu3DGridBroadphase(gPairCache, gWorldMin, gWorldMax,numOfCellsX, numOfCellsY, numOfCellsZ,MAX_SMALL_PROXIES,10,24,24);
//	m_broadphase = new btCudaBroadphase(gPairCache, gWorldMin, gWorldMax,numOfCellsX, numOfCellsY, numOfCellsZ,MAX_SMALL_PROXIES,10,24,24);


	///the default constraint solver
	m_solver = new btSequentialImpulseConstraintSolver();

	btGpuDemoDynamicsWorld* pDdw = new btGpuDemoDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration, MAX_PROXIES);
	m_dynamicsWorld = pDdw;
	pDdw->getSimulationIslandManager()->setSplitIslands(true);
	pDdw->setObjRad(SCALING);
	pDdw->setWorldMin(gWorldMin);
	pDdw->setWorldMax(gWorldMax);
//	gUseCPUSolver = true;
	pDdw->setUseCPUSolver(gUseCPUSolver);
	gUseBulletNarrowphase = false;
	pDdw->setUseBulletNarrowphase(gUseBulletNarrowphase);

//	m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_dynamicsWorld->setGravity(btVector3(0,-10.,0));
	m_dynamicsWorld->getSolverInfo().m_numIterations = 4;

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance



#if 1
		#define SPRADIUS btScalar(SCALING*0.1f)
		#define SPRPOS btScalar(SCALING*0.05f)
		static btVector3 sSphPos[8];
		
		for (int k=0;k<8;k++)
		{
			sSphPos[k].setValue((k-4)*0.25*SCALING,0,0);
		}
		
		btVector3 inertiaHalfExtents(SPRADIUS,  SPRADIUS,  SPRADIUS);
		static btScalar sSphRad[8] = 
		{
//			 SPRADIUS,  SPRADIUS,  SPRADIUS, SPRADIUS,SPRADIUS,  SPRADIUS,  SPRADIUS, 0.3
			 SPRADIUS,  SPRADIUS,  SPRADIUS, SPRADIUS,SPRADIUS,  SPRADIUS,  SPRADIUS, SPRADIUS
		};
//		sSphPos[0].setX(sSphPos[0].getX()-0.15);
		#undef SPR
		btMultiSphereShape* colShape[2];
		colShape[0] = new btMultiSphereShape( sSphPos, sSphRad, 8);
		colShape[1] = new btMultiSphereShape( sSphPos, sSphRad, 2);

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape[0]);
		m_collisionShapes.push_back(colShape[1]);
#endif

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(0.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);

#if OECAKE_LOADER
	BasicDemoOecakeLoader	loader(this);
	if (!loader.processFile("test1.oec"))
	{
		loader.processFile("../../test1.oec");
	}
#if 0 // perfomance test : work-in-progress
	{ // add more object, but share their shapes
		int numNewObjects = 500;
		mass = 1.f;
		for(int n_obj = 0; n_obj < numNewObjects; n_obj++)
		{
			btDefaultMotionState* myMotionState= 0;
			btVector3 localInertia(0,0,0);
			btTransform worldTransform;
			worldTransform.setIdentity();
			btScalar fx = fRandMinMax(-30., 30.);
			btScalar fy = fRandMinMax(5., 30.);
			worldTransform.setOrigin(btVector3(fx, fy, 0.f));
			int sz = m_collisionShapes.size();
			btMultiSphereShape* multiSphere = (btMultiSphereShape*)m_collisionShapes[1];
			myMotionState = new btDefaultMotionState(worldTransform);
			multiSphere->calculateLocalInertia(mass, localInertia);
			btRigidBody* body = new btRigidBody(mass,myMotionState,multiSphere,localInertia);	
			body->setLinearFactor(btVector3(1,1,0));
			body->setAngularFactor(btVector3(0,0,1));
			body->setWorldTransform(worldTransform);
			getDynamicsWorld()->addRigidBody(body);
		}
	}
#endif

#else
#if (!SPEC_TEST)
		float start_x = START_POS_X - ARRAY_SIZE_X * SCALING;
		float start_y = START_POS_Y - ARRAY_SIZE_Y * SCALING;
		float start_z = START_POS_Z - ARRAY_SIZE_Z * SCALING;

		int collisionShapeIndex = 0;
		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					float offs = (2. * (float)rand() / (float)RAND_MAX - 1.f) * 0.05f;
					startTransform.setOrigin(SCALING*btVector3(
										2.0*SCALING*i + start_x + offs,
										2.0*SCALING*k + start_y + offs,
										2.0*SCALING*j + start_z));

					if (isDynamic)
						colShape[collisionShapeIndex]->calculateLocalInertia(mass,localInertia);


			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShape[collisionShapeIndex],localInertia);
					collisionShapeIndex = 1 - collisionShapeIndex;
					rbInfo.m_startWorldTransform=startTransform;
					btRigidBody* body = new btRigidBody(rbInfo);
					m_dynamicsWorld->addRigidBody(body);
				}
			}
		}
#else//SPEC_TEST
		// narrowphase test - 2 bodies at the same position
		float start_x = START_POS_X;
//		float start_y = START_POS_Y;
		float start_y = gWorldMin[1] + SCALING * 0.7f + 5.f;
		float start_z = START_POS_Z;
		startTransform.setOrigin(SCALING*btVector3(start_x,start_y,start_z));
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShape[0],localInertia);
		rbInfo.m_startWorldTransform=startTransform;
		btRigidBody* body = new btRigidBody(rbInfo);
		m_dynamicsWorld->addRigidBody(body);

		btPoint2PointConstraint * p2pConstr =  new btPoint2PointConstraint(*body, btVector3(1., 0., 0.));
		m_dynamicsWorld->addConstraint(p2pConstr);

		startTransform.setOrigin(SCALING*btVector3(start_x-2.f, start_y,start_z));
		rbInfo.m_startWorldTransform=startTransform;
		btRigidBody* body1 = new btRigidBody(rbInfo);
		m_dynamicsWorld->addRigidBody(body1);

		p2pConstr =  new btPoint2PointConstraint(*body, *body1, btVector3(-1., 0., 0.), btVector3(1., 0., 0.));
		m_dynamicsWorld->addConstraint(p2pConstr);


#endif//SPEC_TEST
#endif //OE_CAKE_LOADER
	}
	// now set Ids used by collision detector and constraint solver
	int numObjects = m_dynamicsWorld->getNumCollisionObjects();
	btCollisionObjectArray& collisionObjects = m_dynamicsWorld->getCollisionObjectArray();
	for(int i = 0; i < numObjects; i++)
	{
		btCollisionObject* colObj = collisionObjects[i];
		colObj->setCompanionId(i+1); // 0 reserved for the "world" object
		btCollisionShape* pShape = colObj->getCollisionShape();
		int shapeType = pShape->getShapeType();
		if(shapeType == MULTI_SPHERE_SHAPE_PROXYTYPE)
		{
			btMultiSphereShape* pMs = (btMultiSphereShape*)pShape;
			int numSpheres = pMs->getSphereCount();
			pDdw->addMultiShereObject(numSpheres, i + 1);
			for(int j = 0; j < numSpheres; j++)
			{
				btVector3 sphPos = pMs->getSpherePosition(j);
				float sphRad = pMs->getSphereRadius(j);
				pDdw->addSphere(sphPos, sphRad);
			}
		}
		else
		{
			btAssert(0);
		}
	}
#if OECAKE_LOADER
	clientResetScene();
#endif
}

void BasicDemo::clientResetScene()
{
	DemoApplication::clientResetScene();
#if OECAKE_LOADER
	return;
#endif
#if SPEC_TEST
	{
		float start_x = START_POS_X;
//		float start_y = START_POS_Y;
		float start_y = gWorldMin[1] + SCALING * 0.7f + 5.f;
		float start_z = START_POS_Z;
		int numObjects = m_dynamicsWorld->getNumCollisionObjects();
		btCollisionObjectArray& collisionObjects = m_dynamicsWorld->getCollisionObjectArray();
		btTransform startTransform;
		startTransform.setIdentity();
		for(int n = 0; n < numObjects; n++)
		{
			btCollisionObject* colObj = collisionObjects[n];
			btRigidBody* rb = btRigidBody::upcast(colObj);
			if(!n)
			{
				startTransform.setOrigin(SCALING*btVector3(start_x,start_y,start_z));
			}
			else
			{
//				startTransform.setOrigin(SCALING*btVector3(start_x+0.1f,start_y+SCALING * 0.7f * 2.f, start_z));
				startTransform.setOrigin(SCALING*btVector3(start_x-2.f,start_y, start_z));
			}
			rb->setCenterOfMassTransform(startTransform);
		}
		return;
	}
#endif
// we don't use motionState, so reset transforms here
	int numObjects = m_dynamicsWorld->getNumCollisionObjects();
	btCollisionObjectArray& collisionObjects = m_dynamicsWorld->getCollisionObjectArray();

	float start_x = START_POS_X - ARRAY_SIZE_X * SCALING;
	float start_y = START_POS_Y - ARRAY_SIZE_Y * SCALING;
	float start_z = START_POS_Z - ARRAY_SIZE_Z * SCALING;
	btTransform startTransform;
	startTransform.setIdentity();

	for(int n = 0; n < numObjects; n++)
	{
		btCollisionObject* colObj = collisionObjects[n];
		colObj->setCompanionId(n);
		btRigidBody* rb = btRigidBody::upcast(colObj);
		int offs = ARRAY_SIZE_X * ARRAY_SIZE_Z;
		int indx = n;
		int ky = indx / offs;
		indx -= ky * offs;
		int kx = indx / ARRAY_SIZE_Z;
		indx -= kx * ARRAY_SIZE_Z;
		int kz = indx;
		startTransform.setOrigin(SCALING*btVector3(
									2.0*SCALING*kx + start_x,
									2.0*SCALING*ky + start_y,
									2.0*SCALING*kz + start_z));
		rb->setCenterOfMassTransform(startTransform);
	}
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

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}



void BasicDemo::keyboardCallback(unsigned char key, int x, int y)
{
	(void)x;
	(void)y;
	switch (key) 
	{
		case 'q' : 
			exitPhysics();
			exit(0);
			break;
		case 'c' :
			{
				gbDrawBatches = !gbDrawBatches;
				break;
			}
		case 'b' :
			{
				gSelectedBatch++;
				gSelectedBatch %= (CUDA_DEMO_DYNAMICS_WORLD_MAX_BATCHES + 1);
				break;
			}
		case 'u' :
			{
				btGpuDemoDynamicsWorld* pDdw = (btGpuDemoDynamicsWorld*)m_dynamicsWorld;
				gUseCPUSolver = !gUseCPUSolver;
				pDdw->setUseCPUSolver(gUseCPUSolver);
				break;
			}
		case 'j' :
			{
				btGpuDemoDynamicsWorld* pDdw = (btGpuDemoDynamicsWorld*)m_dynamicsWorld;
				gUseBulletNarrowphase = !gUseBulletNarrowphase;
				pDdw->setUseBulletNarrowphase(gUseBulletNarrowphase);
				if(gUseBulletNarrowphase)
				{
					m_dispatcher->setNearCallback(btCollisionDispatcher::defaultNearCallback);
				}
				else
				{
					m_dispatcher->setNearCallback(cudaDemoNearCallback);
				}
				break;
			}
		default : 
			{
				DemoApplication::keyboardCallback(key, x, y);
			}
			break;
	}
	
	if(key == ' ')
	{
#if USE_CUDA_DEMO_PAIR_CASHE
		((btGpuDemoPairCache*)gPairCache)->reset();
#endif
	}
}




#define BATCH_NUM_COLORS 12

const float cBatchColorTab[BATCH_NUM_COLORS * 3] = 
	{
		1.f, 0.f, 0.f,
		0.f, 1.f, 0.f,
		0.f, 0.f, 1.f,
		1.f, 1.f, 0.f,
		0.f, 1.f, 1.f,
		1.f, 0.f, 1.f,
		1.f, .5f, 0.f,
		.5f, 1.f, 0.f,
		0.f, 1.f, .5f,
		0.f, .5f, 1.f,
		.5f, 0.f, 1.f,
		1.f, 0.f, .5f
	};


void BasicDemo::DrawConstraintInfo()
{
	int fontW = 10; // hack, could be changed
	int fontH = 14; // hack, could be changed
	char buf[32];
	float xOffs;
	float yOffs = fontH * 2;
	glDisable(GL_LIGHTING);
	glColor3f(1, 1, 1);
	sprintf(buf,"solver on %s", gUseCPUSolver ? "CPU" : "CUDA");
	xOffs = m_glutScreenWidth - (strlen(buf) + 1) * fontW;	
	GLDebugDrawString(xOffs, yOffs,buf);
	yOffs += fontH;
	btGpuDemoDynamicsWorld* cddw = (btGpuDemoDynamicsWorld*)m_dynamicsWorld;
	for(int i = 0; i < CUDA_DEMO_DYNAMICS_WORLD_MAX_BATCHES; i++)
	{
		const float* pCol = cBatchColorTab + i * 3;
		glColor3f(pCol[0], pCol[1], pCol[2]);
		sprintf(buf,"%2d : %5d", i, cddw->m_numInBatches[i]);
		xOffs = m_glutScreenWidth - (strlen(buf) + 1) * fontW;	
		GLDebugDrawString(xOffs, yOffs,buf);
		yOffs += fontH;
	}
}


void BasicDemo::renderme()
{
	renderscene(0);
	if(gbDrawBatches)
	{
		((btGpuDemoDynamicsWorld*)m_dynamicsWorld)->debugDrawConstraints(gSelectedBatch, cBatchColorTab);
	}

//	if (0)
	if ((m_debugMode & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		int  xOffset = 10.f;
		int  yStart = 20.f;
		int  yIncr = 20.f;
		showProfileInfo(xOffset, yStart, yIncr);
		DrawConstraintInfo();
		outputDebugInfo(xOffset, yStart, yIncr);
		resetPerspectiveProjection();
	}
}



extern int gNumClampedCcdMotions;
#define SHOW_NUM_DEEP_PENETRATIONS 1
#ifdef SHOW_NUM_DEEP_PENETRATIONS 
	extern int gNumDeepPenetrationChecks;
	extern int gNumSplitImpulseRecoveries;
	extern int gNumGjkChecks;
	extern int gNumAlignedAllocs;
	extern int gNumAlignedFree;
	extern int gTotalBytesAlignedAllocs;
#endif //


void BasicDemo::outputDebugInfo(int & xOffset,int & yStart, int  yIncr)
{
	char buf[124];
	glDisable(GL_LIGHTING);
	glColor3f(0, 0, 0);
	
	sprintf(buf,"mouse move+buttons to interact");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"space to reset");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"cursor keys and z,x to navigate");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"i to toggle simulation, s single step");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"q to quit");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"h to toggle help text");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"p to toggle profiling (+results to file)");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"c to toggle constraint drawing");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;


	sprintf(buf,"b to draw single constraint batch");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"u to toggle between CPU  and CUDA solvers");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"d to toggle between different batch builders");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	if (getDynamicsWorld())
	{
		
		sprintf(buf,"# objects = %d",getDynamicsWorld()->getNumCollisionObjects());
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
		
		sprintf(buf,"# pairs = %d",getDynamicsWorld()->getBroadphase()->getOverlappingPairCache()->getNumOverlappingPairs());
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
	}
} // BasicDemo::outputDebugInfo()
