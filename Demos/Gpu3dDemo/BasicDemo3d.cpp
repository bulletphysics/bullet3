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


#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#include "BulletMultiThreaded/SpuContactManifoldCollisionAlgorithm.h"

#include "btGpuDemoDynamicsWorld3D.h"

#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#include "GLDebugFont.h"
extern int gSkippedCol;
extern int gProcessedCol;



#define SPEC_TEST 0

#ifdef _DEBUG
	#define LARGE_DEMO 1
//	#define LARGE_DEMO 1
#else
	#define LARGE_DEMO 1
#endif

#if LARGE_DEMO
	///create 512 (8x8x8) dynamic object
//	#define ARRAY_SIZE_X 100
//	#define ARRAY_SIZE_Y 100
//	#define ARRAY_SIZE_Z 1
//	#define ARRAY_SIZE_X 228
//	#define ARRAY_SIZE_Y 228
//	#define ARRAY_SIZE_X 30
//	#define ARRAY_SIZE_Y 100

#define ARRAY_SIZE_X 8
#define ARRAY_SIZE_Y 47
#define ARRAY_SIZE_Z 8
#else
	///create 125 (5x5x5) dynamic object
	#define ARRAY_SIZE_X 45
	#define ARRAY_SIZE_Y 45
//	#define ARRAY_SIZE_Z 5
	#define ARRAY_SIZE_Z 1
#endif


//maximum number of objects (and allow user to shoot additional boxes)
#define NUM_SMALL_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z)
#define MAX_PROXIES (NUM_SMALL_PROXIES + 1024)
#define MAX_LARGE_PROXIES 10
#define MAX_SMALL_PROXIES (MAX_PROXIES - MAX_LARGE_PROXIES)

///scaling of the objects (0.1 = 20 centimeter boxes )
//#define SCALING 0.1
#define SCALING 1
#define START_POS_X 0
#define START_POS_Y 5
#define START_POS_Z 0

#include "BasicDemo3d.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include <stdio.h> //printf debugging

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#include "../Extras/CUDA/btCudaBroadphase.h"

btScalar gTimeStep = btScalar(1./60.);

bool gbDrawBatches = false;
int gSelectedBatch = CUDA_DEMO_DYNAMICS_WORLD3D_MAX_BATCHES;
bool gUseCPUSolver = false;
bool gUseSolver2 = true;
bool gDrawWire = false;
bool gUseCudaMotIntegr = true;


void BasicDemo3D::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	///step the simulation
	if (m_dynamicsWorld)
	{
//		btCudaDemoPairCache* pc = (btCudaDemoPairCache*)m_dynamicsWorld->getPairCache();
//		pc->m_numSmallProxies = m_dynamicsWorld->getNumCollisionObjects(); //  - 1; // exclude floor
		m_dynamicsWorld->stepSimulation(gTimeStep,0);//ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
	renderme(); 

	ms = getDeltaTimeMicroseconds();

	glFlush();

	glutSwapBuffers();

}



void BasicDemo3D::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

#define NUM_SOLVERS 11
static btConstraintSolver* sConstraintSolvers[NUM_SOLVERS];
static int sCurrSolverIndex = 9;
static char* sConstraintSolverNames[NUM_SOLVERS] = 
{
	"btSequentialImpulseConstraintSolver",
	"btParallelBatchConstraintSolver",
	"btCudaConstraintSolver",
	"btParallelBatchConstraintSolver2",
	"btParallelBatchConstraintSolver3",
	"btCudaConstraintSolver3",
	"btParallelBatchConstraintSolver4",
	"btCudaConstraintSolver4",
	"btParallelBatchConstraintSolver5",
	"btParallelBatchConstraintSolver6",
	"btCudaConstraintSolver6",
};

//btVector3 gWorldMin(-228,-228,-32);
//btVector3 gWorldMin(-228,0,-32);
//btVector3 gWorldMax(228,228,32);

//btVector3 gWorldMin(-150,-228,-32);
//btVector3 gWorldMax(150,228,32);

#define POS_OFFS_X (ARRAY_SIZE_X * SCALING + 50)
#define POS_OFFS_Y (ARRAY_SIZE_Y * SCALING )
#define POS_OFFS_Z (ARRAY_SIZE_Z * SCALING + 5)

btVector3 gWorldMin(-POS_OFFS_X, -ARRAY_SIZE_Y*SCALING, -80-POS_OFFS_Z);
btVector3 gWorldMax( POS_OFFS_X,  POS_OFFS_Y,  80+POS_OFFS_Z);

//btCudaDemoPairCache* gPairCache;
btHashedOverlappingPairCache* gPairCache;

void	BasicDemo3D::initPhysics()
{
	setTexturing(true);
	setShadows(false);

//	setCameraDistance(btScalar(SCALING*50.));
#if LARGE_DEMO
	setCameraDistance(btScalar(SCALING*50.));
#else
	setCameraDistance(btScalar(SCALING*20.));
#endif

	m_cameraTargetPosition.setValue(START_POS_X, -START_POS_Y-20, START_POS_Z);
	m_azi = btScalar(0.f);
	m_ele = btScalar(0.f);

	///collision configuration contains default setup for memory, collision setup

	btDefaultCollisionConstructionInfo dci;
	dci.m_defaultMaxPersistentManifoldPoolSize=100000;
	dci.m_defaultMaxCollisionAlgorithmPoolSize=100000;
	dci.m_customCollisionAlgorithmMaxElementSize = sizeof(SpuContactManifoldCollisionAlgorithm);	
	
	
	///SpuContactManifoldCollisionAlgorithm is larger than any of the other collision algorithms
//@@	dci.m_customMaxCollisionAlgorithmSize = sizeof(SpuContactManifoldCollisionAlgorithm);

	m_collisionConfiguration = new btDefaultCollisionConfiguration(dci);

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	//m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#ifndef WIN32
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#else
	unsigned int	maxNumOutstandingTasks =4;
	//createCollisionLocalStoreMemory();
	//processSolverTask
	Win32ThreadSupport::Win32ThreadConstructionInfo  threadConstructionInfo("narrowphase_multi",processCollisionTask,createCollisionLocalStoreMemory,maxNumOutstandingTasks);
	class	btThreadSupportInterface*	threadInterface = new Win32ThreadSupport(threadConstructionInfo);
	m_dispatcher = new SpuGatheringCollisionDispatcher(threadInterface,maxNumOutstandingTasks,m_collisionConfiguration);
#endif //SINGLE_THREADED_NARROWPHASE


//##	m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,new btEmptyAlgorithm::CreateFunc);
//##	m_dispatcher->registerCollisionCreateFunc(CUSTOM_CONVEX_SHAPE_TYPE,CUSTOM_CONVEX_SHAPE_TYPE,new btBox2dBox2dCollisionAlgorithm::CreateFunc);

//	m_broadphase = new btDbvtBroadphase();
	

//##	gPairCache = new (btAlignedAlloc(sizeof(btCudaDemoPairCache),16)) btCudaDemoPairCache(MAX_PROXIES, 24, MAX_SMALL_PROXIES); 
//	gPairCache = NULL;
	gPairCache = new (btAlignedAlloc(sizeof(btHashedOverlappingPairCache),16)) btHashedOverlappingPairCache(); 

	//m_broadphase = new btSimpleBroadphase(16384, gPairCache);

/*
btCudaBroadphase::btCudaBroadphase(	btOverlappingPairCache* overlappingPairCache,
									const btVector3& worldAabbMin,const btVector3& worldAabbMax, 
									int gridSizeX, int gridSizeY, int gridSizeZ, 
									int maxSmallProxies, int maxLargeProxies, int maxPairsPerBody,
									int maxBodiesPerCell,
									btScalar cellFactorAABB)
*/
//	btVector3 numOfCells = (gWorldMax - gWorldMin) / (2. * SCALING * 0.7);
	btVector3 numOfCells = (gWorldMax - gWorldMin) / (2. * SCALING);
	int numOfCellsX = (int)numOfCells[0];
	int numOfCellsY = (int)numOfCells[1];
	int numOfCellsZ = (int)numOfCells[2];

//	m_broadphase = new bt3DGridBroadphase(gPairCache, gWorldMin, gWorldMax,numOfCellsX, numOfCellsY, numOfCellsZ,MAX_SMALL_PROXIES,10,8,8,1./1.5);
//#define USE_CUDA_BROADPHASE 1
#ifdef USE_CUDA_BROADPHASE
	m_broadphase = new btCudaBroadphase(gPairCache, gWorldMin, gWorldMax,numOfCellsX, numOfCellsY, numOfCellsZ,MAX_SMALL_PROXIES,20,18,8,1./1.5);
#else
	
#if DBVT
	btDbvtBroadphase* dbvt = new btDbvtBroadphase(gPairCache);
	m_broadphase = dbvt;
	dbvt->m_deferedcollide=false;
	dbvt->m_prediction = 0.f;
#else
	m_broadphase = new btAxisSweep3(gWorldMin,gWorldMax,32000,gPairCache,true);//(btDbvtBroadphase(gPairCache);
#endif //DBVT

#endif	
	

	// create solvers for tests 
	///the default constraint solver
	sConstraintSolvers[0] = new btSequentialImpulseConstraintSolver();
/*
	sConstraintSolvers[1] = new btParallelBatchConstraintSolver();
	sConstraintSolvers[2] = new btCudaConstraintSolver();
	sConstraintSolvers[3] = new btParallelBatchConstraintSolver2();
	sConstraintSolvers[4] = new btParallelBatchConstraintSolver3();
	sConstraintSolvers[5] = new btCudaConstraintSolver3();
	sConstraintSolvers[6] = new btParallelBatchConstraintSolver4();
	sConstraintSolvers[7] = new btCudaConstraintSolver4();
	sConstraintSolvers[8] = new btParallelBatchConstraintSolver5();
	sConstraintSolvers[9] = new btParallelBatchConstraintSolver6();
	sConstraintSolvers[10] = new btCudaConstraintSolver6();
*/
	sCurrSolverIndex = 0;
	m_solver = sConstraintSolvers[sCurrSolverIndex];
	printf("\nUsing %s\n", sConstraintSolverNames[sCurrSolverIndex]);

//	sCudaMotionInterface = new btCudaMotionInterface(MAX_PROXIES);
//  m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration, sCudaMotionInterface);
//	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
//##	btCudaDemoDynamicsWorld* pDdw = new btCudaDemoDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	btCudaDemoDynamicsWorld3D* pDdw = new btCudaDemoDynamicsWorld3D(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = pDdw;
	pDdw->getDispatchInfo().m_enableSPU=true;
	pDdw->getSimulationIslandManager()->setSplitIslands(sCurrSolverIndex == 0);
	pDdw->setObjRad(SCALING);
	pDdw->setWorldMin(gWorldMin);
	pDdw->setWorldMax(gWorldMax);
#ifdef BT_USE_CUDA
	gUseCPUSolver = false;
#else
	gUseCPUSolver = true;
#endif
	pDdw->setUseCPUSolver(gUseCPUSolver);
//	pDdw->setUseSolver2(gUseSolver2);

//	m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_dynamicsWorld->setGravity(btVector3(0.f,-10.f,0.f));
	m_dynamicsWorld->getSolverInfo().m_numIterations = 4;


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance


		//btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,0.1));//SCALING*1));
//##		btCollisionShape* colShape = new btBox2dShape(btVector3(SCALING*.7,SCALING*.7,0.1));//SCALING*1));
		btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*.7,SCALING*.7, SCALING*.7));

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
#if (!SPEC_TEST)
		float start_x = START_POS_X - ARRAY_SIZE_X * SCALING;
		float start_y = START_POS_Y - ARRAY_SIZE_Y * SCALING;
		float start_z = START_POS_Z - ARRAY_SIZE_Z * SCALING;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
										2.0*SCALING*i + start_x,
										2.0*SCALING*k + start_y,
										2.0*SCALING*j + start_z));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShape,localInertia);
					rbInfo.m_startWorldTransform=startTransform;
					btRigidBody* body = new btRigidBody(rbInfo);
					m_dynamicsWorld->addRigidBody(body);
				}
			}
		}
#else
		// narrowphase test - 2 bodies at the same position
		float start_x = START_POS_X;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z;
//		startTransform.setOrigin(SCALING*btVector3(start_x,start_y-14.f,start_z));
		startTransform.setOrigin(SCALING*btVector3(start_x,start_y-11.f,start_z));
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShape,localInertia);
		rbInfo.m_startWorldTransform=startTransform;
		btRigidBody* body = new btRigidBody(rbInfo);
		m_dynamicsWorld->addRigidBody(body);
//		startTransform.setOrigin(SCALING*btVector3(start_x+1.2f,start_y+1.4f-14.f,start_z));
		startTransform.setOrigin(SCALING*btVector3(start_x,start_y + 1.5f -11.f, start_z));
		rbInfo.m_startWorldTransform=startTransform;
		body = new btRigidBody(rbInfo);
		m_dynamicsWorld->addRigidBody(body);
#endif
	}


#if 0
	///create a few basic rigid bodies
//	btCollisionShape* groundShape = new btBox2dShape(btVector3(btScalar(50.),btScalar(1.),btScalar(50.)));
//	btCollisionShape* groundShape = new btBox2dShape(btVector3(btScalar(228.),btScalar(1.),btScalar(228.)));
//	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(228.),btScalar(1.),btScalar(228.)));
//	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	btCollisionShape* groundShape = new btBoxShape(btVector3(POS_OFFS_X, btScalar(1.), POS_OFFS_Z));
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, gWorldMin[1], 0));

//	groundTransform.setOrigin(btVector3(0,-5,0));
//	groundTransform.setOrigin(btVector3(0,-50,0));

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
#endif
	//clientResetScene();
}

void BasicDemo3D::clientResetScene()
{
	DemoApplication::clientResetScene();
	btCudaDemoDynamicsWorld3D* pDdw = (btCudaDemoDynamicsWorld3D*)m_dynamicsWorld;
	pDdw->resetScene();
#if SPEC_TEST
	{
		float start_x = START_POS_X;
		float start_y = START_POS_Y;
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
//				startTransform.setOrigin(SCALING*btVector3(start_x,start_y-14.f,start_z));
				startTransform.setOrigin(SCALING*btVector3(start_x,start_y-11.f,start_z));
			}
			else
			{
//				startTransform.setOrigin(SCALING*btVector3(start_x+1.2f,start_y+1.4f-14.f,start_z));
				startTransform.setOrigin(SCALING*btVector3(start_x, start_y+1.5f-11.f,start_z));
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

	

void	BasicDemo3D::exitPhysics()
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
	
	m_solver = 0;
	for(int j = 0; j < NUM_SOLVERS; j++)
	{
		delete sConstraintSolvers[j];
	}
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}



void BasicDemo3D::keyboardCallback(unsigned char key, int x, int y)
{
	(void)x;
	(void)y;
	switch (key) 
	{
		case 'q' : 
			{
			exitPhysics();
			exit(0);
			}
			break;
#if 0
		case 's' :
			{	
				sCurrSolverIndex++;
				sCurrSolverIndex %= NUM_SOLVERS;
				btDiscreteDynamicsWorld* pDdw = (btDiscreteDynamicsWorld*)m_dynamicsWorld;
				pDdw->getSimulationIslandManager()->setSplitIslands(sCurrSolverIndex == 0);
				pDdw->setConstraintSolver(sConstraintSolvers[sCurrSolverIndex]);
				printf("\nUsing %s\n", sConstraintSolverNames[sCurrSolverIndex]);
			}
			break;
#endif
		case 'c' :
			{
				gbDrawBatches = !gbDrawBatches;
				break;
			}
		case 'b' :
			{
				gSelectedBatch++;
				gSelectedBatch %= (CUDA_DEMO_DYNAMICS_WORLD3D_MAX_BATCHES + 1);
				break;
			}
		case 'u' :
			{
#ifdef BT_USE_CUDA
				btCudaDemoDynamicsWorld3D* pDdw = (btCudaDemoDynamicsWorld3D*)m_dynamicsWorld;
				gUseCPUSolver = !gUseCPUSolver;
				pDdw->setUseCPUSolver(gUseCPUSolver);
#endif
				break;
			}
		case 'w' :
			{
				gDrawWire = !gDrawWire;
				setWireMode(gDrawWire);
				break;
			}
		case 'm' :
			{
				btCudaDemoDynamicsWorld3D* pDdw = (btCudaDemoDynamicsWorld3D*)m_dynamicsWorld;
				gUseCudaMotIntegr = !gUseCudaMotIntegr;
				pDdw->setUseCudaMotIntegr(gUseCudaMotIntegr);
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
		//gPairCache->reset();
	}
}


void BasicDemo3D::mouseFunc(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN) {
        m_mouseButtons |= 1<<button;
    } else if (state == GLUT_UP) {
        m_mouseButtons = 0;
    }
	if (glutGetModifiers() & GLUT_ACTIVE_SHIFT
		&& state == GLUT_DOWN){
		m_mouseButtons |= 2 << 2;
	}
    m_mouseOldX = x;
    m_mouseOldY = y;
    glutPostRedisplay();
}

void BasicDemo3D::mouseMotionFunc(int x,int y)
{
    float dx, dy;
    dx = x - m_mouseOldX;
    dy = y - m_mouseOldY;

	if(m_mouseButtons & (2 << 2) && m_mouseButtons & 1)
	{
	}
	else if(m_mouseButtons & 1) 
	{
		m_azi += dx * 0.2;
		m_azi = fmodf(m_azi, 180.f);
		m_ele += dy * 0.2;
		m_ele = fmodf(m_ele, 180.f);
    } 
	else if(m_mouseButtons & 4) 
	{
		m_cameraDistance += dy * 0.2f;
	} 
	else if(m_mouseButtons & 3)
	{
		m_cameraTargetPosition[0] += dx * 0.05f;
		m_cameraTargetPosition[1] += dy * 0.05f;
	}
    m_mouseOldX = x;
    m_mouseOldY = y;
	updateCamera();
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


void BasicDemo3D::DrawConstraintInfo()
{
	char buf[32];
	float xOffs = m_glutScreenWidth - 50;
	float yOffs = 30;
	glColor4f(1, 1, 1,1);
	glDisable(GL_LIGHTING);
	glRasterPos3f(xOffs-40.f, yOffs, 0);
	sprintf(buf,"solver %2d on %s", gUseSolver2 ? 2 : 1, gUseCPUSolver ? "CPU" : "CUDA");
	GLDebugDrawString(xOffs-140.f, yOffs,buf);
	yOffs += 15.f;
	btCudaDemoDynamicsWorld3D* cddw = (btCudaDemoDynamicsWorld3D*)m_dynamicsWorld;
	for(int i = 0; i < CUDA_DEMO_DYNAMICS_WORLD3D_MAX_BATCHES; i++)
	{
		const float* pCol = cBatchColorTab + i * 3;
		glColor3f(pCol[0], pCol[1], pCol[2]);
		glRasterPos3f(xOffs, yOffs, 0);
		sprintf(buf,"%2d : %5d", i, cddw->m_numInBatches[i]);
		GLDebugDrawString(xOffs-80, yOffs,buf);
		yOffs += 15.f;
	}
}


void BasicDemo3D::renderme()
{
	updateCamera();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if(gDrawWire)
	{
		glColor3f(1.f, 1.f, 1.f);
		glDisable(GL_LIGHTING);
		setTexturing(false);
	}
	else
	{
		myinit();
		setTexturing(true);
	}
		
	renderscene(0);

	if(gbDrawBatches)
	{
		((btCudaDemoDynamicsWorld3D*)m_dynamicsWorld)->debugDrawConstraints(gSelectedBatch, cBatchColorTab);
	}
	glColor3f(0, 0, 0);
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


void BasicDemo3D::outputDebugInfo(int & xOffset,int & yStart, int  yIncr)
{
	char buf[124];
	glDisable(GL_LIGHTING);
	glColor3f(0, 0, 0);
	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"mouse to interact");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"space to reset");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"cursor keys and z,x to navigate");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"i to toggle simulation, s single step");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"q to quit");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"h to toggle help text");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"p to toggle profiling (+results to file)");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"w to toggle wireframe/solid rendering");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"c to toggle constraint drawing");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"b to draw single constraint batch");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"u to toggle between CPU  and CUDA solvers");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"d to toggle between different batch builders");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	glRasterPos3f(xOffset,yStart,0);
	sprintf(buf,"m to toggle between CUDA / CPU motion integrators");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	if (getDynamicsWorld())
	{
		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"# objects = %d",getDynamicsWorld()->getNumCollisionObjects());
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"# pairs = %d",getDynamicsWorld()->getBroadphase()->getOverlappingPairCache()->getNumOverlappingPairs());
		GLDebugDrawString(xOffset,yStart,buf);


		yStart += yIncr;
		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"# skipped collisions=%d",gSkippedCol);
		GLDebugDrawString(xOffset,yStart,buf);

		yStart += yIncr;
		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"# processed collisions=%d",gProcessedCol);
		GLDebugDrawString(xOffset,yStart,buf);
		
		yStart += yIncr;
		glRasterPos3f(xOffset,yStart,0);
		btScalar fract = (gProcessedCol+gSkippedCol)? btScalar(gSkippedCol)/(gProcessedCol+gSkippedCol) : 0.f;
		sprintf(buf,"culled narrowphase collisions=%f",fract);
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
		
		gProcessedCol = 0;
		gSkippedCol = 0;

	}
} // BasicDemo3D::outputDebugInfo()

void BasicDemo3D::setWireMode(bool wireOnOff)
{
	int dbgDrawMode = m_dynamicsWorld->getDebugDrawer()->getDebugMode();
	if(wireOnOff)
	{
		dbgDrawMode |= btIDebugDraw::DBG_FastWireframe;
	}
	else
	{
		dbgDrawMode &= ~btIDebugDraw::DBG_FastWireframe;
	}
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(dbgDrawMode);
	m_debugMode = dbgDrawMode;
} // BasicDemo3D::setWireMode()
