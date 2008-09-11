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

//enable just one, DO_BENCHMARK_PYRAMIDS or DO_WALL
//#define DO_BENCHMARK_PYRAMIDS 1
#define DO_WALL 1

//Note: some of those settings need 'DO_WALL' demo
//#define USE_KINEMATIC_GROUND 1
//#define PRINT_CONTACT_STATISTICS 1
//#define USER_DEFINED_FRICTION_MODEL 1
//#define USE_CUSTOM_NEAR_CALLBACK 1
//#define CENTER_OF_MASS_SHIFT 1
//#define VERBOSE_TIMESTEPPING_CONSOLEOUTPUT 1

//#define USE_PARALLEL_DISPATCHER 1
//#define USE_PARALLEL_SOLVER 1 //experimental parallel solver


//from Bullet 2.68 onwards ODE Quickstep constraint solver is optional part of Bullet, re-distributed under the ZLib license with permission of Russell L. Smith
//#define COMPARE_WITH_QUICKSTEP 1


#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"

#ifdef USE_PARALLEL_DISPATCHER
#include "../../Extras/BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#ifdef WIN32
#include "../../Extras/BulletMultiThreaded/Win32ThreadSupport.h"
#include "../../Extras/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif //WIN32

#ifdef USE_LIBSPE2
#include "../../Extras/BulletMultiThreaded/SpuLibspe2Support.h"
#endif //USE_LIBSPE2

#ifdef USE_PARALLEL_SOLVER
#include "../../Extras/BulletMultiThreaded/SpuParallelSolver.h"
#include "../../Extras/BulletMultiThreaded/SpuSolverTask/SpuParallellSolverTask.h"
#endif //USE_PARALLEL_SOLVER

#endif//USE_PARALLEL_DISPATCHER



#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"


#include "BMF_Api.h"
#include <stdio.h> //printf debugging

static float	gCollisionMargin = 0.05f;
#include "CcdPhysicsDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"

btTransform comOffset;
btVector3 comOffsetVec(0,2,0);

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;
const int maxOverlap = 65535;

bool createConstraint = true;//false;
#ifdef CENTER_OF_MASS_SHIFT
bool useCompound = true;
#else
bool useCompound = false;
#endif



#ifdef _DEBUG
const int gNumObjects = 120;
#else
const int gNumObjects = 120;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif


const int maxNumObjects = 32760;

int	shapeIndex[maxNumObjects];


#define CUBE_HALF_EXTENTS 0.5

#define EXTRA_HEIGHT -10.f
//GL_LineSegmentShape shapeE(btPoint3(-50,0,0),
//						   btPoint3(50,0,0));


void CcdPhysicsDemo::createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos )
{
	btTransform trans;
	trans.setIdentity();

	for(int i=0; i<size; i++)
	{
		// This constructs a row, from left to right
		int rowSize = size - i;
		for(int j=0; j< rowSize; j++)
		{
			btVector4 pos;
			pos.setValue(
				-rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
				halfCubeSize + i * halfCubeSize * 2.0f,
				zPos);

			trans.setOrigin(pos);
			btScalar mass = 1.f;

			btRigidBody* body = 0;
			body = localCreateRigidBody(mass,trans,boxShape);
#ifdef USER_DEFINED_FRICTION_MODEL
		///Advanced use: override the friction solver
		body->m_frictionSolverType = USER_CONTACT_SOLVER_TYPE1;
#endif //USER_DEFINED_FRICTION_MODEL

		}
	}
}


////////////////////////////////////



//by default, Bullet will use its own nearcallback, but you can override it using dispatcher->setNearCallback()
void customNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)
{
		btCollisionObject* colObj0 = (btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
		btCollisionObject* colObj1 = (btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

		if (dispatcher.needsCollision(colObj0,colObj1))
		{
			//dispatcher will keep algorithms persistent in the collision pair
			if (!collisionPair.m_algorithm)
			{
				collisionPair.m_algorithm = dispatcher.findAlgorithm(colObj0,colObj1);
			}

			if (collisionPair.m_algorithm)
			{
				btManifoldResult contactPointResult(colObj0,colObj1);

				if (dispatchInfo.m_dispatchFunc == 		btDispatcherInfo::DISPATCH_DISCRETE)
				{
					//discrete collision detection query
					collisionPair.m_algorithm->processCollision(colObj0,colObj1,dispatchInfo,&contactPointResult);
				} else
				{
					//continuous collision detection query, time of impact (toi)
					float toi = collisionPair.m_algorithm->calculateTimeOfImpact(colObj0,colObj1,dispatchInfo,&contactPointResult);
					if (dispatchInfo.m_timeOfImpact > toi)
						dispatchInfo.m_timeOfImpact = toi;

				}
			}
		}

}






//experimental jitter damping (1 = no damping, 0 = total damping once motion below threshold)
extern btScalar gJitterVelocityDampingFactor;




extern int gNumManifold;
extern int gOverlappingPairs;
extern int gTotalContactPoints;

void CcdPhysicsDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


#ifdef USE_KINEMATIC_GROUND
	//btQuaternion kinRotation(btVector3(0,0,1),0.);
	btVector3 kinTranslation(-0.01,0,0);
	//kinematic object
	btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[0];
	//is this a rigidbody with a motionstate? then use the motionstate to update positions!
	if (btRigidBody::upcast(colObj) && btRigidBody::upcast(colObj)->getMotionState())
	{
		btTransform newTrans;
		btRigidBody::upcast(colObj)->getMotionState()->getWorldTransform(newTrans);
		newTrans.getOrigin()+=kinTranslation;
		btRigidBody::upcast(colObj)->getMotionState()->setWorldTransform(newTrans);
	} else
	{
		m_dynamicsWorld->getCollisionObjectArray()[0]->getWorldTransform().getOrigin() += kinTranslation;
	}

#endif //USE_KINEMATIC_GROUND


	float dt = getDeltaTimeMicroseconds() * 0.000001f;

//	printf("dt = %f: ",dt);

	if (m_dynamicsWorld)
	{

//#define FIXED_STEP 1
#ifdef FIXED_STEP
  		m_dynamicsWorld->stepSimulation(1.0f/60.f,0);

#else
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			dt = 1.0/420.f;

		int numSimSteps = 0;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);

		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();

#ifdef VERBOSE_TIMESTEPPING_CONSOLEOUTPUT
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_TIMESTEPPING_CONSOLEOUTPUT

#endif
	}

#ifdef USE_QUICKPROF
        btProfiler::beginBlock("render");
#endif //USE_QUICKPROF

	renderme();


	//render the graphics objects, with center of mass shift

		updateCamera();



#ifdef USE_QUICKPROF
        btProfiler::endBlock("render");
#endif
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



void CcdPhysicsDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}





///User-defined friction model, the most simple friction model available: no friction
float myFrictionModel(	btRigidBody& body1,	btRigidBody& body2,	btManifoldPoint& contactPoint,	const btContactSolverInfo& solverInfo	)
{
	//don't do any friction
	return 0.f;
}

void	CcdPhysicsDemo::initPhysics()
{
	setTexturing(true);
	setShadows(false);

#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	m_threadSupportSolver = 0;
	m_threadSupportCollision = 0;
#endif //
#endif

//#define USE_GROUND_PLANE 1
#ifdef USE_GROUND_PLANE
	m_collisionShapes.push_back(new btStaticPlaneShape(btVector3(0,1,0),0.5));
#else

	///Please don't make the box sizes larger then 1000: the collision detection will be inaccurate.
	///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=346
	m_collisionShapes.push_back(new btBoxShape (btVector3(200,CUBE_HALF_EXTENTS,200)));
#endif

#ifdef DO_BENCHMARK_PYRAMIDS
	m_collisionShapes.push_back(new btBoxShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
#else
//	m_collisionShapes.push_back(new btBoxShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
	m_collisionShapes.push_back(new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
#endif



#ifdef DO_BENCHMARK_PYRAMIDS
	setCameraDistance(32.5f);
#endif

#ifdef DO_BENCHMARK_PYRAMIDS
	m_azi = 90.f;
#endif //DO_BENCHMARK_PYRAMIDS

	m_dispatcher=0;
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

#ifdef USE_PARALLEL_DISPATCHER
int maxNumOutstandingTasks = 4;

#ifdef USE_WIN32_THREADING

	m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#else

#ifdef USE_LIBSPE2

   spe_program_handle_t * program_handle;
#ifndef USE_CESOF
                        program_handle = spe_image_open ("./spuCollision.elf");
                        if (program_handle == NULL)
                    {
                                perror( "SPU OPEN IMAGE ERROR\n");
                    }
                        else
                        {
                                printf( "IMAGE OPENED\n");
                        }
#else
                        extern spe_program_handle_t spu_program;
                        program_handle = &spu_program;
#endif
        SpuLibspe2Support* threadSupportCollision  = new SpuLibspe2Support( program_handle, maxNumOutstandingTasks);
#endif //USE_LIBSPE2

///Playstation 3 SPU (SPURS)  version is available through PS3 Devnet
/// For Unix/Mac someone could implement a pthreads version of btThreadSupportInterface?
///you can hook it up to your custom task scheduler by deriving from btThreadSupportInterface
#endif


	m_dispatcher = new	SpuGatheringCollisionDispatcher(m_threadSupportCollision,maxNumOutstandingTasks,m_collisionConfiguration);
//	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#else

	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER

#ifdef USE_CUSTOM_NEAR_CALLBACK
	//this is optional
	m_dispatcher->setNearCallback(customNearCallback);
#endif


	m_broadphase = new btDbvtBroadphase();

#ifdef COMPARE_WITH_QUICKSTEP
	m_solver = new btOdeQuickstepConstraintSolver();
#else


#ifdef USE_PARALLEL_SOLVER

	m_threadSupportSolver = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"solver",
								processSolverTask,
								createSolverLocalStoreMemory,
								maxNumOutstandingTasks));

	m_solver = new btParallelSequentialImpulseSolver(m_threadSupportSolver,maxNumOutstandingTasks);
#else
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;//new btOdeQuickstepConstraintSolver();

#endif //USE_PARALLEL_SOLVER

#endif



		btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
		m_dynamicsWorld = world;

#ifdef	USER_DEFINED_FRICTION_MODEL
	//user defined friction model is not supported in 'cache friendly' solver yet, so switch to old solver

		world->getSolverInfo().m_solverMode = SOLVER_RANDMIZE_ORDER;

#endif //USER_DEFINED_FRICTION_MODEL

#ifdef DO_BENCHMARK_PYRAMIDS
		world->getSolverInfo().m_numIterations = 4;
#endif //DO_BENCHMARK_PYRAMIDS

		m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
		m_dynamicsWorld->setGravity(btVector3(0,-10,0));



#ifdef USER_DEFINED_FRICTION_MODEL
	{
		//m_solver->setContactSolverFunc(ContactSolverFunc func,USER_CONTACT_SOLVER_TYPE1,DEFAULT_CONTACT_SOLVER_TYPE);
		solver->SetFrictionSolverFunc(myFrictionModel,USER_CONTACT_SOLVER_TYPE1,DEFAULT_CONTACT_SOLVER_TYPE);
		solver->SetFrictionSolverFunc(myFrictionModel,DEFAULT_CONTACT_SOLVER_TYPE,USER_CONTACT_SOLVER_TYPE1);
		solver->SetFrictionSolverFunc(myFrictionModel,USER_CONTACT_SOLVER_TYPE1,USER_CONTACT_SOLVER_TYPE1);
		//m_physicsEnvironmentPtr->setNumIterations(2);
	}
#endif //USER_DEFINED_FRICTION_MODEL



	int i;

	btTransform tr;
	tr.setIdentity();


	for (i=0;i<gNumObjects;i++)
	{
		if (i>0)
		{
			shapeIndex[i] = 1;//sphere
		}
		else
			shapeIndex[i] = 0;
	}

	if (useCompound)
	{
		btCompoundShape* compoundShape = new btCompoundShape();
		btCollisionShape* oldShape = m_collisionShapes[1];
		m_collisionShapes[1] = compoundShape;
		btVector3 sphereOffset(0,0,2);

		comOffset.setIdentity();

#ifdef CENTER_OF_MASS_SHIFT
		comOffset.setOrigin(comOffsetVec);
		compoundShape->addChildShape(comOffset,oldShape);

#else
		compoundShape->addChildShape(tr,oldShape);
		tr.setOrigin(sphereOffset);
		compoundShape->addChildShape(tr,new btSphereShape(0.9));
#endif
	}

#ifdef DO_WALL

	for (i=0;i<gNumObjects;i++)
	{
		btCollisionShape* shape = m_collisionShapes[shapeIndex[i]];
		shape->setMargin(gCollisionMargin);

		bool isDyna = i>0;

		btTransform trans;
		trans.setIdentity();

		if (i>0)
		{
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
		} else
		{
			trans.setOrigin(btVector3(0,EXTRA_HEIGHT-CUBE_HALF_EXTENTS,0));
		}

		float mass = 1.f;

		if (!isDyna)
			mass = 0.f;

		btRigidBody* body = localCreateRigidBody(mass,trans,shape);
#ifdef USE_KINEMATIC_GROUND
		if (mass == 0.f)
		{
			body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState(DISABLE_DEACTIVATION);
		}
#endif //USE_KINEMATIC_GROUND


		// Only do CCD if  motion in one timestep (1.f/60.f) exceeds CUBE_HALF_EXTENTS
		body->setCcdMotionThreshold( CUBE_HALF_EXTENTS );

		//Experimental: better estimation of CCD Time of Impact:
		body->setCcdSweptSphereRadius( 0.2*CUBE_HALF_EXTENTS );

#ifdef USER_DEFINED_FRICTION_MODEL
		///Advanced use: override the friction solver
		body->m_frictionSolverType = USER_CONTACT_SOLVER_TYPE1;
#endif //USER_DEFINED_FRICTION_MODEL

	}
#endif


#ifdef DO_BENCHMARK_PYRAMIDS
	btTransform trans;
	trans.setIdentity();

	btScalar halfExtents = CUBE_HALF_EXTENTS;

	trans.setOrigin(btVector3(0,-halfExtents,0));



	localCreateRigidBody(0.f,trans,m_collisionShapes[shapeIndex[0]]);

	int numWalls = 15;
	int wallHeight = 15;
	float wallDistance = 3;


	for (int i=0;i<numWalls;i++)
	{
		float zPos = (i-numWalls/2) * wallDistance;
		createStack(m_collisionShapes[shapeIndex[1]],halfExtents,wallHeight,zPos);
	}
//	createStack(m_collisionShapes[shapeIndex[1]],halfExtends,20,10);

//	createStack(m_collisionShapes[shapeIndex[1]],halfExtends,20,20);
#define DESTROYER_BALL 1
#ifdef DESTROYER_BALL
	btTransform sphereTrans;
	sphereTrans.setIdentity();
	sphereTrans.setOrigin(btVector3(0,2,40));
	btSphereShape* ball = new btSphereShape(2.f);
	m_collisionShapes.push_back(ball);
	btRigidBody* ballBody = localCreateRigidBody(10000.f,sphereTrans,ball);
	ballBody->setLinearVelocity(btVector3(0,0,-10));
#endif
#endif //DO_BENCHMARK_PYRAMIDS
//	clientResetScene();


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
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;
#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	if (m_threadSupportSolver)
	{
		delete m_threadSupportSolver;
	}
#endif
#endif

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	if (m_threadSupportCollision)
	{
		delete m_threadSupportCollision;
	}
#endif
#endif

	delete m_collisionConfiguration;


}


