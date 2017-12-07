/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2015 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///May 2015: implemented the wheels using the Hinge2Constraint
///todo: add controls for the motors etc.

#include "Hinge2Vehicle.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"


#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

class btVehicleTuning;

class btCollisionShape;


#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

class Hinge2Vehicle  : public CommonRigidBodyBase
{
	public:

		/* extra stuff*/
	btVector3 m_cameraPosition;
	
	btRigidBody* m_carChassis;
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

	GUIHelperInterface* m_guiHelper;
	int m_wheelInstances[4];

//----------------------------
	btRigidBody* m_liftBody;
	btVector3	m_liftStartPos;
	btHingeConstraint* m_liftHinge;

	btRigidBody* m_forkBody;
	btVector3	m_forkStartPos;
	btSliderConstraint* m_forkSlider;

	btRigidBody* m_loadBody;
	btVector3	m_loadStartPos;

	void lockLiftHinge(void);
	void lockForkSlider(void);

	bool m_useDefaultCamera;
//----------------------------


		class btTriangleIndexVertexArray*	m_indexVertexArrays;

	btVector3*	m_vertices;

	
	
	btCollisionShape*	m_wheelShape;

	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;


	Hinge2Vehicle(struct GUIHelperInterface* helper);

	virtual ~Hinge2Vehicle();

	virtual void stepSimulation(float deltaTime);
	
	virtual void	resetForklift();
		
	virtual void clientResetScene();

	virtual void displayCallback();
	
	
	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y);

	
	virtual bool	keyboardCallback(int key, int state);

	virtual void renderScene();

	virtual void physicsDebugDraw(int debugFlags);
	

	void initPhysics();
	void exitPhysics();

	virtual void resetCamera()
	{
		float dist = 8;
		float pitch = -32;
		float yaw = -45;
		float targetPos[3]={-0.33,-0.72,4.5};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}

	/*static DemoApplication* Create()
	{
		Hinge2Vehicle* demo = new Hinge2Vehicle();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	*/
};


static btScalar maxMotorImpulse = 4000.f;

//the sequential impulse solver has difficulties dealing with large mass ratios (differences), between loadMass and the fork parts
static btScalar loadMass = 350.f;//
//btScalar loadMass = 10.f;//this should work fine for the SI solver


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif


//static int rightIndex = 0;
//static int upIndex = 1;
//static int forwardIndex = 2;
static btVector3 wheelDirectionCS0(0,-1,0);
static btVector3 wheelAxleCS(-1,0,0);

static bool useMCLPSolver = false;//true;


#include <stdio.h> //printf debugging


#include "Hinge2Vehicle.h"


//static const int maxProxies = 32766;
//static const int maxOverlap = 65535;

static float	gEngineForce = 0.f;

static float	defaultBreakingForce = 10.f;
static float	gBreakingForce = 100.f;

static float	maxEngineForce = 1000.f;//this should be engine/velocity dependent
//static float	maxBreakingForce = 100.f;

static float	gVehicleSteering = 0.f;
static float	steeringIncrement = 0.04f;
static float	steeringClamp = 0.3f;
static float	wheelRadius = 0.5f;
static float	wheelWidth = 0.4f;
//static float	wheelFriction = 1000;//BT_LARGE_FLOAT;
//static float	suspensionStiffness = 20.f;
//static float	suspensionDamping = 2.3f;
//static float	suspensionCompression = 4.4f;
//static float	rollInfluence = 0.1f;//1.0f;


//static btScalar suspensionRestLength(0.6);

#define CUBE_HALF_EXTENTS 1



////////////////////////////////////




Hinge2Vehicle::Hinge2Vehicle(struct GUIHelperInterface* helper)
:CommonRigidBodyBase(helper),
m_carChassis(0),
m_guiHelper(helper),
m_liftBody(0),
m_forkBody(0),
m_loadBody(0),
m_indexVertexArrays(0),
m_vertices(0),
m_cameraHeight(4.f),
m_minCameraDistance(3.f),
m_maxCameraDistance(10.f)
{
	helper->setUpAxis(1);

	m_wheelShape = 0;
	m_cameraPosition = btVector3(30,30,30);
	m_useDefaultCamera = false;
//	setTexturing(true);
//	setShadows(true);

}


void Hinge2Vehicle::exitPhysics()
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

			while (body->getNumConstraintRefs())
			{
				btTypedConstraint* constraint = body->getConstraintRef(0);
				m_dynamicsWorld->removeConstraint(constraint);
				delete constraint;
			}
			delete body->getMotionState();
			m_dynamicsWorld->removeRigidBody(body);
		} else
		{
			m_dynamicsWorld->removeCollisionObject( obj );
		}
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete m_dynamicsWorld;
	m_dynamicsWorld=0;

	
	delete m_wheelShape;
	m_wheelShape=0;

	//delete solver
	delete m_solver;
	m_solver=0;

	//delete broadphase
	delete m_broadphase;
	m_broadphase=0;

	//delete dispatcher
	delete m_dispatcher;
	m_dispatcher=0;

	delete m_collisionConfiguration;
	m_collisionConfiguration=0;

}

Hinge2Vehicle::~Hinge2Vehicle()
{
	//exitPhysics();
}



void Hinge2Vehicle::initPhysics()
{
	m_guiHelper->setUpAxis(1);


	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	m_collisionShapes.push_back(groundShape);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_broadphase = new btAxisSweep3(worldMin,worldMax);
	if (useMCLPSolver)
	{
		btDantzigSolver* mlcp = new btDantzigSolver();
		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
		btMLCPSolver* sol = new btMLCPSolver(mlcp);
		m_solver = sol;
	} else
	{
		m_solver = new btSequentialImpulseConstraintSolver();
	}
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	if (useMCLPSolver)
	{
		m_dynamicsWorld ->getSolverInfo().m_minimumSolverBatchSize = 1;//for direct solver it is better to have a small A matrix
	} else
	{
		m_dynamicsWorld ->getSolverInfo().m_minimumSolverBatchSize = 128;//for direct solver, it is better to solve multiple objects together, small batches have high overhead
	}
	m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	

	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
btTransform tr;
tr.setIdentity();
tr.setOrigin(btVector3(0,-3,0));

//either use heightfield or triangle mesh


	//create ground object
	localCreateRigidBody(0,tr,groundShape);

	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f));
	m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0,1,0));

	compound->addChildShape(localTrans,chassisShape);

	{
		btCollisionShape* suppShape = new btBoxShape(btVector3(0.5f,0.1f,0.5f));
		btTransform suppLocalTrans;
		suppLocalTrans.setIdentity();
		//localTrans effectively shifts the center of mass with respect to the chassis
		suppLocalTrans.setOrigin(btVector3(0,1.0,2.5));
		compound->addChildShape(suppLocalTrans, suppShape);
	}

	tr.setOrigin(btVector3(0,0.f,0));

	btScalar chassisMass = 800;
	m_carChassis = localCreateRigidBody(chassisMass,tr,compound);//chassisShape);
	//m_carChassis->setDamping(0.2,0.2);
	
	//m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
	m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
	

	//const float position[4]={0,10,10,0};
	//const float quaternion[4]={0,0,0,1};
	//const float color[4]={0,1,0,1};
	//const float scaling[4] = {1,1,1,1};

	btVector3 wheelPos[4] = {
		btVector3(btScalar(-1.), btScalar(-0.25), btScalar(1.25)),
		btVector3(btScalar(1.), btScalar(-0.25), btScalar(1.25)),
		btVector3(btScalar(1.), btScalar(-0.25), btScalar(-1.25)),
		btVector3(btScalar(-1.), btScalar(-0.25), btScalar(-1.25))
	};

	
	for (int i=0;i<4;i++)
	{ 
		// create a Hinge2 joint
		// create two rigid bodies
		// static bodyA (parent) on top:
		
		
		btRigidBody* pBodyA = this->m_carChassis;//m_chassis;//createRigidBody( 0.0, tr, m_wheelShape);
		pBodyA->setActivationState(DISABLE_DEACTIVATION);
		// dynamic bodyB (child) below it :
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(wheelPos[i]);
		
		btRigidBody* pBodyB = createRigidBody(10.0, tr, m_wheelShape);
		pBodyB->setFriction(1110);
		pBodyB->setActivationState(DISABLE_DEACTIVATION);
		// add some data to build constraint frames
		btVector3 parentAxis(0.f, 1.f, 0.f); 
		btVector3 childAxis(1.f, 0.f, 0.f); 
		btVector3 anchor = tr.getOrigin();//(0.f, 0.f, 0.f);
		btHinge2Constraint* pHinge2 = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, parentAxis, childAxis);
		
		//m_guiHelper->get2dCanvasInterface();
		

		pHinge2->setLowerLimit(-SIMD_HALF_PI * 0.5f);
		pHinge2->setUpperLimit( SIMD_HALF_PI * 0.5f);
		// add constraint to world
		m_dynamicsWorld->addConstraint(pHinge2, true);
		// draw constraint frames and limits for debugging
		{
			int motorAxis = 3;
			pHinge2->enableMotor(motorAxis,true);
			pHinge2->setMaxMotorForce(motorAxis,1000);
			pHinge2->setTargetVelocity(motorAxis,-1);
		}

		{
			int motorAxis = 5;
			pHinge2->enableMotor(motorAxis,true);
			pHinge2->setMaxMotorForce(motorAxis,1000);
			pHinge2->setTargetVelocity(motorAxis,0);
		}

		pHinge2->setDbgDrawSize(btScalar(5.f));
	}


	{
		btCollisionShape* liftShape = new btBoxShape(btVector3(0.5f,2.0f,0.05f));
		m_collisionShapes.push_back(liftShape);
		btTransform liftTrans;
		m_liftStartPos = btVector3(0.0f, 2.5f, 3.05f);
		liftTrans.setIdentity();
		liftTrans.setOrigin(m_liftStartPos);
		m_liftBody = localCreateRigidBody(10,liftTrans, liftShape);

		btTransform localA, localB;
		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, M_PI_2, 0);
		localA.setOrigin(btVector3(0.0, 1.0, 3.05));
		localB.getBasis().setEulerZYX(0, M_PI_2, 0);
		localB.setOrigin(btVector3(0.0, -1.5, -0.05));
		m_liftHinge = new btHingeConstraint(*m_carChassis,*m_liftBody, localA, localB);
//		m_liftHinge->setLimit(-LIFT_EPS, LIFT_EPS);
		m_liftHinge->setLimit(0.0f, 0.0f);
		m_dynamicsWorld->addConstraint(m_liftHinge, true);

		btCollisionShape* forkShapeA = new btBoxShape(btVector3(1.0f,0.1f,0.1f));
		m_collisionShapes.push_back(forkShapeA);
		btCompoundShape* forkCompound = new btCompoundShape();
		m_collisionShapes.push_back(forkCompound);
		btTransform forkLocalTrans;
		forkLocalTrans.setIdentity();
		forkCompound->addChildShape(forkLocalTrans, forkShapeA);

		btCollisionShape* forkShapeB = new btBoxShape(btVector3(0.1f,0.02f,0.6f));
		m_collisionShapes.push_back(forkShapeB);
		forkLocalTrans.setIdentity();
		forkLocalTrans.setOrigin(btVector3(-0.9f, -0.08f, 0.7f));
		forkCompound->addChildShape(forkLocalTrans, forkShapeB);

		btCollisionShape* forkShapeC = new btBoxShape(btVector3(0.1f,0.02f,0.6f));
		m_collisionShapes.push_back(forkShapeC);
		forkLocalTrans.setIdentity();
		forkLocalTrans.setOrigin(btVector3(0.9f, -0.08f, 0.7f));
		forkCompound->addChildShape(forkLocalTrans, forkShapeC);

		btTransform forkTrans;
		m_forkStartPos = btVector3(0.0f, 0.6f, 3.2f);
		forkTrans.setIdentity();
		forkTrans.setOrigin(m_forkStartPos);
		m_forkBody = localCreateRigidBody(5, forkTrans, forkCompound);

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, M_PI_2);
		localA.setOrigin(btVector3(0.0f, -1.9f, 0.05f));
		localB.getBasis().setEulerZYX(0, 0, M_PI_2);
		localB.setOrigin(btVector3(0.0, 0.0, -0.1));
		m_forkSlider = new btSliderConstraint(*m_liftBody, *m_forkBody, localA, localB, true);
		m_forkSlider->setLowerLinLimit(0.1f);
		m_forkSlider->setUpperLinLimit(0.1f);
//		m_forkSlider->setLowerAngLimit(-LIFT_EPS);
//		m_forkSlider->setUpperAngLimit(LIFT_EPS);
		m_forkSlider->setLowerAngLimit(0.0f);
		m_forkSlider->setUpperAngLimit(0.0f);
		m_dynamicsWorld->addConstraint(m_forkSlider, true);


		btCompoundShape* loadCompound = new btCompoundShape();
		m_collisionShapes.push_back(loadCompound);
		btCollisionShape* loadShapeA = new btBoxShape(btVector3(2.0f,0.5f,0.5f));
		m_collisionShapes.push_back(loadShapeA);
		btTransform loadTrans;
		loadTrans.setIdentity();
		loadCompound->addChildShape(loadTrans, loadShapeA);
		btCollisionShape* loadShapeB = new btBoxShape(btVector3(0.1f,1.0f,1.0f));
		m_collisionShapes.push_back(loadShapeB);
		loadTrans.setIdentity();
		loadTrans.setOrigin(btVector3(2.1f, 0.0f, 0.0f));
		loadCompound->addChildShape(loadTrans, loadShapeB);
		btCollisionShape* loadShapeC = new btBoxShape(btVector3(0.1f,1.0f,1.0f));
		m_collisionShapes.push_back(loadShapeC);
		loadTrans.setIdentity();
		loadTrans.setOrigin(btVector3(-2.1f, 0.0f, 0.0f));
		loadCompound->addChildShape(loadTrans, loadShapeC);
		loadTrans.setIdentity();
		m_loadStartPos = btVector3(0.0f, 3.5f, 7.0f);
		loadTrans.setOrigin(m_loadStartPos);
		m_loadBody  = localCreateRigidBody(loadMass, loadTrans, loadCompound);
	}



	
	resetForklift();
	
//	setCameraDistance(26.f);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Hinge2Vehicle::physicsDebugDraw(int debugFlags)
{
	if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		m_dynamicsWorld->debugDrawWorld();
	}
}

//to be implemented by the demo
void Hinge2Vehicle::renderScene()
{
	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
#if 0
	for (int i=0;i<m_vehicle->getNumWheels();i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i,true);

		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		if (renderer)
		{
			btTransform tr = m_vehicle->getWheelInfo(i).m_worldTransform;
			btVector3 pos=tr.getOrigin();
			btQuaternion orn = tr.getRotation();
			renderer->writeSingleInstanceTransformToCPU(pos,orn,m_wheelInstances[i]);
		}
	}
#endif
	
	m_guiHelper->render(m_dynamicsWorld);

	btVector3 wheelColor(1,0,0);

	btVector3	worldBoundsMin,worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);




#if 0
	int lineWidth=400;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if((getDebugMode() & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		sprintf(buf,"SHIFT+Cursor Left/Right - rotate lift");
		GLDebugDrawString(xStart,20,buf);
		yStart+=20;
		sprintf(buf,"SHIFT+Cursor UP/Down - fork up/down");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);

		if (m_useDefaultCamera)
		{
			sprintf(buf,"F5 - camera mode (free)");
		} else
		{
			sprintf(buf,"F5 - camera mode (follow)");
		}
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);

		yStart+=20;
		if (m_dynamicsWorld->getConstraintSolver()->getSolverType()==BT_MLCP_SOLVER)
		{
			sprintf(buf,"F6 - solver (direct MLCP)");
		} else
		{
			sprintf(buf,"F6 - solver (sequential impulse)");
		}
		GLDebugDrawString(xStart,yStart,buf);
		btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*) m_dynamicsWorld;
		if (world->getLatencyMotionStateInterpolation())
		{
			sprintf(buf,"F7 - motionstate interpolation (on)");
		} else
		{
			sprintf(buf,"F7 - motionstate interpolation (off)");
		}
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);

		sprintf(buf,"Click window for keyboard focus");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);


		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}
#endif
}

void Hinge2Vehicle::stepSimulation(float deltaTime)
{

	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

#if 0
	{			
		int wheelIndex = 2;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 3;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);


		wheelIndex = 0;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		wheelIndex = 1;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);

	}
#endif

	float dt = deltaTime;
	
	if (m_dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps =  2;
		
		int numSimSteps;
        numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);

		if (m_dynamicsWorld->getConstraintSolver()->getSolverType()==BT_MLCP_SOLVER)
		{
			btMLCPSolver* sol = (btMLCPSolver*) m_dynamicsWorld->getConstraintSolver();
			int numFallbacks = sol->getNumFallbacks();
			if (numFallbacks)
			{
				static int totalFailures = 0;
				totalFailures+=numFallbacks;
				printf("MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n",totalFailures);
			}
			sol->setNumFallbacks(0);
		}


//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
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
#endif //VERBOSE_FEEDBACK

	}

	
	

}



void Hinge2Vehicle::displayCallback(void) 
{
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//renderme();

//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

//	glFlush();
//	glutSwapBuffers();
}


void Hinge2Vehicle::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void Hinge2Vehicle::resetForklift()
{
	gVehicleSteering = 0.f;
	gBreakingForce = defaultBreakingForce;
	gEngineForce = 0.f;

	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setLinearVelocity(btVector3(0,0,0));
	m_carChassis->setAngularVelocity(btVector3(0,0,0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),getDynamicsWorld()->getDispatcher());
#if 0
	if (m_vehicle)
	{
		m_vehicle->resetSuspension();
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i,true);
		}
	}
#endif
	btTransform liftTrans;
	liftTrans.setIdentity();
	liftTrans.setOrigin(m_liftStartPos);
	m_liftBody->activate();
	m_liftBody->setCenterOfMassTransform(liftTrans);
	m_liftBody->setLinearVelocity(btVector3(0,0,0));
	m_liftBody->setAngularVelocity(btVector3(0,0,0));

	btTransform forkTrans;
	forkTrans.setIdentity();
	forkTrans.setOrigin(m_forkStartPos);
	m_forkBody->activate();
	m_forkBody->setCenterOfMassTransform(forkTrans);
	m_forkBody->setLinearVelocity(btVector3(0,0,0));
	m_forkBody->setAngularVelocity(btVector3(0,0,0));

//	m_liftHinge->setLimit(-LIFT_EPS, LIFT_EPS);
	m_liftHinge->setLimit(0.0f, 0.0f);
	m_liftHinge->enableAngularMotor(false, 0, 0);

	
	m_forkSlider->setLowerLinLimit(0.1f);
	m_forkSlider->setUpperLinLimit(0.1f);
	m_forkSlider->setPoweredLinMotor(false);

	btTransform loadTrans;
	loadTrans.setIdentity();
	loadTrans.setOrigin(m_loadStartPos);
	m_loadBody->activate();
	m_loadBody->setCenterOfMassTransform(loadTrans);
	m_loadBody->setLinearVelocity(btVector3(0,0,0));
	m_loadBody->setAngularVelocity(btVector3(0,0,0));

}


bool	Hinge2Vehicle::keyboardCallback(int key, int state)
{
	bool handled = false;
	bool isShiftPressed = m_guiHelper->getAppInterface()->m_window->isModifierKeyPressed(B3G_SHIFT);

	if (state)
	{
	if (isShiftPressed) 
	{
		switch (key) 
			{
			case B3G_LEFT_ARROW : 
				{
				
					m_liftHinge->setLimit(-M_PI/16.0f, M_PI/8.0f);
					m_liftHinge->enableAngularMotor(true, -0.1, maxMotorImpulse);
					handled = true;
					break;
				}
			case B3G_RIGHT_ARROW : 
				{
					
					m_liftHinge->setLimit(-M_PI/16.0f, M_PI/8.0f);
					m_liftHinge->enableAngularMotor(true, 0.1, maxMotorImpulse);
					handled = true;
					break;
				}
			case B3G_UP_ARROW :
				{
					m_forkSlider->setLowerLinLimit(0.1f);
					m_forkSlider->setUpperLinLimit(3.9f);
					m_forkSlider->setPoweredLinMotor(true);
					m_forkSlider->setMaxLinMotorForce(maxMotorImpulse);
					m_forkSlider->setTargetLinMotorVelocity(1.0);
					handled = true;
					break;
				}
			case B3G_DOWN_ARROW :
				{
					m_forkSlider->setLowerLinLimit(0.1f);
					m_forkSlider->setUpperLinLimit(3.9f);
					m_forkSlider->setPoweredLinMotor(true);
					m_forkSlider->setMaxLinMotorForce(maxMotorImpulse);
					m_forkSlider->setTargetLinMotorVelocity(-1.0);
					handled = true;
					break;
				}
			}

	} else
	{
			switch (key) 
			{
			case B3G_LEFT_ARROW : 
				{
					handled = true;
					gVehicleSteering += steeringIncrement;
					if (	gVehicleSteering > steeringClamp)
						gVehicleSteering = steeringClamp;

					break;
				}
			case B3G_RIGHT_ARROW : 
				{
					handled = true;
					gVehicleSteering -= steeringIncrement;
					if (	gVehicleSteering < -steeringClamp)
						gVehicleSteering = -steeringClamp;

					break;
				}
			case B3G_UP_ARROW :
				{
					handled = true;
					gEngineForce = maxEngineForce;
					gBreakingForce = 0.f;
					break;
				}
			case B3G_DOWN_ARROW :
				{
					handled = true;
					gEngineForce = -maxEngineForce;
					gBreakingForce = 0.f;
					break;
				}

			case B3G_F7:
				{
					handled = true;
					btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*)m_dynamicsWorld;
					world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
					printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
					break;
				}
			case B3G_F6:
				{
					handled = true;
					//switch solver (needs demo restart)
					useMCLPSolver = !useMCLPSolver;
					printf("switching to useMLCPSolver = %d\n", useMCLPSolver);

					delete m_solver;
					if (useMCLPSolver)
					{
						btDantzigSolver* mlcp = new btDantzigSolver();
						//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
						btMLCPSolver* sol = new btMLCPSolver(mlcp);
						m_solver = sol;
					} else
					{
						m_solver = new btSequentialImpulseConstraintSolver();
					}

					m_dynamicsWorld->setConstraintSolver(m_solver);


					//exitPhysics();
					//initPhysics();
					break;
				}

			case B3G_F5:
			handled = true;
				m_useDefaultCamera = !m_useDefaultCamera;
				break;
			default:
				break;
			}
	}

	} else
	{
		switch (key) 
		{
		case B3G_UP_ARROW:
			{
				lockForkSlider();
				gEngineForce = 0.f;
				gBreakingForce = defaultBreakingForce; 
				handled=true;
			break;
			}
		case B3G_DOWN_ARROW:
			{
				lockForkSlider();
				gEngineForce = 0.f;
				gBreakingForce = defaultBreakingForce;
				handled=true;
			break;
			}
		case B3G_LEFT_ARROW:
		case B3G_RIGHT_ARROW:
			{
				lockLiftHinge();
				handled=true;
				break;
			}
		default:
			
			break;
		}
	}
	return handled;
}

void Hinge2Vehicle::specialKeyboardUp(int key, int x, int y)
{
#if 0
   
#endif
}


void Hinge2Vehicle::specialKeyboard(int key, int x, int y)
{
#if 0
	if (key==GLUT_KEY_END)
		return;

	//	printf("key = %i x=%i y=%i\n",key,x,y);

	int state;
	state=glutGetModifiers();
	if (state & GLUT_ACTIVE_SHIFT) 
	{
		switch (key) 
			{
			case GLUT_KEY_LEFT : 
				{
				
					m_liftHinge->setLimit(-M_PI/16.0f, M_PI/8.0f);
					m_liftHinge->enableAngularMotor(true, -0.1, maxMotorImpulse);
					break;
				}
			case GLUT_KEY_RIGHT : 
				{
					
					m_liftHinge->setLimit(-M_PI/16.0f, M_PI/8.0f);
					m_liftHinge->enableAngularMotor(true, 0.1, maxMotorImpulse);
					break;
				}
			case GLUT_KEY_UP :
				{
					m_forkSlider->setLowerLinLimit(0.1f);
					m_forkSlider->setUpperLinLimit(3.9f);
					m_forkSlider->setPoweredLinMotor(true);
					m_forkSlider->setMaxLinMotorForce(maxMotorImpulse);
					m_forkSlider->setTargetLinMotorVelocity(1.0);
					break;
				}
			case GLUT_KEY_DOWN :
				{
					m_forkSlider->setLowerLinLimit(0.1f);
					m_forkSlider->setUpperLinLimit(3.9f);
					m_forkSlider->setPoweredLinMotor(true);
					m_forkSlider->setMaxLinMotorForce(maxMotorImpulse);
					m_forkSlider->setTargetLinMotorVelocity(-1.0);
					break;
				}

			default:
				DemoApplication::specialKeyboard(key,x,y);
				break;
			}

	} else
	{
			switch (key) 
			{
			case GLUT_KEY_LEFT : 
				{
					gVehicleSteering += steeringIncrement;
					if (	gVehicleSteering > steeringClamp)
						gVehicleSteering = steeringClamp;

					break;
				}
			case GLUT_KEY_RIGHT : 
				{
					gVehicleSteering -= steeringIncrement;
					if (	gVehicleSteering < -steeringClamp)
						gVehicleSteering = -steeringClamp;

					break;
				}
			case GLUT_KEY_UP :
				{
					gEngineForce = maxEngineForce;
					gBreakingForce = 0.f;
					break;
				}
			case GLUT_KEY_DOWN :
				{
					gEngineForce = -maxEngineForce;
					gBreakingForce = 0.f;
					break;
				}

			case GLUT_KEY_F7:
				{
					btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*)m_dynamicsWorld;
					world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
					printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
					break;
				}
			case GLUT_KEY_F6:
				{
					//switch solver (needs demo restart)
					useMCLPSolver = !useMCLPSolver;
					printf("switching to useMLCPSolver = %d\n", useMCLPSolver);

					delete m_solver;
					if (useMCLPSolver)
					{
						btDantzigSolver* mlcp = new btDantzigSolver();
						//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
						btMLCPSolver* sol = new btMLCPSolver(mlcp);
						m_solver = sol;
					} else
					{
						m_solver = new btSequentialImpulseConstraintSolver();
					}

					m_dynamicsWorld->setConstraintSolver(m_solver);


					//exitPhysics();
					//initPhysics();
					break;
				}

			case GLUT_KEY_F5:
				m_useDefaultCamera = !m_useDefaultCamera;
				break;
			default:
				DemoApplication::specialKeyboard(key,x,y);
				break;
			}

	}
	//	glutPostRedisplay();

#endif
}


void Hinge2Vehicle::lockLiftHinge(void)
{
	btScalar hingeAngle = m_liftHinge->getHingeAngle();
	btScalar lowLim = m_liftHinge->getLowerLimit();
	btScalar hiLim = m_liftHinge->getUpperLimit();
	m_liftHinge->enableAngularMotor(false, 0, 0);
	if(hingeAngle < lowLim)
	{
//		m_liftHinge->setLimit(lowLim, lowLim + LIFT_EPS);
		m_liftHinge->setLimit(lowLim, lowLim);
	}
	else if(hingeAngle > hiLim)
	{
//		m_liftHinge->setLimit(hiLim - LIFT_EPS, hiLim);
		m_liftHinge->setLimit(hiLim, hiLim);
	}
	else
	{
//		m_liftHinge->setLimit(hingeAngle - LIFT_EPS, hingeAngle + LIFT_EPS);
		m_liftHinge->setLimit(hingeAngle, hingeAngle);
	}
	return;
} // Hinge2Vehicle::lockLiftHinge()

void Hinge2Vehicle::lockForkSlider(void)
{
	btScalar linDepth = m_forkSlider->getLinearPos();
	btScalar lowLim = m_forkSlider->getLowerLinLimit();
	btScalar hiLim = m_forkSlider->getUpperLinLimit();
	m_forkSlider->setPoweredLinMotor(false);
	if(linDepth <= lowLim)
	{
		m_forkSlider->setLowerLinLimit(lowLim);
		m_forkSlider->setUpperLinLimit(lowLim);
	}
	else if(linDepth > hiLim)
	{
		m_forkSlider->setLowerLinLimit(hiLim);
		m_forkSlider->setUpperLinLimit(hiLim);
	}
	else
	{
		m_forkSlider->setLowerLinLimit(linDepth);
		m_forkSlider->setUpperLinLimit(linDepth);
	}
	return;
} // Hinge2Vehicle::lockForkSlider()

btRigidBody* Hinge2Vehicle::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);
	body->setWorldTransform(startTransform);
#endif//

	m_dynamicsWorld->addRigidBody(body);
	return body;
}

CommonExampleInterface*    Hinge2VehicleCreateFunc(struct CommonExampleOptions& options)
{
	return new Hinge2Vehicle(options.m_guiHelper);
}
