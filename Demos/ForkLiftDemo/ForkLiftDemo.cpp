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

///September 2006: VehicleDemo is work in progress, this file is mostly just a placeholder
///This VehicleDemo file is very early in development, please check it later
///@todo is a basic engine model:
///A function that maps user input (throttle) into torque/force applied on the wheels
///with gears etc.
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "GLDebugFont.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

//#define LIFT_EPS 0.0000001f
//
// By default, Bullet Vehicle uses Y as up axis.
// You can override the up axis, for example Z-axis up. Enable this define to see how to:
//#define FORCE_ZAXIS_UP 1
//

#ifdef FORCE_ZAXIS_UP
		int rightIndex = 0; 
		int upIndex = 2; 
		int forwardIndex = 1;
		btVector3 wheelDirectionCS0(0,0,-1);
		btVector3 wheelAxleCS(1,0,0);
#else
		int rightIndex = 0;
		int upIndex = 1;
		int forwardIndex = 2;
		btVector3 wheelDirectionCS0(0,-1,0);
		btVector3 wheelAxleCS(-1,0,0);
#endif

#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging

#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"
#include "ForkLiftDemo.h"


const int maxProxies = 32766;
const int maxOverlap = 65535;

///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts
float	gEngineForce = 0.f;

float	defaultBreakingForce = 10.f;
float	gBreakingForce = 100.f;

float	maxEngineForce = 1000.f;//this should be engine/velocity dependent
float	maxBreakingForce = 100.f;

float	gVehicleSteering = 0.f;
float	steeringIncrement = 0.04f;
float	steeringClamp = 0.3f;
float	wheelRadius = 0.5f;
float	wheelWidth = 0.4f;
float	wheelFriction = 1000;//BT_LARGE_FLOAT;
float	suspensionStiffness = 20.f;
float	suspensionDamping = 2.3f;
float	suspensionCompression = 4.4f;
float	rollInfluence = 0.1f;//1.0f;


btScalar suspensionRestLength(0.6);

#define CUBE_HALF_EXTENTS 1



////////////////////////////////////




ForkLiftDemo::ForkLiftDemo()
:
m_carChassis(0),
m_liftBody(0),
m_forkBody(0),
m_loadBody(0),
m_indexVertexArrays(0),
m_vertices(0),
m_cameraHeight(4.f),
m_minCameraDistance(3.f),
m_maxCameraDistance(10.f)
{
	m_vehicle = 0;
	m_wheelShape = 0;
	m_cameraPosition = btVector3(30,30,30);
	m_useDefaultCamera = false;
	setTexturing(true);
	setShadows(true);

}


void ForkLiftDemo::termPhysics()
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

	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete m_dynamicsWorld;

	delete m_vehicleRayCaster;

	delete m_vehicle;
	
	delete m_wheelShape;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

}

ForkLiftDemo::~ForkLiftDemo()
{
	termPhysics();
}

void ForkLiftDemo::initPhysics()
{
	
#ifdef FORCE_ZAXIS_UP
	m_cameraUp = btVector3(0,0,1);
	m_forwardAxis = 1;
#endif

	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	m_collisionShapes.push_back(groundShape);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
#ifdef FORCE_ZAXIS_UP
	m_dynamicsWorld->setGravity(btVector3(0,0,-10));
#endif 

	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
btTransform tr;
tr.setIdentity();
tr.setOrigin(btVector3(0,-10,0));

//either use heightfield or triangle mesh


	//create ground object
	localCreateRigidBody(0,tr,groundShape);

#ifdef FORCE_ZAXIS_UP
//   indexRightAxis = 0; 
//   indexUpAxis = 2; 
//   indexForwardAxis = 1; 
	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,2.f, 0.5f));
	btCompoundShape* compound = new btCompoundShape();
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0,0,1));
#else
	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f));
	m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0,1,0));
#endif

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

	m_carChassis = localCreateRigidBody(800,tr,compound);//chassisShape);
	//m_carChassis->setDamping(0.2,0.2);
	
	m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));

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
		m_loadStartPos = btVector3(0.0f, -3.5f, 7.0f);
		loadTrans.setOrigin(m_loadStartPos);
		m_loadBody  = localCreateRigidBody(4, loadTrans, loadCompound);
	}



	clientResetScene();

	/// create vehicle
	{
		
		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);
		
		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);

		m_dynamicsWorld->addVehicle(m_vehicle);

		float connectionHeight = 1.2f;

	
		bool isFrontWheel=true;

		//choose coordinate system
		m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

#ifdef FORCE_ZAXIS_UP
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif

		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif

		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif //FORCE_ZAXIS_UP
		isFrontWheel = false;
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
		
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
	}

	
	setCameraDistance(26.f);

}


//to be implemented by the demo
void ForkLiftDemo::renderme()
{
	
	updateCamera();

	ATTRIBUTE_ALIGNED16(btScalar) m[16];
	int i;

	btVector3 wheelColor(1,0,0);

	btVector3	worldBoundsMin,worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);



	for (i=0;i<m_vehicle->getNumWheels();i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i,true);
		//draw wheels (cylinders)
		m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
		m_shapeDrawer->drawOpenGL(m,m_wheelShape,wheelColor,getDebugMode(),worldBoundsMin,worldBoundsMax);
	}


	int lineWidth=250;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if((getDebugMode() & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"SHIFT+Cursor Left/Right - rotate lift");
		GLDebugDrawString(xStart,20,buf);
		yStart+=20;
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"SHIFT+Cursor UP/Down - move fork up/down");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"F5 - toggle camera mode");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
        sprintf(buf,"Click inside this window for keyboard focus");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);


		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}
	DemoApplication::renderme();
}

void ForkLiftDemo::clientMoveAndDisplay()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	
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


	float dt = getDeltaTimeMicroseconds() * 0.000001f;
	
	if (m_dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 2;
		if (m_idle)
			dt = 1.0/420.f;

		int numSimSteps;
        numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);


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

	
	




#ifdef USE_QUICKPROF 
        btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 


	renderme(); 

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

#ifdef USE_QUICKPROF 
        btProfiler::endBlock("render"); 
#endif 
	

	glFlush();
	glutSwapBuffers();

}



void ForkLiftDemo::displayCallback(void) 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}



void ForkLiftDemo::clientResetScene()
{
	gVehicleSteering = 0.f;
	gBreakingForce = defaultBreakingForce;
	gEngineForce = 0.f;

	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setLinearVelocity(btVector3(0,0,0));
	m_carChassis->setAngularVelocity(btVector3(0,0,0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),getDynamicsWorld()->getDispatcher());
	if (m_vehicle)
	{
		m_vehicle->resetSuspension();
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i,true);
		}
	}
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



void ForkLiftDemo::specialKeyboardUp(int key, int x, int y)
{
   switch (key) 
	{
	case GLUT_KEY_UP :
		{
			lockForkSlider();
			gEngineForce = 0.f;
			gBreakingForce = defaultBreakingForce; 
		break;
		}
	case GLUT_KEY_DOWN :
		{
			lockForkSlider();
			gEngineForce = 0.f;
			gBreakingForce = defaultBreakingForce;
		break;
		}
	case GLUT_KEY_LEFT:
	case GLUT_KEY_RIGHT:
		{
			lockLiftHinge();
			break;
		}
	default:
		DemoApplication::specialKeyboardUp(key,x,y);
		break;
	}
}


void ForkLiftDemo::specialKeyboard(int key, int x, int y)
{

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
					m_liftHinge->enableAngularMotor(true, -0.1, 10.0);
					break;
				}
			case GLUT_KEY_RIGHT : 
				{
					
					m_liftHinge->setLimit(-M_PI/16.0f, M_PI/8.0f);
					m_liftHinge->enableAngularMotor(true, 0.1, 10.0);
					break;
				}
			case GLUT_KEY_UP :
				{
					m_forkSlider->setLowerLinLimit(0.1f);
					m_forkSlider->setUpperLinLimit(3.9f);
					m_forkSlider->setPoweredLinMotor(true);
					m_forkSlider->setMaxLinMotorForce(10.0);
					m_forkSlider->setTargetLinMotorVelocity(1.0);
					break;
				}
			case GLUT_KEY_DOWN :
				{
					m_forkSlider->setLowerLinLimit(0.1f);
					m_forkSlider->setUpperLinLimit(3.9f);
					m_forkSlider->setPoweredLinMotor(true);
					m_forkSlider->setMaxLinMotorForce(10.0);
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

			case GLUT_KEY_F5:
				m_useDefaultCamera = !m_useDefaultCamera;
				break;
			default:
				DemoApplication::specialKeyboard(key,x,y);
				break;
			}

	}
	//	glutPostRedisplay();


}

void	ForkLiftDemo::updateCamera()
{
	
//#define DISABLE_CAMERA 1
	if(m_useDefaultCamera)
	{
		DemoApplication::updateCamera();
		return;
	}

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	btTransform chassisWorldTrans;

	//look at the vehicle
	m_carChassis->getMotionState()->getWorldTransform(chassisWorldTrans);
	m_cameraTargetPosition = chassisWorldTrans.getOrigin();

	//interpolate the camera height
#ifdef FORCE_ZAXIS_UP
	m_cameraPosition[2] = (15.0*m_cameraPosition[2] + m_cameraTargetPosition[2] + m_cameraHeight)/16.0;
#else
	m_cameraPosition[1] = (15.0*m_cameraPosition[1] + m_cameraTargetPosition[1] + m_cameraHeight)/16.0;
#endif 

	btVector3 camToObject = m_cameraTargetPosition - m_cameraPosition;

	//keep distance between min and max distance
	float cameraDistance = camToObject.length();
	float correctionFactor = 0.f;
	if (cameraDistance < m_minCameraDistance)
	{
		correctionFactor = 0.15*(m_minCameraDistance-cameraDistance)/cameraDistance;
	}
	if (cameraDistance > m_maxCameraDistance)
	{
		correctionFactor = 0.15*(m_maxCameraDistance-cameraDistance)/cameraDistance;
	}
	m_cameraPosition -= correctionFactor*camToObject;
	
	//update OpenGL camera settings
	btScalar aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
	glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);

	 glMatrixMode(GL_MODELVIEW);
	 glLoadIdentity();

    gluLookAt(m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2],
		      m_cameraTargetPosition[0],m_cameraTargetPosition[1], m_cameraTargetPosition[2],
			  m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
  


}

void ForkLiftDemo::lockLiftHinge(void)
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
} // ForkLiftDemo::lockLiftHinge()

void ForkLiftDemo::lockForkSlider(void)
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
} // ForkLiftDemo::lockForkSlider()
