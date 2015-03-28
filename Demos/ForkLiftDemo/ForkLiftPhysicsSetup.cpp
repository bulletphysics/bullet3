#include "ForkLiftPhysicsSetup.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "OpenGLWindow/CommonRenderInterface.h"

btScalar maxMotorImpulse = 1400.f;
btScalar loadMass = 350.f;//
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

float	defaultBreakingForce = 10.f;
float	gBreakingForce = 100.f;
float	gEngineForce = 0.f;
float	gVehicleSteering = 0.f;
float	steeringIncrement = 0.04f;
float	steeringClamp = 0.3f;
float	wheelRadius = 0.5f;
float	wheelWidth = 0.4f;
btScalar suspensionRestLength(0.6);
#define CUBE_HALF_EXTENTS 1
float	suspensionStiffness = 20.f;
float	suspensionDamping = 2.3f;
float	suspensionCompression = 4.4f;
float	rollInfluence = 0.1f;//1.0f;
float	wheelFriction = 1000;//BT_LARGE_FLOAT;


struct ForkLiftInternalData
{
	btRigidBody* m_carChassis;

//----------------------------
	btRigidBody* m_liftBody;
	btVector3	m_liftStartPos;
	btHingeConstraint* m_liftHinge;
	btRigidBody* m_forkBody;
	btVector3	m_forkStartPos;
	btSliderConstraint* m_forkSlider;
	btRigidBody* m_loadBody;
	btVector3	m_loadStartPos;
	bool m_useDefaultCamera;
	class btTriangleIndexVertexArray*	m_indexVertexArrays;
	btVector3*	m_vertices;
	btRaycastVehicle::btVehicleTuning	m_tuning;
	btVehicleRaycaster*	m_vehicleRayCaster;
	btRaycastVehicle*	m_vehicle;
	btCollisionShape*	m_wheelShape;
	float		m_cameraHeight;
	float	m_minCameraDistance;
	float	m_maxCameraDistance;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	class btBroadphaseInterface*	m_overlappingPairCache;
	class btCollisionDispatcher*	m_dispatcher;
	class btConstraintSolver*	m_constraintSolver;
	class btDefaultCollisionConfiguration* m_collisionConfiguration;
	class btDiscreteDynamicsWorld* m_dynamicsWorld;

	int m_wheelInstances[4];

	bool useMCLPSolver;

	ForkLiftInternalData()
		:m_carChassis(0),
		m_liftBody(0),
		m_forkBody(0),
		m_loadBody(0),
		m_indexVertexArrays(0),
		m_vertices(0),
		m_cameraHeight(4.f),
		m_minCameraDistance(3.f),
		m_maxCameraDistance(10.f),
		m_overlappingPairCache(0),
		m_dispatcher(0),
		m_constraintSolver(0),
		m_collisionConfiguration(0),
		m_dynamicsWorld(0),
		useMCLPSolver(false)
	{
		m_vehicle = 0;
		m_wheelShape = 0;
		m_useDefaultCamera = false;
	}
};

ForkLiftPhysicsSetup::ForkLiftPhysicsSetup()
{
	m_data = new ForkLiftInternalData;
}

ForkLiftPhysicsSetup::~ForkLiftPhysicsSetup()
{
	delete m_data;
}

	
void ForkLiftPhysicsSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{

	
#ifdef FORCE_ZAXIS_UP
	m_cameraUp = btVector3(0,0,1);
	m_forwardAxis = 1;
#endif

	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	m_data->m_collisionShapes.push_back(groundShape);
	m_data->m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_data->m_dispatcher = new btCollisionDispatcher(m_data->m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_data->m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
	if (m_data->useMCLPSolver)
	{
		btDantzigSolver* mlcp = new btDantzigSolver();
		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
		btMLCPSolver* sol = new btMLCPSolver(mlcp);
		m_data->m_constraintSolver = sol;
	} else
	{
		m_data->m_constraintSolver = new btSequentialImpulseConstraintSolver();
	}
	m_data->m_dynamicsWorld = new btDiscreteDynamicsWorld(m_data->m_dispatcher,m_data->m_overlappingPairCache,m_data->m_constraintSolver,m_data->m_collisionConfiguration);
	if (m_data->useMCLPSolver)
	{
		m_data->m_dynamicsWorld ->getSolverInfo().m_minimumSolverBatchSize = 1;//for direct solver it is better to have a small A matrix
	} else
	{
		m_data->m_dynamicsWorld ->getSolverInfo().m_minimumSolverBatchSize = 128;//for direct solver, it is better to solve multiple objects together, small batches have high overhead
	}
#ifdef FORCE_ZAXIS_UP
	m_dynamicsWorld->setGravity(btVector3(0,0,-10));
#endif 

	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
btTransform tr;
tr.setIdentity();
tr.setOrigin(btVector3(0,-3,0));

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
	m_data->m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_data->m_collisionShapes.push_back(compound);
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

	m_data->m_carChassis = localCreateRigidBody(800,tr,compound);//chassisShape);
	//m_carChassis->setDamping(0.2,0.2);
	
	m_data->m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
	gfxBridge.createCollisionShapeGraphicsObject(m_data->m_wheelShape);
	int wheelGraphicsIndex = m_data->m_wheelShape->getUserIndex();

	const float position[4]={0,10,10,0};
	const float quaternion[4]={0,0,0,1};
	const float color[4]={0,1,0,1};
	const float scaling[4] = {1,1,1,1};

	for (int i=0;i<4;i++)
	{
		m_data->m_wheelInstances[i] = gfxBridge.registerGraphicsInstance(wheelGraphicsIndex, position, quaternion, color, scaling);
	}


	{
		btCollisionShape* liftShape = new btBoxShape(btVector3(0.5f,2.0f,0.05f));
		m_data->m_collisionShapes.push_back(liftShape);
		btTransform liftTrans;
		m_data->m_liftStartPos = btVector3(0.0f, 2.5f, 3.05f);
		liftTrans.setIdentity();
		liftTrans.setOrigin(m_data->m_liftStartPos);
		m_data->m_liftBody = localCreateRigidBody(10,liftTrans, liftShape);

		btTransform localA, localB;
		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, SIMD_HALF_PI, 0);
		localA.setOrigin(btVector3(0.0, 1.0, 3.05));
		localB.getBasis().setEulerZYX(0, SIMD_HALF_PI, 0);
		localB.setOrigin(btVector3(0.0, -1.5, -0.05));
		m_data->m_liftHinge = new btHingeConstraint(*m_data->m_carChassis,*m_data->m_liftBody, localA, localB);
//		m_liftHinge->setLimit(-LIFT_EPS, LIFT_EPS);
		m_data->m_liftHinge->setLimit(0.0f, 0.0f);
		m_data->m_dynamicsWorld->addConstraint(m_data->m_liftHinge, true);

		btCollisionShape* forkShapeA = new btBoxShape(btVector3(1.0f,0.1f,0.1f));
		m_data->m_collisionShapes.push_back(forkShapeA);
		btCompoundShape* forkCompound = new btCompoundShape();
		m_data->m_collisionShapes.push_back(forkCompound);
		btTransform forkLocalTrans;
		forkLocalTrans.setIdentity();
		forkCompound->addChildShape(forkLocalTrans, forkShapeA);

		btCollisionShape* forkShapeB = new btBoxShape(btVector3(0.1f,0.02f,0.6f));
		m_data->m_collisionShapes.push_back(forkShapeB);
		forkLocalTrans.setIdentity();
		forkLocalTrans.setOrigin(btVector3(-0.9f, -0.08f, 0.7f));
		forkCompound->addChildShape(forkLocalTrans, forkShapeB);

		btCollisionShape* forkShapeC = new btBoxShape(btVector3(0.1f,0.02f,0.6f));
		m_data->m_collisionShapes.push_back(forkShapeC);
		forkLocalTrans.setIdentity();
		forkLocalTrans.setOrigin(btVector3(0.9f, -0.08f, 0.7f));
		forkCompound->addChildShape(forkLocalTrans, forkShapeC);

		btTransform forkTrans;
		m_data->m_forkStartPos = btVector3(0.0f, 0.6f, 3.2f);
		forkTrans.setIdentity();
		forkTrans.setOrigin(m_data->m_forkStartPos);
		m_data->m_forkBody = localCreateRigidBody(5, forkTrans, forkCompound);

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, SIMD_HALF_PI);
		localA.setOrigin(btVector3(0.0f, -1.9f, 0.05f));
		localB.getBasis().setEulerZYX(0, 0, SIMD_HALF_PI);
		localB.setOrigin(btVector3(0.0, 0.0, -0.1));
		m_data->m_forkSlider = new btSliderConstraint(*m_data->m_liftBody, *m_data->m_forkBody, localA, localB, true);
		m_data->m_forkSlider->setLowerLinLimit(0.1f);
		m_data->m_forkSlider->setUpperLinLimit(0.1f);
//		m_forkSlider->setLowerAngLimit(-LIFT_EPS);
//		m_forkSlider->setUpperAngLimit(LIFT_EPS);
		m_data->m_forkSlider->setLowerAngLimit(0.0f);
		m_data->m_forkSlider->setUpperAngLimit(0.0f);
		m_data->m_dynamicsWorld->addConstraint(m_data->m_forkSlider, true);


		btCompoundShape* loadCompound = new btCompoundShape();
		m_data->m_collisionShapes.push_back(loadCompound);
		btCollisionShape* loadShapeA = new btBoxShape(btVector3(2.0f,0.5f,0.5f));
		m_data->m_collisionShapes.push_back(loadShapeA);
		btTransform loadTrans;
		loadTrans.setIdentity();
		loadCompound->addChildShape(loadTrans, loadShapeA);
		btCollisionShape* loadShapeB = new btBoxShape(btVector3(0.1f,1.0f,1.0f));
		m_data->m_collisionShapes.push_back(loadShapeB);
		loadTrans.setIdentity();
		loadTrans.setOrigin(btVector3(2.1f, 0.0f, 0.0f));
		loadCompound->addChildShape(loadTrans, loadShapeB);
		btCollisionShape* loadShapeC = new btBoxShape(btVector3(0.1f,1.0f,1.0f));
		m_data->m_collisionShapes.push_back(loadShapeC);
		loadTrans.setIdentity();
		loadTrans.setOrigin(btVector3(-2.1f, 0.0f, 0.0f));
		loadCompound->addChildShape(loadTrans, loadShapeC);
		loadTrans.setIdentity();
		m_data->m_loadStartPos = btVector3(0.0f, 3.5f, 7.0f);
		loadTrans.setOrigin(m_data->m_loadStartPos);
		m_data->m_loadBody  = localCreateRigidBody(loadMass, loadTrans, loadCompound);
	}



	
	
	/// create vehicle
	{
		
		m_data->m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_data->m_dynamicsWorld);
		m_data->m_vehicle = new btRaycastVehicle(m_data->m_tuning,m_data->m_carChassis,m_data->m_vehicleRayCaster);
		
		///never deactivate the vehicle
		m_data->m_carChassis->setActivationState(DISABLE_DEACTIVATION);

		m_data->m_dynamicsWorld->addVehicle(m_data->m_vehicle);

		float connectionHeight = 1.2f;

	
		bool isFrontWheel=true;

		//choose coordinate system
		m_data->m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

#ifdef FORCE_ZAXIS_UP
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif

		m_data->m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_data->m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif

		m_data->m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_data->m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif //FORCE_ZAXIS_UP
		isFrontWheel = false;
		m_data->m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_data->m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif
		m_data->m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_data->m_tuning,isFrontWheel);
		
		for (int i=0;i<m_data->m_vehicle->getNumWheels();i++)
		{
			btWheelInfo& wheel = m_data->m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
	}

	resetForklift();
	gfxBridge.autogenerateGraphicsObjects(m_data->m_dynamicsWorld);

//	setCameraDistance(26.f);
}

void ForkLiftPhysicsSetup::resetForklift()
{
	gVehicleSteering = 0.f;
	gBreakingForce = defaultBreakingForce;
	gEngineForce = 0.f;

	m_data->m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_data->m_carChassis->setLinearVelocity(btVector3(0,0,0));
	m_data->m_carChassis->setAngularVelocity(btVector3(0,0,0));
	m_data->m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_data->m_carChassis->getBroadphaseHandle(),m_data->m_dynamicsWorld->getDispatcher());
	if (m_data->m_vehicle)
	{
		m_data->m_vehicle->resetSuspension();
		for (int i=0;i<m_data->m_vehicle->getNumWheels();i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_data->m_vehicle->updateWheelTransform(i,true);
		}
	}
	btTransform liftTrans;
	liftTrans.setIdentity();
	liftTrans.setOrigin(m_data->m_liftStartPos);
	m_data->m_liftBody->activate();
	m_data->m_liftBody->setCenterOfMassTransform(liftTrans);
	m_data->m_liftBody->setLinearVelocity(btVector3(0,0,0));
	m_data->m_liftBody->setAngularVelocity(btVector3(0,0,0));

	btTransform forkTrans;
	forkTrans.setIdentity();
	forkTrans.setOrigin(m_data->m_forkStartPos);
	m_data->m_forkBody->activate();
	m_data->m_forkBody->setCenterOfMassTransform(forkTrans);
	m_data->m_forkBody->setLinearVelocity(btVector3(0,0,0));
	m_data->m_forkBody->setAngularVelocity(btVector3(0,0,0));

//	m_liftHinge->setLimit(-LIFT_EPS, LIFT_EPS);
	m_data->m_liftHinge->setLimit(0.0f, 0.0f);
	m_data->m_liftHinge->enableAngularMotor(false, 0, 0);

	
	m_data->m_forkSlider->setLowerLinLimit(0.1f);
	m_data->m_forkSlider->setUpperLinLimit(0.1f);
	m_data->m_forkSlider->setPoweredLinMotor(false);

	btTransform loadTrans;
	loadTrans.setIdentity();
	loadTrans.setOrigin(m_data->m_loadStartPos);
	m_data->m_loadBody->activate();
	m_data->m_loadBody->setCenterOfMassTransform(loadTrans);
	m_data->m_loadBody->setLinearVelocity(btVector3(0,0,0));
	m_data->m_loadBody->setAngularVelocity(btVector3(0,0,0));

}

btRigidBody* ForkLiftPhysicsSetup::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

//#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	body->setContactProcessingThreshold(BT_LARGE_FLOAT);//m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);
	body->setWorldTransform(startTransform);
#endif//

	m_data->m_dynamicsWorld->addRigidBody(body);
	return body;
}

void ForkLiftPhysicsSetup::exitPhysics()
{
}
void ForkLiftPhysicsSetup::stepSimulation(float deltaTime)
{
	m_data->m_dynamicsWorld->stepSimulation(deltaTime);
}
void    ForkLiftPhysicsSetup::debugDraw(int debugDrawFlags)
{
}
bool ForkLiftPhysicsSetup::pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
{
	return false;
}
bool ForkLiftPhysicsSetup::movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
{
	return false;
}
void ForkLiftPhysicsSetup::removePickingConstraint()
{
}
void ForkLiftPhysicsSetup::syncPhysicsToGraphics(GraphicsPhysicsBridge& gfxBridge)
{
	gfxBridge.syncPhysicsToGraphics(m_data->m_dynamicsWorld);
	//sync wheels

	for (int i=0;i<m_data->m_vehicle->getNumWheels();i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_data->m_vehicle->updateWheelTransform(i,true);

		CommonRenderInterface* renderer = gfxBridge.getRenderInterface();
		if (renderer)
		{
			btTransform tr = m_data->m_vehicle->getWheelInfo(i).m_worldTransform;
			btVector3 pos=tr.getOrigin();
			btQuaternion orn = tr.getRotation();
			renderer->writeSingleInstanceTransformToCPU(pos,orn,m_data->m_wheelInstances[i]);
		}
	}

}

void ForkLiftPhysicsSetup::renderScene(GraphicsPhysicsBridge& gfxBridge)
{
	gfxBridge.drawText3D("hi!",0,10,10,2);
}

void ForkLiftPhysicsSetup::lockLiftHinge()
{
}

void ForkLiftPhysicsSetup::lockForkSlider()
{
}
