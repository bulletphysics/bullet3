#include "Dof6Spring2Setup.h"

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"

#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif


extern float g_additionalBodyMass;

//comment this out to compare with original spring constraint
#define USE_6DOF2
#ifdef USE_6DOF2
	#define CONSTRAINT_TYPE btGeneric6DofSpring2Constraint
	#define EXTRAPARAMS
#else
	#define CONSTRAINT_TYPE btGeneric6DofSpringConstraint
	#define EXTRAPARAMS ,true
#endif

#include "../CommonInterfaces/CommonRigidBodyBase.h"




struct Dof6Spring2Setup : public CommonRigidBodyBase
{
	struct Dof6Spring2SetupInternalData* m_data;

	Dof6Spring2Setup(struct GUIHelperInterface* helper);
	virtual ~Dof6Spring2Setup();
	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);

	void animate();

	virtual void resetCamera()
	{
		float dist = 5;
		float pitch = 722;
		float yaw = 35;
		float targetPos[3]={4,2,-11};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
};



struct Dof6Spring2SetupInternalData
{
		btRigidBody*              m_TranslateSpringBody;
	btRigidBody*              m_TranslateSpringBody2;
	btRigidBody*              m_RotateSpringBody;
	btRigidBody*              m_RotateSpringBody2;
	btRigidBody*              m_BouncingTranslateBody;
	btRigidBody*              m_MotorBody;
	btRigidBody*              m_ServoMotorBody;
	btRigidBody*              m_ChainLeftBody;
	btRigidBody*              m_ChainRightBody;
	CONSTRAINT_TYPE*          m_ServoMotorConstraint;
	CONSTRAINT_TYPE*          m_ChainLeftConstraint;
	CONSTRAINT_TYPE*          m_ChainRightConstraint;

	
	float mDt;

	unsigned int frameID;
	Dof6Spring2SetupInternalData()
		:	mDt(1./60.),frameID(0)
	{
	}
};

Dof6Spring2Setup::Dof6Spring2Setup(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper)
{
	m_data = new Dof6Spring2SetupInternalData;
}
Dof6Spring2Setup::~Dof6Spring2Setup()
{
	exitPhysics();
	delete m_data;
}
void Dof6Spring2Setup::initPhysics()
{
	// Setup the basic world
	
	m_guiHelper->setUpAxis(1);

		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
		btVector3 worldAabbMin(-10000,-10000,-10000);
		btVector3 worldAabbMax(10000,10000,10000);
		m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

/////// uncomment the corresponding line to test a solver.
		//m_solver = new btSequentialImpulseConstraintSolver;
		m_solver = new btNNCGConstraintSolver;
		//m_solver = new btMLCPSolver(new btSolveProjectedGaussSeidel());
		//m_solver = new btMLCPSolver(new btDantzigSolver());
		//m_solver = new btMLCPSolver(new btLemkeSolver());

		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
		m_dynamicsWorld->getDispatchInfo().m_useContinuous = true;
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	
		m_dynamicsWorld->setGravity(btVector3(0,0,0));

		// Setup a big ground box
		{
			btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(5.),btScalar(200.)));
			btTransform groundTransform;
			groundTransform.setIdentity();
			groundTransform.setOrigin(btVector3(0,-10,0));
#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
			btCollisionObject* fixedGround = new btCollisionObject();
			fixedGround->setCollisionShape(groundShape);
			fixedGround->setWorldTransform(groundTransform);
			m_dynamicsWorld->addCollisionObject(fixedGround);
#else
			localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT
		}

		m_dynamicsWorld->getSolverInfo().m_numIterations = 100;

		btCollisionShape* shape;
		btVector3 localInertia(0,0,0);
		btDefaultMotionState* motionState;
		btTransform bodyTransform;
		btScalar mass;
		btTransform localA;
		btTransform localB;
		CONSTRAINT_TYPE* constraint;

		//static body centered in the origo
		mass = 0.0;
		shape= new btBoxShape(btVector3(0.5,0.5,0.5));
		localInertia = btVector3(0,0,0);
		bodyTransform.setIdentity();
		motionState = new btDefaultMotionState(bodyTransform);
		btRigidBody* staticBody = new btRigidBody(mass,motionState,shape,localInertia);

/////////// box with undamped translate spring attached to static body
/////////// the box should oscillate left-to-right forever
		{
			mass = 1.0;
			shape= new btBoxShape(btVector3(0.5,0.5,0.5));
			shape->calculateLocalInertia(mass,localInertia);
			bodyTransform.setIdentity();
			bodyTransform.setOrigin(btVector3(-2,0,-5));
			motionState = new btDefaultMotionState(bodyTransform);
			m_data->m_TranslateSpringBody = new btRigidBody(mass,motionState,shape,localInertia);
			m_data->m_TranslateSpringBody->setActivationState(DISABLE_DEACTIVATION);
			m_dynamicsWorld->addRigidBody(m_data->m_TranslateSpringBody);
 			localA.setIdentity();localA.getOrigin() = btVector3(0,0,-5);
			localB.setIdentity();
 			constraint =  new CONSTRAINT_TYPE(*staticBody, *m_data->m_TranslateSpringBody, localA, localB EXTRAPARAMS);
			constraint->setLimit(0, 1,-1);
			constraint->setLimit(1, 0, 0);
			constraint->setLimit(2, 0, 0);
			constraint->setLimit(3, 0, 0);
			constraint->setLimit(4, 0, 0);
			constraint->setLimit(5, 0, 0);
			constraint->enableSpring(0, true);
			constraint->setStiffness(0, 100);
	#ifdef USE_6DOF2
			constraint->setDamping(0, 0);
	#else
			constraint->setDamping(0, 1);
	#endif
			constraint->setEquilibriumPoint(0, 0);
			constraint->setDbgDrawSize(btScalar(2.f));
			m_dynamicsWorld->addConstraint(constraint, true);  
		}

/////////// box with rotate spring, attached to static body
/////////// box should swing (rotate) left-to-right forever
		{
			mass = 1.0;
			shape= new btBoxShape(btVector3(0.5,0.5,0.5));
			shape->calculateLocalInertia(mass,localInertia);
			bodyTransform.setIdentity();
			bodyTransform.getBasis().setEulerZYX(0,0,M_PI_2);
			motionState = new btDefaultMotionState(bodyTransform);
			m_data->m_RotateSpringBody = new btRigidBody(mass,motionState,shape,localInertia);
			m_data->m_RotateSpringBody->setActivationState(DISABLE_DEACTIVATION);
			m_dynamicsWorld->addRigidBody(m_data->m_RotateSpringBody);
			localA.setIdentity();localA.getOrigin() = btVector3(0,0,0);
			localB.setIdentity();localB.setOrigin(btVector3(0,0.5,0));
			constraint =  new CONSTRAINT_TYPE(*staticBody, *m_data->m_RotateSpringBody, localA, localB EXTRAPARAMS);
			constraint->setLimit(0, 0, 0);
			constraint->setLimit(1, 0, 0);
			constraint->setLimit(2, 0, 0);
			constraint->setLimit(3, 0, 0);
			constraint->setLimit(4, 0, 0);
			constraint->setLimit(5, 1, -1);
			constraint->enableSpring(5, true);
			constraint->setStiffness(5, 100);
	#ifdef USE_6DOF2
			constraint->setDamping(5, 0);
	#else
			constraint->setDamping(5, 1);
	#endif
			constraint->setEquilibriumPoint(0, 0);
			constraint->setDbgDrawSize(btScalar(2.f));
			m_dynamicsWorld->addConstraint(constraint, true);
		}

/////////// box with bouncing constraint, translation is bounced at the positive x limit, but not at the negative limit
/////////// bouncing can not be set independently at low and high limits, so two constraints will be created: one that defines the low (non bouncing) limit, and one that defines the high (bouncing) limit
/////////// the box should move to the left (as an impulse will be applied to it periodically) until it reaches its limit, then bounce back 
		{
			mass = 1.0;
			shape= new btBoxShape(btVector3(0.5,0.5,0.5));
			shape->calculateLocalInertia(mass,localInertia);
			bodyTransform.setIdentity();
			bodyTransform.setOrigin(btVector3(0,0,-3));
			motionState = new btDefaultMotionState(bodyTransform);
			m_data->m_BouncingTranslateBody = new btRigidBody(mass,motionState,shape,localInertia);
			m_data->m_BouncingTranslateBody->setActivationState(DISABLE_DEACTIVATION);
			m_data->m_BouncingTranslateBody->setDeactivationTime(btScalar(20000000));
			m_dynamicsWorld->addRigidBody(m_data->m_BouncingTranslateBody);
			localA.setIdentity();localA.getOrigin() = btVector3(0,0,0);
			localB.setIdentity();
			constraint =  new CONSTRAINT_TYPE(*staticBody, *m_data->m_BouncingTranslateBody, localA, localB EXTRAPARAMS);
			constraint->setLimit(0, -2, SIMD_INFINITY);
			constraint->setLimit(1, 0, 0);
			constraint->setLimit(2, -3, -3);
			constraint->setLimit(3, 0, 0);
			constraint->setLimit(4, 0, 0);
			constraint->setLimit(5, 0, 0);
		#ifdef USE_6DOF2
			constraint->setBounce(0,0);
		#else //bounce is named restitution in 6dofspring, but not implemented for translational limit motor, so the following line has no effect
			constraint->getTranslationalLimitMotor()->m_restitution = 0.0;
		#endif
			constraint->setParam(BT_CONSTRAINT_STOP_ERP,0.995,0);
			constraint->setParam(BT_CONSTRAINT_STOP_CFM,0.0,0);
			constraint->setDbgDrawSize(btScalar(2.f));
			m_dynamicsWorld->addConstraint(constraint, true);
			constraint =  new CONSTRAINT_TYPE(*staticBody, *m_data->m_BouncingTranslateBody, localA, localB EXTRAPARAMS);
			constraint->setLimit(0, -SIMD_INFINITY, 2);
			constraint->setLimit(1, 0, 0);
			constraint->setLimit(2, -3, -3);
			constraint->setLimit(3, 0, 0);
			constraint->setLimit(4, 0, 0);
			constraint->setLimit(5, 0, 0);
		#ifdef USE_6DOF2
			constraint->setBounce(0,1);
		#else //bounce is named restitution in 6dofspring, but not implemented for translational limit motor, so the following line has no effect
			constraint->getTranslationalLimitMotor()->m_restitution = 1.0;
		#endif
			constraint->setParam(BT_CONSTRAINT_STOP_ERP,0.995,0);
			constraint->setParam(BT_CONSTRAINT_STOP_CFM,0.0,0);
			constraint->setDbgDrawSize(btScalar(2.f));
			m_dynamicsWorld->addConstraint(constraint, true);
		}

/////////// box with rotational motor, attached to static body
/////////// the box should rotate around the y axis
		{
			mass = 1.0;
			shape= new btBoxShape(btVector3(0.5,0.5,0.5));
			shape->calculateLocalInertia(mass,localInertia);
			bodyTransform.setIdentity();
			bodyTransform.setOrigin(btVector3(4,0,0));
			motionState = new btDefaultMotionState(bodyTransform);
			m_data->m_MotorBody = new btRigidBody(mass,motionState,shape,localInertia);
			m_data->m_MotorBody->setActivationState(DISABLE_DEACTIVATION);
			m_dynamicsWorld->addRigidBody(m_data->m_MotorBody);
			localA.setIdentity();localA.getOrigin() = btVector3(4,0,0);
			localB.setIdentity();
			constraint =  new CONSTRAINT_TYPE(*staticBody, *m_data->m_MotorBody, localA, localB EXTRAPARAMS);
			constraint->setLimit(0, 0, 0);
			constraint->setLimit(1, 0, 0);
			constraint->setLimit(2, 0, 0);
			constraint->setLimit(3, 0, 0);
			constraint->setLimit(4, 0, 0);
			constraint->setLimit(5, 1,-1);
		#ifdef USE_6DOF2
			constraint->enableMotor(5,true);
			constraint->setTargetVelocity(5,3.f);
			constraint->setMaxMotorForce(5,10.f);
		#else
			constraint->getRotationalLimitMotor(2)->m_enableMotor = true;
			constraint->getRotationalLimitMotor(2)->m_targetVelocity = 3.f;
			constraint->getRotationalLimitMotor(2)->m_maxMotorForce = 10;
		#endif
			constraint->setDbgDrawSize(btScalar(2.f));
			m_dynamicsWorld->addConstraint(constraint, true);
		}

/////////// box with rotational servo motor, attached to static body
/////////// the box should rotate around the y axis until it reaches its target
/////////// the target will be negated periodically
		{
			mass = 1.0;
			shape= new btBoxShape(btVector3(0.5,0.5,0.5));
			shape->calculateLocalInertia(mass,localInertia);
			bodyTransform.setIdentity();
			bodyTransform.setOrigin(btVector3(7,0,0));
			motionState = new btDefaultMotionState(bodyTransform);
			m_data->m_ServoMotorBody = new btRigidBody(mass,motionState,shape,localInertia);
			m_data->m_ServoMotorBody->setActivationState(DISABLE_DEACTIVATION);
			m_dynamicsWorld->addRigidBody(m_data->m_ServoMotorBody);
			localA.setIdentity();localA.getOrigin() = btVector3(7,0,0);
			localB.setIdentity();
			constraint =  new CONSTRAINT_TYPE(*staticBody, *m_data->m_ServoMotorBody, localA, localB EXTRAPARAMS);
			constraint->setLimit(0, 0, 0);
			constraint->setLimit(1, 0, 0);
			constraint->setLimit(2, 0, 0);
			constraint->setLimit(3, 0, 0);
			constraint->setLimit(4, 0, 0);
			constraint->setLimit(5, 1,-1);
		#ifdef USE_6DOF2
			constraint->enableMotor(5,true);
			constraint->setTargetVelocity(5,3.f);
			constraint->setMaxMotorForce(5,10.f);
			constraint->setServo(5,true);
			constraint->setServoTarget(5, M_PI_2);
		#else
			constraint->getRotationalLimitMotor(2)->m_enableMotor = true;
			constraint->getRotationalLimitMotor(2)->m_targetVelocity = 3.f;
			constraint->getRotationalLimitMotor(2)->m_maxMotorForce = 10;
			//servo motor is not implemented in 6dofspring constraint
		#endif
			constraint->setDbgDrawSize(btScalar(2.f));
			m_dynamicsWorld->addConstraint(constraint, true);
			m_data->m_ServoMotorConstraint = constraint;
		}

////////// chain of boxes linked together with fully limited rotational and translational constraints
////////// the chain will be pulled to the left and to the right periodically. They should strictly stick together.
		{
			btScalar limitConstraintStrength = 0.6;
			int bodycount = 10;
			btRigidBody* prevBody = 0;
			for(int i = 0; i < bodycount; ++i)
			{
				mass = 1.0;
				shape= new btBoxShape(btVector3(0.5,0.5,0.5));
				shape->calculateLocalInertia(mass,localInertia);
				bodyTransform.setIdentity();
				bodyTransform.setOrigin(btVector3(- i,0,3));
				motionState = new btDefaultMotionState(bodyTransform);
				btRigidBody* body = new btRigidBody(mass,motionState,shape,localInertia);
				body->setActivationState(DISABLE_DEACTIVATION);
				m_dynamicsWorld->addRigidBody(body);
				if(prevBody != 0)
				{
					localB.setIdentity();
					localB.setOrigin(btVector3(0.5,0,0));
					btTransform localA;
					localA.setIdentity();
					localA.setOrigin(btVector3(-0.5,0,0));
					CONSTRAINT_TYPE* constraint =  new CONSTRAINT_TYPE(*prevBody, *body, localA, localB EXTRAPARAMS);
					constraint->setLimit(0, -0.01, 0.01);
					constraint->setLimit(1, 0, 0);
					constraint->setLimit(2, 0, 0);
					constraint->setLimit(3, 0, 0);
					constraint->setLimit(4, 0, 0);
					constraint->setLimit(5, 0, 0);
					for(int a = 0; a < 6; ++a)
					{
						constraint->setParam(BT_CONSTRAINT_STOP_ERP,0.9,a);
						constraint->setParam(BT_CONSTRAINT_STOP_CFM,0.0,a);
					}
					constraint->setDbgDrawSize(btScalar(1.f));
					m_dynamicsWorld->addConstraint(constraint, true);

					if(i < bodycount - 1)
					{
						localA.setIdentity();localA.getOrigin() = btVector3(0,0,3);
						localB.setIdentity();
						CONSTRAINT_TYPE* constraintZY =  new CONSTRAINT_TYPE(*staticBody, *body, localA, localB EXTRAPARAMS);
						constraintZY->setLimit(0, 1, -1);
						constraintZY->setDbgDrawSize(btScalar(1.f));
						m_dynamicsWorld->addConstraint(constraintZY, true);
						
					}
				}
				else
				{
					localA.setIdentity();localA.getOrigin() = btVector3(bodycount,0,3);
					localB.setIdentity();
					localB.setOrigin(btVector3(0,0,0));
					m_data->m_ChainLeftBody = body;
					m_data->m_ChainLeftConstraint = new CONSTRAINT_TYPE(*staticBody, *body, localA, localB EXTRAPARAMS);
					m_data->m_ChainLeftConstraint->setLimit(3,0,0);
					m_data->m_ChainLeftConstraint->setLimit(4,0,0);
					m_data->m_ChainLeftConstraint->setLimit(5,0,0);
					for(int a = 0; a < 6; ++a)
					{
						m_data->m_ChainLeftConstraint->setParam(BT_CONSTRAINT_STOP_ERP,limitConstraintStrength,a);
						m_data->m_ChainLeftConstraint->setParam(BT_CONSTRAINT_STOP_CFM,0.0,a);
					}
					m_data->m_ChainLeftConstraint->setDbgDrawSize(btScalar(1.f));
					m_dynamicsWorld->addConstraint(m_data->m_ChainLeftConstraint, true);
				}
				prevBody = body;
			}
			m_data->m_ChainRightBody = prevBody;
			localA.setIdentity();localA.getOrigin() = btVector3(-bodycount,0,3);
			localB.setIdentity();
			localB.setOrigin(btVector3(0,0,0));
			m_data->m_ChainRightConstraint = new CONSTRAINT_TYPE(*staticBody, *m_data->m_ChainRightBody, localA, localB EXTRAPARAMS);
			m_data->m_ChainRightConstraint->setLimit(3,0,0);
			m_data->m_ChainRightConstraint->setLimit(4,0,0);
			m_data->m_ChainRightConstraint->setLimit(5,0,0);
			for(int a = 0; a < 6; ++a)
			{
				m_data->m_ChainRightConstraint->setParam(BT_CONSTRAINT_STOP_ERP,limitConstraintStrength,a);
				m_data->m_ChainRightConstraint->setParam(BT_CONSTRAINT_STOP_CFM,0.0,a);
			}
		}
		m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void Dof6Spring2Setup::animate()
{
	
/////// servo motor: flip its target periodically
#ifdef USE_6DOF2
		static float servoNextFrame = -1;
		if(servoNextFrame < 0)
		{
			m_data->m_ServoMotorConstraint->getRotationalLimitMotor(2)->m_servoTarget *= -1;
			servoNextFrame = 3.0;
		}
		servoNextFrame -= m_data->mDt;
#endif

/////// constraint chain: pull the chain left and right periodically
		static float chainNextFrame = -1;
		static bool left = true;
		if(chainNextFrame < 0)
		{
			if(!left)
			{
				m_data->m_ChainRightBody->setActivationState(ACTIVE_TAG);
				m_dynamicsWorld->removeConstraint(m_data->m_ChainRightConstraint);
				m_data->m_ChainLeftConstraint->setDbgDrawSize(btScalar(2.f));
				m_dynamicsWorld->addConstraint(m_data->m_ChainLeftConstraint, true);
			}
			else
			{
				m_data->m_ChainLeftBody->setActivationState(ACTIVE_TAG);
				m_dynamicsWorld->removeConstraint(m_data->m_ChainLeftConstraint);
				m_data->m_ChainRightConstraint->setDbgDrawSize(btScalar(2.f));
				m_dynamicsWorld->addConstraint(m_data->m_ChainRightConstraint, true);
			}
			chainNextFrame = 3.0;
			left = !left;
		}
		chainNextFrame -= m_data->mDt;

/////// bouncing constraint: push the box periodically
		m_data->m_BouncingTranslateBody->setActivationState(ACTIVE_TAG);
		static float bounceNextFrame = -1;
		if(bounceNextFrame < 0)
		{
			m_data->m_BouncingTranslateBody->applyCentralImpulse(btVector3(10,0,0));
			bounceNextFrame = 3.0;
		}
		bounceNextFrame -= m_data->mDt;

		m_data->frameID++;
}


void Dof6Spring2Setup::stepSimulation(float deltaTime)
{
	animate();
	m_dynamicsWorld->stepSimulation(deltaTime);
}

class CommonExampleInterface*    Dof6Spring2CreateFunc( CommonExampleOptions& options)
{
	return new Dof6Spring2Setup(options.m_guiHelper);
}
