#include <math.h>
#include <limits>

#include "Dof6Spring2Setup.h"

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"

#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616
#endif

extern float g_additionalBodyMass;

//comment this out to compare with original spring constraint
#define USE_6DOF2
#ifdef USE_6DOF2
#define CONSTRAINT_TYPE btGeneric6DofSpring2Constraint
#define EXTRAPARAMS
#else
#define CONSTRAINT_TYPE btGeneric6DofSpringConstraint
#define EXTRAPARAMS , true
#endif

#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct Dof6Spring2Setup : public CommonRigidBodyBase
{
	struct Dof6Spring2SetupInternalData* m_data;
	btRigidBody* m_body;
    btScalar m_time;
    btVector3 m_balancePos;
    btScalar m_ks;
    btScalar m_kd;
    btScalar m_mass;
    int m_step;

	Dof6Spring2Setup(struct GUIHelperInterface* helper);
	virtual ~Dof6Spring2Setup();
	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);

	void animate(const btVector3& currPos, const btVector3& currVel, float deltaTime);

	virtual void resetCamera()
	{
		float dist = 6;
		float pitch = -45;
		float yaw = 0;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

struct Dof6Spring2SetupInternalData
{
	btRigidBody* m_TranslateSpringBody;
	btRigidBody* m_TranslateSpringBody2;
	btRigidBody* m_RotateSpringBody;
	btRigidBody* m_RotateSpringBody2;
	btRigidBody* m_BouncingTranslateBody;
	btRigidBody* m_MotorBody;
	btRigidBody* m_ServoMotorBody;
	btRigidBody* m_ChainLeftBody;
	btRigidBody* m_ChainRightBody;
	CONSTRAINT_TYPE* m_ServoMotorConstraint;
	CONSTRAINT_TYPE* m_ChainLeftConstraint;
	CONSTRAINT_TYPE* m_ChainRightConstraint;

	float mDt;

	unsigned int frameID;
	Dof6Spring2SetupInternalData()
		: mDt(1. / 60.), frameID(0)
	{
	}
};

Dof6Spring2Setup::Dof6Spring2Setup(struct GUIHelperInterface* helper)
	: CommonRigidBodyBase(helper)
{
	m_data = new Dof6Spring2SetupInternalData;
	m_time = btScalar(0.0);
    m_balancePos = btVector3(0.0, 0.0, 0.0);
    m_ks = 0.1;
    m_kd = 0.2;
    m_mass = btScalar(1.0);
    m_step = 0;
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
	btVector3 worldAabbMin(-10000, -10000, -10000);
	btVector3 worldAabbMax(10000, 10000, 10000);
	m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);

	/////// uncomment the corresponding line to test a solver.
	//m_solver = new btSequentialImpulseConstraintSolver;
	m_solver = new btNNCGConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld->getDispatchInfo().m_useContinuous = true;
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));

	m_dynamicsWorld->getSolverInfo().m_numIterations = 100;

	btCollisionShape* shape;
	btVector3 localInertia(0, 0, 0);
	btDefaultMotionState* motionState;
	btTransform bodyTransform;
	btScalar mass;
	btTransform localA;
	btTransform localB;
	CONSTRAINT_TYPE* constraint;

	float z_offset = 2.0;
	// this static body is used in the constraint calculation
	//static body centered in the origo
	mass = 0.0;
	shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	localInertia = btVector3(0, 0, 0);
	bodyTransform.setIdentity();
	bodyTransform.setOrigin(btVector3(0, 0, z_offset));
	motionState = new btDefaultMotionState(bodyTransform);
	btRigidBody* staticBody = new btRigidBody(mass, motionState, shape, localInertia);

	/////////// box with translate spring, attached to static body
	///////////
	{
		mass = m_mass;

		float init_x = 2.0;

		shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
		shape->calculateLocalInertia(mass, localInertia);
		bodyTransform.setIdentity();
		bodyTransform.setOrigin(btVector3(init_x, 0, z_offset));
		bodyTransform.getBasis().setEulerZYX(0, 0, 0);
		motionState = new btDefaultMotionState(bodyTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, localInertia);
        // if deactivation time < some value, then the rigid body will become deactivate and stop
#if defined(BT_USE_DOUBLE_PRECISION)
        rbInfo.m_linearSleepingThreshold = 1e-7;
        rbInfo.m_angularSleepingThreshold = 1e-7;
#else
        rbInfo.m_linearSleepingThreshold = 1e-7;
        rbInfo.m_angularSleepingThreshold = 1e-7;
#endif

		m_data->m_TranslateSpringBody = new btRigidBody(rbInfo);
		m_data->m_TranslateSpringBody->setActivationState(ACTIVE_TAG);
		m_dynamicsWorld->addRigidBody(m_data->m_TranslateSpringBody);

		localA.setIdentity();
		localB.setIdentity();
		constraint = new CONSTRAINT_TYPE(*staticBody, *m_data->m_TranslateSpringBody, localA, localB EXTRAPARAMS);
		constraint->setLimit(0, 5, -5);
		constraint->setLimit(1, 0, 0);
		constraint->setLimit(2, 0, 0);
		constraint->setLimit(3, 0, 0);
		constraint->setLimit(4, 0, 0);
		constraint->setLimit(5, 0, 0);
		constraint->enableSpring(0, true);
		constraint->setStiffness(0, m_ks);
		constraint->setDamping(0, m_kd);
#ifdef USE_6DOF2
//		constraint->setDamping(0, 0);
#else
		constraint->setDamping(5, 1);
#endif
		constraint->setEquilibriumPoint(0, 0);
		constraint->setDbgDrawSize(btScalar(2.f));
		m_dynamicsWorld->addConstraint(constraint, true);
	}

    /////////// simulate spring with impulse
    ///////////
	{
		//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		btCollisionShape* shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

		btScalar mass = m_mass;
		btVector3 localInertia(0, 0, 0);
		if (mass > 0)
			shape->calculateLocalInertia(mass, localInertia);

		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(2, 0, 0));
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
		// if deactivation time < some value, then the rigid body will become deactivate and stop
#if defined(BT_USE_DOUBLE_PRECISION)
        rbInfo.m_linearSleepingThreshold = 1e-7;
        rbInfo.m_angularSleepingThreshold = 1e-7;
#else
		rbInfo.m_linearSleepingThreshold = 1e-7;
        rbInfo.m_angularSleepingThreshold = 1e-7;
#endif
		m_body = new btRigidBody(rbInfo);
		m_dynamicsWorld->addRigidBody(m_body);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Dof6Spring2Setup::animate(const btVector3& currPos, const btVector3& currVel, float deltaTime) {
    //limit stiffness (the spring should not be sampled faster that the quarter of its angular frequency)
    btScalar ks = m_ks;
    btScalar angular_freq = btSqrt(m_ks / m_mass);
    if (0.25 < angular_freq * deltaTime)
    {
       ks = BT_ONE / deltaTime / deltaTime / btScalar(16.0) * m_mass;
    }

    btScalar kd = m_kd;
    //avoid damping that would blow up the spring
    if (m_kd * deltaTime > m_mass)
    {
        kd = m_mass / deltaTime;
    }

//    btVector3 force = btVector3(-1, 0, 0);
//    btVector3 impulse = force * deltaTime;
    btVector3 spring_force = -(currPos - m_balancePos) * ks;
    btVector3 damp_force = -currVel * kd;

    btVector3 impulse = (spring_force + damp_force) * deltaTime;
    m_body->applyCentralImpulse(impulse);
}

void Dof6Spring2Setup::stepSimulation(float deltaTime)
{
    if ( ((ceil(m_time) - m_time) < 1e-3) || ((m_time - floor(m_time)) < 1e-3) )
    {
        btTransform trans;
        trans = m_body->getWorldTransform();
        printf("step: %d, time: %f\n", m_step, m_time);
        printf("pos : %f,%f,%f\n", float(trans.getOrigin().getX()),
               float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
        const btVector3 &linear_vel = m_body->getLinearVelocity();
        printf("vel: %f,%f,%f\n", float(linear_vel.getX()),
               float(linear_vel.getY()), float(linear_vel.getZ()));
    }

    // get current pose
    const btVector3 currPos = m_body->getWorldTransform().getOrigin();
    const btVector3 currVel = m_body->getLinearVelocity();

	animate(currPos, currVel, deltaTime);

	m_dynamicsWorld->stepSimulation(deltaTime, 2, btScalar(1.0) / 48);

    m_time += deltaTime;
    m_step += 1;
}

class CommonExampleInterface* Dof6Spring2CreateFunc(CommonExampleOptions& options)
{
	return new Dof6Spring2Setup(options.m_guiHelper);
}
