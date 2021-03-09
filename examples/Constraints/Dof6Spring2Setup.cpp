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
#include "../CommonInterfaces/CommonMultiBodyBase.h"

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
////  helper function
////
/////////////////////////////////////////////////////////////
btScalar inline rad2degree(btScalar rad){
    return rad / SIMD_PI * 180.0f;
}

btScalar inline degree2rad(btScalar degree){
    return degree / 180.0f * SIMD_PI;
}

void getEulerValFromDof(btVector3& eulerVal, const btScalar* q)
{
    btQuaternion qt(q[0], q[1], q[2], q[3]);
    qt.getEulerZYX(eulerVal[2], eulerVal[1], eulerVal[0]);
}

btScalar cosOffset(btScalar amp, btScalar T, btScalar phase, btScalar time){
    return amp * cos(time / T * SIMD_2_PI + phase);
}

btScalar adjustKs(btScalar ks, btScalar mass, btScalar deltaTime)
{
    btScalar r = ks;
    btScalar angular_freq = btSqrt(r / mass);
    if (0.25 < angular_freq * deltaTime)
    {
        r = BT_ONE / deltaTime / deltaTime / btScalar(16.0) * mass;
    }
    return r;
}

btScalar adjustKd(btScalar kd, btScalar mass, btScalar deltaTime)
{
    btScalar r = kd;
    if (kd * deltaTime > mass)
    {
        r = mass / deltaTime;
    }
    return r;
}

btVector3 getSpringImpulse(btScalar ks, btScalar kd, btScalar mass,
        const btVector3& currPos,  const btVector3& balPos, const btVector3& currVel, float deltaTime)
{
    //limit stiffness (the spring should not be sampled faster that the quarter of its angular frequency)
    btScalar _ks = adjustKs(ks, mass, deltaTime);
    btScalar _kd = adjustKd(kd, mass, deltaTime);

    btVector3 spring_force = -(currPos - balPos) * _ks;
    btVector3 damp_force = -currVel * _kd;

    btVector3 impulse = (spring_force + damp_force) * deltaTime;
    return impulse;
};



/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
////  spring test
////
/////////////////////////////////////////////////////////////

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

    btVector3 impulse = getSpringImpulse(m_ks, m_kd, m_mass, currPos, m_balancePos, currVel, deltaTime);
    m_body->applyCentralImpulse(impulse);

	m_dynamicsWorld->stepSimulation(deltaTime, 2, btScalar(1.0) / 48);

    m_time += deltaTime;
    m_step += 1;
}


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
////  multibody pendulum
////
/////////////////////////////////////////////////////////////

static btScalar radius(0.05);

struct Skeleton : public CommonMultiBodyBase
{
    btMultiBody* m_multiBody;
    btRigidBody* m_collider;

    btScalar m_time;
    int m_nextSec;
    int m_step;
    int m_numLinks;
    btAlignedObjectArray<bool> m_useKs;
    btAlignedObjectArray<btQuaternion> m_balanceRot;
    btAlignedObjectArray<btScalar> m_Ks;
    btAlignedObjectArray<btScalar> m_jointDamp;

public:
    Skeleton(struct GUIHelperInterface* helper);
    virtual ~Skeleton();
    virtual void initPhysics();
    virtual void stepSimulation(float deltaTime);
    virtual void resetCamera()
    {
        float dist = 5;
        float pitch = -21;
        float yaw = 270;
        float targetPos[3] = {0, 0, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    void addColliders(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents);
    void applyStiffTorque(btMultiBody* pMultiBody, float deltaTime);
};

Skeleton::Skeleton(struct GUIHelperInterface* helper)
        : CommonMultiBodyBase(helper)
{
    m_time = btScalar(0.0);
    m_nextSec = floor(m_time) + 1;
    m_step = 0;
    m_numLinks = 2;
}

Skeleton::~Skeleton()
{
}

void Skeleton::initPhysics()
{
    int upAxis = 1;

    m_guiHelper->setUpAxis(upAxis);

    this->createEmptyDynamicsWorld();
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    if (m_dynamicsWorld->getDebugDrawer())
    {
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(
                //btIDebugDraw::DBG_DrawConstraints
                +btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawAabb);  //+btIDebugDraw::DBG_DrawConstraintLimits);
    }

    m_dynamicsWorld->setGravity(btVector3(0, 0, 0));

    bool floating = false;
    bool damping = false;
    bool gyro = false;

    bool canSleep = false;
    bool selfCollide = false;
    btVector3 linkHalfExtents(0.05, 0.5, 0.1);
    btVector3 baseHalfExtents(0.0, 0.0, 0.0);

    btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
    float baseMass = 0.f;

    btMultiBody* pMultiBody = new btMultiBody(m_numLinks, baseMass, baseInertiaDiag, !floating, canSleep);
    //pMultiBody->useRK4Integration(true);

    // set base position
    btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
    pMultiBody->setWorldToBaseRot(baseOriQuat);
    btVector3 basePosition(0, 0, 0);
    pMultiBody->setBasePos(basePosition);

    m_multiBody = pMultiBody;

    //y-axis assumed up
    btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);
    for (int i = 0; i < m_numLinks; ++i)
    {
        float linkMass = 1.f;
        btVector3 linkInertiaDiag(0.f, 0.f, 0.f);
        btCollisionShape* shape = 0;
        {
            shape = new btBoxShape(linkHalfExtents);
        }
        shape->calculateLocalInertia(linkMass, linkInertiaDiag);
        delete shape;

        btVector3 parentComToCurrentCom;
        if (i == 0){
            parentComToCurrentCom = btVector3(0, -linkHalfExtents[1] * 1, 0);
        }
        else{
            parentComToCurrentCom = btVector3(0, -linkHalfExtents[1] * 2, 0);
        }
        pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1,
                btQuaternion(0.f, 0.f, 0.f, 1.f),
                                  (parentComToCurrentCom - currentPivotToCurrentCom),
                                  currentPivotToCurrentCom, !selfCollide);
    }

    pMultiBody->finalizeMultiDof();
    btMultiBodyDynamicsWorld* world = m_dynamicsWorld;
    world->addMultiBody(pMultiBody);

    pMultiBody->setCanSleep(canSleep);
    pMultiBody->setHasSelfCollision(selfCollide);
    pMultiBody->setUseGyroTerm(gyro);
    if (!damping)
    {
        pMultiBody->setLinearDamping(0.f);
        pMultiBody->setAngularDamping(0.f);
    }
    else
    {
        pMultiBody->setLinearDamping(0.1f);
        pMultiBody->setAngularDamping(0.9f);
    }

    // set init pose
    if (m_numLinks > 0)
    {
        btScalar angle = -23.57817848 * SIMD_PI / 180.f * 2;
        btQuaternion quat0(btVector3(1, 0, 0).normalized(), angle);
        quat0.normalize();
        pMultiBody->setJointPosMultiDof(0, quat0);
    }

    // set if use ks
    m_useKs.resize(pMultiBody->getNumLinks());
    m_Ks.resize(pMultiBody->getNumLinks());
    m_balanceRot.resize(pMultiBody->getNumLinks());
    m_jointDamp.resize(pMultiBody->getNumLinks());
    for (int i = 0; i < m_numLinks; i++)
    {
        m_useKs[i] = true;
        m_Ks[i] = 1;
        m_balanceRot[i] = btQuaternion(0.0, 0.0, 0.0, 1.0);
        m_jointDamp[0] = 0.0;
    }

    // adjust balance rot, ks and damp
    btScalar angle = -23.57817848 * SIMD_PI / 180.f;
    m_balanceRot[0] = btQuaternion(btVector3(1, 0, 0).normalized(), angle);
    m_jointDamp[0] = 0.1;
    m_jointDamp[1] = 0.1;

    addColliders(pMultiBody, world, baseHalfExtents, linkHalfExtents);

    /////////////////////////////////////////////////////////////////
    // construct the box
    /////////////////////////////////////////////////////////////////
    {
        btVector3 halfExtents(.4, .4, .4);
        btBoxShape* colShape = new btBoxShape(halfExtents);
        //btCollisionShape* colShape = new btSphereShape(btScalar(1.));

        /// Create Dynamic Objects
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0.0, -1.5, -2.0));

        btScalar mass(0.f);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            colShape->calculateLocalInertia(mass, localInertia);

        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
        m_collider = new btRigidBody(rbInfo);

        m_dynamicsWorld->addRigidBody(m_collider);
//        btVector4 color(0, 1, 0, 1);
//        m_guiHelper->createCollisionObjectGraphicsObject(colShape, color);
    }

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Skeleton::addColliders(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents)
{
    btAlignedObjectArray<btQuaternion> world_to_local;
    world_to_local.resize(pMultiBody->getNumLinks() + 1);

    btAlignedObjectArray<btVector3> local_origin;
    local_origin.resize(pMultiBody->getNumLinks() + 1);
    world_to_local[0] = pMultiBody->getWorldToBaseRot();
    local_origin[0] = pMultiBody->getBasePos();

    for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
    {
        const int parent = pMultiBody->getParent(i);
        world_to_local[i + 1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
        local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), pMultiBody->getRVector(i)));
    }

    for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
    {
        btVector3 posr = local_origin[i + 1];
        btScalar quat[4] = {-world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w()};

//        btCollisionShape* shape = new btSphereShape(radius);
        btCollisionShape* shape = new btBoxShape(linkHalfExtents);
//        m_guiHelper->createCollisionShapeGraphicsObject(shape);
        btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);
        col->setCollisionShape(shape);

        bool isDynamic = 1;
        int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
        int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(posr);
        tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
        col->setWorldTransform(tr);

        pWorld->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);  //,2,1+2);
//        btVector4 color(1, 0, 0, 1);
//        m_guiHelper->createCollisionObjectGraphicsObject(col, color);
        pMultiBody->getLink(i).m_collider = col;
    }
}

void Skeleton::applyStiffTorque(btMultiBody* pMultiBody, float deltaTime){
    for (int i = 0; i < pMultiBody->getNumLinks(); ++i) {
        btVector3 stiff_force(0.0, 0.0, 0.0);
        if (m_useKs[i])
        {
            btScalar _ks = adjustKs(m_Ks[i], pMultiBody->getLink(i).m_mass, deltaTime);

            btScalar* q = pMultiBody->getJointPosMultiDof(i);
            btQuaternion qt(q[0], q[1], q[2], q[3]);
            btQuaternion delta = m_balanceRot[i] * qt.inverse();
            delta.getEulerZYX(stiff_force[2], stiff_force[1], stiff_force[0]);
            stiff_force *= _ks;
        }

        for (int d = 0; d < pMultiBody->getLink(i).m_dofCount; d++) {
            btScalar _kd = adjustKd(m_jointDamp[i], pMultiBody->getLink(i).m_mass, deltaTime);

            btScalar angle_force = stiff_force[d] - _kd * pMultiBody->getJointVelMultiDof(i)[d];
            pMultiBody->addJointTorqueMultiDof(i, d, angle_force);
        }
    }
}

void Skeleton::stepSimulation(float deltaTime)
{
    if ( fabs(m_time - m_nextSec) < 1e-3 )
    {
        printf("step: %d, time: %d sec\n", m_step, m_nextSec - 1);

        for (int i = 0; i < m_multiBody->getNumLinks(); ++i)
        {
            btVector3 anglePos(0.0, 0.0, 0.0);
            getEulerValFromDof(anglePos, m_multiBody->getJointPosMultiDof(i));
            printf("anglePos : %f,\t%f,\t%f\n", rad2degree(anglePos[0]), rad2degree(anglePos[1]), rad2degree(anglePos[2]));

            btVector3 angleVel(0.0, 0.0, 0.0);
            getEulerValFromDof(angleVel, m_multiBody->getJointVelMultiDof(i));
            printf("angleVel : %f,\t%f,\t%f\n", rad2degree(angleVel[0]), rad2degree(angleVel[1]), rad2degree(angleVel[2]));
        }

        m_nextSec += 1;
    }

//    m_multiBody->addJointTorque(0, -2.0);
    applyStiffTorque(m_multiBody, deltaTime);

    {
        btScalar amp = 0.5f;
        btScalar T = 10.0f;
        btScalar phase =  SIMD_PI / 2.0;
        m_multiBody->setBasePos(btVector3(0.0, cosOffset(amp, T, phase, m_time), 0.0));
//        m_multiBody->setBaseVel(btVector3(0.0, amp * sin(m_time / T * SIMD_2_PI + phase) * SIMD_2_PI / T, 0.0));
    }

    {
        /// use spring impulse to drive collider
//        const btVector3 currPos = m_collider->getWorldTransform().getOrigin();
//        const btVector3 currVel = m_collider->getLinearVelocity();
//        btVector3 impulse = getSpringImpulse(1.0, 0.0, 1.0, currPos, btVector3(0, -1.5, 0), currVel, deltaTime);
//        m_collider->applyCentralImpulse(impulse);

        /// use cos function to drive
        btTransform tr;
        tr.setIdentity();
        btScalar amp = 0.8f;
        btScalar T = 10.0f;
        btScalar phase = SIMD_PI;
        tr.setOrigin(btVector3(0.0, -1.5, cosOffset(amp, T, phase, m_time)));
        tr.setRotation(btQuaternion(0.0, 0.0, 0.0, 1.0));
        m_collider->setWorldTransform(tr);
    }

//    m_dynamicsWorld->stepSimulation(deltaTime);
    m_dynamicsWorld->stepSimulation(deltaTime, 2, btScalar(1.0) / 48);

    m_time += deltaTime;
    m_step += 1;
}

class CommonExampleInterface* Dof6Spring2CreateFunc(CommonExampleOptions& options)
{
//    return new Dof6Spring2Setup(options.m_guiHelper);
	return new Skeleton(options.m_guiHelper);
}
