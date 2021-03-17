#include <math.h>
#include <limits>

#include "Dof6Spring2Setup.h"

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"

#include "../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "../Utils/b3BulletDefaultFileIO.h"

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

btScalar inline degreeDiff(const btVector3& a1, const btVector3& a2)
{
    btVector3 d1 = btVector3(rad2degree(a1[0]), rad2degree(a1[1]), rad2degree(a1[2]));
    btVector3 d2 = btVector3(rad2degree(a2[0]), rad2degree(a2[1]), rad2degree(a2[2]));
    return (d1 - d2).norm();
}

void getEulerValFromDof(btVector3& eulerVal, const btScalar* q)
{
    btQuaternion qt(q[0], q[1], q[2], q[3]);
    qt.getEulerZYX(eulerVal[2], eulerVal[1], eulerVal[0]);
}

btScalar cosOffset(btScalar amp, btScalar T, btScalar phase, btScalar time){
    return amp * cos(time / T * SIMD_2_PI + phase);
}

btScalar sinOffset(btScalar amp, btScalar T, btScalar phase, btScalar time){
    return amp * sin(time / T * SIMD_2_PI + phase);
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
////  multibody skeleton
////
/////////////////////////////////////////////////////////////
#define WIRE_FRAME 0

char mfileName[1024];

const btVector3 zeroTransOffset(0.0, 0.0, 0.0);

enum CollisionTypes
{
    NOTHING  = 0,        //< Collide with nothing
    BONE_BODY = 1 << 1,
    TARGET_BODY = 1 << 2
};

static btScalar radius(0.05);

struct Skeleton : public CommonMultiBodyBase
{
    btMultiBody* m_multiBody;
    btRigidBody* m_collider;

    int m_solverType;
    btScalar m_time;
    int m_nextSec;
    int m_step;
    int m_numLinks;
    btVector3 m_prevBasePos;
    btVector3 m_prevBaseVel;
    btAlignedObjectArray<btQuaternion> m_balanceRot;
    btAlignedObjectArray<btScalar> m_Ks;
    btAlignedObjectArray<btScalar> m_jointDamp;
    btAlignedObjectArray<btQuaternion> m_prevJointRot;

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
    void applyTorque(float deltaTime);
    static void OnInternalTickCallback(btDynamicsWorld* world, btScalar timeStep);
};

Skeleton::Skeleton(struct GUIHelperInterface* helper)
        : CommonMultiBodyBase(helper)
{
    m_time = btScalar(0.0);
    m_nextSec = floor(m_time) + 1;
    m_step = 0;
    m_numLinks = 4;
    m_solverType = 0;
    m_prevBasePos = btVector3(0.0, 0.0, 0.0);
    m_prevBaseVel = btVector3(0.0, 0.0, 0.0);
}

Skeleton::~Skeleton()
{
}

void Skeleton::initPhysics()
{
    int upAxis = 1;

    m_guiHelper->setUpAxis(upAxis);

    // set btDefaultCollisionConfiguration, dispatcher, solver and other things
    this->createEmptyDynamicsWorld(m_solverType);

    bool isPreTick = false;
    m_dynamicsWorld->setInternalTickCallback(OnInternalTickCallback, this, isPreTick);

    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    if (m_dynamicsWorld->getDebugDrawer())
    {
        int mode = btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawAabb;
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(mode);  //+btIDebugDraw::DBG_DrawConstraintLimits);
    }

    m_dynamicsWorld->setGravity(btVector3(0, 0, 0));

    const bool floating = false;
    const bool damping = false;   // disable bullet internal damp
    const bool gyro = false;
    const bool canSleep = false;
    const bool selfCollide = false;

    btVector3 basePos = btVector3(0.0, 0.0, 0.0);

    /////////////////////////////////////////////////////////////////
    // construct the skeleton
    /////////////////////////////////////////////////////////////////
    btVector3 linkHalfExtents(0.05, 0.5, 0.1);
    btVector3 baseHalfExtents(0.0, 0.0, 0.0);

    btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
    float baseMass = 0.f;

    btMultiBody* pMultiBody = new btMultiBody(m_numLinks, baseMass, baseInertiaDiag, !floating, canSleep);
    //pMultiBody->useRK4Integration(true);

    // set base position
    btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
    pMultiBody->setWorldToBaseRot(baseOriQuat);
    pMultiBody->setBasePos(basePos);

    m_prevBasePos = basePos;
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
    if (damping)
    {
        pMultiBody->setLinearDamping(0.1f);
        pMultiBody->setAngularDamping(0.9f);
    }

    // set init pose
    if (m_numLinks > 0)
    {
        btScalar angle = -23.57817848 * SIMD_PI / 180.f * 2;
//        btScalar angle = 0.0;
        btQuaternion quat0(btVector3(1, 0, 0).normalized(), angle);
        quat0.normalize();
        pMultiBody->setJointPosMultiDof(0, quat0);
    }

    // init default params
    m_Ks.resize(pMultiBody->getNumLinks());
    m_balanceRot.resize(pMultiBody->getNumLinks());
    m_jointDamp.resize(pMultiBody->getNumLinks());
    m_prevJointRot.resize(pMultiBody->getNumLinks());
    for (int i = 0; i < m_numLinks; i++)
    {
        m_Ks[i] = 0.1;
        m_balanceRot[i] = btQuaternion(0.0, 0.0, 0.0, 1.0);
        m_jointDamp[i] = 0.0;
        m_prevJointRot[i] = btQuaternion(0.0, 0.0, 0.0, 1.0);
    }

    // set per joint damp
    m_jointDamp[0] = 0.7;
    m_jointDamp[1] = 0.3;
    m_jointDamp[2] = 0.3;
    m_jointDamp[3] = 0.3;
    // adjust balance rot, ks and damp
    btScalar angle = -23.57817848 * SIMD_PI / 180.f;
    m_balanceRot[0] = btQuaternion(btVector3(1, 0, 0).normalized(), angle);

    addColliders(pMultiBody, world, baseHalfExtents, linkHalfExtents);

    /////////////////////////////////////////////////////////////////
    // construct the box
    /////////////////////////////////////////////////////////////////
//    {
//        btVector3 halfExtents(.4, .4, .4);
//        btCollisionShape* colShape = new btBoxShape(halfExtents);
//
//        /// Create Dynamic Objects
//        btTransform startTransform;
//        startTransform.setIdentity();
//        startTransform.setOrigin(btVector3(0.0, -1.5, -2.0));
//
//        btScalar mass(0.f);
//
//        //rigidbody is dynamic if and only if mass is non zero, otherwise static
//        bool isDynamic = (mass != 0.f);
//
//        btVector3 localInertia(0, 0, 0);
//        if (isDynamic)
//            colShape->calculateLocalInertia(mass, localInertia);
//
//        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
//        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
//        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
//        m_collider = new btRigidBody(rbInfo);
//        m_collider->setRestitution(0.0);
//
//        // set collision group and mask, only collide with objects with mask is true where it collide with bones
//        m_dynamicsWorld->addRigidBody(m_collider, TARGET_BODY, BONE_BODY);
//
//        if (!WIRE_FRAME)
//        {
//            m_guiHelper->createCollisionShapeGraphicsObject(colShape);
//            btVector4 color(1, 0, 0, 1);
//            m_guiHelper->createCollisionObjectGraphicsObject(dynamic_cast<btCollisionObject*>(m_collider), color);
//        }
//    }

    /////////////////////////////////////////////////////////////////
    // load triangles from obj file
    /////////////////////////////////////////////////////////////////
    {
        const char* fileName = "teddy.obj";  //sphere8.obj";//sponza_closed.obj";//sphere8.obj";
        char relativeFileName[1024];
        if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024,0))
        {
            char pathPrefix[1024];
            b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
        }

        b3BulletDefaultFileIO fileIO;
        GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(relativeFileName, "",&fileIO);
        if (!glmesh){
            printf("fail to load file &s\n", fileName);
            return;
        }
        else {
            printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh->m_numvertices, fileName);
        }

        btAlignedObjectArray<btVector3> convertedVerts;
        convertedVerts.reserve(glmesh->m_numvertices);
        for (int i=0; i<glmesh->m_numvertices; i++)
        {
            convertedVerts.push_back(btVector3(
                    glmesh->m_vertices->at(i).xyzw[0],
                    glmesh->m_vertices->at(i).xyzw[1],
                    glmesh->m_vertices->at(i).xyzw[2]));
        }
        btTriangleMesh* meshInterface = new btTriangleMesh();
        for (int i=0; i<glmesh->m_numIndices/3; i++)
        {
            const btVector3& v0 = convertedVerts[glmesh->m_indices->at(i*3)];
            const btVector3& v1 = convertedVerts[glmesh->m_indices->at(i*3+1)];
            const btVector3& v2 = convertedVerts[glmesh->m_indices->at(i*3+2)];
            meshInterface->addTriangle(v0,v1,v2);
        }
        btCollisionShape* shape = new btBvhTriangleMeshShape(meshInterface, true, true);

        float scaling[4] = {0.03, 0.03, 0.03, 1};
        btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
        shape->setLocalScaling(localScaling);

        /// Create Dynamic Objects
        btScalar mass(0.f);
        bool isDynamic = (mass != 0.f);
        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            shape->calculateLocalInertia(mass, localInertia);

        btVector3 startPos(0.0, -1.5, -2.0);
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(startPos);
        m_collider = createRigidBody(mass, startTransform, shape);

        int shapeId = m_guiHelper->registerGraphicsShape(&glmesh->m_vertices->at(0).xyzw[0],
                                                         glmesh->m_numvertices,
                                                         &glmesh->m_indices->at(0),
                                                         glmesh->m_numIndices,
                                                         B3_GL_TRIANGLES, -1);
        shape->setUserIndex(shapeId);
        float color[4] = {1, 0, 0, 1};
        float orn[4] = {0, 0, 0, 1};
        float pos[4] = {float(startPos[0]), float(startPos[1]), float(startPos[2]), 0.0};
        int renderInstance = m_guiHelper->registerGraphicsInstance(shapeId, pos, orn, color, scaling);
        m_collider->setUserIndex(renderInstance);
        m_collider->getBroadphaseHandle()->m_collisionFilterGroup = TARGET_BODY;
        m_collider->getBroadphaseHandle()->m_collisionFilterMask = BONE_BODY;
    }
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

        btCollisionShape* shape = new btBoxShape(linkHalfExtents);
        btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);
        col->setCollisionShape(shape);

        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(posr);
        tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
        col->setWorldTransform(tr);

        col->setRestitution(0.0);

        // set collision group and mask, only collide with objects with mask is true where it collide with collider
        pWorld->addCollisionObject(col, BONE_BODY, TARGET_BODY);  //,2,1+2);

        if (!WIRE_FRAME)
        {
            m_guiHelper->createCollisionShapeGraphicsObject(shape);
            btVector4 color(0, 1, 0, 1);
            m_guiHelper->createCollisionObjectGraphicsObject(col, color);
        }

        pMultiBody->getLink(i).m_collider = col;
    }
}

void Skeleton::applyTorque(float deltaTime){
//    printf("step: %d\n", m_step);
    for (int i = 0; i < m_multiBody->getNumLinks(); ++i) {
        btVector3 force(0.0, 0.0, 0.0);
        btScalar* q = m_multiBody->getJointPosMultiDof(i);
        btQuaternion curr_q = btQuaternion(q[0], q[1], q[2], q[3]);

        // stiff force
        if (m_Ks[i] > 0.0)
        {
            btScalar _ks = m_Ks[i];
            // TODO ?
//            _ks = adjustKs(m_Ks[i], m_multiBody->getLink(i).m_mass, deltaTime);
            btVector3 delta(m_balanceRot[i][0] - curr_q[0],m_balanceRot[i][1] - curr_q[1], m_balanceRot[i][2] - curr_q[2]);
            force += delta * _ks;
        }

        // damp force
        if (m_jointDamp[i] > 0.0) {
            btScalar _kd = m_jointDamp[i];
            // TODO ?
//            btScalar _kd = adjustKd(m_jointDamp[i], m_multiBody->getLink(i).m_mass, deltaTime);
            // TODO set rigid body deactive if the speed is very small for a some frames
            btVector3 damp_force(m_prevJointRot[i][0] - curr_q[0],m_prevJointRot[i][1] - curr_q[1], m_prevJointRot[i][2] - curr_q[2]);
            if (!damp_force.fuzzyZero()) {
                damp_force = damp_force.normalized();
                btQuaternion vel = m_prevJointRot[i] * curr_q.inverse();
                btScalar theta = vel.getAngle() / deltaTime;
                // TODO here we only consider the omega, mass and length need to be considered
                damp_force *= theta;
                // TODO if the omega is big, we need large damp, consider adding the omega * omega
                if (damp_force.length() > 0.3)
                {
                    printf("damp force %f is too big\n", damp_force.length());
                    damp_force = damp_force.normalized() * 0.3;
                }
                if (fabs(damp_force[0]) < 1e-5 && fabs(damp_force[1]) < 1e-5 && fabs(damp_force[2]) < 1e-5) {
                    //printf("damp too small: %f, %f, %f\n", damp_force[0], damp_force[1], damp_force[2]);
                    if (_kd > 1)
                        _kd = 0.9;
                }
                damp_force *= _kd;
            }
            force += damp_force;
        }

        // apply the torque
        for (int d = 0; d < m_multiBody->getLink(i).m_dofCount; d++) {
            m_multiBody->addJointTorqueMultiDof(i, d, force[d]);
        }
    }
}

void Skeleton::OnInternalTickCallback(btDynamicsWorld* world, btScalar timeStep)
{
    Skeleton* demo = static_cast<Skeleton*>(world->getWorldUserInfo());
    int numManifolds = world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();

        // only detect collision between bones and collider
        if (
                (!(obA->getBroadphaseHandle()->m_collisionFilterGroup & BONE_BODY)
                    && !(obB->getBroadphaseHandle()->m_collisionFilterGroup | BONE_BODY))
                ||
                (!(obA->getBroadphaseHandle()->m_collisionFilterGroup & TARGET_BODY)
                 && !(obB->getBroadphaseHandle()->m_collisionFilterGroup | TARGET_BODY))
        )
            continue;

        int numContacts = contactManifold->getNumContacts();
        for (int j = 0; j < numContacts; j++)
        {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            printf("step : %d, collision impulse: %f\n", demo->m_step, pt.getAppliedImpulse());

            btVector3 ptOnBone, normalOnBone;
            // if obA is bone
            if ( obA->getBroadphaseHandle()->m_collisionFilterGroup ){
                ptOnBone = pt.getPositionWorldOnA();
                normalOnBone = pt.m_normalWorldOnB;
            }
            else{
                ptOnBone = pt.getPositionWorldOnB();
                normalOnBone = -pt.m_normalWorldOnB;
            }
            normalOnBone.normalize();
            printf("ptA: %f, %f, %f\n", ptOnBone.getX(), ptOnBone.getY(), ptOnBone.getZ());
            printf("normal: %f, %f, %f\n", normalOnBone.getX(), normalOnBone.getY(), normalOnBone.getZ());

//            btVector3 to = ptOnBone + normalOnBone * (pt.getAppliedImpulse() + 1e-2) * 10;
            btVector3 to = ptOnBone + normalOnBone;
            btVector4 color(1, 0, 0, 1);
            if (demo->m_guiHelper->getRenderInterface())
            {
                if (WIRE_FRAME)
                {
                    demo->m_guiHelper->getRenderInterface()->drawLine(ptOnBone, to, color, btScalar(1));
                }
            }
        }
    }
}

void Skeleton::stepSimulation(float deltaTime) {
    if (m_step == 0) {
        for (int i = 0; i < m_multiBody->getNumLinks(); ++i) {
            btScalar* q = m_multiBody->getJointPosMultiDof(i);
            m_prevJointRot[i] = btQuaternion(q[0], q[1], q[2], q[3]);
        }
    }

//    if (fabs(m_time - m_nextSec) < deltaTime) {
//        printf("step: %d, time: %d sec\n", m_step, m_nextSec - 1);
//        m_nextSec += 1;
//    }

    // calculate and apply the impulse, the damp use the difference between prev pos and curr pos
    applyTorque(deltaTime);

    // TODO need to consider the base bone rotate.

    // btMultiBodyFixedConstraint does not work as we like, cause it is not totally fixed to the body.
    // move the base of the chain does not apply torque on the multibody chains
    btScalar amp = 0.5f;
    btScalar T = 50.0f;
    btScalar phase =  SIMD_PI / 2.0;
    btVector3 currBasePos = btVector3(0.0, cosOffset(amp, T, phase, m_time), 0.0);
    m_multiBody->setBasePos(currBasePos);

    btVector3 currBaseVel = (currBasePos - m_prevBasePos) / deltaTime;
    //  the drag force only affect the first bone
    for (int i = 0; i < 1; ++i) {
        btVector3 orient = m_multiBody->getLink(i).m_dVector;
        orient = m_multiBody->localDirToWorld(i, orient);
        btVector3 torque = btCross(orient, -(currBaseVel - m_prevBaseVel) / deltaTime * m_multiBody->getLink(i).m_mass) / deltaTime;
        // TODO at the beginning, the force is too big. but at the next Cycle, it is small. NEED to check and fix.
        torque *= 1;
        m_multiBody->addLinkTorque(i, torque);
    }
    m_prevBasePos = currBasePos;
    m_prevBaseVel = currBaseVel;

    // update prev pos after using it to calculate the damping force before stepSimulation change the pos
    for (int i = 0; i < m_multiBody->getNumLinks(); ++i) {
        btScalar* q = m_multiBody->getJointPosMultiDof(i);
        m_prevJointRot[i] = btQuaternion(q[0], q[1], q[2], q[3]);
    }

    {
        // use cos function to drive m_collider
        btTransform tr;
        tr.setIdentity();
        btScalar amp = 1.8f;
        btScalar T = 100.0f;
        btScalar phase = SIMD_PI;
        tr.setOrigin(btVector3(0.0, -1.5, cosOffset(amp, T, phase, m_time)));
        tr.setRotation(btQuaternion(0.0, 0.0, 0.0, 1.0));
        m_collider->setWorldTransform(tr);
    }

    // capture the frames
//    if (m_step > 500 && m_step < 550)
//    {
//        const char* gPngFileName = "multibody";
//        sprintf(mfileName, "%s_%d.png", gPngFileName, m_step);
//        this->m_guiHelper->getAppInterface()->dumpNextFrameToPng(mfileName);
//    }

    // step and update positions
//    m_dynamicsWorld->stepSimulation(deltaTime);
    m_dynamicsWorld->stepSimulation(deltaTime, 2, btScalar(1.0) / 48);
    if ( WIRE_FRAME ) {
        m_dynamicsWorld->debugDrawWorld();
    }

    m_time += deltaTime;
    m_step += 1;
}


class CommonExampleInterface* Dof6Spring2CreateFunc(CommonExampleOptions& options)
{
//    return new Dof6Spring2Setup(options.m_guiHelper);
	return new Skeleton(options.m_guiHelper);
}
