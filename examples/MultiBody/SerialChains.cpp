#include "SerialChains.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include "btBulletDynamicsCommon.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodySliderConstraint.h"

#include "../OpenGLWindow/GLInstancingRenderer.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

class SerialChains : public CommonMultiBodyBase
{
public:
	SerialChains(GUIHelperInterface* helper);
	virtual ~SerialChains();

	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);

	virtual void resetCamera()
	{
		float dist = 1;
		float pitch = -35;
		float yaw = 50;
		float targetPos[3] = {-3, 2.8, -2.5};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	btMultiBody* createFeatherstoneMultiBody(class btMultiBodyDynamicsWorld* world, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical = false, bool fixedBase = false);
	void createGround(const btVector3& halfExtents = btVector3(50, 50, 50), btScalar zOffSet = btScalar(-1.55));
	void addColliders(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents);
};

static bool g_fixedBase = true;
static bool g_firstInit = true;
static float scaling = 0.4f;
static float friction = 1.;
static int g_constraintSolverType = 0;

SerialChains::SerialChains(GUIHelperInterface* helper)
	: CommonMultiBodyBase(helper)
{
	m_guiHelper->setUpAxis(1);
}

SerialChains::~SerialChains()
{
	// Do nothing
}

void SerialChains::stepSimulation(float deltaTime)
{
	//use a smaller internal timestep, there are stability issues
	float internalTimeStep = 1. / 240.f;
	m_dynamicsWorld->stepSimulation(deltaTime, 10, internalTimeStep);
}

void SerialChains::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	if (g_firstInit)
	{
		m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraDistance(btScalar(10. * scaling));
		m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraPitch(50);
		g_firstInit = false;
	}
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	if (g_constraintSolverType == 3)
	{
		g_constraintSolverType = 0;
		g_fixedBase = !g_fixedBase;
	}

	btMLCPSolverInterface* mlcp;
	switch (g_constraintSolverType++)
	{
		case 0:
			m_solver = new btMultiBodyConstraintSolver;
			b3Printf("Constraint Solver: Sequential Impulse");
			break;
		case 1:
			mlcp = new btSolveProjectedGaussSeidel();
			m_solver = new btMultiBodyMLCPConstraintSolver(mlcp);
			b3Printf("Constraint Solver: MLCP + PGS");
			break;
		default:
			mlcp = new btDantzigSolver();
			m_solver = new btMultiBodyMLCPConstraintSolver(mlcp);
			b3Printf("Constraint Solver: MLCP + Dantzig");
			break;
	}

	btMultiBodyDynamicsWorld* world = new btMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld = world;
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	m_dynamicsWorld->getSolverInfo().m_globalCfm = btScalar(1e-4);  //todo: what value is good?

	///create a few basic rigid bodies
	btVector3 groundHalfExtents(50, 50, 50);
	btCollisionShape* groundShape = new btBoxShape(groundHalfExtents);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 00));

	/////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////

	bool damping = true;
	bool gyro = true;
	int numLinks = 5;
	bool spherical = true;      //set it ot false -to use 1DoF hinges instead of 3DoF sphericals
	bool multibodyOnly = true;  //false
	bool canSleep = true;
	bool selfCollide = true;
	btVector3 linkHalfExtents(0.05, 0.37, 0.1);
	btVector3 baseHalfExtents(0.05, 0.37, 0.1);

	btMultiBody* mbC1 = createFeatherstoneMultiBody(world, numLinks, btVector3(-0.4f, 3.f, 0.f), linkHalfExtents, baseHalfExtents, spherical, g_fixedBase);
	btMultiBody* mbC2 = createFeatherstoneMultiBody(world, numLinks, btVector3(-0.4f, 3.0f, 0.5f), linkHalfExtents, baseHalfExtents, spherical, g_fixedBase);

	mbC1->setCanSleep(canSleep);
	mbC1->setHasSelfCollision(selfCollide);
	mbC1->setUseGyroTerm(gyro);

	if (!damping)
	{
		mbC1->setLinearDamping(0.f);
		mbC1->setAngularDamping(0.f);
	}
	else
	{
		mbC1->setLinearDamping(0.1f);
		mbC1->setAngularDamping(0.9f);
	}
	//
	m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
	//////////////////////////////////////////////
	if (numLinks > 0)
	{
		btScalar q0 = 45.f * SIMD_PI / 180.f;
		if (!spherical)
		{
			mbC1->setJointPosMultiDof(0, &q0);
		}
		else
		{
			btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
			quat0.normalize();
			mbC1->setJointPosMultiDof(0, quat0);
		}
	}
	///
	addColliders(mbC1, world, baseHalfExtents, linkHalfExtents);

	mbC2->setCanSleep(canSleep);
	mbC2->setHasSelfCollision(selfCollide);
	mbC2->setUseGyroTerm(gyro);
	//
	if (!damping)
	{
		mbC2->setLinearDamping(0.f);
		mbC2->setAngularDamping(0.f);
	}
	else
	{
		mbC2->setLinearDamping(0.1f);
		mbC2->setAngularDamping(0.9f);
	}
	//
	m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
	//////////////////////////////////////////////
	if (numLinks > 0)
	{
		btScalar q0 = -45.f * SIMD_PI / 180.f;
		if (!spherical)
		{
			mbC2->setJointPosMultiDof(0, &q0);
		}
		else
		{
			btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
			quat0.normalize();
			mbC2->setJointPosMultiDof(0, quat0);
		}
	}
	///
	addColliders(mbC2, world, baseHalfExtents, linkHalfExtents);

	/////////////////////////////////////////////////////////////////
	btScalar groundHeight = -51.55;
	btScalar mass(0.);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass, localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, groundHeight, 0));
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	//add the body to the dynamics world
	m_dynamicsWorld->addRigidBody(body, 1, 1 + 2);  //,1,1+2);
	/////////////////////////////////////////////////////////////////

	createGround();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	/////////////////////////////////////////////////////////////////
}

btMultiBody* SerialChains::createFeatherstoneMultiBody(btMultiBodyDynamicsWorld* pWorld, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical, bool fixedBase)
{
	//init the base
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;

	if (baseMass)
	{
		btCollisionShape* pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	bool canSleep = false;

	btMultiBody* pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, fixedBase, canSleep);

	btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
	pMultiBody->setBasePos(basePosition);
	pMultiBody->setWorldToBaseRot(baseOriQuat);
	btVector3 vel(0, 0, 0);

	//init the links
	btVector3 hingeJointAxis(1, 0, 0);
	float linkMass = 1.f;
	btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

	btCollisionShape* pTempBox = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));
	pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
	delete pTempBox;

	//y-axis assumed up
	btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] * 2.f, 0);                      //par body's COM to cur body's COM offset
	btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);                         //cur body's COM to cur body's PIV offset
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;  //par body's COM to cur body's PIV offset

	//////
	btScalar q0 = 0.f * SIMD_PI / 180.f;
	btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
	quat0.normalize();
	/////

	for (int i = 0; i < numLinks; ++i)
	{
		if (!spherical)
			pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, true);
		else
			//pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f)/*quat0*/, btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
			pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	}

	pMultiBody->finalizeMultiDof();

	///
	pWorld->addMultiBody(pMultiBody);
	///
	return pMultiBody;
}

void SerialChains::createGround(const btVector3& halfExtents, btScalar zOffSet)
{
	btCollisionShape* groundShape = new btBoxShape(halfExtents);
	m_collisionShapes.push_back(groundShape);

	// rigidbody is dynamic if and only if mass is non zero, otherwise static
	btScalar mass(0.);
	const bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass, localInertia);

	// using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -halfExtents.z() + zOffSet, 0));
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	// add the body to the dynamics world
	m_dynamicsWorld->addRigidBody(body, 1, 1 + 2);
}

void SerialChains::addColliders(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents)
{
	btAlignedObjectArray<btQuaternion> world_to_local;
	world_to_local.resize(pMultiBody->getNumLinks() + 1);

	btAlignedObjectArray<btVector3> local_origin;
	local_origin.resize(pMultiBody->getNumLinks() + 1);
	world_to_local[0] = pMultiBody->getWorldToBaseRot();
	local_origin[0] = pMultiBody->getBasePos();

	{
		btScalar quat[4] = {-world_to_local[0].x(), -world_to_local[0].y(), -world_to_local[0].z(), world_to_local[0].w()};

		if (1)
		{
			btCollisionShape* box = new btBoxShape(baseHalfExtents);
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
			col->setCollisionShape(box);

			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(local_origin[0]);
			tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
			col->setWorldTransform(tr);

			pWorld->addCollisionObject(col, 2, 1 + 2);

			col->setFriction(friction);
			pMultiBody->setBaseCollider(col);
		}
	}

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

		btCollisionShape* box = new btBoxShape(linkHalfExtents);
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

		col->setCollisionShape(box);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
		col->setWorldTransform(tr);
		col->setFriction(friction);
		pWorld->addCollisionObject(col, 2, 1 + 2);

		pMultiBody->getLink(i).m_collider = col;
	}
}

CommonExampleInterface* SerialChainsCreateFunc(CommonExampleOptions& options)
{
	return new SerialChains(options.m_guiHelper);
}
