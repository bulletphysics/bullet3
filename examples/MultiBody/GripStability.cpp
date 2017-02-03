#include "GripStability.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"

#include "../OpenGLWindow/GLInstancingRenderer.h"
#include "../CommonInterfaces/CommonMultiBodyBase.h"

static const float EFFORT     = 10.0; // task is: hold a 2 kg pan in hand with force of 10 N (it falls out of hand quicker if force is lower, 10N == hover 1kg in earth gravity)
static const float PAN_MASS   = 2.0;
static const float SCALE      = 1.0f; // set scale: 1, 10, 100 all work about the same

static const float TIMESTAMP  = 1./240.f; // almost work at 1./240.f/4, but again become worse at 1./240.f/8
static const float GRIP_WIDTH = 0.1;


class GripStabilityDemo : public CommonMultiBodyBase
{
public:
	GripStabilityDemo(GUIHelperInterface* helper): CommonMultiBodyBase(helper)
	{
		m_guiHelper->setUpAxis(2);
	}

	void initPhysics();
	void stepSimulation(float deltaTime);

	void resetCamera()
	{
		float dist = 2*SCALE;
		float pitch = 50;
		float yaw = 35;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}

	void testScene();
};

void GripStabilityDemo::stepSimulation(float deltaTime)
{
	m_dynamicsWorld->stepSimulation(deltaTime, 10, TIMESTAMP);
}

void GripStabilityDemo::initPhysics()
{	
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();
	btMultiBodyConstraintSolver* sol = new btMultiBodyConstraintSolver;
	m_solver = sol;
	btMultiBodyDynamicsWorld* world = new btMultiBodyDynamicsWorld(m_dispatcher,m_broadphase,sol,m_collisionConfiguration);
	m_dynamicsWorld = world;
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->setGravity(btVector3(0,0,-10*SCALE));

	if (1) { // Best parameters we have so far. With defaults it's even worse :(
		m_dynamicsWorld->getSolverInfo().m_tau = 0.95;
		m_dynamicsWorld->getSolverInfo().m_erp = 0.08;
		m_dynamicsWorld->getSolverInfo().m_erp2 = 0.08;
		m_dynamicsWorld->getSolverInfo().m_linearSlop = 0.00001;
		m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 32;
		m_dynamicsWorld->getSolverInfo().m_numIterations = 50; // increasing this helps a bit (set 250)
		m_dynamicsWorld->getSolverInfo().m_leastSquaresResidualThreshold = 1e-7;
		m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.00000;
	}

	testScene();
}

void GripStabilityDemo::testScene()
{
	btSphereShape* sp = new btSphereShape(0.3f*SCALE);
	m_collisionShapes.push_back(sp);

	btBoxShape* finger_shape = new btBoxShape(btVector3(0.06f*SCALE/2, 0.04f*SCALE/2, 0.3f*SCALE/2));
	m_collisionShapes.push_back(finger_shape);
	btVector3 finger_inertia(0,0,0);
	finger_shape->calculateLocalInertia(0.5, finger_inertia);

	btDefaultMotionState* sp_ms = new btDefaultMotionState( btTransform(btQuaternion(0,0,0,1), btVector3(0,                0, 1.0f*SCALE             ) ) );
	btRigidBody* sp_rigid = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(0.0, sp_ms, sp, btVector3(0,0,0)));

	btDefaultMotionState* f1_ms = new btDefaultMotionState( btTransform(btQuaternion(0,0,0,1), btVector3(0, GRIP_WIDTH*SCALE,(1.0f-0.35f-0.15f)*SCALE)) );
	btRigidBody* f1_rigid = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(0.5, f1_ms, finger_shape, finger_inertia));
	btDefaultMotionState* f2_ms = new btDefaultMotionState( btTransform(btQuaternion(0,0,0,1), btVector3(0,-GRIP_WIDTH*SCALE,(1.0f-0.35f-0.15f)*SCALE)) );
	btRigidBody* f2_rigid = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(0.5, f2_ms, finger_shape, finger_inertia));

	m_dynamicsWorld->addRigidBody(sp_rigid, 1, 1);
	m_dynamicsWorld->addRigidBody(f1_rigid, 1, 1);
	m_dynamicsWorld->addRigidBody(f2_rigid, 1, 1);

	btVector3 attach_fingers_point(0, 0, (1.0f-0.35f)*SCALE);
	btVector3 sphere_local = sp_ms->m_graphicsWorldTrans.inverse() * attach_fingers_point;
	btVector3 f1_local = f1_ms->m_graphicsWorldTrans.inverse() * attach_fingers_point;
	btVector3 f2_local = f2_ms->m_graphicsWorldTrans.inverse() * attach_fingers_point;

	btQuaternion q(0,0,0,1);
	{
		btGeneric6DofSpring2Constraint* dof = new btGeneric6DofSpring2Constraint(
			*f1_rigid, *sp_rigid, btTransform(q,f1_local), btTransform(q,sphere_local), RO_XYZ
			);
		dof->setAngularLowerLimit(btVector3(0,0,0));
		dof->setAngularUpperLimit(btVector3(0,0,0));
		dof->enableMotor(1, true);
		m_dynamicsWorld->addConstraint(dof);
		dof->setTargetVelocity(1, -0.3*SCALE);
		dof->setMaxMotorForce(1, EFFORT*SCALE);
		dof->setLinearLowerLimit(btVector3(0,-GRIP_WIDTH*SCALE,0));
		dof->setLinearUpperLimit(btVector3(0,+GRIP_WIDTH*SCALE,0));
	}
	{
		btGeneric6DofSpring2Constraint* dof = new btGeneric6DofSpring2Constraint(
			*f2_rigid, *sp_rigid, btTransform(q,f2_local), btTransform(q,sphere_local), RO_XYZ
			);
		dof->setAngularLowerLimit(btVector3(0,0,0));
		dof->setAngularUpperLimit(btVector3(0,0,0));
		dof->enableMotor(1, true);
		m_dynamicsWorld->addConstraint(dof);
		dof->setTargetVelocity(1, 0.3*SCALE);
		dof->setMaxMotorForce(1, EFFORT*SCALE);
		dof->setLinearLowerLimit(btVector3(0,-GRIP_WIDTH*SCALE,0));
		dof->setLinearUpperLimit(btVector3(0,+GRIP_WIDTH*SCALE,0));
	}

	btCylinderShape* pan_shape = new btCylinderShape(btVector3(0.06*SCALE/2, 1.20f*SCALE/2, 0)); // rad, rad, half-len
	m_collisionShapes.push_back(pan_shape);
	btVector3 pan_inertia(0,0,0);
	pan_shape->calculateLocalInertia(PAN_MASS, pan_inertia);

	btDefaultMotionState* pan_ms = new btDefaultMotionState( btTransform(btQuaternion(0.707,0.707,0,0), btVector3(0.5f*SCALE, 0, (1.0f-0.35f-0.03f/2)*SCALE)) );
	btRigidBody* pan_rigid = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(PAN_MASS, pan_ms, pan_shape, pan_inertia));
	m_dynamicsWorld->addRigidBody(pan_rigid, 1, 1);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

class CommonExampleInterface* GripStability(struct CommonExampleOptions& options)
{
	return new GripStabilityDemo(options.m_guiHelper);
}
