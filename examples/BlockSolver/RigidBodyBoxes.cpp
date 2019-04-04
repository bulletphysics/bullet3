#include "RigidBodyBoxes.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BlockSolverExample.h"
#include "btBlockSolver.h"

class RigidBodyBoxes : public CommonRigidBodyBase
{
	int m_option;
	int m_numIterations;
	int m_numBoxes = 4;
	btAlignedObjectArray<btRigidBody*> boxes;
	static btScalar numSolverIterations;

public:
	RigidBodyBoxes(GUIHelperInterface* helper, int option);
	virtual ~RigidBodyBoxes();

	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);
	void resetCubePosition();
	virtual void resetCamera()
	{
		float dist = 3;
		float pitch = -35;
		float yaw = 50;
		float targetPos[3] = {0, 0, .1};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1],
								 targetPos[2]);
	}

	void createRigidBodyStack();
};

btScalar RigidBodyBoxes::numSolverIterations = 50;

RigidBodyBoxes::RigidBodyBoxes(GUIHelperInterface* helper, int option)
	: CommonRigidBodyBase(helper),
	  m_option(option),
	  m_numIterations(numSolverIterations)
{
	m_guiHelper->setUpAxis(2);
}

RigidBodyBoxes::~RigidBodyBoxes()
{
	// Do nothing
}

void RigidBodyBoxes::createRigidBodyStack()
{
	// create ground
	btBoxShape* groundShape =
		createBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, -5));
	btScalar mass(0.);
	btRigidBody* body = createRigidBody(mass, groundTransform, groundShape,
										btVector4(0, 0, 1, 1));

	// create a few boxes
	mass = 1;
	for (int i = 0; i < m_numBoxes; i++)
	{
		btBoxShape* boxShape =
			createBoxShape(btVector3(btScalar(.1), btScalar(.1), btScalar(.1)));
		m_collisionShapes.push_back(boxShape);
		mass *= 4;
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, 0.1 + i * 0.2));
		boxes.push_back(createRigidBody(mass, tr, boxShape));
	}
}

void RigidBodyBoxes::initPhysics()
{
	/// collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	/// use the default collision dispatcher. For parallel processing you can use
	/// a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();

	{
		SliderParams slider("numSolverIterations", &numSolverIterations);
		slider.m_minVal = 5;
		slider.m_maxVal = 500;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	if (m_option & BLOCK_SOLVER_SI)
	{
		m_solver = new btSequentialImpulseConstraintSolver;
		b3Printf("Constraint Solver: Sequential Impulse");
	}
	if (m_option & BLOCK_SOLVER_BLOCK)
	{
		m_solver = new btBlockSolver();
		b3Printf("Constraint Solver: Block solver");
	}

	btAssert(m_solver);

	m_dynamicsWorld = new btDiscreteDynamicsWorld(
		m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld->setGravity(btVector3(0, 0, -10));

	createRigidBodyStack();

	m_dynamicsWorld->getSolverInfo().m_numIterations = numSolverIterations;
	m_dynamicsWorld->getSolverInfo().m_globalCfm = btScalar(1e-6);

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void RigidBodyBoxes::resetCubePosition()
{
	for (int i = 0; i < m_numBoxes; i++)
	{
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, 0.1 + i * 0.2));
		boxes[i]->setWorldTransform(tr);
	}
}

void RigidBodyBoxes::stepSimulation(float deltaTime)
{
	if ((int)numSolverIterations != m_numIterations)
	{
		resetCubePosition();
		m_numIterations = (int)numSolverIterations;
		m_dynamicsWorld->getSolverInfo().m_numIterations = m_numIterations;
		b3Printf("New num iterations; %d", m_numIterations);
	}

	m_dynamicsWorld->stepSimulation(deltaTime);
}

CommonExampleInterface* RigidBodyBoxesCreateFunc(
	CommonExampleOptions& options)
{
	return new RigidBodyBoxes(options.m_guiHelper, options.m_option);
}
