#include "BlockSolverExample.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h"
#include "btBlockSolver.h"
//for URDF import support
#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../CommonInterfaces/CommonMultiBodyBase.h"

class BlockSolverExample : public CommonMultiBodyBase
{
	int m_option;

public:
	BlockSolverExample(GUIHelperInterface* helper, int option);
	virtual ~BlockSolverExample();

	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);

	virtual void resetCamera()
	{
		float dist = 3;
		float pitch = -35;
		float yaw = 50;
		float targetPos[3] = {0, 0, .1};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	void createMultiBodyStack();
	btMultiBody* createMultiBody(btScalar mass, const btTransform& trans, btCollisionShape* collisionShape);
	btMultiBody* loadRobot(std::string filepath);
};

BlockSolverExample::BlockSolverExample(GUIHelperInterface* helper, int option)
	: CommonMultiBodyBase(helper),
	  m_option(option)
{
	m_guiHelper->setUpAxis(2);
}

BlockSolverExample::~BlockSolverExample()
{
	// Do nothing
}

void BlockSolverExample::stepSimulation(float deltaTime)
{
	//use a smaller internal timestep, there are stability issues
	btScalar internalTimeStep = 1. / 240.f;
	m_dynamicsWorld->stepSimulation(deltaTime, 10, internalTimeStep);
}

void BlockSolverExample::initPhysics()
{
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	btMLCPSolverInterface* mlcp;
	if (m_option & BLOCK_SOLVER_SI)
	{
		btAssert(!m_solver);
		m_solver = new btMultiBodyConstraintSolver;
		b3Printf("Constraint Solver: Sequential Impulse");
	}
	if (m_option & BLOCK_SOLVER_MLCP_PGS)
	{
		btAssert(!m_solver);
		mlcp = new btSolveProjectedGaussSeidel();
		m_solver = new btMultiBodyMLCPConstraintSolver(mlcp);
		b3Printf("Constraint Solver: MLCP + PGS");
	}
	if (m_option & BLOCK_SOLVER_MLCP_DANTZIG)
	{
		btAssert(!m_solver);
		mlcp = new btDantzigSolver();
		m_solver = new btMultiBodyMLCPConstraintSolver(mlcp);
		b3Printf("Constraint Solver: MLCP + Dantzig");
	}
	if (m_option & BLOCK_SOLVER_BLOCK)
	{
		m_solver = new btBlockSolver();
	}

	btAssert(m_solver);

	btMultiBodyDynamicsWorld* world = new btMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld = world;
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->setGravity(btVector3(0, 0, -10));
	m_dynamicsWorld->getSolverInfo().m_numIterations = 50;
	m_dynamicsWorld->getSolverInfo().m_globalCfm = btScalar(1e-6);  //todo: what value is good?

	if (m_option & BLOCK_SOLVER_SCENE_MB_STACK)
	{
		createMultiBodyStack();
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BlockSolverExample::createMultiBodyStack()
{
	///create a few basic rigid bodies
	bool loadPlaneFromURDF = false;
	if (loadPlaneFromURDF)
	{
		btMultiBody* mb = loadRobot("plane.urdf");
		printf("!\n");
	}
	else
	{
		btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
		m_collisionShapes.push_back(groundShape);
		btScalar mass = 0;
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, -50));
		btMultiBody* body = createMultiBody(mass, tr, groundShape);
	}

	for (int i = 0; i < 10; i++)
	{
		btBoxShape* boxShape = createBoxShape(btVector3(btScalar(.1), btScalar(.1), btScalar(.1)));
		m_collisionShapes.push_back(boxShape);
		btScalar mass = 1;
		if (i == 9)
			mass = 100;
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, 0.1 + i * 0.2));
		btMultiBody* body = createMultiBody(mass, tr, boxShape);
	}
	if (0)
	{
		btMultiBody* mb = loadRobot("cube_small.urdf");
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, 1.));
		mb->setBaseWorldTransform(tr);
	}
}

btMultiBody* BlockSolverExample::createMultiBody(btScalar mass, const btTransform& trans, btCollisionShape* collisionShape)
{
	btVector3 inertia;
	collisionShape->calculateLocalInertia(mass, inertia);

	bool canSleep = false;
	bool isDynamic = mass > 0;
	btMultiBody* mb = new btMultiBody(0, mass, inertia, !isDynamic, canSleep);
	btMultiBodyLinkCollider* collider = new btMultiBodyLinkCollider(mb, -1);
	collider->setWorldTransform(trans);
	mb->setBaseWorldTransform(trans);

	collider->setCollisionShape(collisionShape);

	int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
	int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

	this->m_dynamicsWorld->addCollisionObject(collider, collisionFilterGroup, collisionFilterMask);
	mb->setBaseCollider(collider);

	mb->finalizeMultiDof();

	this->m_dynamicsWorld->addMultiBody(mb);
	m_dynamicsWorld->forwardKinematics();
	return mb;
}

btMultiBody* BlockSolverExample::loadRobot(std::string filepath)
{
	btMultiBody* m_multiBody = 0;
	BulletURDFImporter u2b(m_guiHelper, 0, 0, 1, 0);
	bool loadOk = u2b.loadURDF(filepath.c_str());  // lwr / kuka.urdf");
	if (loadOk)
	{
		int rootLinkIndex = u2b.getRootLinkIndex();
		b3Printf("urdf root link index = %d\n", rootLinkIndex);
		MyMultiBodyCreator creation(m_guiHelper);
		btTransform identityTrans;
		identityTrans.setIdentity();
		ConvertURDF2Bullet(u2b, creation, identityTrans, m_dynamicsWorld, true, u2b.getPathPrefix());
		for (int i = 0; i < u2b.getNumAllocatedCollisionShapes(); i++)
		{
			m_collisionShapes.push_back(u2b.getAllocatedCollisionShape(i));
		}
		m_multiBody = creation.getBulletMultiBody();
		if (m_multiBody)
		{
			b3Printf("Root link name = %s", u2b.getLinkName(u2b.getRootLinkIndex()).c_str());
		}
	}
	return m_multiBody;
}

CommonExampleInterface* BlockSolverExampleCreateFunc(CommonExampleOptions& options)
{
	return new BlockSolverExample(options.m_guiHelper, options.m_option);
}
