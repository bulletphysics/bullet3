#include "BoxStacks_MLCP.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyBlockConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "../RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"


class BoxStacks_MLCP : public CommonMultiBodyBase
{
public:
	BoxStacks_MLCP(GUIHelperInterface* helper);
	virtual ~BoxStacks_MLCP();

	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);

	virtual void resetCamera()
	{
		float dist = 2;
		float pitch = -35;
		float yaw = 50;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	btMultiBody* createMultiBody(btScalar mass, const btTransform& trans, btCollisionShape* collisionShape);

	btMultiBody* loadRobot(std::string filepath = "kuka_iiwa/model.urdf");
};



static int g_constraintSolverType = 0;

BoxStacks_MLCP::BoxStacks_MLCP(GUIHelperInterface* helper)
	: CommonMultiBodyBase(helper)
{
	
}

BoxStacks_MLCP::~BoxStacks_MLCP()
{
	// Do nothing
}

void BoxStacks_MLCP::stepSimulation(float deltaTime)
{
	float internalTimeStep = 1. / 240.f;
	m_dynamicsWorld->stepSimulation(deltaTime, 10, internalTimeStep);
	for (int i = 0; i < m_dynamicsWorld->getNumMultibodies(); i++)
	{
		btVector3 pos = m_dynamicsWorld->getMultiBody(i)->getBaseWorldTransform().getOrigin();
		printf("pos[%d]=%f,%f,%f\n", i, pos.x(), pos.y(), pos.z());
	}
}

void BoxStacks_MLCP::initPhysics()
{
	m_guiHelper->setUpAxis(2);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->getSolverInfo().m_numIterations = 50;
	if (g_constraintSolverType == 5)
	{
		g_constraintSolverType = 0;
	}

	btMultiBodyConstraintSolver* sol = 0;
	
	btMLCPSolverInterface* mlcp;
	switch (g_constraintSolverType++)
	{
		case 0:
			sol = new btMultiBodyConstraintSolver;
			b3Printf("Constraint Solver: Sequential Impulse");
			break;
		case 1:
			mlcp = new btSolveProjectedGaussSeidel();
			sol = new btMultiBodyMLCPConstraintSolver(mlcp);
			b3Printf("Constraint Solver: MLCP + PGS");
			break;
		case 2:
			mlcp = new btDantzigSolver();
			sol = new btMultiBodyMLCPConstraintSolver(mlcp);
			b3Printf("Constraint Solver: MLCP + Dantzig");
			break;
		case 3:
			mlcp = new btLemkeSolver();
			sol = new btMultiBodyMLCPConstraintSolver(mlcp);
			
			b3Printf("Constraint Solver: MLCP + Lemke");
			break;
			
		default:
			sol = new btMultiBodyBlockConstraintSolver();
			b3Printf("btMultiBodyBlockConstraintSolver");
			break;
	}

	m_solver = sol;

	btMultiBodyDynamicsWorld* world = new btMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration);
	m_dynamicsWorld = world;
	m_dynamicsWorld->getSolverInfo().m_globalCfm = btScalar(1e-4);


	m_dynamicsWorld->setGravity(btVector3(0,0,-10));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	bool loadPlaneFromURDF = true;
	if (loadPlaneFromURDF)
	{
		loadRobot("plane.urdf");
	} else
	{
		btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
		m_collisionShapes.push_back(groundShape);
		btScalar mass = 0;
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, -50));
		btMultiBody* body = createMultiBody(mass, tr, groundShape);
	}

	{
		btBoxShape* boxShape = createBoxShape(btVector3(btScalar(.1), btScalar(.1), btScalar(.1)));
		m_collisionShapes.push_back(boxShape);
		btScalar mass = 10;
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, 0.5));
		btMultiBody* body = createMultiBody(mass, tr, boxShape);
	}

	{
		btMultiBody* mb = loadRobot("cube_small.urdf");
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, 1.));
		mb->setBaseWorldTransform(tr);
	}
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}



btMultiBody* BoxStacks_MLCP::createMultiBody(btScalar mass, const btTransform& trans, btCollisionShape* collisionShape)
{
	btVector3 inertia;
	collisionShape->calculateLocalInertia(mass, inertia);
	
	bool canSleep = false;
	bool isDynamic = mass > 0;
	btMultiBody* mb = new btMultiBody(0, mass, inertia, !isDynamic,canSleep);
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



btMultiBody*BoxStacks_MLCP::loadRobot(std::string filepath)
{
	btMultiBody* m_multiBody = 0;
	BulletURDFImporter u2b(m_guiHelper,0,0,1,0);
	bool loadOk = u2b.loadURDF(filepath.c_str());// lwr / kuka.urdf");
	if (loadOk)
	{
			int rootLinkIndex = u2b.getRootLinkIndex();
			b3Printf("urdf root link index = %d\n",rootLinkIndex);
			MyMultiBodyCreator creation(m_guiHelper);
			btTransform identityTrans;
			identityTrans.setIdentity();
			ConvertURDF2Bullet(u2b,creation, identityTrans,m_dynamicsWorld,true,u2b.getPathPrefix());
			for (int i = 0; i < u2b.getNumAllocatedCollisionShapes(); i++)
			{
				m_collisionShapes.push_back(u2b.getAllocatedCollisionShape(i));
			}
			m_multiBody = creation.getBulletMultiBody();
			if (m_multiBody)
			{
				b3Printf("Root link name = %s",u2b.getLinkName(u2b.getRootLinkIndex()).c_str());
			}
	}
	return m_multiBody;
}

CommonExampleInterface* BoxStacks_MLCPCreateFunc(CommonExampleOptions& options)
{
	return new BoxStacks_MLCP(options.m_guiHelper);
}
