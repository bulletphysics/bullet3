#include "ChainDemo.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"


#define NUM_SPHERES 10

static const float scaling=0.35f;

ChainDemo::ChainDemo(SimpleOpenGL3App* app)
:Bullet2RigidBodyDemo(app)
{
}

ChainDemo::~ChainDemo()
{
}

void	ChainDemo::createGround(int cubeShapeId)
{
	{
		btVector4 color(0.3,0.3,1,1);
		btVector4 halfExtents(50,1,50,1);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-5,0));
		m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,groundTransform.getOrigin(),groundTransform.getRotation(),color,halfExtents);
		btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(halfExtents[0]),btScalar(halfExtents[1]),btScalar(halfExtents[2])));
		//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
		{
			btScalar mass(0.);
			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);
			btVector3 localInertia(0,0,0);
			if (isDynamic)
				groundShape->calculateLocalInertia(mass,localInertia);
			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
			btRigidBody* body = new btRigidBody(rbInfo);
			//add the body to the dynamics world
			m_dynamicsWorld->addRigidBody(body);
			body->setActivationState(DISABLE_DEACTIVATION);
		}
	}
}
void	ChainDemo::initPhysics()
{
//	Bullet2RigidBodyDemo::initPhysics();

	m_config = new btDefaultCollisionConfiguration;
	m_dispatcher = new btCollisionDispatcher(m_config);
	m_bp = new btDbvtBroadphase();
	//m_solver = new btNNCGConstraintSolver();
	m_solver = new btSequentialImpulseConstraintSolver();
//	btDantzigSolver* mlcp = new btDantzigSolver();
	//btLemkeSolver* mlcp = new btLemkeSolver();
	//m_solver = new btMLCPSolver(mlcp);
//	m_solver = new btSequentialImpulseConstraintSolver();
	//btMultiBodyConstraintSolver* solver = new btMultiBodyConstraintSolver();
	//m_solver = solver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_bp,m_solver,m_config);
	m_dynamicsWorld->getSolverInfo().m_numIterations = 1000;
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = false;

	int curColor=0;
	//create ground
	btScalar radius=scaling;
	int unitCubeShapeId = m_glApp->registerCubeShape();
	
	float pos[]={0,0,0};
	float orn[]={0,0,0,1};
		

	//eateGround(unitCubeShapeId);
	
	int sphereShapeId = m_glApp->registerGraphicsSphereShape(radius,false);

	{
		btVector4 halfExtents(scaling,scaling,scaling,1);
		btVector4 colors[4] =
		{
			btVector4(1,0,0,1),
			btVector4(0,1,0,1),
			btVector4(0,1,1,1),
			btVector4(1,1,0,1),
		};
		


		btTransform startTransform;
		startTransform.setIdentity();
		
		
		btCollisionShape* colShape = new btSphereShape(scaling);

		btScalar largeMass[]={1000,10,100,1000};
		for (int i=0;i<1;i++)
		{

			btAlignedObjectArray<btRigidBody*> bodies;
			for (int k=0;k<NUM_SPHERES;k++)
			{
				btVector3 localInertia(0,0,0);
				btScalar mass = 0.f;
				curColor = 1;

				switch (k)
				{
					case 0:
						{
							mass = largeMass[i];
							curColor = 0;
							break;
						}
					case NUM_SPHERES-1:
					{
						mass = 0.f;
						curColor = 2;
						break;
					}
					default:
						{
							curColor = 1;
							mass = 1.f;
						}
				};
		
				if (mass)
					colShape ->calculateLocalInertia(mass,localInertia);

				btVector4 color = colors[curColor];
			
				startTransform.setOrigin(btVector3(
									btScalar(7.5+-i*5),
									btScalar(6.*scaling+2.0*scaling*k),
									btScalar(0)));

				m_glApp->m_instancingRenderer->registerGraphicsInstance(sphereShapeId,startTransform.getOrigin(),startTransform.getRotation(),color,halfExtents);
			
				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
				btRigidBody* body = new btRigidBody(rbInfo);
				bodies.push_back(body);
				body->setActivationState(DISABLE_DEACTIVATION);

				m_dynamicsWorld->addRigidBody(body);
			}

			//add constraints
			btVector3 pivotInA(0,radius,0);
			btVector3 pivotInB(0,-radius,0);
			for (int k=0;k<NUM_SPHERES-1;k++)
			{
				btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*bodies[k],*bodies[k+1],pivotInA,pivotInB);
				m_dynamicsWorld->addConstraint(p2p,true);
			}
		}
	}

	m_glApp->m_instancingRenderer->writeTransforms();
}
void	ChainDemo::exitPhysics()
{
	
	Bullet2RigidBodyDemo::exitPhysics();
}
void	ChainDemo::renderScene()
{
	//sync graphics -> physics world transforms
	{
		for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
		{
			btVector3 pos = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().getOrigin();
			btQuaternion orn = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().getRotation();
			m_glApp->m_instancingRenderer->writeSingleInstanceTransformToCPU(pos,orn,i);
		}
		m_glApp->m_instancingRenderer->writeTransforms();
	}

	m_glApp->m_instancingRenderer->renderScene();
}

	
void	ChainDemo::stepSimulation(float dt)
{
	m_dynamicsWorld->stepSimulation(dt,10,1./240.);
	//m_dynamicsWorld->stepSimulation(dt,10,1./60.);
}



