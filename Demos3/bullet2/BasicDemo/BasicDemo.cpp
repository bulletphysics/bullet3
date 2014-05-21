#include "BasicDemo.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"




#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

static const float scaling=0.35f;

BasicDemo::BasicDemo(SimpleOpenGL3App* app)
:Bullet2RigidBodyDemo(app)
{
}

BasicDemo::~BasicDemo()
{
}

void	BasicDemo::createGround(int cubeShapeId)
{
	{
		btVector4 color(0.3,0.3,1,1);
		btVector4 halfExtents(50,50,50,1);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-50,0));
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
		}
	}
}
void	BasicDemo::initPhysics()
{
	m_physicsSetup.m_glApp = m_glApp;
	m_physicsSetup.initPhysics();
	m_dynamicsWorld = m_physicsSetup.m_dynamicsWorld;

	m_glApp->m_instancingRenderer->writeTransforms();
}
void	BasicDemo::exitPhysics()
{
	m_physicsSetup.exitPhysics();
	m_dynamicsWorld = 0;
	//Bullet2RigidBodyDemo::exitPhysics();
}

//SimpleOpenGL3App* m_glApp;

btRigidBody*	MyBasicDemoPhysicsSetup::createRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape, const btVector4& color)
{
	btRigidBody* body = BasicDemoPhysicsSetup::createRigidBody(mass,startTransform,shape);
	int graphicsShapeId = shape->getUserIndex();
	btAssert(graphicsShapeId>=0);
	btVector3 localScaling = shape->getLocalScaling();
	
	int graphicsInstanceId = m_glApp->m_instancingRenderer->registerGraphicsInstance(graphicsShapeId,startTransform.getOrigin(),startTransform.getRotation(),color,localScaling);
	body->setUserIndex(graphicsInstanceId);
	
	//todo: create graphics representation
	return body;

}
	
btBoxShape* MyBasicDemoPhysicsSetup::createBoxShape(const btVector3& halfExtents)
{
	btBoxShape* box = BasicDemoPhysicsSetup::createBoxShape(halfExtents);
	int cubeShapeId = m_glApp->registerCubeShape(halfExtents.x(),halfExtents.y(),halfExtents.z());
	box->setUserIndex(cubeShapeId);
	//todo: create graphics representation
	return box;
}


void	BasicDemo::renderScene()
{
	//sync graphics -> physics world transforms
	{
		for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
		{
			btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btVector3 pos = colObj->getWorldTransform().getOrigin();
			btQuaternion orn = colObj->getWorldTransform().getRotation();
			int index = colObj ->getUserIndex();
			if (index>=0)
			{
				m_glApp->m_instancingRenderer->writeSingleInstanceTransformToCPU(pos,orn,index);
			}
		}
		m_glApp->m_instancingRenderer->writeTransforms();
	}

	m_glApp->m_instancingRenderer->renderScene();
}

	
void	BasicDemo::stepSimulation(float dt)
{
	m_physicsSetup.stepSimulation(dt);
	m_physicsSetup.m_dynamicsWorld->debugDrawWorld();



	/*
	//print applied force
	//contact points
	for (int i=0;i<m_dynamicsWorld->getDispatcher()->getNumManifolds();i++)
	{
		btPersistentManifold* contact = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		for (int c=0;c<contact->getNumContacts();c++)
		{
			btManifoldPoint& pt = contact->getContactPoint(c);
			btScalar dist = pt.getDistance();
			if (dist< contact->getContactProcessingThreshold())
			{
				printf("normalImpulse[%d.%d] = %f\n",i,c,pt.m_appliedImpulse);
				
			} else
			{
				printf("?\n");
			}
		}
	}
	*/
}



