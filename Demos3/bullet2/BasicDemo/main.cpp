
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5


#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Vector3.h"
#include "assert.h"
#include <stdio.h>

#include "btBulletDynamicsCommon.h"

class Bullet2RigidBodyDemo
{
protected:
	btDiscreteDynamicsWorld* m_dynamicsWorld;
	btCollisionDispatcher*	m_dispatcher;
	btBroadphaseInterface*	m_bp;
	btCollisionConfiguration* m_config;
	btConstraintSolver* m_solver;

public:
	Bullet2RigidBodyDemo()
	{
		m_config = 0;
		m_dispatcher = 0;
		m_bp = 0;
		m_solver = 0;
		m_dynamicsWorld = 0;
	}
	virtual void initPhysics()
	{
		m_config = new btDefaultCollisionConfiguration;
		m_dispatcher = new btCollisionDispatcher(m_config);
		m_bp = new btDbvtBroadphase();
		m_solver = new btSequentialImpulseConstraintSolver();
		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_bp,m_solver,m_config);
	}
	virtual void exitPhysics()
	{
		delete m_dynamicsWorld;
		m_dynamicsWorld=0;
		delete m_solver;
		m_solver=0;
		delete m_bp;
		m_bp=0;
		delete m_dispatcher;
		m_dispatcher=0;
		delete m_config;
		m_config=0;
	}

	virtual ~Bullet2RigidBodyDemo()
	{
		btAssert(m_config == 0);
		btAssert(m_dispatcher == 0);
		btAssert(m_bp == 0);
		btAssert(m_solver == 0);
		btAssert(m_dynamicsWorld == 0);
	}

};

class BasicDemo : public Bullet2RigidBodyDemo
{
	SimpleOpenGL3App* m_glApp;

public:
	BasicDemo(SimpleOpenGL3App* app)
	:m_glApp(app)
	{
	}
	virtual ~BasicDemo()
	{
	}

	void	initPhysics()
	{
		Bullet2RigidBodyDemo::initPhysics();

		//create ground
		int cubeShapeId = m_glApp->registerCubeShape();
		float pos[]={0,0,0};
		float orn[]={0,0,0,1};
		

		{
			float color[]={0.3,0.3,1,1};
			float halfExtents[]={50,50,50,1};
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

		{
			float halfExtents[]={1,1,1,1};
			float color[]={0,1,0,1};

			btTransform startTransform;
			startTransform.setIdentity();
			btScalar mass = 1.f;
			btVector3 localInertia;
			btBoxShape* colShape = new btBoxShape(btVector3(halfExtents[0],halfExtents[1],halfExtents[2]));
			colShape ->calculateLocalInertia(mass,localInertia);

			for (int k=0;k<ARRAY_SIZE_Y;k++)
			{
				for (int i=0;i<ARRAY_SIZE_X;i++)
				{
					for(int j = 0;j<ARRAY_SIZE_Z;j++)
					{
						startTransform.setOrigin(btVector3(
											btScalar(2.0*i),
											btScalar(20+2.0*k),
											btScalar(2.0*j)));

						m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,startTransform.getOrigin(),startTransform.getRotation(),color,halfExtents);
			
						//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
						btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
						btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
						btRigidBody* body = new btRigidBody(rbInfo);
					

						m_dynamicsWorld->addRigidBody(body);
					}
				}
			}
		}

		m_glApp->m_instancingRenderer->writeTransforms();
	}
	void	exitPhysics()
	{
		Bullet2RigidBodyDemo::exitPhysics();
	}
	void	drawObjects()
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
	void	stepSimulation()
	{
		m_dynamicsWorld->stepSimulation(1./60,0);
	}
};

int main(int argc, char* argv[])
{
	
	float dt = 1./120.f;
	
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet 2 CPU BasicDemo",1024,768);
	app->m_instancingRenderer->setCameraDistance(40);
	app->m_instancingRenderer->setCameraPitch(0);
	app->m_instancingRenderer->setCameraTargetPosition(b3MakeVector3(0,0,0));

	BasicDemo* demo = new BasicDemo(app);
	demo->initPhysics();

	GLint err = glGetError();
    assert(err==GL_NO_ERROR);
	
	do
	{
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera();
		
		demo->stepSimulation();
		demo->drawObjects();
		app->drawGrid(10,0.01);
		char bla[1024];
		static int frameCount = 0;
		frameCount++;
		sprintf(bla,"Simulation frame %d", frameCount);
		
		app->drawText(bla,10,10);
		app->swapBuffer();
	} while (!app->m_window->requestedExit());


	demo->exitPhysics();
	delete demo;

	delete app;
	return 0;
}
