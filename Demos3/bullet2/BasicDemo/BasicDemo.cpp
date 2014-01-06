#include "BasicDemo.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "btBulletDynamicsCommon.h"
#include "Bullet3Common/b3Vector3.h"

#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5


BasicDemo::BasicDemo(SimpleOpenGL3App* app)
:Bullet2RigidBodyDemo(app)
{
}

BasicDemo::~BasicDemo()
{
}

void	BasicDemo::initPhysics()
{
	Bullet2RigidBodyDemo::initPhysics();
	int curColor=0;
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
		b3Vector4 colors[4] =
		{
			b3MakeVector4(1,0,0,1),
			b3MakeVector4(0,1,0,1),
			b3MakeVector4(0,1,1,1),
			b3MakeVector4(1,1,0,1),
		};
		


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
					
					b3Vector4 color = colors[curColor];
					curColor++;
					curColor&=3;
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
void	BasicDemo::exitPhysics()
{
	
	Bullet2RigidBodyDemo::exitPhysics();
}
void	BasicDemo::renderScene()
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

	
void	BasicDemo::stepSimulation(float dt)
{
	m_dynamicsWorld->stepSimulation(dt);
}
