#include "Bullet2RigidBodyDemo.h"
#include "btBulletDynamicsCommon.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"

struct MyGraphicsPhysicsBridge : public GraphicsPhysicsBridge
{
	SimpleOpenGL3App* m_glApp;

	MyGraphicsPhysicsBridge(SimpleOpenGL3App* glApp)
		:m_glApp(glApp)
	{
	}
	virtual void createRigidBodyGraphicsObject(btRigidBody* body, const btVector3& color)
	{
		btCollisionShape* shape = body->getCollisionShape();
		btTransform startTransform = body->getWorldTransform();
		int graphicsShapeId = shape->getUserIndex();
		btAssert(graphicsShapeId >= 0);
		btVector3 localScaling = shape->getLocalScaling();
		int graphicsInstanceId = m_glApp->m_instancingRenderer->registerGraphicsInstance(graphicsShapeId, startTransform.getOrigin(), startTransform.getRotation(), color, localScaling);
		body->setUserIndex(graphicsInstanceId);
	}
	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
	{
		//todo: support all collision shape types
		switch (collisionShape->getShapeType())
		{
		case BOX_SHAPE_PROXYTYPE:
		{
			btBoxShape* box = (btBoxShape*)collisionShape;
			btVector3 halfExtents = box->getHalfExtentsWithMargin();
			int cubeShapeId = m_glApp->registerCubeShape(halfExtents.x(), halfExtents.y(), halfExtents.z());
			box->setUserIndex(cubeShapeId);
			break;
		}
		default:
		{
				   btAssert(0);
		}
		};
	}
	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)
	{
		int numCollisionObjects = rbWorld->getNumCollisionObjects();
		for (int i = 0; i<numCollisionObjects; i++)
		{
			btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
			btVector3 pos = colObj->getWorldTransform().getOrigin();
			btQuaternion orn = colObj->getWorldTransform().getRotation();
			int index = colObj->getUserIndex();
			if (index >= 0)
			{
				m_glApp->m_instancingRenderer->writeSingleInstanceTransformToCPU(pos, orn, index);
			}
		}
		m_glApp->m_instancingRenderer->writeTransforms();
	}
};

Bullet2RigidBodyDemo::Bullet2RigidBodyDemo(SimpleOpenGL3App* app, CommonPhysicsSetup* physicsSetup)
	:m_glApp(app),
	m_physicsSetup(physicsSetup),
	m_controlPressed(false),
	m_altPressed(false)
{
	
}
void Bullet2RigidBodyDemo::initPhysics()
{
	MyGraphicsPhysicsBridge glBridge(m_glApp);
	m_physicsSetup->initPhysics(glBridge);
	m_glApp->m_instancingRenderer->writeTransforms();

}

void Bullet2RigidBodyDemo::exitPhysics()
{
	
	m_physicsSetup->exitPhysics();

}

void	Bullet2RigidBodyDemo::stepSimulation(float deltaTime)
{
	m_physicsSetup->stepSimulation(deltaTime);

}

void Bullet2RigidBodyDemo::renderScene()
{
	//sync graphics -> physics world transforms

	MyGraphicsPhysicsBridge glBridge(m_glApp);
	m_physicsSetup->syncPhysicsToGraphics(glBridge);

	m_glApp->m_instancingRenderer->renderScene();

}
Bullet2RigidBodyDemo::~Bullet2RigidBodyDemo()
{
}

btVector3	Bullet2RigidBodyDemo::getRayTo(int x,int y)
{
	if (!m_glApp->m_instancingRenderer)
	{
		btAssert(0);
		return btVector3(0,0,0);
	}

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = b3Scalar(2.0) * b3Atan(tanFov);

	btVector3 camPos,camTarget;
	m_glApp->m_instancingRenderer->getCameraPosition(camPos);
	m_glApp->m_instancingRenderer->getCameraTargetPosition(camTarget);

	btVector3	rayFrom = camPos;
	btVector3 rayForward = (camTarget-camPos);
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward*= farPlane;

	btVector3 rightOffset;
	btVector3 m_cameraUp=btVector3(0,1,0);
	btVector3 vertical = m_cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);


	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	b3Scalar aspect;
	float width = m_glApp->m_instancingRenderer->getScreenWidth();
	float height = m_glApp->m_instancingRenderer->getScreenHeight();

	aspect =  width / height;
	
	hor*=aspect;


	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/width;
	btVector3 dVert = vertical * 1.f/height;


	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;
	return rayTo;
}

	
bool	Bullet2RigidBodyDemo::mouseMoveCallback(float x,float y)
{
	btVector3 rayTo = getRayTo(x, y);
	btVector3 rayFrom;
	m_glApp->m_instancingRenderer->getCameraPosition(rayFrom);
	m_physicsSetup->movePickedBody(rayFrom,rayTo);
		
	return false;
}
bool	Bullet2RigidBodyDemo::mouseButtonCallback(int button, int state, float x, float y)
{

	if (state==1)
	{
		if(button==0 && (!m_altPressed && !m_controlPressed))
		{
			btVector3 camPos;
			m_glApp->m_instancingRenderer->getCameraPosition(camPos);

			btVector3 rayFrom = camPos;
			btVector3 rayTo = getRayTo(x,y);

			bool hasPicked = m_physicsSetup->pickBody(rayFrom, rayTo);

				
		}
	} else
	{
		if (button==0)
		{
			m_physicsSetup->removePickingConstraint();
			//remove p2p
		}
	}

	//printf("button=%d, state=%d\n",button,state);
	return false;
}
