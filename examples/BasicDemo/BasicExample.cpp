/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BasicExample.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include <iostream>

struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	virtual void stepSimulation(float deltaTime);
	void animate(float deltaTime);
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
	btRigidBody* m_kinematicBody1;
	btRigidBody* m_kinematicBody2;
	btScalar m_kinematicVelocity1;
	btScalar m_kinematicVelocity2;
};

void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));

	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1), STATIC_OBJECT);
	}

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(.1, .1, .1));

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		for (int k = 0; k < ARRAY_SIZE_Y; k++)
		{
			for (int i = 0; i < ARRAY_SIZE_X; i++)
			{
				for (int j = 0; j < ARRAY_SIZE_Z; j++)
				{
					startTransform.setOrigin(btVector3(
						btScalar(0.2 * i),
						btScalar(2 + .2 * k),
						btScalar(0.2 * j)));

					createRigidBody(mass, startTransform, colShape, btVector4(1, 0, 0, 1), DYNAMIC_OBJECT);
				}
			}
		}

		startTransform.setOrigin(btVector3(
			btScalar(-0.5),
			btScalar(0.1),
			btScalar(0.5)));
		m_kinematicBody1 = createRigidBody(mass, startTransform, colShape, btVector4(1, 0, 0, 1), KINEMATIC_OBJECT);

		startTransform.setOrigin(btVector3(
			btScalar(1.5),
			btScalar(0.1),
			btScalar(0.5)));
		m_kinematicBody2 = createRigidBody(mass, startTransform, colShape, btVector4(1, 0, 0, 1), KINEMATIC_OBJECT);
	}

	m_kinematicVelocity1 = 1.0;
	m_kinematicVelocity2 = 0.1;

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BasicExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

void BasicExample::stepSimulation(float deltaTime)
{
	animate(deltaTime);
	CommonRigidBodyBase::stepSimulation(deltaTime); 
}

void backAndForthMovement(btRigidBody* kinematicBody, btScalar& kinematicVelocity, float deltaTime)
{
	btTransform currentTransform = kinematicBody->getCenterOfMassTransform();
	btVector3 currentOrigin = currentTransform.getOrigin();
	if((kinematicVelocity > 0.0 && currentOrigin.x() >= 1.5) || (kinematicVelocity < 0.0 && currentOrigin.x() <= -0.5))
	{
		kinematicVelocity *= -1.0;
	}
	currentOrigin.setX(currentOrigin.x() + kinematicVelocity * deltaTime);
	currentTransform.setOrigin(currentOrigin);
	kinematicBody->setCenterOfMassTransform(currentTransform);
	kinematicBody->getMotionState()->setWorldTransform(currentTransform);
}

void BasicExample::animate(float deltaTime)
{
	deltaTime = 1./60.;
	backAndForthMovement(m_kinematicBody1, m_kinematicVelocity1, deltaTime);
	backAndForthMovement(m_kinematicBody2, m_kinematicVelocity2, deltaTime);
}

CommonExampleInterface* BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)
