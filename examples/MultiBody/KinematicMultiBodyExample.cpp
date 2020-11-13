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

#include "KinematicMultiBodyExample.h"
//#define USE_MOTIONSTATE 1
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btTransformUtil.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../OpenGLWindow/ShapeData.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

namespace {

void kinematicPreTickCallback(btDynamicsWorld* world, btScalar deltaTime)
{
	btMultiBody* groundBody = (btMultiBody*)world->getWorldUserInfo();
	btTransform predictedTrans;
	btVector3 linearVelocity(0, 0, 0);
	btVector3 angularVelocity(0, 0.1, 0);
	btTransformUtil::integrateTransform(groundBody->getBaseWorldTransform(), linearVelocity, angularVelocity, deltaTime, predictedTrans);
	groundBody->setBaseWorldTransform(predictedTrans);
}

struct KinematicMultiBodyExample : public CommonMultiBodyBase
{
	btMultiBody* m_groundBody;

	KinematicMultiBodyExample(struct GUIHelperInterface* helper)
		: CommonMultiBodyBase(helper),
		  m_groundBody(0)
	{
	}

	virtual void stepSimulation(float deltaTime)
	{
		if (m_dynamicsWorld)
		{
			m_dynamicsWorld->stepSimulation(deltaTime);
		}
	}

	virtual ~KinematicMultiBodyExample() {}
	virtual void initPhysics();
	void resetCamera()
	{
		float dist = 4;
		float pitch = -30;
		float yaw = 50;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void KinematicMultiBodyExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btScalar halfExtentsX = 10.0;
	btScalar halfExtentsY = 0.1;
	btScalar halfExtentsZ = 10.0;

	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(10.), btScalar(0.1), btScalar(10.)));
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -halfExtentsY, 0));
	m_collisionShapes.push_back(groundShape);



	{
		bool floating = false;
		int numLinks = 0;
		bool canSleep = false;
		btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
		float baseMass = 1.f;

		if (baseMass)
		{
			btCollisionShape* pTempBox = new btBoxShape(btVector3(10, 10, 10));
			pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
			delete pTempBox;
		}
		btTransform startTransform;
		startTransform.setIdentity();

		m_groundBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);
		m_groundBody->setBasePos(startTransform.getOrigin());
		m_groundBody->setWorldToBaseRot(startTransform.getRotation());
		m_groundBody->finalizeMultiDof();
		m_dynamicsWorld->addMultiBody(m_groundBody);


		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(m_groundBody, -1);
		col->setCollisionShape(groundShape);
		bool isDynamic = (baseMass > 0 && floating);
		int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
		int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
		m_dynamicsWorld->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);  //, 2,1+2);
		m_groundBody->setBaseCollider(col);
		m_groundBody->setBaseDynamicType(btCollisionObject::CF_KINEMATIC_OBJECT);

	}
	m_dynamicsWorld->setInternalTickCallback(kinematicPreTickCallback, m_groundBody, true);

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

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

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

					createRigidBody(mass, startTransform, colShape);
				}
			}
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

}

CommonExampleInterface* KinematicMultiBodyExampleCreateFunc(CommonExampleOptions& options)
{
	return new KinematicMultiBodyExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(KinematicMultiBodyExampleCreateFunc)
