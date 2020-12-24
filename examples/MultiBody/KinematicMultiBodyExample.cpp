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
	groundBody->setBaseVel(linearVelocity);
	groundBody->setBaseOmega(angularVelocity);

	static float time = 0.0;
	time += deltaTime;
	double old_joint_pos = groundBody->getJointPos(0);
	double joint_pos = 0.5 * sin(time * 3.0 - 0.3);
	double joint_vel = (joint_pos - old_joint_pos) / deltaTime;
	groundBody->setJointPosMultiDof(0, &joint_pos);
	groundBody->setJointVelMultiDof(0, &joint_vel);
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

	///create a kinematic multibody
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(10.), btScalar(0.1), btScalar(10.)));
	m_collisionShapes.push_back(groundShape);

	btBoxShape* secondLevelShape = createBoxShape(btVector3(btScalar(0.5), btScalar(0.1), btScalar(0.5)));
	m_collisionShapes.push_back(secondLevelShape);

	{
		bool floating = false;
		int numLinks = 1;
		bool canSleep = false;
		btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
		float baseMass = 1.f;
		btVector3 secondLevelInertiaDiag(0.f, 0.f, 0.f);
		float secondLevelMass = 0.1f;

		if (baseMass)
		{
			btCollisionShape* pTempBox = new btBoxShape(btVector3(10, 10, 10));
			pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
			delete pTempBox;
		}
		if (secondLevelMass)
		{
			btCollisionShape* pTempBox = new btBoxShape(btVector3(0.5, 0.5, 0.5));
			pTempBox->calculateLocalInertia(secondLevelMass, secondLevelInertiaDiag);
			delete pTempBox;
		}
		btTransform startTransform;
		startTransform.setIdentity();

		m_groundBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);
		m_groundBody->setBasePos(startTransform.getOrigin());
		m_groundBody->setWorldToBaseRot(startTransform.getRotation());

		//init the child link - second level.
		btVector3 hingeJointAxis(0, 1, 0);
		m_groundBody->setupRevolute(0, secondLevelMass, secondLevelInertiaDiag, -1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, btVector3(0, 0.5, 0), btVector3(0, 0, 0), true);

		m_groundBody->finalizeMultiDof();
		m_dynamicsWorld->addMultiBody(m_groundBody);

		// add collision geometries
		bool isDynamic = false; // Kinematic is not treated as dynamic here.
		int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
		int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(m_groundBody, -1);
		col->setCollisionShape(groundShape);
		m_dynamicsWorld->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);  //, 2,1+2);
		m_groundBody->setBaseCollider(col);
		m_groundBody->setBaseDynamicType(btCollisionObject::CF_KINEMATIC_OBJECT);

		btMultiBodyLinkCollider* secondLevelCol = new btMultiBodyLinkCollider(m_groundBody, 0);
		secondLevelCol->setCollisionShape(secondLevelShape);
		m_dynamicsWorld->addCollisionObject(secondLevelCol, collisionFilterGroup, collisionFilterMask);
		m_groundBody->getLink(0).m_collider = secondLevelCol;
		m_groundBody->setLinkDynamicType(0, btCollisionObject::CF_KINEMATIC_OBJECT);
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
