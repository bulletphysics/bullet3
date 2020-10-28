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

#include "KinematicRigidBodyExample.h"
//#define USE_MOTIONSTATE 1
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btTransformUtil.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../OpenGLWindow/ShapeData.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"


void kinematicPreTickCallback(btDynamicsWorld* world, btScalar deltaTime)
{
	btRigidBody* groundBody = (btRigidBody*)world->getWorldUserInfo();
	btTransform predictedTrans;
	btVector3 linearVelocity(0, 0, 0);
	btVector3 angularVelocity(0, 0.1, 0);
	btTransformUtil::integrateTransform(groundBody->getWorldTransform(), linearVelocity, angularVelocity, deltaTime, predictedTrans);
#ifdef USE_MOTIONSTATE
	groundBody->getMotionState()->setWorldTransform(predictedTrans);
#else
	m_groundBody->setWorldTransform(predictedTrans);
#endif
}

struct KinematicRigidBodyExample : public CommonRigidBodyBase
{
	btRigidBody* m_groundBody;

	KinematicRigidBodyExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper),
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

	virtual ~KinematicRigidBodyExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 4;
		float pitch = -30;
		float yaw = 50;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void KinematicRigidBodyExample::initPhysics()
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
		btScalar mass(0.);
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);

#ifdef USE_MOTIONSTATE
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, groundShape, localInertia);
		m_groundBody = new btRigidBody(cInfo);
#else
		m_groundBody = new btRigidBody(mass, 0, shape, localInertia);
		m_groundBody->setWorldTransform(startTransform);
#endif  //

		m_groundBody->setUserIndex(-1);
		
		m_groundBody->forceActivationState(DISABLE_DEACTIVATION);
		m_groundBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_STATIC_OBJECT);
		m_dynamicsWorld->addRigidBody(m_groundBody);
		
	}
	m_dynamicsWorld->setInternalTickCallback(kinematicPreTickCallback, m_groundBody, true);
	{
		int strideInBytes = 9 * sizeof(float);
		int numVertices = sizeof(cube_vertices_textured) / strideInBytes;
		int numIndices = sizeof(cube_indices) / sizeof(int);
		btScalar textureScaling = 40.0;
		btAlignedObjectArray<GfxVertexFormat1> verts;
		verts.resize(numVertices);
		for (int i = 0; i < numVertices; i++)
		{
			verts[i].x = halfExtentsX * cube_vertices_textured[i * 9];
			verts[i].y = halfExtentsY * cube_vertices_textured[i * 9 + 1];
			verts[i].z = halfExtentsZ * cube_vertices_textured[i * 9 + 2];
			verts[i].w = cube_vertices_textured[i * 9 + 3];
			verts[i].nx = cube_vertices_textured[i * 9 + 4];
			verts[i].ny = cube_vertices_textured[i * 9 + 5];
			verts[i].nz = cube_vertices_textured[i * 9 + 6];
			verts[i].u = cube_vertices_textured[i * 9 + 7] * textureScaling;
			verts[i].v = cube_vertices_textured[i * 9 + 8] * textureScaling;
		}

		int red = 173;
		int green = 199;
		int blue = 255;
		btAlignedObjectArray<unsigned char> rgbaTexture;
		int textureWidth = 256;
		int textureHeight = 256;
		rgbaTexture.resize(textureWidth * textureHeight * 3);

		for (int i = 0; i < textureWidth * textureHeight * 3; i++)
			rgbaTexture[i] = 255;
		for (int i = 0; i < textureWidth; i++)
		{
			for (int j = 0; j < textureHeight; j++)
			{
				int a = i < textureWidth / 2 ? 1 : 0;
				int b = j < textureWidth / 2 ? 1 : 0;

				if (a == b)
				{
					rgbaTexture[(i + j * textureWidth) * 3 + 0] = red;
					rgbaTexture[(i + j * textureWidth) * 3 + 1] = green;
					rgbaTexture[(i + j * textureWidth) * 3 + 2] = blue;
				}
			}
		}
		bool flipPixelsY = false;
		int textureIndex = m_guiHelper->getRenderInterface()->registerTexture(&rgbaTexture[0], textureWidth, textureHeight, flipPixelsY);
		int shapeId = m_guiHelper->getRenderInterface()->registerShape(&verts[0].x, numVertices, cube_indices, numIndices, B3_GL_TRIANGLES, textureIndex);
		btVector3 scaling(1, 1, 1);
		btVector4 color(1, 1, 1, 1);
		btQuaternion orn;
		groundTransform.getBasis().getRotation(orn);
		int graphicsInstanceId = m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId, groundTransform.getOrigin(), orn, color, scaling);
		groundShape->setUserIndex(shapeId);
		m_groundBody->setUserIndex(graphicsInstanceId);
	
	}

	if (1)
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

void KinematicRigidBodyExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* KinematicRigidBodyExampleCreateFunc(CommonExampleOptions& options)
{
	return new KinematicRigidBodyExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(KinematicRigidBodyExampleCreateFunc)
