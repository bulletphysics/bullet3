/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "MultiThreadedDemo.h"
#include "CommonRigidBodyMTBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "LinearMath/btAlignedObjectArray.h"

#include <stdio.h>  //printf debugging
#include <algorithm>
#include <cmath>

static btScalar gSliderStackRows = 1.0f;
static btScalar gSliderStackColumns = 1.0f;
static btScalar gSliderStackHeight = 15.0f;
static btScalar gSliderStackWidth = 8.0f;
static btScalar gSliderGroundHorizontalAmplitude = 0.0f;
static btScalar gSliderGroundVerticalAmplitude = 0.0f;
static btScalar gSliderGroundTilt = 0.0f;
static btScalar gSliderRollingFriction = 0.0f;
static bool gSpheresNotBoxes = false;

static void boolPtrButtonCallback(int buttonId, bool buttonState, void* userPointer)
{
	if (bool* val = static_cast<bool*>(userPointer))
	{
		*val = !*val;
	}
}

/// MultiThreadedDemo shows how to setup and use multithreading
class MultiThreadedDemo : public CommonRigidBodyMTBase
{
	static const int kUpAxis = 1;

	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

	btVector3 m_cameraTargetPos;
	float m_cameraPitch;
	float m_cameraYaw;
	float m_cameraDist;
	btRigidBody* m_groundBody;
	btTransform m_groundStartXf;
	float m_groundMovePhase;
	float m_prevRollingFriction;

	void createStack(const btTransform& trans, btCollisionShape* boxShape, const btVector3& halfBoxSize, int size, int width);
	void createSceneObjects();
	void destroySceneObjects();

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	MultiThreadedDemo(struct GUIHelperInterface* helper);

	virtual ~MultiThreadedDemo() {}

	btQuaternion getGroundRotation() const
	{
		btScalar tilt = gSliderGroundTilt * SIMD_2_PI / 360.0f;
		return btQuaternion(btVector3(1.0f, 0.0f, 0.0f), tilt);
	}
	struct TestSumBody : public btIParallelSumBody
	{
		virtual btScalar sumLoop(int iBegin, int iEnd) const BT_OVERRIDE
		{
			btScalar sum = 0.0f;
			for (int i = iBegin; i < iEnd; ++i)
			{
				if (i > 0)
				{
					sum += 1.0f / btScalar(i);
				}
			}
			return sum;
		}
	};
	virtual void stepSimulation(float deltaTime) BT_OVERRIDE
	{
		if (m_dynamicsWorld)
		{
			if (m_prevRollingFriction != gSliderRollingFriction)
			{
				m_prevRollingFriction = gSliderRollingFriction;
				btCollisionObjectArray& objArray = m_dynamicsWorld->getCollisionObjectArray();
				for (int i = 0; i < objArray.size(); ++i)
				{
					btCollisionObject* obj = objArray[i];
					obj->setRollingFriction(gSliderRollingFriction);
				}
			}
			if (m_groundBody)
			{
				// update ground
				const float cyclesPerSecond = 1.0f;
				m_groundMovePhase += cyclesPerSecond * deltaTime;
				m_groundMovePhase -= std::floor(m_groundMovePhase);  // keep phase between 0 and 1
				btTransform xf = m_groundStartXf;
				float gndHOffset = btSin(m_groundMovePhase * SIMD_2_PI) * gSliderGroundHorizontalAmplitude;
				float gndHVel = btCos(m_groundMovePhase * SIMD_2_PI) * gSliderGroundHorizontalAmplitude * cyclesPerSecond * SIMD_2_PI;  // d(gndHOffset)/dt
				float gndVOffset = btSin(m_groundMovePhase * SIMD_2_PI) * gSliderGroundVerticalAmplitude;
				float gndVVel = btCos(m_groundMovePhase * SIMD_2_PI) * gSliderGroundVerticalAmplitude * cyclesPerSecond * SIMD_2_PI;  // d(gndVOffset)/dt
				btVector3 offset(0, 0, 0);
				btVector3 vel(0, 0, 0);
				int horizAxis = 2;
				offset[horizAxis] = gndHOffset;
				vel[horizAxis] = gndHVel;
				offset[kUpAxis] = gndVOffset;
				vel[kUpAxis] = gndVVel;
				xf.setOrigin(xf.getOrigin() + offset);
				xf.setRotation(getGroundRotation());
				m_groundBody->setWorldTransform(xf);
				m_groundBody->setLinearVelocity(vel);
			}
			// always step by 1/60 for benchmarking
			m_dynamicsWorld->stepSimulation(1.0f / 60.0f, 0);
		}
#if 0
        {
            // test parallelSum
            TestSumBody testSumBody;
            float testSum = btParallelSum( 1, 10000000, 10000, testSumBody );
            printf( "sum = %f\n", testSum );
        }
#endif
	}

	virtual void initPhysics() BT_OVERRIDE;
	virtual void resetCamera() BT_OVERRIDE
	{
		m_guiHelper->resetCamera(m_cameraDist,
								 m_cameraYaw,
								 m_cameraPitch,
								 m_cameraTargetPos.x(),
								 m_cameraTargetPos.y(),
								 m_cameraTargetPos.z());
	}
};

MultiThreadedDemo::MultiThreadedDemo(struct GUIHelperInterface* helper)
	: CommonRigidBodyMTBase(helper)
{
	m_groundBody = NULL;
	m_groundMovePhase = 0.0f;
	m_cameraTargetPos = btVector3(0.0f, 0.0f, 0.0f);
	m_cameraPitch = -30.0f;
	m_cameraYaw = 90.0f;
	m_cameraDist = 48.0f;
	m_prevRollingFriction = -1.0f;
	helper->setUpAxis(kUpAxis);
}

void MultiThreadedDemo::initPhysics()
{
	createEmptyDynamicsWorld();

	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

	{
		SliderParams slider("Stack height", &gSliderStackHeight);
		slider.m_minVal = 1.0f;
		slider.m_maxVal = 30.0f;
		slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		SliderParams slider("Stack width", &gSliderStackWidth);
		slider.m_minVal = 1.0f;
		slider.m_maxVal = 30.0f;
		slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		SliderParams slider("Stack rows", &gSliderStackRows);
		slider.m_minVal = 1.0f;
		slider.m_maxVal = 20.0f;
		slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		SliderParams slider("Stack columns", &gSliderStackColumns);
		slider.m_minVal = 1.0f;
		slider.m_maxVal = 20.0f;
		slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		// horizontal ground shake
		SliderParams slider("Ground horiz amp", &gSliderGroundHorizontalAmplitude);
		slider.m_minVal = 0.0f;
		slider.m_maxVal = 1.0f;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		// vertical ground shake
		SliderParams slider("Ground vert amp", &gSliderGroundVerticalAmplitude);
		slider.m_minVal = 0.0f;
		slider.m_maxVal = 1.0f;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		// ground tilt
		SliderParams slider("Ground tilt", &gSliderGroundTilt);
		slider.m_minVal = -45.0f;
		slider.m_maxVal = 45.0f;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		// rolling friction
		SliderParams slider("Rolling friction", &gSliderRollingFriction);
		slider.m_minVal = 0.0f;
		slider.m_maxVal = 1.0f;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		ButtonParams button("Spheres not boxes", 0, false);
		button.m_initialState = gSpheresNotBoxes;
		button.m_userPointer = &gSpheresNotBoxes;
		button.m_callback = boolPtrButtonCallback;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
	}

	createSceneObjects();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
}

btRigidBody* MultiThreadedDemo::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btRigidBody* body = createRigidBody(mass, startTransform, shape);
	if (mass > 0.0f)
	{
		// prevent bodies from sleeping to make profiling/benchmarking easier
		body->forceActivationState(DISABLE_DEACTIVATION);
	}
	return body;
}

void MultiThreadedDemo::createStack(const btTransform& parentTrans, btCollisionShape* boxShape, const btVector3& halfBoxSize, int height, int width)
{
	btTransform trans;
	trans.setIdentity();
	trans.setRotation(parentTrans.getRotation());
	float halfBoxHeight = halfBoxSize.y();
	float halfBoxWidth = halfBoxSize.x();

	btVector3 offset = btVector3(0, 0, -halfBoxSize.z() * (width - 1));
	for (int iZ = 0; iZ < width; iZ++)
	{
		offset += btVector3(0, 0, halfBoxSize.z() * 2.0f);
		for (int iY = 0; iY < height; iY++)
		{
			// This constructs a row, from left to right
			int rowSize = height - iY;
			for (int iX = 0; iX < rowSize; iX++)
			{
				btVector3 pos = offset + btVector3(halfBoxWidth * (1 + iX * 2 - rowSize),
												   halfBoxHeight * (1 + iY * 2),
												   0.0f);

				trans.setOrigin(parentTrans(pos));
				btScalar mass = 1.f;

				btRigidBody* body = localCreateRigidBody(mass, trans, boxShape);
				body->setFriction(1.0f);
				body->setRollingFriction(gSliderRollingFriction);
			}
		}
	}
}

void MultiThreadedDemo::createSceneObjects()
{
	{
		// create ground box
		m_groundStartXf.setOrigin(btVector3(0.f, -3.f, 0.f));
		m_groundStartXf.setRotation(getGroundRotation());

		//either use heightfield or triangle mesh

		btVector3 groundExtents(400, 400, 400);
		groundExtents[kUpAxis] = 3;
		btCollisionShape* groundShape = new btBoxShape(groundExtents);
		m_collisionShapes.push_back(groundShape);

		//create ground object
		m_groundBody = createKinematicBody(m_groundStartXf, groundShape);
		m_groundBody->forceActivationState(DISABLE_DEACTIVATION);
		m_groundBody->setFriction(1.0f);
	}

	{
		// create walls of cubes
		const btVector3 halfExtents = btVector3(0.5f, 0.25f, 0.5f);
		int numStackRows = btMax(1, int(gSliderStackRows));
		int numStackCols = btMax(1, int(gSliderStackColumns));
		int stackHeight = int(gSliderStackHeight);
		int stackWidth = int(gSliderStackWidth);
		float stackZSpacing = 2.0f + stackWidth * halfExtents.x() * 2.0f;
		float stackXSpacing = 20.0f;

		btBoxShape* boxShape = new btBoxShape(halfExtents);
		m_collisionShapes.push_back(boxShape);

		btSphereShape* sphereShape = new btSphereShape(0.5f);
		m_collisionShapes.push_back(sphereShape);

		btCollisionShape* shape = boxShape;
		if (gSpheresNotBoxes)
		{
			shape = sphereShape;
		}

		btTransform groundTrans;
		groundTrans.setIdentity();
		groundTrans.setRotation(getGroundRotation());
		for (int iX = 0; iX < numStackCols; ++iX)
		{
			for (int iZ = 0; iZ < numStackRows; ++iZ)
			{
				btVector3 center = btVector3(iX * stackXSpacing, 0.0f, (iZ - numStackRows / 2) * stackZSpacing);
				btTransform trans = groundTrans;
				trans.setOrigin(groundTrans(center));
				createStack(trans, shape, halfExtents, stackHeight, stackWidth);
			}
		}
	}
#if 0
    if ( false )
    {
        // destroyer ball
        btTransform sphereTrans;
        sphereTrans.setIdentity();
        sphereTrans.setOrigin( btVector3( 0, 2, 40 ) );
        btSphereShape* ball = new btSphereShape( 2.f );
        m_collisionShapes.push_back( ball );
        btRigidBody* ballBody = localCreateRigidBody( 10000.f, sphereTrans, ball );
        ballBody->setLinearVelocity( btVector3( 0, 0, -10 ) );
    }
#endif
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

CommonExampleInterface* MultiThreadedDemoCreateFunc(struct CommonExampleOptions& options)
{
	return new MultiThreadedDemo(options.m_guiHelper);
}
