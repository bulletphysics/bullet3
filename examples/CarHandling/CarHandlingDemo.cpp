/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"

class CarHandlingDemo : public CommonExampleInterface
{
public:

	CarHandlingDemo(struct GUIHelperInterface* helper);

	void initPhysics();

	void exitPhysics();

	virtual ~CarHandlingDemo();

	virtual void stepSimulation(float deltaTime);

	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}

	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}

	virtual bool keyboardCallback(int key, int state);

	virtual void renderScene();

	virtual void physicsDebugDraw(int debugFlags);

	virtual void resetCamera()
	{
		float dist = 8;
		float pitch = -45;
		float yaw = 32;
		float targetPos[3] = { -0.33, -0.72, 4.5 };
		guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}

private:
	GUIHelperInterface* guiHelper;
	btDiscreteDynamicsWorld* dynamicsWorld;

	btRigidBody* createGroundRigidBodyFromShape(btCollisionShape* groundShape);

	btRigidBody* createChassisRigidBodyFromShape(btCollisionShape* chassisShape);
};

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

#include <stdio.h>
#include "CarHandlingDemo.h"

CarHandlingDemo::CarHandlingDemo(struct GUIHelperInterface* helper) : guiHelper(helper)
{
	helper->setUpAxis(1);
}

void CarHandlingDemo::initPhysics()
{
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	this->dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	this->dynamicsWorld->setGravity(btVector3(0, -10, 0));
	guiHelper->createPhysicsDebugDrawer(this->dynamicsWorld);
	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	{
		///create a shape and store for reuse
		btCollisionShape* groundShape = new btBoxShape(btVector3(100, 1, 100));

		collisionShapes.push_back(groundShape);

		btRigidBody* groundRigidBody = this->createGroundRigidBodyFromShape(groundShape);

		this->dynamicsWorld->addRigidBody(groundRigidBody);
	}

	{
		btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));

		collisionShapes.push_back(chassisShape);

		btCompoundShape* compound = new btCompoundShape();
		
		collisionShapes.push_back(compound);

		btTransform localTrans;
		localTrans.setIdentity();
		//localTrans effectively shifts the center of mass with respect to the chassis
		localTrans.setOrigin(btVector3(0, 1, 0));

		compound->addChildShape(localTrans, chassisShape);

		btRigidBody* chassisRigidBody = this->createChassisRigidBodyFromShape(compound);

		this->dynamicsWorld->addRigidBody(chassisRigidBody);

		btVehicleRaycaster* vehicleRayCaster = new btDefaultVehicleRaycaster(this->dynamicsWorld);

		btRaycastVehicle::btVehicleTuning tuning;

		btRaycastVehicle* vehicle = new btRaycastVehicle(tuning, chassisRigidBody, vehicleRayCaster);

		chassisRigidBody->setActivationState(DISABLE_DEACTIVATION);

		this->dynamicsWorld->addVehicle(vehicle);

		btVector3 wheelDirectionCS0(0, -1, 0);
		btVector3 wheelAxleCS(-1, 0, 0);
		btScalar suspensionRestLength(0.6);
		float	wheelRadius = 0.5f;
		float	wheelWidth = 0.4f;
		float connectionHeight = 1.2f;
		bool isFrontWheel = true;
		//choose coordinate system
		vehicle->setCoordinateSystem(0, 1, 2);
		//
#define CUBE_HALF_EXTENTS 1


		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS - (0.3*wheelWidth), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);

		vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, isFrontWheel);
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3*wheelWidth), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);

		vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, isFrontWheel);
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3*wheelWidth), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);

		isFrontWheel = false;
		vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, isFrontWheel);
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS - (0.3*wheelWidth), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);
		vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, isFrontWheel);

		for (int i = 0; i < vehicle->getNumWheels(); i++)
		{
			btWheelInfo& wheel = vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = 20.f;
			wheel.m_wheelsDampingRelaxation = 2.3f;
			wheel.m_wheelsDampingCompression = 4.4f;
			wheel.m_frictionSlip = 1000;
			wheel.m_rollInfluence = 0.1f;
		}

		//chassisRigidBody->setCenterOfMassTransform(btTransform::getIdentity());
		chassisRigidBody->setLinearVelocity(btVector3(0, 0, 0));
		chassisRigidBody->setAngularVelocity(btVector3(0, 0, 0));


		if (vehicle)
		{
			vehicle->resetSuspension();
			for (int i = 0; i < vehicle->getNumWheels(); i++)
			{
				//synchronize the wheels with the (interpolated) chassis worldtransform
				vehicle->updateWheelTransform(i, true);
			}
		}
	}
	guiHelper->autogenerateGraphicsObjects(this->dynamicsWorld);
}

btRigidBody* CarHandlingDemo::createChassisRigidBodyFromShape(btCollisionShape* chassisShape)
{
	btTransform chassisTransform;
	chassisTransform.setIdentity();
	chassisTransform.setOrigin(btVector3(0, 0, 0));

	{
		//chassis mass, its dynamic, so we calculate its local inertia
		btScalar mass(800);

		btVector3 localInertia(0, 0, 0);
		chassisShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* groundMotionState = new btDefaultMotionState(chassisTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, groundMotionState, chassisShape, localInertia);

		return new btRigidBody(rbInfo);
	}
}

btRigidBody* CarHandlingDemo::createGroundRigidBodyFromShape(btCollisionShape* groundShape)
{
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -1, 0));

	{
		//since the ground will not move, we set its mass to 0
		btScalar mass(0.);
		btVector3 localInertia(0, 0, 0);

		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* groundMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, groundMotionState, groundShape, localInertia);

		return new btRigidBody(rbInfo);
	}
}

void CarHandlingDemo::exitPhysics()
{
}

CarHandlingDemo::~CarHandlingDemo()
{
}

void CarHandlingDemo::physicsDebugDraw(int debugFlags)
{
	if (this->dynamicsWorld && this->dynamicsWorld->getDebugDrawer())
	{
		this->dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		this->dynamicsWorld->debugDrawWorld();
	}
}

void CarHandlingDemo::renderScene()
{
	this->guiHelper->syncPhysicsToGraphics(this->dynamicsWorld);
	this->guiHelper->render(this->dynamicsWorld);
}

void CarHandlingDemo::stepSimulation(float deltaTime)
{
	dynamicsWorld->stepSimulation(deltaTime, 2);
}

bool CarHandlingDemo::keyboardCallback(int key, int state)
{
	bool handled = false;
	return handled;
}

CommonExampleInterface* CarHandlingCreateFunc(struct CommonExampleOptions& options)
{
	return new CarHandlingDemo(options.m_guiHelper);
}