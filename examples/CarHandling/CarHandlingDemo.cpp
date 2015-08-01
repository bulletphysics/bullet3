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

	btRaycastVehicle* vehicle;

	btRigidBody* createGroundRigidBodyFromShape(btCollisionShape* groundShape);

	btRigidBody* createChassisRigidBodyFromShape(btCollisionShape* chassisShape);

	void addWheels(
		btVector3* halfExtents,
		btRaycastVehicle* vehicle,
		btRaycastVehicle::btVehicleTuning tuning);
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
	//Collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

	//Use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	//btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

	//The default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	this->dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	this->dynamicsWorld->setGravity(btVector3(0, -10, 0));

	guiHelper->createPhysicsDebugDrawer(this->dynamicsWorld);

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	{
		//Create a shape and store for reuse
		btCollisionShape* groundShape = new btBoxShape(btVector3(100, 1, 100));

		collisionShapes.push_back(groundShape);

		btRigidBody* groundRigidBody = this->createGroundRigidBodyFromShape(groundShape);

		this->dynamicsWorld->addRigidBody(groundRigidBody);
	}

	{
		//the dimensions for the boxShape are half extents, so 0,5 equals to 
		btVector3 halfExtends(1.f, 0.5f, 2.f);

		//The btBoxShape is centered at the origin
		btCollisionShape* chassisShape = new btBoxShape(halfExtends);

		collisionShapes.push_back(chassisShape);

		btRigidBody* chassisRigidBody = this->createChassisRigidBodyFromShape(chassisShape);

		this->dynamicsWorld->addRigidBody(chassisRigidBody);

		btVehicleRaycaster* vehicleRayCaster = new btDefaultVehicleRaycaster(this->dynamicsWorld);

		btRaycastVehicle::btVehicleTuning tuning;

		this->vehicle = new btRaycastVehicle(tuning, chassisRigidBody, vehicleRayCaster);

		//Never deactivate the vehicle
		chassisRigidBody->setActivationState(DISABLE_DEACTIVATION);

		this->dynamicsWorld->addVehicle(this->vehicle);

		this->addWheels(&halfExtends, this->vehicle, tuning);
	}

	guiHelper->autogenerateGraphicsObjects(this->dynamicsWorld);
}

btRigidBody* CarHandlingDemo::createChassisRigidBodyFromShape(btCollisionShape* chassisShape)
{
	btTransform chassisTransform;
	chassisTransform.setIdentity();
	chassisTransform.setOrigin(btVector3(0, 2, 0));

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

void CarHandlingDemo::addWheels(
	btVector3* halfExtents,
	btRaycastVehicle* vehicle,
	btRaycastVehicle::btVehicleTuning tuning)
{
	btVector3 wheelDirectionCS0(0, -1, 0);

	btVector3 wheelAxleCS(-1, 0, 0);

	btScalar suspensionRestLength(0.6);

	btScalar wheelWidth(0.4f); 

	btScalar wheelRadius(0.5f); 

	btScalar connectionHeight(.2f);

	//All the wheel configuration assumes the vehicle is centered at the origin and a right handed coordinate system is used
	btVector3 wheelConnectionPoint(halfExtents->x() - wheelRadius, connectionHeight, halfExtents->z() - wheelWidth);

	//Adds the front wheels to the btRaycastVehicle by shifting the connection point
	vehicle->addWheel(wheelConnectionPoint, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);

	vehicle->addWheel(wheelConnectionPoint * btVector3(1, 1, -1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);

	//Adds the rear wheels, notice that the last parameter value is false
	vehicle->addWheel(wheelConnectionPoint* btVector3(-1, 1, 1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);

	vehicle->addWheel(wheelConnectionPoint * btVector3(-1, 1, -1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);

	//Configures each wheel of our vehicle, setting its friction, damping compression, etc.
	for (int i = 0; i < vehicle->getNumWheels(); i++)
	{
		btWheelInfo& wheel = vehicle->getWheelInfo(i);
		wheel.m_suspensionStiffness = 20.f;
		wheel.m_wheelsDampingRelaxation = 2.3f;
		wheel.m_wheelsDampingCompression = 4.4f;
		wheel.m_frictionSlip = 1000;
		wheel.m_rollInfluence = 0.1f;
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

	if (key == B3G_RIGHT_ARROW)
	{
		vehicle->setBrake(100, 2);
		vehicle->setBrake(100, 3);
		handled = true;
	}

	return handled;
}

CommonExampleInterface* CarHandlingCreateFunc(struct CommonExampleOptions& options)
{
	return new CarHandlingDemo(options.m_guiHelper);
}