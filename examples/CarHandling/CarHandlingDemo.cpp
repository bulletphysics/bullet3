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

	GUIHelperInterface* m_guiHelper;

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
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
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

CarHandlingDemo::CarHandlingDemo(struct GUIHelperInterface* helper) : m_guiHelper(helper)
{
	helper->setUpAxis(1);
}

void CarHandlingDemo::initPhysics()
{
}

void CarHandlingDemo::exitPhysics()
{
}

CarHandlingDemo::~CarHandlingDemo()
{
}

void CarHandlingDemo::physicsDebugDraw(int debugFlags)
{
}

void CarHandlingDemo::renderScene()
{
}

void CarHandlingDemo::stepSimulation(float deltaTime)
{
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