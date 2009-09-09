/*
Bullet Continuous Collision Detection and Physics Library Copyright (c) 2007 Erwin Coumans
Motor Demo

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef MOTORDEMO_H
#define MOTORDEMO_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class MotorDemo : public GlutDemoApplication
{
	float m_Time;
	float m_fCyclePeriod; // in milliseconds
	float m_fMuscleStrength;

	btAlignedObjectArray<class TestRig*> m_rigs;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

public:
	void initPhysics();

	void exitPhysics();

	virtual ~MotorDemo()
	{
		exitPhysics();
	}

	void spawnTestRig(const btVector3& startOffset, bool bFixed);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	static DemoApplication* Create()
	{
		MotorDemo* demo = new MotorDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
	void setMotorTargets(btScalar deltaTime);

};


#endif
