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
#ifndef CONSTRAINT_DEMO_H
#define CONSTRAINT_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

///ConstraintDemo shows how to create a constraint, like Hinge or btGenericD6constraint
class ConstraintDemo : public PlatformDemoApplication
{
	//keep track of variables to delete memory at the end
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	void	setupEmptyDynamicsWorld();

	void	clientResetScene();


	public:


	virtual ~ConstraintDemo();

	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	static DemoApplication* Create()
	{
		ConstraintDemo* demo = new ConstraintDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	virtual void keyboardCallback(unsigned char key, int x, int y);

	// for cone-twist motor driving
	float m_Time;
	class btConeTwistConstraint* m_ctc;
	
};

#endif //CONSTRAINT_DEMO_H

