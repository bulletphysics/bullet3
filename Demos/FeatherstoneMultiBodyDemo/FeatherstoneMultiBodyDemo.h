/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///experimental support for Featherstone multi body (articulated hierarchies)

#ifndef FEATHERSTONE_MULTIBODY_DEMO_H
#define FEATHERSTONE_MULTIBODY_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include "LinearMath/btAlignedObjectArray.h"

class btMultiBody;
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///FeatherstoneMultiBodyDemo is good starting point for learning the code base and porting.

class FeatherstoneMultiBodyDemo : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	
	btMultiBody* createFeatherstoneMultiBody(class btMultiBodyDynamicsWorld* world, int numLinks, const btVector3& basePosition,bool isFixedBase, bool usePrismatic, bool canSleep);

	public:

	FeatherstoneMultiBodyDemo()
	{
	}
	virtual ~FeatherstoneMultiBodyDemo()
	{
		exitPhysics();
	}
	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
	virtual void	clientResetScene();
	
	static DemoApplication* Create()
	{
		FeatherstoneMultiBodyDemo* demo = new FeatherstoneMultiBodyDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	
};

#endif //FEATHERSTONE_MULTIBODY_DEMO_H

