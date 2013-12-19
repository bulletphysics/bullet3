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

struct btMultiBodySettings
{
	btMultiBodySettings()
	{
		m_numLinks = 0;
		m_basePosition.setZero();
		m_isFixedBase = true;
		m_usePrismatic = false;
		m_canSleep = true;
		m_createConstraints = false;
		m_disableParentCollision = false;
	}
	int			m_numLinks;
	btVector3	m_basePosition;
	bool		m_isFixedBase;
	bool		m_usePrismatic;
	bool		m_canSleep;
	bool		m_createConstraints;
	bool		m_disableParentCollision;
};

class FeatherstoneMultiBodyDemo : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	virtual void	mouseMotionFunc(int x,int y);
	virtual void removePickingConstraint();
	virtual void pickObject(const btVector3& pickPos, const class btCollisionObject* hitObj);
	class btMultiBodyPoint2Point*		m_pickingMultiBodyPoint2Point;
	
	btMultiBody* createFeatherstoneMultiBody(class btMultiBodyDynamicsWorld* world, const btMultiBodySettings& settings);

	public:

	FeatherstoneMultiBodyDemo()
		:m_pickingMultiBodyPoint2Point(0)
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

