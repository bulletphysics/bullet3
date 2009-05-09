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
#ifndef BSP_DEMO_H
#define BSP_DEMO_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;


///BspDemo shows the convex collision detection, by converting a Quake BSP file into convex objects and allowing interaction with boxes.
class BspDemo : public GlutDemoApplication
{
	public:

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;


	

	virtual ~BspDemo();

	virtual void	initPhysics();

	void	initPhysics(const char* bspfilename);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
	
	static DemoApplication* Create()
	{
		BspDemo* demo = new BspDemo;
		demo->myinit();
		demo->initPhysics("BspDemo.bsp");
		return demo;
	}

};

#endif //BSP_DEMO_H


