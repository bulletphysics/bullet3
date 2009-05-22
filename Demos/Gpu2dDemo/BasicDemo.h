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
#ifndef BASIC_DEMO_H
#define BASIC_DEMO_H

#include "DemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"


#ifdef BT_USE_CUDA
//#include "btCudaDemoPairCache.h"
//#include <vector_types.h>
#endif //BT_USE_CUDA

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
#include "GlutDemoApplication.h"

///BasicDemo is good starting point for learning the code base and porting.
class BasicDemo : public GlutDemoApplication
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	int m_mouseButtons;
	int m_mouseOldX;
	int m_mouseOldY;

	public:

	BasicDemo()
	{
	}
	virtual ~BasicDemo()
	{
		exitPhysics();
	}
	void	initPhysics();

	void	exitPhysics();

	///don't shoot a box in this demo ;-)
	virtual void	shootBox(const btVector3& dest) {}

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	virtual void clientResetScene();

	static DemoApplication* Create()
	{
		BasicDemo* demo = new BasicDemo;
		demo->myinit();
		demo->initPhysics();
		demo->m_mouseButtons = 0;
		demo->m_mouseOldX = 0;
		demo->m_mouseOldY = 0;
		return demo;
	}

	void DrawConstraintInfo();
	void outputDebugInfo(int & xOffset,int & yStart, int  yIncr);
	virtual void renderme();
	void addCollisionShape(btCollisionShape* pShape) { m_collisionShapes.push_back(pShape); }
};


#endif //BASIC_DEMO_H

