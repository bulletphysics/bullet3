/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BASIC_DEMO3D_H
#define BASIC_DEMO3D_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///BasicDemo is good starting point for learning the code base and porting.
class BasicDemo3D : public GlutDemoApplication
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

	BasicDemo3D()
	{
	}
	virtual ~BasicDemo3D()
	{
		exitPhysics();
	}
	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);
	virtual void mouseFunc(int button, int state, int x, int y);
	virtual void	mouseMotionFunc(int x,int y);

	virtual void clientResetScene();

	static DemoApplication* Create()
	{
		BasicDemo3D* demo = new BasicDemo3D;
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
	
	void setWireMode(bool wireOnOff);
};


#endif //BASIC_DEMO3D_H

