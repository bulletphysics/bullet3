/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Experimental Buoyancy fluid demo written by John McCutchan
*/

#ifndef HFFLUID_DEMO_H
#define HFFLUID_DEMO_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletHfFluid/btHfFluid.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class btHfFluidRigidDynamicsWorld;
///collisions between a btSoftBody and a btRigidBody
class btFluidRididCollisionAlgorithm;

///experimental buyancy fluid demo
///CcdPhysicsDemo shows basic stacking using Bullet physics, and allows toggle of Ccd (using key '1')
class HfFluidDemo : public GlutDemoApplication
{
public:
	btAlignedObjectArray<btFluidRididCollisionAlgorithm*> m_FluidRigidCollisionAlgorithms;

	
	bool								m_autocam;
	bool								m_cutting;
	bool								m_raycast;
	btScalar							m_animtime;
	btClock								m_clock;
	int									m_lastmousepos[2];
	btVector3							m_impact;
	btVector3							m_goal;
	bool								m_drag;


	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>		m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;


	btConstraintSolver*	m_solver;

	btCollisionAlgorithmCreateFunc*	m_boxBoxCF;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

public:

	void	initPhysics();

	void	exitPhysics();

	HfFluidDemo ();

	virtual ~HfFluidDemo()
	{
		exitPhysics();
	}

	virtual	void setDrawClusters(bool drawClusters)
	{

	}

	virtual void setShootBoxShape ();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	void createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos );

	static DemoApplication* Create()
	{
		HfFluidDemo* demo = new HfFluidDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	virtual const btHfFluidRigidDynamicsWorld*	getHfFluidDynamicsWorld() const
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btHfFluidRigidDynamicsWorld*) m_dynamicsWorld;
	}

	virtual btHfFluidRigidDynamicsWorld*	getHfFluidDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btHfFluidRigidDynamicsWorld*) m_dynamicsWorld;
	}

	//
	void	clientResetScene();
	void	renderme();
	void	keyboardCallback(unsigned char key, int x, int y);
	void	mouseFunc(int button, int state, int x, int y);
	void	mouseMotionFunc(int x,int y);

};

#define MACRO_SOFT_DEMO(a) class HfFluidDemo##a : public HfFluidDemo\
{\
public:\
	static DemoApplication* Create()\
	{\
		HfFluidDemo* demo = new HfFluidDemo##a;\
		extern unsigned int current_demo;\
		current_demo=a;\
		demo->myinit();\
		demo->initPhysics();\
		return demo;\
	}\
};


MACRO_SOFT_DEMO(0) //Init_Drops
MACRO_SOFT_DEMO(1) //Init_Wave
MACRO_SOFT_DEMO(2) //Init_RandomDrops
MACRO_SOFT_DEMO(3)

#endif //CCD_PHYSICS_DEMO_H





