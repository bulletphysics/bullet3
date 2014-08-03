
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

///btSoftBody implementation by Nathanael Presson

#ifndef SOFT_DEMO_H
#define SOFT_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletSoftBody/btSoftBody.h"


class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///collisions between two btSoftBody's
class btSoftSoftCollisionAlgorithm;

///collisions between a btSoftBody and a btRigidBody
class btSoftRididCollisionAlgorithm;
class btSoftRigidDynamicsWorld;


///CcdPhysicsDemo shows basic stacking using Bullet physics, and allows toggle of Ccd (using key '1')
class SoftDemo : public PlatformDemoApplication
{
public:

	btAlignedObjectArray<btSoftSoftCollisionAlgorithm*> m_SoftSoftCollisionAlgorithms;

	btAlignedObjectArray<btSoftRididCollisionAlgorithm*> m_SoftRigidCollisionAlgorithms;

	btSoftBodyWorldInfo	m_softBodyWorldInfo;

	

	bool								m_autocam;
	bool								m_cutting;
	bool								m_raycast;
	btScalar							m_animtime;
	btClock								m_clock;
	int									m_lastmousepos[2];
	btVector3							m_impact;
	btSoftBody::sRayCast				m_results;
	btSoftBody::Node*					m_node;
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

	SoftDemo() : m_drag(false)
	{
		setTexturing(true);
		setShadows(true);
	}
	virtual ~SoftDemo()
	{
		exitPhysics();
	}

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	void createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos );

	static DemoApplication* Create()
	{
		SoftDemo* demo = new SoftDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	virtual	void setDrawClusters(bool drawClusters);

	virtual const btSoftRigidDynamicsWorld*	getSoftDynamicsWorld() const
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}

	virtual btSoftRigidDynamicsWorld*	getSoftDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}

	//
	void	clientResetScene();
	void	renderme();
	void	keyboardCallback(unsigned char key, int x, int y);
	void	mouseFunc(int button, int state, int x, int y);
	void	mouseMotionFunc(int x,int y);

};

#define MACRO_SOFT_DEMO(a) class SoftDemo##a : public SoftDemo\
{\
public:\
	static DemoApplication* Create()\
	{\
		SoftDemo* demo = new SoftDemo##a;\
		extern int current_demo;\
		current_demo=a;\
		demo->myinit();\
		demo->initPhysics();\
		return demo;\
	}\
};


MACRO_SOFT_DEMO(0) //Init_Cloth
MACRO_SOFT_DEMO(1) //Init_Pressure
MACRO_SOFT_DEMO(2)//Init_Volume
MACRO_SOFT_DEMO(3)//Init_Ropes
MACRO_SOFT_DEMO(4)//Init_Ropes_Attach
MACRO_SOFT_DEMO(5)//Init_ClothAttach
MACRO_SOFT_DEMO(6)//Init_Sticks
MACRO_SOFT_DEMO(7)//Init_Collide
MACRO_SOFT_DEMO(8)//Init_Collide2
MACRO_SOFT_DEMO(9)//Init_Collide3
MACRO_SOFT_DEMO(10)//Init_Impact
MACRO_SOFT_DEMO(11)//Init_Aero
MACRO_SOFT_DEMO(12)//Init_Friction
MACRO_SOFT_DEMO(13)//Init_Torus
MACRO_SOFT_DEMO(14)//Init_TorusMatch
MACRO_SOFT_DEMO(15)//Init_Bunny
MACRO_SOFT_DEMO(16)//Init_BunnyMatch
MACRO_SOFT_DEMO(17)//Init_Cutting1
MACRO_SOFT_DEMO(18)//Init_ClusterDeform
MACRO_SOFT_DEMO(19)//Init_ClusterCollide1
MACRO_SOFT_DEMO(20)//Init_ClusterCollide2
MACRO_SOFT_DEMO(21)//Init_ClusterSocket
MACRO_SOFT_DEMO(22)//Init_ClusterHinge
MACRO_SOFT_DEMO(23)//Init_ClusterCombine
MACRO_SOFT_DEMO(24)//Init_ClusterCar
MACRO_SOFT_DEMO(25)//Init_ClusterRobot
MACRO_SOFT_DEMO(26)//Init_ClusterStackSoft
MACRO_SOFT_DEMO(27)//Init_ClusterStackMixed
MACRO_SOFT_DEMO(28)//Init_TetraCube
MACRO_SOFT_DEMO(29)//Init_TetraBunny


#endif //CCD_PHYSICS_DEMO_H





