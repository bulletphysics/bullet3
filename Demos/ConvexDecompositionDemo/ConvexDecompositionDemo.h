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
#ifndef CONVEX_DECOMPOSITION_DEMO_H
#define CONVEX_DECOMPOSITION_DEMO_H


#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include "LinearMath/btAlignedObjectArray.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class btTriangleMesh;

///ConvexDecompositionDemo shows automatic convex decomposition of a concave mesh
ATTRIBUTE_ALIGNED16(class) ConvexDecompositionDemo : public PlatformDemoApplication
{

	void setupEmptyDynamicsWorld();
public:
	
	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btAlignedObjectArray<btTriangleMesh*> m_trimeshes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;
	
	btDefaultCollisionConfiguration* m_collisionConfiguration;


	virtual void initPhysics();

	void	initPhysics(const char* filename);

	void	exitPhysics();

	virtual	~ConvexDecompositionDemo()
	{
		exitPhysics();
	}

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	static DemoApplication* Create()
	{
		ConvexDecompositionDemo* demo = new ConvexDecompositionDemo();
		demo->myinit();
		demo->initPhysics("file.obj");
		return demo;
	}	
	
	
};

#endif //CONVEX_DECOMPOSITION_DEMO_H


