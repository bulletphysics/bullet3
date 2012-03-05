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

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btConvexHullComputer.h"
#include "LinearMath/btQuaternion.h"
#include <set>
#include <time.h>

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///VoronoiFractureDemo is good starting point for learning the code base and porting.

class VoronoiFractureDemo : public PlatformDemoApplication
{
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	btClock m_perfmTimer;

	public:

	VoronoiFractureDemo()
	{
		srand((unsigned)time(NULL)); // Seed it...
	}
	virtual ~VoronoiFractureDemo()
	{
		exitPhysics();
	}
	void	initPhysics();

	void	exitPhysics();

	void getVerticesInsidePlanes(const btAlignedObjectArray<btVector3>& planes, btAlignedObjectArray<btVector3>& verticesOut, std::set<int>& planeIndicesOut);
	void voronoiBBShatter(const btAlignedObjectArray<btVector3>& points, const btVector3& bbmin, const btVector3& bbmax, const btQuaternion& bbq, const btVector3& bbt, btScalar matDensity);
	void voronoiConvexHullShatter(const btAlignedObjectArray<btVector3>& points, const btAlignedObjectArray<btVector3>& verts, const btQuaternion& bbq, const btVector3& bbt, btScalar matDensity);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
	virtual void	clientResetScene();
	
	virtual void keyboardCallback(unsigned char key, int x, int y);

	void attachFixedConstraints();


	static DemoApplication* Create()
	{
		VoronoiFractureDemo* demo = new VoronoiFractureDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	
};

#endif //BASIC_DEMO_H

