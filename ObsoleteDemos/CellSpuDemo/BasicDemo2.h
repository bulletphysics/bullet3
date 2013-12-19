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

#include "LinearMath/btQuickprof.h"

class btDiscreteDynamicsworld;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;

///BasicDemo is good starting point for learning the code base and porting.
class BasicDemo 
{
	btClock m_clock;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_overlappingPairCache;

	btCollisionDispatcher*	m_dispatcher;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btConstraintSolver*	m_solver;
	btDiscreteDynamicsWorld*	m_dynamicsWorld;

	btCollisionAlgorithmCreateFunc*	m_sphereSphereCF;
	btCollisionAlgorithmCreateFunc*	m_sphereBoxCF;
	btCollisionAlgorithmCreateFunc*	m_boxSphereCF;

	btRigidBody* localCreateRigidBody(btScalar mass,const btTransform& startTrans,btCollisionShape* colShape);


	public:

	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	
	
};

#endif //BASIC_DEMO_H

