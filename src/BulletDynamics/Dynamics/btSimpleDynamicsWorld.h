/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SIMPLE_DYNAMICS_WORLD_H
#define BT_SIMPLE_DYNAMICS_WORLD_H

#include "btDynamicsWorld.h"

class btDispatcher;
class btOverlappingPairCache;
class btConstraintSolver;

///The btSimpleDynamicsWorld serves as unit-test and to verify more complicated and optimized dynamics worlds.
///Please use btDiscreteDynamicsWorld instead
class btSimpleDynamicsWorld : public btDynamicsWorld
{
protected:
	btConstraintSolver* m_constraintSolver;

	bool m_ownsConstraintSolver;

	void predictUnconstraintMotion(btScalar timeStep);

	void integrateTransforms(btScalar timeStep);

	btVector3 m_gravity;

public:
	///this btSimpleDynamicsWorld constructor creates dispatcher, broadphase pairCache and constraintSolver
	btSimpleDynamicsWorld(btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration);

	virtual ~btSimpleDynamicsWorld() override;

	///maxSubSteps/fixedTimeStep for interpolation is currently ignored for btSimpleDynamicsWorld, use btDiscreteDynamicsWorld instead
	virtual int stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.)) override;

	virtual void setGravity(const btVector3& gravity) override;

	virtual btVector3 getGravity() const override;

	virtual void addRigidBody(btRigidBody* body) override;

	virtual void addRigidBody(btRigidBody* body, int group, int mask) override;

	virtual void removeRigidBody(btRigidBody* body) override;

	virtual void debugDrawWorld() override;

	virtual void addAction(btActionInterface* action) override;

	virtual void removeAction(btActionInterface* action) override;

	///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
	virtual void removeCollisionObject(btCollisionObject* collisionObject) override;

	virtual void updateAabbs() override;

	virtual void synchronizeMotionStates() override;

	virtual void setConstraintSolver(btConstraintSolver* solver) override;

	virtual btConstraintSolver* getConstraintSolver() override;

	virtual btDynamicsWorldType getWorldType() const override
	{
		return BT_SIMPLE_DYNAMICS_WORLD;
	}

	virtual void clearForces() override;
};

#endif  //BT_SIMPLE_DYNAMICS_WORLD_H
