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

#include "btSimpleDynamicsWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"


btSimpleDynamicsWorld::btSimpleDynamicsWorld()
:m_constraintSolver(new btSequentialImpulseConstraintSolver),
m_ownsConstraintSolver(true),
m_debugDrawer(0)
{
}

btSimpleDynamicsWorld::btSimpleDynamicsWorld(btDispatcher* dispatcher,btOverlappingPairCache* pairCache,btConstraintSolver* constraintSolver)
:btDynamicsWorld(dispatcher,pairCache),
m_constraintSolver(constraintSolver),
m_ownsConstraintSolver(false),
m_debugDrawer(0)
{

}


btSimpleDynamicsWorld::~btSimpleDynamicsWorld()
{
	if (m_ownsConstraintSolver)
		delete m_constraintSolver;
}

void	btSimpleDynamicsWorld::stepSimulation(float timeStep)
{
	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	///perform collision detection
	performDiscreteCollisionDetection();

	///solve contact constraints
	btPersistentManifold** manifoldPtr = ((btCollisionDispatcher*)m_dispatcher1)->getInternalManifoldPointer();
	int numManifolds = m_dispatcher1->getNumManifolds();
	btContactSolverInfo infoGlobal;
	infoGlobal.m_timeStep = timeStep;
	
	m_constraintSolver->solveGroup(manifoldPtr, numManifolds,infoGlobal,m_debugDrawer);

	///integrate transforms
	integrateTransforms(timeStep);
		
	updateAabbs();

}



void	btSimpleDynamicsWorld::updateAabbs()
{
	btTransform predictedTrans;
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			if (body->IsActive() && (!body->IsStatic()))
			{
				btPoint3 minAabb,maxAabb;
				colObj->m_collisionShape->getAabb(colObj->m_worldTransform, minAabb,maxAabb);
				btBroadphaseInterface* bp = getBroadphase();
				bp->setAabb(body->m_broadphaseHandle,minAabb,maxAabb);
			}
		}
	}
}

void	btSimpleDynamicsWorld::integrateTransforms(float timeStep)
{
	btTransform predictedTrans;
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			if (body->IsActive() && (!body->IsStatic()))
			{
				body->predictIntegratedTransform(timeStep, predictedTrans);
				body->proceedToTransform( predictedTrans);
			}
		}
	}
}



void	btSimpleDynamicsWorld::predictUnconstraintMotion(float timeStep)
{
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			if (body->IsActive() && (!body->IsStatic()))
			{
				body->applyForces( timeStep);
				body->integrateVelocities( timeStep);
				body->predictIntegratedTransform(timeStep,body->m_interpolationWorldTransform);

			}
		}
	}
}
