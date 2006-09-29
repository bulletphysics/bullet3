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


#include "btDiscreteDynamicsWorld.h"

//collision detection
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

//rigidbody & constraints
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"

//vehicle
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include "BulletDynamics/Vehicle/btVehicleRaycaster.h"
#include "BulletDynamics/Vehicle/btWheelInfo.h"

#include <algorithm>

btDiscreteDynamicsWorld::btDiscreteDynamicsWorld()
:btDynamicsWorld(),
m_constraintSolver(new btSequentialImpulseConstraintSolver)
{
	m_islandManager = new btSimulationIslandManager();
	m_ownsIslandManager = true;
	m_ownsConstraintSolver = true;

}

btDiscreteDynamicsWorld::btDiscreteDynamicsWorld(btDispatcher* dispatcher,btOverlappingPairCache* pairCache,btConstraintSolver* constraintSolver)
:btDynamicsWorld(dispatcher,pairCache),
m_constraintSolver(constraintSolver)
{
	m_islandManager = new btSimulationIslandManager();
	m_ownsIslandManager = true;
	m_ownsConstraintSolver = false;
}


btDiscreteDynamicsWorld::~btDiscreteDynamicsWorld()
{
	//only delete it when we created it
	if (m_ownsIslandManager)
		delete m_islandManager;
	if (m_ownsConstraintSolver)
		 delete m_constraintSolver;
}

void	btDiscreteDynamicsWorld::stepSimulation(float timeStep)
{
	///update aabbs information
	updateAabbs();

	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	///perform collision detection
	performDiscreteCollisionDetection();

	calculateSimulationIslands();

	btContactSolverInfo infoGlobal;
	infoGlobal.m_timeStep = timeStep;
	
	///solve non-contact constraints
	solveNoncontactConstraints(infoGlobal);
	
	///solve contact constraints
	solveContactConstraints(infoGlobal);

	///update vehicle simulation
	updateVehicles(timeStep);
	
	///CallbackTriggers();

	///integrate transforms
	integrateTransforms(timeStep);
		
	updateActivationState( timeStep );

	

}

void	btDiscreteDynamicsWorld::updateVehicles(float timeStep)
{
	for (int i=0;i<m_vehicles.size();i++)
	{
		btRaycastVehicle* vehicle = m_vehicles[i];
		vehicle->updateVehicle( timeStep);
	}
}

void	btDiscreteDynamicsWorld::updateActivationState(float timeStep)
{
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			btRigidBody* body = (btRigidBody*)colObj->m_internalOwner;
	
			body->updateDeactivation(timeStep);

			if (body->wantsSleeping())
			{
				if (body->GetActivationState() == ACTIVE_TAG)
					body->SetActivationState( WANTS_DEACTIVATION );
			} else
			{
				if (body->GetActivationState() != DISABLE_DEACTIVATION)
					body->SetActivationState( ACTIVE_TAG );
			}
		}
	}
}

void	btDiscreteDynamicsWorld::addConstraint(btTypedConstraint* constraint)
{
	m_constraints.push_back(constraint);
}

void	btDiscreteDynamicsWorld::removeConstraint(btTypedConstraint* constraint)
{
	std::vector<btTypedConstraint*>::iterator cit = std::find(m_constraints.begin(),m_constraints.end(),constraint);
	if (!(cit==m_constraints.end()))
	{
		m_constraints.erase(cit);
	}
}

void	btDiscreteDynamicsWorld::addVehicle(btRaycastVehicle* vehicle)
{
	m_vehicles.push_back(vehicle);
}

void	btDiscreteDynamicsWorld::removeVehicle(btRaycastVehicle* vehicle)
{
	std::vector<btRaycastVehicle*>::iterator vit = std::find(m_vehicles.begin(),m_vehicles.end(),vehicle);
	if (!(vit==m_vehicles.end()))
	{
		m_vehicles.erase(vit);
	}
}


void	btDiscreteDynamicsWorld::solveContactConstraints(btContactSolverInfo& solverInfo)
{
	
	struct InplaceSolverIslandCallback : public btSimulationIslandManager::IslandCallback
	{

		btContactSolverInfo& m_solverInfo;
		btConstraintSolver*	m_solver;
		btIDebugDraw*	m_debugDrawer;

		InplaceSolverIslandCallback(
			btContactSolverInfo& solverInfo,
			btConstraintSolver*	solver,
			btIDebugDraw*	debugDrawer)
			:m_solverInfo(solverInfo),
			m_solver(solver),
			m_debugDrawer(debugDrawer)
		{

		}

		virtual	void	ProcessIsland(btPersistentManifold**	manifolds,int numManifolds)
		{
			m_solver->solveGroup( manifolds, numManifolds,m_solverInfo,m_debugDrawer);
		}

	};

	btIDebugDraw* debugDraw = 0;
	InplaceSolverIslandCallback	solverCallback(	solverInfo,	m_constraintSolver,	debugDraw);

	
	/// solve all the contact points and contact friction
	m_islandManager->buildAndProcessIslands(getCollisionWorld()->getDispatcher(),getCollisionWorld()->getCollisionObjectArray(),&solverCallback);


}


void	btDiscreteDynamicsWorld::solveNoncontactConstraints(btContactSolverInfo& solverInfo)
{
	#ifdef USE_QUICKPROF
	Profiler::beginBlock("solveConstraint");
#endif //USE_QUICKPROF



	int i;
	int numConstraints = m_constraints.size();

	///constraint preparation: building jacobians
	for (i=0;i< numConstraints ; i++ )
	{
		btTypedConstraint* constraint = m_constraints[i];
		constraint->buildJacobian();
	}

	//solve the regular non-contact constraints (point 2 point, hinge, generic d6)
	for (int g=0;g<solverInfo.m_numIterations;g++)
	{
		//
		// constraint solving
		//
		for (i=0;i< numConstraints ; i++ )
		{
			btTypedConstraint* constraint = m_constraints[i];
			constraint->solveConstraint( solverInfo.m_timeStep );
		}
	}

#ifdef USE_QUICKPROF
	Profiler::endBlock("solveConstraint");
#endif //USE_QUICKPROF

}

void	btDiscreteDynamicsWorld::calculateSimulationIslands()
{
	
#ifdef USE_QUICKPROF
	Profiler::beginBlock("IslandUnionFind");
#endif //USE_QUICKPROF

	getSimulationIslandManager()->updateActivationState(getCollisionWorld(),getCollisionWorld()->getDispatcher());

	{
		int i;
		int numConstraints = m_constraints.size();
		for (i=0;i< numConstraints ; i++ )
		{
			btTypedConstraint* constraint = m_constraints[i];

			const btRigidBody* colObj0 = &constraint->getRigidBodyA();
			const btRigidBody* colObj1 = &constraint->getRigidBodyB();

			if (((colObj0) && ((colObj0)->mergesSimulationIslands())) &&
				((colObj1) && ((colObj1)->mergesSimulationIslands())))
			{
				if (colObj0->IsActive() || colObj1->IsActive())
				{

					getSimulationIslandManager()->getUnionFind().unite((colObj0)->m_islandTag1,
						(colObj1)->m_islandTag1);
				}
			}
		}
	}

	//Store the island id in each body
	getSimulationIslandManager()->storeIslandActivationState(getCollisionWorld());

#ifdef USE_QUICKPROF
	Profiler::endBlock("IslandUnionFind");
#endif //USE_QUICKPROF

}

void	btDiscreteDynamicsWorld::updateAabbs()
{
	btTransform predictedTrans;
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			btRigidBody* body = (btRigidBody*)colObj->m_internalOwner;
			if (body->IsActive() && (!body->IsStatic()))
			{
				btPoint3 minAabb,maxAabb;
				colObj->m_collisionShape->getAabb(colObj->m_worldTransform, minAabb,maxAabb);
				btSimpleBroadphase* bp = (btSimpleBroadphase*)m_broadphasePairCache;
				bp->setAabb(body->m_broadphaseHandle,minAabb,maxAabb);
			}
		}
	}
}

void	btDiscreteDynamicsWorld::integrateTransforms(float timeStep)
{
	btTransform predictedTrans;
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			btRigidBody* body = (btRigidBody*)colObj->m_internalOwner;
			if (body->IsActive() && (!body->IsStatic()))
			{
				body->predictIntegratedTransform(timeStep, predictedTrans);
				body->proceedToTransform( predictedTrans);
			}
		}
	}
}



void	btDiscreteDynamicsWorld::predictUnconstraintMotion(float timeStep)
{
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			btRigidBody* body = (btRigidBody*)colObj->m_internalOwner;
			body->m_cachedInvertedWorldTransform = body->m_worldTransform.inverse();
			if (body->IsActive() && (!body->IsStatic()))
			{
				body->applyForces( timeStep);
				body->integrateVelocities( timeStep);
				body->predictIntegratedTransform(timeStep,body->m_interpolationWorldTransform);

			}
		}
	}
}
