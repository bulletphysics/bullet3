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
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btMotionState.h"



#include <algorithm>

btDiscreteDynamicsWorld::btDiscreteDynamicsWorld()
:btDynamicsWorld(),
m_constraintSolver(new btSequentialImpulseConstraintSolver),
m_debugDrawer(0),
m_gravity(0,-10,0),
m_profileTimings(0)
{
	m_islandManager = new btSimulationIslandManager();
	m_ownsIslandManager = true;
	m_ownsConstraintSolver = true;

}

btDiscreteDynamicsWorld::btDiscreteDynamicsWorld(btDispatcher* dispatcher,btOverlappingPairCache* pairCache,btConstraintSolver* constraintSolver)
:btDynamicsWorld(dispatcher,pairCache),
m_constraintSolver(constraintSolver? constraintSolver: new btSequentialImpulseConstraintSolver),
m_debugDrawer(0),
m_gravity(0,-10,0),
m_profileTimings(0)
{
	m_islandManager = new btSimulationIslandManager();
	m_ownsIslandManager = true;
	m_ownsConstraintSolver = (constraintSolver==0);
}


btDiscreteDynamicsWorld::~btDiscreteDynamicsWorld()
{
	//only delete it when we created it
	if (m_ownsIslandManager)
		delete m_islandManager;
	if (m_ownsConstraintSolver)
		 delete m_constraintSolver;
}

void	btDiscreteDynamicsWorld::saveKinematicState(float timeStep)
{

	for (unsigned int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
				btTransform predictedTrans;
				if (body->GetActivationState() != ISLAND_SLEEPING)
				{
					if (body->isKinematicObject())
					{
						//to calculate velocities next frame
						body->saveKinematicState(timeStep);
					}
				}
		}
	}
}

void	btDiscreteDynamicsWorld::synchronizeMotionStates()
{
	//todo: iterate over awake simulation islands!
	for (unsigned int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body && body->getMotionState())
		{
			if (body->GetActivationState() != ISLAND_SLEEPING)
			{
				body->getMotionState()->setWorldOrientation(body->getCenterOfMassTransform().getRotation());
				body->getMotionState()->setWorldPosition(body->getCenterOfMassTransform().getOrigin());
			}
		}
	}

}


void	btDiscreteDynamicsWorld::stepSimulation(float timeStep, int numSubsteps)
{

	if (!btFuzzyZero(timeStep) && numSubsteps)
	{

		saveKinematicState(timeStep);

		int i;
		float subTimeStep = timeStep / float(numSubsteps);

		for (i=0;i<numSubsteps;i++)
		{
			internalSingleStepSimulation(subTimeStep);
		}

		synchronizeMotionStates();
	} 
}

void	btDiscreteDynamicsWorld::internalSingleStepSimulation(float timeStep)
{
	
	startProfiling(timeStep);

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

void	btDiscreteDynamicsWorld::setGravity(const btVector3& gravity)
{
	m_gravity = gravity;
	for (unsigned int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			body->setGravity(gravity);
		}
	}
}


void	btDiscreteDynamicsWorld::removeRigidBody(btRigidBody* body)
{
	removeCollisionObject(body);
}

void	btDiscreteDynamicsWorld::addRigidBody(btRigidBody* body)
{
	body->setGravity(m_gravity);
	bool isDynamic = !(body->isStaticObject() || body->isKinematicObject());
	short collisionFilterGroup = isDynamic? 
	btBroadphaseProxy::DefaultFilter : 
	btBroadphaseProxy::StaticFilter;
	short collisionFilterMask = isDynamic? 
	btBroadphaseProxy::AllFilter : 
	btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter;

	addCollisionObject(body,collisionFilterGroup,collisionFilterMask);
}


void	btDiscreteDynamicsWorld::updateVehicles(float timeStep)
{
	BEGIN_PROFILE("updateVehicles");

	for (unsigned int i=0;i<m_vehicles.size();i++)
	{
		btRaycastVehicle* vehicle = m_vehicles[i];
		vehicle->updateVehicle( timeStep);
	}
	END_PROFILE("updateVehicles");
}

void	btDiscreteDynamicsWorld::updateActivationState(float timeStep)
{
	BEGIN_PROFILE("updateActivationState");

	for (unsigned int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
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
	END_PROFILE("updateActivationState");
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
	
	BEGIN_PROFILE("solveContactConstraints");

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

	
	InplaceSolverIslandCallback	solverCallback(	solverInfo,	m_constraintSolver,	m_debugDrawer);

	
	/// solve all the contact points and contact friction
	m_islandManager->buildAndProcessIslands(getCollisionWorld()->getDispatcher(),getCollisionWorld()->getCollisionObjectArray(),&solverCallback);

	END_PROFILE("solveContactConstraints");

}


void	btDiscreteDynamicsWorld::solveNoncontactConstraints(btContactSolverInfo& solverInfo)
{
	BEGIN_PROFILE("solveNoncontactConstraints");

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

	END_PROFILE("solveNoncontactConstraints");

}

void	btDiscreteDynamicsWorld::calculateSimulationIslands()
{
	BEGIN_PROFILE("calculateSimulationIslands");

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

	END_PROFILE("calculateSimulationIslands");

}

static void DrawAabb(btIDebugDraw* debugDrawer,const btVector3& from,const btVector3& to,const btVector3& color)
{

	btVector3 halfExtents = (to-from)* 0.5f;
	btVector3 center = (to+from) *0.5f;
	int i,j;

	btVector3 edgecoord(1.f,1.f,1.f),pa,pb;
	for (i=0;i<4;i++)
	{
		for (j=0;j<3;j++)
		{
			pa = btVector3(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],		
				edgecoord[2]*halfExtents[2]);
			pa+=center;

			int othercoord = j%3;
			edgecoord[othercoord]*=-1.f;
			pb = btVector3(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],	
				edgecoord[2]*halfExtents[2]);
			pb+=center;

			debugDrawer->drawLine(pa,pb,color);
		}
		edgecoord = btVector3(-1.f,-1.f,-1.f);
		if (i<3)
			edgecoord[i]*=-1.f;
	}


}

void	btDiscreteDynamicsWorld::updateAabbs()
{
	BEGIN_PROFILE("updateAabbs");
	
	btVector3 colorvec(1,0,0);
	btTransform predictedTrans;
	for (unsigned int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
		//	if (body->IsActive() && (!body->IsStatic()))
			{
				btPoint3 minAabb,maxAabb;
				colObj->m_collisionShape->getAabb(colObj->m_worldTransform, minAabb,maxAabb);
				btSimpleBroadphase* bp = (btSimpleBroadphase*)m_broadphasePairCache;
				bp->setAabb(body->m_broadphaseHandle,minAabb,maxAabb);
				if (m_debugDrawer && (m_debugDrawer->getDebugMode() & btIDebugDraw::DBG_DrawAabb))
				{
					DrawAabb(m_debugDrawer,minAabb,maxAabb,colorvec);
				}
			}
		}
	}
	
	END_PROFILE("updateAabbs");
}

void	btDiscreteDynamicsWorld::integrateTransforms(float timeStep)
{
	BEGIN_PROFILE("integrateTransforms");
	btTransform predictedTrans;
	for (unsigned int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			if (body->IsActive() && (!body->isStaticObject()))
			{
				body->predictIntegratedTransform(timeStep, predictedTrans);
				body->proceedToTransform( predictedTrans);
			}
		}
	}
	END_PROFILE("integrateTransforms");
}



void	btDiscreteDynamicsWorld::predictUnconstraintMotion(float timeStep)
{
	BEGIN_PROFILE("predictUnconstraintMotion");
	for (unsigned int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			if (!body->isStaticObject())
			{
				if (body->IsActive())
				{
					body->applyForces( timeStep);
					body->integrateVelocities( timeStep);
					body->predictIntegratedTransform(timeStep,body->m_interpolationWorldTransform);
				}
			}
		}
	}
	END_PROFILE("predictUnconstraintMotion");
}


void	btDiscreteDynamicsWorld::startProfiling(float timeStep)
{
	#ifdef USE_QUICKPROF


	//toggle btProfiler
	if ( m_debugDrawer && m_debugDrawer->getDebugMode() & btIDebugDraw::DBG_ProfileTimings)
	{
		if (!m_profileTimings)
		{
			m_profileTimings = 1;
			// To disable profiling, simply comment out the following line.
			static int counter = 0;

			char filename[128];
			sprintf(filename,"quickprof_bullet_timings%i.csv",counter++);
			btProfiler::init(filename, btProfiler::BLOCK_CYCLE_SECONDS);//BLOCK_TOTAL_MICROSECONDS
		} else
		{
			btProfiler::endProfilingCycle();
		}

	} else
	{
		if (m_profileTimings)
		{
			btProfiler::endProfilingCycle();

			m_profileTimings = 0;
			btProfiler::destroy();
		}
	}
#endif //USE_QUICKPROF
}


