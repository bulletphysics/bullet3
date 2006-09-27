
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
:btDynamicsWorld(new	CollisionDispatcher(),new SimpleBroadphase()),
m_constraintSolver(new SequentialImpulseConstraintSolver)
{
	m_islandManager = new SimulationIslandManager();
}

btDiscreteDynamicsWorld::btDiscreteDynamicsWorld(Dispatcher* dispatcher,OverlappingPairCache* pairCache,ConstraintSolver* constraintSolver)
:btDynamicsWorld(dispatcher,pairCache),
m_constraintSolver(constraintSolver)
{
	m_islandManager = new SimulationIslandManager();
}


btDiscreteDynamicsWorld::~btDiscreteDynamicsWorld()
{
	delete m_islandManager ;

	delete m_constraintSolver;

	//delete the dispatcher and paircache
	delete m_dispatcher1;
	m_dispatcher1 = 0;
	delete m_pairCache;
	m_pairCache = 0;
}

void	btDiscreteDynamicsWorld::stepSimulation(float timeStep)
{
	///update aabbs information
	updateAabbs();

	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	///perform collision detection
	PerformDiscreteCollisionDetection();

	calculateSimulationIslands();

	ContactSolverInfo infoGlobal;
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
		RaycastVehicle* vehicle = m_vehicles[i];
		vehicle->UpdateVehicle( timeStep);
	}
}

void	btDiscreteDynamicsWorld::updateActivationState(float timeStep)
{
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		CollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			RigidBody* body = (RigidBody*)colObj->m_internalOwner;
	
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

void	btDiscreteDynamicsWorld::addConstraint(TypedConstraint* constraint)
{
	m_constraints.push_back(constraint);
}

void	btDiscreteDynamicsWorld::removeConstraint(TypedConstraint* constraint)
{
	std::vector<TypedConstraint*>::iterator cit = std::find(m_constraints.begin(),m_constraints.end(),constraint);
	if (!(cit==m_constraints.end()))
	{
		m_constraints.erase(cit);
	}
}

void	btDiscreteDynamicsWorld::addVehicle(RaycastVehicle* vehicle)
{
	m_vehicles.push_back(vehicle);
}

void	btDiscreteDynamicsWorld::removeVehicle(RaycastVehicle* vehicle)
{
	std::vector<RaycastVehicle*>::iterator vit = std::find(m_vehicles.begin(),m_vehicles.end(),vehicle);
	if (!(vit==m_vehicles.end()))
	{
		m_vehicles.erase(vit);
	}
}


void	btDiscreteDynamicsWorld::solveContactConstraints(ContactSolverInfo& solverInfo)
{
	
	struct InplaceSolverIslandCallback : public SimulationIslandManager::IslandCallback
	{

		ContactSolverInfo& m_solverInfo;
		ConstraintSolver*	m_solver;
		IDebugDraw*	m_debugDrawer;

		InplaceSolverIslandCallback(
			ContactSolverInfo& solverInfo,
			ConstraintSolver*	solver,
			IDebugDraw*	debugDrawer)
			:m_solverInfo(solverInfo),
			m_solver(solver),
			m_debugDrawer(debugDrawer)
		{

		}

		virtual	void	ProcessIsland(PersistentManifold**	manifolds,int numManifolds)
		{
			m_solver->SolveGroup( manifolds, numManifolds,m_solverInfo,m_debugDrawer);
		}

	};

	IDebugDraw* debugDraw = 0;
	InplaceSolverIslandCallback	solverCallback(	solverInfo,	m_constraintSolver,	debugDraw);

	
	/// solve all the contact points and contact friction
	m_islandManager->BuildAndProcessIslands(GetCollisionWorld()->GetDispatcher(),GetCollisionWorld()->GetCollisionObjectArray(),&solverCallback);


}


void	btDiscreteDynamicsWorld::solveNoncontactConstraints(ContactSolverInfo& solverInfo)
{
	#ifdef USE_QUICKPROF
	Profiler::beginBlock("SolveConstraint");
#endif //USE_QUICKPROF



	int i;
	int numConstraints = m_constraints.size();

	///constraint preparation: building jacobians
	for (i=0;i< numConstraints ; i++ )
	{
		TypedConstraint* constraint = m_constraints[i];
		constraint->BuildJacobian();
	}

	//solve the regular non-contact constraints (point 2 point, hinge, generic d6)
	for (int g=0;g<solverInfo.m_numIterations;g++)
	{
		//
		// constraint solving
		//
		for (i=0;i< numConstraints ; i++ )
		{
			TypedConstraint* constraint = m_constraints[i];
			constraint->SolveConstraint( solverInfo.m_timeStep );
		}
	}

#ifdef USE_QUICKPROF
	Profiler::endBlock("SolveConstraint");
#endif //USE_QUICKPROF

}

void	btDiscreteDynamicsWorld::calculateSimulationIslands()
{
	
#ifdef USE_QUICKPROF
	Profiler::beginBlock("IslandUnionFind");
#endif //USE_QUICKPROF

	GetSimulationIslandManager()->UpdateActivationState(GetCollisionWorld(),GetCollisionWorld()->GetDispatcher());

	{
		int i;
		int numConstraints = m_constraints.size();
		for (i=0;i< numConstraints ; i++ )
		{
			TypedConstraint* constraint = m_constraints[i];

			const RigidBody* colObj0 = &constraint->GetRigidBodyA();
			const RigidBody* colObj1 = &constraint->GetRigidBodyB();

			if (((colObj0) && ((colObj0)->mergesSimulationIslands())) &&
				((colObj1) && ((colObj1)->mergesSimulationIslands())))
			{
				if (colObj0->IsActive() || colObj1->IsActive())
				{

					GetSimulationIslandManager()->GetUnionFind().unite((colObj0)->m_islandTag1,
						(colObj1)->m_islandTag1);
				}
			}
		}
	}

	//Store the island id in each body
	GetSimulationIslandManager()->StoreIslandActivationState(GetCollisionWorld());

#ifdef USE_QUICKPROF
	Profiler::endBlock("IslandUnionFind");
#endif //USE_QUICKPROF

}

void	btDiscreteDynamicsWorld::updateAabbs()
{
	SimdTransform predictedTrans;
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		CollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			RigidBody* body = (RigidBody*)colObj->m_internalOwner;
			if (body->IsActive() && (!body->IsStatic()))
			{
				SimdPoint3 minAabb,maxAabb;
				colObj->m_collisionShape->GetAabb(colObj->m_worldTransform, minAabb,maxAabb);
				SimpleBroadphase* bp = (SimpleBroadphase*)m_pairCache;
				bp->SetAabb(body->m_broadphaseHandle,minAabb,maxAabb);
			}
		}
	}
}

void	btDiscreteDynamicsWorld::integrateTransforms(float timeStep)
{
	SimdTransform predictedTrans;
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		CollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			RigidBody* body = (RigidBody*)colObj->m_internalOwner;
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
		CollisionObject* colObj = m_collisionObjects[i];
		if (colObj->m_internalOwner)
		{
			RigidBody* body = (RigidBody*)colObj->m_internalOwner;
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