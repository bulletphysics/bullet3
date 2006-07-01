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



#include "ParallelPhysicsEnvironment.h"
#include "CcdPhysicsController.h"
#include "ParallelIslandDispatcher.h"
#include "CollisionDispatch/CollisionWorld.h"
#include "ConstraintSolver/TypedConstraint.h"



ParallelPhysicsEnvironment::ParallelPhysicsEnvironment(ParallelIslandDispatcher* dispatcher, OverlappingPairCache* pairCache):
CcdPhysicsEnvironment(0,pairCache)
{

}

ParallelPhysicsEnvironment::~ParallelPhysicsEnvironment()
{

}



/// Perform an integration step of duration 'timeStep'.
bool	ParallelPhysicsEnvironment::proceedDeltaTimeOneStep(float timeStep)
{

#ifdef USE_QUICKPROF
	Profiler::beginBlock("CalcSimulationIslands");
#endif //USE_QUICKPROF

	/*
	GetCollisionWorld()->UpdateActivationState();

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
					GetDispatcher()->GetUnionFind().unite((colObj0)->m_islandTag1,
						(colObj1)->m_islandTag1);
				}
			}
		}
	}

	GetCollisionWorld()->StoreIslandActivationState();
*/

#ifdef USE_QUICKPROF
	Profiler::endBlock("CalcSimulationIslands");
#endif //USE_QUICKPROF



/*
	//printf("CcdPhysicsEnvironment::proceedDeltaTime\n");

	if (SimdFuzzyZero(timeStep))
		return true;

	if (m_debugDrawer)
	{
		gDisableDeactivation = (m_debugDrawer->GetDebugMode() & IDebugDraw::DBG_NoDeactivation);
	}


#ifdef USE_QUICKPROF
	Profiler::beginBlock("SyncMotionStates");
#endif //USE_QUICKPROF


	//this is needed because scaling is not known in advance, and scaling has to propagate to the shape
	if (!m_scalingPropagated)
	{
		SyncMotionStates(timeStep);
		m_scalingPropagated = true;
	}


#ifdef USE_QUICKPROF
	Profiler::endBlock("SyncMotionStates");

	Profiler::beginBlock("predictIntegratedTransform");
#endif //USE_QUICKPROF

	{
		//		std::vector<CcdPhysicsController*>::iterator i;



		int k;
		for (k=0;k<GetNumControllers();k++)
		{
			CcdPhysicsController* ctrl = m_controllers[k];
			//		SimdTransform predictedTrans;
			RigidBody* body = ctrl->GetRigidBody();
			if (body->IsActive())
			{
				if (!body->IsStatic())
				{
					body->applyForces( timeStep);
					body->integrateVelocities( timeStep);
					body->predictIntegratedTransform(timeStep,body->m_interpolationWorldTransform);
				}
			}

		}
	}

#ifdef USE_QUICKPROF
	Profiler::endBlock("predictIntegratedTransform");
#endif //USE_QUICKPROF

	BroadphaseInterface*	scene = GetBroadphase();


	//
	// collision detection (?)
	//


#ifdef USE_QUICKPROF
	Profiler::beginBlock("DispatchAllCollisionPairs");
#endif //USE_QUICKPROF


	int numsubstep = m_numIterations;


	DispatcherInfo dispatchInfo;
	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_enableSatConvex = m_enableSatCollisionDetection;

	scene->DispatchAllCollisionPairs(*GetDispatcher(),dispatchInfo);///numsubstep,g);


#ifdef USE_QUICKPROF
	Profiler::endBlock("DispatchAllCollisionPairs");
#endif //USE_QUICKPROF


	int numRigidBodies = m_controllers.size();

	


	//contacts
#ifdef USE_QUICKPROF
	Profiler::beginBlock("SolveConstraint");
#endif //USE_QUICKPROF


	//solve the regular constraints (point 2 point, hinge, etc)

	for (int g=0;g<numsubstep;g++)
	{
		//
		// constraint solving
		//


		int i;
		int numConstraints = m_constraints.size();

		//point to point constraints
		for (i=0;i< numConstraints ; i++ )
		{
			TypedConstraint* constraint = m_constraints[i];

			constraint->BuildJacobian();
			constraint->SolveConstraint( timeStep );

		}


	}

#ifdef USE_QUICKPROF
	Profiler::endBlock("SolveConstraint");
#endif //USE_QUICKPROF

	//solve the vehicles

#ifdef NEW_BULLET_VEHICLE_SUPPORT
	//vehicles
	int numVehicles = m_wrapperVehicles.size();
	for (int i=0;i<numVehicles;i++)
	{
		WrapperVehicle* wrapperVehicle = m_wrapperVehicles[i];
		RaycastVehicle* vehicle = wrapperVehicle->GetVehicle();
		vehicle->UpdateVehicle( timeStep);
	}
#endif //NEW_BULLET_VEHICLE_SUPPORT


	struct InplaceSolverIslandCallback : public ParallelIslandDispatcher::IslandCallback
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


	m_solverInfo.m_friction = 0.9f;
	m_solverInfo.m_numIterations = m_numIterations;
	m_solverInfo.m_timeStep = timeStep;
	m_solverInfo.m_restitution = 0.f;//m_restitution;

	InplaceSolverIslandCallback	solverCallback(
		m_solverInfo,
		m_solver,
		m_debugDrawer);

#ifdef USE_QUICKPROF
	Profiler::beginBlock("BuildAndProcessIslands");
#endif //USE_QUICKPROF

	/// solve all the contact points and contact friction
	GetDispatcher()->BuildAndProcessIslands(m_collisionWorld->GetCollisionObjectArray(),&solverCallback);

#ifdef USE_QUICKPROF
	Profiler::endBlock("BuildAndProcessIslands");

	Profiler::beginBlock("CallbackTriggers");
#endif //USE_QUICKPROF

	CallbackTriggers();

#ifdef USE_QUICKPROF
	Profiler::endBlock("CallbackTriggers");


	Profiler::beginBlock("proceedToTransform");

#endif //USE_QUICKPROF
	{



		{

			
			
			UpdateAabbs(timeStep);


			float toi = 1.f;



			if (m_ccdMode == 3)
			{
				DispatcherInfo dispatchInfo;
				dispatchInfo.m_timeStep = timeStep;
				dispatchInfo.m_stepCount = 0;
				dispatchInfo.m_dispatchFunc = DispatcherInfo::DISPATCH_CONTINUOUS;

				scene->DispatchAllCollisionPairs( *GetDispatcher(),dispatchInfo);///numsubstep,g);
				toi = dispatchInfo.m_timeOfImpact;

			}

			

			//
			// integrating solution
			//

			{
				
				std::vector<CcdPhysicsController*>::iterator i;

				for (i=m_controllers.begin();
					!(i==m_controllers.end()); i++)
				{

					CcdPhysicsController* ctrl = *i;

					SimdTransform predictedTrans;
					RigidBody* body = ctrl->GetRigidBody();
					
					if (body->IsActive())
					{

						if (!body->IsStatic())
						{
							body->predictIntegratedTransform(timeStep*	toi, predictedTrans);
							body->proceedToTransform( predictedTrans);
						}

					}
				}

			}





			//
			// disable sleeping physics objects
			//

			std::vector<CcdPhysicsController*> m_sleepingControllers;

			std::vector<CcdPhysicsController*>::iterator i;

			for (i=m_controllers.begin();
				!(i==m_controllers.end()); i++)
			{
				CcdPhysicsController* ctrl = (*i);
				RigidBody* body = ctrl->GetRigidBody();

				ctrl->UpdateDeactivation(timeStep);


				if (ctrl->wantsSleeping())
				{
					if (body->GetActivationState() == ACTIVE_TAG)
						body->SetActivationState( WANTS_DEACTIVATION );
				} else
				{
					if (body->GetActivationState() != DISABLE_DEACTIVATION)
						body->SetActivationState( ACTIVE_TAG );
				}

				if (useIslands)
				{
					if (body->GetActivationState() == ISLAND_SLEEPING)
					{
						m_sleepingControllers.push_back(ctrl);
					}
				} else
				{
					if (ctrl->wantsSleeping())
					{
						m_sleepingControllers.push_back(ctrl);
					}
				}
			}




		}


#ifdef USE_QUICKPROF
		Profiler::endBlock("proceedToTransform");

		Profiler::beginBlock("SyncMotionStates");
#endif //USE_QUICKPROF

		SyncMotionStates(timeStep);

#ifdef USE_QUICKPROF
		Profiler::endBlock("SyncMotionStates");

		Profiler::endProfilingCycle();
#endif //USE_QUICKPROF


#ifdef NEW_BULLET_VEHICLE_SUPPORT
		//sync wheels for vehicles
		int numVehicles = m_wrapperVehicles.size();
		for (int i=0;i<numVehicles;i++)
		{
			WrapperVehicle* wrapperVehicle = m_wrapperVehicles[i];

			wrapperVehicle->SyncWheels();
		}
#endif //NEW_BULLET_VEHICLE_SUPPORT
	}
*/

	return true;
}
