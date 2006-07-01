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

#include "SimulationIsland.h"
#include "SimdTransform.h"
#include "CcdPhysicsController.h"
#include "BroadphaseCollision/OverlappingPairCache.h"
#include "CollisionShapes/CollisionShape.h"
#include "BroadphaseCollision/Dispatcher.h"
#include "ConstraintSolver/ContactSolverInfo.h"
#include "ConstraintSolver/ConstraintSolver.h"

extern float gContactBreakingTreshold;

bool	SimulationIsland::Simulate(Dispatcher* dispatcher,BroadphaseInterface* broadphase,class ConstraintSolver*	solver,float timeStep)
{
	//then execute all stuff below for each simulation island


#ifdef USE_QUICKPROF
	Profiler::endBlock("BuildIslands");
#endif //USE_QUICKPROF




	///build simulation islands, and add them to a job queue, which can be processed in parallel
	///or on the GPU


	//printf("CcdPhysicsEnvironment::proceedDeltaTime\n");

	if (SimdFuzzyZero(timeStep))
		return true;

	//	if (m_debugDrawer)
	//	{
	//		gDisableDeactivation = (m_debugDrawer->GetDebugMode() & IDebugDraw::DBG_NoDeactivation);
	//	}


#ifdef USE_QUICKPROF
	Profiler::beginBlock("SyncMotionStates");
#endif //USE_QUICKPROF


	//this is needed because scaling is not known in advance, and scaling has to propagate to the shape
	//if (!m_scalingPropagated)
	//{
	//	SyncMotionStates(timeStep);
	//	m_scalingPropagated = true;
	//}


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
	
	//BroadphaseInterface*	scene = GetBroadphase();


	//
	// collision detection (?)
	//


	#ifdef USE_QUICKPROF
	Profiler::beginBlock("DispatchAllCollisionPairs");
	#endif //USE_QUICKPROF


//	int numsubstep = m_numIterations;


	DispatcherInfo dispatchInfo;
	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_enableSatConvex = false;//m_enableSatCollisionDetection;

	//pairCache->RefreshOverlappingPairs();
	if (m_overlappingPairs.size())
	{
		dispatcher->DispatchAllCollisionPairs(&m_overlappingPairs[0],m_overlappingPairs.size(),dispatchInfo);///numsubstep,g);
	}


	#ifdef USE_QUICKPROF
	Profiler::endBlock("DispatchAllCollisionPairs");
	#endif //USE_QUICKPROF

/*


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

	}
	*/

	//OverlappingPairCache*	scene = GetCollisionWorld()->GetPairCache();
	
	ContactSolverInfo	solverInfo;

	solverInfo.m_friction = 0.9f;
	solverInfo.m_numIterations = 10;//m_numIterations;
	solverInfo.m_timeStep = timeStep;
	solverInfo.m_restitution = 0.f;//m_restitution;


	if (m_manifolds.size())
	{
		solver->SolveGroup( &m_manifolds[0],m_manifolds.size(),solverInfo,0);
	}


#ifdef USE_QUICKPROF
	Profiler::beginBlock("proceedToTransform");
#endif //USE_QUICKPROF
	{



		{


			UpdateAabbs(broadphase,timeStep);


			float toi = 1.f;



			/*		if (m_ccdMode == 3)
			{
			DispatcherInfo dispatchInfo;
			dispatchInfo.m_timeStep = timeStep;
			dispatchInfo.m_stepCount = 0;
			dispatchInfo.m_dispatchFunc = DispatcherInfo::DISPATCH_CONTINUOUS;

			//			GetCollisionWorld()->GetDispatcher()->DispatchAllCollisionPairs(scene,dispatchInfo);
			toi = dispatchInfo.m_timeOfImpact;

			}
			*/



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

				if (true)
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

		return true;
	}
}



void	SimulationIsland::SyncMotionStates(float timeStep)
{
	std::vector<CcdPhysicsController*>::iterator i;

	//
	// synchronize the physics and graphics transformations
	//

	for (i=m_controllers.begin();
		!(i==m_controllers.end()); i++)
	{
		CcdPhysicsController* ctrl = (*i);
		ctrl->SynchronizeMotionStates(timeStep);

	}

}



void	SimulationIsland::UpdateAabbs(BroadphaseInterface* scene,float	timeStep)
{
	std::vector<CcdPhysicsController*>::iterator i;


	//
			// update aabbs, only for moving objects (!)
			//
			for (i=m_controllers.begin();
				!(i==m_controllers.end()); i++)
			{
				CcdPhysicsController* ctrl = (*i);
				RigidBody* body = ctrl->GetRigidBody();


				SimdPoint3 minAabb,maxAabb;
				CollisionShape* shapeinterface = ctrl->GetCollisionShape();



				shapeinterface->CalculateTemporalAabb(body->getCenterOfMassTransform(),
					body->getLinearVelocity(),
					//body->getAngularVelocity(),
					SimdVector3(0.f,0.f,0.f),//no angular effect for now //body->getAngularVelocity(),
					timeStep,minAabb,maxAabb);


				SimdVector3 manifoldExtraExtents(gContactBreakingTreshold,gContactBreakingTreshold,gContactBreakingTreshold);
				minAabb -= manifoldExtraExtents;
				maxAabb += manifoldExtraExtents;

				BroadphaseProxy* bp = body->m_broadphaseHandle;
				if (bp)
				{

					SimdVector3 color (1,1,0);

					class IDebugDraw*	m_debugDrawer = 0;
/*
					if (m_debugDrawer)
					{	
						//draw aabb
						switch (body->GetActivationState())
						{
						case ISLAND_SLEEPING:
							{
								color.setValue(1,1,1);
								break;
							}
						case WANTS_DEACTIVATION:
							{
								color.setValue(0,0,1);
								break;
							}
						case ACTIVE_TAG:
							{
								break;
							}
						case DISABLE_DEACTIVATION:
							{
								color.setValue(1,0,1);
							};

						};

						if (m_debugDrawer->GetDebugMode() & IDebugDraw::DBG_DrawAabb)
						{
							DrawAabb(m_debugDrawer,minAabb,maxAabb,color);
						}
					}
*/

			
					if ( (maxAabb-minAabb).length2() < 1e12f)
					{
						scene->SetAabb(bp,minAabb,maxAabb);
					} else
					{
						//something went wrong, investigate
						//removeCcdPhysicsController(ctrl);
						body->SetActivationState(DISABLE_SIMULATION);

						static bool reportMe = true;
						if (reportMe)
						{
							reportMe = false;
							printf("Overflow in AABB, object removed from simulation \n");
							printf("If you can reproduce this, please email bugs@continuousphysics.com\n");
							printf("Please include above information, your Platform, version of OS.\n");
							printf("Thanks.\n");
						}
						
					}

				}
			}
}