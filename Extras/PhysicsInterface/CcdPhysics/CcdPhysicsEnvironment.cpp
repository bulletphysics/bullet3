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




#include "CcdPhysicsEnvironment.h"
#include "CcdPhysicsController.h"

#include <algorithm>
#include "LinearMath/btTransform.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"

#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"


//profiling/timings
#include "LinearMath/btQuickprof.h"


#include "LinearMath/btIDebugDraw.h"

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"


#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "PHY_IMotionState.h"

#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"



#include "BulletCollision/CollisionShapes/btSphereShape.h"

bool useIslands = true;

#ifdef NEW_BULLET_VEHICLE_SUPPORT
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include "BulletDynamics/Vehicle/btVehicleRaycaster.h"

#include "BulletDynamics/Vehicle/btWheelInfo.h"
#include "PHY_IVehicle.h"
btRaycastVehicle::btVehicleTuning	gTuning;

#endif //NEW_BULLET_VEHICLE_SUPPORT
#include "LinearMath/btAabbUtil2.h"

#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"


//#include "BroadphaseCollision/QueryDispatcher.h"
//#include "BroadphaseCollision/QueryBox.h"
//todo: change this to allow dynamic registration of types!

#ifdef WIN32
void DrawRasterizerLine(const float* from,const float* to,int color);
#endif


#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"


#include <stdio.h>

#ifdef NEW_BULLET_VEHICLE_SUPPORT
class WrapperVehicle : public PHY_IVehicle
{

	btRaycastVehicle*	m_vehicle;
	PHY_IPhysicsController*	m_chassis;

public:

	WrapperVehicle(btRaycastVehicle* vehicle,PHY_IPhysicsController* chassis)
		:m_vehicle(vehicle),
		m_chassis(chassis)
	{
	}

	btRaycastVehicle*	GetVehicle()
	{
		return m_vehicle;
	}

	PHY_IPhysicsController*	GetChassis()
	{
		return m_chassis;
	}

	virtual void	AddWheel(
		PHY_IMotionState*	motionState,
		PHY__Vector3	connectionPoint,
		PHY__Vector3	downDirection,
		PHY__Vector3	axleDirection,
		float	suspensionRestLength,
		float wheelRadius,
		bool hasSteering
		)
	{
		btVector3 connectionPointCS0(connectionPoint[0],connectionPoint[1],connectionPoint[2]);
		btVector3 wheelDirectionCS0(downDirection[0],downDirection[1],downDirection[2]);
		btVector3 wheelAxle(axleDirection[0],axleDirection[1],axleDirection[2]);


		btWheelInfo& info = m_vehicle->AddWheel(connectionPointCS0,wheelDirectionCS0,wheelAxle,
			suspensionRestLength,wheelRadius,gTuning,hasSteering);
		info.m_clientInfo = motionState;

	}

	void	SyncWheels()
	{
		int numWheels = GetNumWheels();
		int i;
		for (i=0;i<numWheels;i++)
		{
			btWheelInfo& info = m_vehicle->GetWheelInfo(i);
			PHY_IMotionState* motionState = (PHY_IMotionState*)info.m_clientInfo ;
			m_vehicle->UpdateWheelTransform(i);
			btTransform trans = m_vehicle->GetWheelTransformWS(i);
			btQuaternion orn = trans.getRotation();
			const btVector3& pos = trans.getOrigin();
			motionState->setWorldOrientation(orn.x(),orn.y(),orn.z(),orn[3]);
			motionState->setWorldPosition(pos.x(),pos.y(),pos.z());

		}
	}

	virtual	int		GetNumWheels() const
	{
		return m_vehicle->GetNumWheels();
	}

	virtual void	GetWheelPosition(int wheelIndex,float& posX,float& posY,float& posZ) const
	{
		btTransform	trans = m_vehicle->GetWheelTransformWS(wheelIndex);
		posX = trans.getOrigin().x();
		posY = trans.getOrigin().y();
		posZ = trans.getOrigin().z();
	}
	virtual void	GetWheelOrientationQuaternion(int wheelIndex,float& quatX,float& quatY,float& quatZ,float& quatW) const
	{
		btTransform	trans = m_vehicle->GetWheelTransformWS(wheelIndex);
		btQuaternion quat = trans.getRotation();
		btMatrix3x3 orn2(quat);

		quatX = trans.getRotation().x();
		quatY = trans.getRotation().y();
		quatZ = trans.getRotation().z();
		quatW = trans.getRotation()[3];


		//printf("test");


	}

	virtual float	GetWheelRotation(int wheelIndex) const
	{
		float rotation = 0.f;

		if ((wheelIndex>=0) && (wheelIndex< m_vehicle->GetNumWheels()))
		{
			btWheelInfo& info = m_vehicle->GetWheelInfo(wheelIndex);
			rotation = info.m_rotation;
		}
		return rotation;

	}



	virtual int	GetUserConstraintId() const
	{
		return m_vehicle->GetUserConstraintId();
	}

	virtual int	GetUserConstraintType() const
	{
		return m_vehicle->GetUserConstraintType();
	}

	virtual	void	SetSteeringValue(float steering,int wheelIndex)
	{
		m_vehicle->SetSteeringValue(steering,wheelIndex);
	}

	virtual	void	ApplyEngineForce(float force,int wheelIndex)
	{
		m_vehicle->ApplyEngineForce(force,wheelIndex);
	}

	virtual	void	ApplyBraking(float braking,int wheelIndex)
	{
		if ((wheelIndex>=0) && (wheelIndex< m_vehicle->GetNumWheels()))
		{
			btWheelInfo& info = m_vehicle->GetWheelInfo(wheelIndex);
			info.m_brake = braking;
		}
	}

	virtual	void	SetWheelFriction(float friction,int wheelIndex)
	{
		if ((wheelIndex>=0) && (wheelIndex< m_vehicle->GetNumWheels()))
		{
			btWheelInfo& info = m_vehicle->GetWheelInfo(wheelIndex);
			info.m_frictionSlip = friction;
		}

	}

	virtual	void	SetSuspensionStiffness(float suspensionStiffness,int wheelIndex)
	{
		if ((wheelIndex>=0) && (wheelIndex< m_vehicle->GetNumWheels()))
		{
			btWheelInfo& info = m_vehicle->GetWheelInfo(wheelIndex);
			info.m_suspensionStiffness = suspensionStiffness;

		}
	}

	virtual	void	SetSuspensionDamping(float suspensionDamping,int wheelIndex)
	{
		if ((wheelIndex>=0) && (wheelIndex< m_vehicle->GetNumWheels()))
		{
			btWheelInfo& info = m_vehicle->GetWheelInfo(wheelIndex);
			info.m_wheelsDampingRelaxation = suspensionDamping;
		}
	}

	virtual	void	SetSuspensionCompression(float suspensionCompression,int wheelIndex)
	{
		if ((wheelIndex>=0) && (wheelIndex< m_vehicle->GetNumWheels()))
		{
			btWheelInfo& info = m_vehicle->GetWheelInfo(wheelIndex);
			info.m_wheelsDampingCompression = suspensionCompression;
		}
	}



	virtual	void	SetRollInfluence(float rollInfluence,int wheelIndex)
	{
		if ((wheelIndex>=0) && (wheelIndex< m_vehicle->GetNumWheels()))
		{
			btWheelInfo& info = m_vehicle->GetWheelInfo(wheelIndex);
			info.m_rollInfluence = rollInfluence;
		}
	}

	virtual void	SetCoordinateSystem(int rightIndex,int upIndex,int forwardIndex)
	{
		m_vehicle->SetCoordinateSystem(rightIndex,upIndex,forwardIndex);
	}



};
#endif //NEW_BULLET_VEHICLE_SUPPORT



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

			debugDrawer->DrawLine(pa,pb,color);
		}
		edgecoord = btVector3(-1.f,-1.f,-1.f);
		if (i<3)
			edgecoord[i]*=-1.f;
	}


}






CcdPhysicsEnvironment::CcdPhysicsEnvironment(btDispatcher* dispatcher,btOverlappingPairCache* pairCache)
: m_numIterations(10),
m_numTimeSubSteps(1),
m_ccdMode(0),
m_solverType(-1),
m_profileTimings(0),
m_enableSatCollisionDetection(false),
m_scalingPropagated(false)
{

	for (int i=0;i<PHY_NUM_RESPONSE;i++)
	{
		m_triggerCallbacks[i] = 0;
	}
	if (!dispatcher)
		dispatcher = new btCollisionDispatcher();


	if(!pairCache)
	{

		//todo: calculate/let user specify this world sizes
		btVector3 worldMin(-10000,-10000,-10000);
		btVector3 worldMax(10000,10000,10000);

		pairCache = new btAxisSweep3(worldMin,worldMax);

		//broadphase = new btSimpleBroadphase();
	}


	setSolverType(1);//issues with quickstep and memory allocations

	m_collisionWorld = new btCollisionWorld(dispatcher,pairCache);

	m_debugDrawer = 0;
	m_gravity = btVector3(0.f,-10.f,0.f);

	m_islandManager = new btSimulationIslandManager();


}

void	CcdPhysicsEnvironment::addCcdPhysicsController(CcdPhysicsController* ctrl)
{
	btRigidBody* body = ctrl->GetRigidBody();

	//this m_userPointer is just used for triggers, see CallbackTriggers
	body->m_internalOwner = ctrl;

	body->setGravity( m_gravity );
	m_controllers.push_back(ctrl);

	m_collisionWorld->AddCollisionObject(body,ctrl->GetCollisionFilterGroup(),ctrl->GetCollisionFilterMask());

	assert(body->m_broadphaseHandle);

	btCollisionShape* shapeinterface = ctrl->GetCollisionShape();

	assert(shapeinterface);

	const btTransform& t = ctrl->GetRigidBody()->getCenterOfMassTransform();
	
	body->m_cachedInvertedWorldTransform = body->m_worldTransform.inverse();

	btPoint3 minAabb,maxAabb;

	shapeinterface->GetAabb(t,minAabb,maxAabb);

	float timeStep = 0.02f;


	//extent it with the motion

	btVector3 linMotion = body->getLinearVelocity()*timeStep;

	float maxAabbx = maxAabb.getX();
	float maxAabby = maxAabb.getY();
	float maxAabbz = maxAabb.getZ();
	float minAabbx = minAabb.getX();
	float minAabby = minAabb.getY();
	float minAabbz = minAabb.getZ();

	if (linMotion.x() > 0.f)
		maxAabbx += linMotion.x(); 
	else
		minAabbx += linMotion.x();
	if (linMotion.y() > 0.f)
		maxAabby += linMotion.y(); 
	else
		minAabby += linMotion.y();
	if (linMotion.z() > 0.f)
		maxAabbz += linMotion.z(); 
	else
		minAabbz += linMotion.z();


	minAabb = btVector3(minAabbx,minAabby,minAabbz);
	maxAabb = btVector3(maxAabbx,maxAabby,maxAabbz);




}

void	CcdPhysicsEnvironment::removeCcdPhysicsController(CcdPhysicsController* ctrl)
{

	//also remove constraint

	{
		std::vector<btTypedConstraint*>::iterator i;

		for (i=m_constraints.begin();
			!(i==m_constraints.end()); i++)
		{
			btTypedConstraint* constraint = (*i);
			if  ((&constraint->GetRigidBodyA() == ctrl->GetRigidBody() ||
				(&constraint->GetRigidBodyB() == ctrl->GetRigidBody())))
			{
				removeConstraint(constraint->GetUserConstraintId());
				//only 1 constraint per constroller
				break;
			}
		}
	}

	{
		std::vector<btTypedConstraint*>::iterator i;

		for (i=m_constraints.begin();
			!(i==m_constraints.end()); i++)
		{
			btTypedConstraint* constraint = (*i);
			if  ((&constraint->GetRigidBodyA() == ctrl->GetRigidBody() ||
				(&constraint->GetRigidBodyB() == ctrl->GetRigidBody())))
			{
				removeConstraint(constraint->GetUserConstraintId());
				//only 1 constraint per constroller
				break;
			}
		}
	}


	m_collisionWorld->RemoveCollisionObject(ctrl->GetRigidBody());


	{
		std::vector<CcdPhysicsController*>::iterator i =
			std::find(m_controllers.begin(), m_controllers.end(), ctrl);
		if (!(i == m_controllers.end()))
		{
			std::swap(*i, m_controllers.back());
			m_controllers.pop_back();
		}
	}

	//remove it from the triggers
	{
		std::vector<CcdPhysicsController*>::iterator i =
			std::find(m_triggerControllers.begin(), m_triggerControllers.end(), ctrl);
		if (!(i == m_triggerControllers.end()))
		{
			std::swap(*i, m_triggerControllers.back());
			m_triggerControllers.pop_back();
		}
	}


}


void	CcdPhysicsEnvironment::beginFrame()
{

}


bool	CcdPhysicsEnvironment::proceedDeltaTime(double curTime,float timeStep)
{
	//don't simulate without timesubsteps
	if (m_numTimeSubSteps<1)
		return true;

	//printf("proceedDeltaTime\n");
	

#ifdef USE_QUICKPROF
	//toggle btProfiler
	if ( m_debugDrawer && m_debugDrawer->GetDebugMode() & btIDebugDraw::DBG_ProfileTimings)
	{
		if (!m_profileTimings)
		{
			m_profileTimings = 1;
			// To disable profiling, simply comment out the following line.
			static int counter = 0;

			char filename[128];
			sprintf(filename,"quickprof_bullet_timings%i.csv",counter++);
			btProfiler::init(filename, btProfiler::BLOCK_CYCLE_SECONDS);//BLOCK_TOTAL_MICROSECONDS

		}
	} else
	{
		if (m_profileTimings)
		{
			m_profileTimings = 0;
			btProfiler::destroy();
		}
	}
#endif //USE_QUICKPROF



	if (!btFuzzyZero(timeStep))
	{
		
		{
			//do the kinematic calculation here, over the full timestep
			std::vector<CcdPhysicsController*>::iterator i;
			for (i=m_controllers.begin();
						!(i==m_controllers.end()); i++)
			{

						CcdPhysicsController* ctrl = *i;

						btTransform predictedTrans;
						btRigidBody* body = ctrl->GetRigidBody();
						if (body->GetActivationState() != ISLAND_SLEEPING)
						{

							if (body->IsStatic())
							{
								//to calculate velocities next frame
								body->saveKinematicState(timeStep);
							}
						}
			}
		}


		int i;
		float subTimeStep = timeStep / float(m_numTimeSubSteps);

		for (i=0;i<this->m_numTimeSubSteps;i++)
		{
			proceedDeltaTimeOneStep(subTimeStep);
		}
	} else
	{
		//todo: interpolate
	}

	return true;
}













/// Perform an integration step of duration 'timeStep'.
bool	CcdPhysicsEnvironment::proceedDeltaTimeOneStep(float timeStep)
{


	//printf("CcdPhysicsEnvironment::proceedDeltaTime\n");

	if (btFuzzyZero(timeStep))
		return true;

	if (m_debugDrawer)
	{
		gDisableDeactivation = (m_debugDrawer->GetDebugMode() & btIDebugDraw::DBG_NoDeactivation);
	}


#ifdef USE_QUICKPROF
	btProfiler::beginBlock("SyncMotionStates");
#endif //USE_QUICKPROF


	//this is needed because scaling is not known in advance, and scaling has to propagate to the shape
	if (!m_scalingPropagated)
	{
		SyncMotionStates(timeStep);
		m_scalingPropagated = true;
	}


#ifdef USE_QUICKPROF
	btProfiler::endBlock("SyncMotionStates");

	btProfiler::beginBlock("predictIntegratedTransform");
#endif //USE_QUICKPROF

	{
		//		std::vector<CcdPhysicsController*>::iterator i;



		int k;
		for (k=0;k<GetNumControllers();k++)
		{
			CcdPhysicsController* ctrl = m_controllers[k];
			//		btTransform predictedTrans;
			btRigidBody* body = ctrl->GetRigidBody();
			
			body->m_cachedInvertedWorldTransform = body->m_worldTransform.inverse();

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
	btProfiler::endBlock("predictIntegratedTransform");
#endif //USE_QUICKPROF

	btOverlappingPairCache*	scene = m_collisionWorld->GetPairCache();


	//
	// collision detection (?)
	//


#ifdef USE_QUICKPROF
	btProfiler::beginBlock("DispatchAllCollisionPairs");
#endif //USE_QUICKPROF


	int numsubstep = m_numIterations;


	btDispatcherInfo dispatchInfo;
	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_enableSatConvex = m_enableSatCollisionDetection;
	dispatchInfo.m_debugDraw = this->m_debugDrawer;

	scene->RefreshOverlappingPairs();
	
	GetCollisionWorld()->GetDispatcher()->DispatchAllCollisionPairs(scene,dispatchInfo);


#ifdef USE_QUICKPROF
	btProfiler::endBlock("DispatchAllCollisionPairs");
#endif //USE_QUICKPROF

	m_islandManager->UpdateActivationState(GetCollisionWorld(),GetCollisionWorld()->GetDispatcher());

	{
		int i;
		int numConstraints = m_constraints.size();
		for (i=0;i< numConstraints ; i++ )
		{
			btTypedConstraint* constraint = m_constraints[i];

			const btRigidBody* colObj0 = &constraint->GetRigidBodyA();
			const btRigidBody* colObj1 = &constraint->GetRigidBodyB();
			
			if (((colObj0) && ((colObj0)->mergesSimulationIslands())) &&
						((colObj1) && ((colObj1)->mergesSimulationIslands())))
			{
				if (colObj0->IsActive() || colObj1->IsActive())
				{

					m_islandManager->GetUnionFind().unite((colObj0)->m_islandTag1,
						(colObj1)->m_islandTag1);
				}
			}
		}
	}

	m_islandManager->StoreIslandActivationState(GetCollisionWorld());


	//contacts
#ifdef USE_QUICKPROF
	btProfiler::beginBlock("SolveConstraint");
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
			btTypedConstraint* constraint = m_constraints[i];

			constraint->BuildJacobian();
			constraint->SolveConstraint( timeStep );

		}


	}

#ifdef USE_QUICKPROF
	btProfiler::endBlock("SolveConstraint");
#endif //USE_QUICKPROF

	//solve the vehicles

#ifdef NEW_BULLET_VEHICLE_SUPPORT
	//vehicles
	int numVehicles = m_wrapperVehicles.size();
	for (int i=0;i<numVehicles;i++)
	{
		WrapperVehicle* wrapperVehicle = m_wrapperVehicles[i];
		btRaycastVehicle* vehicle = wrapperVehicle->GetVehicle();
		vehicle->UpdateVehicle( timeStep);
	}
#endif //NEW_BULLET_VEHICLE_SUPPORT


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
	btProfiler::beginBlock("BuildAndProcessIslands");
#endif //USE_QUICKPROF

	/// solve all the contact points and contact friction
	m_islandManager->BuildAndProcessIslands(GetCollisionWorld()->GetDispatcher(),m_collisionWorld->GetCollisionObjectArray(),&solverCallback);

#ifdef USE_QUICKPROF
	btProfiler::endBlock("BuildAndProcessIslands");

	btProfiler::beginBlock("CallbackTriggers");
#endif //USE_QUICKPROF

	CallbackTriggers();

#ifdef USE_QUICKPROF
	btProfiler::endBlock("CallbackTriggers");


	btProfiler::beginBlock("proceedToTransform");

#endif //USE_QUICKPROF
	{



		{

			
			
			UpdateAabbs(timeStep);


			float toi = 1.f;



			if (m_ccdMode == 3)
			{
				btDispatcherInfo dispatchInfo;
				dispatchInfo.m_timeStep = timeStep;
				dispatchInfo.m_stepCount = 0;
				dispatchInfo.m_dispatchFunc = btDispatcherInfo::DISPATCH_CONTINUOUS;

				//pairCache->RefreshOverlappingPairs();//??
				GetCollisionWorld()->GetDispatcher()->DispatchAllCollisionPairs(scene,dispatchInfo);
				
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

					btTransform predictedTrans;
					btRigidBody* body = ctrl->GetRigidBody();
					
					if (body->IsActive())
					{

						if (!body->IsStatic())
						{
							if (body->m_hitFraction < 1.f)
							{
								//set velocity to zero... until we have proper CCD integrated
								body->setLinearVelocity(body->getLinearVelocity()*0.5f);
							}
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
				btRigidBody* body = ctrl->GetRigidBody();

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
		btProfiler::endBlock("proceedToTransform");

		btProfiler::beginBlock("SyncMotionStates");
#endif //USE_QUICKPROF

		SyncMotionStates(timeStep);

#ifdef USE_QUICKPROF
		btProfiler::endBlock("SyncMotionStates");

		btProfiler::endProfilingCycle();
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

	return true;
}

void		CcdPhysicsEnvironment::setDebugMode(int debugMode)
{
	if (m_debugDrawer){
		m_debugDrawer->SetDebugMode(debugMode);
	}
}

void		CcdPhysicsEnvironment::setNumIterations(int numIter)
{
	m_numIterations = numIter;
}
void		CcdPhysicsEnvironment::setDeactivationTime(float dTime)
{
	gDeactivationTime = dTime;
}
void		CcdPhysicsEnvironment::setDeactivationLinearTreshold(float linTresh)
{
	gLinearSleepingTreshold = linTresh;
}
void		CcdPhysicsEnvironment::setDeactivationAngularTreshold(float angTresh) 
{
	gAngularSleepingTreshold = angTresh;
}
void		CcdPhysicsEnvironment::setContactBreakingTreshold(float contactBreakingTreshold)
{
	gContactBreakingTreshold = contactBreakingTreshold;

}


void		CcdPhysicsEnvironment::setCcdMode(int ccdMode)
{
	m_ccdMode = ccdMode;
}


void		CcdPhysicsEnvironment::setSolverSorConstant(float sor)
{
	m_solverInfo.m_sor = sor;
}

void		CcdPhysicsEnvironment::setSolverTau(float tau)
{
	m_solverInfo.m_tau = tau;
}
void		CcdPhysicsEnvironment::setSolverDamping(float damping)
{
	m_solverInfo.m_damping = damping;
}


void		CcdPhysicsEnvironment::setLinearAirDamping(float damping)
{
	gLinearAirDamping = damping;
}

void		CcdPhysicsEnvironment::setUseEpa(bool epa)
{
	gUseEpa = epa;
}

void		CcdPhysicsEnvironment::setSolverType(int solverType)
{

	switch (solverType)
	{
	case 1:
		{
			if (m_solverType != solverType)
			{

				m_solver = new btSequentialImpulseConstraintSolver();

				break;
			}
		}

	case 0:
	default:
		if (m_solverType != solverType)
		{
//			m_solver = new OdeConstraintSolver();

			break;
		}

	};

	m_solverType = solverType ;
}





void	CcdPhysicsEnvironment::SyncMotionStates(float timeStep)
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
void		CcdPhysicsEnvironment::setGravity(float x,float y,float z)
{
	m_gravity = btVector3(x,y,z);

	std::vector<CcdPhysicsController*>::iterator i;

	//todo: review this gravity stuff
	for (i=m_controllers.begin();
		!(i==m_controllers.end()); i++)
	{

		CcdPhysicsController* ctrl = (*i);
		ctrl->GetRigidBody()->setGravity(m_gravity);

	}
}


#ifdef NEW_BULLET_VEHICLE_SUPPORT

class DefaultVehicleRaycaster : public btVehicleRaycaster
{
	CcdPhysicsEnvironment* m_physEnv;
	PHY_IPhysicsController*	m_chassis;

public:
	DefaultVehicleRaycaster(CcdPhysicsEnvironment* physEnv,PHY_IPhysicsController* chassis):
	  m_physEnv(physEnv),
		  m_chassis(chassis)
	  {
	  }

		virtual ~DefaultVehicleRaycaster()
		{
		}
	  /*	struct btVehicleRaycasterResult
	  {
	  btVehicleRaycasterResult() :m_distFraction(-1.f){};
	  btVector3	m_hitPointInWorld;
	  btVector3	m_hitNormalInWorld;
	  btScalar	m_distFraction;
	  };
	  */
	  virtual void* CastRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result)
	  {


		  float hit[3];
		  float normal[3];

		  PHY_IPhysicsController*	ignore = m_chassis;
		  void* hitObject = m_physEnv->rayTest(ignore,from.x(),from.y(),from.z(),to.x(),to.y(),to.z(),hit[0],hit[1],hit[2],normal[0],normal[1],normal[2]);
		  if (hitObject)
		  {
			  result.m_hitPointInWorld[0] = hit[0];
			  result.m_hitPointInWorld[1] = hit[1];
			  result.m_hitPointInWorld[2] = hit[2];
			  result.m_hitNormalInWorld[0] = normal[0];
			  result.m_hitNormalInWorld[1] = normal[1];
			  result.m_hitNormalInWorld[2] = normal[2];
			  result.m_hitNormalInWorld.normalize();
			  //calc fraction? or put it in the interface?
			  //calc for now

			  result.m_distFraction = (result.m_hitPointInWorld-from).length() / (to-from).length();
			  //some safety for 'explosion' due to sudden penetration of the full 'ray'
			  /*			if (result.m_distFraction<0.1)
			  {
			  printf("Vehicle Raycast: avoided instability due to penetration. Consider moving the connection points deeper inside vehicle chassis");
			  result.m_distFraction = 1.f;
			  hitObject = 0;
			  }
			  */

			  /*			if (result.m_distFraction>1.)
			  {
			  printf("Vehicle Raycast: avoided instability 1Consider moving the connection points deeper inside vehicle chassis");				
			  result.m_distFraction = 1.f;
			  hitObject = 0;
			  }
			  */



		  }
		  //?
		  return hitObject;
	  }
};
#endif //NEW_BULLET_VEHICLE_SUPPORT

static int gConstraintUid = 1;

int			CcdPhysicsEnvironment::createConstraint(class PHY_IPhysicsController* ctrl0,class PHY_IPhysicsController* ctrl1,PHY_ConstraintType type,
													float pivotX,float pivotY,float pivotZ,
													float axisX,float axisY,float axisZ)
{


	CcdPhysicsController* c0 = (CcdPhysicsController*)ctrl0;
	CcdPhysicsController* c1 = (CcdPhysicsController*)ctrl1;

	btRigidBody* rb0 = c0 ? c0->GetRigidBody() : 0;
	btRigidBody* rb1 = c1 ? c1->GetRigidBody() : 0;

	ASSERT(rb0);

	btVector3 pivotInA(pivotX,pivotY,pivotZ);
	btVector3 pivotInB = rb1 ? rb1->getCenterOfMassTransform().inverse()(rb0->getCenterOfMassTransform()(pivotInA)) : pivotInA;
	btVector3 axisInA(axisX,axisY,axisZ);
	btVector3 axisInB = rb1 ? 
		(rb1->getCenterOfMassTransform().getBasis().inverse()*(rb0->getCenterOfMassTransform().getBasis() * axisInA)) : 
	rb0->getCenterOfMassTransform().getBasis() * axisInA;

	bool angularOnly = false;

	switch (type)
	{
	case PHY_POINT2POINT_CONSTRAINT:
		{

			btPoint2PointConstraint* p2p = 0;

			if (rb1)
			{
				p2p = new btPoint2PointConstraint(*rb0,
					*rb1,pivotInA,pivotInB);
			} else
			{
				p2p = new btPoint2PointConstraint(*rb0,
					pivotInA);
			}

			m_constraints.push_back(p2p);
			p2p->SetUserConstraintId(gConstraintUid++);
			p2p->SetUserConstraintType(type);
			//64 bit systems can't cast pointer to int. could use size_t instead.
			return p2p->GetUserConstraintId();

			break;
		}

	case PHY_GENERIC_6DOF_CONSTRAINT:
		{
			btGeneric6DofConstraint* genericConstraint = 0;

			if (rb1)
			{
				btTransform frameInA;
				btTransform frameInB;
				
				btVector3 axis1, axis2;
				btPlaneSpace1( axisInA, axis1, axis2 );

				frameInA.getBasis().setValue( axisInA.x(), axis1.x(), axis2.x(),
					                          axisInA.y(), axis1.y(), axis2.y(),
											  axisInA.z(), axis1.z(), axis2.z() );

	
				btPlaneSpace1( axisInB, axis1, axis2 );
				frameInB.getBasis().setValue( axisInB.x(), axis1.x(), axis2.x(),
					                          axisInB.y(), axis1.y(), axis2.y(),
											  axisInB.z(), axis1.z(), axis2.z() );

				frameInA.setOrigin( pivotInA );
				frameInB.setOrigin( pivotInB );

				genericConstraint = new btGeneric6DofConstraint(
					*rb0,*rb1,
					frameInA,frameInB);


			} else
			{
				// TODO: Implement single body case...

			}
			

			m_constraints.push_back(genericConstraint);
			genericConstraint->SetUserConstraintId(gConstraintUid++);
			genericConstraint->SetUserConstraintType(type);
			//64 bit systems can't cast pointer to int. could use size_t instead.
			return genericConstraint->GetUserConstraintId();

			break;
		}
	case PHY_ANGULAR_CONSTRAINT:
		angularOnly = true;


	case PHY_LINEHINGE_CONSTRAINT:
		{
			btHingeConstraint* hinge = 0;

			if (rb1)
			{
				hinge = new btHingeConstraint(
					*rb0,
					*rb1,pivotInA,pivotInB,axisInA,axisInB);


			} else
			{
				hinge = new btHingeConstraint(*rb0,
					pivotInA,axisInA);

			}
			hinge->setAngularOnly(angularOnly);

			m_constraints.push_back(hinge);
			hinge->SetUserConstraintId(gConstraintUid++);
			hinge->SetUserConstraintType(type);
			//64 bit systems can't cast pointer to int. could use size_t instead.
			return hinge->GetUserConstraintId();
			break;
		}
#ifdef NEW_BULLET_VEHICLE_SUPPORT

	case PHY_VEHICLE_CONSTRAINT:
		{
			btRaycastVehicle::btVehicleTuning* tuning = new btRaycastVehicle::btVehicleTuning();
			btRigidBody* chassis = rb0;
			DefaultVehicleRaycaster* raycaster = new DefaultVehicleRaycaster(this,ctrl0);
			btRaycastVehicle* vehicle = new btRaycastVehicle(*tuning,chassis,raycaster);
			WrapperVehicle* wrapperVehicle = new WrapperVehicle(vehicle,ctrl0);
			m_wrapperVehicles.push_back(wrapperVehicle);
			vehicle->SetUserConstraintId(gConstraintUid++);
			vehicle->SetUserConstraintType(type);
			return vehicle->GetUserConstraintId();

			break;
		};
#endif //NEW_BULLET_VEHICLE_SUPPORT

	default:
		{
		}
	};

	//RigidBody& rbA,btRigidBody& rbB, const btVector3& pivotInA,const btVector3& pivotInB

	return 0;

}




//Following the COLLADA physics specification for constraints
int			CcdPhysicsEnvironment::createUniversalD6Constraint(
						class PHY_IPhysicsController* ctrlRef,class PHY_IPhysicsController* ctrlOther,
						btTransform& frameInA,
						btTransform& frameInB,
						const btVector3& linearMinLimits,
						const btVector3& linearMaxLimits,
						const btVector3& angularMinLimits,
						const btVector3& angularMaxLimits
)
{

	//we could either add some logic to recognize ball-socket and hinge, or let that up to the user
	//perhaps some warning or hint that hinge/ball-socket is more efficient?
	
	btGeneric6DofConstraint* genericConstraint = 0;
	CcdPhysicsController* ctrl0 = (CcdPhysicsController*) ctrlRef;
	CcdPhysicsController* ctrl1 = (CcdPhysicsController*) ctrlOther;
	
	btRigidBody* rb0 = ctrl0->GetRigidBody();
	btRigidBody* rb1 = ctrl1->GetRigidBody();

	if (rb1)
	{
		

		genericConstraint = new btGeneric6DofConstraint(
			*rb0,*rb1,
			frameInA,frameInB);
		genericConstraint->setLinearLowerLimit(linearMinLimits);
		genericConstraint->setLinearUpperLimit(linearMaxLimits);
		genericConstraint->setAngularLowerLimit(angularMinLimits);
		genericConstraint->setAngularUpperLimit(angularMaxLimits);
	} else
	{
		// TODO: Implement single body case...
		//No, we can use a fixed rigidbody in above code, rather then unnecessary duplation of code

	}
	
	if (genericConstraint)
	{
		m_constraints.push_back(genericConstraint);
		genericConstraint->SetUserConstraintId(gConstraintUid++);
		genericConstraint->SetUserConstraintType(PHY_GENERIC_6DOF_CONSTRAINT);
		//64 bit systems can't cast pointer to int. could use size_t instead.
		return genericConstraint->GetUserConstraintId();
	}
	return 0;
}



void		CcdPhysicsEnvironment::removeConstraint(int	constraintId)
{
	std::vector<btTypedConstraint*>::iterator i;

	for (i=m_constraints.begin();
		!(i==m_constraints.end()); i++)
	{
		btTypedConstraint* constraint = (*i);
		if (constraint->GetUserConstraintId() == constraintId)
		{
			std::swap(*i, m_constraints.back());
			m_constraints.pop_back();
			break;
		}
	}

}


	struct	FilterClosestRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
	{
		PHY_IPhysicsController*	m_ignoreClient;

		FilterClosestRayResultCallback (PHY_IPhysicsController* ignoreClient,const btVector3& rayFrom,const btVector3& rayTo)
			: btCollisionWorld::ClosestRayResultCallback(rayFrom,rayTo),
			m_ignoreClient(ignoreClient)
		{

		}

		virtual ~FilterClosestRayResultCallback()
		{
		}

		virtual	float	AddSingleResult(const btCollisionWorld::LocalRayResult& rayResult)
		{
			CcdPhysicsController* curHit = static_cast<CcdPhysicsController*>(rayResult.m_collisionObject->m_internalOwner);
			//ignore client...
			if (curHit != m_ignoreClient)
			{		
				//if valid
				return ClosestRayResultCallback::AddSingleResult(rayResult);
			}
			return m_closestHitFraction;
		}

	};

PHY_IPhysicsController* CcdPhysicsEnvironment::rayTest(PHY_IPhysicsController* ignoreClient, float fromX,float fromY,float fromZ, float toX,float toY,float toZ, 
													   float& hitX,float& hitY,float& hitZ,float& normalX,float& normalY,float& normalZ)
{
	btVector3 rayFrom(fromX,fromY,fromZ);
	btVector3 rayTo(toX,toY,toZ);

	btVector3	hitPointWorld,normalWorld;

	//Either Ray Cast with or without filtering

	//CollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);
	FilterClosestRayResultCallback	 rayCallback(ignoreClient,rayFrom,rayTo);


	PHY_IPhysicsController* nearestHit = 0;

	m_collisionWorld->RayTest(rayFrom,rayTo,rayCallback);
	if (rayCallback.HasHit())
	{
		nearestHit = static_cast<CcdPhysicsController*>(rayCallback.m_collisionObject->m_internalOwner);
		hitX = 	rayCallback.m_hitPointWorld.getX();
		hitY = 	rayCallback.m_hitPointWorld.getY();
		hitZ = 	rayCallback.m_hitPointWorld.getZ();

		normalX = rayCallback.m_hitNormalWorld.getX();
		normalY = rayCallback.m_hitNormalWorld.getY();
		normalZ = rayCallback.m_hitNormalWorld.getZ();

	}	


	return nearestHit;
}



int	CcdPhysicsEnvironment::getNumContactPoints()
{
	return 0;
}

void CcdPhysicsEnvironment::getContactPoint(int i,float& hitX,float& hitY,float& hitZ,float& normalX,float& normalY,float& normalZ)
{

}




btBroadphaseInterface*	CcdPhysicsEnvironment::GetBroadphase()
{ 
	return m_collisionWorld->GetBroadphase(); 
}




CcdPhysicsEnvironment::~CcdPhysicsEnvironment()
{

#ifdef NEW_BULLET_VEHICLE_SUPPORT
	m_wrapperVehicles.clear();
#endif //NEW_BULLET_VEHICLE_SUPPORT

	//m_broadphase->DestroyScene();
	//delete broadphase ? release reference on broadphase ?

	//first delete scene, then dispatcher, because pairs have to release manifolds on the dispatcher
	//delete m_dispatcher;
	delete m_collisionWorld;
	
	delete m_islandManager;

}


int	CcdPhysicsEnvironment::GetNumControllers()
{
	return m_controllers.size();
}


CcdPhysicsController* CcdPhysicsEnvironment::GetPhysicsController( int index)
{
	return m_controllers[index];
}






btTypedConstraint*	CcdPhysicsEnvironment::getConstraintById(int constraintId)
{
	int numConstraint = m_constraints.size();
	int i;
	for (i=0;i<numConstraint;i++)
	{
		btTypedConstraint* constraint = m_constraints[i];
		if (constraint->GetUserConstraintId()==constraintId)
		{
			return constraint;
		}
	}
	return 0;
}


void CcdPhysicsEnvironment::addSensor(PHY_IPhysicsController* ctrl)
{

	CcdPhysicsController* ctrl1 = (CcdPhysicsController* )ctrl;
	std::vector<CcdPhysicsController*>::iterator i =
		std::find(m_controllers.begin(), m_controllers.end(), ctrl);
	if ((i == m_controllers.end()))
	{
		addCcdPhysicsController(ctrl1);
	}

	requestCollisionCallback(ctrl);
	//printf("addSensor\n");
}

void CcdPhysicsEnvironment::removeCollisionCallback(PHY_IPhysicsController* ctrl)
{
	std::vector<CcdPhysicsController*>::iterator i =
		std::find(m_triggerControllers.begin(), m_triggerControllers.end(), ctrl);
	if (!(i == m_triggerControllers.end()))
	{
		std::swap(*i, m_triggerControllers.back());
		m_triggerControllers.pop_back();
	}
}


void CcdPhysicsEnvironment::removeSensor(PHY_IPhysicsController* ctrl)
{
	removeCollisionCallback(ctrl);
	//printf("removeSensor\n");
}
void CcdPhysicsEnvironment::addTouchCallback(int response_class, PHY_ResponseCallback callback, void *user)
{
	/*	printf("addTouchCallback\n(response class = %i)\n",response_class);

	//map PHY_ convention into SM_ convention
	switch (response_class)
	{
	case	PHY_FH_RESPONSE:
	printf("PHY_FH_RESPONSE\n");
	break;
	case PHY_SENSOR_RESPONSE:
	printf("PHY_SENSOR_RESPONSE\n");
	break;
	case PHY_CAMERA_RESPONSE:
	printf("PHY_CAMERA_RESPONSE\n");
	break;
	case PHY_OBJECT_RESPONSE:
	printf("PHY_OBJECT_RESPONSE\n");
	break;
	case PHY_STATIC_RESPONSE:
	printf("PHY_STATIC_RESPONSE\n");
	break;
	default:
	assert(0);
	return;
	}
	*/

	m_triggerCallbacks[response_class] = callback;
	m_triggerCallbacksUserPtrs[response_class] = user;

}
void CcdPhysicsEnvironment::requestCollisionCallback(PHY_IPhysicsController* ctrl)
{
	CcdPhysicsController* ccdCtrl = static_cast<CcdPhysicsController*>(ctrl);

	//printf("requestCollisionCallback\n");
	m_triggerControllers.push_back(ccdCtrl);
}


void	CcdPhysicsEnvironment::CallbackTriggers()
{
	if (m_triggerCallbacks[PHY_OBJECT_RESPONSE] || (m_debugDrawer && (m_debugDrawer->GetDebugMode() & btIDebugDraw::DBG_DrawContactPoints)))
	{
		//walk over all overlapping pairs, and if one of the involved bodies is registered for trigger callback, perform callback
		int numManifolds = m_collisionWorld->GetDispatcher()->GetNumManifolds();
		for (int i=0;i<numManifolds;i++)
		{
			btPersistentManifold* manifold = m_collisionWorld->GetDispatcher()->GetManifoldByIndexInternal(i);
			int numContacts = manifold->GetNumContacts();
			if (numContacts)
			{
				if (m_debugDrawer && (m_debugDrawer->GetDebugMode() & btIDebugDraw::DBG_DrawContactPoints))
				{
					for (int j=0;j<numContacts;j++)
					{
						btVector3 color(1,0,0);
						const btManifoldPoint& cp = manifold->GetContactPoint(j);
						if (m_debugDrawer)
							m_debugDrawer->DrawContactPoint(cp.m_positionWorldOnB,cp.m_normalWorldOnB,cp.GetDistance(),cp.GetLifeTime(),color);
					}
				}
				btRigidBody* obj0 = static_cast<btRigidBody* >(manifold->GetBody0());
				btRigidBody* obj1 = static_cast<btRigidBody* >(manifold->GetBody1());

				//m_internalOwner is set in 'addPhysicsController'
				CcdPhysicsController* ctrl0 = static_cast<CcdPhysicsController*>(obj0->m_internalOwner);
				CcdPhysicsController* ctrl1 = static_cast<CcdPhysicsController*>(obj1->m_internalOwner);

				std::vector<CcdPhysicsController*>::iterator i =
					std::find(m_triggerControllers.begin(), m_triggerControllers.end(), ctrl0);
				if (i == m_triggerControllers.end())
				{
					i = std::find(m_triggerControllers.begin(), m_triggerControllers.end(), ctrl1);
				}

				if (!(i == m_triggerControllers.end()))
				{
					m_triggerCallbacks[PHY_OBJECT_RESPONSE](m_triggerCallbacksUserPtrs[PHY_OBJECT_RESPONSE],
						ctrl0,ctrl1,0);
				}
			}
		}



	}


}






#ifdef NEW_BULLET_VEHICLE_SUPPORT

//complex constraint for vehicles
PHY_IVehicle*	CcdPhysicsEnvironment::getVehicleConstraint(int constraintId)
{
	int i;

	int numVehicles = m_wrapperVehicles.size();
	for (i=0;i<numVehicles;i++)
	{
		WrapperVehicle* wrapperVehicle = m_wrapperVehicles[i];
		if (wrapperVehicle->GetVehicle()->GetUserConstraintId() == constraintId)
			return wrapperVehicle;
	}

	return 0;
}

#endif //NEW_BULLET_VEHICLE_SUPPORT


int currentController = 0;
int numController = 0;


void	CcdPhysicsEnvironment::UpdateAabbs(float	timeStep)
{
	std::vector<CcdPhysicsController*>::iterator i;
	btBroadphaseInterface* scene =  GetBroadphase();

	numController = m_controllers.size();
	currentController = 0;

	//
			// update aabbs, only for moving objects (!)
			//
			for (i=m_controllers.begin();
				!(i==m_controllers.end()); i++)
			{
				currentController++;
				CcdPhysicsController* ctrl = (*i);
				btRigidBody* body = ctrl->GetRigidBody();


				btPoint3 minAabb,maxAabb;
				btCollisionShape* shapeinterface = ctrl->GetCollisionShape();



				shapeinterface->CalculateTemporalAabb(body->getCenterOfMassTransform(),
					body->getLinearVelocity(),
					//body->getAngularVelocity(),
					btVector3(0.f,0.f,0.f),//no angular effect for now //body->getAngularVelocity(),
					timeStep,minAabb,maxAabb);


				btVector3 manifoldExtraExtents(gContactBreakingTreshold,gContactBreakingTreshold,gContactBreakingTreshold);
				minAabb -= manifoldExtraExtents;
				maxAabb += manifoldExtraExtents;

				btBroadphaseProxy* bp = body->m_broadphaseHandle;
				if (bp)
				{

					btVector3 color (1,1,0);

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

						if (m_debugDrawer->GetDebugMode() & btIDebugDraw::DBG_DrawAabb)
						{
							DrawAabb(m_debugDrawer,minAabb,maxAabb,color);
						}
					}

			
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

PHY_IPhysicsController*	CcdPhysicsEnvironment::CreateSphereController(float radius,const PHY__Vector3& position)
{
	
	btCcdConstructionInfo	cinfo;
	cinfo.m_collisionShape = new btSphereShape(radius);
	cinfo.m_MotionState = 0;
	cinfo.m_physicsEnv = this;
	cinfo.m_collisionFlags |= btCollisionObject::noContactResponse;
	DefaultMotionState* motionState = new DefaultMotionState();
	cinfo.m_MotionState = motionState;
	motionState->m_worldTransform.setIdentity();
	motionState->m_worldTransform.setOrigin(btVector3(position[0],position[1],position[2]));

	CcdPhysicsController* sphereController = new CcdPhysicsController(cinfo);
	

	return sphereController;
}


PHY_IPhysicsController* CcdPhysicsEnvironment::CreateConeController(float coneradius,float coneheight)
{
	btCcdConstructionInfo	cinfo;
	cinfo.m_collisionShape = new btConeShape(coneradius,coneheight);
	cinfo.m_MotionState = 0;
	cinfo.m_physicsEnv = this;
	DefaultMotionState* motionState = new DefaultMotionState();
	cinfo.m_MotionState = motionState;
	motionState->m_worldTransform.setIdentity();
//	motionState->m_worldTransform.setOrigin(btVector3(position[0],position[1],position[2]));

	CcdPhysicsController* sphereController = new CcdPhysicsController(cinfo);


	return sphereController;
}
	
float		CcdPhysicsEnvironment::getAppliedImpulse(int	constraintid)
{
	std::vector<btTypedConstraint*>::iterator i;

	for (i=m_constraints.begin();
		!(i==m_constraints.end()); i++)
	{
		btTypedConstraint* constraint = (*i);
		if (constraint->GetUserConstraintId() == constraintid)
		{
			return constraint->GetAppliedImpulse();
		}
	}
	return 0.f;
}
