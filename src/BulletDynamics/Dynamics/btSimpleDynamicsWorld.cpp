
#include "btSimpleDynamicsWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"


btSimpleDynamicsWorld::btSimpleDynamicsWorld()
:btDynamicsWorld(new	CollisionDispatcher(),new SimpleBroadphase()),
m_constraintSolver(new SequentialImpulseConstraintSolver)
{

}

btSimpleDynamicsWorld::btSimpleDynamicsWorld(Dispatcher* dispatcher,OverlappingPairCache* pairCache,ConstraintSolver* constraintSolver)
:btDynamicsWorld(dispatcher,pairCache),
m_constraintSolver(constraintSolver)
{

}


btSimpleDynamicsWorld::~btSimpleDynamicsWorld()
{
	delete m_constraintSolver;

	//delete the dispatcher and paircache
	delete m_dispatcher1;
	m_dispatcher1 = 0;
	delete m_pairCache;
	m_pairCache = 0;
}

void	btSimpleDynamicsWorld::stepSimulation(float timeStep)
{
	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	///perform collision detection
	PerformDiscreteCollisionDetection();

	///solve contact constraints
	PersistentManifold** manifoldPtr = ((CollisionDispatcher*)m_dispatcher1)->getInternalManifoldPointer();
	int numManifolds = m_dispatcher1->GetNumManifolds();
	ContactSolverInfo infoGlobal;
	infoGlobal.m_timeStep = timeStep;
	IDebugDraw* debugDrawer=0;
	m_constraintSolver->SolveGroup(manifoldPtr, numManifolds,infoGlobal,debugDrawer);

	///integrate transforms
	integrateTransforms(timeStep);
		
	updateAabbs();

}



void	btSimpleDynamicsWorld::updateAabbs()
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

void	btSimpleDynamicsWorld::integrateTransforms(float timeStep)
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



void	btSimpleDynamicsWorld::predictUnconstraintMotion(float timeStep)
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
}=======

#include "btSimpleDynamicsWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"


btSimpleDynamicsWorld::btSimpleDynamicsWorld()
:btDynamicsWorld(new	btCollisionDispatcher(),new btSimpleBroadphase()),
m_constraintSolver(new btSequentialImpulseConstraintSolver)
{

}

btSimpleDynamicsWorld::btSimpleDynamicsWorld(btDispatcher* dispatcher,btOverlappingPairCache* pairCache,btConstraintSolver* constraintSolver)
:btDynamicsWorld(dispatcher,pairCache),
m_constraintSolver(constraintSolver)
{

}


btSimpleDynamicsWorld::~btSimpleDynamicsWorld()
{
	delete m_constraintSolver;

	//delete the dispatcher and paircache
	delete m_dispatcher1;
	m_dispatcher1 = 0;
	delete m_pairCache;
	m_pairCache = 0;
}

void	btSimpleDynamicsWorld::stepSimulation(float timeStep)
{
	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	///perform collision detection
	PerformDiscreteCollisionDetection();

	///solve contact constraints
	btPersistentManifold** manifoldPtr = ((btCollisionDispatcher*)m_dispatcher1)->getInternalManifoldPointer();
	int numManifolds = m_dispatcher1->GetNumManifolds();
	btContactSolverInfo infoGlobal;
	infoGlobal.m_timeStep = timeStep;
	btIDebugDraw* debugDrawer=0;
	m_constraintSolver->SolveGroup(manifoldPtr, numManifolds,infoGlobal,debugDrawer);

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
		if (colObj->m_internalOwner)
		{
			btRigidBody* body = (btRigidBody*)colObj->m_internalOwner;
			if (body->IsActive() && (!body->IsStatic()))
			{
				btPoint3 minAabb,maxAabb;
				colObj->m_collisionShape->GetAabb(colObj->m_worldTransform, minAabb,maxAabb);
				btSimpleBroadphase* bp = (btSimpleBroadphase*)m_pairCache;
				bp->SetAabb(body->m_broadphaseHandle,minAabb,maxAabb);
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



void	btSimpleDynamicsWorld::predictUnconstraintMotion(float timeStep)
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
