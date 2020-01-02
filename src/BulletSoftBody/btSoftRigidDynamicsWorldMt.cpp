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

#include "btSoftRigidDynamicsWorldMt.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btSerializer.h"
#include "LinearMath/btThreads.h"

//softbody & helpers
#include "btSoftBody.h"
#include "btSoftBodyHelpers.h"
#include "btSoftBodySolvers.h"
#include "btDefaultSoftBodySolver.h"

btSoftRigidDynamicsWorldMt::btSoftRigidDynamicsWorldMt(
	btDispatcher* dispatcher,
	btBroadphaseInterface* pairCache,
	btConstraintSolverPoolMt* solverPool,
	btConstraintSolver* constraintSolverMt,
	btCollisionConfiguration* collisionConfiguration,
	btSoftBodySolver* softBodySolver) : btDiscreteDynamicsWorldMt(dispatcher, pairCache, solverPool, constraintSolverMt, collisionConfiguration),
									m_softBodySolver(softBodySolver),
									m_ownsSolver(false)
{
	if (!m_softBodySolver)
	{
		void* ptr = btAlignedAlloc(sizeof(btDefaultSoftBodySolver), 16);
		m_softBodySolver = new (ptr) btDefaultSoftBodySolver();
		m_ownsSolver = true;
	}

	m_drawFlags = fDrawFlags::Std;
	m_drawNodeTree = true;
	m_drawFaceTree = false;
	m_drawClusterTree = false;
	m_sbi.m_broadphase = pairCache;
	m_sbi.m_dispatcher = dispatcher;
	m_sbi.m_sparsesdf.Initialize();
	m_sbi.m_sparsesdf.Reset();

	m_sbi.air_density = (btScalar)1.2;
	m_sbi.water_density = 0;
	m_sbi.water_offset = 0;
	m_sbi.water_normal = btVector3(0, 0, 0);
	m_sbi.m_gravity.setValue(0, -10, 0);

	m_sbi.m_sparsesdf.Initialize();
}

btSoftRigidDynamicsWorldMt::~btSoftRigidDynamicsWorldMt()
{
	if (m_ownsSolver)
	{
		m_softBodySolver->~btSoftBodySolver();
		btAlignedFree(m_softBodySolver);
	}
}

void btSoftRigidDynamicsWorldMt::predictUnconstraintMotion(btScalar timeStep)
{
	btDiscreteDynamicsWorldMt::predictUnconstraintMotion(timeStep);
	{
		BT_PROFILE("predictUnconstraintMotionSoftBody");
		m_softBodySolver->predictMotion(float(timeStep));
	}
}

struct DefaultCollisionHandler : public btIParallelForBody
{
	btAlignedObjectArray<btSoftBody*>* mSoftBodySet;

	DefaultCollisionHandler(btAlignedObjectArray<btSoftBody*>* softBodySet)
	{
		mSoftBodySet = softBodySet;
	}
	void forLoop(int iBegin, int iEnd) const
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			btSoftBody* psb = (btSoftBody*)(*mSoftBodySet)[i];
			psb->defaultCollisionHandler(psb);
		}
	}
};

void btSoftRigidDynamicsWorldMt::internalSingleStepSimulation(btScalar timeStep)
{
	// Let the solver grab the soft bodies and if necessary optimize for it
	m_softBodySolver->optimize(getSoftBodyArray());

	if (!m_softBodySolver->checkInitialized())
	{
		btAssert("Solver initialization failed\n");
	}

	btDiscreteDynamicsWorldMt::internalSingleStepSimulation(timeStep);

	///solve soft bodies constraints
	solveSoftBodiesConstraints(timeStep);

	//self collisions
	if (m_softBodies.size() > 0)
	{
		DefaultCollisionHandler update(&m_softBodies);
		int grainSize = 2;  // num of iterations per task for task scheduler
		btParallelFor(0, m_softBodies.size(), grainSize, update);
	}

	///update soft bodies
	m_softBodySolver->updateSoftBodies();

	// End solver-wise simulation step
	// ///////////////////////////////
}

struct PrepareClusters : public btIParallelForBody
{
	btAlignedObjectArray<btSoftBody*>* mSoftBodySet;
	int mIterations;

	PrepareClusters(btAlignedObjectArray<btSoftBody*>* softBodySet, int iterations)
	{
		mSoftBodySet = softBodySet;
		mIterations = iterations;
	}
	void forLoop(int iBegin, int iEnd) const
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			btSoftBody* psb = (btSoftBody*)(*mSoftBodySet)[i];
			psb->prepareClusters(mIterations);
		}
	}
};

struct SolveClusters : public btIParallelForBody
{
	btAlignedObjectArray<btSoftBody*>* mSoftBodySet;
	int mIterations;

	SolveClusters(btAlignedObjectArray<btSoftBody*>* softBodySet)
	{
		mSoftBodySet = softBodySet;
	}
	void forLoop(int iBegin, int iEnd) const
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			const btScalar sor = 1;
			btSoftBody* psb = (btSoftBody*)(*mSoftBodySet)[i];
			psb->solveClusters(sor);
		}
	}
};

struct CleanupClusters : public btIParallelForBody
{
	btAlignedObjectArray<btSoftBody*>* mSoftBodySet;

	CleanupClusters(btAlignedObjectArray<btSoftBody*>* softBodySet)
	{
		mSoftBodySet = softBodySet;
	}
	void forLoop(int iBegin, int iEnd) const
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			btSoftBody* psb = (btSoftBody*)(*mSoftBodySet)[i];
			psb->cleanupClusters();
		}
	}
};

void btSoftRigidDynamicsWorldMt::solveClusters()
{
	if (m_softBodies.size() > 0)
	{
		const int nb = m_softBodies.size();
		int iterations = 0;
		int i;

		for (i = 0; i < nb; ++i)
		{
			iterations = btMax(iterations, m_softBodies[i]->m_cfg.citerations);
		}

      {
			PrepareClusters update(&m_softBodies, iterations);
			int grainSize = 2;  // num of iterations per task for task scheduler
			btParallelFor(0, m_softBodies.size(), grainSize, update);
		}

		for (i = 0; i < iterations; ++i)
		{
			SolveClusters update(&m_softBodies);
			int grainSize = 2;  // num of iterations per task for task scheduler
			btParallelFor(0, m_softBodies.size(), grainSize, update);
		}

		{
			CleanupClusters update(&m_softBodies);
			int grainSize = 2;  // num of iterations per task for task scheduler
			btParallelFor(0, m_softBodies.size(), grainSize, update);
		}
	}
}

void btSoftRigidDynamicsWorldMt::solveSoftBodiesConstraints(btScalar timeStep)
{
	BT_PROFILE("solveSoftConstraints");

	solveClusters();

	// Solve constraints solver-wise
	m_softBodySolver->solveConstraints(timeStep * m_softBodySolver->getTimeScale());
}

void btSoftRigidDynamicsWorldMt::addSoftBody(btSoftBody* body, int collisionFilterGroup, int collisionFilterMask)
{
	m_softBodies.push_back(body);

	// Set the soft body solver that will deal with this body
	// to be the world's solver
	body->setSoftBodySolver(m_softBodySolver);

	btCollisionWorld::addCollisionObject(body,
										 collisionFilterGroup,
										 collisionFilterMask);
}

void btSoftRigidDynamicsWorldMt::removeSoftBody(btSoftBody* body)
{
	m_softBodies.remove(body);

	btCollisionWorld::removeCollisionObject(body);
}

void btSoftRigidDynamicsWorldMt::removeCollisionObject(btCollisionObject* collisionObject)
{
	btSoftBody* body = btSoftBody::upcast(collisionObject);
	if (body)
		removeSoftBody(body);
	else
		btDiscreteDynamicsWorldMt::removeCollisionObject(collisionObject);
}

void btSoftRigidDynamicsWorldMt::debugDrawWorld()
{
	btDiscreteDynamicsWorldMt::debugDrawWorld();

	if (getDebugDrawer())
	{
		int i;
		for (i = 0; i < this->m_softBodies.size(); i++)
		{
			btSoftBody* psb = (btSoftBody*)this->m_softBodies[i];
			if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
			{
				btSoftBodyHelpers::DrawFrame(psb, m_debugDrawer);
				btSoftBodyHelpers::Draw(psb, m_debugDrawer, m_drawFlags);
			}

			if (m_debugDrawer && (m_debugDrawer->getDebugMode() & btIDebugDraw::DBG_DrawAabb))
			{
				if (m_drawNodeTree) btSoftBodyHelpers::DrawNodeTree(psb, m_debugDrawer);
				if (m_drawFaceTree) btSoftBodyHelpers::DrawFaceTree(psb, m_debugDrawer);
				if (m_drawClusterTree) btSoftBodyHelpers::DrawClusterTree(psb, m_debugDrawer);
			}
		}
	}
}

struct btSoftSingleRayCallback : public btBroadphaseRayCallback
{
	btVector3 m_rayFromWorld;
	btVector3 m_rayToWorld;
	btTransform m_rayFromTrans;
	btTransform m_rayToTrans;
	btVector3 m_hitNormal;

	const btSoftRigidDynamicsWorldMt* m_world;
	btCollisionWorld::RayResultCallback& m_resultCallback;

	btSoftSingleRayCallback(const btVector3& rayFromWorld, const btVector3& rayToWorld, const btSoftRigidDynamicsWorldMt* world, btCollisionWorld::RayResultCallback& resultCallback)
		: m_rayFromWorld(rayFromWorld),
		  m_rayToWorld(rayToWorld),
		  m_world(world),
		  m_resultCallback(resultCallback)
	{
		m_rayFromTrans.setIdentity();
		m_rayFromTrans.setOrigin(m_rayFromWorld);
		m_rayToTrans.setIdentity();
		m_rayToTrans.setOrigin(m_rayToWorld);

		btVector3 rayDir = (rayToWorld - rayFromWorld);

		rayDir.normalize();
		///what about division by zero? --> just set rayDirection[i] to INF/1e30
		m_rayDirectionInverse[0] = rayDir[0] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(m_rayToWorld - m_rayFromWorld);
	}

	virtual bool process(const btBroadphaseProxy* proxy)
	{
		///terminate further ray tests, once the closestHitFraction reached zero
		if (m_resultCallback.m_closestHitFraction == btScalar(0.f))
			return false;

		btCollisionObject* collisionObject = (btCollisionObject*)proxy->m_clientObject;

		//only perform raycast if filterMask matches
		if (m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
		{
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			//btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
#if 0
#ifdef RECALCULATE_AABB
			btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
			collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
#else
			//getBroadphase()->getAabb(collisionObject->getBroadphaseHandle(),collisionObjectAabbMin,collisionObjectAabbMax);
			const btVector3& collisionObjectAabbMin = collisionObject->getBroadphaseHandle()->m_aabbMin;
			const btVector3& collisionObjectAabbMax = collisionObject->getBroadphaseHandle()->m_aabbMax;
#endif
#endif
			//btScalar hitLambda = m_resultCallback.m_closestHitFraction;
			//culling already done by broadphase
			//if (btRayAabb(m_rayFromWorld,m_rayToWorld,collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,m_hitNormal))
			{
				m_world->rayTestSingle(m_rayFromTrans, m_rayToTrans,
									   collisionObject,
									   collisionObject->getCollisionShape(),
									   collisionObject->getWorldTransform(),
									   m_resultCallback);
			}
		}
		return true;
	}
};

void btSoftRigidDynamicsWorldMt::rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const
{
	BT_PROFILE("rayTest");
	/// use the broadphase to accelerate the search for objects, based on their aabb
	/// and for each object with ray-aabb overlap, perform an exact ray test
	btSoftSingleRayCallback rayCB(rayFromWorld, rayToWorld, this, resultCallback);

#ifndef USE_BRUTEFORCE_RAYBROADPHASE
	m_broadphasePairCache->rayTest(rayFromWorld, rayToWorld, rayCB);
#else
	for (int i = 0; i < this->getNumCollisionObjects(); i++)
	{
		rayCB.process(m_collisionObjects[i]->getBroadphaseHandle());
	}
#endif  //USE_BRUTEFORCE_RAYBROADPHASE
}

void btSoftRigidDynamicsWorldMt::rayTestSingle(const btTransform& rayFromTrans, const btTransform& rayToTrans,
											 btCollisionObject* collisionObject,
											 const btCollisionShape* collisionShape,
											 const btTransform& colObjWorldTransform,
											 RayResultCallback& resultCallback)
{
	if (collisionShape->isSoftBody())
	{
		btSoftBody* softBody = btSoftBody::upcast(collisionObject);
		if (softBody)
		{
			btSoftBody::sRayCast softResult;
			if (softBody->rayTest(rayFromTrans.getOrigin(), rayToTrans.getOrigin(), softResult))
			{
				if (softResult.fraction <= resultCallback.m_closestHitFraction)
				{
					btCollisionWorld::LocalShapeInfo shapeInfo;
					shapeInfo.m_shapePart = 0;
					shapeInfo.m_triangleIndex = softResult.index;
					// get the normal
					btVector3 rayDir = rayToTrans.getOrigin() - rayFromTrans.getOrigin();
					btVector3 normal = -rayDir;
					normal.normalize();

					if (softResult.feature == btSoftBody::eFeature::Face)
					{
						normal = softBody->m_faces[softResult.index].m_normal;
						if (normal.dot(rayDir) > 0)
						{
							// normal always point toward origin of the ray
							normal = -normal;
						}
					}

					btCollisionWorld::LocalRayResult rayResult(collisionObject,
															   &shapeInfo,
															   normal,
															   softResult.fraction);
					bool normalInWorldSpace = true;
					resultCallback.addSingleResult(rayResult, normalInWorldSpace);
				}
			}
		}
	}
	else
	{
		btCollisionWorld::rayTestSingle(rayFromTrans, rayToTrans, collisionObject, collisionShape, colObjWorldTransform, resultCallback);
	}
}

void btSoftRigidDynamicsWorldMt::serializeSoftBodies(btSerializer* serializer)
{
	int i;
	//serialize all collision objects
	for (i = 0; i < m_collisionObjects.size(); i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->getInternalType() & btCollisionObject::CO_SOFT_BODY)
		{
			int len = colObj->calculateSerializeBufferSize();
			btChunk* chunk = serializer->allocate(len, 1);
			const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk, structType, BT_SOFTBODY_CODE, colObj);
		}
	}
}

void btSoftRigidDynamicsWorldMt::serialize(btSerializer* serializer)
{
	serializer->startSerialization();

	serializeDynamicsWorldInfo(serializer);

	serializeSoftBodies(serializer);

	serializeRigidBodies(serializer);

	serializeCollisionObjects(serializer);

	serializer->finishSerialization();
}
