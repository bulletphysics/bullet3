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



#include "btGpuDemoDynamicsWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "LinearMath/btQuickprof.h"
#include "GlutStuff.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"




#define BT_GPU_PREF(func) btCuda_##func

#include "../../src/BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#include "btGpuDemo2dSharedDefs.h"
#undef BT_GPU_PREF

#define BT_GPU_PREF(func) btGpu_##func
#include "btGpuDemo2dSharedDefs.h"
#undef BT_GPU_PREF



btGpuDemoDynamicsWorld* gpCudaDemoDynamicsWorld = NULL;



void btGpuDemoDynamicsWorld::grabNonContactConstraintData()
{
	m_numNonContactConstraints = 0;
	int numNonContactConstraints = getNumConstraints(); 
	for(int i = 0; i < numNonContactConstraints; i++)
	{
		btTypedConstraint* ct = m_constraints[i];
		int ctype = ct->getConstraintType();
		switch(ctype)
		{
			case POINT2POINT_CONSTRAINT_TYPE : 
				grabP2PConstraintData((btPoint2PointConstraint*)ct);
				break;
			default : 
				// warning (not supported) here?
				break;
		}
	}
}



void btGpuDemoDynamicsWorld::grabContactData()
{
	int i;
	btDispatcher* dispatcher = getDispatcher();
	btPersistentManifold** manifoldPtr = dispatcher->getInternalManifoldPointer();
	int numManifolds = dispatcher->getNumManifolds();
	btPersistentManifold* manifold = 0;
	m_totalNumConstraints = 0;
	for(i = 0; i < numManifolds; i++)
	{
		manifold = manifoldPtr[i];
		int numPoints = manifold->getNumContacts();
		if(!numPoints)
		{
			continue;
		}
		int numActualPoints = 0;
		for(int n = 0; n < numPoints; n++)
		{	
				btManifoldPoint& cp = manifold->getContactPoint(n);
				if (cp.m_distance1<=0)
				{
					numActualPoints++;
				}
		}
		if (!numActualPoints)
			continue;

		btRigidBody *rbA, *rbB;
		rbA = (btRigidBody*)manifold->getBody0();
		rbB = (btRigidBody*)manifold->getBody1();
		int idA = rbA->getCompanionId();
		int idB = rbB->getCompanionId();
		btVector3* pConstrData = (btVector3*)(m_hContact + m_totalNumConstraints * 2 * m_maxVtxPerObj);
		if(idA < idB)
		{
			m_hIds[m_totalNumConstraints].x = idA;
			m_hIds[m_totalNumConstraints].y = idB;

			for(int n = 0; n < numPoints; n++)
			{	
				btManifoldPoint& cp = manifold->getContactPoint(n);
				btVector3 v = cp.getPositionWorldOnA();
				pConstrData[0] = cp.getPositionWorldOnA();
				float dist = cp.getDistance();
				if(dist > 0.f)
				{
					pConstrData[0][3] = -1.f;
				}
				else
				{
					pConstrData[0][3] = -dist;
				}
				pConstrData[1] = cp.m_normalWorldOnB;
				pConstrData[1][3] = 0.f;
				pConstrData += 2;
			}
		}
		else
		{ // should never happen
			btAssert(0);
		}
		for(int n = numPoints; n < m_maxVtxPerObj; n++)
		{
			pConstrData[0][3] = -1.f;
			pConstrData += 2;
		}
		m_totalNumConstraints++;
	}
} 



void btGpuDemoDynamicsWorld::grabP2PConstraintData(btPoint2PointConstraint* ct)
{
	btRigidBody& bodyA = ct->getRigidBodyA();
	btTransform trA = bodyA.getCenterOfMassTransform();
	btVector3 pivotA = trA.getBasis() * ct->getPivotInA();
	btRigidBody& bodyB = ct->getRigidBodyB();
	btTransform trB = bodyB.getCenterOfMassTransform();
	btVector3 pivotB = trB.getBasis() * ct->getPivotInB();
	btVector3 pivotA_W = pivotA + trA.getOrigin();
	btVector3 pivotB_W = pivotB + trB.getOrigin();
	btVector3 delta = pivotB_W - pivotA_W;
	int idA = bodyA.getCompanionId();
	int idB = bodyB.getCompanionId();
	m_hIds[m_totalNumConstraints].x = idA;
	m_hIds[m_totalNumConstraints].y = (idB > 0) ? idB : 0;
	btVector3* pConstrData = (btVector3*)(m_hContact + m_totalNumConstraints * 2 * m_maxVtxPerObj);
	for(int k = 0; k < 2; k++)
	{
		btScalar penetration = delta[k];
		btScalar sign = (penetration < 0) ? btScalar(-1.f) : btScalar(1.f);
		btVector3 normal = btVector3(0., 0., 0.);
		normal[k] = sign;
		penetration *= sign;
		pConstrData[0] = pivotA_W;
		pConstrData[0][3] = penetration;
		pConstrData[1] = normal;
		pConstrData[1][3] = btScalar(1.f);
		pConstrData += 2;
	}
	for(int n = 2; n < m_maxVtxPerObj; n++)
	{
		pConstrData[0][3] = -1.f;
		pConstrData += 2;
	}
	m_totalNumConstraints++;
	m_numNonContactConstraints++;
} 



void btGpuDemoDynamicsWorld::grabData()
{
	BT_PROFILE("grab data");
	m_numObj = getNumCollisionObjects();
	m_hPos[0].x = m_hPos[0].y = m_hPos[0].z = m_hPos[0].w = 0.f;
	m_hRot[0] = 0.f;
	m_hVel[0].x = m_hVel[0].y = m_hVel[0].z = m_hVel[0].w = 0.f;
	m_hAngVel[0] = 0.f;
	for(int i = 0; i < m_numObj; i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* rb = btRigidBody::upcast(colObj);
		btVector3 v;
		v = rb->getCenterOfMassPosition();
		m_hPos[i+1] = *((float4*)&v);
		const btTransform& tr = rb->getCenterOfMassTransform();
		v = tr.getBasis().getColumn(0);
		float rot = btAtan2(v[1], v[0]);
		m_hRot[i+1] = rot;
		v = rb->getLinearVelocity();
		m_hVel[i+1] = *((float4*)&v);
		v = rb->getAngularVelocity();
		m_hAngVel[i+1] = v[2];
		if(m_copyMassDataToGPU)
		{
			m_hInvMass[i+1] = rb->getInvMass();
		}
	}
	if(m_useBulletNarrowphase)
	{
		grabContactData();
	}
	grabNonContactConstraintData();
} 



void btGpuDemoDynamicsWorld::createBatches2()
{
	BT_PROFILE("create batches");
	int sz = m_maxObjs * m_maxNeighbors;
	for(int idx = 0; idx < sz; idx++)
	{
		m_hBatchIds[idx] = -1;
	}
	for(int i = 0; i < m_totalNumConstraints; i++)
	{
		m_hConstraintUsed[i] = 0;
	}
	int curBatchId=0;
	int* pBatchIds = m_hBatchIds;
	for(int stage = 0; stage < m_maxBatches; stage++)
	{
		bool isLast = (stage == m_maxBatches-1);
		for(int j = 0; j < m_numObj + 1; j++)
		{
			m_hConstraintCounter[j] = 0;
		}
		int numInBatch = 0;
		for(int i = 0; i < m_totalNumConstraints; i++)
		{
			if(m_hConstraintUsed[i])
			{
				continue;
			}
			int2 ids = m_hIds[i];
			if(!isLast)
			{
				if((m_hConstraintCounter[ids.x] == 0) && (m_hConstraintCounter[ids.y] == 0))
				{
					m_hConstraintCounter[ids.x]=1;
					m_hConstraintCounter[ids.y]=1;
					pBatchIds[numInBatch]=i;
					numInBatch++;
					m_hConstraintUsed[i] = 1;
				}
			}
			else
			{
				pBatchIds[numInBatch]=i;
				numInBatch++;
				m_hConstraintUsed[i] = 1;
			}
		}
		m_numInBatches[stage] = numInBatch;
		pBatchIds += numInBatch;
	}
} 



void btGpuDemoDynamicsWorld::writebackData()
{
	BT_PROFILE("copy velocity into btRigidBody");
	for(int i = 0; i < m_numObj; i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* rb = btRigidBody::upcast(colObj);
		btVector3 v;
		v = *((btVector3*)(m_hVel + i + 1));
		v[2] = 0.f;
		rb->setLinearVelocity(v);
		v[0] = btScalar(0.f);
		v[1] = btScalar(0.f);
		v[2] = m_hAngVel[i + 1];
		rb->setAngularVelocity(v);
	}
} 



void btGpuDemoDynamicsWorld::copyDataToGPU()
{
	BT_PROFILE("copyDataToGPU");

#ifdef BT_USE_CUDA

	btCuda_copyArrayToDevice(m_dIds, m_hIds, sizeof(int2) * m_totalNumConstraints);
	btCuda_copyArrayToDevice(m_dBatchIds, m_hBatchIds, sizeof(int) * m_totalNumConstraints);

	if(m_numNonContactConstraints)
	{ // non-contact constraints are set up by CPU, so copy data to GPU
		int nonContConstrOffs = (m_totalNumConstraints - m_numNonContactConstraints) * 2 * m_maxVtxPerObj;
		int nonContConstrSize = 2 * m_numNonContactConstraints * m_maxVtxPerObj;
		btCuda_copyArrayToDevice(m_dContact + nonContConstrOffs, m_hContact + nonContConstrOffs, sizeof(float4) * nonContConstrSize);
	}

	if(m_numSimStep & 1)
	{
		m_dcPos = m_dpPos;
		m_dcVel = m_dpVel;
		m_dcRot = m_dpRot;
		m_dcAngVel = m_dpAngVel;
	}
	else
	{
		m_dcPos = m_dPos;
		m_dcVel = m_dVel;
		m_dcRot = m_dRot;
		m_dcAngVel = m_dAngVel;
	}
	btCuda_copyArrayToDevice(m_dcPos, m_hPos, (m_numObj + 1) * sizeof(float4)); 
	btCuda_copyArrayToDevice(m_dcVel, m_hVel, (m_numObj + 1)  * sizeof(float4));
	btCuda_copyArrayToDevice(m_dcRot, m_hRot, (m_numObj + 1)  * sizeof(float)); 
	btCuda_copyArrayToDevice(m_dcAngVel, m_hAngVel, (m_numObj + 1)  * sizeof(float)); 
	if(m_copyShapeDataToGPU)
	{
		btCuda_copyArrayToDevice(m_dShapeBuffer, m_hShapeBuffer, m_firstFreeShapeBufferOffset); 
		btCuda_copyArrayToDevice(m_dShapeIds, m_hShapeIds, (m_numObj + 1) * sizeof(int2)); 
		m_copyShapeDataToGPU = false;
	}
	if(m_copyMassDataToGPU)
	{
		btCuda_copyArrayToDevice(m_dInvMass, m_hInvMass, (m_numObj + 1) * sizeof(float)); 
		m_copyMassDataToGPU = false;
	}
#endif //BT_USE_CUDA

} 



void btGpuDemoDynamicsWorld::setConstraintData(btCudaPartProps& partProps)
{
		BT_PROFILE("set constraint data");
		partProps.m_mass = 1.0f;
		partProps.m_diameter = m_objRad * 2.0f;
		partProps.m_restCoeff = 1.0f;
#ifdef BT_USE_CUDA
		btCuda_clearAccumulationOfLambdaDt(m_dLambdaDtBox, m_totalNumConstraints, m_maxVtxPerObj * 2);
		if(!m_useBulletNarrowphase)
		{
			btCuda_setConstraintData(m_dIds, m_totalNumConstraints - m_numNonContactConstraints, m_numObj + 1, m_dcPos, m_dcRot, m_dShapeBuffer, m_dShapeIds,
									 partProps,	m_dContact);
		}
#endif //BT_USE_CUDA

} 



void btGpuDemoDynamicsWorld::copyDataFromGPU()
{
	BT_PROFILE("copy velocity data from device");
#ifdef BT_USE_CUDA
	btCuda_copyArrayFromDevice(m_hVel, m_dcVel, (m_numObj + 1) * sizeof(float4));
	btCuda_copyArrayFromDevice(m_hAngVel, m_dcAngVel, (m_numObj + 1) * sizeof(float)); 
#endif //BT_USE_CUDA
} 



void btGpuDemoDynamicsWorld::solveConstraints(btContactSolverInfo& solverInfo)
{
	if(m_useCPUSolver)
	{
		solveConstraintsCPU2(solverInfo);
	}
	else
	{
		solveConstraints2(solverInfo);
	}
	m_totalNumConstraints = 0;
} 



void btGpuDemoDynamicsWorld::solveConstraints2(btContactSolverInfo& solverInfo)
{
#ifdef BT_USE_CUDA
	BT_PROFILE("solveConstraints");
	grabData();
	createBatches2();
	copyDataToGPU();

	btCudaPartProps partProps;
	setConstraintData(partProps);

	btCudaBoxProps boxProps;
	boxProps.minX = m_worldMin[0];
	boxProps.maxX = m_worldMax[0];
	boxProps.minY = m_worldMin[1];
	boxProps.maxY = m_worldMax[1];
	{
		BT_PROFILE("btCuda_collisionBatchResolutionBox");
		
		int nIter=getSolverInfo().m_numIterations;
		btDispatcherInfo& dispatchInfo = getDispatchInfo();
		btScalar timeStep = dispatchInfo.m_timeStep;

		for(int i=0;i<nIter;i++)
		{
			btCuda_collisionWithWallBox(m_dcPos, m_dcVel, m_dcRot, m_dcAngVel,m_dShapeBuffer, m_dShapeIds, m_dInvMass,
				partProps, boxProps, m_numObj + 1, timeStep);
			int* pBatchIds = m_dBatchIds;
			for(int iBatch=0;iBatch < m_maxBatches;iBatch++)
			{
				int numConstraints = m_numInBatches[iBatch]; 
				btCuda_collisionBatchResolutionBox( m_dIds, pBatchIds, numConstraints, m_numObj + 1,
													m_dcPos, m_dcVel,
													m_dcRot, m_dcAngVel,
													m_dLambdaDtBox,
													m_dContact, m_dInvMass,
													partProps, iBatch, timeStep);
				pBatchIds += numConstraints;
			}
		}
	}
	copyDataFromGPU();
	writebackData();
	m_numSimStep++;
#endif //BT_USE_CUDA
} 



void btGpuDemoDynamicsWorld::solveConstraintsCPU2(btContactSolverInfo& solverInfo)
{
	BT_PROFILE("solveConstraints");
	grabData();
	createBatches2();
	btCudaPartProps partProps;
	{
		BT_PROFILE("set constraint data CPU");
		
		partProps.m_mass = 1.0f;
		partProps.m_diameter = m_objRad * 2.0f;
		partProps.m_restCoeff = 1.0f;

		btGpu_clearAccumulationOfLambdaDt(m_hLambdaDtBox, m_totalNumConstraints, m_maxVtxPerObj * 2);

		if(!m_useBulletNarrowphase)
		{
			btGpu_setConstraintData(m_hIds, m_totalNumConstraints - m_numNonContactConstraints, m_numObj + 1, m_hPos, m_hRot,m_hShapeBuffer, m_hShapeIds,
									partProps,	m_hContact);
		}
	}

	btCudaBoxProps boxProps;
	boxProps.minX = m_worldMin[0];
	boxProps.maxX = m_worldMax[0];
	boxProps.minY = m_worldMin[1];
	boxProps.maxY = m_worldMax[1];
	
	{
		BT_PROFILE("btCuda_collisionBatchResolutionBox CPU");
		
		int nIter=getSolverInfo().m_numIterations;
		btDispatcherInfo& dispatchInfo = getDispatchInfo();
		btScalar timeStep = dispatchInfo.m_timeStep;

		for(int i=0;i<nIter;i++){
			btGpu_collisionWithWallBox(m_hPos, m_hVel, m_hRot, m_hAngVel,m_hShapeBuffer, m_hShapeIds,
				m_hInvMass, partProps, boxProps, m_numObj + 1, timeStep);
			int* pBatchIds = m_hBatchIds;
			for(int iBatch=0;iBatch < m_maxBatches;iBatch++)
			{
				int numContConstraints = m_numInBatches[iBatch]; 
				if(!numContConstraints)
				{
					break;
				}
				btGpu_collisionBatchResolutionBox( m_hIds, pBatchIds, numContConstraints, m_numObj + 1,
													m_hPos, m_hVel,
													m_hRot, m_hAngVel,
													m_hLambdaDtBox,
													m_hContact,
													m_hInvMass,
													partProps, iBatch, timeStep);
				pBatchIds += numContConstraints;
			}
		}
	}
	writebackData();
	m_numSimStep++;
} 



void btGpuDemoDynamicsWorld::debugDrawConstraints(int selectedBatch, const float* pColorTab)
{
	int* pBatchIds = m_hBatchIds;
	for(int stage = 0; stage < m_maxBatches; stage++)
	{	
		int numConstraints = m_numInBatches[stage]; 
		if(!numConstraints)
		{
			break;
		}
		const float* pCol = pColorTab + stage * 3;
		if(selectedBatch < CUDA_DEMO_DYNAMICS_WORLD_MAX_BATCHES)
		{
			if(stage != selectedBatch)
			{
				pBatchIds += numConstraints;
				continue;
			}
		}
		glColor3f(pCol[0], pCol[1], pCol[2]);
		glBegin(GL_LINES);
		for(int i = 0; i < numConstraints; i++)
		{
			int indx = pBatchIds[i];
			int idA = m_hIds[indx].x - 1;
			int idB = m_hIds[indx].y - 1;
			if((idA > 0) && (idB > 0))
			{
				btCollisionObject* colObjA = m_collisionObjects[idA];
				btCollisionObject* colObjB = m_collisionObjects[idB];
				btVector3 vA = colObjA->getWorldTransform().getOrigin();
				btVector3 vB = colObjB->getWorldTransform().getOrigin();
				glVertex3f(vA[0], vA[1], vA[2]);
				glVertex3f(vB[0], vB[1], vB[2]);
			}
		}
		pBatchIds += numConstraints;
		glEnd();
	}
} 



void btGpuDemoDynamicsWorld::initShapeBuffer(int maxShapeBufferSize)
{
	m_maxShapeBufferSize = maxShapeBufferSize;
	m_firstFreeShapeBufferOffset = 0;
	m_hShapeBuffer = new char[m_maxShapeBufferSize];
	m_hShapeIds = new int2[m_maxObjs];

#ifdef BT_USE_CUDA
	btCuda_allocateArray((void**)&m_dShapeBuffer, m_maxShapeBufferSize);
	btCuda_allocateArray((void**)&m_dShapeIds, sizeof(int) * 2 * m_maxObjs);
#endif //BT_USE_CUDA

	m_copyShapeDataToGPU = true;
} 



void btGpuDemoDynamicsWorld::freeShapeBuffer()
{
	delete [] m_hShapeBuffer;
	delete [] m_hShapeIds;
#ifdef BT_USE_CUDA
	btCuda_freeArray(m_dShapeBuffer);
	btCuda_freeArray(m_dShapeIds);
#endif //BT_USE_CUDA
} 



void btGpuDemoDynamicsWorld::addSphere(btVector3& pos, btScalar rad)
{
	btVector3* pBuf = (btVector3*)(m_hShapeBuffer + m_firstFreeShapeBufferOffset);
	*pBuf = pos;
	pBuf->setW(rad);
	m_firstFreeShapeBufferOffset += sizeof(btVector3);
} 



void btGpuDemoDynamicsWorld::addMultiShereObject(int numSpheres, int objIndex)
{
	m_hShapeIds[objIndex].x = m_firstFreeShapeBufferOffset;
	m_hShapeIds[objIndex].y  = numSpheres;
	return;
} 


