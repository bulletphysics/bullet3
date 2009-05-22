/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//--------------------------------------------------------------------------

#include "btGpuDemoDynamicsWorld3D.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "LinearMath/btQuickprof.h"
#include "GlutStuff.h"

#include <stdio.h>

//--------------------------------------------------------------------------

#define BT_GPU_PREF(func) btCuda_##func
#include "../../src/BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#include "btGpuDemo3dSharedDefs.h"
#undef BT_GPU_PREF

#define BT_GPU_PREF(func) btGpu_##func
#include "btGpuDemo3dSharedDefs.h"
#undef BT_GPU_PREF

//--------------------------------------------------------------------------

#if 0
static void check_vel(btVector3& v, int id, char* tag)
{
	int i;
	for(i = 0; i < 3; i++)
	{
		btScalar a = v[i];
		a = btFabs(a);
		if(a > 1000.f)
		{
			break;
		}
	}
	if(i < 3)
	{
		printf("\nERROR in %s (%4d) : %7.2f %7.2f %7.2f\n", tag, id, v[0], v[1], v[2]);
		v[0] = v[1] = v[2] = 0.f;
	}
}
#endif

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::grabObjData()
{
	int i;
	m_numObj = getNumCollisionObjects();
	for(i = 0; i < m_numObj; i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		colObj->setCompanionId(i);
		btRigidBody* rb = btRigidBody::upcast(colObj);
		btVector3 v;
		if(m_copyIntegrDataToGPU)
		{
			const btTransform& tr = rb->getCenterOfMassTransform();
			v = tr.getBasis().getColumn(0);
			m_hTrans[i * 4 + 0] = *((float4*)&v);
			v = tr.getBasis().getColumn(1);
			m_hTrans[i * 4 + 1] = *((float4*)&v);
			v = tr.getBasis().getColumn(2);
			m_hTrans[i * 4 + 2] = *((float4*)&v);
			v = rb->getCenterOfMassPosition();
			m_hTrans[i * 4 + 3] = *((float4*)&v);
		}
		if(!m_useCudaMotIntegr)
		{ 
			v = rb->getLinearVelocity();
			m_hVel[i] = *((float4*)&v);
			v = rb->getAngularVelocity();
			m_hAngVel[i] = *((float4*)&v);
		}
	}
} // btCudaDemoDynamicsWorld3D::grabObjData()

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::grabConstrData()
{
	int i;
	btDispatcher* dispatcher = getDispatcher();
	btPersistentManifold** manifoldPtr = dispatcher->getInternalManifoldPointer();
	int numManifolds = dispatcher->getNumManifolds();
	btPersistentManifold* manifold = 0;
	m_numConstraints = 0;
	
/*	// paranoia
	for(int j = 0; j < m_numObj; j++)
	{  
		m_hConstraintCounter[j] = 0;
	}
*/
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
		m_hConstraintCounter[idA]++;
		m_hConstraintCounter[idB]++;
		if(idA < idB)
		{
			m_hIds[m_numConstraints].x = idA;
			m_hIds[m_numConstraints].y = idB;

			for(int n = 0; n < numPoints; n++)
			{	
				btManifoldPoint& cp = manifold->getContactPoint(n);
				btVector3 v = cp.getPositionWorldOnA();
				m_hContact[m_numConstraints * m_maxPointsPerConstr + n] = *((float3*)&v);
				v = cp.m_normalWorldOnB;
				m_hNormal[m_numConstraints * m_maxPointsPerConstr + n] = *((float3*)&v);
				float dist = cp.getDistance();
				if(dist > 0.f)
				{
					dist = 0.f;
				}
				m_hPositionConstraint[m_numConstraints * m_maxPointsPerConstr + n] = -dist;
			}
		}
		else
		{ // should never happen
			btAssert(0);
		}
		for(int n = numPoints; n < m_maxPointsPerConstr; n++)
		{
			m_hPositionConstraint[m_numConstraints * m_maxPointsPerConstr + n] = 0.f;
		}
		m_numConstraints++;
	}
/*
	// paranoia
	for(int j = 0; j < m_numObj; j++)
	{  
		if(m_hConstraintCounter[j] > m_maxNeihbors)
		{
			printf("WARN : constraint connter is %d for object %d\n", m_hConstraintCounter[j], j);
		}
	}
*/
} // btCudaDemoDynamicsWorld3D::grabConstrData()

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::grabData()
{
	BT_PROFILE("grab data from rigidbody and manifold");
	grabObjData();
	// constraints
	grabConstrData();
} // btCudaDemoDynamicsWorld3D::grabGata()

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::createBatches()
{
	BT_PROFILE("create batches");
	int sz = m_numConstraints;
	for(int i = 0; i < m_numConstraints; i++)
	{
		m_hBatchIds[i] = -1;
		m_hConstraintUsed[i] = 0;
	}
	int curBatchId=0;
	int* pBatchIds = m_hBatchIds;
	int stage;
	for(stage = 0; stage < m_maxBatches; stage++)
	{ // don't print junk on demo screen :-)
		m_numInBatches[stage] = 0;
	}
	for(stage = 0; stage < m_maxBatches; stage++)
	{
		bool isLast = (stage == m_maxBatches-1);
		for(int j = 0; j < m_numObj; j++)
		{
			m_hConstraintCounter[j] = 0;
		}
		int numInBatch = 0;
		for(int i = 0; i < m_numConstraints; i++)
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
		if(!numInBatch) break;
	}
} // btCudaDemoDynamicsWorld3D::createBatches()

//--------------------------------------------------------------------------


void btCudaDemoDynamicsWorld3D::writebackData()
{
	BT_PROFILE("copy velocity into btRigidBody");
	for(int i = 0; i < m_numObj; i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* rb = btRigidBody::upcast(colObj);
		btVector3 v;
		v = *((btVector3*)(m_hVel + i));
		rb->setLinearVelocity(v);
		v = *((btVector3*)(m_hAngVel + i));
		rb->setAngularVelocity(v);
	}
} // btCudaDemoDynamicsWorld3D::writebackData()

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::copyDataToGPU()
{
	BT_PROFILE("copyDataToGPU");
#ifdef BT_USE_CUDA
	btCuda_copyArrayToDevice(m_dIds, m_hIds, sizeof(int2) * m_numConstraints);
	btCuda_copyArrayToDevice(m_dBatchIds, m_hBatchIds, sizeof(int) * m_numConstraints);
	btCuda_copyArrayToDevice(m_dContact, m_hContact, m_numConstraints * m_maxPointsPerConstr * sizeof(float3)); 
	btCuda_copyArrayToDevice(m_dNormal, m_hNormal, m_numConstraints * m_maxPointsPerConstr * sizeof(float3)); 
	btCuda_copyArrayToDevice(m_dPositionConstraint, m_hPositionConstraint, m_numConstraints * m_maxPointsPerConstr * sizeof(float)); 

	if(m_copyIntegrDataToGPU)
	{
		btCuda_copyArrayToDevice(m_dTrans, m_hTrans, m_numObj * sizeof(float4) * 4); 
		if(m_useCudaMotIntegr)
		{
			m_copyIntegrDataToGPU = false;
		}
	}

	if(!m_useCudaMotIntegr)
	{ 
		btCuda_copyArrayToDevice(m_dVel, m_hVel, m_numObj * sizeof(float4));
		btCuda_copyArrayToDevice(m_dAngVel, m_hAngVel, m_numObj * sizeof(float4)); 
	}
#endif
} // btCudaDemoDynamicsWorld3D::copyDataToGPU()

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::copyDataFromGPU()
{
	BT_PROFILE("copy velocity data from device");
#ifdef BT_USE_CUDA
	btCuda_copyArrayFromDevice(m_hVel, m_dVel, m_numObj * sizeof(float4));
	btCuda_copyArrayFromDevice(m_hAngVel, m_dAngVel, m_numObj * sizeof(float4)); 
#endif
} // btCudaDemoDynamicsWorld3D::copyDataFromGPU()

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::solveConstraints(btContactSolverInfo& solverInfo)
{
	if(m_useSeqImpSolver)
	{
		btDiscreteDynamicsWorld::solveConstraints(solverInfo);
		return;
	}
	if(m_useCPUSolver)
	{
		solveConstraintsCPU(solverInfo);
		return;
	}
#ifdef BT_USE_CUDA
	BT_PROFILE("solveConstraints");
	grabData();
	createBatches();
	copyDataToGPU();

	btCudaPartProps partProps;
	partProps.m_mass = 1.0f;
	partProps.m_diameter = m_objRad * 2.0f;
	partProps.m_restCoeff = 1.0f;

	btCudaBoxProps boxProps;
	boxProps.minX = m_worldMin[0];
	boxProps.maxX = m_worldMax[0];
	boxProps.minY = m_worldMin[1];
	boxProps.maxY = m_worldMax[1];
	boxProps.minZ = m_worldMin[2];
	boxProps.maxZ = m_worldMax[2];
	{
		BT_PROFILE("btCuda_collisionBatchResolutionBox");
		
		int nIter=getSolverInfo().m_numIterations;
		btDispatcherInfo& dispatchInfo = getDispatchInfo();
		btScalar timeStep = dispatchInfo.m_timeStep;

		btCuda_clearAccumulationOfLambdaDt(m_dLambdaDtBox, m_numConstraints, m_maxPointsPerConstr);

		for(int i=0;i<nIter;i++)
		{
			btCuda_collisionWithWallBox3D(m_dTrans, m_dVel, m_dAngVel, partProps, boxProps, m_numObj, timeStep);
			int* pBatchIds = m_dBatchIds;
			int* pppBatchIds = m_hBatchIds;
			for(int iBatch=0;iBatch < m_maxBatches;iBatch++)
			{
				int numConstraints = m_numInBatches[iBatch]; 
				if(!numConstraints)
				{
					break;
				}
				btCuda_collisionBatchResolutionBox3D( m_dIds, pBatchIds, numConstraints,
													m_dTrans, m_dVel,
													m_dAngVel,
													m_dLambdaDtBox,
													m_dPositionConstraint,
													m_dNormal,
													m_dContact,
													partProps, iBatch, timeStep);
				pBatchIds += numConstraints;
				pppBatchIds += numConstraints;
			}
		}
	}
	copyDataFromGPU();
	writebackData();
#endif // BT_USE_CUDA
	m_numSimStep++;
} // btCudaDemoDynamicsWorld3D::solveConstraints()

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::solveConstraintsCPU(btContactSolverInfo& solverInfo)
{
	BT_PROFILE("solveConstraints");
	grabData();
	createBatches();

	btCudaPartProps partProps;
	partProps.m_mass = 1.0f;
	partProps.m_diameter = m_objRad * 2.0f;
	partProps.m_restCoeff = 1.0f;

	btCudaBoxProps boxProps;
	boxProps.minX = m_worldMin[0];
	boxProps.maxX = m_worldMax[0];
	boxProps.minY = m_worldMin[1];
	boxProps.maxY = m_worldMax[1];
	boxProps.minZ = m_worldMin[2];
	boxProps.maxZ = m_worldMax[2];
	{
		BT_PROFILE("btCuda_collisionBatchResolutionBox");
		
		int nIter=getSolverInfo().m_numIterations;
		btDispatcherInfo& dispatchInfo = getDispatchInfo();
		btScalar timeStep = dispatchInfo.m_timeStep;

		btGpu_clearAccumulationOfLambdaDt(m_hLambdaDtBox, m_numConstraints, m_maxPointsPerConstr);

		for(int i=0;i<nIter;i++)
		{

			btGpu_collisionWithWallBox3D(m_hTrans, m_hVel, m_hAngVel, partProps, boxProps, m_numObj, timeStep);

			int* pBatchIds = m_hBatchIds;
			for(int iBatch=0;iBatch < m_maxBatches;iBatch++)
			{
				int numConstraints = m_numInBatches[iBatch]; 
				if(!numConstraints) 
				{
					break;
				}
				btGpu_collisionBatchResolutionBox3D( m_hIds, pBatchIds, numConstraints,
													m_hTrans, m_hVel,
													m_hAngVel,
													m_hLambdaDtBox,
													m_hPositionConstraint,
													m_hNormal,
													m_hContact,
													partProps, iBatch, timeStep);
				pBatchIds += numConstraints;
			}
		}
	}
	writebackData();
	m_numSimStep++;
} // btCudaDemoDynamicsWorld3D::solveConstraintsCPU()

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::debugDrawConstraints(int selectedBatch, const float* pColorTab)
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
		if(selectedBatch < m_maxBatches)
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
			int idA = m_hIds[indx].x;
			int idB = m_hIds[indx].y;
			btCollisionObject* colObjA = m_collisionObjects[idA];
			btCollisionObject* colObjB = m_collisionObjects[idB];
			btVector3 vA = colObjA->getWorldTransform().getOrigin();
			btVector3 vB = colObjB->getWorldTransform().getOrigin();
			glVertex3f(vA[0], vA[1], vA[2]);
			glVertex3f(vB[0], vB[1], vB[2]);
		}
		pBatchIds += numConstraints;
		glEnd();
	}
} // btCudaDemoDynamicsWorld3D::debugDrawConstraints()

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::predictUnconstraintMotion(btScalar timeStep)
{
	if(m_useCudaMotIntegr)
	{ 
		BT_PROFILE("motIntegr -- predictUnconstraintMotion");
		int i;
		{
			m_numObj = getNumCollisionObjects();
			float* p_fbuf = m_hForceTorqueDamp;
			float* p_mbuf = m_hInvInertiaMass;
			for(i = 0; i < m_numObj; i++)
			{
				btCollisionObject* colObj = m_collisionObjects[i];
				btRigidBody* rb = btRigidBody::upcast(colObj);
				btVector3* pForce = (btVector3*)p_fbuf;
				*pForce = rb->getTotalForce();
				p_fbuf[3] = rb->getLinearDamping();
				p_fbuf += 4;
				btVector3* pTorque = (btVector3*)p_fbuf;
				*pTorque = rb->getTotalTorque();
				p_fbuf[3] = rb->getAngularDamping();
				p_fbuf += 4;
				if(m_copyIntegrDataToGPU)
				{
					for(int k = 0; k < 3; k++)
					{
						btVector3* pInert = (btVector3*)(p_mbuf + k * 4);
						*pInert = rb->getInvInertiaTensorWorld().getRow(k);
					}
					p_mbuf[3] = rb->getInvMass();
					p_mbuf += 12;
				}
				btVector3 v = rb->getLinearVelocity();
				m_hVel[i] = *((float4*)&v);
				v = rb->getAngularVelocity();
				m_hAngVel[i] = *((float4*)&v);
			}
		}
		if(m_useCPUSolver)
		{
			//BT_PROFILE("motIntegr -- integrate on CPU");
			btGpu_integrVel(m_hForceTorqueDamp, m_hInvInertiaMass, m_hVel, m_hAngVel, timeStep, m_numObj); 
			writebackData();
		}
		else
		{
#ifdef BT_USE_CUDA
			//BT_PROFILE("CUDA motIntegr -- integrate on CUDA");
			btCuda_copyArrayToDevice(m_dForceTorqueDamp, m_hForceTorqueDamp, sizeof(float) * m_numObj * 4 * 2);
			if(m_copyIntegrDataToGPU)
			{
				btCuda_copyArrayToDevice(m_dInvInertiaMass, m_hInvInertiaMass, sizeof(float) * m_numObj * 4 * 3);
			}
			btCuda_copyArrayToDevice(m_dVel, m_hVel, m_numObj * sizeof(float4));
			btCuda_copyArrayToDevice(m_dAngVel, m_hAngVel, m_numObj * sizeof(float4)); 
			btCuda_integrVel(m_dForceTorqueDamp, m_dInvInertiaMass, m_dVel, m_dAngVel, timeStep, m_numObj); 
			copyDataFromGPU();
			writebackData();
#endif
		}
	}
	else
	{
		btDiscreteDynamicsWorld::predictUnconstraintMotion(timeStep);
		m_copyIntegrDataToGPU = true;
	}
} // btCudaDemoDynamicsWorld3D::predictUnconstraintMotion()

//--------------------------------------------------------------------------

void btCudaDemoDynamicsWorld3D::integrateTransforms(btScalar timeStep)
{
	if(m_useCudaMotIntegr)
	{ 
		BT_PROFILE("motIntegr -- integrateTransforms");
		if(m_useCPUSolver)
		{
			btGpu_integrTrans(m_hTrans, m_hVel, m_hAngVel, timeStep, m_numObj);
		}
		else
		{
#ifdef BT_USE_CUDA
			btCuda_integrTrans(m_dTrans, m_dVel, m_dAngVel, timeStep, m_numObj);
			btCuda_copyArrayFromDevice(m_hTrans, m_dTrans, m_numObj * sizeof(float4) * 4); 
#endif
		}
		m_numObj = getNumCollisionObjects();
		for(int i = 0; i < m_numObj; i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];
			btRigidBody* rb = btRigidBody::upcast(colObj);
			btVector3 v;
			btTransform tr;
			const btVector3& v0 = *((btVector3*)&m_hTrans[i * 4 + 0]);
			const btVector3& v1 = *((btVector3*)&m_hTrans[i * 4 + 1]);
			const btVector3& v2 = *((btVector3*)&m_hTrans[i * 4 + 2]);
			const btVector3& v3 = *((btVector3*)&m_hTrans[i * 4 + 3]);
			tr.getBasis().setValue(v0[0], v1[0], v2[0], v0[1], v1[1], v2[1], v0[2], v1[2], v2[2]);
			tr.getOrigin().setValue(v3[0], v3[1], v3[2]);
			rb->proceedToTransform(tr);
		}
	}
	else
	{
		btDiscreteDynamicsWorld::integrateTransforms(timeStep);
	}
} // btCudaDemoDynamicsWorld3D::integrateTransforms()

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
