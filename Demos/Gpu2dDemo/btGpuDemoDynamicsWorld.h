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



#ifndef BT_CUDA_DEMO_DYNAMICS_WORLD_H
#define BT_CUDA_DEMO_DYNAMICS_WORLD_H



#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"


//#define BT_USE_CUDA 1
// To enable CUDA : 
// 1. Uncomment //#define BT_USE_CUDA 1
// 2. Build and add libbulletcuda (Extras/CUDA) to project
// 3. Add $(CUDA_LIB_PATH) and cudart.lib to linker properties

#ifdef BT_USE_CUDA
//	#include "btCudaDemoPairCache.h"
//	#include <vector_types.h>
	#include "BulletMultiThreaded/btGpuDefines.h"
	#undef BT_GPU_PREF
	#define BT_GPU_PREF(func) btCuda_##func
	#include "BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#else
	#include "BulletMultiThreaded/btGpuDefines.h"
	#include "../../src/BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#endif

#undef BT_GPU_PREF


#include "btGpuDemo2dSharedTypes.h"



#define CUDA_DEMO_DYNAMICS_WORLD_MAX_BATCHES 20

#define CUDA_DEMO_DYNAMICS_WORLD_MAX_OBJS 1024
#define CUDA_DEMO_DYNAMICS_WORLD_MAX_NEIGHBORS 24

#define CUDA_DEMO_DYNAMICS_WORLD_MAX_SPHERES_PER_OBJ 8

class btGpuDemoDynamicsWorld;

extern btGpuDemoDynamicsWorld* gpCudaDemoDynamicsWorld; // to access world members from pair cache

class btGpuDemoDynamicsWorld : public btDiscreteDynamicsWorld
{
protected:
	int						m_maxObjs;
	int						m_maxNeighbors;

	int						m_numObj;
	int						m_numSimStep;
	bool					m_useCPUSolver;
	bool					m_useBulletNarrowphase;

	float4*					m_hPos;
	float*					m_hRot;
	float4*					m_hVel;
	float*					m_hAngVel;

	float*					m_hInvMass;
	float*					m_dInvMass;
	bool					m_copyMassDataToGPU;

#ifdef BT_USE_CUDA
	float4*					m_dPos;
	float*					m_dRot;
	float4*					m_dVel;
	float*					m_dAngVel;
	float4*					m_dpPos;
	float*					m_dpRot;
	float4*					m_dpVel;
	float*					m_dpAngVel;

	float4*					m_dcPos;
	float*					m_dcRot;
	float4*					m_dcVel;
	float*					m_dcAngVel;
#endif //BT_USE_CUDA


	btOverlappingPairCache*	m_pairCache;
	int*					m_hConstraintBuffer;
	int*					m_hConstraintCounter;
	int						m_maxBatches;
	int						m_numBatches;
	int						m_totalNumConstraints;
	int2*					m_hIds;
	int*					m_hBatchIds;
	
	int						m_maxVtxPerObj;

	int2*					m_dIds;
	int*					m_dBatchIds;

	float*					m_dLambdaDtBox;
	float4*					m_dContact; // 8 floats : pos.x, pos.y, pos.z, penetration, norm.x, norm.y, norm.z, reserved

	// ------------- these are only needed for CPU version and for debugging
	float*					m_hLambdaDtBox;
	float4*					m_hContact; // 8 floats : pos.x, pos.y, pos.z, penetration, norm.x, norm.y, norm.z, reserved
	// ------------- 

	btScalar 				m_objRad;
	btVector3				m_worldMin;
	btVector3				m_worldMax;
	
	
	int*					m_hConstraintUsed;

	
	// shape buffer
	int						m_maxShapeBufferSize;
	int						m_firstFreeShapeBufferOffset;
	char*					m_hShapeBuffer;  // (pos.x, pos.y, pos.z, radius)
	char*					m_dShapeBuffer;//pointer in device memory
	int2*					m_hShapeIds;
	int2*					m_dShapeIds;
	bool					m_copyShapeDataToGPU;
	void					initShapeBuffer(int maxShapeBufferSize);
	void					freeShapeBuffer();
	void					sendShapeDataToGpu();

	
	int						m_numNonContactConstraints;
	void					grabNonContactConstraintData();
	void					grabP2PConstraintData(btPoint2PointConstraint* ct);

public:
	int						m_numInBatches[CUDA_DEMO_DYNAMICS_WORLD_MAX_BATCHES];
	void					addSphere(btVector3& pos, btScalar rad);
	void					addMultiShereObject(int numSpheres, int objIndex);


	btGpuDemoDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration,
			int maxObjs = CUDA_DEMO_DYNAMICS_WORLD_MAX_OBJS, int maxNeighbors = CUDA_DEMO_DYNAMICS_WORLD_MAX_NEIGHBORS)
		: btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration)
	{ 
		m_maxObjs = maxObjs;
		m_maxNeighbors = maxNeighbors;
		m_useCPUSolver = false;
		m_pairCache = pairCache->getOverlappingPairCache();
		int sz = m_maxObjs * m_maxNeighbors;
		m_hConstraintBuffer = new int[sz];
		m_hConstraintCounter = new int[m_maxObjs];
		m_maxBatches = CUDA_DEMO_DYNAMICS_WORLD_MAX_BATCHES;
		m_hIds = new int2[sz];
		m_hBatchIds = new int[sz];
		for(int i = 0; i < sz; i++)
		{
			m_hBatchIds[i] = -1;
		}
		m_hPos = new float4[m_maxObjs];
		m_hVel = new float4[m_maxObjs];
		m_hRot = new float[m_maxObjs];
		m_hAngVel = new float[m_maxObjs];

		m_hInvMass = new float[m_maxObjs];

		m_maxVtxPerObj = 8;

#ifdef BT_USE_CUDA
		btCuda_allocateArray((void**)&m_dPos, sizeof(float4) * m_maxObjs);
		btCuda_allocateArray((void**)&m_dRot, sizeof(float) * m_maxObjs);
		btCuda_allocateArray((void**)&m_dVel, sizeof(float4) * m_maxObjs);
		btCuda_allocateArray((void**)&m_dAngVel, sizeof(float) * m_maxObjs);
		btCuda_allocateArray((void**)&m_dpPos, sizeof(float4) * m_maxObjs);
		btCuda_allocateArray((void**)&m_dpRot, sizeof(float) * m_maxObjs);
		btCuda_allocateArray((void**)&m_dpVel, sizeof(float4) * m_maxObjs);
		btCuda_allocateArray((void**)&m_dpAngVel, sizeof(float) * m_maxObjs);

		btCuda_allocateArray((void**)&m_dInvMass, sizeof(float) * m_maxObjs);

		btCuda_allocateArray((void**)&m_dIds, sizeof(int2) * sz);
		btCuda_allocateArray((void**)&m_dBatchIds, sizeof(int) * sz);

		btCuda_allocateArray((void**)&m_dLambdaDtBox, sizeof(float) * sz * m_maxVtxPerObj);
		btCuda_allocateArray((void**)&m_dContact, sizeof(float) * sz * m_maxVtxPerObj * 8);
//		btCuda_allocateArray((void**)&m_dPositionConstraint, sizeof(float) * sz * m_maxVtxPerObj * 2);
//		btCuda_allocateArray((void**)&m_dNormal, sizeof(float3) * sz * m_maxVtxPerObj * 2);
#endif //BT_USE_CUDA

		
		m_hLambdaDtBox = new float[sz * m_maxVtxPerObj];
		m_hContact = new float4[sz * m_maxVtxPerObj * 2];
//		m_hPositionConstraint = new float[sz * m_maxVtxPerObj * 2];
//		m_hNormal = new float3[sz * m_maxVtxPerObj * 2];

		m_numSimStep = 0;

		m_objRad = 1.0f;

		m_hConstraintUsed = new int[sz];


		gpCudaDemoDynamicsWorld = this;
		m_totalNumConstraints = 0;

		initShapeBuffer(m_maxObjs * CUDA_DEMO_DYNAMICS_WORLD_MAX_SPHERES_PER_OBJ * sizeof(float) * 4);

		m_copyMassDataToGPU = true;

	}
	virtual ~btGpuDemoDynamicsWorld()
	{
		delete [] m_hConstraintBuffer;
		delete [] m_hConstraintCounter;
		delete [] m_hIds;
		delete [] m_hBatchIds;
		delete [] m_hPos;
		delete [] m_hRot;
		delete [] m_hVel;
		delete [] m_hAngVel;
		delete [] m_hInvMass;
#ifdef BT_USE_CUDA
		btCuda_freeArray(m_dPos);
		btCuda_freeArray(m_dRot);
		btCuda_freeArray(m_dVel);
		btCuda_freeArray(m_dAngVel);
		btCuda_freeArray(m_dpPos);
		btCuda_freeArray(m_dpRot);
		btCuda_freeArray(m_dpVel);
		btCuda_freeArray(m_dpAngVel);
		btCuda_freeArray(m_dInvMass);

		btCuda_freeArray(m_dIds);
		btCuda_freeArray(m_dBatchIds);
		btCuda_freeArray(m_dLambdaDtBox);
		btCuda_freeArray(m_dContact);
#endif //BT_USE_CUDA

		delete [] m_hLambdaDtBox;
		delete [] m_hContact;
		delete [] m_hConstraintUsed;

		gpCudaDemoDynamicsWorld = NULL;

		freeShapeBuffer();
	}

	virtual void	calculateSimulationIslands()
	{
	}
	virtual void	solveConstraints(btContactSolverInfo& solverInfo);
	void solveConstraints2(btContactSolverInfo& solverInfo);
	void solveConstraintsCPU2(btContactSolverInfo& solverInfo);

	void debugDrawConstraints(int selectedBatch, const float* pColorTab);

	void setObjRad(btScalar rad) { m_objRad = rad; }
	void setWorldMin(const btVector3& worldMin) { m_worldMin = worldMin; }
	void setWorldMax(const btVector3& worldMax) { m_worldMax = worldMax; }

	void grabData();
	void grabContactData();
	void copyDataToGPU();
	void setConstraintData(btCudaPartProps& partProps);
	void copyDataFromGPU();
	void writebackData();
	void setUseCPUSolver(bool useCPU) { m_useCPUSolver = useCPU; }
	void setUseBulletNarrowphase(bool useBulletNarrowphase) {m_useBulletNarrowphase = useBulletNarrowphase; } 

	void createBatches2();

	int2* getIdsPtr() { return m_hIds; }
	void setTotalNumConstraints(int totalNumConstraints) { m_totalNumConstraints = totalNumConstraints; }
	int getTotalNumConstraints() { return m_totalNumConstraints; }

};


#endif //BT_CUDA_DEMO_DYNAMICS_WORLD_H
