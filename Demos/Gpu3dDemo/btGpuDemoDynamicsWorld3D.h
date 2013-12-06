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

#ifndef BT_CUDA_DEMO_DYNAMICS_WORLD3D_H
#define BT_CUDA_DEMO_DYNAMICS_WORLD3D_H

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

//#define BT_USE_CUDA 1
// To enable CUDA : 
// 1. Uncomment //#define BT_USE_CUDA 1
// 2. Build and add libbulletcuda (Extras/CUDA) to project
// 3. Add $(CUDA_LIB_PATH) and cudart.lib to linker properties


#ifdef BT_USE_CUDA
	#include "BulletMultiThreaded/btGpuDefines.h"
	#undef BT_GPU_PREF
	#define BT_GPU_PREF(func) btCuda_##func
	#include "BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#else
	#include "BulletMultiThreaded/btGpuDefines.h"
	#include "../../src/BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#endif

#undef BT_GPU_PREF


#if 0 // ###
#include <vector_types.h>
#define BT_GPU_PREF(func) btCuda_##func
#include "../../src/BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#undef BT_GPU_PREF
#endif

#include "btGpuDemo3dSharedTypes.h"

//#define CUDA_DEMO_DYNAMICS_WORLD3D_MAX_BATCHES 20
#define CUDA_DEMO_DYNAMICS_WORLD3D_MAX_BATCHES 15

class btCudaDemoDynamicsWorld3D : public btDiscreteDynamicsWorld
{
protected:
	int						m_maxObj;
	int						m_maxNeihbors;
	int						m_maxConstr;
	int						m_maxPointsPerConstr;

	int						m_numObj;
	int						m_numSimStep;
	bool					m_useCPUSolver;
	bool					m_useSeqImpSolver;
	bool					m_useCudaMotIntegr;
	bool					m_copyIntegrDataToGPU;


#ifdef BT_USE_CUDA
	float4*					m_dTrans;
	float4*					m_dVel;
	float4*					m_dAngVel;
	int2*					m_dIds;
	int*					m_dBatchIds;
	float*					m_dLambdaDtBox;
	float*					m_dPositionConstraint;
	float3*					m_dNormal;
	float3*					m_dContact;
	float*					m_dForceTorqueDamp;
	float*					m_dInvInertiaMass;
#endif

	float4*					m_hTrans;
	float4*					m_hVel;
	float4*					m_hAngVel;
	int*					m_hConstraintBuffer;
	int*					m_hConstraintCounter;
	int						m_maxBatches;
	int						m_numBatches;
	int						m_numConstraints;
	int2*					m_hIds;
	int*					m_hBatchIds;
	
	int						m_maxVtxPerObj;


	// ------------- these are only needed for CPU version and for debugging
	float*					m_hLambdaDtBox;
	float*					m_hPositionConstraint;
	float3*					m_hNormal;
	float3*					m_hContact;
	// ------------- 

	btScalar 				m_objRad;
	btVector3				m_worldMin;
	btVector3				m_worldMax;
	
	//-------------------------------
	int*					m_hConstraintUsed;

	//-------------------------------

	float*					m_hForceTorqueDamp;
	float*					m_hInvInertiaMass;

public:
	int						m_numInBatches[CUDA_DEMO_DYNAMICS_WORLD3D_MAX_BATCHES];


	btCudaDemoDynamicsWorld3D(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration, int maxPointsPerConstr = 4)
		: btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration)
	{ 
		m_useCPUSolver = false;
		m_useSeqImpSolver = false;
		m_useCudaMotIntegr = true;
		m_copyIntegrDataToGPU = true;
		m_maxObj = 32768;
		m_maxNeihbors = 26;
		m_maxConstr = m_maxObj * m_maxNeihbors;
		int sz = m_maxConstr;
		m_hConstraintBuffer = new int[sz];
		m_hConstraintCounter = new int[m_maxObj];
		m_maxBatches = CUDA_DEMO_DYNAMICS_WORLD3D_MAX_BATCHES;
		m_hIds = new int2[sz];
		m_hBatchIds = new int[sz];
		for(int i = 0; i < sz; i++)
		{
			m_hBatchIds[i] = -1;
		}
		m_hTrans = new float4[m_maxObj * 4];
		m_hVel = new float4[m_maxObj];
		m_hAngVel = new float4[m_maxObj];

		m_maxPointsPerConstr = maxPointsPerConstr;

#ifdef BT_USE_CUDA
		btCuda_allocateArray((void**)&m_dTrans, sizeof(float4) * m_maxObj * 4);
		btCuda_allocateArray((void**)&m_dVel, sizeof(float4) * m_maxObj);
		btCuda_allocateArray((void**)&m_dAngVel, sizeof(float4) * m_maxObj);

		btCuda_allocateArray((void**)&m_dIds, sizeof(int2) * sz);
		btCuda_allocateArray((void**)&m_dBatchIds, sizeof(int) * sz);


		btCuda_allocateArray((void**)&m_dLambdaDtBox, sizeof(float) * sz * m_maxPointsPerConstr);
		btCuda_allocateArray((void**)&m_dPositionConstraint, sizeof(float) * sz * m_maxPointsPerConstr);
		btCuda_allocateArray((void**)&m_dNormal, sizeof(float3) * sz * m_maxPointsPerConstr);
		btCuda_allocateArray((void**)&m_dContact, sizeof(float3) * sz * m_maxPointsPerConstr);

		btCuda_allocateArray((void**)&m_dForceTorqueDamp, sizeof(float) * m_maxObj * 4 * 2);
		btCuda_allocateArray((void**)&m_dInvInertiaMass, sizeof(float) * m_maxObj * 4 * 3);
#endif

		m_hLambdaDtBox = new float[sz * m_maxPointsPerConstr];
		m_hPositionConstraint = new float[sz * m_maxPointsPerConstr];
		m_hNormal = new float3[sz * m_maxPointsPerConstr];
		m_hContact = new float3[sz * m_maxPointsPerConstr];

		m_numSimStep = 0;

		m_objRad = 1.0f;

		m_hConstraintUsed = new int[sz];

		m_hForceTorqueDamp = new float[m_maxObj * 4 * 2];
		m_hInvInertiaMass = new float[4 * m_maxObj * 3];

	}
	virtual ~btCudaDemoDynamicsWorld3D()
	{
		delete [] m_hConstraintBuffer;
		delete [] m_hConstraintCounter;
		delete [] m_hIds;
		delete [] m_hBatchIds;
		delete [] m_hTrans;
		delete [] m_hVel;
		delete [] m_hAngVel;

		delete [] m_hLambdaDtBox;
		delete [] m_hPositionConstraint;
		delete [] m_hNormal;
		delete [] m_hContact;
		delete [] m_hConstraintUsed;

		delete [] m_hForceTorqueDamp;
		delete [] m_hInvInertiaMass;


#ifdef BT_USE_CUDA
		btCuda_freeArray(m_dTrans);
		btCuda_freeArray(m_dVel);
		btCuda_freeArray(m_dAngVel);

		btCuda_freeArray(m_dIds);
		btCuda_freeArray(m_dBatchIds);
		btCuda_freeArray(m_dLambdaDtBox);
		btCuda_freeArray(m_dPositionConstraint);
		btCuda_freeArray(m_dNormal);
		btCuda_freeArray(m_dContact);
		btCuda_freeArray(m_dForceTorqueDamp);
		btCuda_freeArray(m_dInvInertiaMass);
#endif

	}
	virtual void	calculateSimulationIslands()
	{
		if(m_useSeqImpSolver)
		{
			btDiscreteDynamicsWorld::calculateSimulationIslands();
		}
	}
	virtual void	solveConstraints(btContactSolverInfo& solverInfo);

	virtual void	predictUnconstraintMotion(btScalar timeStep);
	virtual void	integrateTransforms(btScalar timeStep);




	void	solveConstraintsCPU(btContactSolverInfo& solverInfo);

	void debugDrawConstraints(int selectedBatch, const float* pColorTab);

	void setObjRad(btScalar rad) { m_objRad = rad; }
	void setWorldMin(const btVector3& worldMin) { m_worldMin = worldMin; }
	void setWorldMax(const btVector3& worldMax) { m_worldMax = worldMax; }

	void grabData();
	void grabObjData();
	void grabConstrData();
	void createBatches();
	void copyDataToGPU();
	void copyDataFromGPU();
	void writebackData();
	void setUseCPUSolver(bool useCPU) { m_useCPUSolver = useCPU; }
	void setUseSeqImpSolver(bool useSeqImpSolver) { m_useSeqImpSolver = useSeqImpSolver; }
	void setUseCudaMotIntegr(bool useCudaMotIntegr) { m_useCudaMotIntegr = useCudaMotIntegr; }
	void resetScene(void) { m_copyIntegrDataToGPU = true; }
};


#endif //BT_CUDA_DEMO_DYNAMICS_WORLD3D_H
