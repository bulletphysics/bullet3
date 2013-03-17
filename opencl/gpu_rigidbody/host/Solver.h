/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada


#ifndef __ADL_SOLVER_H
#define __ADL_SOLVER_H

#include "../../parallel_primitives/host/btOpenCLArray.h"
#include "../host/btGpuConstraint4.h"
#include "../../gpu_sat/host/btRigidBodyCL.h"
#include "../../gpu_sat/host/btContact4.h"

#include "../host/btGpuConstraint4.h"
#include "../../parallel_primitives/host/btPrefixScanCL.h"
#include "../../parallel_primitives/host/btRadixSort32CL.h"
#include "../../parallel_primitives/host/btBoundSearchCL.h"

#include "../../basic_initialize/btOpenCLUtils.h"


#define BTNEXTMULTIPLEOF(num, alignment) (((num)/(alignment) + (((num)%(alignment)==0)?0:1))*(alignment))

class SolverBase
{
	public:
		

		struct ConstraintData
		{
			ConstraintData(): m_b(0.f), m_appliedRambdaDt(0.f) {}

			btVector3 m_linear; // have to be normalized
			btVector3 m_angular0;
			btVector3 m_angular1;
			float m_jacCoeffInv;
			float m_b;
			float m_appliedRambdaDt;

			unsigned int m_bodyAPtr;
			unsigned int m_bodyBPtr;

			bool isInvalid() const { return ((unsigned int)m_bodyAPtr+(unsigned int)m_bodyBPtr) == 0; }
			float getFrictionCoeff() const { return m_linear[3]; }
			void setFrictionCoeff(float coeff) { m_linear[3] = coeff; }
		};

		struct ConstraintCfg
		{
			ConstraintCfg( float dt = 0.f ): m_positionDrift( 0.005f ), m_positionConstraintCoeff( 0.2f ), m_dt(dt), m_staticIdx(-1) {}

			float m_positionDrift;
			float m_positionConstraintCoeff;
			float m_dt;
			bool m_enableParallelSolve;
			float m_averageExtent;
			int m_staticIdx;
		};

		

		enum
		{
			N_SPLIT = 16,
			N_BATCHES = 4,
			N_OBJ_PER_SPLIT = 10,
			N_TASKS_PER_BATCH = N_SPLIT*N_SPLIT,
		};
};

class Solver : public SolverBase
{
	public:

		cl_context m_context;
		cl_device_id m_device;
		cl_command_queue m_queue;
				

		btOpenCLArray<unsigned int>* m_numConstraints;
		btOpenCLArray<unsigned int>* m_offsets;
		
		
		int m_nIterations;
		cl_kernel m_batchingKernel;
		cl_kernel m_solveContactKernel;
		cl_kernel m_solveFrictionKernel;
		cl_kernel m_contactToConstraintKernel;
		cl_kernel m_setSortDataKernel;
		cl_kernel m_reorderContactKernel;
		cl_kernel m_copyConstraintKernel;

		class btRadixSort32CL*	m_sort32;
		class btBoundSearchCL*	m_search;
		class btPrefixScanCL*	m_scan;

		btOpenCLArray<btSortData>* m_sortDataBuffer;
		btOpenCLArray<btContact4>* m_contactBuffer;

		enum
		{
			DYNAMIC_CONTACT_ALLOCATION_THRESHOLD = 2000000,
		};

		

		
		Solver(cl_context ctx, cl_device_id device, cl_command_queue queue, int pairCapacity);

		virtual ~Solver();
		
		void solveContactConstraint( const btOpenCLArray<btRigidBodyCL>* bodyBuf, const btOpenCLArray<btInertiaCL>* inertiaBuf, 
			btOpenCLArray<btGpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches);

		void solveContactConstraintHost(  btOpenCLArray<btRigidBodyCL>* bodyBuf, btOpenCLArray<btInertiaCL>* shapeBuf, 
			btOpenCLArray<btGpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches);


		void convertToConstraints( const btOpenCLArray<btRigidBodyCL>* bodyBuf, 
			const btOpenCLArray<btInertiaCL>* shapeBuf, 
			btOpenCLArray<btContact4>* contactsIn, btOpenCLArray<btGpuConstraint4>* contactCOut, void* additionalData, 
			int nContacts, const ConstraintCfg& cfg );

		void batchContacts( btOpenCLArray<btContact4>* contacts, int nContacts, btOpenCLArray<unsigned int>* n, btOpenCLArray<unsigned int>* offsets, int staticIdx );

};




#endif //__ADL_SOLVER_H
