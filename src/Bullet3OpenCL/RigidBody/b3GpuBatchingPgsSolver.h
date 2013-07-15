
#ifndef B3_GPU_BATCHING_PGS_SOLVER_H
#define B3_GPU_BATCHING_PGS_SOLVER_H

#include "Bullet3OpenCL/Initialize/b3OpenCLInclude.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RigidBodyCL.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Contact4.h"
#include "b3GpuConstraint4.h"

class b3GpuBatchingPgsSolver
{
protected:

	int m_debugOutput;

	struct b3GpuBatchingPgsSolverInternalData*		m_data;

	void batchContacts( b3OpenCLArray<b3Contact4>* contacts, int nContacts, b3OpenCLArray<unsigned int>* n, b3OpenCLArray<unsigned int>* offsets, int staticIdx );
	
	inline int sortConstraintByBatch( b3Contact4* cs, int n, int simdWidth , int staticIdx, int numBodies);
	inline int sortConstraintByBatch2( b3Contact4* cs, int n, int simdWidth , int staticIdx, int numBodies);
	inline int sortConstraintByBatch3( b3Contact4* cs, int n, int simdWidth , int staticIdx, int numBodies);
	


	void solveContactConstraint(  const b3OpenCLArray<b3RigidBodyCL>* bodyBuf, const b3OpenCLArray<b3InertiaCL>* shapeBuf, 
			b3OpenCLArray<b3GpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches, int numIterations);

public:
	
	b3GpuBatchingPgsSolver(cl_context ctx,cl_device_id device, cl_command_queue  q,int pairCapacity);
	virtual ~b3GpuBatchingPgsSolver();

	void solveContacts(int numBodies, cl_mem bodyBuf, cl_mem inertiaBuf, int numContacts, cl_mem contactBuf, const struct b3Config& config, int static0Index);

};

#endif //B3_GPU_BATCHING_PGS_SOLVER_H

