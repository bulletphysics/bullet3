
#ifndef BT_GPU_BATCHING_PGS_SOLVER_H
#define BT_GPU_BATCHING_PGS_SOLVER_H

#include "../../basic_initialize/b3OpenCLInclude.h"
#include "../../parallel_primitives/host/btOpenCLArray.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RigidBodyCL.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Contact4.h"
#include "b3GpuConstraint4.h"

class b3GpuBatchingPgsSolver
{
protected:

	

	struct btGpuBatchingPgsSolverInternalData*		m_data;

	void batchContacts( btOpenCLArray<b3Contact4>* contacts, int nContacts, btOpenCLArray<unsigned int>* n, btOpenCLArray<unsigned int>* offsets, int staticIdx );
	
	inline int sortConstraintByBatch( b3Contact4* cs, int n, int simdWidth , int staticIdx, int numBodies);
	inline int sortConstraintByBatch2( b3Contact4* cs, int n, int simdWidth , int staticIdx, int numBodies);
	inline int sortConstraintByBatch3( b3Contact4* cs, int n, int simdWidth , int staticIdx, int numBodies);
	


	void solveContactConstraint(  const btOpenCLArray<b3RigidBodyCL>* bodyBuf, const btOpenCLArray<btInertiaCL>* shapeBuf, 
			btOpenCLArray<b3GpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches, int numIterations);

public:
	
	b3GpuBatchingPgsSolver(cl_context ctx,cl_device_id device, cl_command_queue  q,int pairCapacity);
	virtual ~b3GpuBatchingPgsSolver();

	void solveContacts(int numBodies, cl_mem bodyBuf, cl_mem inertiaBuf, int numContacts, cl_mem contactBuf, const struct b3Config& config);

};

#endif //BT_GPU_BATCHING_PGS_SOLVER_H

