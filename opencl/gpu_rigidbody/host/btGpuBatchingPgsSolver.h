
#ifndef BT_GPU_BATCHING_PGS_SOLVER_H
#define BT_GPU_BATCHING_PGS_SOLVER_H

#include "../../basic_initialize/btOpenCLInclude.h"
#include "../../parallel_primitives/host/btOpenCLArray.h"
#include "../../gpu_narrowphase/host/btRigidBodyCL.h"
#include "../../gpu_narrowphase/host/btContact4.h"
#include "btGpuConstraint4.h"

class btGpuBatchingPgsSolver
{
protected:

	

	struct btGpuBatchingPgsSolverInternalData*		m_data;

	void batchContacts( btOpenCLArray<btContact4>* contacts, int nContacts, btOpenCLArray<unsigned int>* n, btOpenCLArray<unsigned int>* offsets, int staticIdx );
	
	inline int sortConstraintByBatch( btContact4* cs, int n, int simdWidth , int staticIdx, int numBodies);
	inline int sortConstraintByBatch2( btContact4* cs, int n, int simdWidth , int staticIdx, int numBodies);
	inline int sortConstraintByBatch3( btContact4* cs, int n, int simdWidth , int staticIdx, int numBodies);
	


	void solveContactConstraint(  const btOpenCLArray<btRigidBodyCL>* bodyBuf, const btOpenCLArray<btInertiaCL>* shapeBuf, 
			btOpenCLArray<btGpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches, int numIterations);

public:
	
	btGpuBatchingPgsSolver(cl_context ctx,cl_device_id device, cl_command_queue  q,int pairCapacity);
	virtual ~btGpuBatchingPgsSolver();

	void solveContacts(int numBodies, cl_mem bodyBuf, cl_mem inertiaBuf, int numContacts, cl_mem contactBuf, const struct btConfig& config);

};

#endif //BT_GPU_BATCHING_PGS_SOLVER_H

