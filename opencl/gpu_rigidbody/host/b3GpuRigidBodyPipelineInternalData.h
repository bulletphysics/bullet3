#ifndef BT_GPU_RIGIDBODY_PIPELINE_INTERNAL_DATA_H
#define BT_GPU_RIGIDBODY_PIPELINE_INTERNAL_DATA_H

#include "../../basic_initialize/b3OpenCLInclude.h"
#include "Bullet3Common/b3AlignedObjectArray.h"

#include "../../parallel_primitives/host/btOpenCLArray.h"
#include "../../gpu_narrowphase/host/b3Collidable.h"

#include "gpu_broadphase/host/b3SapAabb.h"
#include "Bullet3Dynamics/ConstraintSolver/b3TypedConstraint.h"



#include "Bullet3Collision/BroadPhaseCollision/b3OverlappingPair.h"

struct b3GpuRigidBodyPipelineInternalData
{

	cl_context			m_context;
	cl_device_id		m_device;
	cl_command_queue	m_queue;

	cl_kernel	m_integrateTransformsKernel;
	cl_kernel	m_updateAabbsKernel;
	
	class b3PgsJacobiSolver* m_solver;
	class b3GpuBatchingPgsSolver* m_solver2;
	class btGpuJacobiSolver* m_solver3;
	
	class b3GpuSapBroadphase* m_broadphaseSap;
	
	class b3DynamicBvhBroadphase* m_broadphaseDbvt;
	btOpenCLArray<b3SapAabb>*	m_allAabbsGPU;
	b3AlignedObjectArray<b3SapAabb>	m_allAabbsCPU;
	btOpenCLArray<btBroadphasePair>*		m_overlappingPairsGPU;

	b3AlignedObjectArray<b3TypedConstraint*> m_joints;
	class b3GpuNarrowPhase*	m_narrowphase;
	
};

#endif //BT_GPU_RIGIDBODY_PIPELINE_INTERNAL_DATA_H

