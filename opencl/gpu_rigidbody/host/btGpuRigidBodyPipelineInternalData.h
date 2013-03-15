#ifndef BT_GPU_RIGIDBODY_PIPELINE_INTERNAL_DATA_H
#define BT_GPU_RIGIDBODY_PIPELINE_INTERNAL_DATA_H

#include "../../basic_initialize/btOpenCLInclude.h"
#include "BulletCommon/btAlignedObjectArray.h"

#include "../../parallel_primitives/host/btOpenCLArray.h"
#include "../../gpu_sat/host/btCollidable.h"

struct btGpuRigidBodyPipelineInternalData
{

	cl_context			m_context;
	cl_device_id		m_device;
	cl_command_queue	m_queue;

	cl_kernel	m_integrateTransformsKernel;

	class btGpuSapBroadphase* m_broadphaseSap;

	class btGpuNarrowPhase*	m_narrowphase;
	
};

#endif //BT_GPU_RIGIDBODY_PIPELINE_INTERNAL_DATA_H

