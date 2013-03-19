#ifndef GPU_RIGIDBODY_INTERNAL_DATA_H
#define GPU_RIGIDBODY_INTERNAL_DATA_H

#include "basic_initialize/btOpenCLUtils.h"
#include "parallel_primitives/host/btOpenCLArray.h"
#include "BulletCommon/btVector3.h"

struct	GpuRigidBodyDemoInternalData
{
	
	cl_kernel	m_copyTransformsToVBOKernel;

	btOpenCLArray<btVector4>*	m_instancePosOrnColor;

	class btGpuRigidBodyPipeline* m_rigidBodyPipeline;

	class btGpuNarrowPhase* m_np;
	class btGpuSapBroadphase* m_bp;

	GpuRigidBodyDemoInternalData()
		:m_instancePosOrnColor(0),
		m_copyTransformsToVBOKernel(0),	m_rigidBodyPipeline(0),
		m_np(0),
		m_bp(0)
	{
	}
};

#endif//GPU_RIGIDBODY_INTERNAL_DATA_H

