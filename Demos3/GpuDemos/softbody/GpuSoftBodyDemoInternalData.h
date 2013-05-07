#ifndef GPU_SOFTBODY_INTERNAL_DATA_H
#define GPU_SOFTBODY_INTERNAL_DATA_H

#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h"
#include "Bullet3Common/b3Vector3.h"

struct	GpuSoftBodyDemoInternalData
{
	
	cl_kernel	m_copyTransformsToVBOKernel;

	b3OpenCLArray<b3Vector4>*	m_instancePosOrnColor;

	class b3GpuRigidBodyPipeline* m_rigidBodyPipeline;

	class b3GpuNarrowPhase* m_np;
	class b3GpuSapBroadphase* m_bp;
	int m_clothShapeIndex;
	float* m_clothVertices;

	GpuSoftBodyDemoInternalData()
		:m_instancePosOrnColor(0),
		m_copyTransformsToVBOKernel(0),	m_rigidBodyPipeline(0),
		m_np(0),
		m_bp(0),
		m_clothShapeIndex(-1),
		m_clothVertices(0)
	{
	}
};

#endif//GPU_SOFTBODY_INTERNAL_DATA_H

