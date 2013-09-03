#ifndef CPU_SOFTBODY_INTERNAL_DATA_H
#define CPU_SOFTBODY_INTERNAL_DATA_H

//#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

#include "Bullet3Common/b3Vector3.h"

struct	CpuSoftBodyDemoInternalData
{
	int m_clothShapeIndex;
	float* m_clothVertices;

	CpuSoftBodyDemoInternalData()
		:	m_clothShapeIndex(-1),
		m_clothVertices(0)
	{
	}
};

#endif//CPU_SOFTBODY_INTERNAL_DATA_H

