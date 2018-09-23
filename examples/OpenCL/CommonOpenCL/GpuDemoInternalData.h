#ifndef GPU_DEMO_INTERNAL_DATA_H
#define GPU_DEMO_INTERNAL_DATA_H

#include "Bullet3OpenCL/Initialize/b3OpenCLInclude.h"

struct GpuDemoInternalData
{
	cl_platform_id m_platformId;
	cl_context m_clContext;
	cl_device_id m_clDevice;
	cl_command_queue m_clQueue;
	bool m_clInitialized;
	char* m_clDeviceName;

	GpuDemoInternalData()
		: m_platformId(0),
		  m_clContext(0),
		  m_clDevice(0),
		  m_clQueue(0),
		  m_clInitialized(false),
		  m_clDeviceName(0)
	{
	}
};

#endif
