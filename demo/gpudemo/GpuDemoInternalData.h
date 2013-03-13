#ifndef GPU_DEMO_INTERNAL_DATA_H
#define GPU_DEMO_INTERNAL_DATA_H

#include "basic_initialize/btOpenCLInclude.h"

struct GpuDemoInternalData
{
	cl_context m_clContext;
	cl_device_id m_clDevice;
	cl_command_queue m_clQueue;
	bool m_clInitialized;
	char*	m_clDeviceName;

	GpuDemoInternalData()
	:m_clInitialized(false),
	m_clDeviceName(0)
	{
		
	}
};

#endif
