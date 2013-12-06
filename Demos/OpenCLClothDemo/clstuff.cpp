/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2008 Advanced Micro Devices

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include "clstuff.h"
#include "gl_win.h"

#include "btOpenCLUtils.h"

#include "LinearMath/btScalar.h"
#include <stdio.h>

cl_context			g_cxMainContext;
cl_device_id		g_cdDevice;
cl_command_queue	g_cqCommandQue;

void initCL( void* glCtx, void* glDC )
{
	int ciErrNum = 0;

#if defined(CL_PLATFORM_MINI_CL)
	cl_device_type deviceType = CL_DEVICE_TYPE_CPU;//or use CL_DEVICE_TYPE_DEBUG to debug MiniCL
#elif defined(CL_PLATFORM_INTEL)
	cl_device_type deviceType = CL_DEVICE_TYPE_CPU;
#elif defined(CL_PLATFORM_AMD)
	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
#elif defined(CL_PLATFORM_NVIDIA)
	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
#else
#ifdef __APPLE__
	cl_device_type deviceType = CL_DEVICE_TYPE_ALL;//GPU;
#else
	cl_device_type deviceType = CL_DEVICE_TYPE_CPU;//CL_DEVICE_TYPE_ALL
#endif//__APPLE__
#endif
	
	g_cxMainContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	
	int numDev = btOpenCLUtils::getNumDevices(g_cxMainContext);
	if (!numDev)
	{
		btAssert(0);
		exit(0);//this is just a demo, exit now
	}

	g_cdDevice = btOpenCLUtils::getDevice(g_cxMainContext,0);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	btOpenCLDeviceInfo clInfo;
	btOpenCLUtils::getDeviceInfo(g_cdDevice,clInfo);
	btOpenCLUtils::printDeviceInfo(g_cdDevice);
	
	// create a command-queue
	g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, g_cdDevice, 0, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}
