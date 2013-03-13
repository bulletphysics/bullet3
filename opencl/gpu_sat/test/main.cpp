/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <stdio.h>
#include "../basic_initialize/btOpenCLUtils.h"
#include "../host/ConvexHullContact.h"

#include "BulletCommon/btVector3.h"
#include "parallel_primitives/host/btFillCL.h"
#include "parallel_primitives/host/btBoundSearchCL.h"
#include "parallel_primitives/host/btRadixSort32CL.h"
#include "parallel_primitives/host/btPrefixScanCL.h"
#include "BulletCommon/CommandLineArgs.h"
#include "../host/ConvexHullContact.h"

#include "BulletCommon/btMinMax.h"
int g_nPassed = 0;
int g_nFailed = 0;
bool g_testFailed = 0;

#define TEST_INIT g_testFailed = 0;
#define TEST_ASSERT(x) if( !(x) ){g_testFailed = 1;}
#define TEST_REPORT(testName) printf("[%s] %s\n",(g_testFailed)?"X":"O", testName); if(g_testFailed) g_nFailed++; else g_nPassed++;
#define NEXTMULTIPLEOF(num, alignment) (((num)/(alignment) + (((num)%(alignment)==0)?0:1))*(alignment))

cl_context g_context=0;
cl_device_id g_device=0;
cl_command_queue g_queue =0;
const char* g_deviceName = 0;

void initCL(int preferredDeviceIndex, int preferredPlatformIndex)
{
	void* glCtx=0;
	void* glDC = 0;
	int ciErrNum = 0;
	//bound search and radix sort only work on GPU right now (assume 32 or 64 width workgroup without barriers)

	cl_device_type deviceType = CL_DEVICE_TYPE_ALL;

	g_context = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0,0,preferredDeviceIndex, preferredPlatformIndex);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	int numDev = btOpenCLUtils::getNumDevices(g_context);
	if (numDev>0)
	{
		btOpenCLDeviceInfo info;
		g_device= btOpenCLUtils::getDevice(g_context,0);
		g_queue = clCreateCommandQueue(g_context, g_device, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
        btOpenCLUtils::printDeviceInfo(g_device);
		btOpenCLUtils::getDeviceInfo(g_device,&info);
		g_deviceName = info.m_deviceName;
	}
}

void exitCL()
{
	clReleaseCommandQueue(g_queue);
	clReleaseContext(g_context);
}


inline void gpuConvexHullContactTest()
{
	TEST_INIT;

	TEST_ASSERT(1);

	GpuSatCollision* sat = new GpuSatCollision(g_context,g_device,g_queue);

	delete sat;

	TEST_REPORT( "gpuConvexHullContactTest" );
}

int main(int argc, char** argv)
{
	int preferredDeviceIndex = -1;	int preferredPlatformIndex = -1;

	CommandLineArgs args(argc, argv);
	args.GetCmdLineArgument("deviceId", preferredDeviceIndex);
	args.GetCmdLineArgument("platformId", preferredPlatformIndex);

	initCL(preferredDeviceIndex,preferredPlatformIndex);

	gpuConvexHullContactTest();

	printf("%d tests passed\n",g_nPassed, g_nFailed);
	if (g_nFailed)
	{
		printf("%d tests failed\n",g_nFailed);
	}
	printf("End, press <enter>\n");

	getchar();

	exitCL();

}

