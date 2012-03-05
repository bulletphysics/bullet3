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
//Originally written by Erwin Coumans

#include "BasicDemo.h"
#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btHashMap.h"

#ifdef CL_PLATFORM_AMD
#include "../../opencl/basic_initialize/btOpenCLUtils.h"
extern cl_context			g_cxMainContext;
extern cl_command_queue	g_cqCommandQue;
extern cl_device_id		g_clDevice;
#endif


	
int main(int argc,char** argv)
{

	#ifdef CL_PLATFORM_AMD
	int ciErrNum = 0;
	const char* vendorSDK = btOpenCLUtils::getSdkVendorName();
	printf("This program was compiled using the %s OpenCL SDK\n",vendorSDK);

	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;//CPU;//GPU;
	
	
	void* glCtx=0;
	void* glDC = 0;
	g_cxMainContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	int numDev = btOpenCLUtils::getNumDevices(g_cxMainContext);

	if (numDev>0)
	{
		int deviceIndex =0;
		g_clDevice = btOpenCLUtils::getDevice(g_cxMainContext,deviceIndex);
		btOpenCLDeviceInfo clInfo;
		btOpenCLUtils::getDeviceInfo(g_clDevice,clInfo);
		btOpenCLUtils::printDeviceInfo(g_clDevice);
		// create a command-queue
		g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, g_clDevice, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
#endif //#ifdef CL_PLATFORM_AMD


	BasicDemo ccdDemo;
	ccdDemo.initPhysics();
	

#ifdef CHECK_MEMORY_LEAKS
	ccdDemo.exitPhysics();
#else
	glutmain(argc, argv,1024,600,"Bullet Physics Demo. http://bulletphysics.org",&ccdDemo);
#endif
	
	//setupGUI(1024,768);
	glutMainLoop();
	//default glut doesn't return from mainloop
	return 0;
}

