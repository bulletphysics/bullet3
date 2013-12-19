/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BulletXmlImportDemo.h"
#include "GlutStuff.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btHashMap.h"


#ifdef USE_AMD_OPENCL



#include "btOpenCLUtils.h"

#include <LinearMath/btScalar.h>

cl_context        g_cxMainContext;
cl_device_id      g_cdDevice;
cl_command_queue  g_cqCommandQue;


// Returns true if OpenCL is initialized properly, false otherwise.
bool initCL( void* glCtx, void* glDC )
{
	const char* vendorSDK = btOpenCLUtils::getSdkVendorName();
	printf("This program was compiled using the %s OpenCL SDK\n",vendorSDK);

    int ciErrNum = 0;

#ifdef BT_USE_CLEW
    ciErrNum = clewInit( "OpenCL.dll" );
    if ( ciErrNum != CLEW_SUCCESS ) {
        return false;
    }
#endif

#if defined(CL_PLATFORM_MINI_CL)
    cl_device_type deviceType = CL_DEVICE_TYPE_CPU;
#elif defined(CL_PLATFORM_AMD)
    cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
#elif defined(CL_PLATFORM_NVIDIA)
    cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
#else
    cl_device_type deviceType = CL_DEVICE_TYPE_CPU;
#endif

	g_cxMainContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

	int numDev = btOpenCLUtils::getNumDevices(g_cxMainContext);
	if (!numDev)
		return false;

    g_cdDevice =  btOpenCLUtils::getDevice(g_cxMainContext,0);
    
    btOpenCLDeviceInfo clInfo;
	btOpenCLUtils::getDeviceInfo(g_cdDevice,clInfo);
	btOpenCLUtils::printDeviceInfo(g_cdDevice);

    // create a command-queue
    g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, g_cdDevice, 0, &ciErrNum);
    oclCHECKERROR(ciErrNum, CL_SUCCESS);

    return true;
}

#endif //#ifdef USE_AMD_OPENCL

	
int main(int argc,char** argv)
{
	
#ifdef USE_AMD_OPENCL
	
	bool initialized = initCL(0,0);
	btAssert(initialized);
#endif //USE_AMD_OPENCL

	
	BulletXmlImportDemo serializeDemo;
	serializeDemo.initPhysics();
	

#ifdef CHECK_MEMORY_LEAKS
	serializeDemo.exitPhysics();
#else
	return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bulletphysics.org",&serializeDemo);
#endif
	
	//default glut doesn't return from mainloop
	return 0;
}

