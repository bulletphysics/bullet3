/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///original author: Erwin Coumans

#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h"

#include <stdio.h>

cl_context			g_cxMainContext;
cl_command_queue	g_cqCommandQue;


#include "Bullet3Common/b3Logging.h"


void myerrorwarningprintf(const char* msg)
{
	//OutputDebugStringA(msg);
	//printf(msg);
}

void myprintf(const char* msg)
{
	//OutputDebugStringA(msg);
	printf(msg);
}

int main(int argc, char* argv[])
{
	b3SetCustomPrintfFunc(myprintf);
	b3SetCustomWarningMessageFunc(myerrorwarningprintf);
	b3SetCustomErrorMessageFunc(myerrorwarningprintf);

	b3Printf("test b3Printf\n");
	b3Warning("test warning\n");
	b3Error("test error\n");

	int ciErrNum = 0;

	cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
	const char* vendorSDK = b3OpenCLUtils::getSdkVendorName();

	b3Printf("This program was compiled using the %s OpenCL SDK\n",vendorSDK);
	int numPlatforms = b3OpenCLUtils::getNumPlatforms();
	b3Printf("Num Platforms = %d\n", numPlatforms);

	for (int i=0;i<numPlatforms;i++)
	{
		cl_platform_id platform = b3OpenCLUtils::getPlatform(i);
		b3OpenCLPlatformInfo platformInfo;
		b3OpenCLUtils::getPlatformInfo(platform,&platformInfo);
		b3Printf("--------------------------------\n");
		b3Printf("Platform info for platform nr %d:\n",i);
		b3Printf("  CL_PLATFORM_VENDOR: \t\t\t%s\n",platformInfo.m_platformVendor);
		b3Printf("  CL_PLATFORM_NAME: \t\t\t%s\n",platformInfo.m_platformName);
		b3Printf("  CL_PLATFORM_VERSION: \t\t\t%s\n",platformInfo.m_platformVersion);

		g_cxMainContext = b3OpenCLUtils::createContextFromPlatform(platform,deviceType,&ciErrNum);

		int numDevices = b3OpenCLUtils::getNumDevices(g_cxMainContext);
		b3Printf("Num Devices = %d\n", numDevices);
		for (int j=0;j<numDevices;j++)
		{
			cl_device_id device = b3OpenCLUtils::getDevice(g_cxMainContext,j);
			b3OpenCLDeviceInfo devInfo;
			b3OpenCLUtils::getDeviceInfo(device,&devInfo);
			b3OpenCLUtils::printDeviceInfo(device);
			g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, device, 0, &ciErrNum);

			b3OpenCLArray<char> memTester(g_cxMainContext,g_cqCommandQue,0,true);
			int maxMem = 0;
			bool result=true;
			for (size_t i=1;result;i++)
			{
				size_t numBytes = i*1024*1024;
				result = memTester.resize(numBytes,false);

				if (result)
				{
					maxMem = numBytes;
					
				} else
				{
					break;
				}
			}
			printf("allocated %d MB successfully\n",maxMem/(1024*1024));
			clReleaseCommandQueue(g_cqCommandQue);
			g_cqCommandQue=0;

		}

		clReleaseContext(g_cxMainContext);
		g_cxMainContext=0;
	}

	///Easier method to initialize OpenCL using createContextFromType for a GPU
	deviceType = CL_DEVICE_TYPE_GPU;

	void* glCtx=0;
	void* glDC = 0;
	b3Printf("Initialize OpenCL using b3OpenCLUtils::createContextFromType for CL_DEVICE_TYPE_GPU\n");
	g_cxMainContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	if (g_cxMainContext)
	{
		int numDev = b3OpenCLUtils::getNumDevices(g_cxMainContext);

		for (int i=0;i<numDev;i++)
		{
			cl_device_id		device;
			device = b3OpenCLUtils::getDevice(g_cxMainContext,i);
			b3OpenCLDeviceInfo clInfo;
			b3OpenCLUtils::getDeviceInfo(device,&clInfo);
			b3OpenCLUtils::printDeviceInfo(device);
			// create a command-queue
			g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, device, 0, &ciErrNum);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
			//normally you would create and execute kernels using this command queue
			
			 int maxMem = 0;
			{
                b3OpenCLArray<char> memTester(g_cxMainContext,g_cqCommandQue,0,true);
               
                bool result=true;
                for (size_t i=1;result;i++)
                {
                    size_t numBytes = i*1024*1024;
                    result = memTester.resize(numBytes,false);

                    if (result)
                    {
						maxMem=numBytes;
                        
                    } else
                    {
						break;
                    }
                }
				printf("allocated %d MB successfully\n",maxMem/(1024*1024));
			}


			clReleaseCommandQueue(g_cqCommandQue);
		}

		clReleaseContext(g_cxMainContext);

	}
	else {
		b3Printf("No OpenCL capable GPU found!");
	}
	b3Printf("press <Enter>\n");
	getchar();
	return 0;
}
