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

#include "../basic_initialize/btOpenCLUtils.h"
#include <stdio.h>

cl_context			g_cxMainContext;
cl_command_queue	g_cqCommandQue;
cl_kernel			g_atomicsKernel;
static const size_t workGroupSize = 128;//todo figure out an appropriate workgroup size suitable for the OpenCL platform/context/device/kernel
#define NUM_OBJECTS 1024

#include "globalAtomicsKernel.h"


char * findAndReplace(   char const * const original,     char const * const pattern,     char const * const replacement);


#include <string.h>
#include <malloc.h>


int main(int argc, char* argv[])
{
	int ciErrNum = 0;
	
	printf("press a key to start\n");
	getchar();

	const char* vendorSDK = btOpenCLUtils::getSdkVendorName();
	printf("This program was compiled using the %s OpenCL SDK\n",vendorSDK);

	cl_device_type  deviceType = CL_DEVICE_TYPE_GPU;//CL_DEVICE_TYPE_ALL
	
	void* glCtx=0;
	void* glDC = 0;
	printf("Initialize OpenCL using btOpenCLUtils::createContextFromType for CL_DEVICE_TYPE_GPU\n");
	g_cxMainContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	int numDev = btOpenCLUtils::getNumDevices(g_cxMainContext);

	if (numDev>0)
	{
		int deviceIndex=0;

		cl_device_id		device;
		device = btOpenCLUtils::getDevice(g_cxMainContext,deviceIndex);
		btOpenCLDeviceInfo clInfo;
		btOpenCLUtils::getDeviceInfo(device,clInfo);
		btOpenCLUtils::printDeviceInfo(device);


		const char* globalAtomicsKernelStringPatched = globalAtomicsKernelString;
		if (!strstr(clInfo.m_deviceExtensions,"cl_ext_atomic_counters_32"))
		{
			globalAtomicsKernelStringPatched = findAndReplace(globalAtomicsKernelString,"counter32_t", "volatile __global int*");
		}

		

		// create a command-queue
		g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, device, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		
		cl_mem counterBuffer = clCreateBuffer(g_cxMainContext, CL_MEM_READ_WRITE, sizeof(int), NULL, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		char* kernelMethods[] = 
		{
			"globalAtomicKernelOpenCL1_1",
			"counterAtomicKernelExt",
			"globalAtomicKernelExt",
			"globalAtomicKernelCounters32Broken"
		};
		int numKernelMethods = sizeof(kernelMethods)/sizeof(char*);

		for (int i=0;i<numKernelMethods;i++)
		{
			int myCounter = 0;

			//write to counterBuffer
			int deviceOffset=0;
			int hostOffset=0;

			ciErrNum = clEnqueueWriteBuffer(g_cqCommandQue, counterBuffer,CL_FALSE, deviceOffset, sizeof(int), &myCounter, 0, NULL, NULL);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);

			g_atomicsKernel = btOpenCLUtils::compileCLKernelFromString(g_cxMainContext,device,globalAtomicsKernelStringPatched,kernelMethods[i], &ciErrNum);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);

		


			ciErrNum = clSetKernelArg(g_atomicsKernel, 0, sizeof(cl_mem),(void*)&counterBuffer);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);

			size_t	numWorkItems = workGroupSize*((NUM_OBJECTS + (workGroupSize-1)) / workGroupSize);
			ciErrNum = clEnqueueNDRangeKernel(g_cqCommandQue, g_atomicsKernel, 1, NULL, &numWorkItems, &workGroupSize,0 ,0 ,0);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
			
			clFinish(g_cqCommandQue);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);

			//read from counterBuffer
			ciErrNum = clEnqueueReadBuffer(g_cqCommandQue, counterBuffer, CL_TRUE, deviceOffset, sizeof(int), &myCounter, 0, NULL, NULL);
			 oclCHECKERROR(ciErrNum, CL_SUCCESS);

			 if (myCounter != NUM_OBJECTS)
			 {
				 printf("%s is broken, expected %d got %d\n",kernelMethods[i],NUM_OBJECTS,myCounter);
			 } else
			 {
				 printf("%s success, got %d\n",kernelMethods[i],myCounter);
			 }
		}

		clReleaseCommandQueue(g_cqCommandQue);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}

	clReleaseContext(g_cxMainContext);
	
	printf("press a key to end\n");
	getchar();

	return 0;
}


#ifdef _WIN32
#pragma warning( push )
#pragma warning( disable : 4996 )
#endif //_WIN32

#include <string.h>
#include <stdlib.h>

char * findAndReplace(
    char const * const original, 
    char const * const pattern, 
    char const * const replacement
) {
  size_t const replen = strlen(replacement);
  size_t const patlen = strlen(pattern);
  size_t const orilen = strlen(original);

  size_t patcnt = 0;
  const char * oriptr;
  const char * patloc;

  // find how many times the pattern occurs in the original string
  for (oriptr = original; patloc = strstr(oriptr, pattern); oriptr = patloc + patlen)
  {
    patcnt++;
  }

  {
    // allocate memory for the new string
    size_t const retlen = orilen + patcnt * (replen - patlen);
    char * const returned = (char *) malloc( sizeof(char) * (retlen + 1) );

    if (returned != NULL)
    {
      // copy the original string, 
      // replacing all the instances of the pattern
      char * retptr = returned;
      for (oriptr = original; patloc = strstr(oriptr, pattern); oriptr = patloc + patlen)
      {
        size_t const skplen = patloc - oriptr;
        // copy the section until the occurence of the pattern
        strncpy(retptr, oriptr, skplen);
        retptr += skplen;
        // copy the replacement 
        strncpy(retptr, replacement, replen);
        retptr += replen;
      }
      // copy the rest of the string.
      strcpy(retptr, oriptr);
    }
    return returned;
  }
}

#ifdef _WIN32
#pragma warning( pop )
#endif //_WIN32
