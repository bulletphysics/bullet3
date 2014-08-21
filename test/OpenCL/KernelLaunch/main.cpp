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
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include <stdio.h>
#include <string.h>

#include "Bullet3Common/shared/b3Float4.h"

//typedef b3Vector3 b3Float4;
typedef  struct b3Contact4Data b3Contact4Data_t;
struct b3Contact4Data
{
	b3Float4	m_worldPos[4];
	b3Float4	m_localPosA[4];
	b3Float4	m_localPosB[4];
	b3Float4	m_worldNormal;	//	w: m_nPoints
	unsigned short  m_restituitionCoeffCmp;
	unsigned short  m_frictionCoeffCmp;
	int m_batchIdx;
	int m_bodyAPtrAndSignBit;//x:m_bodyAPtr, y:m_bodyBPtr
	int m_bodyBPtrAndSignBit;
	int	m_childIndexA;
	int	m_childIndexB;
	int m_unused1;
	int m_unused2;

};


#define MSTRINGIFY(A) #A

static const char* s_testKernelString= MSTRINGIFY(

struct MyTest
{
	int bla;
};

typedef float4	b3Float4;
typedef  struct b3Contact4Data b3Contact4Data_t;
struct b3Contact4Data
{
	b3Float4	m_worldPos[4];
	b3Float4	m_localPosA[4];
	b3Float4	m_localPosB[4];
	b3Float4	m_worldNormal;	//	w: m_nPoints
	unsigned short  m_restituitionCoeffCmp;
	unsigned short  m_frictionCoeffCmp;
	int m_batchIdx;
	int m_bodyAPtrAndSignBit;//x:m_bodyAPtr, y:m_bodyBPtr
	int m_bodyBPtrAndSignBit;
	int	m_childIndexA;
	int	m_childIndexB;
	int m_unused1;
	int m_unused2;

};
inline int b3Contact4Data_getNumPoints(const struct b3Contact4Data* contact)
{
	return (int)contact->m_worldNormal.w;
};
inline void b3Contact4Data_setNumPoints(struct b3Contact4Data* contact, int numPoints)
{
	contact->m_worldNormal.w = (float)numPoints;
};

typedef volatile __global int* my_counter32_t;


__kernel void   testKernel( __global int* testData, __global b3Contact4Data_t* contactData, my_counter32_t numElements)
{
	int id = get_local_id(0);
	int sz = sizeof(b3Contact4Data_t);
	testData[id]=sz;

	__private b3Contact4Data_t tmp;
	if (id==0)
	{
		tmp = contactData[1];
		contactData[1] = contactData[0];
		contactData[0] = tmp;
	}
}



);



#include "Bullet3Common/b3Logging.h"


void myprintf(const char* msg)
{
	//OutputDebugStringA(msg);
	printf("%s",msg);
}

int main(int argc, char* argv[])
{
	b3SetCustomPrintfFunc(myprintf);
	//b3SetCustomWarningMessageFunc(myprintf);
	//b3SetCustomErrorMessageFunc(myprintf);

	b3Printf("test b3Printf\n");
	b3Warning("test warning\n");
	b3Error("test error\n");

	int ciErrNum = 0;

	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
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

		cl_context context = b3OpenCLUtils::createContextFromPlatform(platform,deviceType,&ciErrNum);
		if (context)
		{
			int numDevices = b3OpenCLUtils::getNumDevices(context);
			b3Printf("Num Devices = %d\n", numDevices);
			for (int j=0;j<numDevices;j++)
			{
				cl_device_id dev = b3OpenCLUtils::getDevice(context,j);
				b3OpenCLDeviceInfo devInfo;
				b3OpenCLUtils::getDeviceInfo(dev,&devInfo);
				b3OpenCLUtils::printDeviceInfo(dev);

				int errNum;

				cl_command_queue queue = clCreateCommandQueue(context, dev, 0, &errNum);


				cl_program pairBenchProg=0;

				cl_kernel testKernel = b3OpenCLUtils::compileCLKernelFromString(context,dev,s_testKernelString,"testKernel",&errNum,pairBenchProg);
				if (testKernel)
				{
					printf("kernel compiled ok\n");

					int numWorkItems = 64;
					b3OpenCLArray<int> deviceElements(context,queue);
					b3OpenCLArray<int> atomicCounter(context,queue);
					b3OpenCLArray<b3Contact4Data> deviceContacts(context,queue);
					b3AlignedObjectArray<b3Contact4Data> hostContacts;

					b3Contact4Data tmp;
					int sz = sizeof(b3Contact4Data);
					memset(&tmp,1,sz);
					deviceContacts.push_back(tmp);
					b3Contact4Data tmp2 = tmp;
					memset(&tmp,2,sz);
					deviceContacts.push_back(tmp);
					b3Contact4Data tmp3 = tmp;


					atomicCounter.push_back(0);
					deviceElements.resize(numWorkItems);
					b3LauncherCL run(queue,testKernel,"testKernel");
					run.setBuffer(deviceElements.getBufferCL());
					run.setBuffer(deviceContacts.getBufferCL());
					run.setBuffer(atomicCounter.getBufferCL());

					run.launch1D(numWorkItems);

					b3AlignedObjectArray<int> hostElements;
					deviceElements.copyToHost(hostElements);
					deviceContacts.copyToHost(hostContacts);
					tmp2 = hostContacts[0];
					tmp3 = hostContacts[1];


					printf("...\n");

				} else
				{
					printf("kernel failed to compile\n");
				}



			}
		}

		clReleaseContext(context);
	}

	b3Printf("\npress <Enter>\n");
	getchar();
	return 0;
}
