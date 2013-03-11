///original author: Erwin Coumans
#include "btOpenCLUtils.h"
#include "../parallel_primitives/host/btOpenCLArray.h"
#include "../parallel_primitives/host/btLauncherCL.h"
#include <stdio.h>


#define MSTRINGIFY(A) #A
const char* kernelString= MSTRINGIFY(
__kernel void VectorAdd(__global const float* a, __global const float* b, __global float* c, int numElements)
{
  int iGID = get_global_id(0);
	if (iGID>=numElements)
		return;
	float aGID = a[iGID];
	float bGID = b[iGID];
	float result = aGID + bGID;
    c[iGID] = result;
}
);

int main(int argc, char* argv[])
{
	int ciErrNum = 0;
	int preferred_device = -1;
	int preferred_platform = -1;
	cl_platform_id		platformId;
	cl_context			ctx;
	cl_command_queue	queue;
	cl_device_id		device;
	cl_kernel			addKernel;
	ctx = btOpenCLUtils::createContextFromType(CL_DEVICE_TYPE_GPU, &ciErrNum,0,0,preferred_device,preferred_platform,&platformId);
	btOpenCLUtils::printPlatformInfo(platformId);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	if (!ctx) {
		printf("No OpenCL capable GPU found!");
		return 0;
	}

	device = btOpenCLUtils::getDevice(ctx,0);
	queue = clCreateCommandQueue(ctx, device, 0, &ciErrNum);
	addKernel = btOpenCLUtils::compileCLKernelFromString(ctx,device,kernelString,"VectorAdd",&ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	int numElements = 32;
	btOpenCLArray<float> a(ctx,queue);
	btOpenCLArray<float> b(ctx,queue);
	btOpenCLArray<float> c(ctx,queue);
	for (int i=0;i<numElements;i++)
	{
		a.push_back(float(i));
		b.push_back(float(i));
	}
	
	c.resize(numElements);
	btLauncherCL launcher( queue, addKernel);
	launcher.setBuffer( a.getBufferCL());
	launcher.setBuffer( b.getBufferCL());
	launcher.setBuffer( c.getBufferCL());
	launcher.setConst(  numElements );
	launcher.launch1D( numElements);
	for (int i=0;i<numElements;i++)
	{
		float v = c.at(i);
		printf("c[%d]=%f\n",i,v);
	}
	clReleaseCommandQueue(queue);
	clReleaseContext(ctx);
	return 0;
}