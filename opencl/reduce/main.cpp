///original author: Erwin Coumans
#include "b3OpenCLUtils.h"
#include "../parallel_primitives/host/btOpenCLArray.h"
#include "../parallel_primitives/host/btLauncherCL.h"
#include <stdio.h>


#define MSTRINGIFY(A) #A
const char* kernelString= MSTRINGIFY(
__kernel void ReduceGlobal(__global int* d_in, __global int* d_out, int numElements)
{
	int myId = get_global_id(0);
	int tid = get_local_id(0);


	int ls = get_local_size(0);
	for (unsigned int s=ls/2;s>0;s>>=1)
	{
		if (myId<numElements)
		{
			if (tid<s)
			{
				d_in[myId] += d_in[myId+s];
			}
		}
		barrier(CLK_GLOBAL_MEM_FENCE);
	}
	if (tid==0)
	{
		if (myId<numElements)
		{
			d_out[get_group_id(0)]=d_in[myId];
		}
	}
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
	ctx = b3OpenCLUtils::createContextFromType(CL_DEVICE_TYPE_ALL, &ciErrNum,0,0,preferred_device,preferred_platform,&platformId);
	b3OpenCLUtils::printPlatformInfo(platformId);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	if (!ctx) {
		printf("No OpenCL capable GPU found!");
		return 0;
	}

	device = b3OpenCLUtils::getDevice(ctx,0);
	queue = clCreateCommandQueue(ctx, device, 0, &ciErrNum);
	addKernel = b3OpenCLUtils::compileCLKernelFromString(ctx,device,kernelString,"ReduceGlobal",&ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	int numElements = 1024*1024;
	btOpenCLArray<int> a(ctx,queue);
	btOpenCLArray<int> b(ctx,queue);
	btAlignedObjectArray<int> hostA;
	btAlignedObjectArray<int> hostB;

	for (int i=0;i<numElements;i++)
	{
		hostA.push_back(1);
		hostB.push_back(0.f);
	}
	a.copyFromHost(hostA);
	b.copyFromHost(hostB);
	
	int hostSum= 0;
	for (int i=0;i<numElements;i++)
	{
		hostSum += hostA.at(i);
	}
	b.resize(numElements);

	{
		btLauncherCL launcher( queue, addKernel);
		launcher.setBuffer( a.getBufferCL());
		launcher.setBuffer( b.getBufferCL());
		launcher.setConst(  numElements );
		launcher.launch1D( numElements,1024);
	}
	clFinish(queue);
	{
		btLauncherCL launcher( queue, addKernel);
		launcher.setBuffer( b.getBufferCL());
		launcher.setBuffer( a.getBufferCL());
		launcher.setConst(  1024 );
		launcher.launch1D( 1024,1024);
	}
	clFinish(queue);

	printf("hostSum = %d\n", hostSum);

	int clSum = a.at(0);
	printf("clSum = %d\n", clSum );
	if (hostSum != clSum)
	{
		printf("Incorrect result\n");
	} else
	{
		printf("Correct result\n");
	}

	
	clReleaseCommandQueue(queue);
	clReleaseContext(ctx);
	printf("press key\n");
	getchar();
	return 0;
}