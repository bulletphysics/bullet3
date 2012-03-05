
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
//Originally written by Roman Ponomarev, Erwin Coumans

#include "findPairsOpenCL.h"
#include "../basic_initialize/btOpenCLUtils.h"

#define MSTRINGIFY(A) #A
static char* broadphaseKernelString = 
#include "broadphaseKernel.cl"

#define GRID_BROADPHASE_PATH "..\\..\\opencl\\broadphase_benchmark\\broadphaseKernel.cl"




void initFindPairs(btFindPairsIO& fpio,cl_context cxMainContext, cl_device_id device, cl_command_queue commandQueue, int maxHandles, int maxPairsPerBody)
{

	//m_proxies.push_back( proxy );

	fpio.m_mainContext = cxMainContext;
	fpio.m_cqCommandQue = commandQueue;
	fpio.m_device = device;
	cl_int pErrNum;
	cl_program prog = btOpenCLUtils::compileCLProgramFromString(cxMainContext, device, broadphaseKernelString, &pErrNum ,"",GRID_BROADPHASE_PATH);

	fpio.m_broadphaseBruteForceKernel = btOpenCLUtils::compileCLKernelFromString(cxMainContext,device, broadphaseKernelString, "broadphaseKernel" ,&pErrNum,prog);
	fpio.m_initializeGpuAabbsKernelSimple = btOpenCLUtils::compileCLKernelFromString(cxMainContext,device, broadphaseKernelString, "initializeGpuAabbsSimple" ,&pErrNum,prog);
	fpio.m_initializeGpuAabbsKernelFull = btOpenCLUtils::compileCLKernelFromString(cxMainContext,device, broadphaseKernelString, "initializeGpuAabbsFull" ,&pErrNum,prog);

	fpio.m_broadphaseColorKernel = btOpenCLUtils::compileCLKernelFromString(cxMainContext,device, broadphaseKernelString, "broadphaseColorKernel" ,&pErrNum,prog);

	fpio.m_setupBodiesKernel = btOpenCLUtils::compileCLKernelFromString(cxMainContext,device, broadphaseKernelString, "setupBodiesKernel" ,&pErrNum,prog);
	fpio.m_copyVelocitiesKernel = btOpenCLUtils::compileCLKernelFromString(cxMainContext,device, broadphaseKernelString, "copyVelocitiesKernel" ,&pErrNum,prog);



}

void	findPairsOpenCLBruteForce(btFindPairsIO& fpio)
{

			int ciErrNum = 0;

			int numObjects = fpio.m_numObjects;
			int offset = fpio.m_positionOffset;

			ciErrNum = clSetKernelArg(fpio.m_broadphaseBruteForceKernel, 0, sizeof(int), &offset);
			ciErrNum = clSetKernelArg(fpio.m_broadphaseBruteForceKernel, 1, sizeof(int), &numObjects);
			ciErrNum = clSetKernelArg(fpio.m_broadphaseBruteForceKernel, 2, sizeof(cl_mem), (void*)&fpio.m_clObjectsBuffer);
		
			size_t numWorkItems = numObjects;///workGroupSize*((NUM_OBJECTS + (workGroupSize)) / workGroupSize);
			size_t workGroupSize = 64;
			ciErrNum = clEnqueueNDRangeKernel(fpio.m_cqCommandQue, fpio.m_broadphaseBruteForceKernel, 1, NULL, &numWorkItems, &workGroupSize,0 ,0 ,0);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
}

void	setupGpuAabbsFull(btFindPairsIO& fpio, cl_mem bodies)
{

			int ciErrNum = 0;

			int numObjects = fpio.m_numObjects;
			int offset = fpio.m_positionOffset;

			ciErrNum = clSetKernelArg(fpio.m_initializeGpuAabbsKernelFull, 0, sizeof(int), &offset);
			ciErrNum = clSetKernelArg(fpio.m_initializeGpuAabbsKernelFull, 1, sizeof(int), &numObjects);
			ciErrNum = clSetKernelArg(fpio.m_initializeGpuAabbsKernelFull, 2, sizeof(cl_mem), (void*)&fpio.m_clObjectsBuffer);
			ciErrNum = clSetKernelArg(fpio.m_initializeGpuAabbsKernelFull, 3, sizeof(cl_mem), (void*)&bodies);
			ciErrNum = clSetKernelArg(fpio.m_initializeGpuAabbsKernelFull, 4, sizeof(cl_mem), (void*)&fpio.m_dlocalShapeAABB);
			ciErrNum = clSetKernelArg(fpio.m_initializeGpuAabbsKernelFull, 5, sizeof(cl_mem), (void*)&fpio.m_dAABB);
				size_t workGroupSize = 64;
			size_t numWorkItems = workGroupSize*((numObjects+ (workGroupSize)) / workGroupSize);
		
			ciErrNum = clEnqueueNDRangeKernel(fpio.m_cqCommandQue, fpio.m_initializeGpuAabbsKernelFull, 1, NULL, &numWorkItems, &workGroupSize,0 ,0 ,0);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
}

void	setupGpuAabbsSimple(btFindPairsIO& fpio)
{

			int ciErrNum = 0;

			int numObjects = fpio.m_numObjects;
			int offset = fpio.m_positionOffset;

			ciErrNum = clSetKernelArg(fpio.m_initializeGpuAabbsKernelSimple, 0, sizeof(int), &offset);
			ciErrNum = clSetKernelArg(fpio.m_initializeGpuAabbsKernelSimple, 1, sizeof(int), &numObjects);
			ciErrNum = clSetKernelArg(fpio.m_initializeGpuAabbsKernelSimple, 2, sizeof(cl_mem), (void*)&fpio.m_clObjectsBuffer);
			ciErrNum = clSetKernelArg(fpio.m_initializeGpuAabbsKernelSimple, 3, sizeof(cl_mem), (void*)&fpio.m_dAABB);
				size_t workGroupSize = 64;
			size_t numWorkItems = workGroupSize*((numObjects+ (workGroupSize)) / workGroupSize);
		
			ciErrNum = clEnqueueNDRangeKernel(fpio.m_cqCommandQue, fpio.m_initializeGpuAabbsKernelSimple, 1, NULL, &numWorkItems, &workGroupSize,0 ,0 ,0);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
}


void	setupBodies(btFindPairsIO& fpio, cl_mem linVelMem, cl_mem angVelMem, cl_mem bodies, cl_mem bodyInertias)
{
	int ciErrNum = 0;

	int numObjects = fpio.m_numObjects;
	int offset = fpio.m_positionOffset;

	ciErrNum = clSetKernelArg(fpio.m_setupBodiesKernel, 0, sizeof(int), &offset);
	ciErrNum = clSetKernelArg(fpio.m_setupBodiesKernel, 1, sizeof(int), &fpio.m_numObjects);
	ciErrNum = clSetKernelArg(fpio.m_setupBodiesKernel, 2, sizeof(cl_mem), (void*)&fpio.m_clObjectsBuffer);

	ciErrNum = clSetKernelArg(fpio.m_setupBodiesKernel, 3, sizeof(cl_mem), (void*)&linVelMem);
	ciErrNum = clSetKernelArg(fpio.m_setupBodiesKernel, 4, sizeof(cl_mem), (void*)&angVelMem);
	ciErrNum = clSetKernelArg(fpio.m_setupBodiesKernel, 5, sizeof(cl_mem), (void*)&bodies);
	ciErrNum = clSetKernelArg(fpio.m_setupBodiesKernel, 6, sizeof(cl_mem), (void*)&bodyInertias);
	
	if (numObjects)
	{
		size_t workGroupSize = 64;
		size_t numWorkItems = workGroupSize*((numObjects+ (workGroupSize)) / workGroupSize);

		ciErrNum = clEnqueueNDRangeKernel(fpio.m_cqCommandQue, fpio.m_setupBodiesKernel, 1, NULL, &numWorkItems, &workGroupSize,0 ,0 ,0);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}

}


void	copyBodyVelocities(btFindPairsIO& fpio, cl_mem linVelMem, cl_mem angVelMem, cl_mem bodies, cl_mem bodyInertias)
{
	int ciErrNum = 0;

	int numObjects = fpio.m_numObjects;
	int offset = fpio.m_positionOffset;

	ciErrNum = clSetKernelArg(fpio.m_copyVelocitiesKernel, 0, sizeof(int), &offset);
	ciErrNum = clSetKernelArg(fpio.m_copyVelocitiesKernel, 1, sizeof(int), &fpio.m_numObjects);
	ciErrNum = clSetKernelArg(fpio.m_copyVelocitiesKernel, 2, sizeof(cl_mem), (void*)&fpio.m_clObjectsBuffer);

	ciErrNum = clSetKernelArg(fpio.m_copyVelocitiesKernel, 3, sizeof(cl_mem), (void*)&linVelMem);
	ciErrNum = clSetKernelArg(fpio.m_copyVelocitiesKernel, 4, sizeof(cl_mem), (void*)&angVelMem);
	ciErrNum = clSetKernelArg(fpio.m_copyVelocitiesKernel, 5, sizeof(cl_mem), (void*)&bodies);
	ciErrNum = clSetKernelArg(fpio.m_copyVelocitiesKernel, 6, sizeof(cl_mem), (void*)&bodyInertias);
	
	if (numObjects)
	{
		size_t workGroupSize = 64;
		size_t numWorkItems = workGroupSize*((numObjects+ (workGroupSize)) / workGroupSize);
				
		ciErrNum = clEnqueueNDRangeKernel(fpio.m_cqCommandQue, fpio.m_copyVelocitiesKernel, 1, NULL, &numWorkItems, &workGroupSize,0 ,0 ,0);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}

}

void	colorPairsOpenCL(btFindPairsIO&	fpio)
{
	int ciErrNum = 0;

	int numObjects = fpio.m_numObjects;
	int offset = fpio.m_positionOffset;

	ciErrNum = clSetKernelArg(fpio.m_broadphaseColorKernel, 0, sizeof(int), &offset);
	ciErrNum = clSetKernelArg(fpio.m_broadphaseColorKernel, 1, sizeof(int), &fpio.m_numObjects);
	ciErrNum = clSetKernelArg(fpio.m_broadphaseColorKernel, 2, sizeof(cl_mem), (void*)&fpio.m_clObjectsBuffer);
	ciErrNum = clSetKernelArg(fpio.m_broadphaseColorKernel, 3, sizeof(cl_mem), (void*)&fpio.m_dAllOverlappingPairs);
	ciErrNum = clSetKernelArg(fpio.m_broadphaseColorKernel, 4, sizeof(int), &fpio.m_numOverlap);


	if (fpio.m_numOverlap)
	{
		size_t workGroupSize = 64;
		size_t numWorkItems = workGroupSize*((fpio.m_numOverlap+ (workGroupSize)) / workGroupSize);
				
		ciErrNum = clEnqueueNDRangeKernel(fpio.m_cqCommandQue, fpio.m_broadphaseColorKernel, 1, NULL, &numWorkItems, &workGroupSize,0 ,0 ,0);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}
}



void releaseFindPairs(btFindPairsIO& fpio)
{
	clReleaseKernel(fpio.m_initializeGpuAabbsKernelSimple);
	clReleaseKernel(fpio.m_initializeGpuAabbsKernelFull);
	clReleaseKernel(fpio.m_broadphaseColorKernel);
	clReleaseKernel(fpio.m_broadphaseBruteForceKernel);
	clReleaseKernel(fpio.m_setupBodiesKernel);
	clReleaseKernel(fpio.m_copyVelocitiesKernel);


}

