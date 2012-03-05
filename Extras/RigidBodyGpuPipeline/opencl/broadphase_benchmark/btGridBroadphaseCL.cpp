
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

#ifdef RELEASE_ME
#define COMPUTE_AABB_KERNEL_PATH "computeAabbKernelOCL.cl"
#else
#define COMPUTE_AABB_KERNEL_PATH "..\\..\\opencl\\broadphase_benchmark\\computeAabbKernelOCL"
#endif


#include "btGridBroadphaseCl.h"
#include "LinearMath/btQuickprof.h"
#include "Adl/Adl.h"
#include "AdlPrimitives/Math/Math.h"

#include "Adl/AdlKernel.h"
#include "../basic_initialize/btOpenCLUtils.h"
#define MSTRINGIFY(A) #A
static const char* spComputeAabbSource= 
#include "computeAabbKernelOCL.cl"

struct  btTmpAabb
{
	float			minfx;
	float			minfy;
	float			minfz;
	unsigned int	index0;
	float			maxfx;
	float			maxfy;
	float			maxfz;
	unsigned int	index1;	
} ;




btGridBroadphaseCl::btGridBroadphaseCl(	btOverlappingPairCache* overlappingPairCache,
							const btVector3& cellSize, 
							int gridSizeX, int gridSizeY, int gridSizeZ, 
							int maxSmallProxies, int maxLargeProxies, int maxPairsPerSmallProxy,
							btScalar maxSmallProxySize,
							int maxSmallProxiesPerCell,
							cl_context context,
							cl_device_id device,
							cl_command_queue queue,
							adl::DeviceCL* deviceCL)
:bt3dGridBroadphaseOCL(overlappingPairCache,cellSize,
				gridSizeX, gridSizeY, gridSizeZ, 
						maxSmallProxies, maxLargeProxies, maxPairsPerSmallProxy,
						maxSmallProxySize,maxSmallProxiesPerCell,
						context,device,queue,deviceCL)			
{
	m_computeAabbKernel = m_deviceCL->getKernel(COMPUTE_AABB_KERNEL_PATH,"computeAabb","",spComputeAabbSource);

	m_countOverlappingPairs = m_deviceCL->getKernel(COMPUTE_AABB_KERNEL_PATH,"countOverlappingpairs","",spComputeAabbSource);

	m_squeezePairCaches = m_deviceCL->getKernel(COMPUTE_AABB_KERNEL_PATH,"squeezePairCaches","",spComputeAabbSource);

	m_aabbConstBuffer = new adl::Buffer<MyAabbConstData >(m_deviceCL,1,adl::BufferBase::BUFFER_CONST);

	size_t memSize = m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int)*2;
	cl_int ciErrNum=0;
	m_dAllOverlappingPairs = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);

	memset(m_hAllOverlappingPairs, 0x00, sizeof(MyUint2)*m_maxHandles * m_maxPairsPerBody);
	copyArrayToDevice(m_dAllOverlappingPairs, m_hAllOverlappingPairs, m_maxHandles * m_maxPairsPerBody * sizeof(MyUint2));

	
	
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	

}

btGridBroadphaseCl::~btGridBroadphaseCl()
{
	clReleaseMemObject(m_dAllOverlappingPairs);
	
	delete m_aabbConstBuffer;

}



void btGridBroadphaseCl::prepareAABB(float* positions, int numObjects)
{
	return;
#if 0
bt3dGridBroadphaseOCL::prepareAABB();
#else
	BT_PROFILE("prepareAABB");
	bt3DGrid3F1U* pBB = m_hAABB;

	int new_largest_index = numObjects;
	unsigned int num_small = numObjects;
	m_LastHandleIndex = new_largest_index;
	new_largest_index = -1;
	unsigned int num_large = 0;
	m_LastLargeHandleIndex = new_largest_index;
	// paranoid checks
	//btAssert(num_small == m_numHandles);
	//btAssert(num_large == m_numLargeHandles);

	//copyArrayFromDevice( m_hAABB, m_dAABB, sizeof(bt3DGrid3F1U) * 2 * (m_numHandles + m_numLargeHandles));
	//clFinish(m_cqCommandQue);
#endif

}
void btGridBroadphaseCl::calcHashAABB()
{
	bt3dGridBroadphaseOCL::calcHashAABB();	
}


void btGridBroadphaseCl::calculateOverlappingPairs(float* positions, int numObjects)
{
	btDispatcher* dispatcher=0;

	// update constants
	{
		BT_PROFILE("setParameters");
		setParameters(&m_params);
	}

	// prepare AABB array
	{
		BT_PROFILE("prepareAABB");
		prepareAABB(positions, numObjects);
	}
	// calculate hash
	{
		BT_PROFILE("calcHashAABB");
		calcHashAABB();
	}

	{
		BT_PROFILE("sortHash");
		// sort bodies based on hash
		sortHash();
	}

	// find start of each cell
	{
		BT_PROFILE("findCellStart");
		findCellStart();
	}
	
	{
		BT_PROFILE("findOverlappingPairs");
		// findOverlappingPairs (small/small)
		findOverlappingPairs();
	}

	// add pairs to CPU cache
	{
		BT_PROFILE("computePairCacheChanges");
#if 0
		computePairCacheChanges();
#else
		int ciErrNum=0;

		ciErrNum=clSetKernelArg((cl_kernel)m_countOverlappingPairs->m_kernel, 0, sizeof(int), (void*)&numObjects);
		ciErrNum=clSetKernelArg((cl_kernel)m_countOverlappingPairs->m_kernel, 1, sizeof(cl_mem),(void*)&m_dPairBuff);
		ciErrNum=clSetKernelArg((cl_kernel)m_countOverlappingPairs->m_kernel, 2, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
		ciErrNum=clSetKernelArg((cl_kernel)m_countOverlappingPairs->m_kernel, 3, sizeof(cl_mem),(void*)&m_dPairScanChanged);
		ciErrNum=clSetKernelArg((cl_kernel)m_countOverlappingPairs->m_kernel, 4, sizeof(cl_mem),(void*)&m_dAABB);


		size_t localWorkSize=64;
		size_t numWorkItems = localWorkSize*((numObjects+ (localWorkSize)) / localWorkSize);

	
		ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, (cl_kernel)m_countOverlappingPairs->m_kernel, 1, NULL, &numWorkItems, &localWorkSize, 0,0,0 );
oclCHECKERROR(ciErrNum, CL_SUCCESS);
		ciErrNum = clFlush(m_cqCommandQue);
#endif


	}
	{
		BT_PROFILE("scanOverlappingPairBuff");
		scanOverlappingPairBuff(false);
	}
	{
		BT_PROFILE("squeezeOverlappingPairBuff");
//#define FORCE_CPU
#ifdef FORCE_CPU
		bt3dGridBroadphaseOCL::squeezeOverlappingPairBuff();
		copyArrayToDevice(m_dPairsChangedXY, m_hPairsChangedXY, sizeof( MyUint2) * m_numPrefixSum); //gSum
#else
		//squeezeOverlappingPairBuff();
		int ciErrNum = 0;
		ciErrNum=clSetKernelArg((cl_kernel)m_squeezePairCaches->m_kernel, 0, sizeof(int), (void*)&numObjects);
		ciErrNum=clSetKernelArg((cl_kernel)m_squeezePairCaches->m_kernel, 1, sizeof(cl_mem),(void*)&m_dPairBuff);
		ciErrNum=clSetKernelArg((cl_kernel)m_squeezePairCaches->m_kernel, 2, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
		ciErrNum=clSetKernelArg((cl_kernel)m_squeezePairCaches->m_kernel, 3, sizeof(cl_mem),(void*)&m_dPairScanChanged);
		ciErrNum=clSetKernelArg((cl_kernel)m_squeezePairCaches->m_kernel, 4, sizeof(cl_mem),(void*)&m_dAllOverlappingPairs);
		ciErrNum=clSetKernelArg((cl_kernel)m_squeezePairCaches->m_kernel, 5, sizeof(cl_mem),(void*)&m_dAABB);

		size_t workGroupSize = 64;
		size_t numWorkItems = workGroupSize*((numObjects+ (workGroupSize)) / workGroupSize);

	
		ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, (cl_kernel)m_squeezePairCaches->m_kernel, 1, NULL, &numWorkItems, &workGroupSize, 0,0,0 );
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		

//		copyArrayFromDevice(m_hAllOverlappingPairs, m_dAllOverlappingPairs, sizeof(unsigned int) * m_numPrefixSum*2); //gSum
//		clFinish(m_cqCommandQue);
#endif

	}


	return;
}

