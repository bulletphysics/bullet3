/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2009 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btQuickprof.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "../basic_initialize/btOpenCLUtils.h"

#include "bt3dGridBroadphaseOCL.h"

#include <stdio.h>
#include <string.h>
#include "Adl/Adl.h"
#include <AdlPrimitives/Scan/PrefixScan.h>
#include <AdlPrimitives/Sort/RadixSort32.h>
#include <AdlPrimitives/Sort/RadixSort.h>

#define ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK

#define GRID_OCL_PATH "..\\..\\opencl\\3dGridBroadphase\\Shared\\bt3dGridBroadphaseOCL.cl"


#define MSTRINGIFY(A) #A

static const char* spProgramSource = 
#include "bt3dGridBroadphaseOCL.cl"

adl::PrefixScan<adl::TYPE_CL>::Data* gData1=0;
adl::Buffer<unsigned int>* m_srcClBuffer=0;

struct MySortData
{
	int key;
	int value;
};

adl::RadixSort32<adl::TYPE_CL>::Data* dataC = 0;
adl::RadixSort<adl::TYPE_HOST>::Data* dataHost = 0;


static unsigned int infElem = 0x2fffffff;

static unsigned int zeroEl = 0;
static unsigned int minusOne= -1;


bt3dGridBroadphaseOCL::bt3dGridBroadphaseOCL(	btOverlappingPairCache* overlappingPairCache,
												const btVector3& cellSize, 
												int gridSizeX, int gridSizeY, int gridSizeZ, 
												int maxSmallProxies, int maxLargeProxies, int maxPairsPerSmallProxy,
												btScalar maxSmallProxySize,
												int maxSmallProxiesPerCell,
												cl_context context, cl_device_id device, cl_command_queue queue,
												adl::DeviceCL* deviceCL
												) : 
	btGpu3DGridBroadphase(overlappingPairCache, cellSize, gridSizeX, gridSizeY, gridSizeZ, maxSmallProxies, maxLargeProxies, maxPairsPerSmallProxy, maxSmallProxySize, maxSmallProxiesPerCell)
{


	initCL(context, device, queue);
	allocateBuffers();
	
	prefillBuffers();

	initKernels();

	//create an Adl device host and OpenCL device

	adl::DeviceUtils::Config cfg;
	m_deviceHost = adl::DeviceUtils::allocate( adl::TYPE_HOST, cfg );
	m_ownsDevice = false;
	if (!deviceCL)
	{
		m_ownsDevice = true;
		deviceCL = new adl::DeviceCL;
		deviceCL->m_context = context;
		deviceCL->m_deviceIdx = device;
		deviceCL->m_commandQueue = queue;
		deviceCL->m_kernelManager = new adl::KernelManager;
	}

	m_deviceCL = deviceCL;

	int minSize = 256*1024;
	int maxSortBuffer = maxSmallProxies < minSize ? minSize :maxSmallProxies;

	m_srcClBuffer = new adl::Buffer<unsigned int> (m_deviceCL,maxSmallProxies+2);
	m_srcClBuffer->write(&zeroEl,1,0);

	//m_srcClBuffer->write(&infElem,maxSmallProxies,0);
	m_srcClBuffer->write(&infElem,1,maxSmallProxies);
	m_srcClBuffer->write(&zeroEl,1,maxSmallProxies+1);
	m_deviceCL->waitForCompletion();
	
	gData1 = adl::PrefixScan<adl::TYPE_CL>::allocate( m_deviceCL, maxSortBuffer+2,adl::PrefixScanBase::EXCLUSIVE );
	dataHost = adl::RadixSort<adl::TYPE_HOST>::allocate( m_deviceHost, maxSmallProxies+2 );
	dataC = adl::RadixSort32<adl::TYPE_CL>::allocate( m_deviceCL, maxSortBuffer+2 );
	
}



bt3dGridBroadphaseOCL::~bt3dGridBroadphaseOCL()
{
	//btSimpleBroadphase will free memory of btSortedOverlappingPairCache, because m_ownsPairCache
	assert(m_bInitialized);
	adl::RadixSort<adl::TYPE_HOST>::deallocate(dataHost);
	adl::PrefixScan<adl::TYPE_CL>::deallocate(gData1);
	adl::RadixSort32<adl::TYPE_CL>::deallocate(dataC);
	adl::DeviceUtils::deallocate(m_deviceHost);
	delete m_srcClBuffer;
	if (m_ownsDevice)
	{
		delete m_deviceCL->m_kernelManager;
		delete m_deviceCL;
	}
}

#ifdef CL_PLATFORM_MINI_CL
// there is a problem with MSVC9 : static constructors are not called if variables defined in library and are not used
// looks like it is because of optimization
// probably this will happen with other compilers as well
// so to make it robust, register kernels again (it is safe)
#define MINICL_DECLARE(a) extern "C" void a();
MINICL_DECLARE(kCalcHashAABB)
MINICL_DECLARE(kClearCellStart)
MINICL_DECLARE(kFindCellStart)
MINICL_DECLARE(kFindOverlappingPairs)
MINICL_DECLARE(kFindPairsLarge)
MINICL_DECLARE(kComputePairCacheChanges)
MINICL_DECLARE(kSqueezeOverlappingPairBuff)
#undef MINICL_DECLARE
#endif

void bt3dGridBroadphaseOCL::initCL(cl_context context, cl_device_id device, cl_command_queue queue)
{

	#ifdef CL_PLATFORM_MINI_CL
		// call constructors here
		MINICL_REGISTER(kCalcHashAABB)
		MINICL_REGISTER(kClearCellStart)
		MINICL_REGISTER(kFindCellStart)
		MINICL_REGISTER(kFindOverlappingPairs)
		MINICL_REGISTER(kFindPairsLarge)
		MINICL_REGISTER(kComputePairCacheChanges)
		MINICL_REGISTER(kSqueezeOverlappingPairBuff)
	#endif

	cl_int ciErrNum;

	btAssert(context);
	m_cxMainContext = context;
	btAssert(device);
	m_cdDevice = device;
	btAssert(queue);
	m_cqCommandQue = queue;
	
	//adl::Kernel kern = m_deviceCL->getKernel(fileName,funcName,options,src);
	
	m_cpProgram = btOpenCLUtils::compileCLProgramFromString(m_cxMainContext,m_cdDevice,spProgramSource, &ciErrNum,"-DGUID_ARG=""""",GRID_OCL_PATH);
	
	printf("OK\n");
}


void bt3dGridBroadphaseOCL::initKernels()
{
	initKernel(GRID3DOCL_KERNEL_CALC_HASH_AABB,	"kCalcHashAABB");
	setKernelArg(GRID3DOCL_KERNEL_CALC_HASH_AABB, 1, sizeof(cl_mem),(void*)&m_dAABB);
	setKernelArg(GRID3DOCL_KERNEL_CALC_HASH_AABB, 2, sizeof(cl_mem),(void*)&m_dBodiesHash);
	setKernelArg(GRID3DOCL_KERNEL_CALC_HASH_AABB, 3, sizeof(cl_mem),(void*)&m_dBpParams);

	initKernel(GRID3DOCL_KERNEL_CLEAR_CELL_START, "kClearCellStart");
	setKernelArg(GRID3DOCL_KERNEL_CLEAR_CELL_START, 1, sizeof(cl_mem),(void*)&m_dCellStart);

	initKernel(GRID3DOCL_KERNEL_FIND_CELL_START, "kFindCellStart");
	setKernelArg(GRID3DOCL_KERNEL_FIND_CELL_START, 1, sizeof(cl_mem),(void*)&m_dBodiesHash);
	setKernelArg(GRID3DOCL_KERNEL_FIND_CELL_START, 2, sizeof(cl_mem),(void*)&m_dCellStart);

	initKernel(GRID3DOCL_KERNEL_FIND_OVERLAPPING_PAIRS, "kFindOverlappingPairs");
	setKernelArg(GRID3DOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 1, sizeof(cl_mem),(void*)&m_dAABB);
	setKernelArg(GRID3DOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 2, sizeof(cl_mem),(void*)&m_dBodiesHash);
	setKernelArg(GRID3DOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 3, sizeof(cl_mem),(void*)&m_dCellStart);
	setKernelArg(GRID3DOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 4, sizeof(cl_mem),(void*)&m_dPairBuff);
	setKernelArg(GRID3DOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 5, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
	setKernelArg(GRID3DOCL_KERNEL_FIND_OVERLAPPING_PAIRS, 6, sizeof(cl_mem),(void*)&m_dBpParams);

	initKernel(GRID3DOCL_KERNEL_FIND_PAIRS_LARGE, "kFindPairsLarge");
	setKernelArg(GRID3DOCL_KERNEL_FIND_PAIRS_LARGE, 1, sizeof(cl_mem),(void*)&m_dAABB);
	setKernelArg(GRID3DOCL_KERNEL_FIND_PAIRS_LARGE, 2, sizeof(cl_mem),(void*)&m_dBodiesHash);
	setKernelArg(GRID3DOCL_KERNEL_FIND_PAIRS_LARGE, 3, sizeof(cl_mem),(void*)&m_dCellStart);
	setKernelArg(GRID3DOCL_KERNEL_FIND_PAIRS_LARGE, 4, sizeof(cl_mem),(void*)&m_dPairBuff);
	setKernelArg(GRID3DOCL_KERNEL_FIND_PAIRS_LARGE, 5, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);

	initKernel(GRID3DOCL_KERNEL_COMPUTE_CACHE_CHANGES, "kComputePairCacheChanges");
	setKernelArg(GRID3DOCL_KERNEL_COMPUTE_CACHE_CHANGES, 1, sizeof(cl_mem),(void*)&m_dPairBuff);
	setKernelArg(GRID3DOCL_KERNEL_COMPUTE_CACHE_CHANGES, 2, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
	setKernelArg(GRID3DOCL_KERNEL_COMPUTE_CACHE_CHANGES, 3, sizeof(cl_mem),(void*)&m_dPairScanChanged);
	setKernelArg(GRID3DOCL_KERNEL_COMPUTE_CACHE_CHANGES, 4, sizeof(cl_mem),(void*)&m_dAABB);

	initKernel(GRID3DOCL_KERNEL_SQUEEZE_PAIR_BUFF, "kSqueezeOverlappingPairBuff");
	setKernelArg(GRID3DOCL_KERNEL_SQUEEZE_PAIR_BUFF, 1, sizeof(cl_mem),(void*)&m_dPairBuff);
	setKernelArg(GRID3DOCL_KERNEL_SQUEEZE_PAIR_BUFF, 2, sizeof(cl_mem),(void*)&m_dPairBuffStartCurr);
	setKernelArg(GRID3DOCL_KERNEL_SQUEEZE_PAIR_BUFF, 3, sizeof(cl_mem),(void*)&m_dPairScanChanged);
	setKernelArg(GRID3DOCL_KERNEL_SQUEEZE_PAIR_BUFF, 4, sizeof(cl_mem),(void*)&m_dPairsChanged);
	setKernelArg(GRID3DOCL_KERNEL_SQUEEZE_PAIR_BUFF, 5, sizeof(cl_mem),(void*)&m_dAABB);

}


void bt3dGridBroadphaseOCL::allocateBuffers()
{
    cl_int ciErrNum;
    unsigned int memSize;
	// current version of bitonic sort works for power of 2 arrays only, so ...
	m_hashSize = 1;
	for(int bit = 1; bit < 32; bit++)
	{
		if(m_hashSize >= m_maxHandles)
		{
			break;
		}
		m_hashSize <<= 1;
	}
	memSize = m_hashSize * 2 * sizeof(unsigned int);
	if (memSize < 1024*1024)
		memSize = 1024*1024;

	m_dBodiesHash = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = m_numCells * sizeof(unsigned int);
	m_dCellStart = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int);
	m_dPairBuff = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = (m_maxHandles * 2 + 1) * sizeof(unsigned int);
	m_dPairBuffStartCurr = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	unsigned int numAABB = m_maxHandles + m_maxLargeHandles;
	memSize = numAABB * sizeof(float) * 4 * 2;
	m_dAABB = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = (m_maxHandles + 2) * sizeof(unsigned int);
	m_dPairScanChanged = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	memSize = m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int);
	m_dPairsChanged = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);

	
	memSize = 3 * 4 * sizeof(float);
	m_dBpParams = clCreateBuffer(m_cxMainContext, CL_MEM_READ_WRITE, memSize, NULL, &ciErrNum);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}

void bt3dGridBroadphaseOCL::prefillBuffers()
{
	memset(m_hBodiesHash, 0xFF, m_maxHandles*2*sizeof(unsigned int));
	copyArrayToDevice(m_dBodiesHash, m_hBodiesHash, m_maxHandles * 2 * sizeof(unsigned int));
	// now fill the rest (bitonic sorting works with size == pow of 2)
	int remainder = m_hashSize - m_maxHandles;
	if(remainder)
	{
		copyArrayToDevice(m_dBodiesHash, m_hBodiesHash, remainder * 2 * sizeof(unsigned int), m_maxHandles * 2 * sizeof(unsigned int), 0);
	}
	copyArrayToDevice(m_dPairBuffStartCurr, m_hPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int)); 
	memset(m_hPairBuff, 0x00, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));
	copyArrayToDevice(m_dPairBuff, m_hPairBuff, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));
}


void bt3dGridBroadphaseOCL::initKernel(int kernelId, char* pName)
{
	
	cl_int ciErrNum;
	cl_kernel kernel = clCreateKernel(m_cpProgram, pName, &ciErrNum);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	size_t wgSize;
	ciErrNum = clGetKernelWorkGroupInfo(kernel, m_cdDevice, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &wgSize, NULL);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	m_kernels[kernelId].m_Id = kernelId;
	m_kernels[kernelId].m_kernel = kernel;
	m_kernels[kernelId].m_name = pName;
	m_kernels[kernelId].m_workgroupSize = (int)wgSize;
	return;
}

void bt3dGridBroadphaseOCL::runKernelWithWorkgroupSize(int kernelId, int globalSize)
{
	if(globalSize <= 0)
	{
		return;
	}
	cl_kernel kernelFunc = m_kernels[kernelId].m_kernel;
	cl_int ciErrNum = clSetKernelArg(kernelFunc, 0, sizeof(int), (void*)&globalSize);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	int workgroupSize = btMin(64,m_kernels[kernelId].m_workgroupSize);

	if(workgroupSize <= 0)
	{ // let OpenCL library calculate workgroup size
		size_t globalWorkSize[2];
		globalWorkSize[0] = globalSize;
		globalWorkSize[1] = 1;
		ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, kernelFunc, 1, NULL, globalWorkSize, NULL, 0,0,0 );
	}
	else
	{
		size_t localWorkSize[2], globalWorkSize[2];
		//workgroupSize = btMin(workgroupSize, globalSize);
		int num_t = globalSize / workgroupSize;
		int num_g = num_t * workgroupSize;
		if(num_g < globalSize)
		{
			num_t++;
		}
		localWorkSize[0]  = workgroupSize;
		globalWorkSize[0] = num_t * workgroupSize;
		localWorkSize[1] = 1;
		globalWorkSize[1] = 1;
		ciErrNum = clEnqueueNDRangeKernel(m_cqCommandQue, kernelFunc, 1, NULL, globalWorkSize, localWorkSize, 0,0,0 );
	}
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	ciErrNum = clFlush(m_cqCommandQue);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}


void bt3dGridBroadphaseOCL::setKernelArg(int kernelId, int argNum, int argSize, void* argPtr)
{
    cl_int ciErrNum;
	ciErrNum  = clSetKernelArg(m_kernels[kernelId].m_kernel, argNum, argSize, argPtr);
	GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
}


void bt3dGridBroadphaseOCL::copyArrayToDevice(cl_mem device, const void* host, unsigned int size, int devOffs, int hostOffs)
{
	if (size)
	{
		cl_int ciErrNum;
		char* pHost = (char*)host + hostOffs;
		ciErrNum = clEnqueueWriteBuffer(m_cqCommandQue, device, CL_TRUE, devOffs, size, pHost, 0, NULL, NULL);
		GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	}
}

void bt3dGridBroadphaseOCL::copyArrayFromDevice(void* host, const cl_mem device, unsigned int size, int hostOffs, int devOffs)
{
	if (size)
    {
		cl_int ciErrNum;
		char* pHost = (char*)host + hostOffs;
		ciErrNum = clEnqueueReadBuffer(m_cqCommandQue, device, CL_TRUE, devOffs, size, pHost, 0, NULL, NULL);
		GRID3DOCL_CHECKERROR(ciErrNum, CL_SUCCESS);
	}
}



//
// overrides
//


void bt3dGridBroadphaseOCL::prepareAABB()
{
	btGpu3DGridBroadphase::prepareAABB();
	copyArrayToDevice(m_dAABB, m_hAABB, sizeof(bt3DGrid3F1U) * 2 * (m_numHandles + m_numLargeHandles)); 
	return;
}

void bt3dGridBroadphaseOCL::setParameters(bt3DGridBroadphaseParams* hostParams)
{
	btGpu3DGridBroadphase::setParameters(hostParams);
	struct btParamsBpOCL
	{
		float m_invCellSize[4];
		int   m_gridSize[4];
	};
	btParamsBpOCL hParams;
	hParams.m_invCellSize[0] = m_params.m_invCellSizeX;
	hParams.m_invCellSize[1] = m_params.m_invCellSizeY;
	hParams.m_invCellSize[2] = m_params.m_invCellSizeZ;
	hParams.m_invCellSize[3] = 0.f;
	hParams.m_gridSize[0] = m_params.m_gridSizeX;
	hParams.m_gridSize[1] = m_params.m_gridSizeY;
	hParams.m_gridSize[2] = m_params.m_gridSizeZ;
	hParams.m_gridSize[3] = m_params.m_maxBodiesPerCell;
	copyArrayToDevice(m_dBpParams, &hParams, sizeof(btParamsBpOCL));
	return;
}


void bt3dGridBroadphaseOCL::calcHashAABB()
{
	BT_PROFILE("calcHashAABB");
#if 1
	runKernelWithWorkgroupSize(GRID3DOCL_KERNEL_CALC_HASH_AABB, m_numHandles);
#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif //ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK

#else
	btGpu3DGridBroadphase::calcHashAABB();
#endif
	
	return;
}


void bt3dGridBroadphaseOCL::sortHash()
{
	BT_PROFILE("sortHash");
#ifdef CL_PLATFORM_MINI_CL
	//copyArrayFromDevice(m_hBodiesHash, m_dBodiesHash, m_numHandles * 2 * sizeof(unsigned int));
	btGpu3DGridBroadphase::sortHash();
	copyArrayToDevice(m_dBodiesHash, m_hBodiesHash, m_numHandles * 2 * sizeof(unsigned int));
#else
	
//#define USE_HOST
#ifdef USE_HOST
	copyArrayFromDevice(m_hBodiesHash, m_dBodiesHash, m_numHandles * 2 * sizeof(unsigned int));
	//adl::Buffer<unsigned int> keysIn,keysOut,valuesIn,valuesOut;
	///adl::RadixSort32<adl::TYPE_CL>::execute(dataC,keysIn,keysOut,valuesIn,valuesOut,m_numHandles);
	adl::HostBuffer<adl::SortData> inoutHost;
	inoutHost.m_device = m_deviceHost;
	inoutHost.m_ptr = (adl::SortData*)m_hBodiesHash;
	inoutHost.m_size = m_numHandles;
	adl::RadixSort<adl::TYPE_HOST>::execute(dataHost, inoutHost,m_numHandles);
	copyArrayToDevice(m_dBodiesHash, m_hBodiesHash, m_numHandles * 2 * sizeof(unsigned int));
#else
	{
	clFinish(m_cqCommandQue);
	BT_PROFILE("RadixSort32::execute");
	adl::Buffer<adl::SortData> inout;
	inout.m_device = this->m_deviceCL;
	inout.m_size = m_numHandles;
	inout.m_ptr = (adl::SortData*)m_dBodiesHash;
	int actualHandles = m_numHandles;
	int dataAlignment = adl::RadixSort32<adl::TYPE_CL>::DATA_ALIGNMENT;

	if (actualHandles%dataAlignment)
	{
		actualHandles += dataAlignment-(actualHandles%dataAlignment);
	}

	adl::RadixSort32<adl::TYPE_CL>::execute(dataC,inout, actualHandles);
#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif //ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	}
	{
		//BT_PROFILE("copyArrayFromDevice");
	//copyArrayFromDevice(m_hBodiesHash, m_dBodiesHash, m_numHandles * 2 * sizeof(unsigned int));
#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif //ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	}


#endif //USE_HOST
#endif

	return;
}



void bt3dGridBroadphaseOCL::findCellStart()
{
#if 1
	BT_PROFILE("findCellStart");
		
	#if defined(CL_PLATFORM_MINI_CL)
		btGpu3DGridBroadphase::findCellStart();
		copyArrayToDevice(m_dCellStart, m_hCellStart, m_numCells * sizeof(unsigned int));
	#else
			runKernelWithWorkgroupSize(GRID3DOCL_KERNEL_CLEAR_CELL_START, m_numCells);	
			runKernelWithWorkgroupSize(GRID3DOCL_KERNEL_FIND_CELL_START, m_numHandles);
	#endif
#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif

#else
	btGpu3DGridBroadphase::findCellStart();
#endif

	return;
}



void bt3dGridBroadphaseOCL::findOverlappingPairs()
{
#if 1
	BT_PROFILE("findOverlappingPairs");
	runKernelWithWorkgroupSize(GRID3DOCL_KERNEL_FIND_OVERLAPPING_PAIRS, m_numHandles);
#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif

#else
	btGpu3DGridBroadphase::findOverlappingPairs();
	copyArrayToDevice(m_dPairBuffStartCurr, m_hPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int)); 
	copyArrayToDevice(m_dPairBuff, m_hPairBuff, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));
#endif
	return;
}


void bt3dGridBroadphaseOCL::findPairsLarge()
{
	BT_PROFILE("findPairsLarge");
#if 1
	if(m_numLargeHandles)
	{
		setKernelArg(GRID3DOCL_KERNEL_FIND_PAIRS_LARGE, 6, sizeof(int),(void*)&m_numLargeHandles);
		runKernelWithWorkgroupSize(GRID3DOCL_KERNEL_FIND_PAIRS_LARGE, m_numHandles);
	}
#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif

#else
	btGpu3DGridBroadphase::findPairsLarge();
#endif
	return;
}



void bt3dGridBroadphaseOCL::computePairCacheChanges()
{
	BT_PROFILE("computePairCacheChanges");
#if 1
	runKernelWithWorkgroupSize(GRID3DOCL_KERNEL_COMPUTE_CACHE_CHANGES, m_numHandles);
#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif
	copyArrayFromDevice( m_hPairScanChanged,m_dPairScanChanged, sizeof(unsigned int)*(m_numHandles + 2)); 

#else
	btGpu3DGridBroadphase::computePairCacheChanges();
	copyArrayToDevice(m_dPairScanChanged, m_hPairScanChanged, sizeof(unsigned int)*(m_numHandles + 2)); 
	

#endif
	return;
}




extern cl_device_type deviceType;

void bt3dGridBroadphaseOCL::scanOverlappingPairBuff(bool copyToCpu)
{

	//Intel/CPU version doesn't handlel Adl scan well
#if 0
	{
		copyArrayFromDevice(m_hPairScanChanged, m_dPairScanChanged, sizeof(unsigned int)*(m_numHandles + 2)); 
		btGpu3DGridBroadphase::scanOverlappingPairBuff();
		copyArrayToDevice(m_dPairScanChanged, m_hPairScanChanged, sizeof(unsigned int)*(m_numHandles + 2)); 
		m_numPrefixSum = m_hPairScanChanged[m_numHandles+1];
		clFinish(m_cqCommandQue);
		//memset(m_hPairScanChanged,0,sizeof(int)*m_maxHandles + 2);
	}
#else
	{

	//	copyArrayFromDevice(m_hPairScanChanged, m_dPairScanChanged, sizeof(unsigned int)*(m_numHandles + 2)); 
	//	btGpu3DGridBroadphase::scanOverlappingPairBuff();

		adl::Buffer<unsigned int> destBuffer;
		
		{
			BT_PROFILE("copy GPU->GPU");
		
			destBuffer.m_ptr = (unsigned int*)m_dPairScanChanged;
			destBuffer.m_device = m_deviceCL;
			destBuffer.m_size =  sizeof(unsigned int)*(m_numHandles+2);
			m_deviceCL->copy(m_srcClBuffer, &destBuffer,m_numHandles,1,1);

#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif

		}

		{
			BT_PROFILE("PrefixScan");
			
			adl::PrefixScan<adl::TYPE_CL>::execute(gData1,*m_srcClBuffer,destBuffer, m_numHandles+2,&m_numPrefixSum);
			
#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif
		//if (m_numPrefixSum>0x1000)
		//	{
		//		printf("error m_numPrefixSum==%d\n",m_numPrefixSum);
		//	}

		}

#if 0
		unsigned int* verifyhPairScanChanged = new unsigned int[m_maxHandles + 2];
		memset(verifyhPairScanChanged,0,sizeof(int)*m_maxHandles + 2);

		copyArrayFromDevice(verifyhPairScanChanged, m_dPairScanChanged, sizeof(unsigned int)*(m_numHandles + 2));
		clFinish(m_cqCommandQue);

		/*for (int i=0;i<m_numHandles+2;i++)
		{
			if (verifyhPairScanChanged[i] != m_hPairScanChanged[i])
			{
				printf("hello!\n");
			}
		}
		*/

#endif


		if (1)
		{
			
			//the data 
			if (copyToCpu)
			{
				BT_PROFILE("copy GPU -> CPU");
				copyArrayFromDevice(m_hPairScanChanged, m_dPairScanChanged, sizeof(unsigned int)*(m_numHandles + 2));
#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif
			}

		}

	}
#endif

	
}



void bt3dGridBroadphaseOCL::squeezeOverlappingPairBuff()
{
	BT_PROFILE("btCuda_squeezeOverlappingPairBuff");
#if 1
	runKernelWithWorkgroupSize(GRID3DOCL_KERNEL_SQUEEZE_PAIR_BUFF, m_numHandles);
//	btCuda_squeezeOverlappingPairBuff(m_dPairBuff, m_dPairBuffStartCurr, m_dPairScanChanged, m_dPairsChanged, m_dAABB, m_numHandles);
	
	//copyArrayFromDevice(m_hPairsChanged, m_dPairsChanged, sizeof(unsigned int) * m_numPrefixSum);//m_hPairScanChanged[m_numHandles+1]); //gSum
#ifdef ADD_BLOCKING_CL_FINISH_FOR_BENCHMARK
	clFinish(m_cqCommandQue);
#endif

#else
	btGpu3DGridBroadphase::squeezeOverlappingPairBuff();
#endif
	return;
}



void bt3dGridBroadphaseOCL::resetPool(btDispatcher* dispatcher)
{
	btGpu3DGridBroadphase::resetPool(dispatcher);
	prefillBuffers();
}


