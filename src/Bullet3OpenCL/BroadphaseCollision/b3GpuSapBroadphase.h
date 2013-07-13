#ifndef B3_GPU_SAP_BROADPHASE_H
#define B3_GPU_SAP_BROADPHASE_H

#include "Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3FillCL.h" //b3Int2
class b3Vector3;
#include "Bullet3OpenCL/ParallelPrimitives/b3RadixSort32CL.h"

#include "b3SapAabb.h"



class b3GpuSapBroadphase
{
	
	cl_context				m_context;
	cl_device_id			m_device;
	cl_command_queue		m_queue;
	cl_kernel				m_flipFloatKernel;
	cl_kernel				m_scatterKernel ;
	cl_kernel				m_copyAabbsKernel;
	cl_kernel				m_sapKernel;
	cl_kernel				m_sap2Kernel;
	cl_kernel				m_prepareSumVarianceKernel;

	class b3RadixSort32CL* m_sorter;

	///test for 3d SAP
	b3AlignedObjectArray<b3SortData>		m_sortedAxisCPU[3][2];
	int	m_currentBuffer;

	public:
	
	b3OpenCLArray<b3SapAabb>	m_allAabbsGPU;
	b3AlignedObjectArray<b3SapAabb>	m_allAabbsCPU;

	b3OpenCLArray<b3Vector3>	m_sum;
	b3OpenCLArray<b3Vector3>	m_sum2;
	b3OpenCLArray<b3Vector3>	m_dst;

	b3OpenCLArray<b3SapAabb>	m_smallAabbsGPU;
	b3AlignedObjectArray<b3SapAabb>	m_smallAabbsCPU;

	b3OpenCLArray<b3SapAabb>	m_largeAabbsGPU;
	b3AlignedObjectArray<b3SapAabb>	m_largeAabbsCPU;

	b3OpenCLArray<b3Int2>		m_overlappingPairs;

	//temporary gpu work memory
	b3OpenCLArray<b3SortData>	m_gpuSmallSortData;
	b3OpenCLArray<b3SapAabb>	m_gpuSmallSortedAabbs;

	class b3PrefixScanFloat4CL*		m_prefixScanFloat4;

	b3GpuSapBroadphase(cl_context ctx,cl_device_id device, cl_command_queue  q );
	virtual ~b3GpuSapBroadphase();
	
	void  calculateOverlappingPairs(int maxPairs);
	void  calculateOverlappingPairsHost(int maxPairs);
	
	void  reset();

	void init3dSap();
	void calculateOverlappingPairsHostIncremental3Sap();

	void createProxy(const b3Vector3& aabbMin,  const b3Vector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask);
	void createLargeProxy(const b3Vector3& aabbMin,  const b3Vector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask);

	//call writeAabbsToGpu after done making all changes (createProxy etc)
	void writeAabbsToGpu();

	cl_mem	getAabbBufferWS();
	int	getNumOverlap();
	cl_mem	getOverlappingPairBuffer();
};

#endif //B3_GPU_SAP_BROADPHASE_H