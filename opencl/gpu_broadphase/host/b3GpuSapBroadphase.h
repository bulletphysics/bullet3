#ifndef BT_GPU_SAP_BROADPHASE_H
#define BT_GPU_SAP_BROADPHASE_H

#include "parallel_primitives/host/btOpenCLArray.h"
#include "parallel_primitives/host/btFillCL.h" //btInt2
class btVector3;
#include "parallel_primitives/host/btRadixSort32CL.h"

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

	class btRadixSort32CL* m_sorter;

	///test for 3d SAP
	btAlignedObjectArray<btSortData>		m_sortedAxisCPU[3][2];
	int	m_currentBuffer;

	public:
	
	btOpenCLArray<b3SapAabb>	m_allAabbsGPU;
	btAlignedObjectArray<b3SapAabb>	m_allAabbsCPU;

	btOpenCLArray<b3SapAabb>	m_smallAabbsGPU;
	btAlignedObjectArray<b3SapAabb>	m_smallAabbsCPU;

	btOpenCLArray<b3SapAabb>	m_largeAabbsGPU;
	btAlignedObjectArray<b3SapAabb>	m_largeAabbsCPU;

	btOpenCLArray<btInt2>		m_overlappingPairs;

	//temporary gpu work memory
	btOpenCLArray<btSortData>	m_gpuSmallSortData;
	btOpenCLArray<b3SapAabb>	m_gpuSmallSortedAabbs;


	b3GpuSapBroadphase(cl_context ctx,cl_device_id device, cl_command_queue  q );
	virtual ~b3GpuSapBroadphase();
	
	void  calculateOverlappingPairs();
	void  calculateOverlappingPairsHost();

	void init3dSap();
	void calculateOverlappingPairsHostIncremental3Sap();

	void createProxy(const btVector3& aabbMin,  const btVector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask);
	void createLargeProxy(const btVector3& aabbMin,  const btVector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask);

	//call writeAabbsToGpu after done making all changes (createProxy etc)
	void writeAabbsToGpu();

	cl_mem	getAabbBufferWS();
	int	getNumOverlap();
	cl_mem	getOverlappingPairBuffer();
};

#endif //BT_GPU_SAP_BROADPHASE_H