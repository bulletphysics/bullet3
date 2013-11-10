#ifndef B3_GPU_GRID_BROADPHASE_H
#define B3_GPU_GRID_BROADPHASE_H

#include "b3GpuBroadphaseInterface.h"

class b3GpuGridBroadphase : public b3GpuBroadphaseInterface
{
protected:
	cl_context				m_context;
	cl_device_id			m_device;
	cl_command_queue		m_queue;

	b3OpenCLArray<b3SapAabb>	m_allAabbsGPU;
	b3AlignedObjectArray<b3SapAabb>	m_allAabbsCPU;

	b3AlignedObjectArray<b3Int4> m_hostPairs;
	b3OpenCLArray<b3Int4>			m_gpuPairs;

public:

	b3GpuGridBroadphase(cl_context ctx,cl_device_id device, cl_command_queue  q );
	virtual ~b3GpuGridBroadphase();



	virtual void createProxy(const b3Vector3& aabbMin,  const b3Vector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask);
	virtual void createLargeProxy(const b3Vector3& aabbMin,  const b3Vector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask);

	virtual void  calculateOverlappingPairs(int maxPairs);
	virtual void  calculateOverlappingPairsHost(int maxPairs);

	//call writeAabbsToGpu after done making all changes (createProxy etc)
	virtual void writeAabbsToGpu();

	virtual cl_mem	getAabbBufferWS();
	virtual int	getNumOverlap();
	virtual cl_mem	getOverlappingPairBuffer();

	virtual b3OpenCLArray<b3SapAabb>&	getAllAabbsGPU();
	virtual b3AlignedObjectArray<b3SapAabb>&	getAllAabbsCPU();

};

#endif //B3_GPU_GRID_BROADPHASE_H