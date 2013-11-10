
#include "b3GpuGridBroadphase.h"
#include "Bullet3Geometry/b3AabbUtil.h"

b3GpuGridBroadphase::b3GpuGridBroadphase(cl_context ctx,cl_device_id device, cl_command_queue  q )
:m_context(ctx),
m_device(device),
m_queue(q),
m_allAabbsGPU(ctx,q),
m_gpuPairs(ctx,q)
{
}
b3GpuGridBroadphase::~b3GpuGridBroadphase()
{
}



void b3GpuGridBroadphase::createProxy(const b3Vector3& aabbMin,  const b3Vector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask)
{
	b3SapAabb aabb;
	aabb.m_minVec = aabbMin;
	aabb.m_maxVec = aabbMax;
	aabb.m_minIndices[3] = userPtr;
	aabb.m_signedMaxIndices[3] = userPtr;
	m_allAabbsCPU.push_back(aabb);
}
void b3GpuGridBroadphase::createLargeProxy(const b3Vector3& aabbMin,  const b3Vector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask)
{
	createProxy(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask);
	
}

void  b3GpuGridBroadphase::calculateOverlappingPairs(int maxPairs)
{
	calculateOverlappingPairsHost(maxPairs);
}
void  b3GpuGridBroadphase::calculateOverlappingPairsHost(int maxPairs)
{
	m_hostPairs.resize(0);
		
	for (int i=0;i<m_allAabbsCPU.size();i++)
	{
		for (int j=i+1;j<m_allAabbsCPU.size();j++)
		{
			if (b3TestAabbAgainstAabb2(m_allAabbsCPU[i].m_minVec, m_allAabbsCPU[i].m_maxVec,
				m_allAabbsCPU[j].m_minVec,m_allAabbsCPU[j].m_maxVec))
			{
				b3Int4 pair;
				int a = m_allAabbsCPU[j].m_minIndices[3];
				int b = m_allAabbsCPU[i].m_minIndices[3];
				if (a<=b)
				{
					pair.x = a; 
					pair.y = b;//store the original index in the unsorted aabb array
				} else
				{
					pair.x = b;
					pair.y = a;//store the original index in the unsorted aabb array
				}
					
				m_hostPairs.push_back(pair);
			}
		}
	}


	m_gpuPairs.copyFromHost(m_hostPairs);
}

	//call writeAabbsToGpu after done making all changes (createProxy etc)
void b3GpuGridBroadphase::writeAabbsToGpu()
{
	m_allAabbsGPU.copyFromHost(m_allAabbsCPU);
}

cl_mem	b3GpuGridBroadphase::getAabbBufferWS()
{
	return this->m_allAabbsGPU.getBufferCL();
}
int	b3GpuGridBroadphase::getNumOverlap()
{
	return m_gpuPairs.size();
}
cl_mem	b3GpuGridBroadphase::getOverlappingPairBuffer()
{
	return m_gpuPairs.getBufferCL();
}

b3OpenCLArray<b3SapAabb>&	b3GpuGridBroadphase::getAllAabbsGPU()
{
	return m_allAabbsGPU;
}

b3AlignedObjectArray<b3SapAabb>&	b3GpuGridBroadphase::getAllAabbsCPU()
{
	return m_allAabbsCPU;
}