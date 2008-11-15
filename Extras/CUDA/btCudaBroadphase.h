/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CUDA_BROADPHASE_H
#define CUDA_BROADPHASE_H

#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"

#include "btCudaBroadphaseKernel.h"


///The btCudaBroadphase uses CUDA to compute overlapping pairs using a GPU.
class btCudaBroadphase : public btSimpleBroadphase
{
	bool			m_bInitialized;
    unsigned int	m_numBodies;
    unsigned int	m_numCells;
	unsigned int	m_maxPairsPerBody;
	btScalar		m_cellFactorAABB;
	// CPU data
    unsigned int*	m_hBodiesHash;
    unsigned int*	m_hCellStart;
	unsigned int*	m_hPairBuffStartCurr;
	btCuda3F1U*		m_hAABB;
	unsigned int*	m_hPairBuff;
	unsigned int*	m_hPairScan;
	unsigned int*	m_hPairOut;
    // GPU data
    unsigned int*	m_dBodiesHash[2];
    unsigned int*	m_dCellStart;
	unsigned int*	m_dPairBuff; 
	unsigned int*	m_dPairBuffStartCurr;
	btCuda3F1U*		m_dAABB;
	unsigned int*	m_dPairScan;
	unsigned int*	m_dPairOut;
    unsigned int	m_maxBodiesPerCell;
	btCudaBroadphaseParams m_params;
	btScalar		m_maxRadius;
// large proxies
	int		m_numLargeHandles;						
	int		m_maxLargeHandles;						
	int		m_LastLargeHandleIndex;							
	btSimpleBroadphaseProxy* m_pLargeHandles;
	void* m_pLargeHandlesRawPtr;
	int		m_firstFreeLargeHandle;
	int allocLargeHandle()
	{
		btAssert(m_numLargeHandles < m_maxLargeHandles);
		int freeLargeHandle = m_firstFreeLargeHandle;
		m_firstFreeLargeHandle = m_pLargeHandles[freeLargeHandle].GetNextFree();
		m_numLargeHandles++;
		if(freeLargeHandle > m_LastLargeHandleIndex)
		{
			m_LastLargeHandleIndex = freeLargeHandle;
		}
		return freeLargeHandle;
	}
	void freeLargeHandle(btSimpleBroadphaseProxy* proxy)
	{
		int handle = int(proxy - m_pLargeHandles);
		btAssert((handle >= 0) && (handle < m_maxHandles));
		if(handle == m_LastLargeHandleIndex)
		{
			m_LastLargeHandleIndex--;
		}
		proxy->SetNextFree(m_firstFreeLargeHandle);
		m_firstFreeLargeHandle = handle;
		proxy->m_clientObject = 0;
		m_numLargeHandles--;
	}
	bool isLargeProxy(const btVector3& aabbMin,  const btVector3& aabbMax);
	bool isLargeProxy(btBroadphaseProxy* proxy);

// debug
	unsigned int	m_numPairsAdded;
	unsigned int	m_numPairsRemoved;
	unsigned int	m_numOverflows;
// 
public:
	btCudaBroadphase(const btVector3& worldAabbMin,const btVector3& worldAabbMax, 
								   int gridSizeX, int gridSizeY, int gridSizeZ, 
								   int maxSmallProxies, int maxLargeProxies, int maxPairsPerBody,
								   int maxBodiesPerCell = 8,
								   btScalar cellFactorAABB = btScalar(1.0f));
	virtual ~btCudaBroadphase();
	virtual void	calculateOverlappingPairs(btDispatcher* dispatcher);

	virtual btBroadphaseProxy*	createProxy(const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy);
	virtual void	destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);
	virtual void	rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback);


protected:
	void _initialize();
	void _finalize();
	void scanOverlappingPairBuffCPU();
	void addPairsToCacheCPU(btDispatcher* dispatcher);
	void addLarge2LargePairsToCache(btDispatcher* dispatcher);
};
#endif //CUDA_BROADPHASE_H