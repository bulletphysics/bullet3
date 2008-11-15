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

//--------------------------------------------------------------------------

#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btQuickprof.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "btCudaBroadphaseKernel.h"
#include "btCudaBroadphase.h"
#include "radixsort.cuh"
//#include "vector_functions.h"

//--------------------------------------------------------------------------

#include <stdio.h>

//--------------------------------------------------------------------------

btCudaBroadphase::btCudaBroadphase(const btVector3& worldAabbMin,const btVector3& worldAabbMax, 
								   int gridSizeX, int gridSizeY, int gridSizeZ, 
								   int maxSmallProxies, int maxLargeProxies, int maxPairsPerBody,
								   int maxBodiesPerCell,
								   btScalar cellFactorAABB) :
	btSimpleBroadphase(maxSmallProxies,
//				     new (btAlignedAlloc(sizeof(btSortedOverlappingPairCache),16)) btSortedOverlappingPairCache),
				     new (btAlignedAlloc(sizeof(btHashedOverlappingPairCache),16)) btHashedOverlappingPairCache),
	m_bInitialized(false),
    m_numBodies(0)
{
	m_ownsPairCache = true;
	m_params.m_gridSizeX = gridSizeX;
	m_params.m_gridSizeY = gridSizeY;
	m_params.m_gridSizeZ = gridSizeZ;
	m_params.m_numCells = m_params.m_gridSizeX * m_params.m_gridSizeY * m_params.m_gridSizeZ;
	btVector3 w_org = worldAabbMin;
	m_params.m_worldOriginX = w_org.getX();
	m_params.m_worldOriginY = w_org.getY();
	m_params.m_worldOriginZ = w_org.getZ();
	btVector3 w_size = worldAabbMax - worldAabbMin;
	m_params.m_cellSizeX = w_size.getX() / m_params.m_gridSizeX;
	m_params.m_cellSizeY = w_size.getY() / m_params.m_gridSizeY;
	m_params.m_cellSizeZ = w_size.getZ() / m_params.m_gridSizeZ;
	m_maxRadius = btMin(btMin(m_params.m_cellSizeX, m_params.m_cellSizeY), m_params.m_cellSizeZ);
	m_maxRadius *= btScalar(0.5f);
	m_params.m_numBodies = m_numBodies;
	m_params.m_maxBodiesPerCell = maxBodiesPerCell;

	m_numLargeHandles = 0;						
	m_maxLargeHandles = maxLargeProxies;

	m_maxPairsPerBody = maxPairsPerBody;

	m_cellFactorAABB = cellFactorAABB;

	_initialize();
} // btCudaBroadphase::btCudaBroadphase()

//--------------------------------------------------------------------------

btCudaBroadphase::~btCudaBroadphase()
{
	//btSimpleBroadphase will free memory of btSortedOverlappingPairCache, because m_ownsPairCache
	assert(m_bInitialized);
	_finalize();
} // btCudaBroadphase::~btCudaBroadphase()

//--------------------------------------------------------------------------

void btCudaBroadphase::_initialize()
{
    assert(!m_bInitialized);
    // allocate host storage
    m_hBodiesHash = new unsigned int[m_maxHandles * 2];
    memset(m_hBodiesHash, 0x00, m_maxHandles*2*sizeof(unsigned int));

    m_hCellStart = new unsigned int[m_params.m_numCells];
    memset(m_hCellStart, 0x00, m_params.m_numCells * sizeof(unsigned int));

	m_hPairBuffStartCurr = new unsigned int[m_maxHandles * 2 + 2];
	// --------------- for now, init with m_maxPairsPerBody for each body
	m_hPairBuffStartCurr[0] = 0;
	m_hPairBuffStartCurr[1] = 0;
	for(int i = 1; i <= m_maxHandles; i++) 
	{
		m_hPairBuffStartCurr[i * 2] = m_hPairBuffStartCurr[(i-1) * 2] + m_maxPairsPerBody;
		m_hPairBuffStartCurr[i * 2 + 1] = 0;
	}
	//----------------
	unsigned int numAABB = m_maxHandles + m_maxLargeHandles;
	m_hAABB = new btCuda3F1U[numAABB * 2]; // AABB Min & Max

	m_hPairBuff = new unsigned int[m_maxHandles * m_maxPairsPerBody];
	memset(m_hPairBuff, 0x00, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int)); // needed?

	m_hPairScan = new unsigned int[m_maxHandles + 1];

	m_hPairOut = new unsigned int[m_maxHandles * m_maxPairsPerBody];

    // allocate GPU data
    btCuda_allocateArray((void**)&m_dBodiesHash[0], m_maxHandles * 2 * sizeof(unsigned int));
    btCuda_allocateArray((void**)&m_dBodiesHash[1], m_maxHandles * 2 * sizeof(unsigned int));

	btCuda_allocateArray((void**)&m_dCellStart, m_params.m_numCells * sizeof(unsigned int));

    btCuda_allocateArray((void**)&m_dPairBuff, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));
	btCuda_copyArrayToDevice(m_dPairBuff, m_hPairBuff, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));  // needed?

    btCuda_allocateArray((void**)&m_dPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int));
	btCuda_copyArrayToDevice(m_dPairBuffStartCurr, m_hPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int)); 

	btCuda_allocateArray((void**)&m_dAABB, numAABB * sizeof(btCuda3F1U) * 2);

    btCuda_allocateArray((void**)&m_dPairScan, (m_maxHandles + 1) * sizeof(unsigned int));

	btCuda_allocateArray((void**)&m_dPairOut, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));

    btCuda_setParameters(&m_params);

// large proxies

	// allocate handles buffer and put all handles on free list
	m_pLargeHandlesRawPtr = btAlignedAlloc(sizeof(btSimpleBroadphaseProxy) * m_maxLargeHandles, 16);
	m_pLargeHandles = new(m_pLargeHandlesRawPtr) btSimpleBroadphaseProxy[m_maxLargeHandles];
	m_firstFreeLargeHandle = 0;
	{
		for (int i = m_firstFreeLargeHandle; i < m_maxLargeHandles; i++)
		{
			m_pLargeHandles[i].SetNextFree(i + 1);
			m_pLargeHandles[i].m_uniqueId = m_maxHandles+2+i;
		}
		m_pLargeHandles[m_maxLargeHandles - 1].SetNextFree(0);
	}

// debug data
	m_numPairsAdded = 0;
	m_numOverflows = 0;

    m_bInitialized = true;
} // btCudaBroadphase::_initialize()

//--------------------------------------------------------------------------

void btCudaBroadphase::_finalize()
{
    assert(m_bInitialized);
    delete [] m_hBodiesHash;
    delete [] m_hCellStart;
    delete [] m_hPairBuffStartCurr;
    delete [] m_hAABB;
	delete [] m_hPairBuff;
	delete [] m_hPairScan;
	delete [] m_hPairOut;
    btCuda_freeArray(m_dBodiesHash[0]);
    btCuda_freeArray(m_dBodiesHash[1]);
    btCuda_freeArray(m_dCellStart);
    btCuda_freeArray(m_dPairBuffStartCurr);
    btCuda_freeArray(m_dAABB);
    btCuda_freeArray(m_dPairBuff);
	btCuda_freeArray(m_dPairScan);
	btCuda_freeArray(m_dPairOut);

	btAlignedFree(m_pLargeHandlesRawPtr);

	m_bInitialized = false;
} // btCudaBroadphase::_finalize()

//--------------------------------------------------------------------------

void btCudaBroadphase::calculateOverlappingPairs(btDispatcher* dispatcher)
{
	if(m_numHandles <= 0)
	{
		BT_PROFILE("addLarge2LargePairsToCache -- CPU");
		addLarge2LargePairsToCache(dispatcher);
		return;
	}
	// update constants
	btCuda_setParameters(&m_params);
	// move AABB array to GPU
	{
		BT_PROFILE("copy AABB");
		// do it faster ? 
		btCuda3F1U* pBB = m_hAABB;
		int i;
		int new_largest_index = -1;
		unsigned int num_small = 0;
		for(i = 0; i <= m_LastHandleIndex; i++) 
		{
			btSimpleBroadphaseProxy* proxy0 = &m_pHandles[i];
			if(!proxy0->m_clientObject)
			{
				continue;
			}
			new_largest_index = i;
			pBB->fx = proxy0->m_aabbMin.getX();
			pBB->fy = proxy0->m_aabbMin.getY();
			pBB->fz = proxy0->m_aabbMin.getZ();
			pBB->uw = i;
			pBB++;
			pBB->fx = proxy0->m_aabbMax.getX();
			pBB->fy = proxy0->m_aabbMax.getY();
			pBB->fz = proxy0->m_aabbMax.getZ();
			pBB->uw = num_small;
			pBB++;
			num_small++;
		}
		m_LastHandleIndex = new_largest_index;
		new_largest_index = -1;
		unsigned int num_large = 0;
		for(i = 0; i <= m_LastLargeHandleIndex; i++) 
		{
			btSimpleBroadphaseProxy* proxy0 = &m_pLargeHandles[i];
			if(!proxy0->m_clientObject)
			{
				continue;
			}
			new_largest_index = i;
			pBB->fx = proxy0->m_aabbMin.getX();
			pBB->fy = proxy0->m_aabbMin.getY();
			pBB->fz = proxy0->m_aabbMin.getZ();
			pBB->uw = i + m_maxHandles;
			pBB++;
			pBB->fx = proxy0->m_aabbMax.getX();
			pBB->fy = proxy0->m_aabbMax.getY();
			pBB->fz = proxy0->m_aabbMax.getZ();
			pBB->uw = num_large + m_maxHandles;
			pBB++;
			num_large++;
		}
		m_LastLargeHandleIndex = new_largest_index;
		// paranoid checks
		btAssert(num_small == m_numHandles);
		btAssert(num_large == m_numLargeHandles);
	}
	{
		BT_PROFILE("CopyBB to CUDA");
		btCuda_copyArrayToDevice(m_dAABB, m_hAABB, sizeof(btCuda3F1U) * 2 * (m_numHandles + m_numLargeHandles)); 
	}
	// calculate hash
	{
		BT_PROFILE("calcHash -- CUDA");
		btCuda_calcHashAABB(m_dAABB, m_dBodiesHash[0], m_numHandles);
	}
//	btCuda_copyArrayFromDevice((void*)m_hBodiesHash, (void*)m_dBodiesHash[0], sizeof(unsigned int) * 2 * m_numHandles);
	// sort bodies based on hash
	{
		BT_PROFILE("RadixSort-- CUDA");
		RadixSort((KeyValuePair*)m_dBodiesHash[0], (KeyValuePair*)m_dBodiesHash[1], m_numHandles, 32);
	}
	// find start of each cell
	{
		BT_PROFILE("Find cell start -- CUDA");
		btCuda_findCellStart(m_dBodiesHash[0],	m_dCellStart, m_numHandles, m_params.m_numCells);
	}
//	btCuda_copyArrayFromDevice((void*)m_hBodiesHash, (void*)m_dBodiesHash[0], sizeof(unsigned int) * 2 * m_numHandles);
//	btCuda_copyArrayFromDevice((void*)m_hCellStart, (void*)m_dCellStart, sizeof(unsigned int) * m_params.m_numCells);
	{
		BT_PROFILE("FindOverlappingPairs -- CUDA");
		btCuda_findOverlappingPairs(m_dAABB, m_dBodiesHash[0], m_dCellStart, m_dPairBuff, m_dPairBuffStartCurr,	m_numHandles);
	}
	{
		BT_PROFILE("FindPairsLarge -- CUDA");
		btCuda_findPairsLarge(m_dAABB, m_dBodiesHash[0], m_dCellStart, m_dPairBuff, m_dPairBuffStartCurr,	m_numHandles, m_numLargeHandles);
	}
	{
		BT_PROFILE("ComputePairCacheChanges -- CUDA");
		btCuda_computePairCacheChanges(m_dPairBuff, m_dPairBuffStartCurr, m_dPairScan, m_dAABB, m_numHandles);
	}
	{
		BT_PROFILE("scanOverlappingPairBuff -- CPU");
		btCuda_copyArrayFromDevice(m_hPairScan, m_dPairScan, sizeof(unsigned int)*(m_numHandles + 1)); 
		scanOverlappingPairBuffCPU();
		btCuda_copyArrayToDevice(m_dPairScan, m_hPairScan, sizeof(unsigned int)*(m_numHandles + 1)); 
	}
	{
		BT_PROFILE("SqueezeOverlappingPairBuff -- CUDA");
		btCuda_squeezeOverlappingPairBuff(m_dPairBuff, m_dPairBuffStartCurr, m_dPairScan, m_dPairOut, m_dAABB, m_numHandles);
	}
	{
		BT_PROFILE("SqueezeOverlappingPairBuff -- CUDA");
		btCuda_copyArrayFromDevice(m_hPairOut, m_dPairOut, sizeof(unsigned int) * m_hPairScan[m_numHandles]); 
	}
	{
		BT_PROFILE("addPairsToCache -- CPU");
		addPairsToCacheCPU(dispatcher);
	}
	{
		BT_PROFILE("addLarge2LargePairsToCache -- CPU");
		addLarge2LargePairsToCache(dispatcher);
	}
	return;
} // btCudaBroadphase::calculateOverlappingPairs()

//--------------------------------------------------------------------------

void btCudaBroadphase::scanOverlappingPairBuffCPU()
{
	m_hPairScan[0] = 0;
	for(int i = 1; i <= m_numHandles; i++) 
	{
		unsigned int delta = m_hPairScan[i];
		m_hPairScan[i] = m_hPairScan[i-1] + delta;
	}
} // btCudaBroadphase::scanOverlappingPairBuffCPU()

//--------------------------------------------------------------------------

void btCudaBroadphase::addPairsToCacheCPU(btDispatcher* dispatcher)
{
	m_numPairsAdded = 0;
	m_numPairsRemoved = 0;
	for(int i = 0; i < m_numHandles; i++) 
	{
		unsigned int num = m_hPairScan[i+1] - m_hPairScan[i];
		if(!num)
		{
			continue;
		}
		unsigned int* pInp = m_hPairOut + m_hPairScan[i];
		unsigned int index0 = m_hAABB[i * 2].uw;
		btSimpleBroadphaseProxy* proxy0 = &m_pHandles[index0];
		for(unsigned int j = 0; j < num; j++)
		{
			unsigned int indx1_s = pInp[j];
			unsigned int index1 = indx1_s & (~BT_CUDA_PAIR_ANY_FLG);
			btSimpleBroadphaseProxy* proxy1;
			if(index1 < (unsigned int)m_maxHandles)
			{
				proxy1 = &m_pHandles[index1];
			}
			else
			{
				index1 -= m_maxHandles;
				btAssert((index1 >= 0) && (index1 < (unsigned int)m_maxLargeHandles));
				proxy1 = &m_pLargeHandles[index1];
			}
			if(indx1_s & BT_CUDA_PAIR_NEW_FLG)
			{
				m_pairCache->addOverlappingPair(proxy0,proxy1);
				m_numPairsAdded++;
			}
			else
			{
				m_pairCache->removeOverlappingPair(proxy0,proxy1,dispatcher);
				m_numPairsRemoved++;
			}
		}
	}
} // btCudaBroadphase::addPairsToCacheCPU()

//--------------------------------------------------------------------------

btBroadphaseProxy* btCudaBroadphase::createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy)
{
	btBroadphaseProxy*  proxy;
	bool bIsLarge = isLargeProxy(aabbMin, aabbMax);
	if(bIsLarge)
	{
		if (m_numLargeHandles >= m_maxLargeHandles)
		{
			btAssert(0);
			return 0; //should never happen, but don't let the game crash ;-)
		}
		btAssert((aabbMin[0]<= aabbMax[0]) && (aabbMin[1]<= aabbMax[1]) && (aabbMin[2]<= aabbMax[2]));
		int newHandleIndex = allocLargeHandle();
		proxy = new (&m_pLargeHandles[newHandleIndex])btSimpleBroadphaseProxy(aabbMin,aabbMax,shapeType,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy);
	}
	else
	{
		proxy = btSimpleBroadphase::createProxy(aabbMin, aabbMax, shapeType, userPtr, collisionFilterGroup, collisionFilterMask, dispatcher, multiSapProxy);
	}
	return proxy;
} // btCudaBroadphase::createProxy()

//--------------------------------------------------------------------------

void btCudaBroadphase::destroyProxy(btBroadphaseProxy* proxy, btDispatcher* dispatcher)
{
	bool bIsLarge = isLargeProxy(proxy);
	if(bIsLarge)
	{
		
		btSimpleBroadphaseProxy* proxy0 = static_cast<btSimpleBroadphaseProxy*>(proxy);
		freeLargeHandle(proxy0);
		// TODO : remove pair from cache on GPU as well !!!
		// UPD: they will not be used anyway, so don't waste time
		m_pairCache->removeOverlappingPairsContainingProxy(proxy,dispatcher);
	}
	else
	{
		btSimpleBroadphase::destroyProxy(proxy, dispatcher);
	}
	return;
} // btCudaBroadphase::destroyProxy()

//--------------------------------------------------------------------------

bool btCudaBroadphase::isLargeProxy(const btVector3& aabbMin,  const btVector3& aabbMax)
{
	btVector3 diag = aabbMax - aabbMin;
	btScalar radius = diag.length() * btScalar(0.5f);

	radius *= m_cellFactorAABB; // user-defined factor

	return (radius > m_maxRadius);
} // btCudaBroadphase::isLargeProxy()

//--------------------------------------------------------------------------

bool btCudaBroadphase::isLargeProxy(btBroadphaseProxy* proxy)
{
	return (proxy->getUid() >= (m_maxHandles+2));
} // btCudaBroadphase::isLargeProxy()

//--------------------------------------------------------------------------

void btCudaBroadphase::addLarge2LargePairsToCache(btDispatcher* dispatcher)
{
	int i,j;
	if (m_numLargeHandles <= 0)
	{
		return;
	}
	int new_largest_index = -1;
	for(i = 0; i <= m_LastLargeHandleIndex; i++)
	{
		btSimpleBroadphaseProxy* proxy0 = &m_pLargeHandles[i];
		if(!proxy0->m_clientObject)
		{
			continue;
		}
		new_largest_index = i;
		for(j = i + 1; j <= m_LastLargeHandleIndex; j++)
		{
			btSimpleBroadphaseProxy* proxy1 = &m_pLargeHandles[j];
			if(!proxy1->m_clientObject)
			{
				continue;
			}
			btAssert(proxy0 != proxy1);
			btSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
			btSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);
			if(aabbOverlap(p0,p1))
			{
				if (!m_pairCache->findPair(proxy0,proxy1))
				{
					m_pairCache->addOverlappingPair(proxy0,proxy1);
				}
			} 
			else
			{
				if(m_pairCache->findPair(proxy0,proxy1))
				{
					m_pairCache->removeOverlappingPair(proxy0,proxy1,dispatcher);
				}
			}
		}
	}
	m_LastLargeHandleIndex = new_largest_index;
	return;
} // btCudaBroadphase::addLarge2LargePairsToCache()

//--------------------------------------------------------------------------

void btCudaBroadphase::rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback)
{
	btSimpleBroadphase::rayTest(rayFrom, rayTo, rayCallback);
	for (int i=0; i <= m_LastLargeHandleIndex; i++)
	{
		btSimpleBroadphaseProxy* proxy = &m_pLargeHandles[i];
		if(!proxy->m_clientObject)
		{
			continue;
		}
		rayCallback.process(proxy);
	}
} // btCudaBroadphase::rayTest()

//--------------------------------------------------------------------------
