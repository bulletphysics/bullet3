/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

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

#include "btCudaBroadphase.h"
#include "radixsort.cuh"



#define BT_GPU_PREF(func) btCuda_##func
#include "../../src/BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#include "../../src/BulletMultiThreaded/btGpu3DGridBroadphaseSharedDefs.h"
#undef BT_GPU_PREF

extern "C" void btCuda_setParameters(bt3DGridBroadphaseParams* hostParams);



#include <stdio.h>




btCudaBroadphase::btCudaBroadphase(	btOverlappingPairCache* overlappingPairCache,
									const btVector3& worldAabbMin,const btVector3& worldAabbMax, 
									int gridSizeX, int gridSizeY, int gridSizeZ, 
									int maxSmallProxies, int maxLargeProxies, int maxPairsPerSmallProxy,
									int maxSmallProxiesPerCell) :
	btGpu3DGridBroadphase(overlappingPairCache, worldAabbMin, worldAabbMax, gridSizeX, gridSizeY, gridSizeZ, maxSmallProxies, maxLargeProxies, maxPairsPerSmallProxy, maxSmallProxiesPerCell)
{
	_initialize();
}



btCudaBroadphase::~btCudaBroadphase()
{
	//btSimpleBroadphase will free memory of btSortedOverlappingPairCache, because m_ownsPairCache
	assert(m_bInitialized);
	_finalize();
}



void btCudaBroadphase::_initialize()
{
    // allocate GPU data
    btCuda_allocateArray((void**)&m_dBodiesHash[0], m_maxHandles * 2 * sizeof(unsigned int));
    btCuda_allocateArray((void**)&m_dBodiesHash[1], m_maxHandles * 2 * sizeof(unsigned int));

	btCuda_allocateArray((void**)&m_dCellStart, m_params.m_numCells * sizeof(unsigned int));

    btCuda_allocateArray((void**)&m_dPairBuff, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));
	btCuda_copyArrayToDevice(m_dPairBuff, m_hPairBuff, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));  // needed?

    btCuda_allocateArray((void**)&m_dPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int));
	btCuda_copyArrayToDevice(m_dPairBuffStartCurr, m_hPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int)); 

	unsigned int numAABB = m_maxHandles + m_maxLargeHandles;
	btCuda_allocateArray((void**)&m_dAABB, numAABB * sizeof(bt3DGrid3F1U) * 2);

    btCuda_allocateArray((void**)&m_dPairScan, (m_maxHandles + 1) * sizeof(unsigned int));

	btCuda_allocateArray((void**)&m_dPairOut, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int));
}



void btCudaBroadphase::_finalize()
{
    assert(m_bInitialized);
    btCuda_freeArray(m_dBodiesHash[0]);
    btCuda_freeArray(m_dBodiesHash[1]);
    btCuda_freeArray(m_dCellStart);
    btCuda_freeArray(m_dPairBuffStartCurr);
    btCuda_freeArray(m_dAABB);
    btCuda_freeArray(m_dPairBuff);
	btCuda_freeArray(m_dPairScan);
	btCuda_freeArray(m_dPairOut);
}



//
// overrides for CUDA version
//



void btCudaBroadphase::prepareAABB()
{
	btGpu3DGridBroadphase::prepareAABB();
	btCuda_copyArrayToDevice(m_dAABB, m_hAABB, sizeof(bt3DGrid3F1U) * 2 * (m_numHandles + m_numLargeHandles)); 
	return;
}



void btCudaBroadphase::setParameters(bt3DGridBroadphaseParams* hostParams)
{
	btCuda_setParameters(hostParams);
	return;
}



void btCudaBroadphase::calcHashAABB()
{
	BT_PROFILE("btCuda_calcHashAABB");
	btCuda_calcHashAABB(m_dAABB, m_dBodiesHash[0], m_numHandles);
//	btCuda_copyArrayFromDevice((void*)m_hBodiesHash, (void*)m_dBodiesHash[0], sizeof(unsigned int) * 2 * m_numHandles);
	return;
}



void btCudaBroadphase::sortHash()
{
	BT_PROFILE("RadixSort-- CUDA");
	RadixSort((KeyValuePair*)m_dBodiesHash[0], (KeyValuePair*)m_dBodiesHash[1], m_numHandles, 32);
	return;
}



void btCudaBroadphase::findCellStart()
{
	BT_PROFILE("btCuda_findCellStart");
	btCuda_findCellStart(m_dBodiesHash[0],	m_dCellStart, m_numHandles, m_params.m_numCells);
//	btCuda_copyArrayFromDevice((void*)m_hBodiesHash, (void*)m_dBodiesHash[0], sizeof(unsigned int) * 2 * m_numHandles);
//	btCuda_copyArrayFromDevice((void*)m_hCellStart, (void*)m_dCellStart, sizeof(unsigned int) * m_params.m_numCells);
	return;
}



void btCudaBroadphase::findOverlappingPairs()
{
	BT_PROFILE("btCuda_findOverlappingPairs");
	btCuda_findOverlappingPairs(m_dAABB, m_dBodiesHash[0], m_dCellStart, m_dPairBuff, m_dPairBuffStartCurr,	m_numHandles);
	return;
}



void btCudaBroadphase::findPairsLarge()
{
	BT_PROFILE("btCuda_findPairsLarge");
	btCuda_findPairsLarge(m_dAABB, m_dBodiesHash[0], m_dCellStart, m_dPairBuff, m_dPairBuffStartCurr,	m_numHandles, m_numLargeHandles);
	return;
}



void btCudaBroadphase::computePairCacheChanges()
{
	BT_PROFILE("btCuda_computePairCacheChanges");
	btCuda_computePairCacheChanges(m_dPairBuff, m_dPairBuffStartCurr, m_dPairScan, m_dAABB, m_numHandles);
	return;
}



void btCudaBroadphase::scanOverlappingPairBuff()
{
	btCuda_copyArrayFromDevice(m_hPairScan, m_dPairScan, sizeof(unsigned int)*(m_numHandles + 1)); 
	btGpu3DGridBroadphase::scanOverlappingPairBuff();
	btCuda_copyArrayToDevice(m_dPairScan, m_hPairScan, sizeof(unsigned int)*(m_numHandles + 1)); 
	return;
}



void btCudaBroadphase::squeezeOverlappingPairBuff()
{
	BT_PROFILE("btCuda_squeezeOverlappingPairBuff");
	btCuda_squeezeOverlappingPairBuff(m_dPairBuff, m_dPairBuffStartCurr, m_dPairScan, m_dPairOut, m_dAABB, m_numHandles);
	btCuda_copyArrayFromDevice(m_hPairOut, m_dPairOut, sizeof(unsigned int) * m_hPairScan[m_numHandles]); 
	return;
}



void btCudaBroadphase::resetPool(btDispatcher* dispatcher)
{
	btGpu3DGridBroadphase::resetPool(dispatcher);
	btCuda_copyArrayToDevice(m_dPairBuffStartCurr, m_hPairBuffStartCurr, (m_maxHandles * 2 + 1) * sizeof(unsigned int)); 
}


