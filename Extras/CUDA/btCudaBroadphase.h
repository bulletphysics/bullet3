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



#ifndef CUDA_BROADPHASE_H
#define CUDA_BROADPHASE_H



#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"

#include "../../src/BulletMultiThreaded/btGpu3DGridBroadphaseSharedTypes.h"
#include "../../src/BulletMultiThreaded/btGpu3DGridBroadphase.h"



///The btCudaBroadphase uses CUDA-capable GPU to compute overlapping pairs

class btCudaBroadphase : public btGpu3DGridBroadphase
{
protected:
    // GPU data
    unsigned int*	m_dBodiesHash[2];
    unsigned int*	m_dCellStart;
	unsigned int*	m_dPairBuff; 
	unsigned int*	m_dPairBuffStartCurr;
	bt3DGrid3F1U*		m_dAABB;
	unsigned int*	m_dPairScan;
	unsigned int*	m_dPairOut;
public:
	btCudaBroadphase(	btOverlappingPairCache* overlappingPairCache,
						const btVector3& worldAabbMin,const btVector3& worldAabbMax, 
						int gridSizeX, int gridSizeY, int gridSizeZ, 
						int maxSmallProxies, int maxLargeProxies, int maxPairsPerSmallProxies,
						int maxSmallProxiesPerCell = 8);
	virtual ~btCudaBroadphase();
protected:
	void _initialize();
	void _finalize();
	void allocateArray(void** devPtr, unsigned int size);
	void freeArray(void* devPtr);
// overrides for CUDA version
	virtual void setParameters(bt3DGridBroadphaseParams* hostParams);
	virtual void prepareAABB();
	virtual void calcHashAABB();
	virtual void sortHash();	
	virtual void findCellStart();
	virtual void findOverlappingPairs();
	virtual void findPairsLarge();
	virtual void computePairCacheChanges();
	virtual void scanOverlappingPairBuff();
	virtual void squeezeOverlappingPairBuff();
	virtual void resetPool(btDispatcher* dispatcher);
};

#endif //CUDA_BROADPHASE_H