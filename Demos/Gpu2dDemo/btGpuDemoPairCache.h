/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CUDA_DEMO_PAIR_CACHE_H
#define CUDA_DEMO_PAIR_CACHE_H


#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "LinearMath/btAlignedObjectArray.h"

class btGpuDemoPairCache : public btNullPairCache
{
public:
	int		m_maxProxies;
	int		m_maxNeighbors;
	int*	m_hNeighbors;
	int		m_numPairs;
	int		m_numSmallProxies;
	int		m_maxSmallProxies;

	btGpuDemoPairCache(int maxProxies, int maxNeighbors, int maxSmallProxies)
	{
		m_maxProxies = maxProxies;
		m_maxNeighbors = maxNeighbors;
		m_maxSmallProxies = maxSmallProxies;
		int sz = maxProxies * maxNeighbors;
		m_hNeighbors = new int [sz];
		reset();
	}

	~btGpuDemoPairCache()
	{
		delete [] m_hNeighbors;
	}
	
	void reset(void)
	{
		int sz = m_maxProxies * m_maxNeighbors;
		for(int i = 0; i < sz; i++)
		{
			m_hNeighbors[i] = -1;
		}
		m_numPairs = 0;
	}

	virtual int getNumOverlappingPairs() const
	{
		return 	m_numPairs;
		//return 0; // to skip btSimulationIslandManager::findUnions()
	}

	virtual void	processAllOverlappingPairs(btOverlapCallback* callback, btDispatcher* dispatcher);

	virtual btBroadphasePair*	addOverlappingPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1)
	{
		int id0 = proxy0->m_uniqueId - 2;
		int id1 = proxy1->m_uniqueId - 2;
		if(id0 >= m_maxSmallProxies)
		{
			id0 -= m_maxSmallProxies - m_numSmallProxies;
		}
		if(id1 >= m_maxSmallProxies)
		{
			id1 -= m_maxSmallProxies - m_numSmallProxies;
		}
		if(id0 > id1)
		{
			int tmp = id0; id0 = id1; id1 = tmp;
		}
		int offs = id0 * m_maxNeighbors;
		int i;
		for(i = 0; i < m_maxNeighbors; i++)
		{
			if(m_hNeighbors[offs + i] < 0)
			{
				m_hNeighbors[offs + i] = id1;
				m_numPairs++;
				break;
			}
		}
		// btAssert(i < m_maxNeighbors);
		return 0;
	}

	virtual void*	removeOverlappingPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1,btDispatcher* /*dispatcher*/)
	{
		int id0 = proxy0->m_uniqueId - 2;
		int id1 = proxy1->m_uniqueId - 2;
		if(id0 >= m_maxSmallProxies)
		{
			id0 -= m_maxSmallProxies - m_numSmallProxies;
		}
		if(id1 >= m_maxSmallProxies)
		{
			id1 -= m_maxSmallProxies - m_numSmallProxies;
		}
		if(id0 > id1)
		{
			int tmp = id0; id0 = id1; id1 = tmp;
		}
		int offs = id0 * m_maxNeighbors;
		int i;
		for(i = 0; i < m_maxNeighbors; i++)
		{
			if(m_hNeighbors[offs + i] == id1)
			{
				m_hNeighbors[offs + i] = -1;
				m_numPairs--;
				break;
			}
		}
//		btAssert(i < m_maxNeighbors);
		return 0;
	}
};

extern void cudaDemoNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo);

#endif //CUDA_DEMO_PAIR_CACHE_H


