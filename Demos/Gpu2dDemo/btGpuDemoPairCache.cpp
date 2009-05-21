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



#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

#include "btGpuDemoPairCache.h"

#include "btGpuDemoDynamicsWorld.h"



void btGpuDemoPairCache::processAllOverlappingPairs(btOverlapCallback* callback, btDispatcher* dispatcher)
{
	int sz = m_maxProxies * m_maxNeighbors;
	int numContConstraints = 0;
	int2* pIds = gpCudaDemoDynamicsWorld->getIdsPtr();
	for(int idx = 0; idx < sz; idx++)
	{
		int neigh  = m_hNeighbors[idx];
		if(neigh >= 0)
		{
			int i=idx / m_maxNeighbors;
			int j=idx % m_maxNeighbors;
			pIds[numContConstraints].x = i;
			pIds[numContConstraints].y = m_hNeighbors[i * m_maxNeighbors + j];
			numContConstraints++;
		}
	}
	gpCudaDemoDynamicsWorld->setTotalNumConstraints(numContConstraints);	
} 



// this will be called for each overlapping pair if collision detection uses pairCache other than btGpuDemoPairCache
// IMPORTANT : m_numConstraints in gpCudaDemoDynamicsWorld is set to 0 at start of simulation step
// IMPORTANT : companionIds for all objects should be properly set at start of simulation step
void cudaDemoNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)
{
	btCollisionObject* colObj0 = (btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
	btCollisionObject* colObj1 = (btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;
	if (dispatcher.needsCollision(colObj0,colObj1))
	{
		// int id0 = collisionPair.m_pProxy0->m_uniqueId - 2;
		// int id1 = collisionPair.m_pProxy1->m_uniqueId - 2;
		// cannot use m_uniqueId : it may be altered by broadphase code
		// so we'll use companionIds set on the initialization stage
		unsigned int id0 = colObj0->getCompanionId();
		unsigned int id1 = colObj1->getCompanionId();
		if(id0 > id1)
		{
			int tmp = id0; id0 = id1; id1 = tmp;
		}
		int totalNumConstraints = gpCudaDemoDynamicsWorld->getTotalNumConstraints();
		int2* pIds = gpCudaDemoDynamicsWorld->getIdsPtr();
		pIds += totalNumConstraints;
		pIds->x = id0;
		pIds->y = id1;
		totalNumConstraints++;
		gpCudaDemoDynamicsWorld->setTotalNumConstraints(totalNumConstraints);	
	}
} // cudaDemoNearCallback()

