
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



#include "btOverlappingPairCache.h"

#include "btDispatcher.h"
#include "btCollisionAlgorithm.h"

int	gOverlappingPairs = 0;

btOverlappingPairCache::btOverlappingPairCache():
m_blockedForChanges(false),
m_overlapFilterCallback(0)
//m_NumOverlapBroadphasePair(0)
{

#ifdef USE_HASH_PAIRCACHE
	m_overlappingPairArray.reserve(b2_maxPairs);

	for (int32 i = 0; i < b2_tableCapacity; ++i)
	{
		m_hashTable[i] = b2_nullPair;
	}
	for (int32 i = 0; i < b2_maxPairs; ++i)
	{
		m_next[i] = b2_nullPair;
	}
#endif //USE_HASH_PAIRCACHE
}


btOverlappingPairCache::~btOverlappingPairCache()
{
	//todo/test: show we erase/delete data, or is it automatic
}

void	btOverlappingPairCache::cleanOverlappingPair(btBroadphasePair& pair,btDispatcher* dispatcher)
{
	if (pair.m_algorithm)
	{
		{
			pair.m_algorithm->~btCollisionAlgorithm();
			dispatcher->freeCollisionAlgorithm(pair.m_algorithm);
			pair.m_algorithm=0;
		}
	}
}


void	btOverlappingPairCache::cleanProxyFromPairs(btBroadphaseProxy* proxy,btDispatcher* dispatcher)
{

	class	CleanPairCallback : public btOverlapCallback
	{
		btBroadphaseProxy* m_cleanProxy;
		btOverlappingPairCache*	m_pairCache;
		btDispatcher* m_dispatcher;

	public:
		CleanPairCallback(btBroadphaseProxy* cleanProxy,btOverlappingPairCache* pairCache,btDispatcher* dispatcher)
			:m_cleanProxy(cleanProxy),
			m_pairCache(pairCache),
			m_dispatcher(dispatcher)
		{
		}
		virtual	bool	processOverlap(btBroadphasePair& pair)
		{
			if ((pair.m_pProxy0 == m_cleanProxy) ||
				(pair.m_pProxy1 == m_cleanProxy))
			{
				m_pairCache->cleanOverlappingPair(pair,m_dispatcher);
			}
			return false;
		}
		
	};

	CleanPairCallback cleanPairs(proxy,this,dispatcher);

	processAllOverlappingPairs(&cleanPairs,dispatcher);

}

void	btOverlappingPairCache::removeOverlappingPairsContainingProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher)
{

	class	RemovePairCallback : public btOverlapCallback
	{
		btBroadphaseProxy* m_obsoleteProxy;

	public:
		RemovePairCallback(btBroadphaseProxy* obsoleteProxy)
			:m_obsoleteProxy(obsoleteProxy)
		{
		}
		virtual	bool	processOverlap(btBroadphasePair& pair)
		{
			return ((pair.m_pProxy0 == m_obsoleteProxy) ||
				(pair.m_pProxy1 == m_obsoleteProxy));
		}
		
	};


	RemovePairCallback removeCallback(proxy);

	processAllOverlappingPairs(&removeCallback,dispatcher);
}


#ifdef USE_HASH_PAIRCACHE

// Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
// This assumes proxyId1 and proxyId2 are 16-bit.
inline uint32 Hash(uint32 proxyId1, uint32 proxyId2)
{
	uint32 key = (proxyId2 << 16) | proxyId1;
	key = ~key + (key << 15);
	key = key ^ (key >> 12);
	key = key + (key << 2);
	key = key ^ (key >> 4);
	key = key * 2057;
	key = key ^ (key >> 16);
	return key;
}

inline bool Equals(const btBroadphasePair& pair, int32 proxyId1, int32 proxyId2)
{	
	return pair.m_pProxy0->getUid() == proxyId1 && pair.m_pProxy1->getUid() == proxyId2;
}

inline btBroadphasePair* btOverlappingPairCache::Find(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1, uint32 hash)
{
	int32 proxyId1 = proxy0->getUid();
	int32 proxyId2 = proxy1->getUid();
	if (proxyId1 > proxyId2) 
		btSwap(proxyId1, proxyId2);

	int32 index = m_hashTable[hash];
	
	while( index != b2_nullPair && Equals(m_overlappingPairArray[index], proxyId1, proxyId2) == false)
	{
		index = m_next[index];
	}

	if ( index == b2_nullPair )
	{
		return NULL;
	}

	btAssert(index < m_overlappingPairArray.size());

	return &m_overlappingPairArray[index];
}

btBroadphasePair* btOverlappingPairCache::Find(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1)
{
	int32 proxyId1 = proxy0->getUid();
	int32 proxyId2 = proxy1->getUid();

	if (proxyId1 > proxyId2) 
		btSwap(proxyId1, proxyId2);

	int32 hash = Hash(proxyId1, proxyId2) & b2_tableMask;

	int32 index = m_hashTable[hash];
	while (index != b2_nullPair && Equals(m_overlappingPairArray[index], proxyId1, proxyId2) == false)
	{
		index = m_next[index];
	}

	if (index == b2_nullPair)
	{
		return NULL;
	}

	btAssert(index < m_overlappingPairArray.size());

	return &m_overlappingPairArray[index];
}

#include <stdio.h>

btBroadphasePair* btOverlappingPairCache::Add(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1)
{
	int32 proxyId1 = proxy0->getUid();
	int32 proxyId2 = proxy1->getUid();

	if (proxyId1 > proxyId2) 
		btSwap(proxyId1, proxyId2);

	int32 hash = Hash(proxyId1, proxyId2) & b2_tableMask;

	

	btBroadphasePair* pair = Find(proxy0, proxy1, hash);
	if (pair != NULL)
	{
		return pair;
	}

	if (m_overlappingPairArray.size() == b2_maxPairs)
	{
		btAssert(false);
		return NULL;
	}
	
	int count = m_overlappingPairArray.size();
	void* mem = &m_overlappingPairArray.expand();
	
	pair = new (mem) btBroadphasePair(*proxy0,*proxy1);
//	pair->m_pProxy0 = proxy0;
//	pair->m_pProxy1 = proxy1;
	pair->m_algorithm = 0;
	pair->m_userInfo = 0;
	

	m_next[count] = m_hashTable[hash];
	m_hashTable[hash] = (uint16)count;

	return pair;
}


void* btOverlappingPairCache::Remove(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1,btDispatcher* dispatcher)
{
	int32 proxyId1 = proxy0->getUid();
	int32 proxyId2 = proxy1->getUid();

	if (proxyId1 > proxyId2) 
		btSwap(proxyId1, proxyId2);

	int32 hash = Hash(proxyId1, proxyId2) & b2_tableMask;

	btBroadphasePair* pair = Find(proxy0, proxy1, hash);
	if (pair == NULL)
	{
		return NULL;
	}

	cleanOverlappingPair(*pair,dispatcher);

	void* userData = pair->m_userInfo;

	btAssert(pair->m_pProxy0->getUid() == proxyId1);
	btAssert(pair->m_pProxy1->getUid() == proxyId2);

	int32 pairIndex = int32(pair - &m_overlappingPairArray[0]);
	btAssert(pairIndex < m_overlappingPairArray.size());

	// Remove the pair from the hash table.
	int32 index = m_hashTable[hash];
	btAssert(index != b2_nullPair);

	int32 previous = b2_nullPair;
	while (index != pairIndex)
	{
		previous = index;
		index = m_next[index];
	}

	if (previous != b2_nullPair)
	{
		btAssert(m_next[previous] == pairIndex);
		m_next[previous] = m_next[pairIndex];
	}
	else
	{
		m_hashTable[hash] = m_next[pairIndex];
	}

	// We now move the last pair into spot of the
	// pair being removed. We need to fix the hash
	// table indices to support the move.

	int32 lastPairIndex = m_overlappingPairArray.size() - 1;

	// If the removed pair is the last pair, we are done.
	if (lastPairIndex == pairIndex)
	{
		m_overlappingPairArray.pop_back();
		return userData;
	}

	// Remove the last pair from the hash table.
	const btBroadphasePair* last = &m_overlappingPairArray[lastPairIndex];
	int32 lastHash = Hash(last->m_pProxy0->getUid(), last->m_pProxy1->getUid()) & b2_tableMask;

	index = m_hashTable[lastHash];
	btAssert(index != b2_nullPair);

	previous = b2_nullPair;
	while (index != lastPairIndex)
	{
		previous = index;
		index = m_next[index];
	}

	if (previous != b2_nullPair)
	{
		btAssert(m_next[previous] == lastPairIndex);
		m_next[previous] = m_next[lastPairIndex];
	}
	else
	{
		m_hashTable[lastHash] = m_next[lastPairIndex];
	}

	// Copy the last pair into the remove pair's spot.
	m_overlappingPairArray[pairIndex] = m_overlappingPairArray[lastPairIndex];

	// Insert the last pair into the hash table
	m_next[pairIndex] = m_hashTable[lastHash];
	m_hashTable[lastHash] = (uint16)pairIndex;

	m_overlappingPairArray.pop_back();

	return userData;
}
#include <stdio.h>

void	btOverlappingPairCache::processAllOverlappingPairs(btOverlapCallback* callback,btDispatcher* dispatcher)
{

	int i;

	//printf("m_overlappingPairArray.size()=%d\n",m_overlappingPairArray.size());
	for (i=0;i<m_overlappingPairArray.size();)
	{
	
		btBroadphasePair* pair = &m_overlappingPairArray[i];
		if (callback->processOverlap(*pair))
		{
			removeOverlappingPair(pair->m_pProxy0,pair->m_pProxy1,dispatcher);

			gOverlappingPairs--;
		} else
		{
			i++;
		}
	}
}

#else




void	btOverlappingPairCache::removeOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1, btDispatcher* dispatcher )
{
#ifndef USE_LAZY_REMOVAL

	btBroadphasePair findPair(*proxy0,*proxy1);

	int findIndex = m_overlappingPairArray.findLinearSearch(findPair);
	if (findIndex < m_overlappingPairArray.size())
	{
		gOverlappingPairs--;
		btBroadphasePair& pair = m_overlappingPairArray[findIndex];
		cleanOverlappingPair(pair,dispatcher);
		
		m_overlappingPairArray.swap(findIndex,m_overlappingPairArray.size()-1);
		m_overlappingPairArray.pop_back();
	}
#endif //USE_LAZY_REMOVAL

}








void	btOverlappingPairCache::addOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
{
	//don't add overlap with own
	assert(proxy0 != proxy1);

	if (!needsBroadphaseCollision(proxy0,proxy1))
		return;


	btBroadphasePair pair(*proxy0,*proxy1);
	m_overlappingPairArray.push_back(pair);
	gOverlappingPairs++;
	
}

///this findPair becomes really slow. Either sort the list to speedup the query, or
///use a different solution. It is mainly used for Removing overlapping pairs. Removal could be delayed.
///we could keep a linked list in each proxy, and store pair in one of the proxies (with lowest memory address)
///Also we can use a 2D bitmap, which can be useful for a future GPU implementation
 btBroadphasePair*	btOverlappingPairCache::findPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
{
	if (!needsBroadphaseCollision(proxy0,proxy1))
		return 0;

	btBroadphasePair tmpPair(*proxy0,*proxy1);
	int findIndex = m_overlappingPairArray.findLinearSearch(tmpPair);

	if (findIndex < m_overlappingPairArray.size())
	{
		//assert(it != m_overlappingPairSet.end());
		 btBroadphasePair* pair = &m_overlappingPairArray[findIndex];
		return pair;
	}
	return 0;
}










#include <stdio.h>

void	btOverlappingPairCache::processAllOverlappingPairs(btOverlapCallback* callback,btDispatcher* dispatcher)
{

	int i;

	for (i=0;i<m_overlappingPairArray.size();)
	{
	
		btBroadphasePair* pair = &m_overlappingPairArray[i];
		if (callback->processOverlap(*pair))
		{
			cleanOverlappingPair(*pair,dispatcher);

			m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
			m_overlappingPairArray.pop_back();
			gOverlappingPairs--;
		} else
		{
			i++;
		}
	}
}



#endif //USE_HASH_PAIRCACHE
