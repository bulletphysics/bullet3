
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

#ifndef OVERLAPPING_PAIR_CACHE_H
#define OVERLAPPING_PAIR_CACHE_H


#include "btBroadphaseInterface.h"
#include "btBroadphaseProxy.h"
#include "LinearMath/btPoint3.h"
#include "LinearMath/btAlignedObjectArray.h"
class btDispatcher;

#define USE_HASH_PAIRCACHE 1


struct	btOverlapCallback
{
	virtual ~btOverlapCallback()
	{}
	//return true for deletion of the pair
	virtual bool	processOverlap(btBroadphasePair& pair) = 0;
};

struct btOverlapFilterCallback
{
	virtual ~btOverlapFilterCallback()
	{}
	// return true when pairs need collision
	virtual bool	needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const = 0;
};

typedef btAlignedObjectArray<btBroadphasePair>	btBroadphasePairArray;

#ifdef USE_HASH_PAIRCACHE



const int b2_maxPairs = 65536;//32768;
typedef unsigned short int uint16;
typedef int int32;
typedef unsigned int uint32;

/// Hash-space based Pair Cache, thanks to Erin Catto, Box2D, http://www.box2d.org, and Pierre Terdiman, Codercorner, http://codercorner.com
const uint16 b2_nullPair = 0xffff;
const uint16 b2_nullProxy = 0xffff;
const int32 b2_tableCapacity = b2_maxPairs;	// must be a power of two
const int32 b2_tableMask = b2_tableCapacity - 1;


class btOverlappingPairCache
{
	btBroadphasePairArray	m_overlappingPairArray;
	btOverlapFilterCallback* m_overlapFilterCallback;
	bool		m_blockedForChanges;

public:
	btOverlappingPairCache();
	~btOverlappingPairCache();

	
	void	removeOverlappingPairsContainingProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);

	void	removeOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,btDispatcher* dispatcher)
	{
		Remove(proxy0,proxy1,dispatcher);
	}

	inline bool needsBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
	{
		if (m_overlapFilterCallback)
			return m_overlapFilterCallback->needBroadphaseCollision(proxy0,proxy1);

		bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
		
		return collides;
	}

	void	addOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
	{
		if (!needsBroadphaseCollision(proxy0,proxy1))
			return;

		Add(proxy0,proxy1);
	}

	btBroadphasePair*	findPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
	{
		return Find(proxy0,proxy1);
	}

	void	cleanProxyFromPairs(btBroadphaseProxy* proxy,btDispatcher* dispatcher);

	
	virtual void	processAllOverlappingPairs(btOverlapCallback*,btDispatcher* dispatcher);

	btBroadphasePair*	getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}

	const btBroadphasePair*	getOverlappingPairArrayPtr() const
	{
		return &m_overlappingPairArray[0];
	}

	btBroadphasePairArray&	getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	const btBroadphasePairArray&	getOverlappingPairArray() const
	{
		return m_overlappingPairArray;
	}

	void	cleanOverlappingPair(btBroadphasePair& pair,btDispatcher* dispatcher);

	// Add a pair and return the new pair. If the pair already exists,
	// no new pair is created and the old one is returned.
	btBroadphasePair* Add(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1);

	// Remove a pair, return the pair's userData.
	void* Remove(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1,btDispatcher* dispatcher);

	btBroadphasePair* Find(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1);

	int32 GetCount() const { return m_overlappingPairArray.size(); }
//	btBroadphasePair* GetPairs() { return m_pairs; }

	btOverlapFilterCallback* getOverlapFilterCallback()
	{
		return m_overlapFilterCallback;
	}

	void setOverlapFilterCallback(btOverlapFilterCallback* callback)
	{
		m_overlapFilterCallback = callback;
	}

	int	getNumOverlappingPairs() const
	{
		return m_overlappingPairArray.size();
	}
private:
	btBroadphasePair* Find(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1, uint32 hashValue);

public:
	
	uint16 m_hashTable[b2_tableCapacity];
	uint16 m_next[b2_maxPairs];
};



#else//USE_HASH_PAIRCACHE

#define USE_LAZY_REMOVAL 1

///btOverlappingPairCache maintains the objects with overlapping AABB
///Typically managed by the Broadphase, Axis3Sweep or btSimpleBroadphase
class	btOverlappingPairCache
{
	protected:
		//avoid brute-force finding all the time
		btBroadphasePairArray	m_overlappingPairArray;

		//during the dispatch, check that user doesn't destroy/create proxy
		bool		m_blockedForChanges;
		
		//if set, use the callback instead of the built in filter in needBroadphaseCollision
		btOverlapFilterCallback* m_overlapFilterCallback;

	public:
			
		btOverlappingPairCache();	
		virtual ~btOverlappingPairCache();

		virtual void	processAllOverlappingPairs(btOverlapCallback*,btDispatcher* dispatcher);

		void	removeOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,btDispatcher* dispatcher);

		void	cleanOverlappingPair(btBroadphasePair& pair,btDispatcher* dispatcher);
		
		void	addOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);

		btBroadphasePair*	findPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);
			
		
		void	cleanProxyFromPairs(btBroadphaseProxy* proxy,btDispatcher* dispatcher);

		void	removeOverlappingPairsContainingProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);


		inline bool needsBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
		{
			if (m_overlapFilterCallback)
				return m_overlapFilterCallback->needBroadphaseCollision(proxy0,proxy1);

			bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
			collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			
			return collides;
		}
		
		btBroadphasePairArray&	getOverlappingPairArray()
		{
			return m_overlappingPairArray;
		}

		const btBroadphasePairArray&	getOverlappingPairArray() const
		{
			return m_overlappingPairArray;
		}

		


		btBroadphasePair*	getOverlappingPairArrayPtr()
		{
			return &m_overlappingPairArray[0];
		}

		const btBroadphasePair*	getOverlappingPairArrayPtr() const
		{
			return &m_overlappingPairArray[0];
		}

		int	getNumOverlappingPairs() const
		{
			return m_overlappingPairArray.size();
		}
		
		btOverlapFilterCallback* getOverlapFilterCallback()
		{
			return m_overlapFilterCallback;
		}

		void setOverlapFilterCallback(btOverlapFilterCallback* callback)
		{
			m_overlapFilterCallback = callback;
		}

};
#endif //USE_HASH_PAIRCACHE

#endif //OVERLAPPING_PAIR_CACHE_H


