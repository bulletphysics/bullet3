/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_GHOST_OBJECT_H
#define BT_GHOST_OBJECT_H


#include "btCollisionObject.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h"
#include "LinearMath/btAlignedAllocator.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"

///work-in-progress, not complete

///The btGhostObject can keep track of all objects that are overlapping
///By default, this overlap is based on the AABB
///This is useful for creating a character controller, collision sensors/triggers, explosions etc.
///We plan on adding rayTest and other queries for the btGhostObject
ATTRIBUTE_ALIGNED16(class) btGhostObject : public btCollisionObject
{
protected:

	btOverlappingPairCache*	m_overlappingPairCache;
	
	struct btGhostOverlapFilterCallback : public btOverlapFilterCallback
	{
		bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
		{
			bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
			collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			return collides;
		}
	};
	
	btGhostOverlapFilterCallback	m_ghostOverlapFilterCallback;


public:

	btGhostObject();

	virtual ~btGhostObject();


	virtual void	addOverlappingObject(btCollisionObject* otherObject);

	virtual void	removeOverlappingObject(btCollisionObject* otherObject);

	btOverlappingPairCache*	getPairCache()
	{
		return m_overlappingPairCache;
	};

	const btOverlappingPairCache*	getPairCache() const
	{
		return m_overlappingPairCache;
	};

	btBroadphasePairArray&	getOverlappingPairArray()
	{
		return m_overlappingPairCache->getOverlappingPairArray();
	}

	btOverlappingPairCache* getOverlappingPairCache()
	{
		return m_overlappingPairCache;
	}

	//
	// Cast
	//

	static const btGhostObject*	upcast(const btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_GHOST_OBJECT)
			return (const btGhostObject*)colObj;
		return 0;
	}
	static btGhostObject*			upcast(btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_GHOST_OBJECT)
			return (btGhostObject*)colObj;
		return 0;
	}



};


///btGhostPairCache  keeps track of overlapping objects that have AABB overlap with the ghost
class btGhostPairCallback : public btOverlappingPairCallback
{
	
public:
	btGhostPairCallback()
	{
	}

	virtual ~btGhostPairCallback()
	{
		
	}

	virtual btBroadphasePair*	addOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
	{
		btCollisionObject* colObj0 = (btCollisionObject*) proxy0->m_clientObject;
		btCollisionObject* colObj1 = (btCollisionObject*) proxy1->m_clientObject;
		btGhostObject* ghost0 = 		btGhostObject::upcast(colObj0);
		btGhostObject* ghost1 = 		btGhostObject::upcast(colObj1);
		if (ghost0)
			ghost0->addOverlappingObject(colObj1);
		if (ghost1)
			ghost1->addOverlappingObject(colObj0);
	}

	virtual void*	removeOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,btDispatcher* dispatcher)
	{
		btCollisionObject* colObj0 = (btCollisionObject*) proxy0->m_clientObject;
		btCollisionObject* colObj1 = (btCollisionObject*) proxy1->m_clientObject;
		btGhostObject* ghost0 = 		btGhostObject::upcast(colObj0);
		btGhostObject* ghost1 = 		btGhostObject::upcast(colObj1);
		if (ghost0)
			ghost0->removeOverlappingObject(colObj1);
		if (ghost1)
			ghost1->removeOverlappingObject(colObj0);
	}

	virtual void	removeOverlappingPairsContainingProxy(btBroadphaseProxy* proxy0,btDispatcher* dispatcher)
	{
		btAssert(0);
		//need to keep track of all ghost objects and call them here
		//m_hashPairCache->removeOverlappingPairsContainingProxy(proxy0,dispatcher);
	}

	

};

#endif