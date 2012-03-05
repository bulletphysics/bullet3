/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef BT_LOW_LEVEL_COLLISION__DISPATCHER_H
#define BT_LOW_LEVEL_COLLISION__DISPATCHER_H

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
struct btLowLevelBroadphase;//struct instead of class?
struct btLowLevelData;
///Tuning value to optimized SPU utilization 
///Too small value means Task overhead is large compared to computation (too fine granularity)
///Too big value might render some SPUs are idle, while a few other SPUs are doing all work.
//#define SPU_BATCHSIZE_BROADPHASE_PAIRS 8
//#define SPU_BATCHSIZE_BROADPHASE_PAIRS 16
//#define SPU_BATCHSIZE_BROADPHASE_PAIRS 64
#define SPU_BATCHSIZE_BROADPHASE_PAIRS 128
//#define SPU_BATCHSIZE_BROADPHASE_PAIRS 256
//#define SPU_BATCHSIZE_BROADPHASE_PAIRS 512
//#define SPU_BATCHSIZE_BROADPHASE_PAIRS 1024
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

class btLowLevelCollisionAlgorithm : public btCollisionAlgorithm
{
	btPersistentManifold* m_manifold;

public:

	btLowLevelCollisionAlgorithm()
		:m_manifold(0)
	{
	}

	btLowLevelCollisionAlgorithm(btPersistentManifold* manifold, btCollisionDispatcher* dispatcher)
		:m_manifold(manifold)
	{
		m_dispatcher = dispatcher;
	}
	virtual ~btLowLevelCollisionAlgorithm()
	{
		if (m_manifold)
			m_dispatcher->releaseManifold(m_manifold);
	}
	virtual void processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
	{
		btAssert(0);
	}

	virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
	{
		btAssert(0);
		return 0.f;
	}

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray)
	{
		manifoldArray.push_back(m_manifold);
	}
	
};


///btLowLevelCollisionDispatcher can use SPU to gather and calculate collision detection
///Time of Impact, Closest Points and Penetration Depth.
class btLowLevelCollisionDispatcher : public btCollisionDispatcher
{

	btLowLevelData* m_lowLevelData;

	btAlignedObjectArray<btPersistentManifold>	m_manifoldArray;
	btAlignedObjectArray<btLowLevelCollisionAlgorithm> m_algorithms;

protected:

	void	collision();

public:

	
	btLowLevelCollisionDispatcher (btLowLevelData* lowLevelData, btCollisionConfiguration* collisionConfiguration, int maxNumManifolds);
	
	virtual ~btLowLevelCollisionDispatcher();

	bool	supportsDispatchPairOnSpu(int proxyType0,int proxyType1);

	virtual void	dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,const btDispatcherInfo& dispatchInfo,btDispatcher* dispatcher) ;

	

	virtual btPersistentManifold*	getNewManifold(void* b0,void* b1)
	{
		btAssert(0);
		return 0;
	}
	virtual void releaseManifold(btPersistentManifold* manifold);

	
	virtual	void* allocateCollisionAlgorithm(int size)
	{
		btAssert(0);
		return 0;
	}

	virtual	void freeCollisionAlgorithm(void* ptr)
	{

	}



};



#endif //BT_LOW_LEVEL_COLLISION__DISPATCHER_H


