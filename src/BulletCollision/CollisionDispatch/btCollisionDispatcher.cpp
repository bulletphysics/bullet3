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


#include "btCollisionDispatcher.h"


#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include <algorithm>
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"

int gNumManifold = 0;


	

	
btCollisionDispatcher::btCollisionDispatcher (): 
	m_useIslands(true),
		m_defaultManifoldResult(0,0,0),
		m_count(0)
{
	int i;
	
	//default CreationFunctions, filling the m_doubleDispatch table
	m_convexConvexCreateFunc = new btConvexConvexAlgorithm::CreateFunc;
	m_convexConcaveCreateFunc = new btConvexConcaveCollisionAlgorithm::CreateFunc;
	m_swappedConvexConcaveCreateFunc = new btConvexConcaveCollisionAlgorithm::SwappedCreateFunc;
	m_compoundCreateFunc = new btCompoundCollisionAlgorithm::CreateFunc;
	m_swappedCompoundCreateFunc = new btCompoundCollisionAlgorithm::SwappedCreateFunc;
	m_emptyCreateFunc = new btEmptyAlgorithm::CreateFunc;

	for (i=0;i<MAX_BROADPHASE_COLLISION_TYPES;i++)
	{
		for (int j=0;j<MAX_BROADPHASE_COLLISION_TYPES;j++)
		{
			m_doubleDispatch[i][j] = internalFindCreateFunc(i,j);
			assert(m_doubleDispatch[i][j]);
		}
	}
	
	
};

void btCollisionDispatcher::registerCollisionCreateFunc(int proxyType0, int proxyType1, btCollisionAlgorithmCreateFunc *createFunc)
{
	m_doubleDispatch[proxyType0][proxyType1] = createFunc;
}

btCollisionDispatcher::~btCollisionDispatcher()
{
	delete m_convexConvexCreateFunc;
	delete m_convexConcaveCreateFunc;
	delete m_swappedConvexConcaveCreateFunc;
	delete m_compoundCreateFunc;
	delete m_swappedCompoundCreateFunc;
	delete m_emptyCreateFunc;
}

btPersistentManifold*	btCollisionDispatcher::getNewManifold(void* b0,void* b1) 
{ 
	gNumManifold++;
	
	//ASSERT(gNumManifold < 65535);
	

	btCollisionObject* body0 = (btCollisionObject*)b0;
	btCollisionObject* body1 = (btCollisionObject*)b1;
	
	btPersistentManifold* manifold = new btPersistentManifold (body0,body1);
	m_manifoldsPtr.push_back(manifold);

	return manifold;
}

void btCollisionDispatcher::clearManifold(btPersistentManifold* manifold)
{
	manifold->clearManifold();
}

	
void btCollisionDispatcher::releaseManifold(btPersistentManifold* manifold)
{
	
	gNumManifold--;

	//printf("releaseManifold: gNumManifold %d\n",gNumManifold);

	clearManifold(manifold);

	std::vector<btPersistentManifold*>::iterator i =
		std::find(m_manifoldsPtr.begin(), m_manifoldsPtr.end(), manifold);
	if (!(i == m_manifoldsPtr.end()))
	{
		std::swap(*i, m_manifoldsPtr.back());
		m_manifoldsPtr.pop_back();
		delete manifold;

	}
	
	
}

	

btCollisionAlgorithm* btCollisionDispatcher::findAlgorithm(btBroadphaseProxy& proxy0,btBroadphaseProxy& proxy1)
{
#define USE_DISPATCH_REGISTRY_ARRAY 1
#ifdef USE_DISPATCH_REGISTRY_ARRAY
	btCollisionObject* body0 = (btCollisionObject*)proxy0.m_clientObject;
	btCollisionObject* body1 = (btCollisionObject*)proxy1.m_clientObject;
	btCollisionAlgorithmConstructionInfo ci;
	ci.m_dispatcher = this;
	btCollisionAlgorithm* algo = m_doubleDispatch[body0->m_collisionShape->getShapeType()][body1->m_collisionShape->getShapeType()]
	->CreateCollisionAlgorithm(ci,&proxy0,&proxy1);
#else
	btCollisionAlgorithm* algo = internalFindAlgorithm(proxy0,proxy1);
#endif //USE_DISPATCH_REGISTRY_ARRAY
	return algo;
}


btCollisionAlgorithmCreateFunc* btCollisionDispatcher::internalFindCreateFunc(int proxyType0,int proxyType1)
{
	
	if (btBroadphaseProxy::isConvex(proxyType0) && btBroadphaseProxy::isConvex(proxyType1))
	{
		return m_convexConvexCreateFunc;
	}

	if (btBroadphaseProxy::isConvex(proxyType0) && btBroadphaseProxy::isConcave(proxyType1))
	{
		return m_convexConcaveCreateFunc;
	}

	if (btBroadphaseProxy::isConvex(proxyType1) && btBroadphaseProxy::isConcave(proxyType0))
	{
		return m_swappedConvexConcaveCreateFunc;
	}

	if (btBroadphaseProxy::isCompound(proxyType0))
	{
		return m_compoundCreateFunc;
	} else
	{
		if (btBroadphaseProxy::isCompound(proxyType1))
		{
			return m_swappedCompoundCreateFunc;
		}
	}

	//failed to find an algorithm
	return m_emptyCreateFunc;
}



btCollisionAlgorithm* btCollisionDispatcher::internalFindAlgorithm(btBroadphaseProxy& proxy0,btBroadphaseProxy& proxy1)
{
	m_count++;
	btCollisionObject* body0 = (btCollisionObject*)proxy0.m_clientObject;
	btCollisionObject* body1 = (btCollisionObject*)proxy1.m_clientObject;

	btCollisionAlgorithmConstructionInfo ci;
	ci.m_dispatcher = this;
	
	if (body0->m_collisionShape->isConvex() && body1->m_collisionShape->isConvex() )
	{
		return new btConvexConvexAlgorithm(0,ci,&proxy0,&proxy1);			
	}

	if (body0->m_collisionShape->isConvex() && body1->m_collisionShape->isConcave())
	{
		return new btConvexConcaveCollisionAlgorithm(ci,&proxy0,&proxy1);
	}

	if (body1->m_collisionShape->isConvex() && body0->m_collisionShape->isConcave())
	{
		return new btConvexConcaveCollisionAlgorithm(ci,&proxy1,&proxy0);
	}

	if (body0->m_collisionShape->isCompound())
	{
		return new btCompoundCollisionAlgorithm(ci,&proxy0,&proxy1);
	} else
	{
		if (body1->m_collisionShape->isCompound())
		{
			return new btCompoundCollisionAlgorithm(ci,&proxy1,&proxy0);
		}
	}

	//failed to find an algorithm
	return new btEmptyAlgorithm(ci);
	
}

bool	btCollisionDispatcher::needsResponse(const  btCollisionObject& colObj0,const btCollisionObject& colObj1)
{

	
	//here you can do filtering
	bool hasResponse = 
		(!(colObj0.m_collisionFlags & btCollisionObject::noContactResponse)) &&
		(!(colObj1.m_collisionFlags & btCollisionObject::noContactResponse));
	hasResponse = hasResponse &&
		(colObj0.IsActive() || colObj1.IsActive());
	return hasResponse;
}

bool	btCollisionDispatcher::needsCollision(btBroadphaseProxy& proxy0,btBroadphaseProxy& proxy1)
{

	btCollisionObject* body0 = (btCollisionObject*)proxy0.m_clientObject;
	btCollisionObject* body1 = (btCollisionObject*)proxy1.m_clientObject;

	assert(body0);
	assert(body1);

	bool needsCollision = true;

	if ((body0->m_collisionFlags & btCollisionObject::isStatic) && 
		(body1->m_collisionFlags & btCollisionObject::isStatic))
		needsCollision = false;
		
	if ((!body0->IsActive()) && (!body1->IsActive()))
		needsCollision = false;
	
	return needsCollision ;

}

///allows the user to get contact point callbacks 
btManifoldResult*	btCollisionDispatcher::getNewManifoldResult(btCollisionObject* obj0,btCollisionObject* obj1,btPersistentManifold* manifold)
{


	//in-place, this prevents parallel dispatching, but just adding a list would fix that.
	btManifoldResult* manifoldResult = new (&m_defaultManifoldResult)	btManifoldResult(obj0,obj1,manifold);
	return manifoldResult;
}
	
///allows the user to get contact point callbacks 
void	btCollisionDispatcher::releaseManifoldResult(btManifoldResult*)
{

}


class btCollisionPairCallback : public btOverlapCallback
{
	btDispatcherInfo& m_dispatchInfo;
	btCollisionDispatcher*	m_dispatcher;
	int		m_dispatcherId;
public:

	btCollisionPairCallback(btDispatcherInfo& dispatchInfo,btCollisionDispatcher*	dispatcher,int		dispatcherId)
	:m_dispatchInfo(dispatchInfo),
	m_dispatcher(dispatcher),
	m_dispatcherId(dispatcherId)
	{
	}

	virtual bool	processOverlap(btBroadphasePair& pair)
	{
		if (m_dispatcherId>= 0)
		{
			//dispatcher will keep algorithms persistent in the collision pair
			if (!pair.m_algorithms[m_dispatcherId])
			{
				pair.m_algorithms[m_dispatcherId] = m_dispatcher->findAlgorithm(
					*pair.m_pProxy0,
					*pair.m_pProxy1);
			}

			if (pair.m_algorithms[m_dispatcherId])
			{
				if (m_dispatchInfo.m_dispatchFunc == 		btDispatcherInfo::DISPATCH_DISCRETE)
				{
					pair.m_algorithms[m_dispatcherId]->processCollision(pair.m_pProxy0,pair.m_pProxy1,m_dispatchInfo);
				} else
				{
					float toi = pair.m_algorithms[m_dispatcherId]->calculateTimeOfImpact(pair.m_pProxy0,pair.m_pProxy1,m_dispatchInfo);
					if (m_dispatchInfo.m_timeOfImpact > toi)
						m_dispatchInfo.m_timeOfImpact = toi;

				}
			}
		} else
		{
			//non-persistent algorithm dispatcher
			btCollisionAlgorithm* algo = m_dispatcher->findAlgorithm(
				*pair.m_pProxy0,
				*pair.m_pProxy1);

			if (algo)
			{
				if (m_dispatchInfo.m_dispatchFunc == 		btDispatcherInfo::DISPATCH_DISCRETE)
				{
					algo->processCollision(pair.m_pProxy0,pair.m_pProxy1,m_dispatchInfo);
				} else
				{
					float toi = algo->calculateTimeOfImpact(pair.m_pProxy0,pair.m_pProxy1,m_dispatchInfo);
					if (m_dispatchInfo.m_timeOfImpact > toi)
						m_dispatchInfo.m_timeOfImpact = toi;
				}
			}
		}
		return false;

	}
};


void	btCollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,btDispatcherInfo& dispatchInfo)
{
	//m_blockedForChanges = true;

	int dispatcherId = getUniqueId();

	btCollisionPairCallback	collisionCallback(dispatchInfo,this,dispatcherId);

	pairCache->processAllOverlappingPairs(&collisionCallback);

	//m_blockedForChanges = false;

}


