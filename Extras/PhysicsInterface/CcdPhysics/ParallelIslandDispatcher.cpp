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


#include "ParallelIslandDispatcher.h"


#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h"

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include <algorithm>

static int gNumManifold2 = 0;


	

	
ParallelIslandDispatcher::ParallelIslandDispatcher (): 
	m_useIslands(true),
		m_defaultManifoldResult(0,0,0),
		m_count(0)
{
	int i;
	
	for (i=0;i<MAX_BROADPHASE_COLLISION_TYPES;i++)
	{
		for (int j=0;j<MAX_BROADPHASE_COLLISION_TYPES;j++)
		{
			m_doubleDispatch[i][j] = 0;
		}
	}
	
	
};
	
btPersistentManifold*	ParallelIslandDispatcher::getNewManifold(void* b0,void* b1) 
{ 
	gNumManifold2++;
	
	//ASSERT(gNumManifold < 65535);
	

	btCollisionObject* body0 = (btCollisionObject*)b0;
	btCollisionObject* body1 = (btCollisionObject*)b1;
	
	btPersistentManifold* manifold = new btPersistentManifold (body0,body1);
	m_manifoldsPtr.push_back(manifold);

	return manifold;
}

void ParallelIslandDispatcher::clearManifold(btPersistentManifold* manifold)
{
	manifold->clearManifold();
}

	
void ParallelIslandDispatcher::releaseManifold(btPersistentManifold* manifold)
{
	
	gNumManifold2--;

	//printf("releaseManifold: gNumManifold2 %d\n",gNumManifold2);

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

	
//
// todo: this is random access, it can be walked 'cache friendly'!
//
void ParallelIslandDispatcher::buildAndProcessIslands(btCollisionObjectArray& collisionObjects, IslandCallback* callback)
{
	int numBodies  = collisionObjects.size();

	for (int islandId=0;islandId<numBodies;islandId++)
	{

		std::vector<btPersistentManifold*>  islandmanifold;
		
		//int numSleeping = 0;

		bool allSleeping = true;

		int i;
		for (i=0;i<numBodies;i++)
		{
			btCollisionObject* colObj0 = collisionObjects[i];
			if (colObj0->m_islandTag1 == islandId)
			{
				if (colObj0->GetActivationState()== ACTIVE_TAG)
				{
					allSleeping = false;
				}
				if (colObj0->GetActivationState()== DISABLE_DEACTIVATION)
				{
					allSleeping = false;
				}
			}
		}

		
		for (i=0;i<getNumManifolds();i++)
		{
			 btPersistentManifold* manifold = this->getManifoldByIndexInternal(i);
			 
			 //filtering for response

			 btCollisionObject* colObj0 = static_cast<btCollisionObject*>(manifold->getBody0());
			 btCollisionObject* colObj1 = static_cast<btCollisionObject*>(manifold->getBody1());
			 {
				if (((colObj0) && (colObj0)->m_islandTag1 == (islandId)) ||
					((colObj1) && (colObj1)->m_islandTag1 == (islandId)))
				{

					if (needsResponse(*colObj0,*colObj1))
						islandmanifold.push_back(manifold);
				}
			 }
		}
		if (allSleeping)
		{
			int i;
			for (i=0;i<numBodies;i++)
			{
				btCollisionObject* colObj0 = collisionObjects[i];
				if (colObj0->m_islandTag1 == islandId)
				{
					colObj0->SetActivationState( ISLAND_SLEEPING );
				}
			}

			
		} else
		{

			int i;
			for (i=0;i<numBodies;i++)
			{
				btCollisionObject* colObj0 = collisionObjects[i];
				if (colObj0->m_islandTag1 == islandId)
				{
					if ( colObj0->GetActivationState() == ISLAND_SLEEPING)
					{
						colObj0->SetActivationState( WANTS_DEACTIVATION);
					}
				}
			}

			/// Process the actual simulation, only if not sleeping/deactivated
			if (islandmanifold.size())
			{
				callback->ProcessIsland(&islandmanifold[0],islandmanifold.size());
			}

		}
	}
}



btCollisionAlgorithm* ParallelIslandDispatcher::internalFindAlgorithm(btBroadphaseProxy& proxy0,btBroadphaseProxy& proxy1)
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

bool	ParallelIslandDispatcher::needsResponse(const  btCollisionObject& colObj0,const btCollisionObject& colObj1)
{

	
	//here you can do filtering
	bool hasResponse = 
		(!(colObj0.m_collisionFlags & btCollisionObject::noContactResponse)) &&
		(!(colObj1.m_collisionFlags & btCollisionObject::noContactResponse));
	hasResponse = hasResponse &&
		(colObj0.IsActive() || colObj1.IsActive());
	return hasResponse;
}

bool	ParallelIslandDispatcher::needsCollision(btBroadphaseProxy& proxy0,btBroadphaseProxy& proxy1)
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
btManifoldResult*	ParallelIslandDispatcher::getNewManifoldResult(btCollisionObject* obj0,btCollisionObject* obj1,btPersistentManifold* manifold)
{


	//in-place, this prevents parallel dispatching, but just adding a list would fix that.
	btManifoldResult* manifoldResult = new (&m_defaultManifoldResult)	btManifoldResult(obj0,obj1,manifold);
	return manifoldResult;
}
	
///allows the user to get contact point callbacks 
void	ParallelIslandDispatcher::releaseManifoldResult(btManifoldResult*)
{

}


void	ParallelIslandDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,btDispatcherInfo& dispatchInfo)
{
	//m_blockedForChanges = true;



	assert(0);
	
/*
	int dispatcherId = getUniqueId();
	int i;
	for (i=0;i<numPairs;i++)
	{

		btBroadphasePair& pair = pairs[i];

		if (dispatcherId>= 0)
		{
			//dispatcher will keep algorithms persistent in the collision pair
			if (!pair.m_algorithms[dispatcherId])
			{
				pair.m_algorithms[dispatcherId] = findAlgorithm(
					*pair.m_pProxy0,
					*pair.m_pProxy1);
			}

			if (pair.m_algorithms[dispatcherId])
			{
				if (dispatchInfo.m_dispatchFunc == 		btDispatcherInfo::DISPATCH_DISCRETE)
				{
					pair.m_algorithms[dispatcherId]->processCollision(pair.m_pProxy0,pair.m_pProxy1,dispatchInfo);
				} else
				{
					float toi = pair.m_algorithms[dispatcherId]->calculateTimeOfImpact(pair.m_pProxy0,pair.m_pProxy1,dispatchInfo);
					if (dispatchInfo.m_timeOfImpact > toi)
						dispatchInfo.m_timeOfImpact = toi;

				}
			}
		} else
		{
			//non-persistent algorithm dispatcher
			btCollisionAlgorithm* algo = findAlgorithm(
				*pair.m_pProxy0,
				*pair.m_pProxy1);

			if (algo)
			{
				if (dispatchInfo.m_dispatchFunc == 		btDispatcherInfo::DISPATCH_DISCRETE)
				{
					algo->processCollision(pair.m_pProxy0,pair.m_pProxy1,dispatchInfo);
				} else
				{
					float toi = algo->calculateTimeOfImpact(pair.m_pProxy0,pair.m_pProxy1,dispatchInfo);
					if (dispatchInfo.m_timeOfImpact > toi)
						dispatchInfo.m_timeOfImpact = toi;
				}
			}
		}

	}
*/
	//m_blockedForChanges = false;

}

