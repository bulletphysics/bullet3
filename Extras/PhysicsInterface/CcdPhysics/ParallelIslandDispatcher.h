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

#ifndef PARALLEL_ISLAND_DISPATCHER_H
#define PARALLEL_ISLAND_DISPATCHER_H

#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/CollisionDispatch/btUnionFind.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include <vector>

class btIDebugDraw;


#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"




///ParallelIslandDispatcher dispatches entire simulation islands in parallel.
///For both collision detection and constraint solving.
///This development heads toward multi-core, CELL SPU and GPU approach
class ParallelIslandDispatcher : public btDispatcher
{
	
	std::vector<btPersistentManifold*>	m_manifoldsPtr;

	btUnionFind m_unionFind;

	bool m_useIslands;
	
	btManifoldResult	m_defaultManifoldResult;
	
	btCollisionAlgorithmCreateFunc* m_doubleDispatch[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];
	
public:
	
	btUnionFind& getUnionFind() { return m_unionFind;}

	struct	IslandCallback
	{
		virtual ~IslandCallback() {};

		virtual	void	ProcessIsland(btPersistentManifold**	manifolds,int numManifolds) = 0;
	};


	int	getNumManifolds() const
	{ 
		return m_manifoldsPtr.size();
	}

	 btPersistentManifold* getManifoldByIndexInternal(int index)
	{
		return m_manifoldsPtr[index];
	}

	 const btPersistentManifold* getManifoldByIndexInternal(int index) const
	{
		return m_manifoldsPtr[index];
	}

	void initUnionFind(int n)
	{
		if (m_useIslands)
			m_unionFind.reset(n);
	}
	
	void findUnions();
	
	int m_count;
	
	ParallelIslandDispatcher ();
	virtual ~ParallelIslandDispatcher() {};

	virtual btPersistentManifold*	getNewManifold(void* b0,void* b1);
	
	virtual void releaseManifold(btPersistentManifold* manifold);

	
	virtual void buildAndProcessIslands(btCollisionObjectArray& collisionObjects, IslandCallback* callback);

	///allows the user to get contact point callbacks 
	virtual	btManifoldResult*	getNewManifoldResult(btCollisionObject* obj0,btCollisionObject* obj1,btPersistentManifold* manifold);

	///allows the user to get contact point callbacks 
	virtual	void	releaseManifoldResult(btManifoldResult*);

	virtual void clearManifold(btPersistentManifold* manifold);

			
	btCollisionAlgorithm* findAlgorithm(btBroadphaseProxy& proxy0,btBroadphaseProxy& proxy1)
	{
		btCollisionAlgorithm* algo = internalFindAlgorithm(proxy0,proxy1);
		return algo;
	}
	
	btCollisionAlgorithm* internalFindAlgorithm(btBroadphaseProxy& proxy0,btBroadphaseProxy& proxy1);
	
	virtual bool	needsCollision(btBroadphaseProxy& proxy0,btBroadphaseProxy& proxy1);
	
	virtual bool	needsResponse(const btCollisionObject& colObj0,const btCollisionObject& colObj1);

	virtual int getUniqueId() { return RIGIDBODY_DISPATCHER;}

	virtual void	dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,btDispatcherInfo& dispatchInfo);
	
	

};

#endif //PARALLEL_ISLAND_DISPATCHER_H

