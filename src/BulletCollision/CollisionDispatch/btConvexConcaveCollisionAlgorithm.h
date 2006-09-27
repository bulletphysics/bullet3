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

#ifndef CONVEX_CONCAVE_COLLISION_ALGORITHM_H
#define CONVEX_CONCAVE_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "btCollisionCreateFunc.h"

///For each triangle in the concave mesh that overlaps with the AABB of a convex (m_convexProxy), ProcessTriangle is called.
class btConvexTriangleCallback : public btTriangleCallback
{
	btBroadphaseProxy* m_convexProxy;
	btBroadphaseProxy m_triangleProxy;

	btVector3	m_aabbMin;
	btVector3	m_aabbMax ;

	btDispatcher*	m_dispatcher;
	const btDispatcherInfo* m_dispatchInfoPtr;
	float m_collisionMarginTriangle;
	
public:
int	m_triangleCount;
	
	btPersistentManifold*	m_manifoldPtr;

	btConvexTriangleCallback(btDispatcher* dispatcher,btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);

	void	SetTimeStepAndCounters(float collisionMarginTriangle,const btDispatcherInfo& dispatchInfo);

	virtual ~btConvexTriangleCallback();

	virtual void ProcessTriangle(btVector3* triangle, int partId, int triangleIndex);
	
	void ClearCache();

	inline const btVector3& GetAabbMin() const
	{
		return m_aabbMin;
	}
	inline const btVector3& GetAabbMax() const
	{
		return m_aabbMax;
	}

};




/// btConvexConcaveCollisionAlgorithm  supports collision between convex shapes and (concave) trianges meshes.
class btConvexConcaveCollisionAlgorithm  : public btCollisionAlgorithm
{

	btBroadphaseProxy m_convex;

	btBroadphaseProxy m_concave;

	btConvexTriangleCallback m_btConvexTriangleCallback;


public:

	btConvexConcaveCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci,btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);

	virtual ~btConvexConcaveCollisionAlgorithm();

	virtual void ProcessCollision (btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,const btDispatcherInfo& dispatchInfo);

	float	CalculateTimeOfImpact(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,const btDispatcherInfo& dispatchInfo);

	void	ClearCache();

	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
		{
			return new btConvexConcaveCollisionAlgorithm(ci,proxy0,proxy1);
		}
	};

	struct SwappedCreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
		{
			return new btConvexConcaveCollisionAlgorithm(ci,proxy1,proxy0);
		}
	};

};

#endif //CONVEX_CONCAVE_COLLISION_ALGORITHM_H
