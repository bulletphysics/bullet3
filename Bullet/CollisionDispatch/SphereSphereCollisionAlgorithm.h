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

#ifndef SPHERE_SPHERE_COLLISION_ALGORITHM_H
#define SPHERE_SPHERE_COLLISION_ALGORITHM_H

#include "BroadphaseCollision/CollisionAlgorithm.h"
#include "BroadphaseCollision/BroadphaseProxy.h"
#include "CollisionDispatch/CollisionCreateFunc.h"
class PersistentManifold;

/// SphereSphereCollisionAlgorithm  provides sphere-sphere collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
/// Also provides the most basic sample for custom/user CollisionAlgorithm
class SphereSphereCollisionAlgorithm : public CollisionAlgorithm
{
	bool	m_ownManifold;
	PersistentManifold*	m_manifoldPtr;
	
public:
	SphereSphereCollisionAlgorithm(const CollisionAlgorithmConstructionInfo& ci)
		: CollisionAlgorithm(ci) {}

	virtual void ProcessCollision (BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const DispatcherInfo& dispatchInfo);

	virtual float CalculateTimeOfImpact(BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const DispatcherInfo& dispatchInfo);

	SphereSphereCollisionAlgorithm(PersistentManifold* mf,const CollisionAlgorithmConstructionInfo& ci,BroadphaseProxy* proxy0,BroadphaseProxy* proxy1);

	virtual ~SphereSphereCollisionAlgorithm();

	struct CreateFunc :public 	CollisionAlgorithmCreateFunc
	{
		virtual	CollisionAlgorithm* CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo& ci, BroadphaseProxy* proxy0,BroadphaseProxy* proxy1)
		{
			return new SphereSphereCollisionAlgorithm(0,ci,proxy0,proxy1);
		}
	};

};

#endif //SPHERE_SPHERE_COLLISION_ALGORITHM_H