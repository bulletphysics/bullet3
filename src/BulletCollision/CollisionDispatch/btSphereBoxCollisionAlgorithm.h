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

#ifndef SPHERE_BOX_COLLISION_ALGORITHM_H
#define SPHERE_BOX_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
class btPersistentManifold;
#include "LinearMath/btVector3.h"

/// btSphereBoxCollisionAlgorithm  provides sphere-box collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
class btSphereBoxCollisionAlgorithm : public btCollisionAlgorithm
{
	bool	m_ownManifold;
	btPersistentManifold*	m_manifoldPtr;
	btCollisionObject*	m_boxColObj;
	btCollisionObject*	m_sphereColObj;
	
public:

	btSphereBoxCollisionAlgorithm(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);

	virtual ~btSphereBoxCollisionAlgorithm();

	virtual void processCollision (btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,const struct btDispatcherInfo& dispatchInfo);

	virtual float calculateTimeOfImpact(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,const struct btDispatcherInfo& dispatchInfo);

	btScalar getSphereDistance( btVector3& v3PointOnBox, btVector3& v3PointOnSphere, const btVector3& v3SphereCenter, btScalar fRadius );

	btScalar getSpherePenetration( btVector3& v3PointOnBox, btVector3& v3PointOnSphere, const btVector3& v3SphereCenter, btScalar fRadius, const btVector3& aabbMin, const btVector3& aabbMax);
	
	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
		{
			if (!m_swapped)
			{
				return new btSphereBoxCollisionAlgorithm(0,ci,proxy0,proxy1);
			} else
			{
				return new btSphereBoxCollisionAlgorithm(0,ci,proxy1,proxy0);
			}
		}
	};

};

#endif //SPHERE_BOX_COLLISION_ALGORITHM_H