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
class PersistentManifold;
#include "LinearMath/SimdVector3.h"

/// SphereBoxCollisionAlgorithm  provides sphere-box collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
class SphereBoxCollisionAlgorithm : public CollisionAlgorithm
{
	bool	m_ownManifold;
	PersistentManifold*	m_manifoldPtr;
	CollisionObject*	m_boxColObj;
	CollisionObject*	m_sphereColObj;
	
public:

	SphereBoxCollisionAlgorithm(PersistentManifold* mf,const CollisionAlgorithmConstructionInfo& ci,BroadphaseProxy* proxy0,BroadphaseProxy* proxy1);

	virtual ~SphereBoxCollisionAlgorithm();

	virtual void ProcessCollision (BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const DispatcherInfo& dispatchInfo);

	virtual float CalculateTimeOfImpact(BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const DispatcherInfo& dispatchInfo);
	
	SimdScalar GetSphereDistance( SimdVector3& v3PointOnBox, SimdVector3& v3PointOnSphere, const SimdVector3& v3SphereCenter, SimdScalar fRadius );

	SimdScalar GetSpherePenetration( SimdVector3& v3PointOnBox, SimdVector3& v3PointOnSphere, const SimdVector3& v3SphereCenter, SimdScalar fRadius, const SimdVector3& aabbMin, const SimdVector3& aabbMax);
	
	struct CreateFunc :public 	CollisionAlgorithmCreateFunc
	{
		virtual	CollisionAlgorithm* CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo& ci, BroadphaseProxy* proxy0,BroadphaseProxy* proxy1)
		{
			if (!m_swapped)
			{
				return new SphereBoxCollisionAlgorithm(0,ci,proxy0,proxy1);
			} else
			{
				return new SphereBoxCollisionAlgorithm(0,ci,proxy1,proxy0);
			}
		}
	};

};

#endif //SPHERE_BOX_COLLISION_ALGORITHM_H