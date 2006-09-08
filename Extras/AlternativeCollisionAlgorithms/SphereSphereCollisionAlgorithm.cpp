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

#include "SphereSphereCollisionAlgorithm.h"
#include "CollisionDispatch/CollisionDispatcher.h"
#include "CollisionShapes/SphereShape.h"
#include "CollisionDispatch/CollisionObject.h"

SphereSphereCollisionAlgorithm::SphereSphereCollisionAlgorithm(PersistentManifold* mf,const CollisionAlgorithmConstructionInfo& ci,BroadphaseProxy* proxy0,BroadphaseProxy* proxy1)
: CollisionAlgorithm(ci),
m_ownManifold(false),
m_manifoldPtr(mf)
{
	if (!m_manifoldPtr && m_dispatcher->NeedsCollision(*proxy0,*proxy1))
	{
		m_manifoldPtr = m_dispatcher->GetNewManifold(proxy0->m_clientObject,proxy1->m_clientObject);
		m_ownManifold = true;
	}
}

SphereSphereCollisionAlgorithm::~SphereSphereCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->ReleaseManifold(m_manifoldPtr);
	}
}

void SphereSphereCollisionAlgorithm::ProcessCollision (BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const DispatcherInfo& dispatchInfo)
{

	if (!m_manifoldPtr)
		return;

	CollisionObject*	col0 = static_cast<CollisionObject*>(proxy0->m_clientObject);
	CollisionObject*	col1 = static_cast<CollisionObject*>(proxy1->m_clientObject);

	SphereShape* sphere0 = (SphereShape*)col0->m_collisionShape;
	SphereShape* sphere1 = (SphereShape*)col1->m_collisionShape;

	SimdScalar radius0 = sphere0->GetRadius();
	SimdScalar radius1 = sphere1->GetRadius();

	SimdVector3 diff = col0->m_worldTransform.getOrigin()- 
		col1->m_worldTransform.getOrigin();

	float len = diff.length();
	if ( len > (radius0+radius1))
		return;

	SimdScalar dist = len - (radius0+radius1);

	SimdVector3 normalOnSurfaceB = diff / len;
	SimdVector3 pos0 = col0->m_worldTransform.getOrigin() - radius0 * normalOnSurfaceB;
	SimdVector3 pos1 = col1->m_worldTransform.getOrigin() + radius1* normalOnSurfaceB;

	ManifoldResult* resultOut = m_dispatcher->GetNewManifoldResult(col0,col1,m_manifoldPtr);
	resultOut->AddContactPoint(normalOnSurfaceB,pos1,dist);

}

float SphereSphereCollisionAlgorithm::CalculateTimeOfImpact(BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const DispatcherInfo& dispatchInfo)
{
	//not yet
	return 1.f;
}
