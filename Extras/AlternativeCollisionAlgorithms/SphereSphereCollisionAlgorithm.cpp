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

#include "btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

btSphereSphereCollisionAlgorithm::btSphereSphereCollisionAlgorithm(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
: btCollisionAlgorithm(ci),
m_ownManifold(false),
m_manifoldPtr(mf)
{
	if (!m_manifoldPtr && m_dispatcher->NeedsCollision(*proxy0,*proxy1))
	{
		m_manifoldPtr = m_dispatcher->GetNewManifold(proxy0->m_clientObject,proxy1->m_clientObject);
		m_ownManifold = true;
	}
}

btSphereSphereCollisionAlgorithm::~btSphereSphereCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->ReleaseManifold(m_manifoldPtr);
	}
}

void btSphereSphereCollisionAlgorithm::ProcessCollision (btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,const btDispatcherInfo& dispatchInfo)
{
	if (!m_manifoldPtr)
		return;

	btCollisionObject*	col0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
	btCollisionObject*	col1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);
	btSphereShape* sphere0 = (btSphereShape*)col0->m_collisionShape;
	btSphereShape* sphere1 = (btSphereShape*)col1->m_collisionShape;

	btVector3 diff = col0->m_worldTransform.getOrigin()-  col1->m_worldTransform.getOrigin();
	float len = diff.length();
	btScalar radius0 = sphere0->GetRadius();
	btScalar radius1 = sphere1->GetRadius();

	///iff distance positive, don't generate a new contact
	if ( len > (radius0+radius1))
		return;

	///distance (negative means penetration)
	btScalar dist = len - (radius0+radius1);

	btVector3 normalOnSurfaceB = diff / len;
	///point on A (worldspace)
	btVector3 pos0 = col0->m_worldTransform.getOrigin() - radius0 * normalOnSurfaceB;
	///point on B (worldspace)
	btVector3 pos1 = col1->m_worldTransform.getOrigin() + radius1* normalOnSurfaceB;

	/// report a contact. internally this will be kept persistent, and contact reduction is done
	btManifoldResult* resultOut = m_dispatcher->GetNewManifoldResult(col0,col1,m_manifoldPtr);
	resultOut->AddContactPoint(normalOnSurfaceB,pos1,dist);
	m_dispatcher->ReleaseManifoldResult(resultOut);

}

float btSphereSphereCollisionAlgorithm::CalculateTimeOfImpact(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,const btDispatcherInfo& dispatchInfo)
{
	//not yet
	return 1.f;
}
