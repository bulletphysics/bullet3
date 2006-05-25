#include "BulletOdeManifoldResult.h"
#include "NarrowPhaseCollision/PersistentManifold.h"
#include "BulletOdeTransformConvert.h"

BulletOdeManifoldResult::BulletOdeManifoldResult(dGeomID geom1,dGeomID geom2,PersistentManifold* manifoldPtr)
		:m_manifoldPtr(manifoldPtr),
		m_geom1(geom1),
		m_geom2(geom2)
{
}

void BulletOdeManifoldResult::AddContactPoint(const SimdVector3& normalOnBInWorld,const SimdVector3& pointInWorld,float depth)
{
	if (depth > m_manifoldPtr->GetManifoldMargin())
		return;

	SimdTransform transAInv = GetTransformFromGeom(m_geom1).inverse();
	SimdTransform transBInv = GetTransformFromGeom(m_geom2).inverse();

	SimdVector3 pointA = pointInWorld + normalOnBInWorld * depth;
	SimdVector3 localA = transAInv(pointA );
	SimdVector3 localB = transBInv(pointInWorld);
	ManifoldPoint newPt(localA,localB,normalOnBInWorld,depth);

	int insertIndex = m_manifoldPtr->GetCacheEntry(newPt);
	if (insertIndex >= 0)
	{
		m_manifoldPtr->ReplaceContactPoint(newPt,insertIndex);
	} else
	{
		m_manifoldPtr->AddManifoldPoint(newPt);
	}
}