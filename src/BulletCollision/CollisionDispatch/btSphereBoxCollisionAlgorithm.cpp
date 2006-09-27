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

#include "btSphereBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
//#include <stdio.h>

SphereBoxCollisionAlgorithm::SphereBoxCollisionAlgorithm(PersistentManifold* mf,const CollisionAlgorithmConstructionInfo& ci,BroadphaseProxy* proxy0,BroadphaseProxy* proxy1)
: CollisionAlgorithm(ci),
m_ownManifold(false),
m_manifoldPtr(mf)
{
	m_sphereColObj = static_cast<CollisionObject*>(proxy0->m_clientObject);
	m_boxColObj    = static_cast<CollisionObject*>(proxy1->m_clientObject);

	if (!m_manifoldPtr && m_dispatcher->NeedsCollision(*proxy0,*proxy1))
	{
		m_manifoldPtr = m_dispatcher->GetNewManifold(proxy0->m_clientObject,proxy1->m_clientObject);
		m_ownManifold = true;
	}
}


SphereBoxCollisionAlgorithm::~SphereBoxCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->ReleaseManifold(m_manifoldPtr);
	}
}



void SphereBoxCollisionAlgorithm::ProcessCollision (BroadphaseProxy*,BroadphaseProxy*,const DispatcherInfo& dispatchInfo)
{
	
	if (!m_manifoldPtr)
		return;

	SphereShape* sphere0 = (SphereShape*)m_sphereColObj ->m_collisionShape;
	BoxShape* box = (BoxShape*)m_boxColObj->m_collisionShape;

	SimdVector3 normalOnSurfaceB;
	SimdVector3 pOnBox,pOnSphere;
	SimdVector3 sphereCenter = m_sphereColObj->m_worldTransform.getOrigin();
	SimdScalar radius = sphere0->GetRadius();
	
	float dist = GetSphereDistance(pOnBox,pOnSphere,sphereCenter,radius);

	if (dist < SIMD_EPSILON)
	{
		SimdVector3 normalOnSurfaceB = (pOnBox- pOnSphere).normalize();

		/// report a contact. internally this will be kept persistent, and contact reduction is done
		ManifoldResult* resultOut = m_dispatcher->GetNewManifoldResult(m_sphereColObj,m_boxColObj,m_manifoldPtr);
		resultOut->AddContactPoint(normalOnSurfaceB,pOnBox,dist);
		m_dispatcher->ReleaseManifoldResult(resultOut);
	}

	

}

float SphereBoxCollisionAlgorithm::CalculateTimeOfImpact(BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const DispatcherInfo& dispatchInfo)
{
	//not yet
	return 1.f;
}


SimdScalar SphereBoxCollisionAlgorithm::GetSphereDistance( SimdVector3& pointOnBox, SimdVector3& v3PointOnSphere, const SimdVector3& sphereCenter, SimdScalar fRadius ) 
{

	SimdScalar margins;
	SimdVector3 bounds[2];
	BoxShape* boxShape= (BoxShape*)m_boxColObj->m_collisionShape;
	
	bounds[0] = -boxShape->GetHalfExtents();
	bounds[1] = boxShape->GetHalfExtents();

	margins = boxShape->GetMargin();//also add sphereShape margin?

	const SimdTransform&	m44T = m_boxColObj->m_worldTransform;

	SimdVector3	boundsVec[2];
	SimdScalar	fPenetration;

	boundsVec[0] = bounds[0];
	boundsVec[1] = bounds[1];

	SimdVector3	marginsVec( margins, margins, margins );

	// add margins
	bounds[0] += marginsVec;
	bounds[1] -= marginsVec;

	/////////////////////////////////////////////////

	SimdVector3	tmp, prel, n[6], normal, v3P;
	SimdScalar   fSep = 10000000.0f, fSepThis;

	n[0].setValue( -1.0f,  0.0f,  0.0f );
	n[1].setValue(  0.0f, -1.0f,  0.0f );
	n[2].setValue(  0.0f,  0.0f, -1.0f );
	n[3].setValue(  1.0f,  0.0f,  0.0f );
	n[4].setValue(  0.0f,  1.0f,  0.0f );
	n[5].setValue(  0.0f,  0.0f,  1.0f );

	// convert  point in local space
	prel = m44T.invXform( sphereCenter);
	
	bool	bFound = false;

	v3P = prel;

	for (int i=0;i<6;i++)
	{
		int j = i<3? 0:1;
		if ( (fSepThis = ((v3P-bounds[j]) .dot(n[i]))) > 0.0f )
		{
			v3P = v3P - n[i]*fSepThis;		
			bFound = true;
		}
	}
	
	//

	if ( bFound )
	{
		bounds[0] = boundsVec[0];
		bounds[1] = boundsVec[1];

		normal = (prel - v3P).normalize();
		pointOnBox = v3P + normal*margins;
		v3PointOnSphere = prel - normal*fRadius;

		if ( ((v3PointOnSphere - pointOnBox) .dot (normal)) > 0.0f )
		{
			return 1.0f;
		}

		// transform back in world space
		tmp = m44T( pointOnBox);
		pointOnBox    = tmp;
		tmp  = m44T( v3PointOnSphere);		
		v3PointOnSphere = tmp;
		SimdScalar fSeps2 = (pointOnBox-v3PointOnSphere).length2();
		
		//if this fails, fallback into deeper penetration case, below
		if (fSeps2 > SIMD_EPSILON)
		{
			fSep = - SimdSqrt(fSeps2);
			normal = (pointOnBox-v3PointOnSphere);
			normal *= 1.f/fSep;
		}

		return fSep;
	}

	//////////////////////////////////////////////////
	// Deep penetration case

	fPenetration = GetSpherePenetration( pointOnBox, v3PointOnSphere, sphereCenter, fRadius,bounds[0],bounds[1] );

	bounds[0] = boundsVec[0];
	bounds[1] = boundsVec[1];

	if ( fPenetration <= 0.0f )
		return (fPenetration-margins);
	else
		return 1.0f;
}

SimdScalar SphereBoxCollisionAlgorithm::GetSpherePenetration( SimdVector3& pointOnBox, SimdVector3& v3PointOnSphere, const SimdVector3& sphereCenter, SimdScalar fRadius, const SimdVector3& aabbMin, const SimdVector3& aabbMax) 
{

	SimdVector3 bounds[2];

	bounds[0] = aabbMin;
	bounds[1] = aabbMax;

	SimdVector3	p0, tmp, prel, n[6], normal;
	SimdScalar   fSep = -10000000.0f, fSepThis;

	n[0].setValue( -1.0f,  0.0f,  0.0f );
	n[1].setValue(  0.0f, -1.0f,  0.0f );
	n[2].setValue(  0.0f,  0.0f, -1.0f );
	n[3].setValue(  1.0f,  0.0f,  0.0f );
	n[4].setValue(  0.0f,  1.0f,  0.0f );
	n[5].setValue(  0.0f,  0.0f,  1.0f );

	const SimdTransform&	m44T = m_boxColObj->m_worldTransform;

	// convert  point in local space
	prel = m44T.invXform( sphereCenter);

	///////////

	for (int i=0;i<6;i++)
	{
		int j = i<3 ? 0:1;
		if ( (fSepThis = ((prel-bounds[j]) .dot( n[i]))-fRadius) > 0.0f )	return 1.0f;
		if ( fSepThis > fSep )
		{
			p0 = bounds[j];	normal = (SimdVector3&)n[i];
			fSep = fSepThis;
		}
	}

	pointOnBox = prel - normal*(normal.dot((prel-p0)));
	v3PointOnSphere = pointOnBox + normal*fSep;

	// transform back in world space
	tmp  = m44T( pointOnBox);		
	pointOnBox    = tmp;
	tmp  = m44T( v3PointOnSphere);		v3PointOnSphere = tmp;
	normal = (pointOnBox-v3PointOnSphere).normalize();

	return fSep;

}

