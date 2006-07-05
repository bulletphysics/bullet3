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


#include "ConvexConcaveCollisionAlgorithm.h"
#include "CollisionDispatch/CollisionObject.h"
#include "CollisionShapes/MultiSphereShape.h"
#include "ConvexConvexAlgorithm.h"
#include "BroadphaseCollision/BroadphaseProxy.h"
#include "CollisionShapes/ConcaveShape.h"
#include "CollisionDispatch/ManifoldResult.h"
#include "NarrowPhaseCollision/RaycastCallback.h"
#include "CollisionShapes/TriangleShape.h"
#include "IDebugDraw.h"

ConvexConcaveCollisionAlgorithm::ConvexConcaveCollisionAlgorithm( const CollisionAlgorithmConstructionInfo& ci,BroadphaseProxy* proxy0,BroadphaseProxy* proxy1)
: CollisionAlgorithm(ci),m_convex(*proxy0),m_concave(*proxy1),
m_ConvexTriangleCallback(ci.m_dispatcher,proxy0,proxy1)

{
}

ConvexConcaveCollisionAlgorithm::~ConvexConcaveCollisionAlgorithm()
{
}



ConvexTriangleCallback::ConvexTriangleCallback(Dispatcher*  dispatcher,BroadphaseProxy* proxy0,BroadphaseProxy* proxy1):
  m_convexProxy(proxy0),m_triangleProxy(*proxy1),m_dispatcher(dispatcher),
	m_dispatchInfoPtr(0)
{

	  //
	  // create the manifold from the dispatcher 'manifold pool'
	  //
	  m_manifoldPtr = m_dispatcher->GetNewManifold(proxy0->m_clientObject,proxy1->m_clientObject);

  	  ClearCache();
}

ConvexTriangleCallback::~ConvexTriangleCallback()
{
	ClearCache();
	m_dispatcher->ReleaseManifold( m_manifoldPtr );
  
}
  

void	ConvexTriangleCallback::ClearCache()
{
	m_dispatcher->ClearManifold(m_manifoldPtr);
};



void ConvexTriangleCallback::ProcessTriangle(SimdVector3* triangle,int partId, int triangleIndex)
{
 
	//just for debugging purposes
	//printf("triangle %d",m_triangleCount++);


	//aabb filter is already applied!	

	CollisionAlgorithmConstructionInfo ci;
	ci.m_dispatcher = m_dispatcher;

	CollisionObject* ob = static_cast<CollisionObject*>(m_triangleProxy.m_clientObject);


	
	///debug drawing of the overlapping triangles
	if (m_dispatchInfoPtr && m_dispatchInfoPtr->m_debugDraw && m_dispatchInfoPtr->m_debugDraw->GetDebugMode() > 0)
	{
		SimdVector3 color(255,255,0);
		SimdTransform& tr = ob->m_worldTransform;
		m_dispatchInfoPtr->m_debugDraw->DrawLine(tr(triangle[0]),tr(triangle[1]),color);
		m_dispatchInfoPtr->m_debugDraw->DrawLine(tr(triangle[1]),tr(triangle[2]),color);
		m_dispatchInfoPtr->m_debugDraw->DrawLine(tr(triangle[2]),tr(triangle[0]),color);

		//SimdVector3 center = triangle[0] + triangle[1]+triangle[2];
		//center *= 0.333333f;
		//m_dispatchInfoPtr->m_debugDraw->DrawLine(tr(triangle[0]),tr(center),color);
		//m_dispatchInfoPtr->m_debugDraw->DrawLine(tr(triangle[1]),tr(center),color);
		//m_dispatchInfoPtr->m_debugDraw->DrawLine(tr(triangle[2]),tr(center),color);

	}


	CollisionObject* colObj = static_cast<CollisionObject*>(m_convexProxy->m_clientObject);
	
	if (colObj->m_collisionShape->IsConvex())
	{
		TriangleShape tm(triangle[0],triangle[1],triangle[2]);	
		tm.SetMargin(m_collisionMarginTriangle);
	
		
		CollisionShape* tmpShape = ob->m_collisionShape;
		ob->m_collisionShape = &tm;
		
		ConvexConvexAlgorithm cvxcvxalgo(m_manifoldPtr,ci,m_convexProxy,&m_triangleProxy);
		cvxcvxalgo.ProcessCollision(m_convexProxy,&m_triangleProxy,*m_dispatchInfoPtr);
		ob->m_collisionShape = tmpShape;

	}

	

}



void	ConvexTriangleCallback::SetTimeStepAndCounters(float collisionMarginTriangle,const DispatcherInfo& dispatchInfo)
{
	m_dispatchInfoPtr = &dispatchInfo;
	m_collisionMarginTriangle = collisionMarginTriangle;

	//recalc aabbs
	CollisionObject* convexBody = (CollisionObject* )m_convexProxy->m_clientObject;
	CollisionObject* triBody = (CollisionObject* )m_triangleProxy.m_clientObject;

	SimdTransform convexInTriangleSpace;
	convexInTriangleSpace = triBody->m_worldTransform.inverse() * convexBody->m_worldTransform;

	CollisionShape* convexShape = static_cast<CollisionShape*>(convexBody->m_collisionShape);
	//CollisionShape* triangleShape = static_cast<CollisionShape*>(triBody->m_collisionShape);

	convexShape->GetAabb(convexInTriangleSpace,m_aabbMin,m_aabbMax);

	float extraMargin = collisionMarginTriangle;//CONVEX_DISTANCE_MARGIN;//+0.1f;

	SimdVector3 extra(extraMargin,extraMargin,extraMargin);

	m_aabbMax += extra;
	m_aabbMin -= extra;
	
}

void ConvexConcaveCollisionAlgorithm::ClearCache()
{
	m_ConvexTriangleCallback.ClearCache();

}

void ConvexConcaveCollisionAlgorithm::ProcessCollision (BroadphaseProxy* ,BroadphaseProxy* ,const DispatcherInfo& dispatchInfo)
{
	
	CollisionObject* convexBody = static_cast<CollisionObject* >(m_convex.m_clientObject);
	CollisionObject* triBody = static_cast<CollisionObject* >(m_concave.m_clientObject);

	if (triBody->m_collisionShape->IsConcave())
	{

		if (!m_dispatcher->NeedsCollision(m_convex,m_concave))
			return;

		

		CollisionObject*	triOb = static_cast<CollisionObject*>(m_concave.m_clientObject);
		ConcaveShape* concaveShape = static_cast<ConcaveShape*>( triOb->m_collisionShape);
		
		if (convexBody->m_collisionShape->IsConvex())
		{
			float collisionMarginTriangle = concaveShape->GetMargin();
					
			m_ConvexTriangleCallback.SetTimeStepAndCounters(collisionMarginTriangle,dispatchInfo);

			//Disable persistency. previously, some older algorithm calculated all contacts in one go, so you can clear it here.
			//m_dispatcher->ClearManifold(m_ConvexTriangleCallback.m_manifoldPtr);


			m_ConvexTriangleCallback.m_manifoldPtr->SetBodies(m_convex.m_clientObject,m_concave.m_clientObject);

			concaveShape->ProcessAllTriangles( &m_ConvexTriangleCallback,m_ConvexTriangleCallback.GetAabbMin(),m_ConvexTriangleCallback.GetAabbMax());
			
	
		}

	}

}


float ConvexConcaveCollisionAlgorithm::CalculateTimeOfImpact(BroadphaseProxy* ,BroadphaseProxy* ,const DispatcherInfo& dispatchInfo)
{

	//quick approximation using raycast, todo: hook up to the continuous collision detection (one of the ConvexCast)
	CollisionObject* convexbody = (CollisionObject* )m_convex.m_clientObject;
	CollisionObject* triBody = static_cast<CollisionObject* >(m_concave.m_clientObject);

	const SimdVector3& from = convexbody->m_worldTransform.getOrigin();
	
	SimdVector3 to = convexbody->m_interpolationWorldTransform.getOrigin();
	//todo: only do if the motion exceeds the 'radius'

	struct LocalTriangleRaycastCallback	: public TriangleRaycastCallback
	{
		LocalTriangleRaycastCallback(const SimdVector3& from,const SimdVector3& to)
			:TriangleRaycastCallback(from,to)
		{
		}
		
		virtual float ReportHit(const SimdVector3& hitNormalLocal, float hitFraction, int partId, int triangleIndex )
		{
			//todo: handle ccd here
			return 0.f;

		}
	};


	LocalTriangleRaycastCallback raycastCallback(from,to);

	raycastCallback.m_hitFraction = convexbody->m_hitFraction;

	SimdVector3 aabbMin (-1e30f,-1e30f,-1e30f);
	SimdVector3 aabbMax (SIMD_INFINITY,SIMD_INFINITY,SIMD_INFINITY);

	if (triBody->m_collisionShape->IsConcave())
	{

		CollisionObject* concavebody = (CollisionObject* )m_concave.m_clientObject;

		ConcaveShape* triangleMesh = (ConcaveShape*) concavebody->m_collisionShape;
		
		if (triangleMesh)
		{
			triangleMesh->ProcessAllTriangles(&raycastCallback,aabbMin,aabbMax);
		}
	}


	if (raycastCallback.m_hitFraction < convexbody->m_hitFraction)
	{
		convexbody->m_hitFraction = raycastCallback.m_hitFraction;
		return raycastCallback.m_hitFraction;
	}

	return 1.f;

}
