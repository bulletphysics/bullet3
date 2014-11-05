/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btConvexConvexMprAlgorithm.h"

//#include <stdio.h>
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"

#include "BulletCollision/CollisionShapes/btTriangleShape.h"



#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"



#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

#include "BulletCollision/NarrowPhaseCollision/btComputeGjkEpaPenetration.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa3.h"
#include "BulletCollision/NarrowPhaseCollision/btMprPenetration.h"

//this is just an internal debug variable to switch between GJK+MPR or GJK+EPA
bool gUseMprCollisionFunction = true;

btConvexConvexMprAlgorithm::CreateFunc::CreateFunc()
{

}

btConvexConvexMprAlgorithm::CreateFunc::~CreateFunc() 
{ 
}

btConvexConvexMprAlgorithm::btConvexConvexMprAlgorithm(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap)
: btActivatingCollisionAlgorithm(ci,body0Wrap,body1Wrap),
m_ownManifold (false),
m_manifoldPtr(mf)
{
	(void)body0Wrap;
	(void)body1Wrap;
}




btConvexConvexMprAlgorithm::~btConvexConvexMprAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}


btVector3 btBulletShapeSupportFunc(const void* shapeAptr, const btVector3& dir, bool includeMargin)
{
	btConvexShape* shape = (btConvexShape*) shapeAptr;
	if (includeMargin)
	{
		return shape->localGetSupportingVertex(dir);
	}
	
	return shape->localGetSupportingVertexWithoutMargin(dir);
}

btVector3 btBulletShapeCenterFunc(const void* shapeAptr)
{
	return btVector3(0,0,0);
}


struct btMprConvexWrap
{
	const btConvexShape* m_convex;
	btTransform m_worldTrans;
	inline btScalar getMargin() const
	{
		return m_convex->getMargin();
	}
	inline btVector3 getObjectCenterInWorld() const
	{
		return m_worldTrans.getOrigin();
	}
	inline const btTransform& getWorldTransform() const
	{
		return m_worldTrans;
	}
	inline btVector3 getLocalSupportWithMargin(const btVector3& dir) const
	{
		return m_convex->localGetSupportingVertex(dir);
	}
	inline btVector3 getLocalSupportWithoutMargin(const btVector3& dir) const
	{
		return m_convex->localGetSupportingVertexWithoutMargin(dir);
	}
};

struct btMyDistanceInfo
{
	btVector3	m_pointOnA;
	btVector3	m_pointOnB;
	btVector3	m_normalBtoA;
	btScalar	m_distance;
};

//
// Convex-Convex collision algorithm
//
void btConvexConvexMprAlgorithm ::processCollision (const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{

	if (!m_manifoldPtr)
	{
		//swapped?
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(),body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
	resultOut->setPersistentManifold(m_manifoldPtr);

	//comment-out next line to test multi-contact generation
	//resultOut->getPersistentManifold()->clearManifold();
	

	const btConvexShape* min0 = static_cast<const btConvexShape*>(body0Wrap->getCollisionShape());
	const btConvexShape* min1 = static_cast<const btConvexShape*>(body1Wrap->getCollisionShape());

	btVector3  normalOnB;
	btVector3  pointOnBWorld;

	btGjkPairDetector::ClosestPointInput input;

	btVoronoiSimplexSolver vs;
	btGjkEpaPenetrationDepthSolver epa;
	
	
	if (gUseMprCollisionFunction)
	{

		btMprConvexWrap a,b;
		a.m_worldTrans = body0Wrap->getWorldTransform();
		b.m_worldTrans = body1Wrap->getWorldTransform();
		a.m_convex = (const btConvexShape*)body0Wrap->getCollisionShape();
		b.m_convex = (const btConvexShape*)body1Wrap->getCollisionShape();
		btVoronoiSimplexSolver simplexSolver;
		simplexSolver.reset();
		btGjkCollisionDescription colDesc;
		btMyDistanceInfo distInfo;
		int res = btComputeGjkDistance(a,b,colDesc,&distInfo);
		if (res==0)
		{
			   //printf("use GJK results in distance %f\n",distInfo.m_distance);
		} else
		{
			btMprCollisionDescription mprDesc;
			res = btComputeMprPenetration(a,b,mprDesc, &distInfo);

			//printf("use MPR results in distance %f\n",distInfo.m_distance);

		}
		if (res == 0)
		{
#if 0
			printf("Dist=%f,normalOnB[%f,%f,%f],pA=[%f,%f,%f],pB[%f,%f,%f]\n",
				distInfo.m_distance, distInfo.m_normalBtoA[0], distInfo.m_normalBtoA[1], distInfo.m_normalBtoA[2],
				distInfo.m_pointOnA[0], distInfo.m_pointOnA[1], distInfo.m_pointOnA[2],
				distInfo.m_pointOnB[0], distInfo.m_pointOnB[1], distInfo.m_pointOnB[2]);
#endif

			if (distInfo.m_distance<=0)
			{
				resultOut->addContactPoint(distInfo.m_normalBtoA, distInfo.m_pointOnB, distInfo.m_distance);
			}
			//ASSERT_EQ(0,result);
			//ASSERT_NEAR(btFabs(btScalar(i-z))-btScalar(j)-ssd.m_radiusB, distInfo.m_distance, abs_error);
			//btVector3 computedA = distInfo.m_pointOnB+distInfo.m_distance*distInfo.m_normalBtoA;
			//ASSERT_NEAR(computedA.x(),distInfo.m_pointOnA.x(),abs_error);
			//ASSERT_NEAR(computedA.y(),distInfo.m_pointOnA.y(),abs_error);
			//ASSERT_NEAR(computedA.z(),distInfo.m_pointOnA.z(),abs_error);
		}


#if 0
		btCollisionDescription colDesc;
		colDesc.m_objA = min0;
		colDesc.m_objB = min1;
		colDesc.m_localSupportFuncA = &btBulletShapeSupportFunc;
		colDesc.m_localSupportFuncB = &btBulletShapeSupportFunc;
		colDesc.m_localOriginFuncA = &btBulletShapeCenterFunc;
		colDesc.m_localOriginFuncB = &btBulletShapeCenterFunc;

		colDesc.m_transformA = body0Wrap->getWorldTransform();
		colDesc.m_transformB = body1Wrap->getWorldTransform();
		colDesc.m_marginA = body0Wrap->getCollisionShape()->getMargin();
		colDesc.m_marginB = body1Wrap->getCollisionShape()->getMargin();
		btDistanceInfo distInfo;
		//int	result = btComputeGjkEpaPenetration(colDesc, &distInfo);
		//int	result = btComputeGjkEpaPenetration2(colDesc, &distInfo);
		int	result = btComputeMprPenetration(colDesc, &distInfo);
		
		if (result==0)
		{
			resultOut->addContactPoint(distInfo.m_normalBtoA,distInfo.m_pointOnB,distInfo.m_distance);
		}

		//bool res = b3MprPenetration(pairIndex,bodyIndexA,bodyIndexB,cpuBodyBuf,convexData,collidable2,cpuVertices,sepAxis,hasSepAxis,depthOut,dirOut,posOut);
	
		/*btCollisionDescription colDesc;
		btDistanceInfo distInfo;
		int	btComputeGjkEpaPenetration(min0, min1, &colDesc, &distInfo);
		*/
#endif
	} else
	{

		btGjkPairDetector	gjkPairDetector(min0,min1,&vs,&epa);//m_simplexSolver,m_pdSolver);
		//TODO: if (dispatchInfo.m_useContinuous)
		gjkPairDetector.setMinkowskiA(min0);
		gjkPairDetector.setMinkowskiB(min1);

		{
			//if (dispatchInfo.m_convexMaxDistanceUseCPT)
			//{
			//	input.m_maximumDistanceSquared = min0->getMargin() + min1->getMargin() + m_manifoldPtr->getContactProcessingThreshold();
			//} else
			//{
			input.m_maximumDistanceSquared = min0->getMargin() + min1->getMargin() + m_manifoldPtr->getContactBreakingThreshold();
	//		}

			input.m_maximumDistanceSquared*= input.m_maximumDistanceSquared;
		}

		input.m_transformA = body0Wrap->getWorldTransform();
		input.m_transformB = body1Wrap->getWorldTransform();


		gjkPairDetector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);
	}
	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}

}


btScalar	btConvexConvexMprAlgorithm::calculateTimeOfImpact(btCollisionObject* col0,btCollisionObject* col1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	btAssert(0);
	return 0;
}

