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

#include "btConvexConvexAlgorithm.h"

#include <stdio.h>
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"



#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"

//#include "NarrowPhaseCollision/EpaPenetrationDepthSolver.h"

#ifdef WIN32
#if _MSC_VER >= 1310
//only use SIMD Hull code under Win32
#ifdef TEST_HULL
#define USE_HULL 1
#endif //TEST_HULL
#endif //_MSC_VER 
#endif //WIN32


#ifdef USE_HULL

#include "NarrowPhaseCollision/Hull.h"
#include "NarrowPhaseCollision/HullContactCollector.h"


#endif //USE_HULL

bool gUseEpa = false;


#ifdef WIN32
void DrawRasterizerLine(const float* from,const float* to,int color);
#endif




//#define PROCESS_SINGLE_CONTACT
#ifdef WIN32
bool gForceBoxBox = false;//false;//true;

#else
bool gForceBoxBox = false;//false;//true;
#endif
bool gBoxBoxUseGjk = true;//true;//false;
bool gDisableConvexCollision = false;



btConvexConvexAlgorithm::btConvexConvexAlgorithm(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
: btCollisionAlgorithm(ci),
m_gjkPairDetector(0,0,&m_simplexSolver,0),
m_useEpa(!gUseEpa),
m_box0(*proxy0),
m_box1(*proxy1),
m_ownManifold (false),
m_manifoldPtr(mf),
m_lowLevelOfDetail(false)
{
	CheckPenetrationDepthSolver();

	{
		if (!m_manifoldPtr && m_dispatcher->NeedsCollision(m_box0,m_box1))
		{
			m_manifoldPtr = m_dispatcher->GetNewManifold(proxy0->m_clientObject,proxy1->m_clientObject);
			m_ownManifold = true;
		}
	}

}



btConvexConvexAlgorithm::~btConvexConvexAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->ReleaseManifold(m_manifoldPtr);
	}
}

void	btConvexConvexAlgorithm ::SetLowLevelOfDetail(bool useLowLevel)
{
	m_lowLevelOfDetail = useLowLevel;
}



class FlippedContactResult : public btDiscreteCollisionDetectorInterface::Result
{
	btDiscreteCollisionDetectorInterface::Result* m_org;

public:

	FlippedContactResult(btDiscreteCollisionDetectorInterface::Result* org)
		: m_org(org)
	{

	}

	virtual void AddContactPoint(const btVector3& normalOnBInWorld,const btVector3& pointInWorld,float depth)
	{
		btVector3 flippedNormal = -normalOnBInWorld;

		m_org->AddContactPoint(flippedNormal,pointInWorld,depth);
	}

};

static btMinkowskiPenetrationDepthSolver	gPenetrationDepthSolver;

//static EpaPenetrationDepthSolver	gEpaPenetrationDepthSolver;

#ifdef USE_EPA
Solid3EpaPenetrationDepth	gSolidEpaPenetrationSolver;
#endif //USE_EPA

void	btConvexConvexAlgorithm::CheckPenetrationDepthSolver()
{
	if (m_useEpa != gUseEpa)
	{
		m_useEpa  = gUseEpa;
		if (m_useEpa)
		{
			
		//	m_gjkPairDetector.SetPenetrationDepthSolver(&gEpaPenetrationDepthSolver);
						
			
		} else
		{
			m_gjkPairDetector.SetPenetrationDepthSolver(&gPenetrationDepthSolver);
		}
	}
	
}

#ifdef USE_HULL

Transform	GetTransformFrombtTransform(const btTransform& trans)
{
			//const btVector3& rowA0 = trans.getBasis().getRow(0);
			////const btVector3& rowA1 = trans.getBasis().getRow(1);
			//const btVector3& rowA2 = trans.getBasis().getRow(2);

			btVector3 rowA0 = trans.getBasis().getColumn(0);
			btVector3 rowA1 = trans.getBasis().getColumn(1);
			btVector3 rowA2 = trans.getBasis().getColumn(2);


			Vector3	x(rowA0.getX(),rowA0.getY(),rowA0.getZ());
			Vector3	y(rowA1.getX(),rowA1.getY(),rowA1.getZ());
			Vector3	z(rowA2.getX(),rowA2.getY(),rowA2.getZ());
			
			Matrix33 ornA(x,y,z);
	
			Point3 transA(
				trans.getOrigin().getX(),
				trans.getOrigin().getY(),
				trans.getOrigin().getZ());

			return Transform(ornA,transA);
}

class btManifoldResultCollector : public HullContactCollector
{
public:
	btManifoldResult& m_manifoldResult;

	btManifoldResultCollector(btManifoldResult& manifoldResult)
		:m_manifoldResult(manifoldResult)
	{

	}
	

	virtual ~btManifoldResultCollector() {};

	virtual int	BatchAddContactGroup(const btSeparation& sep,int numContacts,const Vector3& normalWorld,const Vector3& tangent,const Point3* positionsWorld,const float* depths)
	{
		for (int i=0;i<numContacts;i++)
		{
			//printf("numContacts = %i\n",numContacts);
			btVector3 normalOnBInWorld(sep.m_axis.GetX(),sep.m_axis.GetY(),sep.m_axis.GetZ());
			//normalOnBInWorld.normalize();
			btVector3 pointInWorld(positionsWorld[i].GetX(),positionsWorld[i].GetY(),positionsWorld[i].GetZ());
			float depth = -depths[i];
			m_manifoldResult.AddContactPoint(normalOnBInWorld,pointInWorld,depth);

		}
		return 0;
	}

	virtual int		GetMaxNumContacts() const
	{
		return 4;
	}

};
#endif //USE_HULL


//
// Convex-Convex collision algorithm
//
void btConvexConvexAlgorithm ::ProcessCollision (btBroadphaseProxy* ,btBroadphaseProxy* ,const btDispatcherInfo& dispatchInfo)
{

	if (!m_manifoldPtr)
		return;

	CheckPenetrationDepthSolver();

//	printf("btConvexConvexAlgorithm::ProcessCollision\n");

	bool needsCollision = m_dispatcher->NeedsCollision(m_box0,m_box1);
	if (!needsCollision)
		return;
	
	btCollisionObject*	col0 = static_cast<btCollisionObject*>(m_box0.m_clientObject);
	btCollisionObject*	col1 = static_cast<btCollisionObject*>(m_box1.m_clientObject);

#ifdef USE_HULL


	if (dispatchInfo.m_enableSatConvex)
	{
		if ((col0->m_collisionShape->IsPolyhedral()) &&
			(col1->m_collisionShape->IsPolyhedral()))
		{
		
			
			btPolyhedralConvexShape* polyhedron0 = static_cast<btPolyhedralConvexShape*>(col0->m_collisionShape);
			btPolyhedralConvexShape* polyhedron1 = static_cast<btPolyhedralConvexShape*>(col1->m_collisionShape);
			if (polyhedron0->m_optionalHull && polyhedron1->m_optionalHull)
			{
				//printf("Hull-Hull");

				//todo: cache this information, rather then initialize
				btSeparation sep;
				sep.m_featureA = 0;
				sep.m_featureB = 0;
				sep.m_contact = -1;
				sep.m_separator = 0;

				//convert from btTransform to Transform
				
				Transform trA = GetTransformFrombtTransform(col0->m_worldTransform);
				Transform trB = GetTransformFrombtTransform(col1->m_worldTransform);

				//either use persistent manifold or clear it every time
				m_dispatcher->ClearManifold(m_manifoldPtr);
				btManifoldResult* resultOut = m_dispatcher->GetNewManifoldResult(col0,col1,m_manifoldPtr);

				btManifoldResultCollector hullContactCollector(*resultOut);
				
				Hull::ProcessHullHull(sep,*polyhedron0->m_optionalHull,*polyhedron1->m_optionalHull,
					trA,trB,&hullContactCollector);

				
				//user provided hull's, so we use SAT Hull collision detection
				return;
			}
		}
	}

#endif //USE_HULL

	
	btManifoldResult* resultOut = m_dispatcher->GetNewManifoldResult(col0,col1,m_manifoldPtr);
	
	btConvexShape* min0 = static_cast<btConvexShape*>(col0->m_collisionShape);
	btConvexShape* min1 = static_cast<btConvexShape*>(col1->m_collisionShape);
	
	btGjkPairDetector::ClosestPointInput input;


	//TODO: if (dispatchInfo.m_useContinuous)
	m_gjkPairDetector.SetMinkowskiA(min0);
	m_gjkPairDetector.SetMinkowskiB(min1);
	input.m_maximumDistanceSquared = min0->GetMargin() + min1->GetMargin() + m_manifoldPtr->GetContactBreakingTreshold();
	input.m_maximumDistanceSquared*= input.m_maximumDistanceSquared;
	
//	input.m_maximumDistanceSquared = 1e30f;
	
	input.m_transformA = col0->m_worldTransform;
	input.m_transformB = col1->m_worldTransform;
    
	m_gjkPairDetector.GetClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);

	m_dispatcher->ReleaseManifoldResult(resultOut);
}



bool disableCcd = false;
float	btConvexConvexAlgorithm::CalculateTimeOfImpact(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,const btDispatcherInfo& dispatchInfo)
{
	///Rather then checking ALL pairs, only calculate TOI when motion exceeds treshold
    
	///Linear motion for one of objects needs to exceed m_ccdSquareMotionTreshold
	///col0->m_worldTransform,
	float resultFraction = 1.f;

	btCollisionObject* col1 = static_cast<btCollisionObject*>(m_box1.m_clientObject);
	btCollisionObject* col0 = static_cast<btCollisionObject*>(m_box0.m_clientObject);

	float squareMot0 = (col0->m_interpolationWorldTransform.getOrigin() - col0->m_worldTransform.getOrigin()).length2();
    
	if (squareMot0 < col0->m_ccdSquareMotionTreshold &&
		squareMot0 < col0->m_ccdSquareMotionTreshold)
		return resultFraction;



	if (disableCcd)
		return 1.f;

	CheckPenetrationDepthSolver();

	//An adhoc way of testing the Continuous Collision Detection algorithms
	//One object is approximated as a sphere, to simplify things
	//Starting in penetration should report no time of impact
	//For proper CCD, better accuracy and handling of 'allowed' penetration should be added
	//also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)

	bool needsCollision = m_dispatcher->NeedsCollision(m_box0,m_box1);

	if (!needsCollision)
		return 1.f;
	
		
	/// Convex0 against sphere for Convex1
	{
		btConvexShape* convex0 = static_cast<btConvexShape*>(col0->m_collisionShape);

		btSphereShape	sphere1(col1->m_ccdSweptShereRadius); //todo: allow non-zero sphere sizes, for better approximation
		btConvexCast::CastResult result;
		btVoronoiSimplexSolver voronoiSimplex;
		//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
		///Simplification, one object is simplified as a sphere
		btGjkConvexCast ccd1( convex0 ,&sphere1,&voronoiSimplex);
		//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
		if (ccd1.calcTimeOfImpact(col0->m_worldTransform,col0->m_interpolationWorldTransform,
			col1->m_worldTransform,col1->m_interpolationWorldTransform,result))
		{
		
			//store result.m_fraction in both bodies
		
			if (col0->m_hitFraction	> result.m_fraction)
				col0->m_hitFraction  = result.m_fraction;

			if (col1->m_hitFraction > result.m_fraction)
				col1->m_hitFraction  = result.m_fraction;

			if (resultFraction > result.m_fraction)
				resultFraction = result.m_fraction;

		}
		
		


	}

	/// Sphere (for convex0) against Convex1
	{
		btConvexShape* convex1 = static_cast<btConvexShape*>(col1->m_collisionShape);

		btSphereShape	sphere0(col0->m_ccdSweptShereRadius); //todo: allow non-zero sphere sizes, for better approximation
		btConvexCast::CastResult result;
		btVoronoiSimplexSolver voronoiSimplex;
		//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
		///Simplification, one object is simplified as a sphere
		btGjkConvexCast ccd1(&sphere0,convex1,&voronoiSimplex);
		//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
		if (ccd1.calcTimeOfImpact(col0->m_worldTransform,col0->m_interpolationWorldTransform,
			col1->m_worldTransform,col1->m_interpolationWorldTransform,result))
		{
		
			//store result.m_fraction in both bodies
		
			if (col0->m_hitFraction	> result.m_fraction)
				col0->m_hitFraction  = result.m_fraction;

			if (col1->m_hitFraction > result.m_fraction)
				col1->m_hitFraction  = result.m_fraction;

			if (resultFraction > result.m_fraction)
				resultFraction = result.m_fraction;

		}
	}
	
	return resultFraction;

}
