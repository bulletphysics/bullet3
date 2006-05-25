#include "BulletOdeCollisionPair.h"

#include "NarrowPhaseCollision/PersistentManifold.h"
//quick hack, we need internals, not just the public ode api
#include <../ode/src/collision_kernel.h>
#include "CollisionShapes/BoxShape.h"
#include "BulletOdeCollide.h"

#include "CollisionShapes/SphereShape.h"
#include "CollisionShapes/CylinderShape.h"
#include "CollisionShapes/MultiSphereShape.h"//capped cylinder is convex hull of two spheres
#include "CollisionShapes/ConvexTriangleCallback.h"

#include "BVHTrimeshShape.h"

#include "NarrowPhaseCollision/GjkPairDetector.h"
#include "NarrowPhaseCollision/VoronoiSimplexSolver.h"
#include "NarrowPhaseCollision/MinkowskiPenetrationDepthSolver.h"
///for comparison/unit testing
#ifdef UNIT_TEST_COMPARE_PENETRATION_DEPTH
#include "../ExtraSolid35/Solid3EpaPenetrationDepth.h"
#endif //UNIT_TEST_COMPARE_PENETRATION_DEPTH

#include "BulletOdeManifoldResult.h"
#include "BulletOdeTransformConvert.h"

BulletOdeCollisionPair::BulletOdeCollisionPair(dGeomID o1,dGeomID o2)
{
	m_manifold = new PersistentManifold();
	
	m_o1 = o1;
	m_o2 = o2;

	m_shape1 = CreateShapeFromGeom(o1);

	m_shape2 = CreateShapeFromGeom(o2);
	
}


BulletOdeCollisionPair::~BulletOdeCollisionPair()
{
}

void	BulletOdeCollisionPair::CalculateContacts()
{
	//perform gjk, passing the points to the persistent manifold and convert to ode contacts
	
	if ((!m_shape1) || (!m_shape2))
		return;

	if (m_shape1->IsConcave() && m_shape2->IsConvex())
	{
		TriangleMeshShape* triangleMesh = (TriangleMeshShape*)m_shape1;
		ConvexShape* convexShape2 = (ConvexShape*)m_shape2;

		float collisionMarginTriangle = 0.02f;//triangleMesh->GetMargin();
		SimdTransform triangleMeshTrans = GetTransformFromGeom(m_o1);
		SimdTransform convexTrans = GetTransformFromGeom(m_o2);

		ConvexTriangleCallback	convexTriangleCallback(m_manifold,convexShape2,convexTrans,triangleMeshTrans);
		convexTriangleCallback.Update(collisionMarginTriangle);

		triangleMesh->ProcessAllTriangles( &convexTriangleCallback,convexTriangleCallback.GetAabbMin(),convexTriangleCallback.GetAabbMax());

		m_manifold->RefreshContactPoints(triangleMeshTrans,convexTrans);

	}

	if (m_shape1->IsConvex() && m_shape2->IsConvex())
	{
		ConvexShape* convexShape1 = (ConvexShape*)m_shape1;
		ConvexShape* convexShape2 = (ConvexShape*)m_shape2;


		
		BulletOdeManifoldResult output(m_o1,m_o2,m_manifold);
		
		GjkPairDetector::ClosestPointInput input;
		
		VoronoiSimplexSolver simplexSolver;

#ifdef UNIT_TEST_COMPARE_PENETRATION_DEPTH
		Solid3EpaPenetrationDepth	penetrationDepthSolver;
#else 
		MinkowskiPenetrationDepthSolver	penetrationDepthSolver;
#endif//		UNIT_TEST_COMPARE_PENETRATION_DEPTH

		GjkPairDetector gjkDetector(convexShape1,convexShape2,&simplexSolver,&penetrationDepthSolver);


		gjkDetector.SetMinkowskiA(convexShape1);
		gjkDetector.SetMinkowskiB(convexShape2);
		input.m_maximumDistanceSquared = m_shape1->GetMargin()+ m_shape2->GetMargin() + m_manifold->GetManifoldMargin();
		input.m_maximumDistanceSquared*= input.m_maximumDistanceSquared;

		input.m_maximumDistanceSquared = 1e30;//?
		
		input.m_transformA = GetTransformFromGeom(m_o1);
		input.m_transformB = GetTransformFromGeom(m_o2);
		 
		gjkDetector.GetClosestPoints(input,output,0);

		m_manifold->RefreshContactPoints(input.m_transformA,input.m_transformB);

	}
}

void BulletOdeCollisionPair::ProcessCollision (BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const struct DispatcherInfo& dispatchInfo)
{

}

float BulletOdeCollisionPair::CalculateTimeOfImpact(BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const struct DispatcherInfo& dispatchInfo)
{
	return 1.f;
}


CollisionShape*	BulletOdeCollisionPair::CreateShapeFromGeom(dGeomID geom)
{
	CollisionShape* shape = 0;

	if (geom->type == dConvexClass)
	{
		shape = GetCollisionShapeFromConvex(geom);
	} else
	{
		//bullet shape versus ode geom, create a compatible shape from the ode types
			switch (geom->type)
			{
			case dPlaneClass:
				break;
			case dBoxClass:
				{
					dVector3 size;
					dGeomBoxGetLengths (geom,size);
					SimdVector3 halfExtents(0.5f*size[0],0.5f*size[1],0.5f*size[2]);
					shape = new BoxShape(halfExtents);
					break;
				}
				case dSphereClass:
				{
					dVector3 size;
					shape = new SphereShape(dGeomSphereGetRadius(geom));
					break;
				}
				case dCylinderClass:
				{
					dVector3 size;
					dGeomBoxGetLengths (geom,size);
					SimdVector3 boxHalfExtents(size[0],size[0],size[1]);
					shape = new CylinderShapeZ(boxHalfExtents);
					break;
				}
				case dCCylinderClass:
				{
					dReal radius,length;
					dGeomCCylinderGetParams (geom, &radius, &length);
					SimdVector3 boxHalfExtents(radius,radius,0.5f*length);
					int numspheres = 2;
					SimdVector3 centers[2]={ SimdVector3(0,0,0.5f*length),SimdVector3(0,0,-0.5f*length)};
					SimdScalar radi[2]={radius,radius};
					shape = new MultiSphereShape(boxHalfExtents,centers,radi,numspheres);
					break;
				} 
				case dTriMeshClass:
					{
						dxTriMesh* trimesh = (dxTriMesh*) geom;
						shape = 0;//new BVHTrimeshShape(trimesh);
						break;
					}


			default:
				{
				}
			};

	}
	
	return shape;

}
