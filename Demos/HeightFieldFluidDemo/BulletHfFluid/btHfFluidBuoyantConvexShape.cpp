#include <stdio.h>

#include "LinearMath/btAabbUtil2.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"

#include "BulletHfFluid/btHfFluidBuoyantConvexShape.h"

btHfFluidBuoyantConvexShape::btHfFluidBuoyantConvexShape (btConvexShape* convexShape)
{
	m_convexShape = convexShape;
	m_shapeType = HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE;
	m_radius = btScalar(0.f);
	m_totalVolume = btScalar(0.0f);
	m_floatyness = btScalar(1.5f);
}

void btHfFluidBuoyantConvexShape::getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
{
	return m_convexShape->getAabb (t, aabbMin, aabbMax);
}

void btHfFluidBuoyantConvexShape::setMargin(btScalar margin)
{
	m_convexShape->setMargin (margin);
}

void btHfFluidBuoyantConvexShape::setLocalScaling(const btVector3& scaling)
{
	m_convexShape->setLocalScaling (scaling);
}

const char*	btHfFluidBuoyantConvexShape::getName() const
{
	return "HF_FLUID_BUOYANT_CONVEX_SHAPE";
}

const btVector3& btHfFluidBuoyantConvexShape::getLocalScaling() const
{
	return m_convexShape->getLocalScaling();
}

void btHfFluidBuoyantConvexShape::calculateLocalInertia(btScalar mass,btVector3& inertia) const
{
	m_convexShape->calculateLocalInertia (mass, inertia);
}

btScalar btHfFluidBuoyantConvexShape::getMargin() const
{
	return m_convexShape->getMargin();
}

//must be above the machine epsilon
#define REL_ERROR2 btScalar(1.0e-6)
static bool intersect(btVoronoiSimplexSolver* simplexSolver, 
					  const btTransform& transformA, 
					  const btTransform& transformB,
					  btConvexShape* a,
					  btConvexShape* b)
{
	
	btScalar squaredDistance = SIMD_INFINITY;
	btTransform	localTransA = transformA;
	btTransform localTransB = transformB;
	btVector3 positionOffset = (localTransA.getOrigin() + localTransB.getOrigin()) * btScalar(0.5);
	localTransA.getOrigin() -= positionOffset;
	localTransB.getOrigin() -= positionOffset;
	btScalar delta = btScalar(0.);
	btVector3 v = btVector3(1.0f, 0.0f, 0.0f);
	simplexSolver->reset ();
	do
	{
			btVector3 seperatingAxisInA = (-v)* transformA.getBasis();
			btVector3 seperatingAxisInB = v* transformB.getBasis();

			btVector3 pInA = a->localGetSupportVertexNonVirtual(seperatingAxisInA);
			btVector3 qInB = b->localGetSupportVertexNonVirtual(seperatingAxisInB);

			btVector3  pWorld = localTransA(pInA);	
			btVector3  qWorld = localTransB(qInB);

			btVector3 w	= pWorld - qWorld;
			delta = v.dot(w);

			// potential exit, they don't overlap
			if ((delta > btScalar(0.0)))
			{
				return false;
			}

			if (simplexSolver->inSimplex (w))
			{
				return false;
			}

			simplexSolver->addVertex (w, pWorld, qWorld);

			if (!simplexSolver->closest(v))
			{
				return false;
			}

			btScalar previousSquaredDistance = squaredDistance;
			squaredDistance = v.length2();

			if (previousSquaredDistance - squaredDistance <= SIMD_EPSILON * previousSquaredDistance) 
			{ 
				return false;
			}
	} while (!simplexSolver->fullSimplex() && squaredDistance > REL_ERROR2 * simplexSolver->maxVertex());

    return true;
}

void btHfFluidBuoyantConvexShape::generateShape (btScalar radius, btScalar gap)
{
	btTransform T;
	T.setIdentity ();
	btVector3 aabbMin, aabbMax;
	getAabb (T, aabbMin, aabbMax);

	m_radius = radius;
	m_numVoxels = 0;

	btVoronoiSimplexSolver simplexSolver;
	btSphereShape sphereShape(radius);
	for (int i = 0; i < MAX_VOXEL_DIMENSION; i++)
	{
		for (int j = 0; j < MAX_VOXEL_DIMENSION; j++)
		{
			for (int k = 0; k < MAX_VOXEL_DIMENSION; k++)
			{
				btVector3 point;
				btTransform sT;
				sT.setIdentity ();
				
				point.setX(aabbMin.getX() + (i * btScalar(2.0f) * radius) + (i * gap));
				point.setY(aabbMin.getY() + (j * btScalar(2.0f) * radius) + (j * gap));
				point.setZ(aabbMin.getZ() + (k * btScalar(2.0f) * radius) + (k * gap));
				
				if (TestPointAgainstAabb2(aabbMin, aabbMax, point))
				{
					btTransform sT;
					sT.setIdentity ();
					sT.setOrigin (point);

					if (intersect (&simplexSolver, T, sT, m_convexShape, &sphereShape))
					{
						m_voxelPositions[m_numVoxels] = point;
						m_numVoxels++;
					}
				}
			}
		}
	}
	m_volumePerVoxel = btScalar(4.0f)/btScalar(3.0f)*SIMD_PI*radius*radius*radius;
	m_totalVolume = m_numVoxels * m_volumePerVoxel;
	m_radius = radius;
}