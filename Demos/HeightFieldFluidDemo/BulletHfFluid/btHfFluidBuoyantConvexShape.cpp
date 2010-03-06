/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Experimental Buoyancy fluid demo written by John McCutchan
*/
#include <stdio.h>

#include "LinearMath/btAabbUtil2.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"

#include "btHfFluidBuoyantConvexShape.h"

btHfFluidBuoyantConvexShape::btHfFluidBuoyantConvexShape (btConvexShape* convexShape)
{
	m_convexShape = convexShape;
	m_shapeType = HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE;
	m_radius = btScalar(0.f);
	m_numVoxels = 0;
	m_voxelPositions = NULL;
	m_totalVolume = btScalar(0.0f);
	m_floatyness = btScalar(1.5f);
}

btHfFluidBuoyantConvexShape::~btHfFluidBuoyantConvexShape ()
{
	if (m_voxelPositions)
		btAlignedFree (m_voxelPositions);
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
	btVector3* voxelPositions = (btVector3*)btAlignedAlloc (sizeof(btVector3)*MAX_VOXEL_DIMENSION*MAX_VOXEL_DIMENSION*MAX_VOXEL_DIMENSION,16);
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
						voxelPositions[m_numVoxels] = point;
						m_numVoxels++;
					}
				}
			}
		}
	}
	m_voxelPositions = (btVector3*)btAlignedAlloc (sizeof(btVector3)*m_numVoxels, 16);
	for (int i = 0; i < m_numVoxels;i++)
	{
		m_voxelPositions[i] = voxelPositions[i];
	}
	btAlignedFree (voxelPositions);
	m_volumePerVoxel = btScalar(4.0f)/btScalar(3.0f)*SIMD_PI*radius*radius*radius;
	m_totalVolume = m_numVoxels * m_volumePerVoxel;
	m_radius = radius;
}

