/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

EPA Copyright (c) Ricardo Padrela 2006

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btPoint3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btSimdMinMax.h"

#include <list>

#include "BulletCollision/CollisionShapes/btConvexShape.h"

#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"

#include "NarrowPhaseCollision/EpaCommon.h"

#include "NarrowPhaseCollision/EpaVertex.h"
#include "NarrowPhaseCollision/EpaHalfEdge.h"
#include "NarrowPhaseCollision/EpaFace.h"
#include "NarrowPhaseCollision/EpaPolyhedron.h"
#include "NarrowPhaseCollision/Epa.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "NarrowPhaseCollision/EpaPenetrationDepthSolver.h"

btScalar	g_GJKMaxRelError = 1e-3f;
btScalar	g_GJKMaxRelErrorSqrd = g_GJKMaxRelError * g_GJKMaxRelError;

bool EpaPenetrationDepthSolver::calcPenDepth( btSimplexSolverInterface& simplexSolver,
											  btConvexShape* pConvexA, btConvexShape* pConvexB,
											  const btTransform& transformA, const btTransform& transformB,
											  btVector3& v, btPoint3& wWitnessOnA, btPoint3& wWitnessOnB,
											  class btIDebugDraw* debugDraw )
{
	EPA_DEBUG_ASSERT( pConvexA ,"Convex shape A is invalid!" );
	EPA_DEBUG_ASSERT( pConvexB ,"Convex shape B is invalid!" );

	btScalar penDepth;

#ifdef EPA_USE_HYBRID
	bool needsEPA = !HybridPenDepth( simplexSolver, pConvexA, pConvexB, transformA, transformB,
									 wWitnessOnA, wWitnessOnB, penDepth, v );

	if ( needsEPA )
	{
#endif
		penDepth = EpaPenDepth( simplexSolver, pConvexA, pConvexB,
								transformA, transformB,
								wWitnessOnA, wWitnessOnB );
		EPA_DEBUG_ASSERT( ( penDepth > 0 ) ,"EPA or Hybrid Technique failed to calculate penetration depth!" );

#ifdef EPA_USE_HYBRID
	}
#endif

	return ( penDepth > 0 );
}

#ifdef EPA_USE_HYBRID
bool EpaPenetrationDepthSolver::HybridPenDepth( btSimplexSolverInterface& simplexSolver,
												btConvexShape* pConvexA, btConvexShape* pConvexB,
												const btTransform& transformA, const btTransform& transformB,
												btPoint3& wWitnessOnA, btPoint3& wWitnessOnB,
											    btScalar& penDepth, btVector3& v )
{
	btScalar squaredDistance = SIMD_INFINITY;
	btScalar delta = 0.f;

	const btScalar margin     = pConvexA->getMargin() + pConvexB->getMargin();
	const btScalar marginSqrd = margin * margin;

	simplexSolver.reset();

	int nbIterations = 0;

	while ( true )
	{
		assert( ( v.length2() > 0 ) && "Warning: v is the zero vector!" );

		btVector3 seperatingAxisInA = -v * transformA.getBasis();
		btVector3 seperatingAxisInB =  v * transformB.getBasis();

		btVector3 pInA = pConvexA->localGetSupportingVertexWithoutMargin( seperatingAxisInA );
		btVector3 qInB = pConvexB->localGetSupportingVertexWithoutMargin( seperatingAxisInB );

		btPoint3  pWorld = transformA( pInA );
		btPoint3  qWorld = transformB( qInB );

		btVector3 w = pWorld - qWorld;
		delta = v.dot( w );

		// potential exit, they don't overlap
		if ( ( delta > 0 ) && ( ( delta * delta / squaredDistance ) > marginSqrd ) )
		{
			// Convex shapes do not overlap
			// Returning true means that Hybrid's result is ok and there's no need to run EPA
			penDepth = 0;
			return true;
		}

		//exit 0: the new point is already in the simplex, or we didn't come any closer
		if ( ( squaredDistance - delta <= squaredDistance * g_GJKMaxRelErrorSqrd ) || simplexSolver.inSimplex( w ) )
		{
			simplexSolver.compute_points( wWitnessOnA, wWitnessOnB );

			assert( ( squaredDistance > 0 ) && "squaredDistance is zero!" );
			btScalar vLength = sqrt( squaredDistance );

			wWitnessOnA -= v * ( pConvexA->getMargin() / vLength );
			wWitnessOnB += v * ( pConvexB->getMargin() / vLength );

			penDepth = pConvexA->getMargin() + pConvexB->getMargin() - vLength;

			// Returning true means that Hybrid's result is ok and there's no need to run EPA
			return true;
		}

		//add current vertex to simplex
		simplexSolver.addVertex( w, pWorld, qWorld );

		//calculate the closest point to the origin (update vector v)
		if ( !simplexSolver.closest( v ) )
		{
			simplexSolver.compute_points( wWitnessOnA, wWitnessOnB );

			assert( ( squaredDistance > 0 ) && "squaredDistance is zero!" );
			btScalar vLength = sqrt( squaredDistance );

			wWitnessOnA -= v * ( pConvexA->getMargin() / vLength );
			wWitnessOnB += v * ( pConvexB->getMargin() / vLength );

			penDepth = pConvexA->getMargin() + pConvexB->getMargin() - vLength;

			// Returning true means that Hybrid's result is ok and there's no need to run EPA
			return true;
		}

		btScalar previousSquaredDistance = squaredDistance;
		squaredDistance = v.length2();

		//are we getting any closer ?
		if ( previousSquaredDistance - squaredDistance <= SIMD_EPSILON * previousSquaredDistance ) 
		{ 
			simplexSolver.backup_closest( v );
			squaredDistance = v.length2();

			simplexSolver.compute_points( wWitnessOnA, wWitnessOnB );

			assert( ( squaredDistance > 0 ) && "squaredDistance is zero!" );
			btScalar vLength = sqrt( squaredDistance );

			wWitnessOnA -= v * ( pConvexA->getMargin() / vLength );
			wWitnessOnB += v * ( pConvexB->getMargin() / vLength );

			penDepth = pConvexA->getMargin() + pConvexB->getMargin() - vLength;

			// Returning true means that Hybrid's result is ok and there's no need to run EPA
			return true;
		}

		if ( simplexSolver.fullSimplex() || ( squaredDistance <= SIMD_EPSILON * simplexSolver.maxVertex() ) )
		{
			// Convex Shapes intersect - we need to run EPA
			// Returning false means that Hybrid couldn't do anything for us
			// and that we need to run EPA to calculate the pen depth
			return false;
		}

		++nbIterations;
	}
}
#endif

btScalar EpaPenetrationDepthSolver::EpaPenDepth( btSimplexSolverInterface& simplexSolver,
												   btConvexShape* pConvexA, btConvexShape* pConvexB,
												   const btTransform& transformA, const btTransform& transformB,
												   btPoint3& wWitnessOnA, btPoint3& wWitnessOnB )
{
	Epa epa( pConvexA, pConvexB, transformA, transformB );

	if ( !epa.Initialize( simplexSolver ) )
	{
		EPA_DEBUG_ASSERT( false ,"Epa failed to initialize!" );
		return 0;
	}

	return epa.calcPenDepth( wWitnessOnA, wWitnessOnB );
}

