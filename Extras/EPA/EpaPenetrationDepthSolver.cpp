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
#include "SimdScalar.h"
#include "SimdVector3.h"
#include "SimdPoint3.h"
#include "SimdTransform.h"
#include "SimdMinMax.h"

#include <list>

#include "CollisionShapes/ConvexShape.h"

#include "NarrowPhaseCollision/SimplexSolverInterface.h"

#include "NarrowPhaseCollision/EpaCommon.h"

#include "NarrowPhaseCollision/EpaVertex.h"
#include "NarrowPhaseCollision/EpaHalfEdge.h"
#include "NarrowPhaseCollision/EpaFace.h"
#include "NarrowPhaseCollision/EpaPolyhedron.h"
#include "NarrowPhaseCollision/Epa.h"
#include "NarrowPhaseCollision/ConvexPenetrationDepthSolver.h"
#include "NarrowPhaseCollision/EpaPenetrationDepthSolver.h"

SimdScalar	g_GJKMaxRelError = 1e-3f;
SimdScalar	g_GJKMaxRelErrorSqrd = g_GJKMaxRelError * g_GJKMaxRelError;

bool EpaPenetrationDepthSolver::CalcPenDepth( SimplexSolverInterface& simplexSolver,
											  ConvexShape* pConvexA, ConvexShape* pConvexB,
											  const SimdTransform& transformA, const SimdTransform& transformB,
											  SimdVector3& v, SimdPoint3& wWitnessOnA, SimdPoint3& wWitnessOnB,
											  class IDebugDraw* debugDraw )
{
	assert( pConvexA && "Convex shape A is invalid!" );
	assert( pConvexB && "Convex shape B is invalid!" );

	SimdScalar penDepth;

#ifdef EPA_USE_HYBRID
	bool needsEPA = !HybridPenDepth( simplexSolver, pConvexA, pConvexB, transformA, transformB,
									 wWitnessOnA, wWitnessOnB, penDepth, v );

	if ( needsEPA )
	{
#endif
		penDepth = EpaPenDepth( simplexSolver, pConvexA, pConvexB,
								transformA, transformB,
								wWitnessOnA, wWitnessOnB );
		assert( ( penDepth > 0 ) &&  "EPA or Hybrid Technique failed to calculate penetration depth!" );

#ifdef EPA_USE_HYBRID
	}
#endif

	return ( penDepth > 0 );
}

#ifdef EPA_USE_HYBRID
bool EpaPenetrationDepthSolver::HybridPenDepth( SimplexSolverInterface& simplexSolver,
												ConvexShape* pConvexA, ConvexShape* pConvexB,
												const SimdTransform& transformA, const SimdTransform& transformB,
												SimdPoint3& wWitnessOnA, SimdPoint3& wWitnessOnB,
											    SimdScalar& penDepth, SimdVector3& v )
{
	SimdScalar squaredDistance = SIMD_INFINITY;
	SimdScalar delta = 0.f;

	const SimdScalar margin     = pConvexA->GetMargin() + pConvexB->GetMargin();
	const SimdScalar marginSqrd = margin * margin;

	simplexSolver.reset();

	int nbIterations = 0;

	while ( true )
	{
		assert( ( v.length2() > 0 ) && "Warning: v is the zero vector!" );

		SimdVector3 seperatingAxisInA = -v * transformA.getBasis();
		SimdVector3 seperatingAxisInB =  v * transformB.getBasis();

		SimdVector3 pInA = pConvexA->LocalGetSupportingVertexWithoutMargin( seperatingAxisInA );
		SimdVector3 qInB = pConvexB->LocalGetSupportingVertexWithoutMargin( seperatingAxisInB );

		SimdPoint3  pWorld = transformA( pInA );
		SimdPoint3  qWorld = transformB( qInB );

		SimdVector3 w = pWorld - qWorld;
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
			SimdScalar vLength = sqrt( squaredDistance );

			wWitnessOnA -= v * ( pConvexA->GetMargin() / vLength );
			wWitnessOnB += v * ( pConvexB->GetMargin() / vLength );

			penDepth = pConvexA->GetMargin() + pConvexB->GetMargin() - vLength;

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
			SimdScalar vLength = sqrt( squaredDistance );

			wWitnessOnA -= v * ( pConvexA->GetMargin() / vLength );
			wWitnessOnB += v * ( pConvexB->GetMargin() / vLength );

			penDepth = pConvexA->GetMargin() + pConvexB->GetMargin() - vLength;

			// Returning true means that Hybrid's result is ok and there's no need to run EPA
			return true;
		}

		SimdScalar previousSquaredDistance = squaredDistance;
		squaredDistance = v.length2();

		//are we getting any closer ?
		if ( previousSquaredDistance - squaredDistance <= SIMD_EPSILON * previousSquaredDistance ) 
		{ 
			simplexSolver.backup_closest( v );
			squaredDistance = v.length2();

			simplexSolver.compute_points( wWitnessOnA, wWitnessOnB );

			assert( ( squaredDistance > 0 ) && "squaredDistance is zero!" );
			SimdScalar vLength = sqrt( squaredDistance );

			wWitnessOnA -= v * ( pConvexA->GetMargin() / vLength );
			wWitnessOnB += v * ( pConvexB->GetMargin() / vLength );

			penDepth = pConvexA->GetMargin() + pConvexB->GetMargin() - vLength;

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

SimdScalar EpaPenetrationDepthSolver::EpaPenDepth( SimplexSolverInterface& simplexSolver,
												   ConvexShape* pConvexA, ConvexShape* pConvexB,
												   const SimdTransform& transformA, const SimdTransform& transformB,
												   SimdPoint3& wWitnessOnA, SimdPoint3& wWitnessOnB )
{
	Epa epa( pConvexA, pConvexB, transformA, transformB );

	if ( !epa.Initialize( simplexSolver ) )
	{
		assert( false && "Epa failed to initialize!" );
		return 0;
	}

	return epa.CalcPenDepth( wWitnessOnA, wWitnessOnB );
}

