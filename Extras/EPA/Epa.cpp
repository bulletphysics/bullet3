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

#include "LinearMath/SimdScalar.h"
#include "LinearMath/SimdVector3.h"
#include "LinearMath/SimdPoint3.h"
#include "LinearMath/SimdTransform.h"
#include "LinearMath/SimdMinMax.h"

#include "BulletCollision/CollisionShapes/btConvexShape.h"

#include <vector>
#include <list>
#include <algorithm>

#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"

#include "NarrowPhaseCollision/EpaCommon.h"

#include "NarrowPhaseCollision/EpaVertex.h"
#include "NarrowPhaseCollision/EpaHalfEdge.h"
#include "NarrowPhaseCollision/EpaFace.h"
#include "NarrowPhaseCollision/EpaPolyhedron.h"
#include "NarrowPhaseCollision/Epa.h"

const SimdScalar EPA_MAX_RELATIVE_ERROR = 1e-2f;
const SimdScalar EPA_MAX_RELATIVE_ERROR_SQRD = EPA_MAX_RELATIVE_ERROR * EPA_MAX_RELATIVE_ERROR;

Epa::Epa( ConvexShape* pConvexShapeA, ConvexShape* pConvexShapeB,
		  const SimdTransform& transformA, const SimdTransform& transformB ) : m_pConvexShapeA( pConvexShapeA ),
																			   m_pConvexShapeB( pConvexShapeB ),
																			   m_transformA( transformA ),
																			   m_transformB( transformB )
{
	m_faceEntries.reserve( EPA_MAX_FACE_ENTRIES );
}

Epa::~Epa()
{
}

bool Epa::Initialize( SimplexSolverInterface& simplexSolver )
{
	// Run GJK on the enlarged shapes to obtain a simplex of the enlarged CSO

	SimdVector3 v( 1, 0, 0 );
	SimdScalar squaredDistance = SIMD_INFINITY;

	SimdScalar delta = 0.f;

	simplexSolver.reset();

	int nbIterations = 0;

	while ( true )
	{
		EPA_DEBUG_ASSERT( ( v.length2() > 0 ) ,"Warning : v has zero magnitude!" );

		SimdVector3 seperatingAxisInA = -v * m_transformA.getBasis();
		SimdVector3 seperatingAxisInB =  v * m_transformB.getBasis();

		SimdVector3 pInA = m_pConvexShapeA->LocalGetSupportingVertex( seperatingAxisInA );
		SimdVector3 qInB = m_pConvexShapeB->LocalGetSupportingVertex( seperatingAxisInB );

		SimdPoint3  pWorld = m_transformA( pInA );
		SimdPoint3  qWorld = m_transformB( qInB );

		SimdVector3 w = pWorld - qWorld;
		delta = v.dot( w );

		EPA_DEBUG_ASSERT( ( delta <= 0 ) ,"Shapes are disjoint, EPA should have never    been called!" );
		if ( delta > 0.f )
			return false;

		EPA_DEBUG_ASSERT( !simplexSolver.inSimplex( w ) ,"Shapes are disjoint, EPA should have never been called!" );
		if (simplexSolver.inSimplex( w ))
			return false;

		// Add support point to simplex
		simplexSolver.addVertex( w, pWorld, qWorld );

		bool closestOk = simplexSolver.closest( v );
		EPA_DEBUG_ASSERT( closestOk ,"Shapes are disjoint, EPA should have never been called!" );
		if (!closestOk)
			return false;
		
		SimdScalar prevVSqrd = squaredDistance;
		squaredDistance = v.length2();

		// Is v converging to v(A-B) ?
		EPA_DEBUG_ASSERT( ( ( prevVSqrd - squaredDistance ) > SIMD_EPSILON * prevVSqrd ) ,
				"Shapes are disjoint, EPA should have never been called!" );
		if (( ( prevVSqrd - squaredDistance ) <= SIMD_EPSILON * prevVSqrd ))
			return false;

		if ( simplexSolver.fullSimplex() || ( squaredDistance <= SIMD_EPSILON * simplexSolver.maxVertex() ) )
		{
			break;
		}

		++nbIterations;
	}

	SimdPoint3 simplexPoints[ 5 ];
	SimdPoint3 wSupportPointsOnA[ 5 ];
	SimdPoint3 wSupportPointsOnB[ 5 ];

	int nbSimplexPoints = simplexSolver.getSimplex( wSupportPointsOnA, wSupportPointsOnB, simplexPoints );

	// nbSimplexPoints can't be one because cases where the origin is on the boundary are handled
	// by hybrid penetration depth
	EPA_DEBUG_ASSERT( ( ( nbSimplexPoints > 1 ) ,( nbSimplexPoints <= 4 ) ) ,
		    "Hybrid Penetration Depth algorithm failed!" );

	int nbPolyhedronPoints = nbSimplexPoints;

#ifndef EPA_POLYHEDRON_USE_PLANES
	int initTetraIndices[ 4 ] = { 0, 1, 2, 3 };
#endif

	//	Prepare initial polyhedron to start EPA from
	if ( nbSimplexPoints == 1 )
	{
		return false;
	}
	else if ( nbSimplexPoints == 2 )
	{
		// We have a line segment inside the CSO that contains the origin
		// Create an hexahedron ( two tetrahedron glued together ) by adding 3 new points

		SimdVector3 d = simplexPoints[ 0 ] - simplexPoints[ 1 ];
		d.normalize();

		SimdVector3 v1;
		SimdVector3 v2;
		SimdVector3 v3;

		SimdVector3 e1;

		SimdScalar absx = abs( d.getX() );
		SimdScalar absy = abs( d.getY() );
		SimdScalar absz = abs( d.getZ() );

		if ( absx < absy )
		{
			if ( absx < absz )
			{
				e1.setX( 1 );
			}
			else
			{
				e1.setZ( 1 );
			}
		}
		else
		{
			if ( absy < absz )
			{
				e1.setY( 1 );
			}
			else
			{
				e1.setZ( 1 );
			}
		}

		v1 = d.cross( e1 );
		v1.normalize();

		v2 = v1.rotate( d, 120 * SIMD_RADS_PER_DEG );
		v3 = v2.rotate( d, 120 * SIMD_RADS_PER_DEG );

		nbPolyhedronPoints = 5;

		SimdVector3 seperatingAxisInA =  v1 * m_transformA.getBasis();
		SimdVector3 seperatingAxisInB = -v1 * m_transformB.getBasis();

		SimdVector3 p = m_pConvexShapeA->LocalGetSupportingVertex( seperatingAxisInA );
		SimdVector3 q = m_pConvexShapeB->LocalGetSupportingVertex( seperatingAxisInB );

		SimdPoint3 pWorld = m_transformA( p );
		SimdPoint3 qWorld = m_transformB( q );

		wSupportPointsOnA[ 2 ] = pWorld;
		wSupportPointsOnB[ 2 ] = qWorld;
		simplexPoints[ 2 ]	   = wSupportPointsOnA[ 2 ] - wSupportPointsOnB[ 2 ];

		seperatingAxisInA =  v2 * m_transformA.getBasis();
		seperatingAxisInB = -v2 * m_transformB.getBasis();

		p = m_pConvexShapeA->LocalGetSupportingVertex( seperatingAxisInA );
		q = m_pConvexShapeB->LocalGetSupportingVertex( seperatingAxisInB );

		pWorld = m_transformA( p );
		qWorld = m_transformB( q );

		wSupportPointsOnA[ 3 ] = pWorld;
		wSupportPointsOnB[ 3 ] = qWorld;
		simplexPoints[ 3 ]	   = wSupportPointsOnA[ 3 ] - wSupportPointsOnB[ 3 ];

		seperatingAxisInA =  v3 * m_transformA.getBasis();
		seperatingAxisInB = -v3 * m_transformB.getBasis();

		p = m_pConvexShapeA->LocalGetSupportingVertex( seperatingAxisInA );
		q = m_pConvexShapeB->LocalGetSupportingVertex( seperatingAxisInB );

		pWorld = m_transformA( p );
		qWorld = m_transformB( q );

		wSupportPointsOnA[ 4 ] = pWorld;
		wSupportPointsOnB[ 4 ] = qWorld;
		simplexPoints[ 4 ]	   = wSupportPointsOnA[ 4 ] - wSupportPointsOnB[ 4 ];

#ifndef EPA_POLYHEDRON_USE_PLANES
		if ( TetrahedronContainsOrigin( simplexPoints[ 0 ], simplexPoints[ 2 ], simplexPoints[ 3 ], simplexPoints[ 4 ] ) )
		{
			initTetraIndices[ 1 ] = 2;
			initTetraIndices[ 2 ] = 3;
			initTetraIndices[ 3 ] = 4;
		}
		else
		{
			if ( TetrahedronContainsOrigin( simplexPoints[ 1 ], simplexPoints[ 2 ], simplexPoints[ 3 ], simplexPoints[ 4 ] ) )
			{
				initTetraIndices[ 0 ] = 1;
				initTetraIndices[ 1 ] = 2;
				initTetraIndices[ 2 ] = 3;
				initTetraIndices[ 3 ] = 4;
			}
			else
			{
				// No tetrahedron contains the origin
				assert( false && "Unable to find an initial tetrahedron that contains the origin!" );
				return false;
			}
		}
#endif
	}
	else if ( nbSimplexPoints == 3 )
	{
		// We have a triangle inside the CSO that contains the origin
		// Create an hexahedron ( two tetrahedron glued together ) by adding 2 new points

		SimdVector3 v0 = simplexPoints[ 2 ] - simplexPoints[ 0 ];
		SimdVector3 v1 = simplexPoints[ 1 ] - simplexPoints[ 0 ];
		SimdVector3 triangleNormal = v0.cross( v1 );
		triangleNormal.normalize();

		nbPolyhedronPoints = 5;

		SimdVector3 seperatingAxisInA =  triangleNormal * m_transformA.getBasis();
		SimdVector3 seperatingAxisInB = -triangleNormal * m_transformB.getBasis();

		SimdVector3 p = m_pConvexShapeA->LocalGetSupportingVertex( seperatingAxisInA );
		SimdVector3 q = m_pConvexShapeB->LocalGetSupportingVertex( seperatingAxisInB );

		SimdPoint3 pWorld = m_transformA( p );
		SimdPoint3 qWorld = m_transformB( q );

		wSupportPointsOnA[ 3 ] = pWorld;
		wSupportPointsOnB[ 3 ] = qWorld;
		simplexPoints[ 3 ]	   = wSupportPointsOnA[ 3 ] - wSupportPointsOnB[ 3 ];

#ifndef EPA_POLYHEDRON_USE_PLANES
		// We place this check here because if the tetrahedron contains the origin
		// there is no need to sample another support point
		if ( !TetrahedronContainsOrigin( simplexPoints[ 0 ], simplexPoints[ 1 ], simplexPoints[ 2 ], simplexPoints[ 3 ] ) )
		{
#endif
			seperatingAxisInA = -triangleNormal * m_transformA.getBasis();
			seperatingAxisInB =  triangleNormal * m_transformB.getBasis();

			p = m_pConvexShapeA->LocalGetSupportingVertex( seperatingAxisInA );
			q = m_pConvexShapeB->LocalGetSupportingVertex( seperatingAxisInB );

			pWorld = m_transformA( p );
			qWorld = m_transformB( q );

			wSupportPointsOnA[ 4 ] = pWorld;
			wSupportPointsOnB[ 4 ] = qWorld;
			simplexPoints[ 4 ]	   = wSupportPointsOnA[ 4 ] - wSupportPointsOnB[ 4 ];

#ifndef EPA_POLYHEDRON_USE_PLANES
			if ( TetrahedronContainsOrigin( simplexPoints[ 0 ], simplexPoints[ 1 ], simplexPoints[ 2 ], simplexPoints[ 4 ] ) )
			{
				initTetraIndices[ 3 ] = 4;
			}
			else
			{
				// No tetrahedron contains the origin
				assert( false && "Unable to find an initial tetrahedron that contains the origin!" );
				return false;
			}
		}
#endif
	}
#ifdef _DEBUG
	else if ( nbSimplexPoints == 4 )
	{
		EPA_DEBUG_ASSERT( TetrahedronContainsOrigin( simplexPoints ) ,"Initial tetrahedron does not contain the origin!" );
	}
#endif

#ifndef EPA_POLYHEDRON_USE_PLANES
	SimdPoint3 wTetraPoints[ 4 ] = { simplexPoints[ initTetraIndices[ 0 ] ],
									 simplexPoints[ initTetraIndices[ 1 ] ],
									 simplexPoints[ initTetraIndices[ 2 ] ],
									 simplexPoints[ initTetraIndices[ 3 ] ] };

	SimdPoint3 wTetraSupportPointsOnA[ 4 ] = { wSupportPointsOnA[ initTetraIndices[ 0 ] ],
											   wSupportPointsOnA[ initTetraIndices[ 1 ] ],
											   wSupportPointsOnA[ initTetraIndices[ 2 ] ],
											   wSupportPointsOnA[ initTetraIndices[ 3 ] ] };

	SimdPoint3 wTetraSupportPointsOnB[ 4 ] = { wSupportPointsOnB[ initTetraIndices[ 0 ] ],
											   wSupportPointsOnB[ initTetraIndices[ 1 ] ],
											   wSupportPointsOnB[ initTetraIndices[ 2 ] ],
											   wSupportPointsOnB[ initTetraIndices[ 3 ] ] };
#endif

#ifdef EPA_POLYHEDRON_USE_PLANES
	if ( !m_polyhedron.Create( simplexPoints, wSupportPointsOnA, wSupportPointsOnB, nbPolyhedronPoints ) )
#else
	if ( !m_polyhedron.Create( wTetraPoints, wTetraSupportPointsOnA, wTetraSupportPointsOnB, 4 ) )
#endif
	{
		// Failed to create initial polyhedron
		EPA_DEBUG_ASSERT( false ,"Failed to create initial polyhedron!" );
		return false;
	}

	// Add initial faces to priority queue

#ifdef _DEBUG
	//m_polyhedron._dbgSaveToFile( "epa_start.dbg" );
#endif

	std::list< EpaFace* >& faces = m_polyhedron.GetFaces();

	std::list< EpaFace* >::iterator facesItr( faces.begin() );

	while ( facesItr != faces.end() )
	{
		EpaFace* pFace = *facesItr;

		if ( !pFace->m_deleted )
		{
//#ifdef EPA_POLYHEDRON_USE_PLANES
//			if ( pFace->m_planeDistance >= 0 )
//			{
//				m_polyhedron._dbgSaveToFile( "epa_start.dbg" );
//				assert( false && "Face's plane distance equal or greater than 0!" );
//			}
//#endif

			if ( pFace->IsAffinelyDependent() )
			{
				EPA_DEBUG_ASSERT( false ,"One initial face is affinely dependent!" );
				return false;
			}

			if ( pFace->m_vSqrd <= 0 )
			{
				EPA_DEBUG_ASSERT( false ,"Face containing the origin!" );
				return false;
			}

			if ( pFace->IsClosestPointInternal() )
			{
				m_faceEntries.push_back( pFace );
				std::push_heap( m_faceEntries.begin(), m_faceEntries.end(), CompareEpaFaceEntries );
			}
		}

		++facesItr;
	}

#ifdef _DEBUG
	//m_polyhedron._dbgSaveToFile( "epa_start.dbg" );
#endif

	EPA_DEBUG_ASSERT( !m_faceEntries.empty() ,"No faces added to heap!" );

	return true;
}

SimdScalar Epa::CalcPenDepth( SimdPoint3& wWitnessOnA, SimdPoint3& wWitnessOnB )
{
	SimdVector3 v;

	SimdScalar upperBoundSqrd = SIMD_INFINITY;
	SimdScalar vSqrd = 0;
#ifdef _DEBUG
	SimdScalar prevVSqrd;
#endif
	SimdScalar delta;

	bool isCloseEnough = false;

	EpaFace* pEpaFace = NULL;

	int nbIterations = 0;
	//int nbMaxIterations = 1000;

	do
	{
		pEpaFace = m_faceEntries.front();
		std::pop_heap( m_faceEntries.begin(), m_faceEntries.end(), CompareEpaFaceEntries );
		m_faceEntries.pop_back();

		if ( !pEpaFace->m_deleted )
		{
#ifdef _DEBUG
			prevVSqrd = vSqrd;
#endif

			vSqrd = pEpaFace->m_vSqrd;

			if ( pEpaFace->m_planeDistance >= 0 )
			{
				v = pEpaFace->m_planeNormal;
			}
			else
			{
				v = pEpaFace->m_v;
			}

#ifdef _DEBUG
			//assert_msg( vSqrd <= upperBoundSqrd, "A triangle was falsely rejected!" );
			EPA_DEBUG_ASSERT( ( vSqrd >= prevVSqrd ) ,"vSqrd decreased!" );
#endif //_DEBUG
			EPA_DEBUG_ASSERT( ( v.length2() > 0 ) ,"Zero vector not allowed!" );

			SimdVector3 seperatingAxisInA =  v * m_transformA.getBasis();
			SimdVector3 seperatingAxisInB = -v * m_transformB.getBasis();

			SimdVector3 p = m_pConvexShapeA->LocalGetSupportingVertex( seperatingAxisInA );
			SimdVector3 q = m_pConvexShapeB->LocalGetSupportingVertex( seperatingAxisInB );

			SimdPoint3 pWorld = m_transformA( p );
			SimdPoint3 qWorld = m_transformB( q );

			SimdPoint3 w = pWorld - qWorld;
			delta = v.dot( w );

			// Keep tighest upper bound
			upperBoundSqrd = SimdMin( upperBoundSqrd, delta * delta / vSqrd );
			//assert_msg( vSqrd <= upperBoundSqrd, "A triangle was falsely rejected!" );

			isCloseEnough = ( upperBoundSqrd <= ( 1 + 1e-4f ) * vSqrd );

			if ( !isCloseEnough )
			{
				std::list< EpaFace* > newFaces;
				bool expandOk = m_polyhedron.Expand( w, pWorld, qWorld, pEpaFace, newFaces );

				if ( expandOk )
				{
					EPA_DEBUG_ASSERT( !newFaces.empty() ,"EPA polyhedron not expanding ?" );

					bool check    = true;
					bool areEqual = false;

					while ( !newFaces.empty() )
					{
						EpaFace* pNewFace = newFaces.front();
						EPA_DEBUG_ASSERT( !pNewFace->m_deleted ,"New face is deleted!" );

						if ( !pNewFace->m_deleted )
						{
							EPA_DEBUG_ASSERT( ( pNewFace->m_vSqrd > 0 ) ,"Face containing the origin!" );
							EPA_DEBUG_ASSERT( !pNewFace->IsAffinelyDependent() ,"Face is affinely dependent!" );

//#ifdef EPA_POLYHEDRON_USE_PLANES
////							if ( pNewFace->m_planeDistance >= 0 )
////							{
//								// assert( false && "Face's plane distance greater than 0!" );
//#ifdef _DEBUG
////								m_polyhedron._dbgSaveToFile( "epa_beforeFix.dbg" );
//#endif
//								//pNewFace->FixOrder();
//#ifdef _DEBUG
//								//m_polyhedron._dbgSaveToFile( "epa_afterFix.dbg" );
//#endif
////							}
//#endif
//
//#ifdef EPA_POLYHEDRON_USE_PLANES
//							//assert( ( pNewFace->m_planeDistance < 0 ) && "Face's plane distance equal or greater than 0!" );
//#endif

							if ( pNewFace->IsClosestPointInternal() && ( vSqrd <= pNewFace->m_vSqrd ) && ( pNewFace->m_vSqrd <= upperBoundSqrd ) )
							{
								m_faceEntries.push_back( pNewFace );
								std::push_heap( m_faceEntries.begin(), m_faceEntries.end(), CompareEpaFaceEntries );
							}
						}

						newFaces.pop_front();
					}
				}
				else
				{
					pEpaFace->CalcClosestPointOnA( wWitnessOnA );
					pEpaFace->CalcClosestPointOnB( wWitnessOnB );

#ifdef _DEBUG
					//m_polyhedron._dbgSaveToFile( "epa_end.dbg" );
#endif

					return v.length();
				}
			}
		}

		++nbIterations;
	}
	while ( ( m_polyhedron.GetNbFaces() < EPA_MAX_FACE_ENTRIES ) &&/*( nbIterations < nbMaxIterations ) &&*/
			!isCloseEnough && ( m_faceEntries.size() > 0 ) && ( m_faceEntries[ 0 ]->m_vSqrd <= upperBoundSqrd ) );

#ifdef _DEBUG
	//m_polyhedron._dbgSaveToFile( "epa_end.dbg" );
#endif

	EPA_DEBUG_ASSERT( pEpaFace ,"Invalid epa face!" );

	pEpaFace->CalcClosestPointOnA( wWitnessOnA );
	pEpaFace->CalcClosestPointOnB( wWitnessOnB );

	return v.length();
}

bool Epa::TetrahedronContainsOrigin( const SimdPoint3& point0, const SimdPoint3& point1,
									 const SimdPoint3& point2, const SimdPoint3& point3 )
{
	SimdVector3 facesNormals[ 4 ] = { ( point1 - point0 ).cross( point2 - point0 ),
									  ( point2 - point1 ).cross( point3 - point1 ),
									  ( point3 - point2 ).cross( point0 - point2 ),
									  ( point0 - point3 ).cross( point1 - point3 ) };

	return ( ( facesNormals[ 0 ].dot( point0 ) > 0 ) != ( facesNormals[ 0 ].dot( point3 ) > 0 ) ) &&
		   ( ( facesNormals[ 1 ].dot( point1 ) > 0 ) != ( facesNormals[ 1 ].dot( point0 ) > 0 ) ) &&
		   ( ( facesNormals[ 2 ].dot( point2 ) > 0 ) != ( facesNormals[ 2 ].dot( point1 ) > 0 ) ) &&
		   ( ( facesNormals[ 3 ].dot( point3 ) > 0 ) != ( facesNormals[ 3 ].dot( point2 ) > 0 ) );
}

bool Epa::TetrahedronContainsOrigin( SimdPoint3* pPoints )
{
	return TetrahedronContainsOrigin( pPoints[ 0 ], pPoints[ 1 ], pPoints[ 2 ], pPoints[ 3 ] );
}

bool CompareEpaFaceEntries( EpaFace* pFaceA, EpaFace* pFaceB )
{
	return ( pFaceA->m_vSqrd > pFaceB->m_vSqrd );
}
