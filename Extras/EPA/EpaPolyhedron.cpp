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
#include "Memory2.h"

#include <list>
#ifdef _DEBUG
#include <hash_map>
#endif


#include "NarrowPhaseCollision/EpaCommon.h"

#include "NarrowPhaseCollision/EpaVertex.h"
#include "NarrowPhaseCollision/EpaHalfEdge.h"
#include "NarrowPhaseCollision/EpaFace.h"
#include "NarrowPhaseCollision/EpaPolyhedron.h"


EpaPolyhedron::EpaPolyhedron() : m_nbFaces( 0 )
{
}

EpaPolyhedron::~EpaPolyhedron()
{
	Destroy();
}

bool EpaPolyhedron::Create( SimdPoint3* pInitialPoints,
							SimdPoint3* pSupportPointsOnA, SimdPoint3* pSupportPointsOnB,
							const int nbInitialPoints )
{
#ifndef EPA_POLYHEDRON_USE_PLANES
	assert( ( nbInitialPoints <= 4 ) && "nbInitialPoints greater than 4!" );
#endif

	if ( nbInitialPoints < 4 )
	{
		// Insufficient nb of points
		return false;
	}

	////////////////////////////////////////////////////////////////////////////////

#ifdef EPA_POLYHEDRON_USE_PLANES
	int nbDiffCoords[ 3 ] = { 0, 0, 0 };

	bool* pDiffCoords = new bool[ 3 * nbInitialPoints ];
	

	int i;
	for (i=0;i<nbInitialPoints*3;i++)
	{
		pDiffCoords[i] = false;
	}

	//::memset( pDiffCoords, 0, sizeof( bool ) * 3 * nbInitialPoints );


	int axis;

	for ( axis = 0; axis < 3; ++axis )
	{
		for ( int i = 0; i < nbInitialPoints; ++i )
		{
			bool isDifferent = true;

			for ( int j = 0; j < i; ++j )
			{
				if ( pInitialPoints[ i ][ axis ] == pInitialPoints[ j ][ axis ] )
				{
					isDifferent = false;
					break;
				}
			}

			if ( isDifferent )
			{
				++nbDiffCoords[ axis ];
				pDiffCoords[ axis * nbInitialPoints + i ] = true;
			}
		}

		if ( nbDiffCoords[ axis ] <= 1 )
		{
			// The input is degenerate
			return false;
		}
	}

	int finalPointsIndices[ 4 ] = { -1, -1, -1, -1 };

	int axisOrderIndices[ 3 ] = { 0, 1, 2 };

	for ( i = 0; i < 2/*round( nbAxis / 2 )*/; ++i )
	{
		if ( nbDiffCoords[ i ] > nbDiffCoords[ i + 1 ] )
		{
			int tmp = nbDiffCoords[ i ];
			nbDiffCoords[ i ] = nbDiffCoords[ i + 1 ];
			nbDiffCoords[ i + 1 ] = tmp;

			tmp = axisOrderIndices[ i ];
			axisOrderIndices[ i ] = axisOrderIndices[ i + 1 ];
			axisOrderIndices[ i + 1 ] = tmp;
		}
	}

	int nbSuccessfullAxis = 0;

	// The axes with less different coordinates choose first
	int minsIndices[ 3 ] = { -1, -1, -1 };
	int maxsIndices[ 3 ] = { -1, -1, -1 };

	int finalPointsIndex = 0;

	for ( axis = 0; ( axis < 3 ) && ( nbSuccessfullAxis < 2 ); ++axis )
	{
		int axisIndex = axisOrderIndices[ axis ];
			
		SimdScalar axisMin =  SIMD_INFINITY;
		SimdScalar axisMax = -SIMD_INFINITY;

		for ( int i = 0; i < 4; ++i )
		{
			// Among the diff coords pick the min and max coords

			if ( pDiffCoords[ axisIndex * nbInitialPoints + i ] )
			{
				if ( pInitialPoints[ i ][ axisIndex ] < axisMin )
				{
					axisMin = pInitialPoints[ i ][ axisIndex ];
					minsIndices[ axisIndex ] = i;
				}

				if ( pInitialPoints[ i ][ axisIndex ] > axisMax )
				{
					axisMax = pInitialPoints[ i ][ axisIndex ];
					maxsIndices[ axisIndex ] = i;
				}
			}
		}

		//assert( ( minsIndices[ axisIndex ] != maxsIndices[ axisIndex ] ) &&
		//		"min and max have the same index!" );

		if ( ( minsIndices[ axisIndex ] != -1 ) && ( maxsIndices[ axisIndex ] != -1 ) &&
			 ( minsIndices[ axisIndex ] != maxsIndices[ axisIndex ] ) )
		{
			++nbSuccessfullAxis;

			finalPointsIndices[ finalPointsIndex++ ] = minsIndices[ axisIndex ];
			finalPointsIndices[ finalPointsIndex++ ] = maxsIndices[ axisIndex ];

			// Make the choosen points to be impossible for other axes to choose

			//assert( ( minsIndices[ axisIndex ] != -1 ) && "Invalid index!" );
			//assert( ( maxsIndices[ axisIndex ] != -1 ) && "Invalid index!" );

			for ( int i = 0; i < 3; ++i )
			{
				pDiffCoords[ i * nbInitialPoints + minsIndices[ axisIndex ] ] = false;
				pDiffCoords[ i * nbInitialPoints + maxsIndices[ axisIndex ] ] = false;
			}
		}
	}

	if ( nbSuccessfullAxis <= 1 )
	{
		// Degenerate input ?
		assert( false && "nbSuccessfullAxis must be greater than 1!" );
		return false;
	}
	
	delete[] pDiffCoords;
#endif

	//////////////////////////////////////////////////////////////////////////

#ifdef EPA_POLYHEDRON_USE_PLANES
	SimdVector3 v0 = pInitialPoints[ finalPointsIndices[ 1 ] ] - pInitialPoints[ finalPointsIndices[ 0 ] ];
	SimdVector3 v1 = pInitialPoints[ finalPointsIndices[ 2 ] ] - pInitialPoints[ finalPointsIndices[ 0 ] ];
#else
	SimdVector3 v0 = pInitialPoints[ 1 ] - pInitialPoints[ 0 ];
	SimdVector3 v1 = pInitialPoints[ 2 ] - pInitialPoints[ 0 ];
#endif

	SimdVector3 planeNormal = v1.cross( v0 );
	planeNormal.normalize();

#ifdef EPA_POLYHEDRON_USE_PLANES
	SimdScalar planeDistance = pInitialPoints[ finalPointsIndices[ 0 ] ].dot( -planeNormal );
#else
	SimdScalar planeDistance = pInitialPoints[ 0 ].dot( -planeNormal );
#endif

#ifdef EPA_POLYHEDRON_USE_PLANES
	assert( SimdEqual( pInitialPoints[ finalPointsIndices[ 0 ] ].dot( planeNormal ) + planeDistance, PLANE_THICKNESS ) &&
			"Point should be on plane!" );
	assert( SimdEqual( pInitialPoints[ finalPointsIndices[ 1 ] ].dot( planeNormal ) + planeDistance, PLANE_THICKNESS ) &&
			"Point should be on plane!" );
	assert( SimdEqual( pInitialPoints[ finalPointsIndices[ 2 ] ].dot( planeNormal ) + planeDistance, PLANE_THICKNESS ) &&
			"Point should be on plane!" );
#endif

#ifndef EPA_POLYHEDRON_USE_PLANES
	{
		if ( planeDistance > 0 )
		{
			SimdVector3 tmp = pInitialPoints[ 1 ];
			pInitialPoints[ 1 ] = pInitialPoints[ 2 ];
			pInitialPoints[ 2 ] = tmp;

			tmp = pSupportPointsOnA[ 1 ];
			pSupportPointsOnA[ 1 ] = pSupportPointsOnA[ 2 ];
			pSupportPointsOnA[ 2 ] = tmp;

			tmp = pSupportPointsOnB[ 1 ];
			pSupportPointsOnB[ 1 ] = pSupportPointsOnB[ 2 ];
			pSupportPointsOnB[ 2 ] = tmp;
		}
	}

	EpaVertex* pVertexA = CreateVertex( pInitialPoints[ 0 ], pSupportPointsOnA[ 0 ], pSupportPointsOnB[ 0 ] );
	EpaVertex* pVertexB = CreateVertex( pInitialPoints[ 1 ], pSupportPointsOnA[ 1 ], pSupportPointsOnB[ 1 ] );
	EpaVertex* pVertexC = CreateVertex( pInitialPoints[ 2 ], pSupportPointsOnA[ 2 ], pSupportPointsOnB[ 2 ] );
	EpaVertex* pVertexD = CreateVertex( pInitialPoints[ 3 ], pSupportPointsOnA[ 3 ], pSupportPointsOnB[ 3 ] );
#else
	finalPointsIndices[ 3 ] = -1;

	SimdScalar absMaxDist = -SIMD_INFINITY;
	SimdScalar maxDist;

	for ( int pointIndex = 0; pointIndex < nbInitialPoints; ++pointIndex )
	{
		SimdScalar dist    = planeNormal.dot( pInitialPoints[ pointIndex ] ) + planeDistance;
		SimdScalar absDist = abs( dist );

		if ( ( absDist > absMaxDist ) &&
			!SimdEqual( dist, PLANE_THICKNESS ) )
		{
			absMaxDist = absDist;
			maxDist    = dist;
			finalPointsIndices[ 3 ] = pointIndex;
		}
	}

	if ( finalPointsIndices[ 3 ] == -1 )
	{
		Destroy();
		return false;
	}

	if ( maxDist > PLANE_THICKNESS )
	{
		// Can swap indices only

		SimdPoint3 tmp = pInitialPoints[ finalPointsIndices[ 1 ] ];
		pInitialPoints[ finalPointsIndices[ 1 ] ] = pInitialPoints[ finalPointsIndices[ 2 ] ];
		pInitialPoints[ finalPointsIndices[ 2 ] ] = tmp;

		tmp = pSupportPointsOnA[ finalPointsIndices[ 1 ] ];
		pSupportPointsOnA[ finalPointsIndices[ 1 ] ] = pSupportPointsOnA[ finalPointsIndices[ 2 ] ];
		pSupportPointsOnA[ finalPointsIndices[ 2 ] ] = tmp;

		tmp = pSupportPointsOnB[ finalPointsIndices[ 1 ] ];
		pSupportPointsOnB[ finalPointsIndices[ 1 ] ] = pSupportPointsOnB[ finalPointsIndices[ 2 ] ];
		pSupportPointsOnB[ finalPointsIndices[ 2 ] ] = tmp;
	}

	EpaVertex* pVertexA = CreateVertex( pInitialPoints[ finalPointsIndices[ 0 ] ], pSupportPointsOnA[ finalPointsIndices[ 0 ] ], pSupportPointsOnB[ finalPointsIndices[ 0 ] ] );
	EpaVertex* pVertexB = CreateVertex( pInitialPoints[ finalPointsIndices[ 1 ] ], pSupportPointsOnA[ finalPointsIndices[ 1 ] ], pSupportPointsOnB[ finalPointsIndices[ 1 ] ] );
	EpaVertex* pVertexC = CreateVertex( pInitialPoints[ finalPointsIndices[ 2 ] ], pSupportPointsOnA[ finalPointsIndices[ 2 ] ], pSupportPointsOnB[ finalPointsIndices[ 2 ] ] );
	EpaVertex* pVertexD = CreateVertex( pInitialPoints[ finalPointsIndices[ 3 ] ], pSupportPointsOnA[ finalPointsIndices[ 3 ] ], pSupportPointsOnB[ finalPointsIndices[ 3 ] ] );
#endif

	EpaFace* pFaceA = CreateFace();
	EpaFace* pFaceB = CreateFace();
	EpaFace* pFaceC = CreateFace();
	EpaFace* pFaceD = CreateFace();

	EpaHalfEdge* pFaceAHalfEdges[ 3 ];
	EpaHalfEdge* pFaceCHalfEdges[ 3 ];
	EpaHalfEdge* pFaceBHalfEdges[ 3 ];
	EpaHalfEdge* pFaceDHalfEdges[ 3 ];

	pFaceAHalfEdges[ 0 ] = CreateHalfEdge();
	pFaceAHalfEdges[ 1 ] = CreateHalfEdge();
	pFaceAHalfEdges[ 2 ] = CreateHalfEdge();

	pFaceBHalfEdges[ 0 ] = CreateHalfEdge();
	pFaceBHalfEdges[ 1 ] = CreateHalfEdge();
	pFaceBHalfEdges[ 2 ] = CreateHalfEdge();

	pFaceCHalfEdges[ 0 ] = CreateHalfEdge();
	pFaceCHalfEdges[ 1 ] = CreateHalfEdge();
	pFaceCHalfEdges[ 2 ] = CreateHalfEdge();

	pFaceDHalfEdges[ 0 ] = CreateHalfEdge();
	pFaceDHalfEdges[ 1 ] = CreateHalfEdge();
	pFaceDHalfEdges[ 2 ] = CreateHalfEdge();

	pFaceA->m_pHalfEdge = pFaceAHalfEdges[ 0 ];
	pFaceB->m_pHalfEdge = pFaceBHalfEdges[ 0 ];
	pFaceC->m_pHalfEdge = pFaceCHalfEdges[ 0 ];
	pFaceD->m_pHalfEdge = pFaceDHalfEdges[ 0 ];

	pFaceAHalfEdges[ 0 ]->m_pNextCCW = pFaceAHalfEdges[ 1 ];
	pFaceAHalfEdges[ 1 ]->m_pNextCCW = pFaceAHalfEdges[ 2 ];
	pFaceAHalfEdges[ 2 ]->m_pNextCCW = pFaceAHalfEdges[ 0 ];

	pFaceBHalfEdges[ 0 ]->m_pNextCCW = pFaceBHalfEdges[ 1 ];
	pFaceBHalfEdges[ 1 ]->m_pNextCCW = pFaceBHalfEdges[ 2 ];
	pFaceBHalfEdges[ 2 ]->m_pNextCCW = pFaceBHalfEdges[ 0 ];

	pFaceCHalfEdges[ 0 ]->m_pNextCCW = pFaceCHalfEdges[ 1 ];
	pFaceCHalfEdges[ 1 ]->m_pNextCCW = pFaceCHalfEdges[ 2 ];
	pFaceCHalfEdges[ 2 ]->m_pNextCCW = pFaceCHalfEdges[ 0 ];

	pFaceDHalfEdges[ 0 ]->m_pNextCCW = pFaceDHalfEdges[ 1 ];
	pFaceDHalfEdges[ 1 ]->m_pNextCCW = pFaceDHalfEdges[ 2 ];
	pFaceDHalfEdges[ 2 ]->m_pNextCCW = pFaceDHalfEdges[ 0 ];


	pFaceAHalfEdges[ 0 ]->m_pFace = pFaceA;
	pFaceAHalfEdges[ 1 ]->m_pFace = pFaceA;
	pFaceAHalfEdges[ 2 ]->m_pFace = pFaceA;

	pFaceBHalfEdges[ 0 ]->m_pFace = pFaceB;
	pFaceBHalfEdges[ 1 ]->m_pFace = pFaceB;
	pFaceBHalfEdges[ 2 ]->m_pFace = pFaceB;

	pFaceCHalfEdges[ 0 ]->m_pFace = pFaceC;
	pFaceCHalfEdges[ 1 ]->m_pFace = pFaceC;
	pFaceCHalfEdges[ 2 ]->m_pFace = pFaceC;

	pFaceDHalfEdges[ 0 ]->m_pFace = pFaceD;
	pFaceDHalfEdges[ 1 ]->m_pFace = pFaceD;
	pFaceDHalfEdges[ 2 ]->m_pFace = pFaceD;


	pFaceAHalfEdges[ 0 ]->m_pVertex = pVertexA;
	pFaceAHalfEdges[ 1 ]->m_pVertex = pVertexB;
	pFaceAHalfEdges[ 2 ]->m_pVertex = pVertexC;

	pFaceBHalfEdges[ 0 ]->m_pVertex = pVertexB;
	pFaceBHalfEdges[ 1 ]->m_pVertex = pVertexD;
	pFaceBHalfEdges[ 2 ]->m_pVertex = pVertexC;

	pFaceCHalfEdges[ 0 ]->m_pVertex = pVertexD;
	pFaceCHalfEdges[ 1 ]->m_pVertex = pVertexA;
	pFaceCHalfEdges[ 2 ]->m_pVertex = pVertexC;

	pFaceDHalfEdges[ 0 ]->m_pVertex = pVertexB;
	pFaceDHalfEdges[ 1 ]->m_pVertex = pVertexA;
	pFaceDHalfEdges[ 2 ]->m_pVertex = pVertexD;

	//pVertexA->m_pHalfEdge = pFaceAHalfEdges[ 0 ];
	//pVertexB->m_pHalfEdge = pFaceAHalfEdges[ 1 ];
	//pVertexC->m_pHalfEdge = pFaceAHalfEdges[ 2 ];
	//pVertexD->m_pHalfEdge = pFaceBHalfEdges[ 1 ];

	pFaceAHalfEdges[ 0 ]->m_pTwin = pFaceDHalfEdges[ 0 ];
	pFaceAHalfEdges[ 1 ]->m_pTwin = pFaceBHalfEdges[ 2 ];
	pFaceAHalfEdges[ 2 ]->m_pTwin = pFaceCHalfEdges[ 1 ];

	pFaceBHalfEdges[ 0 ]->m_pTwin = pFaceDHalfEdges[ 2 ];
	pFaceBHalfEdges[ 1 ]->m_pTwin = pFaceCHalfEdges[ 2 ];
	pFaceBHalfEdges[ 2 ]->m_pTwin = pFaceAHalfEdges[ 1 ];

	pFaceCHalfEdges[ 0 ]->m_pTwin = pFaceDHalfEdges[ 1 ];
	pFaceCHalfEdges[ 1 ]->m_pTwin = pFaceAHalfEdges[ 2 ];
	pFaceCHalfEdges[ 2 ]->m_pTwin = pFaceBHalfEdges[ 1 ];

	pFaceDHalfEdges[ 0 ]->m_pTwin = pFaceAHalfEdges[ 0 ];
	pFaceDHalfEdges[ 1 ]->m_pTwin = pFaceCHalfEdges[ 0 ];
	pFaceDHalfEdges[ 2 ]->m_pTwin = pFaceBHalfEdges[ 0 ];

	if ( !pFaceA->Initialize() || !pFaceB->Initialize() ||
		 !pFaceC->Initialize() || !pFaceD->Initialize() )
	{
		assert( false && "One initial face failed to initialize!" );
		return false;
	}

#ifdef EPA_POLYHEDRON_USE_PLANES
	if ( nbInitialPoints > 4 )
	{
		for ( int i = 0; i < nbInitialPoints; ++i )
		{
			if ( ( i != finalPointsIndices[ 0 ] ) && ( i != finalPointsIndices[ 1 ] ) &&
				 ( i != finalPointsIndices[ 2 ] ) && ( i != finalPointsIndices[ 3 ] ) )
			{
				std::list< EpaFace* >::iterator facesItr( m_faces.begin() );

				while ( facesItr != m_faces.end() )
				{
					EpaFace* pFace = *facesItr;

					SimdScalar dist = pFace->m_planeNormal.dot( pInitialPoints[ i ] ) + pFace->m_planeDistance;

					if ( dist > PLANE_THICKNESS )
					{
						std::list< EpaFace* > newFaces;

						bool expandOk = Expand( pInitialPoints[ i ], pSupportPointsOnA[ i ], pSupportPointsOnB[ i ],
												pFace, newFaces );

						if ( !expandOk )
						{
							// One or more new faces are affinely dependent
							return false;
						}

						assert( !newFaces.empty() && "Polyhedron should have expanded!" );
						
						break;
					}

					++facesItr;
				}
			}
		}
	}
#endif

	return true;
}

void EpaPolyhedron::Destroy()
{
	DestroyAllVertices();

	DestroyAllHalfEdges();

	DestroyAllFaces();
}

EpaFace* EpaPolyhedron::CreateFace()
{
	EpaFace* pNewFace = new EpaFace();
	assert( pNewFace && "Failed to allocate memory for a new EpaFace!" );

	m_faces.push_back( pNewFace );

	++m_nbFaces;

	return pNewFace;
}

EpaHalfEdge* EpaPolyhedron::CreateHalfEdge()
{
	EpaHalfEdge* pNewHalfEdge = new EpaHalfEdge();
	assert( pNewHalfEdge && "Failed to allocate memory for a new EpaHalfEdge!" );

	m_halfEdges.push_back( pNewHalfEdge );

	return pNewHalfEdge;
}

EpaVertex* EpaPolyhedron::CreateVertex( const SimdPoint3& wSupportPoint,
									    const SimdPoint3& wSupportPointOnA,
									    const SimdPoint3& wSupportPointOnB )
{
	EpaVertex* pNewVertex = new EpaVertex( wSupportPoint, wSupportPointOnA, wSupportPointOnB );
	assert( pNewVertex && "Failed to allocate memory for a new EpaVertex!" );

	m_vertices.push_back( pNewVertex );

	return pNewVertex;
}

void EpaPolyhedron::DeleteFace( EpaFace* pFace )
{
	pFace->m_deleted = true;
	--m_nbFaces;
}

void EpaPolyhedron::DestroyAllFaces()
{
	while ( !m_faces.empty() )
	{
		EpaFace* pFace = m_faces.front();

		delete pFace;

		m_faces.pop_front();
	}

	m_nbFaces = 0;
}

void EpaPolyhedron::DestroyAllHalfEdges()
{
	while ( !m_halfEdges.empty() )
	{
		EpaHalfEdge* pHalfEdge = m_halfEdges.front();

		delete pHalfEdge;

		m_halfEdges.pop_front();
	}
}

void EpaPolyhedron::DestroyAllVertices()
{
	while ( !m_vertices.empty() )
	{
		EpaVertex* pVertex = m_vertices.front();

		delete pVertex;

		m_vertices.pop_front();
	}
}

bool EpaPolyhedron::Expand( const SimdPoint3& wSupportPoint,
						    const SimdPoint3& wSupportPointOnA,
						    const SimdPoint3& wSupportPointOnB,
						    EpaFace* pFace, std::list< EpaFace* >& newFaces )
{
	assert( !pFace->m_deleted && "Face is already deleted!" );
	assert( newFaces.empty() && "NewFaces list must be empty!" );

	assert( !pFace->m_deleted && "Cannot expand deleted face!" );

	// wSupportPoint must be front of face's plane used to do the expansion

#ifdef EPA_POLYHEDRON_USE_PLANES
	SimdScalar dist = pFace->m_planeNormal.dot( wSupportPoint ) + pFace->m_planeDistance;
	if ( dist <= PLANE_THICKNESS )
	{
		return false;
	}
#endif

	std::list< EpaHalfEdge* > coneBaseEdges;
	DeleteVisibleFaces( wSupportPoint, pFace, coneBaseEdges );

	assert( ( coneBaseEdges.size() >= 3 ) && "Cone base must have at least 3 edges!" );

	EpaVertex* pConeAppex = CreateVertex( wSupportPoint, wSupportPointOnA, wSupportPointOnB );
	assert( pConeAppex && "Failed to create vertex!" );

	CreateCone( pConeAppex, coneBaseEdges, newFaces );

	// Initialize new faces

	std::list< EpaFace* >::iterator newFacesItr( newFaces.begin() );

	while ( newFacesItr != newFaces.end() )
	{
		EpaFace* pNewFace = *newFacesItr;

		if ( !pNewFace->Initialize() )
		{
			return false;
		}

		++newFacesItr;
	}

	return true;
}

std::list< EpaFace* >& EpaPolyhedron::GetFaces()
{
	return m_faces;
}

int EpaPolyhedron::GetNbFaces() const
{
	return m_faces.size();
}

void EpaPolyhedron::DeleteVisibleFaces( const SimdPoint3& point, EpaFace* pFace,
										std::list< EpaHalfEdge* >& coneBaseTwinHalfEdges )
{
	assert( !pFace->m_deleted && "Face is already deleted!" );

	DeleteFace( pFace );

	EpaHalfEdge* pCurrentHalfEdge = pFace->m_pHalfEdge;

	do
	{
		assert( pCurrentHalfEdge->m_pTwin && "Half-edge without a twin!" );

		EpaFace* pAdjacentFace = pCurrentHalfEdge->m_pTwin->m_pFace;
		assert( pAdjacentFace && "Invalid adjacent face!" );

		if ( !pAdjacentFace->m_deleted )
		{
#ifdef EPA_POLYHEDRON_USE_PLANES
			assert( ( pAdjacentFace->m_planeNormal.length2() > 0 ) && "Invalid plane!" );

			SimdScalar pointDist = pAdjacentFace->m_planeNormal.dot( point ) +
								   pAdjacentFace->m_planeDistance;

			if ( pointDist > PLANE_THICKNESS )
#else
			SimdScalar dot = pAdjacentFace->m_v.dot( point );
			if ( dot >= pAdjacentFace->m_vSqrd )
#endif
			{
				DeleteVisibleFaces( point, pAdjacentFace, coneBaseTwinHalfEdges );
			}
			else
			{
				coneBaseTwinHalfEdges.push_back( pCurrentHalfEdge->m_pTwin );
			}
		}

		pCurrentHalfEdge = pCurrentHalfEdge->m_pNextCCW;
	}
	while( pCurrentHalfEdge != pFace->m_pHalfEdge );
}

void EpaPolyhedron::CreateCone( EpaVertex* pAppexVertex, std::list< EpaHalfEdge* >& baseTwinHalfEdges,
								std::list< EpaFace* >& newFaces )
{
	assert( ( baseTwinHalfEdges.size() >= 3 ) && "DeleteVisibleFaces method didn't do its job right!" );

	std::list< EpaHalfEdge* >::iterator baseHalfEdgesItr( baseTwinHalfEdges.begin() );
	std::list< EpaHalfEdge* > halfEdgesToLink;

	while ( baseHalfEdgesItr != baseTwinHalfEdges.end() )
	{
		EpaFace* pNewFace = CreateConeFace( pAppexVertex, *baseHalfEdgesItr, halfEdgesToLink );

		newFaces.push_back( pNewFace );
		
		++baseHalfEdgesItr;
	}

	// Connect consecutive faces by linking twin half-edges

	assert( ( halfEdgesToLink.size() % 2 == 0 ) && "Nb half-edges to link is odd!" );

	int nbLinksToCreate = halfEdgesToLink.size() / 2;
	int nbLinksCreated  = 0;

	std::list< EpaHalfEdge* >::iterator halfEdgesItr( halfEdgesToLink.begin() );

	for ( ; ( halfEdgesItr != halfEdgesToLink.end() ) && ( nbLinksCreated < nbLinksToCreate ); ++halfEdgesItr )
	{
		std::list< EpaHalfEdge* >::iterator halfEdgesItr2( halfEdgesItr );
		++halfEdgesItr2;

		for ( ; ( halfEdgesItr2 != halfEdgesToLink.end() ) && ( nbLinksCreated < nbLinksToCreate ); ++halfEdgesItr2 )
		{
			EpaHalfEdge* pHalfEdge1 = *halfEdgesItr;
			EpaHalfEdge* pHalfEdge2 = *halfEdgesItr2;

			EpaHalfEdge* pHalfEdgeNextCCW1 = pHalfEdge1->m_pNextCCW;
			EpaHalfEdge* pHalfEdgeNextCCW2 = pHalfEdge2->m_pNextCCW;

			if ( ( pHalfEdge2->m_pVertex == pHalfEdgeNextCCW1->m_pVertex ) &&
				 ( pHalfEdgeNextCCW2->m_pVertex == pHalfEdge1->m_pVertex ) )
			{
				pHalfEdge1->m_pTwin = pHalfEdge2;
				pHalfEdge2->m_pTwin = pHalfEdge1;

				++nbLinksCreated;
			}
		}
	}

	assert( ( nbLinksCreated == nbLinksToCreate ) && "Mesh topology not ok!" );
}

EpaFace* EpaPolyhedron::CreateConeFace( EpaVertex* pAppexVertex, EpaHalfEdge* pBaseTwinHalfEdge,
									    std::list< EpaHalfEdge* >& halfEdgesToLink )
{
	EpaFace* pNewFace = CreateFace();

	EpaHalfEdge* pNewHalfEdge0 = CreateHalfEdge();
	EpaHalfEdge* pNewHalfEdge1 = CreateHalfEdge();
	EpaHalfEdge* pNewHalfEdge2 = CreateHalfEdge();

	// Let new face point to one of its new half-edges
	pNewFace->m_pHalfEdge = pNewHalfEdge0;

	// Link new half-edges in a loop

	pNewHalfEdge0->m_pNextCCW = pNewHalfEdge1;
	pNewHalfEdge1->m_pNextCCW = pNewHalfEdge2;
	pNewHalfEdge2->m_pNextCCW = pNewHalfEdge0;

	// Let new half-edges point to new face

	pNewHalfEdge0->m_pFace = pNewFace;
	pNewHalfEdge1->m_pFace = pNewFace;
	pNewHalfEdge2->m_pFace = pNewFace;
	
	// Let new half-edge 0 and base twin half-edge point to each other

	pNewHalfEdge0->m_pTwin	   = pBaseTwinHalfEdge;
	pBaseTwinHalfEdge->m_pTwin = pNewHalfEdge0;

	// Let new half-edges know about their origin vertex

	pNewHalfEdge0->m_pVertex = pBaseTwinHalfEdge->m_pNextCCW->m_pVertex;
	pNewHalfEdge1->m_pVertex = pBaseTwinHalfEdge->m_pVertex;
	pNewHalfEdge2->m_pVertex = pAppexVertex;

	// Let vertices know about one of their outgoing edges

	//pNewHalfEdge0->m_pVertex->m_pHalfEdge = pNewHalfEdge0;
	//pNewHalfEdge1->m_pVertex->m_pHalfEdge = pNewHalfEdge1;
	//pNewHalfEdge2->m_pVertex->m_pHalfEdge = pNewHalfEdge2;

	halfEdgesToLink.push_back( pNewHalfEdge1 );
	halfEdgesToLink.push_back( pNewHalfEdge2 );

	return pNewFace;
}


#ifdef DO_SOME_DEBUGGING_
#ifdef _DEBUG
bool EpaPolyhedron::_dbgSaveToFile( const char* pFileName )
{
	FILE* fp = NULL;

	if ( fopen_s( &fp, pFileName, "wb" ) != 0 )
	{
		return false;
	}

	unsigned long int nbBytesWritten;
	unsigned long int byteIndex = 0;

	unsigned long int fileID = 0xBADC0DE;
	fwrite( &fileID, sizeof( fileID ), 1, fp );
	nbBytesWritten = sizeof( fileID );
	byteIndex += nbBytesWritten;

	unsigned char reservedByte = 0;
	fwrite( &reservedByte, sizeof( reservedByte ), 1, fp );
	nbBytesWritten = sizeof( reservedByte );
	byteIndex += nbBytesWritten;
	fwrite( &reservedByte, sizeof( reservedByte ), 1, fp );
	nbBytesWritten = sizeof( reservedByte );
	byteIndex += nbBytesWritten;
	fwrite( &reservedByte, sizeof( reservedByte ), 1, fp );
	nbBytesWritten = sizeof( reservedByte );
	byteIndex += nbBytesWritten;

	fwrite( &reservedByte, sizeof( reservedByte ), 1, fp );
	nbBytesWritten = sizeof( reservedByte );
	byteIndex += nbBytesWritten;
	fwrite( &reservedByte, sizeof( reservedByte ), 1, fp );
	nbBytesWritten = sizeof( reservedByte );
	byteIndex += nbBytesWritten;
	fwrite( &reservedByte, sizeof( reservedByte ), 1, fp );
	nbBytesWritten = sizeof( reservedByte );
	byteIndex += nbBytesWritten;

	unsigned char stringSize = 5;
	fwrite( &stringSize, sizeof( stringSize ), 1, fp );
	nbBytesWritten = sizeof( stringSize );
	byteIndex += nbBytesWritten;

	char exportedFrom[ 6 ] = "01234";
	fwrite( exportedFrom, stringSize * sizeof( char ), 1, fp );
	nbBytesWritten = stringSize * sizeof( char );
	byteIndex += nbBytesWritten;

	unsigned short int w = 0;

	fwrite( &w, sizeof( w ), 1, fp );
	nbBytesWritten = sizeof( w );
	byteIndex += nbBytesWritten;
	fwrite( &w, sizeof( w ), 1, fp );
	nbBytesWritten = sizeof( w );
	byteIndex += nbBytesWritten;
	fwrite( &w, sizeof( w ), 1, fp );
	nbBytesWritten = sizeof( w );
	byteIndex += nbBytesWritten;

	fwrite( &w, sizeof( w ), 1, fp );
	nbBytesWritten = sizeof( w );
	byteIndex += nbBytesWritten;
	fwrite( &w, sizeof( w ), 1, fp );
	nbBytesWritten = sizeof( w );
	byteIndex += nbBytesWritten;
	fwrite( &w, sizeof( w ), 1, fp );
	nbBytesWritten = sizeof( w );
	byteIndex += nbBytesWritten;

	unsigned long int geometryOffsetAtByteNb = byteIndex;

	unsigned long int geometryOffset = 0;
	unsigned long int geometrySize   = 0;

	fseek( fp, sizeof( geometryOffset ) + sizeof( geometrySize ), SEEK_CUR );
	byteIndex += sizeof( geometryOffset ) + sizeof( geometrySize );

	unsigned long int mappingOffset = 0;
	unsigned long int mappingSize	= 0;

	fwrite( &mappingOffset, sizeof( mappingOffset ), 1, fp );
	nbBytesWritten = sizeof( mappingOffset );
	byteIndex += nbBytesWritten;

	fwrite( &mappingSize, sizeof( mappingSize ), 1, fp );
	nbBytesWritten = sizeof( mappingSize );
	byteIndex += nbBytesWritten;

	unsigned long int animationOffset = 0;
	unsigned long int animationSize   = 0;

	fwrite( &animationOffset, sizeof( animationOffset ), 1, fp );
	nbBytesWritten = sizeof( animationOffset );
	byteIndex += nbBytesWritten;
	fwrite( &animationSize, sizeof( animationSize ), 1, fp );
	nbBytesWritten = sizeof( animationSize );
	byteIndex += nbBytesWritten;

	unsigned long int reservedDword = 0;
	fwrite( &reservedDword, sizeof( reservedDword ), 1, fp );
	nbBytesWritten = sizeof( reservedDword );
	byteIndex += nbBytesWritten;
	fwrite( &reservedDword, sizeof( reservedDword ), 1, fp );
	nbBytesWritten = sizeof( reservedDword );
	byteIndex += nbBytesWritten;

	geometryOffset = byteIndex;

	unsigned short int nbMeshs = 1;
	fwrite( &nbMeshs, sizeof( nbMeshs ), 1, fp );
	nbBytesWritten = sizeof( nbMeshs );
	byteIndex += nbBytesWritten;

	char meshName[] = "noname mesh";
	unsigned char meshNameSize = strlen( meshName );

	fwrite( &meshNameSize, sizeof( meshNameSize ), 1, fp );
	nbBytesWritten = sizeof( meshNameSize );
	byteIndex += nbBytesWritten;

	fwrite( meshName, meshNameSize * sizeof( char ), 1, fp );
	nbBytesWritten = meshNameSize * sizeof( char );
	byteIndex += nbBytesWritten;

	stdext::hash_map< unsigned long int, int > verticesMap;
	typedef std::pair< unsigned long int, int > PR;

	int vertexIndex = 0;

	// Hash only vertices from faces that are not deleted

	std::list< EpaFace* >::iterator facesItr( m_faces.begin() );
	int nbFaces = 0;

	while ( facesItr != m_faces.end() )
	{
		EpaFace* pFace = *facesItr;

		if ( !pFace->m_deleted )
		{
			stdext::hash_map< unsigned long int, int >::const_iterator vertexItr;

			vertexItr = verticesMap.find( ( unsigned long int ) pFace->m_pVertices[ 0 ] );
			if ( vertexItr == verticesMap.end() )
			{
				verticesMap.insert( PR( ( unsigned long int ) pFace->m_pVertices[ 0 ], vertexIndex ) );
				++vertexIndex;
			}

			vertexItr = verticesMap.find( ( unsigned long int ) pFace->m_pVertices[ 1 ] );
			if ( vertexItr == verticesMap.end() )
			{
				verticesMap.insert( PR( ( unsigned long int ) pFace->m_pVertices[ 1 ], vertexIndex ) );
				++vertexIndex;
			}

			vertexItr = verticesMap.find( ( unsigned long int ) pFace->m_pVertices[ 2 ] );
			if ( vertexItr == verticesMap.end() )
			{
				verticesMap.insert( PR( ( unsigned long int ) pFace->m_pVertices[ 2 ], vertexIndex ) );
				++vertexIndex;
			}

			++nbFaces;
		}

		++facesItr;
	}

	unsigned long int nbVertices = verticesMap.size();
	fwrite( &nbVertices, sizeof( nbVertices ), 1, fp );
	nbBytesWritten = sizeof( nbVertices );
	byteIndex += nbBytesWritten;

	// Collect all safe vertices

	float* pVertices = new float[ verticesMap.size() * 3 ];

	stdext::hash_map< unsigned long int, int >::iterator verticesItr( verticesMap.begin() );

	while ( verticesItr != verticesMap.end() )
	{
		stdext::hash_map< unsigned long int, int >::const_iterator vertexItr;

		PR pr = *verticesItr;

		vertexItr = verticesMap.find( pr.first );
		assert( ( vertexItr != verticesMap.end() ) && "Vertex not found in hash table!" );

		EpaVertex* pVertex = ( EpaVertex* ) vertexItr->first;

		pVertices[ vertexItr->second * 3	 ] = pVertex->m_point.x();
		pVertices[ vertexItr->second * 3 + 1 ] = pVertex->m_point.y();
		pVertices[ vertexItr->second * 3 + 2 ] = pVertex->m_point.z();

		++verticesItr;
	}

	unsigned long int* pIndices = new unsigned long int[ nbFaces * 3 ];

	facesItr = m_faces.begin();

	int facesIndex = 0;
	while ( facesItr != m_faces.end() )
	{
		EpaFace* pFace = *facesItr;

		if ( !pFace->m_deleted )
		{
			stdext::hash_map< unsigned long int, int >::const_iterator vertexItr;
	
			int verticesIndices[ 3 ];

			vertexItr = verticesMap.find( ( unsigned long int ) pFace->m_pVertices[ 0 ] );
			assert( ( vertexItr != verticesMap.end() ) && "Vertex not found in hash table!" );
			verticesIndices[ 0 ] = vertexItr->second;

			vertexItr = verticesMap.find( ( unsigned long int ) pFace->m_pVertices[ 1 ] );
			assert( ( vertexItr != verticesMap.end() ) && "Vertex not found in hash table!" );
			verticesIndices[ 1 ] = vertexItr->second;

			vertexItr = verticesMap.find( ( unsigned long int ) pFace->m_pVertices[ 2 ] );
			assert( ( vertexItr != verticesMap.end() ) && "Vertex not found in hash table!" );
			verticesIndices[ 2 ] = vertexItr->second;

			pIndices[ facesIndex * 3	 ] = verticesIndices[ 0 ];
			pIndices[ facesIndex * 3 + 1 ] = verticesIndices[ 1 ];
			pIndices[ facesIndex * 3 + 2 ] = verticesIndices[ 2 ];

			++facesIndex;
		}

		++facesItr;
	}

	fwrite( &nbFaces, sizeof( nbFaces ), 1, fp );
	nbBytesWritten = sizeof( nbFaces );
	byteIndex += nbBytesWritten;

	bool hasSmoothingGroups = false;
	fwrite( &hasSmoothingGroups, sizeof( hasSmoothingGroups ), 1, fp );
	nbBytesWritten = sizeof( hasSmoothingGroups );
	byteIndex += nbBytesWritten;

	fwrite( pVertices, verticesMap.size() * 3 * sizeof( float ), 1, fp );
	nbBytesWritten = verticesMap.size() * 3 * sizeof( float );
	byteIndex += nbBytesWritten;

	// write indices
	fwrite( pIndices, nbFaces * 3 * sizeof( unsigned long int ), 1, fp );
	nbBytesWritten = nbFaces * 3 * sizeof( unsigned long int );
	byteIndex += nbBytesWritten;

	delete[] pVertices;
	delete[] pIndices;

	geometrySize = byteIndex - geometryOffset;

	fseek( fp, geometryOffsetAtByteNb, SEEK_SET );

	fwrite( &geometryOffset, sizeof( geometryOffset ), 1, fp );
	nbBytesWritten = sizeof( geometryOffset );
	byteIndex += nbBytesWritten;
	fwrite( &geometrySize, sizeof( geometrySize ), 1, fp );
	nbBytesWritten = sizeof( geometrySize );
	byteIndex += nbBytesWritten;

	fseek( fp, byteIndex, SEEK_SET );

	fclose( fp );

	return true;
}
#endif
#endif

