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

#include "NarrowPhaseCollision/EpaCommon.h"

#include "NarrowPhaseCollision/EpaVertex.h"
#include "NarrowPhaseCollision/EpaHalfEdge.h"
#include "NarrowPhaseCollision/EpaFace.h"

#ifdef EPA_POLYHEDRON_USE_PLANES
btScalar PLANE_THICKNESS = 1e-5f;
#endif

EpaFace::EpaFace() : m_pHalfEdge( 0 ), m_deleted( false )
{
	m_pVertices[ 0 ] = m_pVertices[ 1 ] = m_pVertices[ 2 ] = 0;
}

EpaFace::~EpaFace()
{
}

bool EpaFace::Initialize()
{
	assert( m_pHalfEdge && "Must setup half-edge first!" );

	CollectVertices( m_pVertices );
	
	const btVector3 e0 = m_pVertices[ 1 ]->m_point - m_pVertices[ 0 ]->m_point;
	const btVector3 e1 = m_pVertices[ 2 ]->m_point - m_pVertices[ 0 ]->m_point;

	const btScalar e0Sqrd = e0.length2();
	const btScalar e1Sqrd = e1.length2();
	const btScalar e0e1   = e0.dot( e1 );

	m_determinant = e0Sqrd * e1Sqrd - e0e1 * e0e1;

	const btScalar e0v0 = e0.dot( m_pVertices[ 0 ]->m_point );
	const btScalar e1v0 = e1.dot( m_pVertices[ 0 ]->m_point );

	m_lambdas[ 0 ] = e0e1 * e1v0 - e1Sqrd * e0v0;
	m_lambdas[ 1 ] = e0e1 * e0v0 - e0Sqrd * e1v0;

	if ( IsAffinelyDependent() )
	{
		return false;
	}

	CalcClosestPoint();

#ifdef EPA_POLYHEDRON_USE_PLANES
	if ( !CalculatePlane() )
	{
		return false;
	}
#endif

	return true;
}

#ifdef EPA_POLYHEDRON_USE_PLANES
bool EpaFace::CalculatePlane()
{
	assert( ( m_pVertices[ 0 ] && m_pVertices[ 1 ] && m_pVertices[ 2 ] )
			&& "Must setup vertices pointers first!" );

	// Traditional method

	const btVector3 v1 = m_pVertices[ 1 ]->m_point - m_pVertices[ 0 ]->m_point;
	const btVector3 v2 = m_pVertices[ 2 ]->m_point - m_pVertices[ 0 ]->m_point;

	m_planeNormal = v2.cross( v1 );

	if ( m_planeNormal.length2() == 0 )
	{
		return false;
	}

	m_planeNormal.normalize();

	m_planeDistance = m_pVertices[ 0 ]->m_point.dot( -m_planeNormal );

	// Robust method

	//btVector3 _v1 = m_pVertices[ 1 ]->m_point - m_pVertices[ 0 ]->m_point;
	//btVector3 _v2 = m_pVertices[ 2 ]->m_point - m_pVertices[ 0 ]->m_point;

	//btVector3 n;

	//n = _v2.cross( _v1 );

	//_v1 = m_pVertices[ 0 ]->m_point - m_pVertices[ 1 ]->m_point;
	//_v2 = m_pVertices[ 2 ]->m_point - m_pVertices[ 1 ]->m_point;

	//n += ( _v1.cross( _v2 ) );

	//_v1 = m_pVertices[ 0 ]->m_point - m_pVertices[ 2 ]->m_point;
	//_v2 = m_pVertices[ 1 ]->m_point - m_pVertices[ 2 ]->m_point;

	//n += ( _v2.cross( _v1 ) );

	//n /= 3;	
	//n.normalize();

	//btVector3 c = ( m_pVertices[ 0 ]->m_point + m_pVertices[ 1 ]->m_point + m_pVertices[ 2 ]->m_point ) / 3;
	//btScalar d  = c.dot( -n );

	//m_robustPlaneNormal   = n;
	//m_robustPlaneDistance = d;

	// Compare results from both methods and check whether they disagree

	//if ( d < 0 )
	//{
	//	assert( ( m_planeDistance < 0 ) && "He he! Busted!" );
	//}
	//else
	//{
	//	assert( ( m_planeDistance >= 0 ) && "He he! Busted!" );
	//}

	return true;
}
#endif

void EpaFace::CalcClosestPoint()
{
	const btVector3 e0 = m_pVertices[ 1 ]->m_point - m_pVertices[ 0 ]->m_point;
	const btVector3 e1 = m_pVertices[ 2 ]->m_point - m_pVertices[ 0 ]->m_point;

	m_v =  m_pVertices[ 0 ]->m_point +
		 ( e0 * m_lambdas[ 0 ] + e1 * m_lambdas[ 1 ] ) / m_determinant;

	m_vSqrd = m_v.length2();
}

void EpaFace::CalcClosestPointOnA( btVector3& closestPointOnA )
{
	const btVector3 e0 = m_pVertices[ 1 ]->m_wSupportPointOnA - m_pVertices[ 0 ]->m_wSupportPointOnA;
	const btVector3 e1 = m_pVertices[ 2 ]->m_wSupportPointOnA - m_pVertices[ 0 ]->m_wSupportPointOnA;

	closestPointOnA =  m_pVertices[ 0 ]->m_wSupportPointOnA +
					 ( e0 * m_lambdas[ 0 ] + e1 * m_lambdas[ 1 ] ) /
					   m_determinant;
}

void EpaFace::CalcClosestPointOnB( btVector3& closestPointOnB )
{
	const btVector3 e0 = m_pVertices[ 1 ]->m_wSupportPointOnB - m_pVertices[ 0 ]->m_wSupportPointOnB;
	const btVector3 e1 = m_pVertices[ 2 ]->m_wSupportPointOnB - m_pVertices[ 0 ]->m_wSupportPointOnB;

	closestPointOnB =  m_pVertices[ 0 ]->m_wSupportPointOnB +
					 ( e0 * m_lambdas[  0 ] + e1 * m_lambdas[ 1 ] ) /
					   m_determinant;
}

bool EpaFace::IsAffinelyDependent() const
{
	return ( m_determinant <= SIMD_EPSILON );
}

bool EpaFace::IsClosestPointInternal() const
{
	return ( ( m_lambdas[ 0 ] >= 0 ) && ( m_lambdas[ 1 ] >= 0 ) && ( ( m_lambdas[ 0 ] + m_lambdas[ 1 ] <= m_determinant ) ) );
}

void EpaFace::CollectVertices( EpaVertex** ppVertices )
{
	assert( m_pHalfEdge && "Invalid half-edge pointer!" );

	int vertexIndex = 0;

	EpaHalfEdge* pCurrentHalfEdge = m_pHalfEdge;

	do
	{
		assert( ( ( vertexIndex >= 0 ) && ( vertexIndex < 3 ) ) &&
				"Face is not a triangle!" );

		assert( pCurrentHalfEdge->m_pVertex && "Half-edge has an invalid vertex pointer!" );

		ppVertices[ vertexIndex++ ] = pCurrentHalfEdge->m_pVertex;

		pCurrentHalfEdge = pCurrentHalfEdge->m_pNextCCW;
				
	}
	while( pCurrentHalfEdge != m_pHalfEdge );
}

//void EpaFace::FixOrder()
//{
//	EpaHalfEdge* pHalfEdges[ 3 ];
//
//	int halfEdgeIndex = 0;
//
//	EpaHalfEdge* pCurrentHalfEdge = m_pHalfEdge;
//
//	do
//	{
//		assert( ( ( halfEdgeIndex >= 0 ) && ( halfEdgeIndex < 3 ) ) &&
//				"Face is not a triangle!" );
//
//		pHalfEdges[ halfEdgeIndex++ ] = pCurrentHalfEdge;
//
//		pCurrentHalfEdge = pCurrentHalfEdge->m_pNextCCW;
//	}
//	while( pCurrentHalfEdge != m_pHalfEdge );
//
//	EpaVertex* pVertices[ 3 ] = { pHalfEdges[ 0 ]->m_pVertex,
//								  pHalfEdges[ 1 ]->m_pVertex,
//								  pHalfEdges[ 2 ]->m_pVertex };
//	
//	// Make them run in the opposite direction
//	pHalfEdges[ 0 ]->m_pNextCCW = pHalfEdges[ 2 ];
//	pHalfEdges[ 1 ]->m_pNextCCW = pHalfEdges[ 0 ];
//	pHalfEdges[ 2 ]->m_pNextCCW = pHalfEdges[ 1 ];
//
//	// Make half-edges point to their correct origin vertices
//
//	pHalfEdges[ 1 ]->m_pVertex = pVertices[ 2 ];
//	pHalfEdges[ 2 ]->m_pVertex = pVertices[ 0 ];
//	pHalfEdges[ 0 ]->m_pVertex = pVertices[ 1 ];
//
//	// Make vertices point to the correct half-edges
//
//	//pHalfEdges[ 0 ]->m_pVertex->m_pHalfEdge = pHalfEdges[ 0 ];
//	//pHalfEdges[ 1 ]->m_pVertex->m_pHalfEdge = pHalfEdges[ 1 ];
//	//pHalfEdges[ 2 ]->m_pVertex->m_pHalfEdge = pHalfEdges[ 2 ];
//
//	// Flip normal and change the sign of plane distance
//
//#ifdef EPA_POLYHEDRON_USE_PLANES
//	m_planeNormal	= -m_planeNormal;
//	m_planeDistance = -m_planeDistance;
//#endif
//}

