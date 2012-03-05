/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "../../../include/physics_effects/base_level/collision/pfx_box.h"
#include "../../../include/physics_effects/base_level/collision/pfx_capsule.h"
#include "pfx_contact_box_capsule.h"

namespace sce {
namespace PhysicsEffects {

enum BoxCapsSepAxisType
{
	BOX_AXIS, CROSS_AXIS
};

//-------------------------------------------------------------------------------------------------
// voronoiTol: bevels Voronoi planes slightly which helps when features are parallel.
//-------------------------------------------------------------------------------------------------

static const PfxFloat voronoiTol = -1.0e-5f;

//-------------------------------------------------------------------------------------------------
// lenSqrTol: minimum square of length for safe normalize.
//-------------------------------------------------------------------------------------------------

static const PfxFloat lenSqrTol = 1.0e-30f;

//-------------------------------------------------------------------------------------------------
// separating axis tests: gaps along each axis are computed, and the axis with the maximum
// gap is stored.  cross product axes are normalized.
//-------------------------------------------------------------------------------------------------

#define AaxisTest( dim, letter, first )                                                         \
{                                                                                               \
   if ( first )                                                                                 \
   {                                                                                            \
      maxGap = gapsA.get##letter();                                                             \
      if ( maxGap - capsuleB.m_radius > distanceThreshold ) return maxGap - capsuleB.m_radius;      \
      axisType = BOX_AXIS;                                                                      \
      faceDimA = dim;                                                                           \
      axisA = ident[dim];                                                          \
   }                                                                                            \
   else                                                                                         \
   {                                                                                            \
      PfxFloat gap = gapsA.get##letter();                                                          \
      if ( gap - capsuleB.m_radius > distanceThreshold ) return gap - capsuleB.m_radius;            \
      else if ( gap > maxGap )                                                                  \
      {                                                                                         \
         maxGap = gap;                                                                          \
         axisType = BOX_AXIS;                                                                   \
         faceDimA = dim;                                                                        \
         axisA = ident[dim];                                                       \
      }                                                                                         \
   }                                                                                            \
}

#define CrossAxisTest( dima, lettera )                                                          \
{                                                                                               \
   const PfxFloat lsqr_tolerance = 1.0e-30f;                                                       \
   PfxFloat lsqr;                                                                                  \
                                                                                                \
   lsqr = lsqrs.get##lettera();                                                                 \
                                                                                                \
   if ( lsqr > lsqr_tolerance )                                                                 \
   {                                                                                            \
      PfxFloat l_recip = 1.0f / sqrtf( lsqr );                                                     \
      PfxFloat gap = PfxFloat(gapsAxB.get##lettera()) * l_recip;                                      \
                                                                                                \
      if ( gap - capsuleB.m_radius > distanceThreshold )                                          \
      {                                                                                         \
         return gap - capsuleB.m_radius;                                                          \
      }                                                                                         \
                                                                                                \
      if ( gap > maxGap )                                                                       \
      {                                                                                         \
         maxGap = gap;                                                                          \
         axisType = CROSS_AXIS;                                                                 \
         edgeDimA = dima;                                                                       \
         axisA = crossProdMat.getCol##dima() * l_recip;                                         \
      }                                                                                         \
   }                                                                                            \
}

//-------------------------------------------------------------------------------------------------
// tests whether a vertex of box B and a face of box A are the closest features
//-------------------------------------------------------------------------------------------------

inline
PfxFloat
VertexBFaceATest(
	PfxBool& inVoronoi,
	PfxFloat& t0,
	PfxFloat& t1,
	PfxVector3& ptsVec,
	const PfxVector3& hA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG offsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG capsDirection,
	PfxFloat signB,
	PfxFloat scaleB )
{
	// compute endpoint of capsule in box's coordinate system

	PfxVector3 endpoint = PfxVector3( offsetAB + capsDirection * scaleB );

	// compute the parameters of the point on the box face closest to this corner.

	t0 = endpoint[0];
	t1 = endpoint[1];

	if ( t0 > hA[0] )
		t0 = hA[0];
	else if ( t0 < -hA[0] )
		t0 = -hA[0];
	if ( t1 > hA[1] )
		t1 = hA[1];
	else if ( t1 < -hA[1] )
		t1 = -hA[1];

	// get vector from face point to capsule endpoint

	endpoint[0] -= t0;
	endpoint[1] -= t1;
	ptsVec = PfxVector3(endpoint);

	// do the Voronoi test: already know the point on B is in the Voronoi region of the
	// point on A, check the reverse.

	inVoronoi = ( -signB * dot(ptsVec,capsDirection) >= voronoiTol );

	return (lengthSqr(ptsVec));
}

#define VertexBFaceA_SetNewMin()                \
{                                               \
   minDistSqr = distSqr;                        \
   closestPtsVec = ptsVec;                      \
   localPointA.setX(t0);                        \
   localPointA.setY(t1);                        \
   segmentParamB = scaleB;                      \
}

void
VertexBFaceATests(
	PfxBool& done,
	PfxFloat& minDistSqr,
	PfxVector3& closestPtsVec,
	PfxPoint3& localPointA,
	PfxFloat& segmentParamB,
	const PfxVector3& hA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG offsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG capsDirection,
	PfxFloat signB, PfxFloat scaleB,
	PfxBool first )
{
	PfxVector3 ptsVec;
	PfxFloat t0, t1;
	PfxFloat distSqr;

	// test endpoint of capsule nearest to face

	distSqr = VertexBFaceATest( done, t0, t1, ptsVec, hA, offsetAB, capsDirection, signB, scaleB );

	if ( first ) {
		VertexBFaceA_SetNewMin();
	} else {
		if ( distSqr < minDistSqr ) {
			VertexBFaceA_SetNewMin();
		}
	}

	if ( done )
		return;

	signB = -signB;
	scaleB = -scaleB;

	// test other endpoint if necessary

	distSqr = VertexBFaceATest( done, t0, t1, ptsVec, hA, offsetAB, capsDirection, signB, scaleB );

	if ( distSqr < minDistSqr ) {
		VertexBFaceA_SetNewMin();
	}
}

//-------------------------------------------------------------------------------------------------
// EdgeEdgeTest:
//
// tests whether a pair of edges are the closest features
//
// note on the shorthand:
// 'a' & 'b' refer to the edges.
// 'c' is the dimension of the axis that points from the face center to the edge Center
// 'd' is the dimension of the edge Direction
// the dimension of the face normal is 2
//-------------------------------------------------------------------------------------------------

#define EdgeEdgeTest( ac, ac_letter, ad, ad_letter )                                            \
{                                                                                               \
   /* get vector between edge centers */                                                        \
                                                                                                \
   ptsVec = offsetAB;                                                                           \
   ptsVec.set##ac_letter( ptsVec.get##ac_letter() - scalesA.get##ac_letter() );                 \
                                                                                                \
   /* find parameters of closest points on line segments. */                                    \
                                                                                                \
   PfxFloat capsDirection_ad = capsDirection.get##ad_letter();                                     \
   PfxFloat ptsVec_ad = ptsVec.get##ad_letter();                                                   \
   PfxFloat capsDirDotPtsVec = dot(capsDirection,ptsVec);                                          \
   PfxFloat denom = 1.0f - capsDirection_ad * capsDirection_ad;                                    \
                                                                                                \
   if ( denom == 0.0f )                                                                         \
   {                                                                                            \
      tA = 0.0f;                                                                                \
   }                                                                                            \
   else                                                                                         \
   {                                                                                            \
      tA = ( ptsVec_ad - capsDirDotPtsVec * capsDirection_ad ) / denom;                         \
      if ( tA < -hA[ad] ) tA = -hA[ad];                                                         \
      else if ( tA > hA[ad] ) tA = hA[ad];                                                      \
   }                                                                                            \
                                                                                                \
   tB = tA * capsDirection_ad - capsDirDotPtsVec;                                               \
                                                                                                \
   if ( tB < -hB )                                                                              \
   {                                                                                            \
      tB = -hB;                                                                                 \
      tA = tB * capsDirection_ad + ptsVec_ad;                                                   \
                                                                                                \
      if ( tA < -hA[ad] ) tA = -hA[ad];                                                         \
      else if ( tA > hA[ad] ) tA = hA[ad];                                                      \
   }                                                                                            \
   else if ( tB > hB )                                                                          \
   {                                                                                            \
      tB = hB;                                                                                  \
      tA = tB * capsDirection_ad + ptsVec_ad;                                                   \
                                                                                                \
      if ( tA < -hA[ad] ) tA = -hA[ad];                                                         \
      else if ( tA > hA[ad] ) tA = hA[ad];                                                      \
   }                                                                                            \
                                                                                                \
   /* make vector to point at tB on edge B from the center of edge A. */                        \
   /* test that it lies inside edge A's voronoi region. */                                      \
                                                                                                \
   ptsVec += capsDirection * tB;                                                                \
                                                                                                \
   PfxVector3 cptsVec( mulPerElem( ptsVec, signsA ) );                                             \
                                                                                                \
   inVoronoi = ( cptsVec[ac] >= voronoiTol * cptsVec[2] ) &&                                    \
               ( cptsVec[2] >= voronoiTol * cptsVec[ac] );                                      \
                                                                                                \
   ptsVec.set##ad_letter( ptsVec.get##ad_letter() - tA );                                       \
                                                                                                \
   return lengthSqr(ptsVec);                                                                    \
}

PfxFloat
EdgeEdgeTest_01(
	PfxBool& inVoronoi,
	PfxFloat& tA,
	PfxFloat& tB,
	PfxVector3& ptsVec,
	const PfxVector3& hA,
	PfxFloat hB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG offsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG capsDirection,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesA )
{
	EdgeEdgeTest( 0, X, 1, Y );
}

PfxFloat
EdgeEdgeTest_10(
	PfxBool& inVoronoi,
	PfxFloat& tA,
	PfxFloat& tB,
	PfxVector3& ptsVec,
	const PfxVector3& hA,
	PfxFloat hB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG offsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG capsDirection,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesA )
{
	EdgeEdgeTest( 1, Y, 0, X );
}

#define EdgeEdge_SetNewMin( ac_letter, ad_letter )                         \
{                                                                          \
   minDistSqr = distSqr;                                                   \
   closestPtsVec = ptsVec;                                                 \
   localPointA.set##ac_letter(scalesA.get##ac_letter());                   \
   localPointA.set##ad_letter(tA);                                         \
   segmentParamB = tB;                                                     \
   otherFaceDimA = testOtherFaceDimA;                                      \
}

void
EdgeEdgeTests(
	PfxBool& done,
	PfxFloat& minDistSqr,
	PfxVector3& closestPtsVec,
	PfxPoint3& localPointA,
	PfxFloat& segmentParamB,
	int & otherFaceDimA,
	const PfxVector3& hA,
	PfxFloat hB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG offsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG capsDirection,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesA,
	PfxBool first )
{
	PfxVector3 ptsVec;
	PfxFloat tA, tB;
	int testOtherFaceDimA;

	testOtherFaceDimA = 0;

	PfxFloat distSqr = EdgeEdgeTest_01( done, tA, tB, ptsVec, hA, hB,
									 offsetAB, capsDirection, signsA, scalesA );

	if ( first ) {
		EdgeEdge_SetNewMin( X, Y );
	} else {
		if ( distSqr < minDistSqr ) {
			EdgeEdge_SetNewMin( X, Y );
		}
	}

	if ( done )
		return;

	signsA.setX( -signsA.getX() );
	scalesA.setX( -scalesA.getX() );

	distSqr = EdgeEdgeTest_01( done, tA, tB, ptsVec, hA, hB,
							   offsetAB, capsDirection, signsA, scalesA );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( X, Y );
	}

	if ( done )
		return;

	testOtherFaceDimA = 1;

	distSqr = EdgeEdgeTest_10( done, tA, tB, ptsVec, hA, hB,
							   offsetAB, capsDirection, signsA, scalesA );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X );
	}

	if ( done )
		return;

	signsA.setY( -signsA.getY() );
	scalesA.setY( -scalesA.getY() );

	distSqr = EdgeEdgeTest_10( done, tA, tB, ptsVec, hA, hB,
							   offsetAB, capsDirection, signsA, scalesA );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X );
	}
}

PfxFloat pfxContactBoxCapsule(
	PfxVector3 &normal,PfxPoint3 &pointA,PfxPoint3 &pointB,
	void *shapeA,const PfxTransform3 &transformA,
	void *shapeB,const PfxTransform3 &transformB,
	PfxFloat distanceThreshold)
{
	PfxBox boxA = *((PfxBox*)shapeA);
	PfxCapsule capsuleB = *((PfxCapsule*)shapeB);

	PfxVector3 ident[3] = {
		PfxVector3(1.0,0.0,0.0),
		PfxVector3(0.0,1.0,0.0),
		PfxVector3(0.0,0.0,1.0),
	};

	// get capsule position and direction in box's coordinate system

	PfxMatrix3 matrixA = transformA.getUpper3x3();
	PfxMatrix3 matrixAinv = transpose(matrixA);

	PfxVector3 directionB = transformB.getUpper3x3().getCol0();
	PfxVector3 translationB = transformB.getTranslation();

	PfxVector3 capsDirection = matrixAinv * directionB;
	PfxVector3 absCapsDirection = absPerElem(capsDirection);
	PfxVector3 offsetAB = matrixAinv * (translationB - transformA.getTranslation());

	// find separating axis with largest gap between projections

	BoxCapsSepAxisType axisType;
	PfxVector3 axisA;
	PfxFloat maxGap;
	int faceDimA = 0, edgeDimA = 0;

	// face axes

	// can compute all the gaps at once with VU0

	PfxVector3 gapsA = absPerElem(offsetAB) - boxA.m_half - absCapsDirection * capsuleB.m_halfLen;

	AaxisTest( 0, X, true );
	AaxisTest( 1, Y, false );
	AaxisTest( 2, Z, false );

	// cross product axes

	// compute gaps on all cross product axes using some VU0 math.  suppose there's a tradeoff
	// between doing this with SIMD all at once or without SIMD in each cross product test, since
	// some test might exit early.

	PfxVector3 lsqrs, projOffset, projAhalf;

	PfxMatrix3 crossProdMat = crossMatrix(capsDirection) * PfxMatrix3::identity();
	PfxMatrix3 crossProdMatT = crossMatrix(-capsDirection) * PfxMatrix3::identity();

	lsqrs = mulPerElem( crossProdMatT.getCol0(), crossProdMatT.getCol0() ) +
			mulPerElem( crossProdMatT.getCol1(), crossProdMatT.getCol1() ) +
			mulPerElem( crossProdMatT.getCol2(), crossProdMatT.getCol2() );

	projOffset = crossProdMatT * offsetAB;
	projAhalf = absPerElem(crossProdMatT) * boxA.m_half;

	PfxVector3 gapsAxB = absPerElem(projOffset) - projAhalf;

	CrossAxisTest( 0, X );
	CrossAxisTest( 1, Y );
	CrossAxisTest( 2, Z );

	// make axis point from box center towards capsule center.

	if ( dot(axisA,offsetAB) < 0.0f )
		axisA = -axisA;

	// find the face on box whose normal best matches the separating axis. will use the entire
	// face only in degenerate cases.
	//
	// to make things simpler later, change the coordinate system so that the face normal is the z
	// direction.  if an edge cross product axis was chosen above, also align the box edge to the y
	// axis.  this saves the later tests from having to know which face was chosen.  changing the
	// coordinate system involves permuting vector elements, so construct a permutation matrix.
	// I believe this is a faster way to permute a bunch of vectors than using arrays.

	int dimA[3];

	if ( axisType == CROSS_AXIS ) {
		PfxVector3 absAxisA = PfxVector3(absPerElem(axisA));

		dimA[1] = edgeDimA;

		if ( edgeDimA == 0 ) {
			if ( absAxisA[1] > absAxisA[2] ) {
				dimA[0] = 2;
				dimA[2] = 1;
			} else                             {
				dimA[0] = 1;
				dimA[2] = 2;
			}
		} else if ( edgeDimA == 1 ) {
			if ( absAxisA[2] > absAxisA[0] ) {
				dimA[0] = 0;
				dimA[2] = 2;
			} else                             {
				dimA[0] = 2;
				dimA[2] = 0;
			}
		} else {
			if ( absAxisA[0] > absAxisA[1] ) {
				dimA[0] = 1;
				dimA[2] = 0;
			} else                             {
				dimA[0] = 0;
				dimA[2] = 1;
			}
		}
	} else {
		dimA[2] = faceDimA;
		dimA[0] = (faceDimA+1)%3;
		dimA[1] = (faceDimA+2)%3;
	}

	PfxMatrix3 aperm_col;

	aperm_col.setCol0(ident[dimA[0]]);
	aperm_col.setCol1(ident[dimA[1]]);
	aperm_col.setCol2(ident[dimA[2]]);

	PfxMatrix3 aperm_row = transpose(aperm_col);

	// permute vectors to be in face coordinate system.

	PfxVector3 offsetAB_perm = aperm_row * offsetAB;
	PfxVector3 halfA_perm = aperm_row * boxA.m_half;
	PfxVector3 signsA_perm = copySignPerElem(PfxVector3(1.0f), aperm_row * axisA);
	PfxVector3 scalesA_perm = mulPerElem( signsA_perm, halfA_perm );
	PfxVector3 capsDirection_perm = aperm_row * capsDirection;
	PfxFloat signB = (-dot(capsDirection,axisA) > 0.0f)? 1.0f : -1.0f;
	PfxFloat scaleB = signB * capsuleB.m_halfLen;

	// compute the vector between the center of the box face and the capsule center

	offsetAB_perm.setZ( offsetAB_perm.getZ() - scalesA_perm.getZ() );

	// if box and capsule overlap, this will separate them for finding points of penetration.

	if ( maxGap < 0.0f ) {
		offsetAB_perm -= aperm_row * axisA * maxGap * 1.01f;
	}

	// for each vertex/face or edge/edge pair of box face and line segment, find the closest
	// points.
	//
	// these points each have an associated feature (vertex, edge, or face).  if each
	// point is in the external Voronoi region of the other's feature, they are the
	// closest points of the objects, and the algorithm can exit.
	//
	// the feature pairs are arranged so that in the general case, the first test will
	// succeed.  degenerate cases (line segment parallel to face) may require up to all tests
	// in the worst case.
	//
	// if for some reason no case passes the Voronoi test, the features with the minimum
	// distance are returned.

	PfxVector3 closestPtsVec_perm;
	PfxPoint3 localPointA_perm;
	PfxFloat minDistSqr;
	PfxFloat segmentParamB;
	PfxBool done;

	localPointA_perm.setZ( scalesA_perm.getZ() );
	scalesA_perm.setZ(0.0f);

	PfxVector3 hA_perm( halfA_perm );

	int otherFaceDimA;

	if ( axisType == CROSS_AXIS ) {
		EdgeEdgeTests( done, minDistSqr, closestPtsVec_perm, localPointA_perm, segmentParamB,
					   otherFaceDimA,
					   hA_perm, capsuleB.m_halfLen, offsetAB_perm, capsDirection_perm, signsA_perm,
					   scalesA_perm, true );

		if ( !done ) {
			VertexBFaceATests( done, minDistSqr, closestPtsVec_perm, localPointA_perm, segmentParamB,
							   hA_perm, offsetAB_perm, capsDirection_perm, signB, scaleB, false );
		}
	} else {
		VertexBFaceATests( done, minDistSqr, closestPtsVec_perm, localPointA_perm, segmentParamB,
						   hA_perm, offsetAB_perm, capsDirection_perm, signB, scaleB, true );

		if ( !done ) {
			EdgeEdgeTests( done, minDistSqr, closestPtsVec_perm, localPointA_perm, segmentParamB,
						   otherFaceDimA,
						   hA_perm, capsuleB.m_halfLen, offsetAB_perm, capsDirection_perm, signsA_perm,
						   scalesA_perm, false );
		}
	}

	// compute normal

	PfxBool centerInside = ( signsA_perm.getZ() * closestPtsVec_perm.getZ() < 0.0f );

	if ( centerInside || ( minDistSqr < lenSqrTol ) ) {
		normal = matrixA * axisA;
	} else {
		PfxVector3 closestPtsVec = aperm_col * closestPtsVec_perm;
		normal = matrixA * ( closestPtsVec * (1.0f/sqrtf( minDistSqr )) );
	}

	// compute box point

	pointA = PfxPoint3( aperm_col * PfxVector3( localPointA_perm ) );

	// compute capsule point

	pointB = PfxPoint3( transpose(transformB.getUpper3x3()) * ( directionB * segmentParamB - normal * capsuleB.m_radius ) );

	if ( centerInside ) {
		return (-sqrtf( minDistSqr ) - capsuleB.m_radius);
	} else {
		return (sqrtf( minDistSqr ) - capsuleB.m_radius);
	}
}

} //namespace PhysicsEffects
} //namespace sce
