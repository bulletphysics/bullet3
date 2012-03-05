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
#include "pfx_contact_box_box.h"

namespace sce {
namespace PhysicsEffects {

enum BoxSepAxisType
{
	A_AXIS, B_AXIS, CROSS_AXIS
};

//-------------------------------------------------------------------------------------------------
// voronoiTol: bevels Voronoi planes slightly which helps when features are parallel.
//-------------------------------------------------------------------------------------------------

static const PfxFloat voronoiTol = -1.0e-5f;

//-------------------------------------------------------------------------------------------------
// separating axis tests: gaps along each axis are computed, and the axis with the maximum
// gap is stored.  cross product axes are normalized.
//-------------------------------------------------------------------------------------------------

#define AaxisTest( dim, letter, first )                                                         \
{                                                                                               \
   if ( first )                                                                                 \
   {                                                                                            \
      maxGap = gap = gapsA.get##letter();                                                      \
      if ( gap > distanceThreshold ) return gap;                                                \
      axisType = A_AXIS;                                                                        \
      faceDimA = dim;                                                                           \
      axisA = ident[dim];                                                          \
   }                                                                                            \
   else                                                                                         \
   {                                                                                            \
      gap = gapsA.get##letter();                                                               \
      if ( gap > distanceThreshold ) return gap;                                                \
      else if ( gap > maxGap )                                                                  \
      {                                                                                         \
         maxGap = gap;                                                                          \
         axisType = A_AXIS;                                                                     \
         faceDimA = dim;                                                                        \
         axisA = ident[dim];                                                       \
      }                                                                                         \
   }                                                                                            \
}


#define BaxisTest( dim, letter )                                                                \
{                                                                                               \
   gap = gapsB.get##letter();                                                                  \
   if ( gap > distanceThreshold ) return gap;                                                   \
   else if ( gap > maxGap )                                                                     \
   {                                                                                            \
      maxGap = gap;                                                                             \
      axisType = B_AXIS;                                                                        \
      faceDimB = dim;                                                                           \
      axisB = ident[dim];                                                          \
   }                                                                                            \
}

#define CrossAxisTest( dima, dimb, letterb )                                                    \
{                                                                                               \
   const PfxFloat lsqr_tolerance = 1.0e-30f;                                                       \
   PfxFloat lsqr;                                                                                  \
                                                                                                \
   lsqr = lsqrs.getCol##dima().get##letterb();                                                \
                                                                                                \
   if ( lsqr > lsqr_tolerance )                                                                 \
   {                                                                                            \
      PfxFloat l_recip = 1.0f / sqrtf( lsqr );                                                     \
      gap = PfxFloat(gapsAxB.getCol##dima().get##letterb()) * l_recip;                           \
                                                                                                \
      if ( gap > distanceThreshold )                                                            \
      {                                                                                         \
         return gap;                                                                            \
      }                                                                                         \
                                                                                                \
      if ( gap > maxGap )                                                                       \
      {                                                                                         \
         maxGap = gap;                                                                          \
         axisType = CROSS_AXIS;                                                                 \
         edgeDimA = dima;                                                                       \
         edgeDimB = dimb;                                                                       \
         axisA = cross(ident[dima],matrixAB.getCol##dimb()) * l_recip;            \
      }                                                                                         \
   }                                                                                            \
}

//-------------------------------------------------------------------------------------------------
// tests whether a vertex of box B and a face of box A are the closest features
//-------------------------------------------------------------------------------------------------

inline
PfxFloat
VertexBFaceATest(
	PfxBool & inVoronoi,
	PfxFloat & t0,
	PfxFloat & t1,
	const PfxVector3 & hA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetBA,
	const PfxMatrix3 & matrixAB,
	const PfxMatrix3 & matrixBA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesB )
{
	// compute a corner of box B in A's coordinate system

	PfxVector3 corner =
		PfxVector3( faceOffsetAB + matrixAB.getCol0() * scalesB.getX() + matrixAB.getCol1() * scalesB.getY() );

	// compute the parameters of the point on A, closest to this corner

	t0 = corner[0];
	t1 = corner[1];

	if ( t0 > hA[0] )
		t0 = hA[0];
	else if ( t0 < -hA[0] )
		t0 = -hA[0];
	if ( t1 > hA[1] )
		t1 = hA[1];
	else if ( t1 < -hA[1] )
		t1 = -hA[1];

	// do the Voronoi test: already know the point on B is in the Voronoi region of the
	// point on A, check the reverse.

	PfxVector3 facePointB =
		PfxVector3( mulPerElem( faceOffsetBA + matrixBA.getCol0() * t0 + matrixBA.getCol1() * t1 - scalesB, signsB ) );

	inVoronoi = ( ( facePointB[0] >= voronoiTol * facePointB[2] ) &&
				  ( facePointB[1] >= voronoiTol * facePointB[0] ) &&
				  ( facePointB[2] >= voronoiTol * facePointB[1] ) );

	return (SCE_PFX_SQR( corner[0] - t0 ) + SCE_PFX_SQR( corner[1] - t1 ) + SCE_PFX_SQR( corner[2] ));
}

#define VertexBFaceA_SetNewMin()                \
{                                               \
   minDistSqr = distSqr;                        \
   localPointA.setX(t0);                        \
   localPointA.setY(t1);                        \
   localPointB.setX( scalesB.getX() );          \
   localPointB.setY( scalesB.getY() );          \
}

void
VertexBFaceATests(
	PfxBool & done,
	PfxFloat & minDistSqr,
	PfxPoint3 & localPointA,
	PfxPoint3 & localPointB,
	const PfxVector3 & hA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetBA,
	const PfxMatrix3 & matrixAB,
	const PfxMatrix3 & matrixBA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesB,
	PfxBool first )
{
	PfxFloat t0, t1;
	PfxFloat distSqr;

	distSqr = VertexBFaceATest( done, t0, t1, hA, faceOffsetAB, faceOffsetBA,
								matrixAB, matrixBA, signsB, scalesB );

	if ( first ) {
		VertexBFaceA_SetNewMin();
	} else {
		if ( distSqr < minDistSqr ) {
			VertexBFaceA_SetNewMin();
		}
	}

	if ( done )
		return;

	signsB.setX( -signsB.getX() );
	scalesB.setX( -scalesB.getX() );

	distSqr = VertexBFaceATest( done, t0, t1, hA, faceOffsetAB, faceOffsetBA,
								matrixAB, matrixBA, signsB, scalesB );

	if ( distSqr < minDistSqr ) {
		VertexBFaceA_SetNewMin();
	}

	if ( done )
		return;

	signsB.setY( -signsB.getY() );
	scalesB.setY( -scalesB.getY() );

	distSqr = VertexBFaceATest( done, t0, t1, hA, faceOffsetAB, faceOffsetBA,
								matrixAB, matrixBA, signsB, scalesB );

	if ( distSqr < minDistSqr ) {
		VertexBFaceA_SetNewMin();
	}

	if ( done )
		return;

	signsB.setX( -signsB.getX() );
	scalesB.setX( -scalesB.getX() );

	distSqr = VertexBFaceATest( done, t0, t1, hA, faceOffsetAB, faceOffsetBA,
								matrixAB, matrixBA, signsB, scalesB );

	if ( distSqr < minDistSqr ) {
		VertexBFaceA_SetNewMin();
	}
}

//-------------------------------------------------------------------------------------------------
// VertexAFaceBTest: tests whether a vertex of box A and a face of box B are the closest features
//-------------------------------------------------------------------------------------------------

inline
PfxFloat
VertexAFaceBTest(
	PfxBool & inVoronoi,
	PfxFloat & t0,
	PfxFloat & t1,
	const PfxVector3 & hB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetBA,
	const PfxMatrix3 & matrixAB,
	const PfxMatrix3 & matrixBA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesA )
{
	PfxVector3 corner =
		PfxVector3( faceOffsetBA + matrixBA.getCol0() * scalesA.getX() + matrixBA.getCol1() * scalesA.getY() );

	t0 = corner[0];
	t1 = corner[1];

	if ( t0 > hB[0] )
		t0 = hB[0];
	else if ( t0 < -hB[0] )
		t0 = -hB[0];
	if ( t1 > hB[1] )
		t1 = hB[1];
	else if ( t1 < -hB[1] )
		t1 = -hB[1];

	PfxVector3 facePointA =
		PfxVector3( mulPerElem( faceOffsetAB + matrixAB.getCol0() * t0 + matrixAB.getCol1() * t1 - scalesA, signsA ) );

	inVoronoi = ( ( facePointA[0] >= voronoiTol * facePointA[2] ) &&
				  ( facePointA[1] >= voronoiTol * facePointA[0] ) &&
				  ( facePointA[2] >= voronoiTol * facePointA[1] ) );

	return (SCE_PFX_SQR( corner[0] - t0 ) + SCE_PFX_SQR( corner[1] - t1 ) + SCE_PFX_SQR( corner[2] ));
}

#define VertexAFaceB_SetNewMin()                \
{                                               \
   minDistSqr = distSqr;                        \
   localPointB.setX(t0);                        \
   localPointB.setY(t1);                        \
   localPointA.setX( scalesA.getX() );          \
   localPointA.setY( scalesA.getY() );          \
}

void
VertexAFaceBTests(
	PfxBool & done,
	PfxFloat & minDistSqr,
	PfxPoint3 & localPointA,
	PfxPoint3 & localPointB,
	const PfxVector3 & hB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetBA,
	const PfxMatrix3 & matrixAB,
	const PfxMatrix3 & matrixBA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesA,
	PfxBool first )
{
	PfxFloat t0, t1;
	PfxFloat distSqr;

	distSqr = VertexAFaceBTest( done, t0, t1, hB, faceOffsetAB, faceOffsetBA,
								matrixAB, matrixBA, signsA, scalesA );

	if ( first ) {
		VertexAFaceB_SetNewMin();
	} else {
		if ( distSqr < minDistSqr ) {
			VertexAFaceB_SetNewMin();
		}
	}

	if ( done )
		return;

	signsA.setX( -signsA.getX() );
	scalesA.setX( -scalesA.getX() );

	distSqr = VertexAFaceBTest( done, t0, t1, hB, faceOffsetAB, faceOffsetBA,
								matrixAB, matrixBA, signsA, scalesA );

	if ( distSqr < minDistSqr ) {
		VertexAFaceB_SetNewMin();
	}

	if ( done )
		return;

	signsA.setY( -signsA.getY() );
	scalesA.setY( -scalesA.getY() );

	distSqr = VertexAFaceBTest( done, t0, t1, hB, faceOffsetAB, faceOffsetBA,
								matrixAB, matrixBA, signsA, scalesA );

	if ( distSqr < minDistSqr ) {
		VertexAFaceB_SetNewMin();
	}

	if ( done )
		return;

	signsA.setX( -signsA.getX() );
	scalesA.setX( -scalesA.getX() );

	distSqr = VertexAFaceBTest( done, t0, t1, hB, faceOffsetAB, faceOffsetBA,
								matrixAB, matrixBA, signsA, scalesA );

	if ( distSqr < minDistSqr ) {
		VertexAFaceB_SetNewMin();
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

#define EdgeEdgeTest( ac, ac_letter, ad, ad_letter, bc, bc_letter, bd, bd_letter )              \
{                                                                                               \
   PfxVector3 edgeOffsetAB;                                                                          \
   PfxVector3 edgeOffsetBA;                                                                          \
                                                                                                \
   edgeOffsetAB = faceOffsetAB + matrixAB.getCol##bc() * scalesB.get##bc_letter();            \
   edgeOffsetAB.set##ac_letter( edgeOffsetAB.get##ac_letter() - scalesA.get##ac_letter() );  \
                                                                                                \
   edgeOffsetBA = faceOffsetBA + matrixBA.getCol##ac() * scalesA.get##ac_letter();            \
   edgeOffsetBA.set##bc_letter( edgeOffsetBA.get##bc_letter() - scalesB.get##bc_letter() );  \
                                                                                                \
   PfxFloat dirDot = matrixAB.getCol##bd().get##ad_letter();                                     \
   PfxFloat denom = 1.0f - dirDot*dirDot;                                                          \
   PfxFloat edgeOffsetAB_ad = edgeOffsetAB.get##ad_letter();                                      \
   PfxFloat edgeOffsetBA_bd = edgeOffsetBA.get##bd_letter();                                      \
                                                                                                \
   if ( denom == 0.0f )                                                                         \
   {                                                                                            \
      tA = 0.0f;                                                                                \
   }                                                                                            \
   else                                                                                         \
   {                                                                                            \
      tA = ( edgeOffsetAB_ad + edgeOffsetBA_bd * dirDot ) / denom;                              \
   }                                                                                            \
                                                                                                \
   if ( tA < -hA[ad] ) tA = -hA[ad];                                                            \
   else if ( tA > hA[ad] ) tA = hA[ad];                                                         \
                                                                                                \
   tB = tA * dirDot + edgeOffsetBA_bd;                                                          \
                                                                                                \
   if ( tB < -hB[bd] )                                                                          \
   {                                                                                            \
      tB = -hB[bd];                                                                             \
      tA = tB * dirDot + edgeOffsetAB_ad;                                                       \
                                                                                                \
      if ( tA < -hA[ad] ) tA = -hA[ad];                                                         \
      else if ( tA > hA[ad] ) tA = hA[ad];                                                      \
   }                                                                                            \
   else if ( tB > hB[bd] )                                                                      \
   {                                                                                            \
      tB = hB[bd];                                                                              \
      tA = tB * dirDot + edgeOffsetAB_ad;                                                       \
                                                                                                \
      if ( tA < -hA[ad] ) tA = -hA[ad];                                                         \
      else if ( tA > hA[ad] ) tA = hA[ad];                                                      \
   }                                                                                            \
                                                                                                \
   PfxVector3 edgeOffAB = PfxVector3( mulPerElem( edgeOffsetAB + matrixAB.getCol##bd() * tB, signsA ) );\
   PfxVector3 edgeOffBA = PfxVector3( mulPerElem( edgeOffsetBA + matrixBA.getCol##ad() * tA, signsB ) );\
                                                                                                \
   inVoronoi = ( edgeOffAB[ac] >= voronoiTol * edgeOffAB[2] ) &&                                \
               ( edgeOffAB[2] >= voronoiTol * edgeOffAB[ac] ) &&                                \
               ( edgeOffBA[bc] >= voronoiTol * edgeOffBA[2] ) &&                                \
               ( edgeOffBA[2] >= voronoiTol * edgeOffBA[bc] );                                  \
                                                                                                \
   edgeOffAB[ad] -= tA;                                                                         \
   edgeOffBA[bd] -= tB;                                                                         \
                                                                                                \
   return dot(edgeOffAB,edgeOffAB);                                                             \
}

PfxFloat
EdgeEdgeTest_0101(
	PfxBool & inVoronoi,
	PfxFloat & tA,
	PfxFloat & tB,
	const PfxVector3 & hA,
	const PfxVector3 & hB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetBA,
	const PfxMatrix3 & matrixAB,
	const PfxMatrix3 & matrixBA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesB )
{
	EdgeEdgeTest( 0, X, 1, Y, 0, X, 1, Y );
}

PfxFloat
EdgeEdgeTest_0110(
	PfxBool & inVoronoi,
	PfxFloat & tA,
	PfxFloat & tB,
	const PfxVector3 & hA,
	const PfxVector3 & hB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetBA,
	const PfxMatrix3 & matrixAB,
	const PfxMatrix3 & matrixBA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesB )
{
	EdgeEdgeTest( 0, X, 1, Y, 1, Y, 0, X );
}

PfxFloat
EdgeEdgeTest_1001(
	PfxBool & inVoronoi,
	PfxFloat & tA,
	PfxFloat & tB,
	const PfxVector3 & hA,
	const PfxVector3 & hB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetBA,
	const PfxMatrix3 & matrixAB,
	const PfxMatrix3 & matrixBA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesB )
{
	EdgeEdgeTest( 1, Y, 0, X, 0, X, 1, Y );
}

PfxFloat
EdgeEdgeTest_1010(
	PfxBool & inVoronoi,
	PfxFloat & tA,
	PfxFloat & tB,
	const PfxVector3 & hA,
	const PfxVector3 & hB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetBA,
	const PfxMatrix3 & matrixAB,
	const PfxMatrix3 & matrixBA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesB )
{
	EdgeEdgeTest( 1, Y, 0, X, 1, Y, 0, X );
}

#define EdgeEdge_SetNewMin( ac_letter, ad_letter, bc_letter, bd_letter )   \
{                                                                          \
   minDistSqr = distSqr;                                                   \
   localPointA.set##ac_letter(scalesA.get##ac_letter());                 \
   localPointA.set##ad_letter(tA);                                        \
   localPointB.set##bc_letter(scalesB.get##bc_letter());                 \
   localPointB.set##bd_letter(tB);                                        \
   otherFaceDimA = testOtherFaceDimA;                                      \
   otherFaceDimB = testOtherFaceDimB;                                      \
}

void
EdgeEdgeTests(
	PfxBool & done,
	PfxFloat & minDistSqr,
	PfxPoint3 & localPointA,
	PfxPoint3 & localPointB,
	int & otherFaceDimA,
	int & otherFaceDimB,
	const PfxVector3 & hA,
	const PfxVector3 & hB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetAB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG faceOffsetBA,
	const PfxMatrix3 & matrixAB,
	const PfxMatrix3 & matrixBA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG signsB,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesA,
	PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG scalesB,
	PfxBool first )
{
	PfxFloat distSqr;
	PfxFloat tA, tB;

	int testOtherFaceDimA, testOtherFaceDimB;

	testOtherFaceDimA = 0;
	testOtherFaceDimB = 0;

	distSqr = EdgeEdgeTest_0101( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( first ) {
		EdgeEdge_SetNewMin( X, Y, X, Y );
	} else {
		if ( distSqr < minDistSqr ) {
			EdgeEdge_SetNewMin( X, Y, X, Y );
		}
	}

	if ( done )
		return;

	signsA.setX( -signsA.getX() );
	scalesA.setX( -scalesA.getX() );

	distSqr = EdgeEdgeTest_0101( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( X, Y, X, Y );
	}

	if ( done )
		return;

	signsB.setX( -signsB.getX() );
	scalesB.setX( -scalesB.getX() );

	distSqr = EdgeEdgeTest_0101( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( X, Y, X, Y );
	}

	if ( done )
		return;

	signsA.setX( -signsA.getX() );
	scalesA.setX( -scalesA.getX() );

	distSqr = EdgeEdgeTest_0101( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( X, Y, X, Y );
	}

	if ( done )
		return;

	testOtherFaceDimA = 1;
	testOtherFaceDimB = 0;
	signsB.setX( -signsB.getX() );
	scalesB.setX( -scalesB.getX() );

	distSqr = EdgeEdgeTest_1001( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X, X, Y );
	}

	if ( done )
		return;

	signsA.setY( -signsA.getY() );
	scalesA.setY( -scalesA.getY() );

	distSqr = EdgeEdgeTest_1001( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X, X, Y );
	}

	if ( done )
		return;

	signsB.setX( -signsB.getX() );
	scalesB.setX( -scalesB.getX() );

	distSqr = EdgeEdgeTest_1001( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X, X, Y );
	}

	if ( done )
		return;

	signsA.setY( -signsA.getY() );
	scalesA.setY( -scalesA.getY() );

	distSqr = EdgeEdgeTest_1001( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X, X, Y );
	}

	if ( done )
		return;

	testOtherFaceDimA = 0;
	testOtherFaceDimB = 1;
	signsB.setX( -signsB.getX() );
	scalesB.setX( -scalesB.getX() );

	distSqr = EdgeEdgeTest_0110( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( X, Y, Y, X );
	}

	if ( done )
		return;

	signsA.setX( -signsA.getX() );
	scalesA.setX( -scalesA.getX() );

	distSqr = EdgeEdgeTest_0110( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( X, Y, Y, X );
	}

	if ( done )
		return;

	signsB.setY( -signsB.getY() );
	scalesB.setY( -scalesB.getY() );

	distSqr = EdgeEdgeTest_0110( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( X, Y, Y, X );
	}

	if ( done )
		return;

	signsA.setX( -signsA.getX() );
	scalesA.setX( -scalesA.getX() );

	distSqr = EdgeEdgeTest_0110( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( X, Y, Y, X );
	}

	if ( done )
		return;

	testOtherFaceDimA = 1;
	testOtherFaceDimB = 1;
	signsB.setY( -signsB.getY() );
	scalesB.setY( -scalesB.getY() );

	distSqr = EdgeEdgeTest_1010( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X, Y, X );
	}

	if ( done )
		return;

	signsA.setY( -signsA.getY() );
	scalesA.setY( -scalesA.getY() );

	distSqr = EdgeEdgeTest_1010( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X, Y, X );
	}

	if ( done )
		return;

	signsB.setY( -signsB.getY() );
	scalesB.setY( -scalesB.getY() );

	distSqr = EdgeEdgeTest_1010( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X, Y, X );
	}

	if ( done )
		return;

	signsA.setY( -signsA.getY() );
	scalesA.setY( -scalesA.getY() );

	distSqr = EdgeEdgeTest_1010( done, tA, tB, hA, hB, faceOffsetAB, faceOffsetBA,
								 matrixAB, matrixBA, signsA, signsB, scalesA, scalesB );

	if ( distSqr < minDistSqr ) {
		EdgeEdge_SetNewMin( Y, X, Y, X );
	}
}

PfxFloat pfxContactBoxBox(
	PfxVector3 &normal,PfxPoint3 &pointA,PfxPoint3 &pointB,
	void *shapeA,const PfxTransform3 &transformA,
	void *shapeB,const PfxTransform3 &transformB,
	PfxFloat distanceThreshold)
{
	PfxBox &boxA = *((PfxBox*)shapeA);
	PfxBox &boxB = *((PfxBox*)shapeB);

	PfxVector3 ident[3] = {
		PfxVector3(1.0,0.0,0.0),
		PfxVector3(0.0,1.0,0.0),
		PfxVector3(0.0,0.0,1.0),
	};

	// get relative transformations

	PfxTransform3 transformAB, transformBA;
	PfxMatrix3 matrixAB, matrixBA;
	PfxVector3 offsetAB, offsetBA;

	transformAB = orthoInverse(transformA) * transformB;
	transformBA = orthoInverse(transformAB);

	matrixAB = transformAB.getUpper3x3();
	offsetAB = transformAB.getTranslation();
	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	PfxMatrix3 absMatrixAB = absPerElem(matrixAB);
	PfxMatrix3 absMatrixBA = absPerElem(matrixBA);

	// find separating axis with largest gap between projections

	BoxSepAxisType axisType;
	PfxVector3 axisA(0.0f), axisB(0.0f);
	PfxFloat gap, maxGap;
	int faceDimA = 0, faceDimB = 0, edgeDimA = 0, edgeDimB = 0;

	// face axes

	PfxVector3  gapsA   = absPerElem(offsetAB) - boxA.m_half - absMatrixAB * boxB.m_half;

	AaxisTest(0,X,true);
	AaxisTest(1,Y,false);
	AaxisTest(2,Z,false);

	PfxVector3  gapsB   = absPerElem(offsetBA) - boxB.m_half - absMatrixBA * boxA.m_half;

	BaxisTest(0,X);
	BaxisTest(1,Y);
	BaxisTest(2,Z);

	// cross product axes

	// 外積が０のときの対策
	absMatrixAB += PfxMatrix3(1.0e-5f);
	absMatrixBA += PfxMatrix3(1.0e-5f);

	PfxMatrix3 lsqrs, projOffset, projAhalf, projBhalf;

	lsqrs.setCol0( mulPerElem( matrixBA.getCol2(), matrixBA.getCol2() ) +
				   mulPerElem( matrixBA.getCol1(), matrixBA.getCol1() ) );
	lsqrs.setCol1( mulPerElem( matrixBA.getCol2(), matrixBA.getCol2() ) +
				   mulPerElem( matrixBA.getCol0(), matrixBA.getCol0() ) );
	lsqrs.setCol2( mulPerElem( matrixBA.getCol1(), matrixBA.getCol1() ) +
				   mulPerElem( matrixBA.getCol0(), matrixBA.getCol0() ) );

	projOffset.setCol0(matrixBA.getCol1() * offsetAB.getZ() - matrixBA.getCol2() * offsetAB.getY());
	projOffset.setCol1(matrixBA.getCol2() * offsetAB.getX() - matrixBA.getCol0() * offsetAB.getZ());
	projOffset.setCol2(matrixBA.getCol0() * offsetAB.getY() - matrixBA.getCol1() * offsetAB.getX());

	projAhalf.setCol0(absMatrixBA.getCol1() * boxA.m_half.getZ() + absMatrixBA.getCol2() * boxA.m_half.getY());
	projAhalf.setCol1(absMatrixBA.getCol2() * boxA.m_half.getX() + absMatrixBA.getCol0() * boxA.m_half.getZ());
	projAhalf.setCol2(absMatrixBA.getCol0() * boxA.m_half.getY() + absMatrixBA.getCol1() * boxA.m_half.getX());

	projBhalf.setCol0(absMatrixAB.getCol1() * boxB.m_half.getZ() + absMatrixAB.getCol2() * boxB.m_half.getY());
	projBhalf.setCol1(absMatrixAB.getCol2() * boxB.m_half.getX() + absMatrixAB.getCol0() * boxB.m_half.getZ());
	projBhalf.setCol2(absMatrixAB.getCol0() * boxB.m_half.getY() + absMatrixAB.getCol1() * boxB.m_half.getX());

	PfxMatrix3 gapsAxB = absPerElem(projOffset) - projAhalf - transpose(projBhalf);

	CrossAxisTest(0,0,X);
	CrossAxisTest(0,1,Y);
	CrossAxisTest(0,2,Z);
	CrossAxisTest(1,0,X);
	CrossAxisTest(1,1,Y);
	CrossAxisTest(1,2,Z);
	CrossAxisTest(2,0,X);
	CrossAxisTest(2,1,Y);
	CrossAxisTest(2,2,Z);

	// need to pick the face on each box whose normal best matches the separating axis.
	// will transform vectors to be in the coordinate system of this face to simplify things later.
	// for this, a permutation matrix can be used, which the next section computes.

	int dimA[3], dimB[3];

	if ( axisType == A_AXIS ) {
		if ( dot(axisA,offsetAB) < 0.0f )
			axisA = -axisA;
		axisB = matrixBA * -axisA;

		PfxVector3 absAxisB = PfxVector3(absPerElem(axisB));

		if ( ( absAxisB[0] > absAxisB[1] ) && ( absAxisB[0] > absAxisB[2] ) )
			faceDimB = 0;
		else if ( absAxisB[1] > absAxisB[2] )
			faceDimB = 1;
		else
			faceDimB = 2;
	} else if ( axisType == B_AXIS ) {
		if ( dot(axisB,offsetBA) < 0.0f )
			axisB = -axisB;
		axisA = matrixAB * -axisB;

		PfxVector3 absAxisA = PfxVector3(absPerElem(axisA));

		if ( ( absAxisA[0] > absAxisA[1] ) && ( absAxisA[0] > absAxisA[2] ) )
			faceDimA = 0;
		else if ( absAxisA[1] > absAxisA[2] )
			faceDimA = 1;
		else
			faceDimA = 2;
	}

	if ( axisType == CROSS_AXIS ) {
		if ( dot(axisA,offsetAB) < 0.0f )
			axisA = -axisA;
		axisB = matrixBA * -axisA;

		PfxVector3 absAxisA = PfxVector3(absPerElem(axisA));
		PfxVector3 absAxisB = PfxVector3(absPerElem(axisB));

		dimA[1] = edgeDimA;
		dimB[1] = edgeDimB;

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

		if ( edgeDimB == 0 ) {
			if ( absAxisB[1] > absAxisB[2] ) {
				dimB[0] = 2;
				dimB[2] = 1;
			} else                             {
				dimB[0] = 1;
				dimB[2] = 2;
			}
		} else if ( edgeDimB == 1 ) {
			if ( absAxisB[2] > absAxisB[0] ) {
				dimB[0] = 0;
				dimB[2] = 2;
			} else                             {
				dimB[0] = 2;
				dimB[2] = 0;
			}
		} else {
			if ( absAxisB[0] > absAxisB[1] ) {
				dimB[0] = 1;
				dimB[2] = 0;
			} else                             {
				dimB[0] = 0;
				dimB[2] = 1;
			}
		}
	} else {
		dimA[2] = faceDimA;
		dimA[0] = (faceDimA+1)%3;
		dimA[1] = (faceDimA+2)%3;
		dimB[2] = faceDimB;
		dimB[0] = (faceDimB+1)%3;
		dimB[1] = (faceDimB+2)%3;
	}

	PfxMatrix3 aperm_col, bperm_col;

	aperm_col.setCol0(ident[dimA[0]]);
	aperm_col.setCol1(ident[dimA[1]]);
	aperm_col.setCol2(ident[dimA[2]]);

	bperm_col.setCol0(ident[dimB[0]]);
	bperm_col.setCol1(ident[dimB[1]]);
	bperm_col.setCol2(ident[dimB[2]]);

	PfxMatrix3 aperm_row, bperm_row;

	aperm_row = transpose(aperm_col);
	bperm_row = transpose(bperm_col);

	// permute all box parameters to be in the face coordinate systems

	PfxMatrix3 matrixAB_perm = aperm_row * matrixAB * bperm_col;
	PfxMatrix3 matrixBA_perm = transpose(matrixAB_perm);

	PfxVector3 offsetAB_perm, offsetBA_perm;

	offsetAB_perm = aperm_row * offsetAB;
	offsetBA_perm = bperm_row * offsetBA;

	PfxVector3 halfA_perm, halfB_perm;

	halfA_perm = aperm_row * boxA.m_half;
	halfB_perm = bperm_row * boxB.m_half;

	// compute the vector between the centers of each face, in each face's coordinate frame

	PfxVector3 signsA_perm, signsB_perm, scalesA_perm, scalesB_perm, faceOffsetAB_perm, faceOffsetBA_perm;

	signsA_perm = copySignPerElem(PfxVector3(1.0f),aperm_row * axisA);
	signsB_perm = copySignPerElem(PfxVector3(1.0f),bperm_row * axisB);
	scalesA_perm = mulPerElem( signsA_perm, halfA_perm );
	scalesB_perm = mulPerElem( signsB_perm, halfB_perm );

	faceOffsetAB_perm = offsetAB_perm + matrixAB_perm.getCol2() * scalesB_perm.getZ();
	faceOffsetAB_perm.setZ( faceOffsetAB_perm.getZ() - scalesA_perm.getZ() );

	faceOffsetBA_perm = offsetBA_perm + matrixBA_perm.getCol2() * scalesA_perm.getZ();
	faceOffsetBA_perm.setZ( faceOffsetBA_perm.getZ() - scalesB_perm.getZ() );

	if ( maxGap < 0.0f ) {
		// if boxes overlap, this will separate the faces for finding points of penetration.

		faceOffsetAB_perm -= aperm_row * axisA * maxGap * 1.01f;
		faceOffsetBA_perm -= bperm_row * axisB * maxGap * 1.01f;
	}

	// for each vertex/face or edge/edge pair of the two faces, find the closest points.
	//
	// these points each have an associated box feature (vertex, edge, or face).  if each
	// point is in the external Voronoi region of the other's feature, they are the
	// closest points of the boxes, and the algorithm can exit.
	//
	// the feature pairs are arranged so that in the general case, the first test will
	// succeed.  degenerate cases (parallel faces) may require up to all tests in the
	// worst case.
	//
	// if for some reason no case passes the Voronoi test, the features with the minimum
	// distance are returned.

	PfxPoint3 localPointA_perm, localPointB_perm;
	PfxFloat minDistSqr;
	PfxBool done;

	PfxVector3 hA_perm( halfA_perm ), hB_perm( halfB_perm );

	localPointA_perm.setZ( scalesA_perm.getZ() );
	localPointB_perm.setZ( scalesB_perm.getZ() );
	scalesA_perm.setZ(0.0f);
	scalesB_perm.setZ(0.0f);

	int otherFaceDimA, otherFaceDimB;

	if ( axisType == CROSS_AXIS ) {
		EdgeEdgeTests( done, minDistSqr, localPointA_perm, localPointB_perm,
					   otherFaceDimA, otherFaceDimB,
					   hA_perm, hB_perm, faceOffsetAB_perm, faceOffsetBA_perm,
					   matrixAB_perm, matrixBA_perm, signsA_perm, signsB_perm,
					   scalesA_perm, scalesB_perm, true );

		if ( !done ) {
			VertexBFaceATests( done, minDistSqr, localPointA_perm, localPointB_perm,
							   hA_perm, faceOffsetAB_perm, faceOffsetBA_perm,
							   matrixAB_perm, matrixBA_perm, signsB_perm, scalesB_perm, false );

			if ( !done ) {
				VertexAFaceBTests( done, minDistSqr, localPointA_perm, localPointB_perm,
								   hB_perm, faceOffsetAB_perm, faceOffsetBA_perm,
								   matrixAB_perm, matrixBA_perm, signsA_perm, scalesA_perm, false );
			}
		}
	} else if ( axisType == B_AXIS ) {
		VertexAFaceBTests( done, minDistSqr, localPointA_perm, localPointB_perm,
						   hB_perm, faceOffsetAB_perm, faceOffsetBA_perm,
						   matrixAB_perm, matrixBA_perm, signsA_perm, scalesA_perm, true );

		if ( !done ) {
			VertexBFaceATests( done, minDistSqr, localPointA_perm, localPointB_perm,
							   hA_perm, faceOffsetAB_perm, faceOffsetBA_perm,
							   matrixAB_perm, matrixBA_perm, signsB_perm, scalesB_perm, false );

			if ( !done ) {
				EdgeEdgeTests( done, minDistSqr, localPointA_perm, localPointB_perm,
							   otherFaceDimA, otherFaceDimB,
							   hA_perm, hB_perm, faceOffsetAB_perm, faceOffsetBA_perm,
							   matrixAB_perm, matrixBA_perm, signsA_perm, signsB_perm,
							   scalesA_perm, scalesB_perm, false );
			}
		}
	} else {
		VertexBFaceATests( done, minDistSqr, localPointA_perm, localPointB_perm,
						   hA_perm, faceOffsetAB_perm, faceOffsetBA_perm,
						   matrixAB_perm, matrixBA_perm, signsB_perm, scalesB_perm, true );

		if ( !done ) {
			VertexAFaceBTests( done, minDistSqr, localPointA_perm, localPointB_perm,
							   hB_perm, faceOffsetAB_perm, faceOffsetBA_perm,
							   matrixAB_perm, matrixBA_perm, signsA_perm, scalesA_perm, false );

			if ( !done ) {
				EdgeEdgeTests( done, minDistSqr, localPointA_perm, localPointB_perm,
							   otherFaceDimA, otherFaceDimB,
							   hA_perm, hB_perm, faceOffsetAB_perm, faceOffsetBA_perm,
							   matrixAB_perm, matrixBA_perm, signsA_perm, signsB_perm,
							   scalesA_perm, scalesB_perm, false );
			}
		}
	}

	// convert local points from face-local to box-local coordinate system

	pointA = PfxPoint3( aperm_col * PfxVector3( localPointA_perm ) );
	pointB = PfxPoint3( bperm_col * PfxVector3( localPointB_perm ) );

	normal = transformA * axisA;

	//SCE_PFX_PRINTF("minDistSqr %f maxGap %f\n",minDistSqr,maxGap);
	//SCE_PFX_PRINTF("normal %f %f %f\n",normal[0],normal[1],normal[2]);
	//SCE_PFX_PRINTF("pointA %f %f %f\n",pointA[0],pointA[1],pointA[2]);
	//SCE_PFX_PRINTF("pointB %f %f %f\n",pointB[0],pointB[1],pointB[2]);

	if ( maxGap < 0.0f ) {
		return (maxGap);
	} else {
		return (sqrtf( minDistSqr ));
	}
}

} //namespace PhysicsEffects
} //namespace sce
