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
#include "../../../include/physics_effects/base_level/collision/pfx_sphere.h"
#include "pfx_contact_box_sphere.h"

namespace sce {
namespace PhysicsEffects {

static const PfxFloat lenSqrTol = 1.0e-30f;

inline
PfxFloat
VertexBFaceATest(
	PfxVector3& ptsVec,
	PfxFloat& t0,
	PfxFloat& t1,
	const PfxVector3& hA,
	const PfxVector3 &offsetAB )
{
	// compute center of sphere in box's coordinate system

	PfxVector3 cptsVec = PfxVector3(offsetAB);

	// compute the parameters of the point on the face

	t0 = cptsVec[0];
	t1 = cptsVec[1];

	if ( t0 > hA[0] )
		t0 = hA[0];
	else if ( t0 < -hA[0] )
		t0 = -hA[0];
	if ( t1 > hA[1] )
		t1 = hA[1];
	else if ( t1 < -hA[1] )
		t1 = -hA[1];

	cptsVec[0] -= t0;
	cptsVec[1] -= t1;

	ptsVec = PfxVector3( cptsVec );

	return dot(ptsVec,ptsVec);
}

PfxFloat pfxContactBoxSphere(
	PfxVector3 &normal,PfxPoint3 &pointA,PfxPoint3 &pointB,
	void *shapeA,const PfxTransform3 &transformA,
	void *shapeB,const PfxTransform3 &transformB,
	PfxFloat distanceThreshold)
{
	PfxBox &boxA = *((PfxBox*)shapeA);
	PfxSphere &sphereB = *((PfxSphere*)shapeB);
	
	PfxVector3 ident[3] = {
		PfxVector3(1.0,0.0,0.0),
		PfxVector3(0.0,1.0,0.0),
		PfxVector3(0.0,0.0,1.0),
	};
	
	//{
	//	PfxMatrix3 identity = PfxMatrix3::identity();
	//	ident[0] = identity.getCol0();
	//	ident[1] = identity.getCol1();
	//	ident[2] = identity.getCol2();
	//}

	// offsetAB is vector from A's center to B's center, in A's coordinate system

	PfxVector3 translationB = transformB.getTranslation();
	PfxVector3 offsetAB = transpose(transformA.getUpper3x3()) * ( translationB -
					   transformA.getTranslation() );

	// find separating axis with largest gap between objects

	PfxVector3 axisA;
	int   faceDimA;
	PfxFloat maxGap;

	PfxVector3 gapsA = absPerElem(offsetAB) - boxA.m_half - PfxVector3(sphereB.m_radius);
	PfxVector3 signsA = copySignPerElem(PfxVector3(1.0f),offsetAB);

	{
		PfxFloat gap = gapsA[0];

		if( gap > distanceThreshold ) {
			return gap;
		}

		maxGap = gap;
		faceDimA = 0;
		axisA = mulPerElem( ident[0], signsA );

		if( gap > maxGap ) {
			maxGap = gap;
			faceDimA = 0;
			axisA = mulPerElem( ident[0], signsA );
		}

		gap = gapsA[1];

		if( gap > distanceThreshold ) {
			return gap;
		}

		if( gap > maxGap ) {
			maxGap = gap;
			faceDimA = 1;
			axisA = mulPerElem( ident[1], signsA );
		}

		gap = gapsA[2];

		if( gap > distanceThreshold ) {
			return gap;
		}

		if( gap > maxGap ) {
			maxGap = gap;
			faceDimA = 2;
			axisA = mulPerElem( ident[2], signsA );
		}
	}

	// choose face in this direction, and make a new coordinate system which the z axis = face
	// normal, x and y axes tangent to the face.  to transform vectors into this coordinate
	// system, will use a permutation matrix.

	int dimA[3];

	dimA[2] = faceDimA;
	dimA[0] = (faceDimA+1)%3;
	dimA[1] = (faceDimA+2)%3;

	PfxMatrix3 apermCol;
	
	apermCol.setCol0(ident[dimA[0]]);
	apermCol.setCol1(ident[dimA[1]]);
	apermCol.setCol2(ident[dimA[2]]);

	PfxMatrix3 apermRow = transpose(apermCol);

	// permute vectors

	PfxVector3 halfA_perm = apermRow * boxA.m_half;
	PfxVector3 offsetAB_perm = apermRow * offsetAB;
	PfxVector3 signsA_perm = apermRow * signsA;

	// compute the vector between the center of the box face and the sphere center

	PfxFloat signA2 = signsA_perm.getZ();
	PfxFloat scaleA2 = halfA_perm.getZ() * signA2;
	offsetAB_perm.setZ( offsetAB_perm.getZ() - scaleA2 );

	// find point on face closest to sphere center

	PfxFloat t0, t1;
	PfxFloat minDistSqr;
	PfxVector3 closestPtsVec_perm;
	PfxPoint3 localPointA_perm;

	minDistSqr = VertexBFaceATest( closestPtsVec_perm, t0, t1, PfxVector3( halfA_perm ), offsetAB_perm );

	//SCE_PFX_PRINTF("faceDimA %d dimA %d %d %d\n",faceDimA,dimA[0],dimA[1],dimA[2]);
	//SCE_PFX_PRINTF("boxA.m_half %f %f %f\n",boxA.m_half[0],boxA.m_half[1],boxA.m_half[2]);
	//SCE_PFX_PRINTF("ident %f %f %f | %f %f %f | %f %f %f\n",
	//	ident[0][0],ident[0][1],ident[0][2],
	//	ident[1][0],ident[1][1],ident[1][2],
	//	ident[2][0],ident[2][1],ident[2][2]);
	//SCE_PFX_PRINTF("apermCol %f %f %f | %f %f %f | %f %f %f\n",
	//	apermCol[0][0],apermCol[0][1],apermCol[0][2],
	//	apermCol[1][0],apermCol[1][1],apermCol[1][2],
	//	apermCol[2][0],apermCol[2][1],apermCol[2][2]);
	//SCE_PFX_PRINTF("apermRow %f %f %f | %f %f %f | %f %f %f\n",
	//	apermRow[0][0],apermRow[0][1],apermRow[0][2],
	//	apermRow[1][0],apermRow[1][1],apermRow[1][2],
	//	apermRow[2][0],apermRow[2][1],apermRow[2][2]);

	//SCE_PFX_PRINTF("closestPtsVec_perm %f %f %f\n",closestPtsVec_perm[0],closestPtsVec_perm[1],closestPtsVec_perm[2]);
	//SCE_PFX_PRINTF("halfA_perm %f %f %f\n",halfA_perm[0],halfA_perm[1],halfA_perm[2]);
	//SCE_PFX_PRINTF("offsetAB_perm %f %f %f\n",offsetAB_perm[0],offsetAB_perm[1],offsetAB_perm[2]);
	//SCE_PFX_PRINTF("t0 %f t1 %f scaleA2 %f\n",t0,t1,scaleA2);

	//SCE_PFX_PRINTF("minDistSqr %f sphereB.m_radius %f\n",minDistSqr,sphereB.m_radius);

	localPointA_perm = PfxPoint3( t0, t1, scaleA2 );

	// compute normal

	bool centerInside = ( signA2 * closestPtsVec_perm.getZ() < 0.0f );

	if ( centerInside || ( minDistSqr < lenSqrTol ) ) {
		normal = transformA * axisA;
	} else {
		PfxVector3 closestPtsVec = apermCol * closestPtsVec_perm;
		normal = transformA * ( closestPtsVec * ( 1.0f / sqrtf( minDistSqr ) ) );
	}

	// compute box point

	pointA = PfxPoint3( apermCol * PfxVector3( localPointA_perm ) );

	// compute sphere point

	pointB = PfxPoint3( transpose(transformB.getUpper3x3()) * ( -normal * sphereB.m_radius ) );

	// return distance

	//SCE_PFX_PRINTF("normal %f %f %f\n",(float)normal[0],(float)normal[1],(float)normal[2]);
	//SCE_PFX_PRINTF("pointA %f %f %f\n",(float)pointA[0],(float)pointA[1],(float)pointA[2]);
	//SCE_PFX_PRINTF("pointB %f %f %f\n",(float)pointB[0],(float)pointB[1],(float)pointB[2]);

	if ( centerInside ) {
		return -sqrtf( minDistSqr ) - sphereB.m_radius;
	} else {
		return sqrtf( minDistSqr ) - sphereB.m_radius;
	}
}

} //namespace PhysicsEffects
} //namespace sce
