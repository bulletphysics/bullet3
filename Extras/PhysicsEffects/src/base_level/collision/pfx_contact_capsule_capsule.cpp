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

#include "../../../include/physics_effects/base_level/base/pfx_vec_utils.h"
#include "../../../include/physics_effects/base_level/collision/pfx_capsule.h"
#include "pfx_contact_capsule_capsule.h"

namespace sce {
namespace PhysicsEffects {

inline
void
segmentsClosestPoints(
	PfxVector3& ptsVector,
	PfxVector3& offsetA,
	PfxVector3& offsetB,
	PfxFloat& tA, PfxFloat& tB,
	const PfxVector3 &translation,
	const PfxVector3 &dirA, PfxFloat hlenA,
	const PfxVector3 &dirB, PfxFloat hlenB )
{
	// compute the parameters of the closest points on each line segment

	PfxFloat dirA_dot_dirB = dot(dirA,dirB);
	PfxFloat dirA_dot_trans = dot(dirA,translation);
	PfxFloat dirB_dot_trans = dot(dirB,translation);

	PfxFloat denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

	if ( denom == 0.0f ) {
		tA = 0.0f;
	} else {
		tA = ( dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB ) / denom;
		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	}

	tB = tA * dirA_dot_dirB - dirB_dot_trans;

	if ( tB < -hlenB ) {
		tB = -hlenB;
		tA = tB * dirA_dot_dirB + dirA_dot_trans;

		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	} else if ( tB > hlenB ) {
		tB = hlenB;
		tA = tB * dirA_dot_dirB + dirA_dot_trans;

		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	}

	// compute the closest points relative to segment centers.

	offsetA = dirA * tA;
	offsetB = dirB * tB;

	ptsVector = translation - offsetA + offsetB;
}

inline
void
segmentsNormal( PfxVector3& normal, const PfxVector3 &ptsVector )
{
	// compute the unit direction vector between the closest points.
	// with convex objects, you want the unit direction providing the largest gap between the
	// objects when they're projected onto it.  So, if you have a few candidates covering different
	// configurations of the objects, you can compute them all, test the gaps and pick best axis
	// based on this.  Some directions might be degenerate, and the normalized() function tests for
	// degeneracy and returns an arbitrary unit vector in that case.

	PfxVector3 testDir;

	// closest points vector

	normal = pfxSafeNormalize(ptsVector);
}

PfxFloat pfxContactCapsuleCapsule(
	PfxVector3 &normal,PfxPoint3 &pointA,PfxPoint3 &pointB,
	void *shapeA,const PfxTransform3 &transformA,
	void *shapeB,const PfxTransform3 &transformB,
	PfxFloat distanceThreshold)
{
	PfxCapsule &capsuleA = *((PfxCapsule*)shapeA);
	PfxCapsule &capsuleB = *((PfxCapsule*)shapeB);

	PfxVector3 directionA = transformA.getUpper3x3().getCol0();
	PfxVector3 translationA = transformA.getTranslation();
	PfxVector3 directionB = transformB.getUpper3x3().getCol0();
	PfxVector3 translationB = transformB.getTranslation();

	// translation between centers

	PfxVector3 translation = translationB - translationA;

	// compute the closest points of the capsule line segments

	PfxVector3 ptsVector;           // the vector between the closest points
	PfxVector3 offsetA, offsetB;    // offsets from segment centers to their closest points
	PfxFloat tA, tB;              // parameters on line segment

	segmentsClosestPoints( ptsVector, offsetA, offsetB, tA, tB, translation,
						   directionA, capsuleA.m_halfLen, directionB, capsuleB.m_halfLen );

	PfxFloat distance = length(ptsVector) - capsuleA.m_radius - capsuleB.m_radius;

	if ( distance > distanceThreshold )
		return distance;

	// compute the contact normal

	segmentsNormal( normal, ptsVector );

	// compute points on capsules

	pointA = PfxPoint3( transpose(transformA.getUpper3x3()) * ( offsetA + normal * capsuleA.m_radius ) );
	pointB = PfxPoint3( transpose(transformB.getUpper3x3()) * ( offsetB - normal * capsuleB.m_radius ) );

	return distance;
}

} //namespace PhysicsEffects
} //namespace sce
