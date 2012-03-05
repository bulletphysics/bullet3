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

#include "../../../include/physics_effects/base_level/collision/pfx_sphere.h"
#include "pfx_contact_sphere_sphere.h"

namespace sce {
namespace PhysicsEffects {

const PfxFloat lenSqrTol = 1.0e-30f;

PfxFloat pfxContactSphereSphere(
	PfxVector3 &normal,PfxPoint3 &pointA,PfxPoint3 &pointB,
	void *shapeA,const PfxTransform3 &transformA,
	void *shapeB,const PfxTransform3 &transformB,
	PfxFloat distanceThreshold)
{
	PfxSphere &sphereA = *((PfxSphere*)shapeA);
	PfxSphere &sphereB = *((PfxSphere*)shapeB);

	PfxVector3 direction(0.0f);

	PfxVector3 translationA = transformA.getTranslation();
	PfxVector3 translationB = transformB.getTranslation();

	// get the offset vector between sphere centers

	PfxVector3 offsetAB;

	offsetAB = translationB - translationA;

	// normalize the offset to compute the direction vector

	PfxFloat distSqr = dot(offsetAB,offsetAB);
	PfxFloat dist = sqrtf(distSqr);
	PfxFloat sphereDist = dist - sphereA.m_radius - sphereB.m_radius;

	if ( sphereDist > distanceThreshold ) {
		return sphereDist;
	}

	if ( distSqr > lenSqrTol ) {
		PfxFloat distInv = 1.0f / dist;

		direction = offsetAB * distInv;
	} else {
		direction = PfxVector3(0.0f, 0.0f, 1.0f);
	}

	normal = direction;

	// compute the points on the spheres, in world space

	pointA = PfxPoint3( transpose(transformA.getUpper3x3()) * ( direction * sphereA.m_radius ) );
	pointB = PfxPoint3( transpose(transformB.getUpper3x3()) * ( -direction * sphereB.m_radius ) );

	// return the distance between the spheres

	return sphereDist;
}

} //namespace PhysicsEffects
} //namespace sce
