/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef SPHERE_SPHERE_COLLISION_H
#define SPHERE_SPHERE_COLLISION_H

#include "LinearMath/btTransform.h"  // Note that btVector3 might be double precision...
#include "btDistanceInfo.h"

struct btSphereSphereCollisionDescription
{
	btTransform m_sphereTransformA;
	btTransform m_sphereTransformB;
	btScalar m_radiusA;
	btScalar m_radiusB;
};

///compute the distance between two spheres, where the distance is zero when the spheres are touching
///positive distance means the spheres are separate and negative distance means penetration
///point A and pointB are witness points, and normalOnB points from sphere B to sphere A
inline int btComputeSphereSphereCollision(const btSphereSphereCollisionDescription& input, btDistanceInfo* distInfo)
{
	btVector3 diff = input.m_sphereTransformA.getOrigin() - input.m_sphereTransformB.getOrigin();
	btScalar len = diff.length();
	btScalar radiusA = input.m_radiusA;
	btScalar radiusB = input.m_radiusB;

	///distance (negative means penetration)
	btScalar dist = len - (radiusA + radiusB);
	btVector3 normalOnSurfaceB(1, 0, 0);
	if (len > SIMD_EPSILON)
	{
		normalOnSurfaceB = diff / len;
	}
	distInfo->m_distance = dist;
	distInfo->m_normalBtoA = normalOnSurfaceB;
	distInfo->m_pointOnA = input.m_sphereTransformA.getOrigin() - input.m_radiusA * normalOnSurfaceB;
	distInfo->m_pointOnB = input.m_sphereTransformB.getOrigin() + input.m_radiusB * normalOnSurfaceB;
	return 0;  //sphere-sphere cannot fail
}

#endif  //SPHERE_SPHERE_COLLISION_H
