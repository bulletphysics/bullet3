/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B3_REFERENCEFRAMEHELPER_H
#define B3_REFERENCEFRAMEHELPER_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

class b3ReferenceFrameHelper {
public:
	static btVector3 getPointWorldToLocal(const btTransform& localObjectCenterOfMassTransform, const btVector3& point) {
		  return localObjectCenterOfMassTransform.inverse() * point; // transforms the point from the world frame into the local frame
	}

	static btVector3 getPointLocalToWorld(const btTransform& localObjectCenterOfMassTransform,const btVector3& point) {
		  return localObjectCenterOfMassTransform * point; // transforms the point from the world frame into the local frame
	}

	static btVector3 getAxisWorldToLocal(const btTransform& localObjectCenterOfMassTransform, const btVector3& axis) {
	  btTransform local1 = localObjectCenterOfMassTransform.inverse(); // transforms the axis from the local frame into the world frame
	  btVector3 zero(0,0,0);
	  local1.setOrigin(zero);
	  return local1 * axis;
	}

	static btVector3 getAxisLocalToWorld(const btTransform& localObjectCenterOfMassTransform, const btVector3& axis) {
	  btTransform local1 = localObjectCenterOfMassTransform; // transforms the axis from the local frame into the world frame
	  btVector3 zero(0,0,0);
	  local1.setOrigin(zero);
	  return local1 * axis;
	}

	static btTransform getTransformWorldToLocal(const btTransform& localObjectCenterOfMassTransform, const btTransform& transform) {
	  return localObjectCenterOfMassTransform.inverse() * transform; // transforms the axis from the local frame into the world frame
	}

	static btTransform getTransformLocalToWorld(const btTransform& localObjectCenterOfMassTransform, const btTransform& transform) {
	  return localObjectCenterOfMassTransform * transform; // transforms the axis from the local frame into the world frame
	}

};

#endif /* B3_REFERENCEFRAMEHELPER_H */
