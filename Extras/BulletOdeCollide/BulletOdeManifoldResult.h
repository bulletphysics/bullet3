/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BULLET_ODE_MANIFOLD_RESULT_H
#define BULLET_ODE_MANIFOLD_RESULT_H

#include "NarrowPhaseCollision/DiscreteCollisionDetectorInterface.h"
class PersistentManifold;
#include <ode/common.h>

class BulletOdeManifoldResult : public DiscreteCollisionDetectorInterface::Result
{
	PersistentManifold* m_manifoldPtr;
	dGeomID m_geom1;
	dGeomID m_geom2;

public:

	BulletOdeManifoldResult(dGeomID geom1,dGeomID geom2,PersistentManifold* manifoldPtr);

	virtual void AddContactPoint(const SimdVector3& normalOnBInWorld,const SimdVector3& pointInWorld,float depth);

};

#endif //BULLET_ODE_MANIFOLD_RESULT_H
