/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Experimental Buoyancy fluid demo written by John McCutchan
*/
#ifndef BT_HF_FLUID_COLLISION_SHAPE_H
#define BT_HF_FLUID_COLLISION_SHAPE_H

#include "btHfFluid.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"

class btHfFluidCollisionShape : public btConcaveShape
{
	public:
	btHfFluid*						m_fluid;
	
	btHfFluidCollisionShape(btHfFluid* backptr) : btConcaveShape ()
	{
		m_shapeType = HFFLUID_SHAPE_PROXYTYPE;
		m_fluid=backptr;
	}

	virtual ~btHfFluidCollisionShape()
	{

	}

	void	processAllTriangles(btTriangleCallback* /*callback*/,const btVector3& /*aabbMin*/,const btVector3& /*aabbMax*/) const
	{
		//not yet
		btAssert(0);
	}

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
	{
		/* t should be identity, but better be safe than...fast? */ 
		btVector3	mins;
		btVector3	maxs;

		m_fluid->getAabb (mins, maxs);

		const btVector3	crns[]={t*btVector3(mins.x(),mins.y(),mins.z()),
								t*btVector3(maxs.x(),mins.y(),mins.z()),
								t*btVector3(maxs.x(),maxs.y(),mins.z()),
								t*btVector3(mins.x(),maxs.y(),mins.z()),
								t*btVector3(mins.x(),mins.y(),maxs.z()),
								t*btVector3(maxs.x(),mins.y(),maxs.z()),
								t*btVector3(maxs.x(),maxs.y(),maxs.z()),
								t*btVector3(mins.x(),maxs.y(),maxs.z())};
		aabbMin=aabbMax=crns[0];
		for(int i=1;i<8;++i)
		{
			aabbMin.setMin(crns[i]);
			aabbMax.setMax(crns[i]);
		}
	}

	virtual void	setLocalScaling(const btVector3& /*scaling*/)
	{		
		///na
		btAssert(0);
	}
	virtual const btVector3& getLocalScaling() const
	{
		static const btVector3 dummy(1,1,1);
		return dummy;
	}
	virtual void	calculateLocalInertia(btScalar /*mass*/,btVector3& /*inertia*/) const
	{
		///not yet
		btAssert(0);
	}
	virtual const char*	getName()const
	{
		return "HfFluid";
	}
};

#endif
