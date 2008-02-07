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

#ifndef BT_CAPSULE_SHAPE_H
#define BT_CAPSULE_SHAPE_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types


///btCapsuleShape represents a capsule around the Y axis
///A more general solution that can represent capsules is the btMultiSphereShape
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class btCapsuleShape : public btConvexInternalShape
{
protected:
	///only used for btCapsuleShapeZ and btCapsuleShapeX subclasses.
	btCapsuleShape() {};

public:
	btCapsuleShape(btScalar radius,btScalar height);

	///CollisionShape Interface
	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const;

	/// btConvexShape Interface
	virtual btVector3	localGetSupportingVertexWithoutMargin(const btVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
	
	virtual int	getShapeType() const { return CAPSULE_SHAPE_PROXYTYPE; }

	virtual const char*	getName()const 
	{
		return "CapsuleShape";
	}

	virtual int	getUpAxis() const
	{
		return 1;
	}

	virtual btScalar	getRadius() const
	{
		return m_implicitShapeDimensions.getX();
	}

	virtual btScalar	getHalfHeight() const
	{
		return m_implicitShapeDimensions.getY();
	}

};

///btCapsuleShapeX represents a capsule around the Z axis
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class btCapsuleShapeX : public btCapsuleShape
{
public:

	btCapsuleShapeX(btScalar radius,btScalar height);
	
	virtual int	getUpAxis() const
	{
		return 0;
	}

		//debugging
	virtual const char*	getName()const
	{
		return "CapsuleX";
	}

	virtual btScalar getRadius() const
	{
		return m_implicitShapeDimensions.getY();
	}

	virtual btScalar	getHalfHeight() const
	{
		return m_implicitShapeDimensions.getX();
	}

};

///btCapsuleShapeZ represents a capsule around the Z axis
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class btCapsuleShapeZ : public btCapsuleShape
{
public:
	btCapsuleShapeZ(btScalar radius,btScalar height);

	virtual int	getUpAxis() const
	{
		return 2;
	}
		//debugging
	virtual const char*	getName()const
	{
		return "CapsuleZ";
	}

	virtual btScalar getRadius() const
	{
		return m_implicitShapeDimensions.getX();
	}

	virtual btScalar	getHalfHeight() const
	{
		return m_implicitShapeDimensions.getZ();
	}
};



#endif //BT_CAPSULE_SHAPE_H
