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

#ifndef COMPOUND_SHAPE_H
#define COMPOUND_SHAPE_H

#include "btCollisionShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include <vector>
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

class btOptimizedBvh;

/// btCompoundShape allows to store multiple other btCollisionShapes
/// This allows for concave collision objects. This is more general then the Static Concave btTriangleMeshShape.
class btCompoundShape	: public btCollisionShape
{
	std::vector<btTransform>		m_childTransforms;
	std::vector<btCollisionShape*>	m_childShapes;
	btVector3						m_localAabbMin;
	btVector3						m_localAabbMax;

	btOptimizedBvh*					m_aabbTree;

public:
	btCompoundShape();

	virtual ~btCompoundShape();

	void	AddChildShape(const btTransform& localTransform,btCollisionShape* shape);

	int		GetNumChildShapes() const
	{
		return m_childShapes.size();
	}

	btCollisionShape* GetChildShape(int index)
	{
		return m_childShapes[index];
	}
	const btCollisionShape* GetChildShape(int index) const
	{
		return m_childShapes[index];
	}

	btTransform	GetChildTransform(int index)
	{
		return m_childTransforms[index];
	}
	const btTransform	GetChildTransform(int index) const
	{
		return m_childTransforms[index];
	}

	///GetAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void GetAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;


	virtual void	setLocalScaling(const btVector3& scaling)
	{
		m_localScaling = scaling;
	}
	virtual const btVector3& getLocalScaling() const 
	{
		return m_localScaling;
	}

	virtual void	CalculateLocalInertia(btScalar mass,btVector3& inertia);
	
	virtual int	GetShapeType() const { return COMPOUND_SHAPE_PROXYTYPE;}

	virtual void	SetMargin(float margin)
	{
		m_collisionMargin = margin;
	}
	virtual float	GetMargin() const
	{
		return m_collisionMargin;
	}
	virtual char*	GetName()const
	{
		return "Compound";
	}

	//this is optional, but should make collision queries faster, by culling non-overlapping nodes
	void	CreateAabbTreeFromChildren();

	const btOptimizedBvh*					GetAabbTree() const
	{
		return m_aabbTree;
	}

private:
	btScalar	m_collisionMargin;
protected:
	btVector3	m_localScaling;

};



#endif //COMPOUND_SHAPE_H
