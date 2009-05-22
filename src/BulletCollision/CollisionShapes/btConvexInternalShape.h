/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONVEX_INTERNAL_SHAPE_H
#define BT_CONVEX_INTERNAL_SHAPE_H

#include "btConvexShape.h"
#include "LinearMath/btAabbUtil2.h"

///The btConvexInternalShape is an internal base class, shared by most convex shape implementations.
class btConvexInternalShape : public btConvexShape
{

	protected:

	//local scaling. collisionMargin is not scaled !
	btVector3	m_localScaling;

	btVector3	m_implicitShapeDimensions;
	
	btScalar	m_collisionMargin;

	btScalar	m_padding;

	btConvexInternalShape();

public:

	

	virtual ~btConvexInternalShape()
	{

	}

	virtual btVector3	localGetSupportingVertex(const btVector3& vec)const;

	const btVector3& getImplicitShapeDimensions() const
	{
		return m_implicitShapeDimensions;
	}

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
	{
		getAabbSlow(t,aabbMin,aabbMax);
	}


	
	virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;


	virtual void	setLocalScaling(const btVector3& scaling);
	virtual const btVector3& getLocalScaling() const 
	{
		return m_localScaling;
	}

	const btVector3& getLocalScalingNV() const 
	{
		return m_localScaling;
	}

	virtual void	setMargin(btScalar margin)
	{
		m_collisionMargin = margin;
	}
	virtual btScalar	getMargin() const
	{
		return m_collisionMargin;
	}

	btScalar	getMarginNV() const
	{
		return m_collisionMargin;
	}

	virtual int		getNumPreferredPenetrationDirections() const
	{
		return 0;
	}
	
	virtual void	getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const
	{
		(void)penetrationVector;
		(void)index;
		btAssert(0);
	}


	
};


///btConvexInternalAabbCachingShape adds local aabb caching for convex shapes, to avoid expensive bounding box calculations
class btConvexInternalAabbCachingShape : public btConvexInternalShape
{
	btVector3	m_localAabbMin;
	btVector3	m_localAabbMax;
	bool		m_isLocalAabbValid;
	
protected:
					
	btConvexInternalAabbCachingShape();
	
	void setCachedLocalAabb (const btVector3& aabbMin, const btVector3& aabbMax)
	{
		m_isLocalAabbValid = true;
		m_localAabbMin = aabbMin;
		m_localAabbMax = aabbMax;
	}

	inline void getCachedLocalAabb (btVector3& aabbMin, btVector3& aabbMax) const
	{
		btAssert(m_isLocalAabbValid);
		aabbMin = m_localAabbMin;
		aabbMax = m_localAabbMax;
	}

	inline void getNonvirtualAabb(const btTransform& trans,btVector3& aabbMin,btVector3& aabbMax, btScalar margin) const
	{

		//lazy evaluation of local aabb
		btAssert(m_isLocalAabbValid);
		btTransformAabb(m_localAabbMin,m_localAabbMax,margin,trans,aabbMin,aabbMax);
	}
		
public:
		
	virtual void	setLocalScaling(const btVector3& scaling);

	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

	void	recalcLocalAabb();

};

#endif //BT_CONVEX_INTERNAL_SHAPE_H
