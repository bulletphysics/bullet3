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

#ifndef MINKOWSKI_SUM_SHAPE_H
#define MINKOWSKI_SUM_SHAPE_H

#include "btConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

/// btMinkowskiSumShape represents implicit (getSupportingVertex) based minkowski sum of two convex implicit shapes.
class btMinkowskiSumShape : public btConvexShape
{

	btTransform	m_transA;
	btTransform	m_transB;
	btConvexShape*	m_shapeA;
	btConvexShape*	m_shapeB;

public:

	btMinkowskiSumShape(btConvexShape* shapeA,btConvexShape* shapeB);

	virtual btVector3	LocalGetSupportingVertexWithoutMargin(const btVector3& vec)const;

	virtual void	BatchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;


	virtual void	CalculateLocalInertia(btScalar mass,btVector3& inertia);

	void	SetTransformA(const btTransform&	transA) { m_transA = transA;}
	void	SetTransformB(const btTransform&	transB) { m_transB = transB;}

	const btTransform& GetTransformA()const  { return m_transA;}
	const btTransform& GetTransformB()const  { return m_transB;}


	virtual int	GetShapeType() const { return MINKOWSKI_SUM_SHAPE_PROXYTYPE; }

	virtual float	GetMargin() const;

	const btConvexShape*	GetShapeA() const { return m_shapeA;}
	const btConvexShape*	GetShapeB() const { return m_shapeB;}

	virtual char*	GetName()const 
	{
		return "MinkowskiSum";
	}
};

#endif //MINKOWSKI_SUM_SHAPE_H
