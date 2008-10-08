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

#ifndef BT_CONVEX_POINT_CLOUD_SHAPE_H
#define BT_CONVEX_POINT_CLOUD_SHAPE_H

#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btAlignedObjectArray.h"

///The btConvexPointCloudShape implements an implicit convex hull of an array of vertices.
ATTRIBUTE_ALIGNED16(class) btConvexPointCloudShape : public btPolyhedralConvexShape
{
	btVector3* m_points;
	int m_numPoints;
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btConvexPointCloudShape(btVector3* points,int numPoints, bool computeAabb = true)
	{
		m_shapeType = CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
		m_points = points;
		m_numPoints = numPoints;

		if (computeAabb)
			recalcLocalAabb();
	}

	void setPoints (btVector3* points, int numPoints, bool computeAabb = true)
	{
		m_points = points;
		m_numPoints = numPoints;

		if (computeAabb)
			recalcLocalAabb();
	}

	btPoint3* getPoints()
	{
		return m_points;
	}

	const btPoint3* getPoints() const
	{
		return m_points;
	}

	int getNumPoints() const 
	{
		return m_numPoints;
	}

#ifndef __SPU__
	virtual btVector3	localGetSupportingVertex(const btVector3& vec)const;
	virtual btVector3	localGetSupportingVertexWithoutMargin(const btVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
#endif


	//debugging
	virtual const char*	getName()const {return "ConvexPointCloud";}

	virtual int	getNumVertices() const;
	virtual int getNumEdges() const;
	virtual void getEdge(int i,btPoint3& pa,btPoint3& pb) const;
	virtual void getVertex(int i,btPoint3& vtx) const;
	virtual int	getNumPlanes() const;
	virtual void getPlane(btVector3& planeNormal,btPoint3& planeSupport,int i ) const;
	virtual	bool isInside(const btPoint3& pt,btScalar tolerance) const;

	///in case we receive negative scaling
	virtual void	setLocalScaling(const btVector3& scaling);
};


#endif //BT_CONVEX_POINT_CLOUD_SHAPE_H

