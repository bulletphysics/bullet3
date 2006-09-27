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

#ifndef OBB_TRIANGLE_MINKOWSKI_H
#define OBB_TRIANGLE_MINKOWSKI_H

#include "btConvexShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

class btTriangleShape : public btPolyhedralConvexShape
{


public:

	btVector3	m_vertices1[3];


	virtual int GetNumVertices() const
	{
		return 3;
	}

	const btVector3& GetVertexPtr(int index) const
	{
		return m_vertices1[index];
	}
	virtual void GetVertex(int index,btVector3& vert) const
	{
		vert = m_vertices1[index];
	}
	virtual int	GetShapeType() const
	{
		return TRIANGLE_SHAPE_PROXYTYPE;
	}

	virtual int GetNumEdges() const
	{
		return 3;
	}
	
	virtual void GetEdge(int i,btPoint3& pa,btPoint3& pb) const
	{
		GetVertex(i,pa);
		GetVertex((i+1)%3,pb);
	}

	virtual void GetAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax)const 
	{
//		ASSERT(0);
		GetAabbSlow(t,aabbMin,aabbMax);
	}

	btVector3 LocalGetSupportingVertexWithoutMargin(const btVector3& dir)const 
	{
		btVector3 dots(dir.dot(m_vertices1[0]), dir.dot(m_vertices1[1]), dir.dot(m_vertices1[2]));
	  	return m_vertices1[dots.maxAxis()];

	}

	virtual void	BatchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const
	{
		for (int i=0;i<numVectors;i++)
		{
			const btVector3& dir = vectors[i];
			btVector3 dots(dir.dot(m_vertices1[0]), dir.dot(m_vertices1[1]), dir.dot(m_vertices1[2]));
  			supportVerticesOut[i] = m_vertices1[dots.maxAxis()];
		}

	}



	btTriangleShape(const btVector3& p0,const btVector3& p1,const btVector3& p2)
	{
		m_vertices1[0] = p0;
		m_vertices1[1] = p1;
		m_vertices1[2] = p2;
	}

	

	virtual void GetPlane(btVector3& planeNormal,btPoint3& planeSupport,int i) const
	{
		GetPlaneEquation(i,planeNormal,planeSupport);
	}

	virtual int	GetNumPlanes() const
	{
		return 1;
	}

	void CalcNormal(btVector3& normal) const
	{
		normal = (m_vertices1[1]-m_vertices1[0]).cross(m_vertices1[2]-m_vertices1[0]);
		normal.normalize();
	}

	virtual void GetPlaneEquation(int i, btVector3& planeNormal,btPoint3& planeSupport) const
	{
		CalcNormal(planeNormal);
		planeSupport = m_vertices1[0];
	}

	virtual void	CalculateLocalInertia(btScalar mass,btVector3& inertia)
	{
		ASSERT(0);
		inertia.setValue(0.f,0.f,0.f);
	}

		virtual	bool IsInside(const btPoint3& pt,btScalar tolerance) const
	{
		btVector3 normal;
		CalcNormal(normal);
		//distance to plane
		btScalar dist = pt.dot(normal);
		btScalar planeconst = m_vertices1[0].dot(normal);
		dist -= planeconst;
		if (dist >= -tolerance && dist <= tolerance)
		{
			//inside check on edge-planes
			int i;
			for (i=0;i<3;i++)
			{
				btPoint3 pa,pb;
				GetEdge(i,pa,pb);
				btVector3 edge = pb-pa;
				btVector3 edgeNormal = edge.cross(normal);
				edgeNormal.normalize();
				btScalar dist = pt.dot( edgeNormal);
				btScalar edgeConst = pa.dot(edgeNormal);
				dist -= edgeConst;
				if (dist < -tolerance)
					return false;
			}
			
			return true;
		}

		return false;
	}
		//debugging
		virtual char*	GetName()const
		{
			return "Triangle";
		}


};

#endif //OBB_TRIANGLE_MINKOWSKI_H

