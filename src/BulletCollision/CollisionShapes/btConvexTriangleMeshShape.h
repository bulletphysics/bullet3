#ifndef CONVEX_TRIANGLEMESH_SHAPE_H
#define CONVEX_TRIANGLEMESH_SHAPE_H


#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

#include <vector>

/// btConvexTriangleMeshShape is a convex hull of a triangle mesh. If you just have a point cloud, you can use btConvexHullShape instead.
/// It uses the btStridingMeshInterface instead of a point cloud. This can avoid the duplication of the triangle mesh data.
class btConvexTriangleMeshShape : public btPolyhedralConvexShape
{

	class btStridingMeshInterface*	m_stridingMesh;

public:
	btConvexTriangleMeshShape(btStridingMeshInterface* meshInterface);

	class btStridingMeshInterface*	GetStridingMesh()
	{
		return m_stridingMesh;
	}
	
	virtual btVector3	LocalGetSupportingVertex(const btVector3& vec)const;
	virtual btVector3	LocalGetSupportingVertexWithoutMargin(const btVector3& vec)const;
	virtual void	BatchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
	
	virtual int	GetShapeType()const { return CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE; }

	//debugging
	virtual char*	GetName()const {return "ConvexTrimesh";}
	
	virtual int	GetNumVertices() const;
	virtual int GetNumEdges() const;
	virtual void GetEdge(int i,btPoint3& pa,btPoint3& pb) const;
	virtual void GetVertex(int i,btPoint3& vtx) const;
	virtual int	GetNumPlanes() const;
	virtual void GetPlane(btVector3& planeNormal,btPoint3& planeSupport,int i ) const;
	virtual	bool IsInside(const btPoint3& pt,btScalar tolerance) const;

	
	void	setLocalScaling(const btVector3& scaling);

};



#endif //CONVEX_TRIANGLEMESH_SHAPE_H


