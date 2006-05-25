
#ifndef BVH_TRIMESH_SHAPE_H
#define BVH_TRIMESH_SHAPE_H

#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include <../ode/src/collision_util.h>
#include <../ode/src/collision_kernel.h>
#include <../ode/src/collision_kernel.h>
#include "CollisionShapes/TriangleMeshShape.h"
struct dxTriMesh;

///if this USE_AABB_TREE is not defined, it will brute force go through all triangles
#define USE_AABB_TREE


class TrimeshBridge : public StridingMeshInterface
{

	public:

	dxTriMesh* m_trimesh;

	TrimeshBridge(dxTriMesh* trimesh);
	
	virtual void	getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart=0);

	virtual void	getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,const unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart=0) const;

	/// unLockVertexBase finishes the access to a subpart of the triangle mesh
	/// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
	virtual void	unLockVertexBase(int subpart) {}

	virtual void	unLockReadOnlyVertexBase(int subpart) const {}

	/// getNumSubParts returns the number of seperate subparts
	/// each subpart has a continuous array of vertices and indices
	virtual int		getNumSubParts() const { return 1;}
	
	virtual void	preallocateVertices(int numverts){}
	virtual void	preallocateIndices(int numindices){}

};


/// BVHTrimeshShape bridges the Opcode to provide backwards compatibility with dxTriMesh.
/// You can also avoid using Opcode and use Bullet trimesh support See: BvhTriangleMeshShape
class BVHTrimeshShape : public TriangleMeshShape 
{ 
   TrimeshBridge * m_triangleBridge; 

public:
	BVHTrimeshShape(dxTriMesh* trimesh);
	virtual ~BVHTrimeshShape ();


#ifdef	USE_AABB_TREE
	//ProcessAllTriangles first gets the overlapping triangles using BVH culling
	//and passes them on to the TriangleCallback
	void	ProcessAllTriangles(TriangleCallback* callback,const SimdVector3& aabbMin,const SimdVector3& aabbMax) const;
#endif

};


#endif //BVH_TRIMESH_SHAPE_H