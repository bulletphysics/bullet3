
#include "BVHTrimeshShape.h"

#define TRIMESH_INTERNAL //requires OPCODE for the AABB tree
#include <../ode/src/collision_trimesh_internal.h>

//Thanks AndrewONeil for the TrimeshBridge fix

BVHTrimeshShape::BVHTrimeshShape(dxTriMesh* trimesh) 
: TriangleMeshShape(new TrimeshBridge(trimesh)) 
, m_triangleBridge( static_cast<TrimeshBridge*>(m_meshInterface)) 
{ 
} 

BVHTrimeshShape::~BVHTrimeshShape () 
{ 
   delete m_triangleBridge; 
} 





TrimeshBridge::TrimeshBridge(dxTriMesh* trimesh)
:m_trimesh(trimesh)
{
}

void	TrimeshBridge::getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,const unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart) const
{
	numverts = m_trimesh->Data->Mesh.GetNbVertices();
	(*vertexbase) = (unsigned char *)m_trimesh->Data->Mesh.GetVerts();
	type = PHY_FLOAT;
	stride = m_trimesh->Data->Mesh.GetVertexStride();

	numfaces = m_trimesh->Data->Mesh.GetNbTriangles();
	(*indexbase) = (unsigned char *)m_trimesh->Data->Mesh.GetTris();
	indexstride = m_trimesh->Data->Mesh.GetTriStride();
	indicestype = PHY_INTEGER;
}

void	TrimeshBridge::getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart)
{

	numverts = m_trimesh->Data->Mesh.GetNbVertices();
	(*vertexbase) = (unsigned char *)m_trimesh->Data->Mesh.GetVerts();
	type = PHY_FLOAT;
	stride = m_trimesh->Data->Mesh.GetVertexStride();

	numfaces = m_trimesh->Data->Mesh.GetNbTriangles();
	(*indexbase) = (unsigned char *)m_trimesh->Data->Mesh.GetTris();
	indexstride = m_trimesh->Data->Mesh.GetTriStride();
	indicestype = PHY_INTEGER;
	
}


#ifdef USE_AABB_TREE

	//ProcessAllTriangles first gets the overlapping triangles using BVH culling
	//and passes them on to the TriangleCallback
void	BVHTrimeshShape::ProcessAllTriangles(TriangleCallback* callback,const SimdVector3& aabbMin,const SimdVector3& aabbMax) const
{
	dxTriMesh* TriMesh = m_triangleBridge->m_trimesh;

	// Init
	const dVector3& TLPosition = *(const dVector3*)dGeomGetPosition(TriMesh);
	const dMatrix3& TLRotation = *(const dMatrix3*)dGeomGetRotation(TriMesh);

	SphereCollider& Collider = TriMesh->_SphereCollider;

	
	SimdVector3 he = (aabbMax-aabbMin)*0.5f;
	SimdVector3 cen = (aabbMax+aabbMin)*0.5f;


	dVector3 aabbHalfExtents;
	aabbHalfExtents[0] = he.x();
	aabbHalfExtents[1] = he.y();
	aabbHalfExtents[2] = he.z();

	dVector3 Position;
	Position[0]=cen.x();
	Position[1]=cen.y();
	Position[2]=cen.z();
	
	dReal Radius = he.length();

	// Bounding Sphere (from aabb)
	Sphere Sphere;
	Sphere.mCenter.x = Position[0];
	Sphere.mCenter.y = Position[1];
	Sphere.mCenter.z = Position[2];
	Sphere.mRadius = Radius;

	Matrix4x4 trimeshTransform;
	MakeMatrix(TLPosition, TLRotation, trimeshTransform);

//	bvhTraversal.getOverlappingPrimitiveIndices(
//		indicesCache,
//		Sphere, 
//		TriMesh->Data->BVTree,
//		&trimeshTransform);

	Collider.SetTemporalCoherence(false);
	Collider.SetPrimitiveTests(false);
	Collider.Collide(dxTriMesh::defaultSphereCache, Sphere, TriMesh->Data->BVTree, null, 
					 &trimeshTransform);

	// get results
	int TriCount = Collider.GetNbTouchedPrimitives();
	const int* Triangles = (const int*)Collider.GetTouchedPrimitives();

	if (TriCount != 0){
		
		int OutTriCount = 0;
		for (int i = 0; i < TriCount; i++){
			//what was this
			//if (OutTriCount == (Flags & 0xffff)){
			//	break;
			//}

			const int& TriIndex = Triangles[i];

			dVector3 dv[3];
			FetchTriangle(TriMesh, TriIndex, TLPosition, TLRotation, dv);
	
			SimdVector3 vts[3] = 
			{	SimdVector3(dv[0][0],dv[0][1],dv[0][2]),
				SimdVector3 (dv[1][0],dv[1][1],dv[1][2]),
				SimdVector3 (dv[2][0],dv[2][1],dv[2][2])
			};

			callback->ProcessTriangle(&vts[0]);

			OutTriCount++;
		}
		if (OutTriCount)
		{

		}
	}


}
#endif //USE_AABB_TREE

