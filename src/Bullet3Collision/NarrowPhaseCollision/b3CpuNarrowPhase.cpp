#include "b3CpuNarrowPhase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"

#include "Bullet3Collision/NarrowPhaseCollision/shared/b3ConvexPolyhedronData.h"

struct b3CpuNarrowPhaseInternalData
{
	b3AlignedObjectArray<b3Aabb> m_localShapeAABBCPU;
	b3AlignedObjectArray<b3Collidable>	m_collidablesCPU;
	b3AlignedObjectArray<b3ConvexUtility*> m_convexData;
	b3Config m_config;


	b3AlignedObjectArray<b3ConvexPolyhedronData> m_convexPolyhedra;
	b3AlignedObjectArray<b3Vector3> m_uniqueEdges;
	b3AlignedObjectArray<b3Vector3> m_convexVertices;
	b3AlignedObjectArray<int> m_convexIndices;
	b3AlignedObjectArray<b3GpuFace> m_convexFaces;


	int	m_numAcceleratedShapes;
};


b3CpuNarrowPhase::b3CpuNarrowPhase(const struct b3Config& config)
{
	m_data = new b3CpuNarrowPhaseInternalData;
	m_data->m_config = config;
	m_data->m_numAcceleratedShapes = 0;
}

b3CpuNarrowPhase::~b3CpuNarrowPhase()
{
	delete m_data;
}

void b3CpuNarrowPhase::computeContacts(b3AlignedObjectArray<b3Int4>* broadphasePairs, b3AlignedObjectArray<b3Aabb>* aabbsWorldSpace)
{

}

int	b3CpuNarrowPhase::registerConvexHullShape(b3ConvexUtility* utilPtr)
{
	int collidableIndex = allocateCollidable();
	if (collidableIndex<0)
		return collidableIndex;

	
	b3Collidable& col = m_data->m_collidablesCPU[collidableIndex];
	col.m_shapeType = SHAPE_CONVEX_HULL;
	col.m_shapeIndex = -1;
	
	
	{
		b3Vector3 localCenter=b3MakeVector3(0,0,0);
		for (int i=0;i<utilPtr->m_vertices.size();i++)
			localCenter+=utilPtr->m_vertices[i];
		localCenter*= (1.f/utilPtr->m_vertices.size());
		utilPtr->m_localCenter = localCenter;

		col.m_shapeIndex = registerConvexHullShapeInternal(utilPtr,col);
	}

	if (col.m_shapeIndex>=0)
	{
		b3Aabb aabb;
		
		b3Vector3 myAabbMin=b3MakeVector3(1e30f,1e30f,1e30f);
		b3Vector3 myAabbMax=b3MakeVector3(-1e30f,-1e30f,-1e30f);

		for (int i=0;i<utilPtr->m_vertices.size();i++)
		{
			myAabbMin.setMin(utilPtr->m_vertices[i]);
			myAabbMax.setMax(utilPtr->m_vertices[i]);
		}
		aabb.m_min[0] = myAabbMin[0];
		aabb.m_min[1] = myAabbMin[1];
		aabb.m_min[2] = myAabbMin[2];
		aabb.m_minIndices[3] = 0;

		aabb.m_max[0] = myAabbMax[0];
		aabb.m_max[1] = myAabbMax[1];
		aabb.m_max[2] = myAabbMax[2];
		aabb.m_signedMaxIndices[3] = 0;

		m_data->m_localShapeAABBCPU.push_back(aabb);

	}
	
	return collidableIndex;
}

int	b3CpuNarrowPhase::allocateCollidable()
{
	int curSize = m_data->m_collidablesCPU.size();
	if (curSize<m_data->m_config.m_maxConvexShapes)
	{
		m_data->m_collidablesCPU.expand();
		return curSize;
	}
	else
	{
		b3Error("allocateCollidable out-of-range %d\n",m_data->m_config.m_maxConvexShapes);
	}
	return -1;

}

int	b3CpuNarrowPhase::registerConvexHullShape(const float* vertices, int strideInBytes, int numVertices, const float* scaling)
{
	b3AlignedObjectArray<b3Vector3> verts;

	unsigned char* vts = (unsigned char*) vertices;
	for (int i=0;i<numVertices;i++)
	{
		float* vertex = (float*) &vts[i*strideInBytes];
		verts.push_back(b3MakeVector3(vertex[0]*scaling[0],vertex[1]*scaling[1],vertex[2]*scaling[2]));
	}

	b3ConvexUtility* utilPtr = new b3ConvexUtility();
	bool merge = true;
	if (numVertices)
	{
		utilPtr->initializePolyhedralFeatures(&verts[0],verts.size(),merge);
	}

	int collidableIndex = registerConvexHullShape(utilPtr);

	delete utilPtr;
	return collidableIndex;
}


int b3CpuNarrowPhase::registerConvexHullShapeInternal(b3ConvexUtility* convexPtr,b3Collidable& col)
{

	m_data->m_convexData.resize(m_data->m_numAcceleratedShapes+1);
	m_data->m_convexPolyhedra.resize(m_data->m_numAcceleratedShapes+1);
	
    
	b3ConvexPolyhedronData& convex = m_data->m_convexPolyhedra.at(m_data->m_convexPolyhedra.size()-1);
	convex.mC = convexPtr->mC;
	convex.mE = convexPtr->mE;
	convex.m_extents= convexPtr->m_extents;
	convex.m_localCenter = convexPtr->m_localCenter;
	convex.m_radius = convexPtr->m_radius;
	
	convex.m_numUniqueEdges = convexPtr->m_uniqueEdges.size();
	int edgeOffset = m_data->m_uniqueEdges.size();
	convex.m_uniqueEdgesOffset = edgeOffset;
	
	m_data->m_uniqueEdges.resize(edgeOffset+convex.m_numUniqueEdges);
    
	//convex data here
	int i;
	for ( i=0;i<convexPtr->m_uniqueEdges.size();i++)
	{
		m_data->m_uniqueEdges[edgeOffset+i] = convexPtr->m_uniqueEdges[i];
	}
    
	int faceOffset = m_data->m_convexFaces.size();
	convex.m_faceOffset = faceOffset;
	convex.m_numFaces = convexPtr->m_faces.size();

	m_data->m_convexFaces.resize(faceOffset+convex.m_numFaces);
	

	for (i=0;i<convexPtr->m_faces.size();i++)
	{
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane = b3MakeVector3(convexPtr->m_faces[i].m_plane[0],
																			convexPtr->m_faces[i].m_plane[1],
																			convexPtr->m_faces[i].m_plane[2],
																			convexPtr->m_faces[i].m_plane[3]);

		
		int indexOffset = m_data->m_convexIndices.size();
		int numIndices = convexPtr->m_faces[i].m_indices.size();
		m_data->m_convexFaces[convex.m_faceOffset+i].m_numIndices = numIndices;
		m_data->m_convexFaces[convex.m_faceOffset+i].m_indexOffset = indexOffset;
		m_data->m_convexIndices.resize(indexOffset+numIndices);
		for (int p=0;p<numIndices;p++)
		{
			m_data->m_convexIndices[indexOffset+p] = convexPtr->m_faces[i].m_indices[p];
		}
	}
    
	convex.m_numVertices = convexPtr->m_vertices.size();
	int vertexOffset = m_data->m_convexVertices.size();
	convex.m_vertexOffset =vertexOffset;
	
	m_data->m_convexVertices.resize(vertexOffset+convex.m_numVertices);
	for (int i=0;i<convexPtr->m_vertices.size();i++)
	{
		m_data->m_convexVertices[vertexOffset+i] = convexPtr->m_vertices[i];
	}

	(m_data->m_convexData)[m_data->m_numAcceleratedShapes] = convexPtr;
	
    
    
	return m_data->m_numAcceleratedShapes++;
}

const b3Aabb& b3CpuNarrowPhase::getLocalSpaceAabb(int collidableIndex) const
{
	return m_data->m_localShapeAABBCPU[collidableIndex];
}
