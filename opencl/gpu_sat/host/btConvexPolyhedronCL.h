#ifndef CONVEX_POLYHEDRON_CL
#define CONVEX_POLYHEDRON_CL

#include "parallel_primitives/host/btTransform.h"

struct btGpuFace
{
	btVector4 m_plane;
	int m_indexOffset;
	int m_numIndices;
};

ATTRIBUTE_ALIGNED16(struct) btConvexPolyhedronCL
{
	btVector3		m_localCenter;
	btVector3		m_extents;
	btVector3		mC;
	btVector3		mE;

	btScalar		m_radius;
	int	m_faceOffset;
	int m_numFaces;
	int	m_numVertices;

	int m_vertexOffset;
	int	m_uniqueEdgesOffset;
	int	m_numUniqueEdges;
	int m_unused;	
	


	inline void project(const btTransform& trans, const btVector3& dir, const btAlignedObjectArray<btVector3>& vertices, btScalar& min, btScalar& max) const
	{
		min = FLT_MAX;
		max = -FLT_MAX;
		int numVerts = m_numVertices;

		const btVector3 localDir = trans.getBasis().transpose()*dir;
		const btVector3 localDi2 = quatRotate(trans.getRotation().inverse(),dir);
		
		btScalar offset = trans.getOrigin().dot(dir);

		for(int i=0;i<numVerts;i++)
		{
			//btVector3 pt = trans * vertices[m_vertexOffset+i];
			//btScalar dp = pt.dot(dir);
			btScalar dp = vertices[m_vertexOffset+i].dot(localDir);
			//btAssert(dp==dpL);
			if(dp < min)	min = dp;
			if(dp > max)	max = dp;
		}
		if(min>max)
		{
			btScalar tmp = min;
			min = max;
			max = tmp;
		}
		min += offset;
		max += offset;
	}

};

#endif //CONVEX_POLYHEDRON_CL