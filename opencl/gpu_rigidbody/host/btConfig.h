#ifndef BT_CONFIG_H
#define BT_CONFIG_H

struct	btConfig
{
	int	m_maxConvexBodies;
	int	m_maxConvexShapes;
	int	m_maxBroadphasePairs;
	int m_maxContactCapacity;

	int m_maxVerticesPerFace;
	int m_maxFacesPerShape;
	int	m_maxConvexVertices;
	int m_maxConvexIndices;
	int m_maxConvexUniqueEdges;
	
	int	m_maxCompoundChildShapes;
	
	int m_maxTriConvexPairCapacity;

	btConfig()
		:m_maxConvexBodies(32*1024),
		m_maxConvexShapes(8192),
		m_maxVerticesPerFace(64),
		m_maxFacesPerShape(64),
		m_maxConvexVertices(8192),
		m_maxConvexIndices(8192),
		m_maxConvexUniqueEdges(8192),
		m_maxCompoundChildShapes(8192),
		m_maxTriConvexPairCapacity(512*1024)
	{
		m_maxBroadphasePairs = 16*m_maxConvexBodies;
		m_maxContactCapacity = m_maxBroadphasePairs;
	}
};


#endif//BT_CONFIG_H

