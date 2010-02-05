
#ifndef BT_INTERNAL_EDGE_UTILITY_H
#define BT_INTERNAL_EDGE_UTILITY_H

#include "LinearMath/btHashMap.h"
#include "LinearMath/btVector3.h"

///The btInternalEdgeUtility helps to avoid or reduce artifacts due to wrong collision normals caused by internal edges.
///See also http://code.google.com/p/bullet/issues/detail?id=27

class btBvhTriangleMeshShape;
class btCollisionObject;
class btManifoldPoint;
class btIDebugDraw;

///for btTriangleInfo m_flags
#define TRI_INFO_V0V1_CONVEX 1
#define TRI_INFO_V1V2_CONVEX 2
#define TRI_INFO_V2V0_CONVEX 4

#define TRI_INFO_V0V1_SWAP_NORMALB 8
#define TRI_INFO_V1V2_SWAP_NORMALB 16
#define TRI_INFO_V2V0_SWAP_NORMALB 32


///The btTriangleInfo structure stores information to adjust collision normals to avoid collisions against internal edges
///it can be generated using 
struct	btTriangleInfo
{
	btTriangleInfo()
	{
		m_edgeV0V1Angle = SIMD_2_PI;
		m_edgeV1V2Angle = SIMD_2_PI;
		m_edgeV2V0Angle = SIMD_2_PI;
		m_flags=0;
	}

	int			m_flags;

	btScalar	m_edgeV0V1Angle;
	btScalar	m_edgeV1V2Angle;
	btScalar	m_edgeV2V0Angle;

};

typedef btHashMap<btHashInt,btTriangleInfo> btInternalTriangleInfoMap;


///The btTriangleInfoMap stores edge angle information for some triangles. You can compute this information yourself or using btGenerateInternalEdgeInfo.
struct	btTriangleInfoMap : public btInternalTriangleInfoMap
{
	btScalar	m_convexEpsilon;///used to determine if an edge or contact normal is convex, using the dot product
	btScalar	m_planarEpsilon; ///used to determine if a triangle edge is planar with zero angle
	btScalar	m_equalVertexThreshold; ///used to compute connectivity: if the distance between two vertices is smaller than m_equalVertexThreshold, they are considered to be 'shared'
	btScalar	m_edgeDistanceThreshold; ///used to determine edge contacts: if the closest distance between a contact point and an edge is smaller than this distance threshold it is considered to "hit the edge"
	btScalar	m_zeroAreaThreshold; ///used to determine if a triangle is degenerate (length squared of cross product of 2 triangle edges < threshold)
	
	
	btTriangleInfoMap()
	{
		m_convexEpsilon = 0.00f;
		m_planarEpsilon = 0.0001f;
		m_equalVertexThreshold = btScalar(0.0001)*btScalar(0.0001);
		m_edgeDistanceThreshold = btScalar(0.1);
		m_zeroAreaThreshold = btScalar(0.0001)*btScalar(0.0001);
	}
};

enum btInternalEdgeAdjustFlags
{
	BT_TRIANGLE_CONVEX_BACKFACE_MODE = 1,
	BT_TRIANGLE_CONCAVE_DOUBLE_SIDED = 2, //double sided options are experimental, single sided is recommended
	BT_TRIANGLE_CONVEX_DOUBLE_SIDED = 4
};


///Call btGenerateInternalEdgeInfo to create triangle info, store in the shape 'userInfo'
void	btGenerateInternalEdgeInfo (btBvhTriangleMeshShape*trimeshShape, btTriangleInfoMap* triangleInfoMap);


///Call the btFixMeshNormal to adjust the collision normal, using the triangle info map (generated using btGenerateInternalEdgeInfo)
///If this info map is missing, or the triangle is not store in this map, nothing will be done
void	btAdjustInternalEdgeContacts(btManifoldPoint& cp, const btCollisionObject* trimeshColObj0,const btCollisionObject* otherColObj1, int partId0, int index0, int normalAdjustFlags = 0);

///Enable the BT_INTERNAL_EDGE_DEBUG_DRAW define and call btSetDebugDrawer, to get visual info to see if the internal edge utility works properly.
///If the utility doesn't work properly, you might have to adjust the threshold values in btTriangleInfoMap
//#define BT_INTERNAL_EDGE_DEBUG_DRAW

#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
void	btSetDebugDrawer(btIDebugDraw* debugDrawer);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW


#endif //BT_INTERNAL_EDGE_UTILITY_H

