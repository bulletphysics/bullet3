#ifndef __SPU_COLLISION_SHAPES_H
#define __SPU_COLLISION_SHAPES_H

#include "../SpuDoubleBuffer.h"

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"

#include "BulletCollision/CollisionShapes/btOptimizedBvh.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/CollisionShapes/btCapsuleShape.h"

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"

#define MAX_SHAPE_SIZE 256
#define MAX_NUM_SPU_CONVEX_POINTS 128

struct SpuInternalShape
{
	ATTRIBUTE_ALIGNED16(char m_collisionShapeStorage[MAX_SHAPE_SIZE]);
	btConvexShape* m_convexShape;
	btCollisionShape* m_collisionShape;
	ppu_address_t m_ppuConvexShapePtr;
	SpuInternalShape ()
	{
		m_convexShape = (btConvexShape*)&m_collisionShapeStorage[0];
		m_collisionShape = (btCollisionShape*)&m_collisionShapeStorage[0];
		m_ppuConvexShapePtr = 0;
	}

	void dmaShapeData (ppu_address_t ppuAddress, int shapeType, uint32_t dmaTag);
};

struct SpuInternalConvexHull
{
	ATTRIBUTE_ALIGNED16(btVector3 m_pointsBuffer[MAX_NUM_SPU_CONVEX_POINTS]);
	int m_numPoints;
	btVector3* m_points;
	ppu_address_t m_ppuPointsPtr;

	SpuInternalConvexHull ()
	{
		m_points = (btVector3*)&m_pointsBuffer[0];
		m_numPoints = 0;
		m_ppuPointsPtr = 0;
	}

	void dmaPointsData (const SpuInternalShape& shape, uint32_t dmaTag);
	void dmaPointsData (ppu_address_t ppuPointsAddress, int numPoints, uint32_t dmaTag);
};



struct SpuBvhMeshShape
{
	ATTRIBUTE_ALIGNED16(char m_optimizedBvhBuffer[sizeof(btOptimizedBvh)+16]);
	ATTRIBUTE_ALIGNED16(btTriangleIndexVertexArray m_triangleMeshInterfaceBuffer);
	#define MAX_SPU_SUBTREE_HEADERS 32
	///only a single mesh part for now, we can add support for multiple parts, but quantized trees don't support this at the moment 
	ATTRIBUTE_ALIGNED16(btIndexedMesh	m_indexMesh);
	ATTRIBUTE_ALIGNED16(btBvhSubtreeInfo m_subtreeHeaders[MAX_SPU_SUBTREE_HEADERS]);
	ATTRIBUTE_ALIGNED16(btQuantizedBvhNode m_subtreeNodes[MAX_SUBTREE_SIZE_IN_BYTES/sizeof(btQuantizedBvhNode)]);
	btOptimizedBvh* m_optimizedBvh;
	btTriangleIndexVertexArray* m_triangleMeshInterface;

	SpuBvhMeshShape ()
	{
		m_optimizedBvh = (btOptimizedBvh*)&m_optimizedBvhBuffer[0];
		m_triangleMeshInterface = (btTriangleIndexVertexArray*)&m_triangleMeshInterfaceBuffer;
	}

	void dmaMeshInterfaceAndOptimizedBvh (const SpuInternalShape& triangleMeshShape, uint32_t dmaTag1, uint32_t dmaTag2);
	void dmaIndexedMesh (int index, uint32_t dmaTag);
	void dmaSubTreeHeaders (ppu_address_t subTreePtr, int numHeaders, uint32_t dmaTag);
	void dmaSubTreeNodes (const btBvhSubtreeInfo& subtree, QuantizedNodeArray&	nodeArray, int dmaTag);
};

struct	SpuConvexPolyhedronVertexData
{
	void*	gSpuConvexShapePtr;
	btPoint3* gConvexPoints;
	int gNumConvexPoints;
	ATTRIBUTE_ALIGNED16(btPoint3 g_convexPointBuffer[MAX_NUM_SPU_CONVEX_POINTS]);
};

struct SpuCompoundShape
{
	#define MAX_SPU_COMPOUND_SUBSHAPES 16
	ATTRIBUTE_ALIGNED16(btCompoundShapeChild m_subshapes[MAX_SPU_COMPOUND_SUBSHAPES]);

	void dmaChildShapeInfo (btCompoundShape* compoundShape, uint32_t dmaTag);
	void dmaChildShape (int childShape,
							SpuInternalShape* localShape,
							SpuInternalConvexHull* localShapeHull,
							uint32_t dmaTag);
};



struct CollisionShape_LocalStoreMemory
{
	ATTRIBUTE_ALIGNED16(char collisionShape[MAX_SHAPE_SIZE]);
};

struct CompoundShape_LocalStoreMemory
{
	// Compound data
#define MAX_SPU_COMPOUND_SUBSHAPES 16
	ATTRIBUTE_ALIGNED16(btCompoundShapeChild gSubshapes[MAX_SPU_COMPOUND_SUBSHAPES]);
	ATTRIBUTE_ALIGNED16(char gSubshapeShape[MAX_SPU_COMPOUND_SUBSHAPES][MAX_SHAPE_SIZE]);
};

struct bvhMeshShape_LocalStoreMemory
{
	//ATTRIBUTE_ALIGNED16(btOptimizedBvh	gOptimizedBvh);
	ATTRIBUTE_ALIGNED16(char gOptimizedBvh[sizeof(btOptimizedBvh)+16]);
	btOptimizedBvh*	getOptimizedBvh()
	{
		return (btOptimizedBvh*) gOptimizedBvh;
	}

	ATTRIBUTE_ALIGNED16(btTriangleIndexVertexArray	gTriangleMeshInterfaceStorage);
	btTriangleIndexVertexArray*	gTriangleMeshInterfacePtr;
	///only a single mesh part for now, we can add support for multiple parts, but quantized trees don't support this at the moment 
	ATTRIBUTE_ALIGNED16(btIndexedMesh	gIndexMesh);
	#define MAX_SPU_SUBTREE_HEADERS 32
	//1024
	ATTRIBUTE_ALIGNED16(btBvhSubtreeInfo	gSubtreeHeaders[MAX_SPU_SUBTREE_HEADERS]);
	ATTRIBUTE_ALIGNED16(btQuantizedBvhNode	gSubtreeNodes[MAX_SUBTREE_SIZE_IN_BYTES/sizeof(btQuantizedBvhNode)]);
};


btPoint3 localGetSupportingVertexWithoutMargin(int shapeType, void* shape, const btVector3& localDir,struct	SpuConvexPolyhedronVertexData* convexVertexData);//, int *featureIndex)
void computeAabb (btVector3& aabbMin, btVector3& aabbMax, btConvexInternalShape* convexShape, ppu_address_t convexShapePtr, int shapeType, btTransform xform);
void dmaBvhShapeData (bvhMeshShape_LocalStoreMemory* bvhMeshShape, btBvhTriangleMeshShape* triMeshShape);
void dmaBvhIndexedMesh (btIndexedMesh* IndexMesh, IndexedMeshArray& indexArray, int index, uint32_t dmaTag);
void dmaBvhSubTreeHeaders (btBvhSubtreeInfo* subTreeHeaders, ppu_address_t subTreePtr, int batchSize, uint32_t dmaTag);
void dmaBvhSubTreeNodes (btQuantizedBvhNode* nodes, const btBvhSubtreeInfo& subtree, QuantizedNodeArray&	nodeArray, int dmaTag);

int  getShapeTypeSize(int shapeType);
void dmaConvexVertexData (SpuConvexPolyhedronVertexData* convexVertexData, btConvexHullShape* convexShapeSPU);
void dmaCollisionShape (void* collisionShapeLocation, ppu_address_t collisionShapePtr, uint32_t dmaTag, int shapeType);
void dmaCompoundShapeInfo (CompoundShape_LocalStoreMemory* compoundShapeLocation, btCompoundShape* spuCompoundShape, uint32_t dmaTag);
void dmaCompoundSubShapes (CompoundShape_LocalStoreMemory* compoundShapeLocation, btCompoundShape* spuCompoundShape, uint32_t dmaTag);

#define USE_BRANCHFREE_TEST 1
#ifdef USE_BRANCHFREE_TEST
SIMD_FORCE_INLINE unsigned int spuTestQuantizedAabbAgainstQuantizedAabb(unsigned short int* aabbMin1,unsigned short int* aabbMax1,const unsigned short int* aabbMin2,const unsigned short int* aabbMax2)
{		
	return btSelect((unsigned)((aabbMin1[0] <= aabbMax2[0]) & (aabbMax1[0] >= aabbMin2[0])
		& (aabbMin1[2] <= aabbMax2[2]) & (aabbMax1[2] >= aabbMin2[2])
		& (aabbMin1[1] <= aabbMax2[1]) & (aabbMax1[1] >= aabbMin2[1])),
		1, 0);
}
#else

unsigned int spuTestQuantizedAabbAgainstQuantizedAabb(const unsigned short int* aabbMin1,const unsigned short int* aabbMax1,const unsigned short int* aabbMin2,const unsigned short int*  aabbMax2)
{
	unsigned int overlap = 1;
	overlap = (aabbMin1[0] > aabbMax2[0] || aabbMax1[0] < aabbMin2[0]) ? 0 : overlap;
	overlap = (aabbMin1[2] > aabbMax2[2] || aabbMax1[2] < aabbMin2[2]) ? 0 : overlap;
	overlap = (aabbMin1[1] > aabbMax2[1] || aabbMax1[1] < aabbMin2[1]) ? 0 : overlap;
	return overlap;
}
#endif

void	spuWalkStacklessQuantizedTree(btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax,const btQuantizedBvhNode* rootNode,int startNodeIndex,int endNodeIndex);

#endif
