
#include "SpuGatheringCollisionTask.h"

//#define DEBUG_SPU_COLLISION_DETECTION 1
#include "../SpuDoubleBuffer.h"

#include "../SpuCollisionTaskProcess.h"
#include "../SpuGatheringCollisionDispatcher.h" //for SPU_BATCHSIZE_BROADPHASE_PAIRS

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "../SpuContactManifoldCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "SpuContactResult.h"
#include "BulletCollision/CollisionShapes/btOptimizedBvh.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/CollisionShapes/btCapsuleShape.h"

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"

#include "SpuMinkowskiPenetrationDepthSolver.h"
#include "SpuGjkPairDetector.h"
#include "SpuVoronoiSimplexSolver.h"

#include "SpuLocalSupport.h" //definition of SpuConvexPolyhedronVertexData

#ifdef __CELLOS_LV2__
///Software caching from the IBM Cell SDK, it reduces 25% SPU time for our test cases
#define USE_SOFTWARE_CACHE 1
#endif //__CELLOS_LV2__

////////////////////////////////////////////////
/// software caching
#if USE_SOFTWARE_CACHE
#include <spu_intrinsics.h>
#include <sys/spu_thread.h>
#include <sys/spu_event.h>
#include <stdint.h>
#define SPE_CACHE_NWAY   		4
//#define SPE_CACHE_NSETS 		32, 16
#define SPE_CACHE_NSETS 		8
//#define SPE_CACHELINE_SIZE 		512
#define SPE_CACHELINE_SIZE 		128
#define SPE_CACHE_SET_TAGID(set) 	15
///make sure that spe_cache.h is below those defines!
#include "spe_cache.h"


int g_CacheMisses=0;
int g_CacheHits=0;

#if 0 // Added to allow cache misses and hits to be tracked, change this to 1 to restore unmodified version
#define spe_cache_read(ea)		_spe_cache_lookup_xfer_wait_(ea, 0, 1)
#else
#define spe_cache_read(ea)		\
({								\
    int set, idx, line, byte;					\
    _spe_cache_nway_lookup_(ea, set, idx);			\
								\
    if (btUnlikely(idx < 0)) {					\
        ++g_CacheMisses;                        \
	    idx = _spe_cache_miss_(ea, set, -1);			\
        spu_writech(22, SPE_CACHE_SET_TAGMASK(set));		\
        spu_mfcstat(MFC_TAG_UPDATE_ALL);			\
    } 								\
    else                            \
    {                               \
        ++g_CacheHits;              \
    }                               \
    line = _spe_cacheline_num_(set, idx);			\
    byte = _spe_cacheline_byte_offset_(ea);			\
    (void *) &spe_cache_mem[line + byte];			\
})

#endif

#endif // USE_SOFTWARE_CACHE


#ifdef USE_SN_TUNER
#include <LibSN_SPU.h>
#endif //USE_SN_TUNER

#ifdef WIN32
#define IGNORE_ALIGNMENT 1
#define spu_printf printf
#include <stdio.h>
#endif

#define MAX_SHAPE_SIZE 256

//int gNumConvexPoints0=0;



///Make sure no destructors are called on this memory
struct	CollisionTask_LocalStoreMemory
{

	ATTRIBUTE_ALIGNED16(char	bufferProxy0[16]);
	ATTRIBUTE_ALIGNED16(char	bufferProxy1[16]);

	ATTRIBUTE_ALIGNED16(btBroadphaseProxy*	gProxyPtr0);
	ATTRIBUTE_ALIGNED16(btBroadphaseProxy*	gProxyPtr1);

	//ATTRIBUTE_ALIGNED16(btCollisionObject	gColObj0);
	//ATTRIBUTE_ALIGNED16(btCollisionObject	gColObj1);
	ATTRIBUTE_ALIGNED16(char gColObj0 [sizeof(btCollisionObject)+16]);
	ATTRIBUTE_ALIGNED16(char gColObj1 [sizeof(btCollisionObject)+16]);
	
	btCollisionObject* getColObj0()
	{
		return (btCollisionObject*) gColObj0;
	}
	btCollisionObject* getColObj1()
	{
		return (btCollisionObject*) gColObj1;
	}

	DoubleBuffer<unsigned char, MIDPHASE_WORKUNIT_PAGE_SIZE> g_workUnitTaskBuffers;
	ATTRIBUTE_ALIGNED16(btBroadphasePair	gBroadphasePairs[SPU_BATCHSIZE_BROADPHASE_PAIRS]);


	//SpuContactManifoldCollisionAlgorithm	gSpuContactManifoldAlgo;
	//ATTRIBUTE_ALIGNED16(char	gSpuContactManifoldAlgo[sizeof(SpuContactManifoldCollisionAlgorithm)+128]);

	SpuContactManifoldCollisionAlgorithm	gSpuContactManifoldAlgo;

	SpuContactManifoldCollisionAlgorithm*	getlocalCollisionAlgorithm()
	{
		return (SpuContactManifoldCollisionAlgorithm*)&gSpuContactManifoldAlgo;

	}
	btPersistentManifold	gPersistentManifold;

	ATTRIBUTE_ALIGNED16(char	gCollisionShape0[MAX_SHAPE_SIZE]);
	ATTRIBUTE_ALIGNED16(char	gCollisionShape1[MAX_SHAPE_SIZE]);

	ATTRIBUTE_ALIGNED16(int	spuIndices[16]);

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

	SpuConvexPolyhedronVertexData convexVertexData;

	// Compound data
#define MAX_SPU_COMPOUND_SUBSHAPES 16
	ATTRIBUTE_ALIGNED16(btCompoundShapeChild gSubshapes[MAX_SPU_COMPOUND_SUBSHAPES*2]);
	ATTRIBUTE_ALIGNED16(char gSubshapeShape[MAX_SPU_COMPOUND_SUBSHAPES*2][MAX_SHAPE_SIZE]);
	
};



#ifdef WIN32
void* createCollisionLocalStoreMemory()
{
	return new CollisionTask_LocalStoreMemory;
};


#elif defined(__CELLOS_LV2__) || defined(USE_LIBSPE2) 

ATTRIBUTE_ALIGNED16(CollisionTask_LocalStoreMemory	gLocalStoreMemory);

void* createCollisionLocalStoreMemory()
{
	return &gLocalStoreMemory;
}
#endif


void	ProcessSpuConvexConvexCollision(SpuCollisionPairInput* wuInput, CollisionTask_LocalStoreMemory* lsMemPtr, SpuContactResult& spuContacts);

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



void	spuWalkStacklessQuantizedTree(btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax,const btQuantizedBvhNode* rootNode,int startNodeIndex,int endNodeIndex)
{

	int curIndex = startNodeIndex;
	int walkIterations = 0;
	int subTreeSize = endNodeIndex - startNodeIndex;

	int escapeIndex;

	unsigned int aabbOverlap, isLeafNode;

	while (curIndex < endNodeIndex)
	{
		//catch bugs in tree data
		assert (walkIterations < subTreeSize);

		walkIterations++;
		aabbOverlap = spuTestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode->m_quantizedAabbMin,rootNode->m_quantizedAabbMax);
		isLeafNode = rootNode->isLeafNode();

		if (isLeafNode && aabbOverlap)
		{
			//printf("overlap with node %d\n",rootNode->getTriangleIndex());
			nodeCallback->processNode(0,rootNode->getTriangleIndex());
			//			spu_printf("SPU: overlap detected with triangleIndex:%d\n",rootNode->getTriangleIndex());
		} 

		if (aabbOverlap || isLeafNode)
		{
			rootNode++;
			curIndex++;
		} else
		{
			escapeIndex = rootNode->getEscapeIndex();
			rootNode += escapeIndex;
			curIndex += escapeIndex;
		}
	}

}


#ifdef USE_ADDR64
SIMD_FORCE_INLINE void small_cache_read(void* buffer, uint64_t ea, size_t size)
#else
SIMD_FORCE_INLINE void small_cache_read(void* buffer, uint32_t ea, size_t size)
#endif
{
#if USE_SOFTWARE_CACHE
	// Check for alignment requirements. We need to make sure the entire request fits within one cache line,
	// so the first and last bytes should fall on the same cache line
	btAssert((ea & ~SPE_CACHELINE_MASK) == ((ea + size - 1) & ~SPE_CACHELINE_MASK));

	void* ls = spe_cache_read(ea);
	memcpy(buffer, ls, size);
#else
	stallingUnalignedDmaSmallGet(buffer,ea,size);
#endif
}


#ifdef USE_LIBSPE2
#ifdef USE_ADDR64
SIMD_FORCE_INLINE void small_cache_read_triple(	void* ls0, uint64_t ea0,
												void* ls1, uint64_t ea1,
												void* ls2, uint64_t ea2,
												size_t size)
#else
SIMD_FORCE_INLINE void small_cache_read_triple(	void* ls0, uint32_t ea0,
												void* ls1, uint32_t ea1,
												void* ls2, uint32_t ea2,
												size_t size)
#endif
{
		btAssert(size<16);
		ATTRIBUTE_ALIGNED16(char	tmpBuffer0[32]);
		ATTRIBUTE_ALIGNED16(char	tmpBuffer1[32]);
		ATTRIBUTE_ALIGNED16(char	tmpBuffer2[32]);

		uint32_t i;
		

		///make sure last 4 bits are the same, for cellDmaSmallGet
		char* localStore0 = (char*)ls0;
		uint32_t last4BitsOffset = ea0 & 0x0f;
		char* tmpTarget0 = tmpBuffer0 + last4BitsOffset;
		cellDmaSmallGet(tmpTarget0,ea0,size,DMA_TAG(1),0,0);


		char* localStore1 = (char*)ls1;
		last4BitsOffset = ea1 & 0x0f;
		char* tmpTarget1 = tmpBuffer1 + last4BitsOffset;
		cellDmaSmallGet(tmpTarget1,ea1,size,DMA_TAG(1),0,0);
		
		char* localStore2 = (char*)ls2;
		last4BitsOffset = ea2 & 0x0f;
		char* tmpTarget2 = tmpBuffer2 + last4BitsOffset;
		cellDmaSmallGet(tmpTarget2,ea2,size,DMA_TAG(1),0,0);
		
		
		cellDmaWaitTagStatusAll( DMA_MASK(1) );

		//this is slowish, perhaps memcpy on SPU is smarter?
		for (i=0; btLikely( i<size );i++)
		{
			localStore0[i] = tmpTarget0[i];
			localStore1[i] = tmpTarget1[i];
			localStore2[i] = tmpTarget2[i];
		}

		
}
#endif



class spuNodeCallback : public btNodeOverlapCallback
{
	SpuCollisionPairInput* m_wuInput;
	SpuContactResult&		m_spuContacts;
	CollisionTask_LocalStoreMemory*	m_lsMemPtr;

	ATTRIBUTE_ALIGNED16(btVector3	spuTriangleVertices[3]);
	ATTRIBUTE_ALIGNED16(btScalar	spuUnscaledVertex[4]);
	ATTRIBUTE_ALIGNED16(int	spuIndices[16]);


public:
	spuNodeCallback(SpuCollisionPairInput* wuInput, CollisionTask_LocalStoreMemory*	lsMemPtr,SpuContactResult& spuContacts)
		:	m_wuInput(wuInput),
		m_lsMemPtr(lsMemPtr),
		m_spuContacts(spuContacts)
	{
	}

	virtual void processNode(int subPart, int triangleIndex)
	{
		///Create a triangle on the stack, call process collision, with GJK
		///DMA the vertices, can benefit from software caching

		//		spu_printf("processNode with triangleIndex %d\n",triangleIndex);



		int* indexBasePtr = (int*)(m_lsMemPtr->gIndexMesh.m_triangleIndexBase+triangleIndex*m_lsMemPtr->gIndexMesh.m_triangleIndexStride);

		///DMA the indices
#ifdef USE_LIBSPE2
#ifdef USE_ADDR64
		small_cache_read_triple(&m_lsMemPtr->spuIndices[0],(uint64_t)&indexBasePtr[0],
								&m_lsMemPtr->spuIndices[1],(uint64_t)&indexBasePtr[1],
								&m_lsMemPtr->spuIndices[2],(uint64_t)&indexBasePtr[2],
								sizeof(int));
#else
		small_cache_read_triple(&m_lsMemPtr->spuIndices[0],(uint32_t)&indexBasePtr[0],
								&m_lsMemPtr->spuIndices[1],(uint32_t)&indexBasePtr[1],
								&m_lsMemPtr->spuIndices[2],(uint32_t)&indexBasePtr[2],
								sizeof(int));
#endif
#else
#ifdef USE_ADDR64
		small_cache_read(&m_lsMemPtr->spuIndices[0],(uint64_t)&indexBasePtr[0],sizeof(int));
		small_cache_read(&m_lsMemPtr->spuIndices[1],(uint64_t)&indexBasePtr[1],sizeof(int));
		small_cache_read(&m_lsMemPtr->spuIndices[2],(uint64_t)&indexBasePtr[2],sizeof(int));
#else
		small_cache_read(&m_lsMemPtr->spuIndices[0],(uint32_t)&indexBasePtr[0],sizeof(int));
		small_cache_read(&m_lsMemPtr->spuIndices[1],(uint32_t)&indexBasePtr[1],sizeof(int));
		small_cache_read(&m_lsMemPtr->spuIndices[2],(uint32_t)&indexBasePtr[2],sizeof(int));
#endif
#endif
		
		//		spu_printf("SPU index0=%d ,",spuIndices[0]);
		//		spu_printf("SPU index1=%d ,",spuIndices[1]);
		//		spu_printf("SPU index2=%d ,",spuIndices[2]);
		//		spu_printf("SPU: indexBasePtr=%llx\n",indexBasePtr);

		const btVector3& meshScaling = m_lsMemPtr->gTriangleMeshInterfacePtr->getScaling();
		for (int j=2;btLikely( j>=0 );j--)
		{
			int graphicsindex = m_lsMemPtr->spuIndices[j];

			//			spu_printf("SPU index=%d ,",graphicsindex);
			btScalar* graphicsbasePtr = (btScalar*)(m_lsMemPtr->gIndexMesh.m_vertexBase+graphicsindex*m_lsMemPtr->gIndexMesh.m_vertexStride);
			//			spu_printf("SPU graphicsbasePtr=%llx\n",graphicsbasePtr);


			///handle un-aligned vertices...

			//another DMA for each vertex
#ifdef USE_LIBSPE2
#ifdef USE_ADDR64
			small_cache_read_triple(	&spuUnscaledVertex[0],(uint64_t)&graphicsbasePtr[0],
									&spuUnscaledVertex[1],(uint64_t)&graphicsbasePtr[1],
									&spuUnscaledVertex[2],(uint64_t)&graphicsbasePtr[2],
									sizeof(btScalar));
#else
			small_cache_read_triple(	&spuUnscaledVertex[0],(uint32_t)&graphicsbasePtr[0],
									&spuUnscaledVertex[1],(uint32_t)&graphicsbasePtr[1],
									&spuUnscaledVertex[2],(uint32_t)&graphicsbasePtr[2],
									sizeof(btScalar));
#endif
#else
#ifdef USE_ADDR64
			small_cache_read(&spuUnscaledVertex[0],(uint64_t)&graphicsbasePtr[0],sizeof(btScalar));
			small_cache_read(&spuUnscaledVertex[1],(uint64_t)&graphicsbasePtr[1],sizeof(btScalar));
			small_cache_read(&spuUnscaledVertex[2],(uint64_t)&graphicsbasePtr[2],sizeof(btScalar));
#else
			small_cache_read(&spuUnscaledVertex[0],(uint32_t)&graphicsbasePtr[0],sizeof(btScalar));
			small_cache_read(&spuUnscaledVertex[1],(uint32_t)&graphicsbasePtr[1],sizeof(btScalar));
			small_cache_read(&spuUnscaledVertex[2],(uint32_t)&graphicsbasePtr[2],sizeof(btScalar));
#endif
#endif		
			
			spuTriangleVertices[j] = btVector3(
				spuUnscaledVertex[0]*meshScaling.getX(),
				spuUnscaledVertex[1]*meshScaling.getY(),
				spuUnscaledVertex[2]*meshScaling.getZ());

			//			spu_printf("SPU:triangle vertices:%f,%f,%f\n",spuTriangleVertices[j].x(),spuTriangleVertices[j].y(),spuTriangleVertices[j].z());
		}



		//btTriangleShape	tmpTriangleShape(spuTriangleVertices[0],spuTriangleVertices[1],spuTriangleVertices[2]);


		SpuCollisionPairInput triangleConcaveInput(*m_wuInput);
		triangleConcaveInput.m_spuCollisionShapes[1] = &spuTriangleVertices[0];
		triangleConcaveInput.m_shapeType1 = TRIANGLE_SHAPE_PROXYTYPE;

		m_spuContacts.setShapeIdentifiers(-1,-1,subPart,triangleIndex);

		//		m_spuContacts.flush();

		ProcessSpuConvexConvexCollision(&triangleConcaveInput, m_lsMemPtr,m_spuContacts);
		///this flush should be automatic
		//	m_spuContacts.flush();
	}

};




////////////////////////
/// Convex versus Concave triangle mesh collision detection (handles concave triangle mesh versus sphere, box, cylinder, triangle, cone, convex polyhedron etc)
///////////////////
void	ProcessConvexConcaveSpuCollision(SpuCollisionPairInput* wuInput, CollisionTask_LocalStoreMemory* lsMemPtr, SpuContactResult& spuContacts)
{
	//order: first collision shape is convex, second concave. m_isSwapped is true, if the original order was opposite


	register int dmaSize;
#ifdef USE_ADDR64
	register uint64_t	dmaPpuAddress2;
#else
		register uint32_t	dmaPpuAddress2;
#endif	


	btBvhTriangleMeshShape*	trimeshShape = (btBvhTriangleMeshShape*)wuInput->m_spuCollisionShapes[1];
	//need the mesh interface, for access to triangle vertices
	
	dmaSize = sizeof(btTriangleIndexVertexArray);
#ifdef USE_ADDR64
	dmaPpuAddress2 = reinterpret_cast<uint64_t>(trimeshShape->getMeshInterface());
#else
	dmaPpuAddress2 = reinterpret_cast<uint32_t>(trimeshShape->getMeshInterface());
#endif
	//	spu_printf("trimeshShape->getMeshInterface() == %llx\n",dmaPpuAddress2);
	lsMemPtr->gTriangleMeshInterfacePtr = (btTriangleIndexVertexArray*)cellDmaGetReadOnly(&lsMemPtr->gTriangleMeshInterfaceStorage, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
	//cellDmaWaitTagStatusAll(DMA_MASK(1));
	

	///now DMA over the BVH
	
	dmaSize = sizeof(btOptimizedBvh);
#ifdef USE_ADDR64
	dmaPpuAddress2 = reinterpret_cast<uint64_t>(trimeshShape->getOptimizedBvh());
#else
	dmaPpuAddress2 = reinterpret_cast<uint32_t>(trimeshShape->getOptimizedBvh());
#endif
	//spu_printf("trimeshShape->getOptimizedBvh() == %llx\n",dmaPpuAddress2);
	cellDmaGet(&lsMemPtr->gOptimizedBvh, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);
	//cellDmaWaitTagStatusAll(DMA_MASK(2));
	cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
	

	btVector3 aabbMin(-1,-400,-1);
	btVector3 aabbMax(1,400,1);


	//recalc aabbs
	btTransform convexInTriangleSpace;
	convexInTriangleSpace = wuInput->m_worldTransform1.inverse() * wuInput->m_worldTransform0;
	btConvexInternalShape* convexShape = (btConvexInternalShape*)wuInput->m_spuCollisionShapes[0];
	//calculate the aabb, given the types...
	switch (wuInput->m_shapeType0)
	{
	case CYLINDER_SHAPE_PROXYTYPE:

	case BOX_SHAPE_PROXYTYPE:
		{
			float margin=convexShape->getMarginNV();
			btVector3 halfExtents = convexShape->getImplicitShapeDimensions();
			btTransform& t = convexInTriangleSpace;
			btMatrix3x3 abs_b = t.getBasis().absolute();  
			btPoint3 center = t.getOrigin();
			btVector3 extent = btVector3(abs_b[0].dot(halfExtents),
				abs_b[1].dot(halfExtents),
				abs_b[2].dot(halfExtents));
			extent += btVector3(margin,margin,margin);
			aabbMin = center - extent;
			aabbMax = center + extent;
			break;
		}

	case CAPSULE_SHAPE_PROXYTYPE:
		{
			float margin=convexShape->getMarginNV();
			btVector3 halfExtents = convexShape->getImplicitShapeDimensions();
			//add the radius to y-axis to get full height
			btScalar radius = halfExtents[0];
			halfExtents[1] += radius;
			btTransform& t = convexInTriangleSpace;
			btMatrix3x3 abs_b = t.getBasis().absolute();  
			btPoint3 center = t.getOrigin();
			btVector3 extent = btVector3(abs_b[0].dot(halfExtents),
				abs_b[1].dot(halfExtents),
				abs_b[2].dot(halfExtents));
			extent += btVector3(margin,margin,margin);
			aabbMin = center - extent;
			aabbMax = center + extent;
			break;
		}


	case SPHERE_SHAPE_PROXYTYPE:
		{
			float radius = convexShape->getImplicitShapeDimensions().getX();// * convexShape->getLocalScaling().getX();
			float margin = radius + convexShape->getMarginNV();
			btTransform& t = convexInTriangleSpace;
			const btVector3& center = t.getOrigin();
			btVector3 extent(margin,margin,margin);
			aabbMin = center - extent;
			aabbMax = center + extent;
			break;
		}
	case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			dmaSize = sizeof(btConvexHullShape);
#ifdef USE_ADDR64
			dmaPpuAddress2 = wuInput->m_collisionShapes[0];
#else
			dmaPpuAddress2 = wuInput->m_collisionShapes[0];
#endif
			ATTRIBUTE_ALIGNED16(char convexHullShape0[sizeof(btConvexHullShape)]);

			cellDmaGet(&convexHullShape0, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
			cellDmaWaitTagStatusAll(DMA_MASK(1));
			btConvexHullShape* localPtr = (btConvexHullShape*)&convexHullShape0;
			btTransform& t = convexInTriangleSpace;

			btScalar margin = convexShape->getMarginNV();

			localPtr->getNonvirtualAabb(t,aabbMin,aabbMax,margin);

			//spu_printf("SPU convex aabbMin=%f,%f,%f=\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
			//spu_printf("SPU convex aabbMax=%f,%f,%f=\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());

			break;
		}

	default:
		spu_printf("SPU: unsupported shapetype %d in AABB calculation\n");
	};

	//CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody->m_collisionShape);
	//convexShape->getAabb(convexInTriangleSpace,m_aabbMin,m_aabbMax);

	//	btScalar extraMargin = collisionMarginTriangle;
	//	btVector3 extra(extraMargin,extraMargin,extraMargin);
	//	aabbMax += extra;
	//	aabbMin -= extra;



	///quantize query AABB
	unsigned short int quantizedQueryAabbMin[3];
	unsigned short int quantizedQueryAabbMax[3];
	lsMemPtr->getOptimizedBvh()->quantizeWithClamp(quantizedQueryAabbMin,aabbMin);
	lsMemPtr->getOptimizedBvh()->quantizeWithClamp(quantizedQueryAabbMax,aabbMax);

	QuantizedNodeArray&	nodeArray = lsMemPtr->getOptimizedBvh()->getQuantizedNodeArray();
	//spu_printf("SPU: numNodes = %d\n",nodeArray.size());

	BvhSubtreeInfoArray& subTrees = lsMemPtr->getOptimizedBvh()->getSubtreeInfoArray();

	spuNodeCallback	nodeCallback(wuInput,lsMemPtr,spuContacts);
	IndexedMeshArray&	indexArray = lsMemPtr->gTriangleMeshInterfacePtr->getIndexedMeshArray();
	//spu_printf("SPU:indexArray.size() = %d\n",indexArray.size());


	//	spu_printf("SPU: numSubTrees = %d\n",subTrees.size());
	//not likely to happen
	if (subTrees.size() && indexArray.size() == 1)
	{
		///DMA in the index info
		
		dmaSize = sizeof(btIndexedMesh);
#ifdef USE_ADDR64
		dmaPpuAddress2 = reinterpret_cast<uint64_t>(&indexArray[0]);
#else
		dmaPpuAddress2 = reinterpret_cast<uint32_t>(&indexArray[0]);
#endif
		cellDmaGet(&lsMemPtr->gIndexMesh, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
		cellDmaWaitTagStatusAll(DMA_MASK(1));
		

		//spu_printf("SPU gIndexMesh dma finished\n");

		//display the headers
		int numBatch = subTrees.size();
		for (int i=0;i<numBatch;)
		{

// BEN: TODO - can reorder DMA transfers for less stall
			int remaining = subTrees.size() - i;
			int nextBatch = remaining < MAX_SPU_SUBTREE_HEADERS ? remaining : MAX_SPU_SUBTREE_HEADERS;
			
			dmaSize = nextBatch* sizeof(btBvhSubtreeInfo);
#ifdef USE_ADDR64
			dmaPpuAddress2 = reinterpret_cast<uint64_t>(&subTrees[i]);
#else
			dmaPpuAddress2 = reinterpret_cast<uint32_t>(&subTrees[i]);
#endif
				//				spu_printf("&subtree[i]=%llx, dmaSize = %d\n",dmaPpuAddress2,dmaSize);
			cellDmaGet(&lsMemPtr->gSubtreeHeaders[0], dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
			cellDmaWaitTagStatusAll(DMA_MASK(1));
			

			//			spu_printf("nextBatch = %d\n",nextBatch);

			for (int j=0;j<nextBatch;j++)
			{
				const btBvhSubtreeInfo& subtree = lsMemPtr->gSubtreeHeaders[j];

				unsigned int overlap = spuTestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
				if (overlap)
				{
					btAssert(subtree.m_subtreeSize);

					//dma the actual nodes of this subtree
					
					dmaSize = subtree.m_subtreeSize* sizeof(btQuantizedBvhNode);
#ifdef USE_ADDR64
					dmaPpuAddress2 = reinterpret_cast<uint64_t>(&nodeArray[subtree.m_rootNodeIndex]);
#else
					dmaPpuAddress2 = reinterpret_cast<uint32_t>(&nodeArray[subtree.m_rootNodeIndex]);
#endif
					cellDmaGet(&lsMemPtr->gSubtreeNodes[0], dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);
					cellDmaWaitTagStatusAll(DMA_MASK(2));
					



					spuWalkStacklessQuantizedTree(&nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,
						&lsMemPtr->gSubtreeNodes[0],
						0,
						subtree.m_subtreeSize);

				}


				//				spu_printf("subtreeSize = %d\n",gSubtreeHeaders[j].m_subtreeSize);
			}

			//	unsigned short int	m_quantizedAabbMin[3];
			//	unsigned short int	m_quantizedAabbMax[3];
			//	int			m_rootNodeIndex;
			//	int			m_subtreeSize;
			i+=nextBatch;
		}

		//pre-fetch first tree, then loop and double buffer



	}

}

///getShapeTypeSize could easily be optimized, but it is not likely a bottleneck
SIMD_FORCE_INLINE int		getShapeTypeSize(int shapeType)
{


	switch (shapeType)
	{
	case CYLINDER_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(btCylinderShape);
			btAssert(shapeSize < MAX_SHAPE_SIZE);
			return shapeSize;
		}
	case BOX_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(btBoxShape);
			btAssert(shapeSize < MAX_SHAPE_SIZE);
			return shapeSize;
		}
	case SPHERE_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(btSphereShape);
			btAssert(shapeSize < MAX_SHAPE_SIZE);
			return shapeSize;
		}
	case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(btBvhTriangleMeshShape);
			btAssert(shapeSize < MAX_SHAPE_SIZE);
			return shapeSize;
		}
	case CAPSULE_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(btCapsuleShape);
			btAssert(shapeSize < MAX_SHAPE_SIZE);
			return shapeSize;
		}

	case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(btConvexHullShape);
			btAssert(shapeSize < MAX_SHAPE_SIZE);
			return shapeSize;
		}

	case COMPOUND_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(btCompoundShape);
			btAssert(shapeSize < MAX_SHAPE_SIZE);
			return shapeSize;
		}

	default:
		btAssert(0);
		//unsupported shapetype, please add here
		return 0;
	}
}




////////////////////////
/// Convex versus Convex collision detection (handles collision between sphere, box, cylinder, triangle, cone, convex polyhedron etc)
///////////////////
void	ProcessSpuConvexConvexCollision(SpuCollisionPairInput* wuInput, CollisionTask_LocalStoreMemory* lsMemPtr, SpuContactResult& spuContacts)
{

	
	register int dmaSize;
#ifdef USE_ADDR64
	register uint64_t	dmaPpuAddress2;
#else
	register uint32_t	dmaPpuAddress2;
#endif	
	
#ifdef DEBUG_SPU_COLLISION_DETECTION
	//spu_printf("SPU: ProcessSpuConvexConvexCollision\n");
#endif //DEBUG_SPU_COLLISION_DETECTION
	//CollisionShape* shape0 = (CollisionShape*)wuInput->m_collisionShapes[0];
	//CollisionShape* shape1 = (CollisionShape*)wuInput->m_collisionShapes[1];
	btPersistentManifold* manifold = (btPersistentManifold*)wuInput->m_persistentManifoldPtr;



	bool genericGjk = true;



	if (genericGjk)
	{
		//try generic GJK

		SpuVoronoiSimplexSolver vsSolver;
		SpuMinkowskiPenetrationDepthSolver	penetrationSolver;



		///DMA in the vertices for convex shapes
		ATTRIBUTE_ALIGNED16(char convexHullShape0[sizeof(btConvexHullShape)]);
		ATTRIBUTE_ALIGNED16(char convexHullShape1[sizeof(btConvexHullShape)]);

		if ( btLikely( wuInput->m_shapeType0== CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{
			//	spu_printf("SPU: DMA btConvexHullShape\n");
			
			dmaSize = sizeof(btConvexHullShape);
#ifdef USE_ADDR64
			dmaPpuAddress2 = wuInput->m_collisionShapes[0];
#else
			dmaPpuAddress2 = wuInput->m_collisionShapes[0];
#endif
			cellDmaGet(&convexHullShape0, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(1));
		}

		
		
		if ( btLikely( wuInput->m_shapeType1 == CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{


			//	spu_printf("SPU: DMA btConvexHullShape\n");
			dmaSize = sizeof(btConvexHullShape);
#ifdef USE_ADDR64
			dmaPpuAddress2 = wuInput->m_collisionShapes[1];
#else
			dmaPpuAddress2 = wuInput->m_collisionShapes[1];
#endif
			cellDmaGet(&convexHullShape1, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(1));
		}
		
		

		if ( btLikely( wuInput->m_shapeType0 == CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{		

			cellDmaWaitTagStatusAll(DMA_MASK(1));
			btConvexHullShape* localPtr = (btConvexHullShape*)&convexHullShape0;

			lsMemPtr->convexVertexData.gNumConvexPoints0 = localPtr->getNumPoints();
			if (lsMemPtr->convexVertexData.gNumConvexPoints0>MAX_NUM_SPU_CONVEX_POINTS)
			{
				btAssert(0);
				spu_printf("SPU: Error: MAX_NUM_SPU_CONVEX_POINTS(%d) exceeded: %d\n",MAX_NUM_SPU_CONVEX_POINTS,lsMemPtr->convexVertexData.gNumConvexPoints0);
				return;
			}
			
			dmaSize = lsMemPtr->convexVertexData.gNumConvexPoints0*sizeof(btPoint3);
#ifdef USE_ADDR64
			dmaPpuAddress2 = (uint64_t) localPtr->getPoints();
#else
			dmaPpuAddress2 = (uint32_t) localPtr->getPoints();
#endif
			cellDmaGet(&lsMemPtr->convexVertexData.g_convexPointBuffer0, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);

			lsMemPtr->convexVertexData.gSpuConvexShapePtr0 = wuInput->m_spuCollisionShapes[0];
			

		}

			
		if ( btLikely( wuInput->m_shapeType1 == CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{
			
			cellDmaWaitTagStatusAll(DMA_MASK(1));
			btConvexHullShape* localPtr = (btConvexHullShape*)&convexHullShape1;

			lsMemPtr->convexVertexData.gNumConvexPoints1 = localPtr->getNumPoints();
			if (lsMemPtr->convexVertexData.gNumConvexPoints1>MAX_NUM_SPU_CONVEX_POINTS)
			{
				btAssert(0);
				spu_printf("SPU: Error: MAX_NUM_SPU_CONVEX_POINTS(%d) exceeded: %d\n",MAX_NUM_SPU_CONVEX_POINTS,lsMemPtr->convexVertexData.gNumConvexPoints1);
				return;
			}
			
			
			dmaSize = lsMemPtr->convexVertexData.gNumConvexPoints1*sizeof(btPoint3);
#ifdef USE_ADDR64
			dmaPpuAddress2 = (uint64_t) localPtr->getPoints();
#else
			dmaPpuAddress2 = (uint32_t) localPtr->getPoints();
#endif
			cellDmaGet(&lsMemPtr->convexVertexData.g_convexPointBuffer1, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);

			lsMemPtr->convexVertexData.gSpuConvexShapePtr1 = wuInput->m_spuCollisionShapes[1];
			

		}

		if ( btLikely( wuInput->m_shapeType0 == CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{		
			cellDmaWaitTagStatusAll(DMA_MASK(2));
			
			lsMemPtr->convexVertexData.gConvexPoints0 = &lsMemPtr->convexVertexData.g_convexPointBuffer0[0];
		}

		if ( btLikely( wuInput->m_shapeType1 == CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{
			cellDmaWaitTagStatusAll(DMA_MASK(2));
			
			lsMemPtr->convexVertexData.gConvexPoints1 = &lsMemPtr->convexVertexData.g_convexPointBuffer1[0];
		}


		void* shape0Ptr = wuInput->m_spuCollisionShapes[0];
		void* shape1Ptr = wuInput->m_spuCollisionShapes[1];
		int shapeType0 = wuInput->m_shapeType0;
		int shapeType1 = wuInput->m_shapeType1;
		float marginA = wuInput->m_collisionMargin0;
		float marginB = wuInput->m_collisionMargin1;

		SpuClosestPointInput	cpInput;
		cpInput.m_convexVertexData = &lsMemPtr->convexVertexData;
		cpInput.m_transformA = wuInput->m_worldTransform0;
		cpInput.m_transformB = wuInput->m_worldTransform1;
		float sumMargin = (marginA+marginB+lsMemPtr->gPersistentManifold.getContactBreakingThreshold());
		cpInput.m_maximumDistanceSquared = sumMargin * sumMargin;

#ifdef USE_ADDR64
		uint64_t manifoldAddress = (uint64_t)manifold;
#else
		uint32_t manifoldAddress = (uint32_t)manifold;
#endif
		btPersistentManifold* spuManifold=&lsMemPtr->gPersistentManifold;
		//spuContacts.setContactInfo(spuManifold,manifoldAddress,wuInput->m_worldTransform0,wuInput->m_worldTransform1,wuInput->m_isSwapped);
		spuContacts.setContactInfo(spuManifold,manifoldAddress,lsMemPtr->getColObj0()->getWorldTransform(),lsMemPtr->getColObj1()->getWorldTransform(),wuInput->m_isSwapped);

		SpuGjkPairDetector gjk(shape0Ptr,shape1Ptr,shapeType0,shapeType1,marginA,marginB,&vsSolver,&penetrationSolver);
		gjk.getClosestPoints(cpInput,spuContacts);//,debugDraw);
	}


}


template<typename T> void DoSwap(T& a, T& b)
{
	char tmp[sizeof(T)];
	memcpy(tmp, &a, sizeof(T));
	memcpy(&a, &b, sizeof(T));
	memcpy(&b, tmp, sizeof(T));
}

SIMD_FORCE_INLINE void	dmaAndSetupCollisionObjects(SpuCollisionPairInput& collisionPairInput, CollisionTask_LocalStoreMemory& lsMem)
{
	register int dmaSize;
#ifdef USE_ADDR64
	register uint64_t	dmaPpuAddress2;
#else
	register uint32_t	dmaPpuAddress2;
#endif
		
	
		dmaSize = sizeof(btCollisionObject);
#ifdef USE_ADDR64
		dmaPpuAddress2 = /*collisionPairInput.m_isSwapped ? (uint64_t)lsMem.gProxyPtr1->m_clientObject :*/ (uint64_t)lsMem.gProxyPtr0->m_clientObject;
#else
		dmaPpuAddress2 = /*collisionPairInput.m_isSwapped ? (uint32_t)lsMem.gProxyPtr1->m_clientObject :*/ (uint32_t)lsMem.gProxyPtr0->m_clientObject;
#endif
		cellDmaGet(&lsMem.gColObj0, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);		
	
	
		dmaSize = sizeof(btCollisionObject);
#ifdef USE_ADDR64
		dmaPpuAddress2 = /*collisionPairInput.m_isSwapped ? (uint64_t)lsMem.gProxyPtr0->m_clientObject :*/ (uint64_t)lsMem.gProxyPtr1->m_clientObject;
#else
		dmaPpuAddress2 = /*collisionPairInput.m_isSwapped ? (uint32_t)lsMem.gProxyPtr0->m_clientObject :*/ (uint32_t)lsMem.gProxyPtr1->m_clientObject;
#endif
		cellDmaGet(&lsMem.gColObj1, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);		
	

	cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));

	collisionPairInput.m_worldTransform0 = lsMem.getColObj0()->getWorldTransform();
	collisionPairInput.m_worldTransform1 = lsMem.getColObj1()->getWorldTransform();



#ifdef DEBUG_SPU_COLLISION_DETECTION
#endif //DEBUG_SPU_COLLISION_DETECTION

}



#ifdef USE_ADDR64
void	handleCollisionPair(SpuCollisionPairInput& collisionPairInput, CollisionTask_LocalStoreMemory& lsMem,
							SpuContactResult &spuContacts,
							uint64_t collisionShape0Ptr, void* collisionShape0Loc,
							uint64_t collisionShape1Ptr, void* collisionShape1Loc, bool dmaShapes = true)
#else
void	handleCollisionPair(SpuCollisionPairInput& collisionPairInput, CollisionTask_LocalStoreMemory& lsMem,
							SpuContactResult &spuContacts,
							uint32_t collisionShape0Ptr, void* collisionShape0Loc,
							uint32_t collisionShape1Ptr, void* collisionShape1Loc, bool dmaShapes = true)
#endif
{
	register int dmaSize;
#ifdef USE_ADDR64
	register	uint64_t	dmaPpuAddress2;
#else
	register	uint32_t	dmaPpuAddress2;
#endif	
	
	if (btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType0) 
		&& btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType1))
	{

		//dmaAndSetupCollisionObjects(collisionPairInput, lsMem);

		if (dmaShapes)
		{
			
				dmaSize = getShapeTypeSize(collisionPairInput.m_shapeType0);
				//uint64_t	dmaPpuAddress2 = (uint64_t)lsMem.gColObj0.getCollisionShape();
#ifdef USE_ADDR64
				dmaPpuAddress2 = collisionShape0Ptr;
#else
				dmaPpuAddress2 = collisionShape0Ptr;
#endif
				cellDmaGet(collisionShape0Loc, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
				//cellDmaWaitTagStatusAll(DMA_MASK(1));
			
			
				dmaSize = getShapeTypeSize(collisionPairInput.m_shapeType1);
#ifdef USE_ADDR64
				dmaPpuAddress2 = collisionShape1Ptr;
#else
				dmaPpuAddress2 = collisionShape1Ptr;
#endif
				cellDmaGet(collisionShape1Loc, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);
				//cellDmaWaitTagStatusAll(DMA_MASK(2));
				
				cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
			
		}

		btConvexInternalShape* spuConvexShape0 = (btConvexInternalShape*)collisionShape0Loc;
		btConvexInternalShape* spuConvexShape1 = (btConvexInternalShape*)collisionShape1Loc;

		btVector3 dim0 = spuConvexShape0->getImplicitShapeDimensions();
		btVector3 dim1 = spuConvexShape1->getImplicitShapeDimensions();

		collisionPairInput.m_primitiveDimensions0 = dim0;
		collisionPairInput.m_primitiveDimensions1 = dim1;
		collisionPairInput.m_collisionShapes[0] = collisionShape0Ptr;
		collisionPairInput.m_collisionShapes[1] = collisionShape1Ptr;
		collisionPairInput.m_spuCollisionShapes[0] = spuConvexShape0;
		collisionPairInput.m_spuCollisionShapes[1] = spuConvexShape1;
		ProcessSpuConvexConvexCollision(&collisionPairInput,&lsMem,spuContacts);
	} 
	else if (btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType0) && 
			btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType1))
	{
		//snPause();

		// Both are compounds, do N^2 CD for now
		// TODO: add some AABB-based pruning
		
			dmaSize = getShapeTypeSize(collisionPairInput.m_shapeType0);
#ifdef USE_ADDR64
			dmaPpuAddress2 = collisionShape0Ptr;
#else
			dmaPpuAddress2 = collisionShape0Ptr;
#endif
			cellDmaGet(collisionShape0Loc, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(1));
		
		
			dmaSize = getShapeTypeSize(collisionPairInput.m_shapeType1);
#ifdef USE_ADDR64
			dmaPpuAddress2 = collisionShape1Ptr;
#else
			dmaPpuAddress2 = collisionShape1Ptr;
#endif
			cellDmaGet(collisionShape1Loc, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(2));
			
			cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
		

		btCompoundShape* spuCompoundShape0 = (btCompoundShape*)collisionShape0Loc;
		btCompoundShape* spuCompoundShape1 = (btCompoundShape*)collisionShape1Loc;

		int childShapeCount0 = spuCompoundShape0->getNumChildShapes();
		int childShapeCount1 = spuCompoundShape1->getNumChildShapes();

		// dma the first list of child shapes
		
			dmaSize = childShapeCount0 * sizeof(btCompoundShapeChild);
#ifdef USE_ADDR64
			dmaPpuAddress2 = (uint64_t)spuCompoundShape0->getChildList();
#else
			dmaPpuAddress2 = (uint32_t)spuCompoundShape0->getChildList();
#endif
			cellDmaGet(lsMem.gSubshapes, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(1));
		

		// dma the second list of child shapes
		
			dmaSize = childShapeCount1 * sizeof(btCompoundShapeChild);
#ifdef USE_ADDR64
			dmaPpuAddress2 = (uint64_t)spuCompoundShape1->getChildList();
#else
			dmaPpuAddress2 = (uint32_t)spuCompoundShape1->getChildList();
#endif
			cellDmaGet(&lsMem.gSubshapes[MAX_SPU_COMPOUND_SUBSHAPES], dmaPpuAddress2, dmaSize, DMA_TAG(2), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(2));
			cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
		

			int i;

		// DMA all the subshapes 
		for ( i = 0; i < childShapeCount0; ++i)
		{
			btCompoundShapeChild& childShape = lsMem.gSubshapes[i];

			dmaSize = getShapeTypeSize(childShape.m_childShapeType);
#ifdef USE_ADDR64
			dmaPpuAddress2 = (uint64_t)childShape.m_childShape;
#else
			dmaPpuAddress2 = (uint32_t)childShape.m_childShape;
#endif
			cellDmaGet(lsMem.gSubshapeShape[i], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(1));
		}
		cellDmaWaitTagStatusAll(DMA_MASK(1));

		for ( i = 0; i < childShapeCount1; ++i)
		{
			btCompoundShapeChild& childShape = lsMem.gSubshapes[MAX_SPU_COMPOUND_SUBSHAPES+i];

			dmaSize = getShapeTypeSize(childShape.m_childShapeType);
#ifdef USE_ADDR64
			dmaPpuAddress2 = (uint64_t)childShape.m_childShape;
#else
			dmaPpuAddress2 = (uint32_t)childShape.m_childShape;
#endif
			cellDmaGet(lsMem.gSubshapeShape[MAX_SPU_COMPOUND_SUBSHAPES+i], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(1));
		}
		cellDmaWaitTagStatusAll(DMA_MASK(1));

		// Start the N^2
		for ( i = 0; i < childShapeCount0; ++i)
		{
			btCompoundShapeChild& childShape0 = lsMem.gSubshapes[i];

			for (int j = 0; j < childShapeCount1; ++j)
			{
				btCompoundShapeChild& childShape1 = lsMem.gSubshapes[MAX_SPU_COMPOUND_SUBSHAPES+j];

				SpuCollisionPairInput cinput (collisionPairInput);
				cinput.m_worldTransform0 = collisionPairInput.m_worldTransform0 * childShape0.m_transform;
				cinput.m_shapeType0 = childShape0.m_childShapeType;
				cinput.m_collisionMargin0 = childShape0.m_childMargin;

				cinput.m_worldTransform1 = collisionPairInput.m_worldTransform1 * childShape1.m_transform;
				cinput.m_shapeType1 = childShape1.m_childShapeType;
				cinput.m_collisionMargin1 = childShape1.m_childMargin;

#ifdef USE_ADDR64
				handleCollisionPair(cinput, lsMem, spuContacts,			
					(uint64_t)childShape0.m_childShape, lsMem.gSubshapeShape[i], 
					(uint64_t)childShape1.m_childShape, lsMem.gSubshapeShape[MAX_SPU_COMPOUND_SUBSHAPES+i], false);
#else
				handleCollisionPair(cinput, lsMem, spuContacts,			
					(uint32_t)childShape0.m_childShape, lsMem.gSubshapeShape[i], 
					(uint32_t)childShape1.m_childShape, lsMem.gSubshapeShape[MAX_SPU_COMPOUND_SUBSHAPES+i], false);
#endif
			}
		}
	}
	else if (btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType0) )
	{
		//snPause();
		
			dmaSize = getShapeTypeSize(collisionPairInput.m_shapeType0);
#ifdef USE_ADDR64
			dmaPpuAddress2 = collisionShape0Ptr;
#else
			dmaPpuAddress2 = collisionShape0Ptr;
#endif
			cellDmaGet(collisionShape0Loc, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(1));
		
		
			dmaSize = getShapeTypeSize(collisionPairInput.m_shapeType1);
#ifdef USE_ADDR64
			dmaPpuAddress2 = collisionShape1Ptr;
#else
			dmaPpuAddress2 = collisionShape1Ptr;
#endif
			cellDmaGet(collisionShape1Loc, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);
//			cellDmaWaitTagStatusAll(DMA_MASK(2));
			cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
		

		// object 0 compound, object 1 non-compound
		btCompoundShape* spuCompoundShape = (btCompoundShape*)collisionShape0Loc;

		int childShapeCount = spuCompoundShape->getNumChildShapes();

		// dma the list of child shapes
		
			dmaSize = childShapeCount * sizeof(btCompoundShapeChild);
#ifdef USE_ADDR64
			dmaPpuAddress2 = (uint64_t)spuCompoundShape->getChildList();
#else
			dmaPpuAddress2 = (uint32_t)spuCompoundShape->getChildList();
#endif
			cellDmaGet(lsMem.gSubshapes, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
			cellDmaWaitTagStatusAll(DMA_MASK(1));
		

		for (int i = 0; i < childShapeCount; ++i)
		{
			btCompoundShapeChild& childShape = lsMem.gSubshapes[i];

			// Dma the child shape
			
				dmaSize = getShapeTypeSize(childShape.m_childShapeType);
#ifdef USE_ADDR64
				dmaPpuAddress2 = (uint64_t)childShape.m_childShape;
#else
				dmaPpuAddress2 = (uint32_t)childShape.m_childShape;
#endif
				cellDmaGet(lsMem.gSubshapeShape[i], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
				cellDmaWaitTagStatusAll(DMA_MASK(1));
			

			SpuCollisionPairInput cinput (collisionPairInput);
			cinput.m_worldTransform0 = collisionPairInput.m_worldTransform0 * childShape.m_transform;
			cinput.m_shapeType0 = childShape.m_childShapeType;
			cinput.m_collisionMargin0 = childShape.m_childMargin;

#ifdef USE_ADDR64
			handleCollisionPair(cinput, lsMem, spuContacts,			
				(uint64_t)childShape.m_childShape, lsMem.gSubshapeShape[i], 
				collisionShape1Ptr, collisionShape1Loc, false);
#else
			handleCollisionPair(cinput, lsMem, spuContacts,			
				(uint32_t)childShape.m_childShape, lsMem.gSubshapeShape[i], 
				collisionShape1Ptr, collisionShape1Loc, false);
#endif
		}
	}
	else if (btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType1) )
	{
		//snPause();
		
			dmaSize = getShapeTypeSize(collisionPairInput.m_shapeType0);
#ifdef USE_ADDR64
			dmaPpuAddress2 = collisionShape0Ptr;
#else
			dmaPpuAddress2 = collisionShape0Ptr;
#endif
			cellDmaGet(collisionShape0Loc, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(1));
		
		
			dmaSize = getShapeTypeSize(collisionPairInput.m_shapeType1);
#ifdef USE_ADDR64
			dmaPpuAddress2 = collisionShape1Ptr;
#else
			dmaPpuAddress2 = collisionShape1Ptr;
#endif
			cellDmaGet(collisionShape1Loc, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);
			//cellDmaWaitTagStatusAll(DMA_MASK(2));
			cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
		

		// object 0 non-compound, object 1 compound
		btCompoundShape* spuCompoundShape = (btCompoundShape*)collisionShape1Loc;

		int childShapeCount = spuCompoundShape->getNumChildShapes();

		// dma the list of child shapes
		
			dmaSize = childShapeCount * sizeof(btCompoundShapeChild);
#ifdef USE_ADDR64
			dmaPpuAddress2 = (uint64_t)spuCompoundShape->getChildList();
#else
			dmaPpuAddress2 = (uint32_t)spuCompoundShape->getChildList();
#endif
			cellDmaGet(lsMem.gSubshapes, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
			cellDmaWaitTagStatusAll(DMA_MASK(1));
		

		for (int i = 0; i < childShapeCount; ++i)
		{
			btCompoundShapeChild& childShape = lsMem.gSubshapes[i];

			// Dma the child shape
			
				dmaSize = getShapeTypeSize(childShape.m_childShapeType);
#ifdef USE_ADDR64
				dmaPpuAddress2 = (uint64_t)childShape.m_childShape;
#else
				dmaPpuAddress2 = (uint32_t)childShape.m_childShape;
#endif
				cellDmaGet(lsMem.gSubshapeShape[i], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
				cellDmaWaitTagStatusAll(DMA_MASK(1));
			

			SpuCollisionPairInput cinput (collisionPairInput);
			cinput.m_worldTransform1 = collisionPairInput.m_worldTransform1 * childShape.m_transform;
			cinput.m_shapeType1 = childShape.m_childShapeType;
			cinput.m_collisionMargin1 = childShape.m_childMargin;

#ifdef USE_ADDR64
			handleCollisionPair(cinput, lsMem, spuContacts,
				collisionShape0Ptr, collisionShape0Loc, 
				(uint64_t)childShape.m_childShape, lsMem.gSubshapeShape[i], false);
#else
			handleCollisionPair(cinput, lsMem, spuContacts,
				collisionShape0Ptr, collisionShape0Loc, 
				(uint32_t)childShape.m_childShape, lsMem.gSubshapeShape[i], false);
#endif
		}
		
	}
	else
	{
		//a non-convex shape is involved									
		bool handleConvexConcave = false;

		//snPause();

		if (btBroadphaseProxy::isConcave(collisionPairInput.m_shapeType0) &&
			btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType1))
		{
			// Swap stuff
			DoSwap(collisionShape0Ptr, collisionShape1Ptr);
			DoSwap(collisionShape0Loc, collisionShape1Loc);
			DoSwap(collisionPairInput.m_shapeType0, collisionPairInput.m_shapeType1);
			DoSwap(collisionPairInput.m_worldTransform0, collisionPairInput.m_worldTransform1);
			DoSwap(collisionPairInput.m_collisionMargin0, collisionPairInput.m_collisionMargin1);
			
			collisionPairInput.m_isSwapped = true;
		}
		
		if (btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType0)&&
			btBroadphaseProxy::isConcave(collisionPairInput.m_shapeType1))
		{
			handleConvexConcave = true;
		}
		if (handleConvexConcave)
		{

			if (dmaShapes)
			{
				///dma and initialize the convex object
				
					dmaSize = getShapeTypeSize(collisionPairInput.m_shapeType0);
					//uint64_t	dmaPpuAddress2 = (uint64_t)lsMem.gColObj0.getCollisionShape();
#ifdef USE_ADDR64
					dmaPpuAddress2 = collisionShape0Ptr;
#else
					dmaPpuAddress2 = collisionShape0Ptr;
#endif
					cellDmaGet(collisionShape0Loc, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
					//cellDmaWaitTagStatusAll(DMA_MASK(1));
				
				///dma and initialize the concave object
				
					dmaSize = getShapeTypeSize(collisionPairInput.m_shapeType1);
#ifdef USE_ADDR64
					dmaPpuAddress2 = collisionShape1Ptr;
#else
					dmaPpuAddress2 = collisionShape1Ptr;
#endif
					cellDmaGet(collisionShape1Loc, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);
					//cellDmaWaitTagStatusAll(DMA_MASK(2));
					cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
				
			}
			
			btConvexInternalShape* spuConvexShape0 = (btConvexInternalShape*)collisionShape0Loc;
			btBvhTriangleMeshShape* trimeshShape = (btBvhTriangleMeshShape*)collisionShape1Loc;

			btVector3 dim0 = spuConvexShape0->getImplicitShapeDimensions();
			collisionPairInput.m_primitiveDimensions0 = dim0;
			collisionPairInput.m_collisionShapes[0] = collisionShape0Ptr;
			collisionPairInput.m_collisionShapes[1] = collisionShape1Ptr;
			collisionPairInput.m_spuCollisionShapes[0] = spuConvexShape0;
			collisionPairInput.m_spuCollisionShapes[1] = trimeshShape;

			ProcessConvexConcaveSpuCollision(&collisionPairInput,&lsMem,spuContacts);
		}

	}

	spuContacts.flush();
}



void	processCollisionTask(void* userPtr, void* lsMemPtr)
{

	SpuGatherAndProcessPairsTaskDesc* taskDescPtr = (SpuGatherAndProcessPairsTaskDesc*)userPtr;
	SpuGatherAndProcessPairsTaskDesc& taskDesc = *taskDescPtr;
	CollisionTask_LocalStoreMemory*	colMemPtr = (CollisionTask_LocalStoreMemory*)lsMemPtr;
	CollisionTask_LocalStoreMemory& lsMem = *(colMemPtr);

	//	spu_printf("taskDescPtr=%llx\n",taskDescPtr);

	SpuContactResult spuContacts;

	////////////////////

	uint64_t dmaInPtr = taskDesc.inPtr;
	unsigned int numPages = taskDesc.numPages;
	unsigned int numOnLastPage = taskDesc.numOnLastPage;

	// prefetch first set of inputs and wait
	lsMem.g_workUnitTaskBuffers.init();

	unsigned int nextNumOnPage = (numPages > 1)? MIDPHASE_NUM_WORKUNITS_PER_PAGE : numOnLastPage;
	lsMem.g_workUnitTaskBuffers.backBufferDmaGet(dmaInPtr, nextNumOnPage*sizeof(SpuGatherAndProcessWorkUnitInput), DMA_TAG(3));
	dmaInPtr += MIDPHASE_WORKUNIT_PAGE_SIZE;

	
	register unsigned char *inputPtr;
	register unsigned int numOnPage;
	register unsigned int j;
	SpuGatherAndProcessWorkUnitInput* wuInputs;	
	register int dmaSize;
#ifdef USE_ADDR64
	register uint64_t	dmaPpuAddress;
	register uint64_t	dmaPpuAddress2;
#else
	register uint32_t	dmaPpuAddress;
	register uint32_t	dmaPpuAddress2;
#endif	
	int userInfo;
	int numPairs;
	register int p;
	SpuCollisionPairInput collisionPairInput;
	
	for (unsigned int i = 0; btLikely(i < numPages); i++)
	{

		// wait for back buffer dma and swap buffers
		inputPtr = lsMem.g_workUnitTaskBuffers.swapBuffers();

		// number on current page is number prefetched last iteration
		numOnPage = nextNumOnPage;


		// prefetch next set of inputs
#if MIDPHASE_NUM_WORKUNIT_PAGES > 2
		if ( btLikely( i < numPages-1 ) )
#else
		if ( btUnlikely( i < numPages-1 ) )
#endif
		{
			nextNumOnPage = (i == numPages-2)? numOnLastPage : MIDPHASE_NUM_WORKUNITS_PER_PAGE;
			lsMem.g_workUnitTaskBuffers.backBufferDmaGet(dmaInPtr, nextNumOnPage*sizeof(SpuGatherAndProcessWorkUnitInput), DMA_TAG(3));
			dmaInPtr += MIDPHASE_WORKUNIT_PAGE_SIZE;
		}

		wuInputs = reinterpret_cast<SpuGatherAndProcessWorkUnitInput *>(inputPtr);
		
		
		for (j = 0; btLikely( j < numOnPage ); j++)
		{
#ifdef DEBUG_SPU_COLLISION_DETECTION
		//	printMidphaseInput(&wuInputs[j]);
#endif //DEBUG_SPU_COLLISION_DETECTION


			numPairs = wuInputs[j].m_endIndex - wuInputs[j].m_startIndex;
			
			if ( btLikely( numPairs ) )
			{
					dmaSize = numPairs*sizeof(btBroadphasePair);
#ifdef USE_ADDR64
					dmaPpuAddress = wuInputs[j].m_pairArrayPtr+wuInputs[j].m_startIndex * sizeof(btBroadphasePair);
#else
					dmaPpuAddress = wuInputs[j].m_pairArrayPtr+wuInputs[j].m_startIndex * sizeof(btBroadphasePair);
#endif
					cellDmaGet(&lsMem.gBroadphasePairs, dmaPpuAddress  , dmaSize, DMA_TAG(1), 0, 0);
					cellDmaWaitTagStatusAll(DMA_MASK(1));
				

				for (p=0;p<numPairs;p++)
				{

					//for each broadphase pair, do something

					btBroadphasePair& pair = lsMem.gBroadphasePairs[p];
#ifdef DEBUG_SPU_COLLISION_DETECTION
					spu_printf("pair->m_userInfo = %d\n",pair.m_userInfo);
					spu_printf("pair->m_algorithm = %d\n",pair.m_algorithm);
					spu_printf("pair->m_pProxy0 = %d\n",pair.m_pProxy0);
					spu_printf("pair->m_pProxy1 = %d\n",pair.m_pProxy1);
#endif //DEBUG_SPU_COLLISION_DETECTION

					userInfo = int(pair.m_userInfo);

					if (userInfo == 2 && pair.m_algorithm && pair.m_pProxy0 && pair.m_pProxy1)
					{


						
							dmaSize = sizeof(SpuContactManifoldCollisionAlgorithm);
#ifdef USE_ADDR64
							dmaPpuAddress2 = (uint64_t)pair.m_algorithm;
#else
							dmaPpuAddress2 = (uint32_t)pair.m_algorithm;
#endif
							cellDmaGet(&lsMem.gSpuContactManifoldAlgo, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
							//cellDmaWaitTagStatusAll(DMA_MASK(1));
						



						//snPause();

#ifdef DEBUG_SPU_COLLISION_DETECTION
						//spu_printf("SPU: manifoldPtr: %llx",collisionPairInput->m_persistentManifoldPtr);
#endif //DEBUG_SPU_COLLISION_DETECTION

						
						dmaSize = sizeof(btBroadphaseProxy);
#ifdef USE_ADDR64
						dmaPpuAddress2 = (uint64_t)pair.m_pProxy0;
#else
						dmaPpuAddress2 = (uint32_t)pair.m_pProxy0;
#endif							
						lsMem.gProxyPtr0 = (btBroadphaseProxy*) lsMem.bufferProxy0;
						stallingUnalignedDmaSmallGet(lsMem.gProxyPtr0, dmaPpuAddress2  , dmaSize);

						collisionPairInput.m_persistentManifoldPtr = (uint64_t) lsMem.gSpuContactManifoldAlgo.getContactManifoldPtr();
						collisionPairInput.m_isSwapped = false;
						
						
						dmaSize = sizeof(btBroadphaseProxy);
#ifdef USE_ADDR64
						dmaPpuAddress2 = (uint64_t)pair.m_pProxy1;
#else
						dmaPpuAddress2 = (uint32_t)pair.m_pProxy1;
#endif							
						lsMem.gProxyPtr1 = (btBroadphaseProxy*) lsMem.bufferProxy1;
						stallingUnalignedDmaSmallGet(lsMem.gProxyPtr1, dmaPpuAddress2  , dmaSize);
						

						//btCollisionObject* colObj0 = (btCollisionObject*)gProxy0.m_clientObject;
						//btCollisionObject* colObj1 = (btCollisionObject*)gProxy1.m_clientObject;
						

						if (1)
						{

							///can wait on the combined DMA_MASK, or dma on the same tag


#ifdef DEBUG_SPU_COLLISION_DETECTION
					//		spu_printf("SPU collisionPairInput->m_shapeType0 = %d\n",collisionPairInput->m_shapeType0);
					//		spu_printf("SPU collisionPairInput->m_shapeType1 = %d\n",collisionPairInput->m_shapeType1);
#endif //DEBUG_SPU_COLLISION_DETECTION

							
							dmaSize = sizeof(btPersistentManifold);

							dmaPpuAddress2 = collisionPairInput.m_persistentManifoldPtr;
							cellDmaGet(&lsMem.gPersistentManifold, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);

							collisionPairInput.m_shapeType0 = lsMem.gSpuContactManifoldAlgo.getShapeType0();
							collisionPairInput.m_shapeType1 = lsMem.gSpuContactManifoldAlgo.getShapeType1();
							collisionPairInput.m_collisionMargin0 = lsMem.gSpuContactManifoldAlgo.getCollisionMargin0();
							collisionPairInput.m_collisionMargin1 = lsMem.gSpuContactManifoldAlgo.getCollisionMargin1();
							
							
							
							cellDmaWaitTagStatusAll(DMA_MASK(1));
							

							if (1)
							{
								//snPause();

								// Get the collision objects
								dmaAndSetupCollisionObjects(collisionPairInput, lsMem);
#ifdef USE_ADDR64
								handleCollisionPair(collisionPairInput, lsMem, spuContacts, 
									(uint64_t)lsMem.getColObj0()->getCollisionShape(), lsMem.gCollisionShape0,
									(uint64_t)lsMem.getColObj1()->getCollisionShape(), lsMem.gCollisionShape1);
#else
								handleCollisionPair(collisionPairInput, lsMem, spuContacts, 
									(uint32_t)lsMem.getColObj0()->getCollisionShape(), lsMem.gCollisionShape0,
									(uint32_t)lsMem.getColObj1()->getCollisionShape(), lsMem.gCollisionShape1);
#endif
							}		
						}

					}
				}
			}
		} //end for (j = 0; j < numOnPage; j++)

	}//	for 


	return;
}
