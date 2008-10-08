#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btOptimizedBvh.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btConvexPointCloudShape.h"


#include "SpuGatheringCollisionTask.h"

//#define DEBUG_SPU_COLLISION_DETECTION 1
#include "../SpuDoubleBuffer.h"

#include "../SpuCollisionTaskProcess.h"
#include "../SpuGatheringCollisionDispatcher.h" //for SPU_BATCHSIZE_BROADPHASE_PAIRS

#include "../SpuContactManifoldCollisionAlgorithm.h"
#include "SpuContactResult.h"

#include "SpuCollisionShapes.h" //definition of SpuConvexPolyhedronVertexData

#ifdef __SPU__
///Software caching from the IBM Cell SDK, it reduces 25% SPU time for our test cases
#ifndef USE_LIBSPE2
#define USE_SOFTWARE_CACHE 1
#endif
#endif //__SPU__

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

bool gUseEpa = false;

#ifdef USE_SN_TUNER
#include <LibSN_SPU.h>
#endif //USE_SN_TUNER

#if defined (__CELLOS_LV2__) || defined (USE_LIBSPE2)
#else
#define IGNORE_ALIGNMENT 1
#define spu_printf printf
#include <stdio.h>
#endif

//int gNumConvexPoints0=0;

///Make sure no destructors are called on this memory
struct	CollisionTask_LocalStoreMemory
{
	ATTRIBUTE_ALIGNED16(char bufferProxy0[16]);
	ATTRIBUTE_ALIGNED16(char bufferProxy1[16]);
	ATTRIBUTE_ALIGNED16(btBroadphaseProxy*	gProxyPtr0);
	ATTRIBUTE_ALIGNED16(btBroadphaseProxy*	gProxyPtr1);
	btBroadphaseProxy* getProxyPtr0 ()
	{
		return (btBroadphaseProxy*)bufferProxy0;
	}
	btBroadphaseProxy* getProxyPtr1 ()
	{
		return (btBroadphaseProxy*)bufferProxy1;
	}
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
	SpuContactManifoldCollisionAlgorithm	gSpuContactManifoldAlgo;
	SpuContactManifoldCollisionAlgorithm*	getlocalCollisionAlgorithm()
	{
		return (SpuContactManifoldCollisionAlgorithm*)&gSpuContactManifoldAlgo;

	}
	btPersistentManifold gPersistentManifold;
	SpuInternalShape gInternalShapes[3]; // the third is temporary storage
	SpuInternalConvexHull gInternalConvexHull[3]; // the third is for temporary storage

	///we reserve 32bit integer indices, even though they might be 16bit
	ATTRIBUTE_ALIGNED16(int	spuIndices[16]);
	SpuBvhMeshShape gBvhMeshShape;
	SpuCompoundShape compoundShapeData[2];
};

static void print_size_stats ()
{
	spu_printf("sizeof(CollisionTask_LocalStoreMemory) = %d\n", sizeof(CollisionTask_LocalStoreMemory));
	spu_printf("sizeof(SpuInternalShape) = %d\n", sizeof(SpuInternalShape));
	spu_printf("sizeof(SpuInternalConvexHull) = %d\n", sizeof(SpuInternalConvexHull));
	spu_printf("sizeof(SpuBvhMeshShape) = %d\n", sizeof(SpuBvhMeshShape));
}

#if defined(__CELLOS_LV2__) || defined(USE_LIBSPE2) 

ATTRIBUTE_ALIGNED16(CollisionTask_LocalStoreMemory	gLocalStoreMemory);

void* createCollisionLocalStoreMemory()
{
	//print_size_stats ();
	return &gLocalStoreMemory;
}
#else
void* createCollisionLocalStoreMemory()
{
        return new CollisionTask_LocalStoreMemory;
};

#endif

void	ProcessSpuConvexConvexCollision(SpuCollisionPairInput* wuInput, CollisionTask_LocalStoreMemory* lsMemPtr, SpuContactResult& spuContacts);


SIMD_FORCE_INLINE void small_cache_read(void* buffer, ppu_address_t ea, size_t size)
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

SIMD_FORCE_INLINE void small_cache_read_triple(	void* ls0, ppu_address_t ea0,
												void* ls1, ppu_address_t ea1,
												void* ls2, ppu_address_t ea2,
												size_t size)
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
#ifdef __SPU__
		cellDmaSmallGet(tmpTarget0,ea0,size,DMA_TAG(1),0,0);
#else
		tmpTarget0 = (char*)cellDmaSmallGetReadOnly(tmpTarget0,ea0,size,DMA_TAG(1),0,0);
#endif


		char* localStore1 = (char*)ls1;
		last4BitsOffset = ea1 & 0x0f;
		char* tmpTarget1 = tmpBuffer1 + last4BitsOffset;
#ifdef __SPU__
		cellDmaSmallGet(tmpTarget1,ea1,size,DMA_TAG(1),0,0);
#else
		tmpTarget1 = (char*)cellDmaSmallGetReadOnly(tmpTarget1,ea1,size,DMA_TAG(1),0,0);
#endif
		
		char* localStore2 = (char*)ls2;
		last4BitsOffset = ea2 & 0x0f;
		char* tmpTarget2 = tmpBuffer2 + last4BitsOffset;
#ifdef __SPU__
		cellDmaSmallGet(tmpTarget2,ea2,size,DMA_TAG(1),0,0);
#else
		tmpTarget2 = (char*)cellDmaSmallGetReadOnly(tmpTarget2,ea2,size,DMA_TAG(1),0,0);
#endif
		
		
		cellDmaWaitTagStatusAll( DMA_MASK(1) );

		//this is slowish, perhaps memcpy on SPU is smarter?
		for (i=0; btLikely( i<size );i++)
		{
			localStore0[i] = tmpTarget0[i];
			localStore1[i] = tmpTarget1[i];
			localStore2[i] = tmpTarget2[i];
		}

		
}



#if 0
////////////////////////
/// Convex versus Concave triangle mesh collision detection (handles concave triangle mesh versus sphere, box, cylinder, triangle, cone, convex polyhedron etc)
///////////////////
void	ProcessConvexConcaveSpuCollision(SpuCollisionPairInput* wuInput, CollisionTask_LocalStoreMemory* lsMemPtr, SpuContactResult& spuContacts)
{
	//order: first collision shape is convex, second concave. m_isSwapped is true, if the original order was opposite
	register int dmaSize;
	register ppu_address_t	dmaPpuAddress2;

	btBvhTriangleMeshShape*	trimeshShape = (btBvhTriangleMeshShape*)wuInput->m_spuCollisionShapes[1];
	//need the mesh interface, for access to triangle vertices
	dmaBvhShapeData (&lsMemPtr->bvhShapeData, trimeshShape);

	btVector3 aabbMin(-1,-400,-1);
	btVector3 aabbMax(1,400,1);


	//recalc aabbs
	btTransform convexInTriangleSpace;
	convexInTriangleSpace = wuInput->m_worldTransform1.inverse() * wuInput->m_worldTransform0;
	btConvexInternalShape* convexShape = (btConvexInternalShape*)wuInput->m_spuCollisionShapes[0];

	computeAabb (aabbMin, aabbMax, convexShape, wuInput->m_collisionShapes[0], wuInput->m_shapeType0, convexInTriangleSpace);


	//CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody->m_collisionShape);
	//convexShape->getAabb(convexInTriangleSpace,m_aabbMin,m_aabbMax);

	//	btScalar extraMargin = collisionMarginTriangle;
	//	btVector3 extra(extraMargin,extraMargin,extraMargin);
	//	aabbMax += extra;
	//	aabbMin -= extra;

	///quantize query AABB
	unsigned short int quantizedQueryAabbMin[3];
	unsigned short int quantizedQueryAabbMax[3];
	lsMemPtr->bvhShapeData.getOptimizedBvh()->quantizeWithClamp(quantizedQueryAabbMin,aabbMin,0);
	lsMemPtr->bvhShapeData.getOptimizedBvh()->quantizeWithClamp(quantizedQueryAabbMax,aabbMax,1);

	QuantizedNodeArray&	nodeArray = lsMemPtr->bvhShapeData.getOptimizedBvh()->getQuantizedNodeArray();
	//spu_printf("SPU: numNodes = %d\n",nodeArray.size());

	BvhSubtreeInfoArray& subTrees = lsMemPtr->bvhShapeData.getOptimizedBvh()->getSubtreeInfoArray();


	spuNodeCallback	nodeCallback(wuInput,lsMemPtr,spuContacts);
	IndexedMeshArray&	indexArray = lsMemPtr->bvhShapeData.gTriangleMeshInterfacePtr->getIndexedMeshArray();
	//spu_printf("SPU:indexArray.size() = %d\n",indexArray.size());

	//	spu_printf("SPU: numSubTrees = %d\n",subTrees.size());
	//not likely to happen
	if (subTrees.size() && indexArray.size() == 1)
	{
		///DMA in the index info
		dmaBvhIndexedMesh (&lsMemPtr->bvhShapeData.gIndexMesh, indexArray, 0 /* index into indexArray */, 1 /* dmaTag */);
		cellDmaWaitTagStatusAll(DMA_MASK(1));
		
		//display the headers
		int numBatch = subTrees.size();
		for (int i=0;i<numBatch;)
		{
// BEN: TODO - can reorder DMA transfers for less stall
			int remaining = subTrees.size() - i;
			int nextBatch = remaining < MAX_SPU_SUBTREE_HEADERS ? remaining : MAX_SPU_SUBTREE_HEADERS;
			
			dmaBvhSubTreeHeaders (&lsMemPtr->bvhShapeData.gSubtreeHeaders[0], (ppu_address_t)(&subTrees[i]), nextBatch, 1);
			cellDmaWaitTagStatusAll(DMA_MASK(1));
			

			//			spu_printf("nextBatch = %d\n",nextBatch);

			for (int j=0;j<nextBatch;j++)
			{
				const btBvhSubtreeInfo& subtree = lsMemPtr->bvhShapeData.gSubtreeHeaders[j];

				unsigned int overlap = spuTestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
				if (overlap)
				{
					btAssert(subtree.m_subtreeSize);

					//dma the actual nodes of this subtree
					dmaBvhSubTreeNodes (&lsMemPtr->bvhShapeData.gSubtreeNodes[0], subtree, nodeArray, 2);
					cellDmaWaitTagStatusAll(DMA_MASK(2));

					/* Walk this subtree */
					spuWalkStacklessQuantizedTree(&nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,
						&lsMemPtr->bvhShapeData.gSubtreeNodes[0],
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
#endif
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
	register ppu_address_t	dmaPpuAddress2;
		
	dmaSize = sizeof(btCollisionObject);
	dmaPpuAddress2 = /*collisionPairInput.m_isSwapped ? (ppu_address_t)lsMem.gProxyPtr1->m_clientObject :*/ (ppu_address_t)lsMem.gProxyPtr0->m_clientObject;
	cellDmaGet(&lsMem.gColObj0, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);		

	dmaSize = sizeof(btCollisionObject);
	dmaPpuAddress2 = /*collisionPairInput.m_isSwapped ? (ppu_address_t)lsMem.gProxyPtr0->m_clientObject :*/ (ppu_address_t)lsMem.gProxyPtr1->m_clientObject;
	cellDmaGet(&lsMem.gColObj1, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);		
	
	cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));

	collisionPairInput.m_worldTransform0 = lsMem.getColObj0()->getWorldTransform();
	collisionPairInput.m_worldTransform1 = lsMem.getColObj1()->getWorldTransform();
}


void SpuConvexConvexCollisionAlgorithm (SpuCollisionPairInput& collisionPairInput,
									    CollisionTask_LocalStoreMemory& lsMem,
									    SpuContactResult &spuContacts,
									    SpuInternalShape* shape0Internal,
									    SpuInternalShape* shape1Internal,
									    SpuInternalConvexHull* shape0Points,
									    SpuInternalConvexHull* shape1Points)
{
	btConvexPointCloudShape shape0ConvexHull (NULL, 0, false);
	btConvexPointCloudShape shape1ConvexHull (NULL, 0, false);
	btConvexShape* shape0;
	btConvexShape* shape1;
	btPersistentManifold* manifold = (btPersistentManifold*)collisionPairInput.m_persistentManifoldPtr;

#ifdef DEBUG_SPU_COLLISION_DETECTION
	spu_printf("SpuConvexConvexCollisionAlgorithm ()\n");
#endif
	if (collisionPairInput.m_shapeType0== CONVEX_HULL_SHAPE_PROXYTYPE )
	{
		shape0ConvexHull.setPoints (shape0Points->m_points, shape0Points->m_numPoints, false);
		{
			btVector3 aabbMin, aabbMax;
			btPolyhedralConvexShape* pcs = (btPolyhedralConvexShape*)shape0Internal->m_convexShape;
			pcs->getCachedLocalAabb (aabbMin, aabbMax);
			shape0ConvexHull.setCachedLocalAabb (aabbMin, aabbMax);
		}
		shape0 = &shape0ConvexHull;
	} else {
		shape0 = shape0Internal->m_convexShape;
	}

	if (collisionPairInput.m_shapeType1 == CONVEX_HULL_SHAPE_PROXYTYPE )
	{
		shape1ConvexHull.setPoints (shape1Points->m_points, shape1Points->m_numPoints, false);
		{
			btVector3 aabbMin, aabbMax;
			btPolyhedralConvexShape* pcs = (btPolyhedralConvexShape*)shape1Internal->m_convexShape;
			pcs->getCachedLocalAabb (aabbMin, aabbMax);
			shape1ConvexHull.setCachedLocalAabb (aabbMin, aabbMax);
		}
		shape1 = &shape1ConvexHull;
	} else {
		shape1 = shape1Internal->m_convexShape;
	}

#ifdef DEBUG_SPU_COLLISION_DETECTION
	spu_printf("shape0 = %p shape1 = %p\n", shape0, shape1);
#endif
	{
		btVoronoiSimplexSolver vsSolver;
		btConvexPenetrationDepthSolver* penetrationSolver = NULL;
//#define SPU_ENABLE_EPA 1
#ifdef SPU_ENABLE_EPA
		btGjkEpaPenetrationDepthSolver epaPenetrationSolver;
		btMinkowskiPenetrationDepthSolver	minkowskiPenetrationSolver;
		if (gUseEpa)
		{
			penetrationSolver = &epaPenetrationSolver;
		} else {
			penetrationSolver = &minkowskiPenetrationSolver;
		}
#else
		btMinkowskiPenetrationDepthSolver	minkowskiPenetrationSolver;
		penetrationSolver = &minkowskiPenetrationSolver;
#endif
		btDiscreteCollisionDetectorInterface::ClosestPointInput cpInput;
		cpInput.m_transformA = collisionPairInput.m_worldTransform0;
		cpInput.m_transformB = collisionPairInput.m_worldTransform1;
		btPersistentManifold* spuManifold= &lsMem.gPersistentManifold;
		ppu_address_t manifoldAddress = (ppu_address_t)manifold;
		spuContacts.setContactInfo(spuManifold,manifoldAddress,
			lsMem.getColObj0()->getWorldTransform(),
			lsMem.getColObj1()->getWorldTransform(),
			lsMem.getColObj0()->getRestitution(),
			lsMem.getColObj1()->getRestitution(),
			lsMem.getColObj0()->getFriction(),
			lsMem.getColObj1()->getFriction(),
			collisionPairInput.m_isSwapped);

#ifdef DEBUG_SPU_COLLISION_DETECTION
	spu_printf("start GJK\n");
#endif
		btGjkPairDetector gjk(shape0,shape1,&vsSolver,penetrationSolver);
		gjk.getClosestPoints(cpInput,spuContacts, NULL);//,debugDraw);
#ifdef DEBUG_SPU_COLLISION_DETECTION
	spu_printf("stop GJK\n");
#endif
	}	
}

class spuNodeCallback : public btNodeOverlapCallback
{
	const SpuCollisionPairInput& m_collisionPairInput;
	CollisionTask_LocalStoreMemory&	m_lsMem;
	SpuContactResult& m_spuContacts;

	ATTRIBUTE_ALIGNED16(btVector3	spuTriangleVertices[3]);
	ATTRIBUTE_ALIGNED16(btScalar	spuUnscaledVertex[4]);
public:
	spuNodeCallback(const SpuCollisionPairInput& collisionPairInput,
		            CollisionTask_LocalStoreMemory& lsMem,
					SpuContactResult& spuContacts)
		: m_collisionPairInput(collisionPairInput),
		m_lsMem(lsMem),
		m_spuContacts(spuContacts)
	{
	}

	virtual void processNode(int subPart, int triangleIndex)
	{
		///Create a triangle on the stack, call process collision, with GJK
		///DMA the vertices, can benefit from software caching

		//spu_printf("processNode with triangleIndex %d\n",triangleIndex);

		///TODO: add switch between short int, and int indices, based on indexType

		// ugly solution to support both 16bit and 32bit indices
		if (m_lsMem.gBvhMeshShape.m_indexMesh.m_indexType == PHY_SHORT)
		{
			unsigned short int* indexBasePtr = (unsigned short int*)(m_lsMem.gBvhMeshShape.m_indexMesh.m_triangleIndexBase+triangleIndex*m_lsMem.gBvhMeshShape.m_indexMesh.m_triangleIndexStride);
			ATTRIBUTE_ALIGNED16(unsigned short int tmpIndices[3]);

			small_cache_read_triple(&tmpIndices[0],(ppu_address_t)&indexBasePtr[0],
									&tmpIndices[1],(ppu_address_t)&indexBasePtr[1],
									&tmpIndices[2],(ppu_address_t)&indexBasePtr[2],
									sizeof(unsigned short int));

			m_lsMem.spuIndices[0] = int(tmpIndices[0]);
			m_lsMem.spuIndices[1] = int(tmpIndices[1]);
			m_lsMem.spuIndices[2] = int(tmpIndices[2]);
		} else
		{
			unsigned int* indexBasePtr = (unsigned int*)(m_lsMem.gBvhMeshShape.m_indexMesh.m_triangleIndexBase+triangleIndex*m_lsMem.gBvhMeshShape.m_indexMesh.m_triangleIndexStride);

			small_cache_read_triple(&m_lsMem.spuIndices[0],(ppu_address_t)&indexBasePtr[0],
								&m_lsMem.spuIndices[1],(ppu_address_t)&indexBasePtr[1],
								&m_lsMem.spuIndices[2],(ppu_address_t)&indexBasePtr[2],
								sizeof(int));
		}
		
		//spu_printf("SPU index0=%d ,",spuIndices[0]);
		//spu_printf("SPU index1=%d ,",spuIndices[1]);
		//spu_printf("SPU index2=%d ,",spuIndices[2]);
		//spu_printf("SPU: indexBasePtr=%llx\n",indexBasePtr);

		const btVector3& meshScaling = m_lsMem.gBvhMeshShape.m_triangleMeshInterface->getScaling();
		for (int j=2;btLikely( j>=0 );j--)
		{
			int graphicsindex = m_lsMem.spuIndices[j];

			//spu_printf("SPU index=%d ,",graphicsindex);
			btScalar* graphicsbasePtr = (btScalar*)(m_lsMem.gBvhMeshShape.m_indexMesh.m_vertexBase+graphicsindex*m_lsMem.gBvhMeshShape.m_indexMesh.m_vertexStride);
			//spu_printf("SPU graphicsbasePtr=%llx\n",graphicsbasePtr);


			///handle un-aligned vertices...

			//another DMA for each vertex
			small_cache_read_triple(&spuUnscaledVertex[0],(ppu_address_t)&graphicsbasePtr[0],
									&spuUnscaledVertex[1],(ppu_address_t)&graphicsbasePtr[1],
									&spuUnscaledVertex[2],(ppu_address_t)&graphicsbasePtr[2],
									sizeof(btScalar));
			
			spuTriangleVertices[j] = btVector3(
				spuUnscaledVertex[0]*meshScaling.getX(),
				spuUnscaledVertex[1]*meshScaling.getY(),
				spuUnscaledVertex[2]*meshScaling.getZ());

			//spu_printf("SPU:triangle vertices:%f,%f,%f\n",spuTriangleVertices[j].x(),spuTriangleVertices[j].y(),spuTriangleVertices[j].z());
		}

		btTriangleShape tmpTriangleShape (spuTriangleVertices[0], spuTriangleVertices[1], spuTriangleVertices[2]);

		SpuCollisionPairInput triangleConcaveInput(m_collisionPairInput);
		triangleConcaveInput.m_shapeType1 = TRIANGLE_SHAPE_PROXYTYPE;
		m_spuContacts.setShapeIdentifiers(-1,-1,subPart,triangleIndex);

		//m_spuContacts.flush();

		m_lsMem.gInternalShapes[2].m_collisionShape = &tmpTriangleShape;
		m_lsMem.gInternalShapes[2].m_convexShape = &tmpTriangleShape;

		SpuConvexConvexCollisionAlgorithm (triangleConcaveInput,
										   m_lsMem,
										   m_spuContacts,
										   &m_lsMem.gInternalShapes[0],
										   &m_lsMem.gInternalShapes[2],
										   &m_lsMem.gInternalConvexHull[0],
										   NULL);
		///this flush should be automatic
		//m_spuContacts.flush();
	}

};

void SpuConvexConcaveCollisionAlgorithm (SpuCollisionPairInput& collisionPairInput,
									     CollisionTask_LocalStoreMemory& lsMem,
									     SpuContactResult &spuContacts,
									     SpuInternalShape* shape0Internal,
									     SpuInternalShape* shape1Internal,
									     SpuInternalConvexHull* shape0Points,
									     SpuInternalConvexHull* shape1Points)
{
	btConvexPointCloudShape shape0ConvexHull (NULL, 0, false);
	btConvexShape* shape0;

#ifdef DEBUG_SPU_COLLISION_DETECTION
	spu_printf("SpuConvexConcaveCollisionAlgorithm ()\n");
#endif
	if (collisionPairInput.m_shapeType0== CONVEX_HULL_SHAPE_PROXYTYPE )
	{
		shape0ConvexHull.setPoints (shape0Points->m_points, shape0Points->m_numPoints, false);
		{
			btVector3 aabbMin, aabbMax;
			btPolyhedralConvexShape* pcs = (btPolyhedralConvexShape*)shape0Internal->m_convexShape;
			pcs->getCachedLocalAabb (aabbMin, aabbMax);
			shape0ConvexHull.setCachedLocalAabb (aabbMin, aabbMax);
		}
		shape0 = &shape0ConvexHull;
	} else {
		shape0 = shape0Internal->m_convexShape;
	}

	SpuBvhMeshShape* bvhShape = &lsMem.gBvhMeshShape;

	bvhShape->dmaMeshInterfaceAndOptimizedBvh (*shape1Internal, 1, 2);
	cellDmaWaitTagStatusAll (DMA_MASK(1) | DMA_MASK(2));

	// determine aabb of convex shape in triangle mesh
	btVector3 aabbMin;
	btVector3 aabbMax;
	btTransform convexInTriangleSpace;
	convexInTriangleSpace = collisionPairInput.m_worldTransform1.inverse() * collisionPairInput.m_worldTransform0;
	shape0->getAabbNonVirtual (convexInTriangleSpace, aabbMin, aabbMax);

	//	btScalar extraMargin = collisionMarginTriangle;
	//	btVector3 extra(extraMargin,extraMargin,extraMargin);
	//	aabbMax += extra;
	//	aabbMin -= extra;

	//quantize query AABB
	unsigned short int quantizedQueryAabbMin[3];
	unsigned short int quantizedQueryAabbMax[3];
	bvhShape->m_optimizedBvh->quantizeWithClamp(quantizedQueryAabbMin,aabbMin,0);
	bvhShape->m_optimizedBvh->quantizeWithClamp(quantizedQueryAabbMax,aabbMax,1);

	QuantizedNodeArray&	nodeArray = bvhShape->m_optimizedBvh->getQuantizedNodeArray();
	//spu_printf("SPU: numNodes = %d\n",nodeArray.size());

	BvhSubtreeInfoArray& subTrees = bvhShape->m_optimizedBvh->getSubtreeInfoArray();
	spuNodeCallback	nodeCallback(collisionPairInput,lsMem,spuContacts);
	IndexedMeshArray& indexArray = bvhShape->m_triangleMeshInterface->getIndexedMeshArray();
	//spu_printf("SPU:indexArray.size() = %d\n",indexArray.size());
	//spu_printf("SPU: numSubTrees = %d\n",subTrees.size());
	if (subTrees.size() && indexArray.size() == 1)
	{
		//DMA in the index info, we only support meshes with a single index (hence the '0')
		bvhShape->dmaIndexedMesh (0, 1);
		cellDmaWaitTagStatusAll(DMA_MASK(1));
		
		//display the headers
		int numBatch = subTrees.size();
		for (int i=0;i<numBatch;)
		{
// BEN: TODO - can reorder DMA transfers for less stall
			int remaining = subTrees.size() - i;
			int nextBatch = remaining < MAX_SPU_SUBTREE_HEADERS ? remaining : MAX_SPU_SUBTREE_HEADERS;

			bvhShape->dmaSubTreeHeaders ((ppu_address_t)(&subTrees[i]), nextBatch, 1);
			cellDmaWaitTagStatusAll(DMA_MASK(1));
			

			//			spu_printf("nextBatch = %d\n",nextBatch);

			for (int j=0;j<nextBatch;j++)
			{
				const btBvhSubtreeInfo& subtree = bvhShape->m_subtreeHeaders[j];

				unsigned int overlap = spuTestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
				if (overlap)
				{
					btAssert(subtree.m_subtreeSize);

					//dma the actual nodes of this subtree
					bvhShape->dmaSubTreeNodes (subtree, nodeArray, 2);
					cellDmaWaitTagStatusAll(DMA_MASK(2));

					/* Walk this subtree */
					spuWalkStacklessQuantizedTree(&nodeCallback,
												  quantizedQueryAabbMin,
												  quantizedQueryAabbMax,
												  &bvhShape->m_subtreeNodes[0],
												  0,subtree.m_subtreeSize);
						
				}
				//				spu_printf("subtreeSize = %d\n",gSubtreeHeaders[j].m_subtreeSize);
			}
			i+=nextBatch;
		}
		//pre-fetch first tree, then loop and double buffer
	}

}

void	handleCollisionPair(SpuCollisionPairInput& collisionPairInput, CollisionTask_LocalStoreMemory& lsMem,
							SpuContactResult &spuContacts,
							ppu_address_t collisionShape0Ptr,
							ppu_address_t collisionShape1Ptr, bool dmaShapes = true)
{
#ifdef DEBUG_SPU_COLLISION_DETECTION
	spu_printf("handleCollisionPair ()\n");
#endif
	if (btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType0) 
		&& btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType1))
	{
		if (dmaShapes)
		{
			/* Don't load the same shape twice. */
			if (collisionShape0Ptr != lsMem.gInternalShapes[0].m_ppuConvexShapePtr)
			{
				lsMem.gInternalShapes[0].dmaShapeData (collisionShape0Ptr, collisionPairInput.m_shapeType0, 1);
			}

			/* Don't load the same shape twice. */
			if (collisionShape1Ptr != lsMem.gInternalShapes[1].m_ppuConvexShapePtr)
			{
				lsMem.gInternalShapes[1].dmaShapeData (collisionShape1Ptr, collisionPairInput.m_shapeType1, 2);
			} 

			if (collisionPairInput.m_shapeType0 == CONVEX_HULL_SHAPE_PROXYTYPE)
			{
				btConvexHullShape* convexHull = (btConvexHullShape*)lsMem.gInternalShapes[0].m_convexShape;
				cellDmaWaitTagStatusAll (DMA_MASK(1));
				/* Don't load the same verts twice */
				if ((ppu_address_t)convexHull->getPoints() != lsMem.gInternalConvexHull[0].m_ppuPointsPtr)
				{
					lsMem.gInternalConvexHull[0].dmaPointsData (lsMem.gInternalShapes[0], 1);
				}
			}

			if (collisionPairInput.m_shapeType1 == CONVEX_HULL_SHAPE_PROXYTYPE)
			{
				btConvexHullShape* convexHull = (btConvexHullShape*)lsMem.gInternalShapes[1].m_convexShape;
				cellDmaWaitTagStatusAll (DMA_MASK(2));
				/* Don't load the same verts twice */
				if ((ppu_address_t)convexHull->getPoints() != lsMem.gInternalConvexHull[1].m_ppuPointsPtr)
				{
					lsMem.gInternalConvexHull[1].dmaPointsData (lsMem.gInternalShapes[1], 2);
				} 
			}

			cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));

#if 0
			if (collisionPairInput.m_shapeType0 == CONVEX_HULL_SHAPE_PROXYTYPE)
			{
				for (int i = 0; i < lsMem.gInternalConvexHull[0].m_numPoints; i++)
				{
					btVector3 vtx = lsMem.gInternalConvexHull[0].m_points[i];
					spu_printf("%d %f %f %f\n", i, vtx.getX(), vtx.getY(), vtx.getZ());
				}
			}

			if (collisionPairInput.m_shapeType1 == CONVEX_HULL_SHAPE_PROXYTYPE)
			{
				for (int i = 0; i < lsMem.gInternalConvexHull[1].m_numPoints; i++)
				{
					btVector3 vtx = lsMem.gInternalConvexHull[1].m_points[i];
					spu_printf("%d %f %f %f\n", i, vtx.getX(), vtx.getY(), vtx.getZ());
				}
			}
#endif
		}

		SpuConvexConvexCollisionAlgorithm (collisionPairInput, lsMem, spuContacts,
									       &lsMem.gInternalShapes[0],
										   &lsMem.gInternalShapes[1],
										   &lsMem.gInternalConvexHull[0],
										   &lsMem.gInternalConvexHull[1]);
	} 
	else if (btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType0) && 
			btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType1))
	{
		//snPause();

		/* Don't load the same shape twice. */
		if (collisionShape0Ptr != lsMem.gInternalShapes[0].m_ppuConvexShapePtr)
		{
			lsMem.gInternalShapes[0].dmaShapeData (collisionShape0Ptr, collisionPairInput.m_shapeType0, 1);
		}

		/* Don't load the same shape twice. */
		if (collisionShape1Ptr != lsMem.gInternalShapes[1].m_ppuConvexShapePtr)
		{
			lsMem.gInternalShapes[1].dmaShapeData (collisionShape1Ptr, collisionPairInput.m_shapeType1, 2);
		} 

		cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));

		// Both are compounds, do N^2 CD for now
		// TODO: add some AABB-based pruning
	
		btCompoundShape* spuCompoundShape0 = (btCompoundShape*)lsMem.gInternalShapes[0].m_collisionShape;
		btCompoundShape* spuCompoundShape1 = (btCompoundShape*)lsMem.gInternalShapes[1].m_collisionShape;

		lsMem.compoundShapeData[0].dmaChildShapeInfo (spuCompoundShape0, 1);
		lsMem.compoundShapeData[1].dmaChildShapeInfo (spuCompoundShape1, 2);
		cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));

		int childShapeCount0 = spuCompoundShape0->getNumChildShapes();
		int childShapeCount1 = spuCompoundShape1->getNumChildShapes();

		// Start the N^2
		for (int i = 0; i < childShapeCount0; ++i)
		{
			btCompoundShapeChild& childShape0 = lsMem.compoundShapeData[0].m_subshapes[i];
			// dma childshape0 into gInternalShapes 0 (this overwrites the original compound shape)
			lsMem.compoundShapeData[0].dmaChildShape (i, &lsMem.gInternalShapes[0], &lsMem.gInternalConvexHull[0], 1);
			for (int j = 0; j < childShapeCount1; ++j)
			{
				btCompoundShapeChild& childShape1 = lsMem.compoundShapeData[1].m_subshapes[j];
				lsMem.compoundShapeData[1].dmaChildShape (j, &lsMem.gInternalShapes[1], &lsMem.gInternalConvexHull[1], 2);

				/* Create a new collision pair input struct using the two child shapes */
				SpuCollisionPairInput cinput (collisionPairInput);

				cinput.m_worldTransform0 = collisionPairInput.m_worldTransform0 * childShape0.m_transform;
				cinput.m_shapeType0 = childShape0.m_childShapeType;
				cinput.m_collisionMargin0 = childShape0.m_childMargin;

				cinput.m_worldTransform1 = collisionPairInput.m_worldTransform1 * childShape1.m_transform;
				cinput.m_shapeType1 = childShape1.m_childShapeType;
				cinput.m_collisionMargin1 = childShape1.m_childMargin;
				
				cellDmaWaitTagStatusAll (DMA_MASK(1) | DMA_MASK(2));

				/* Recursively call handleCollisionPair () with new collision pair input */
				handleCollisionPair(cinput, lsMem, spuContacts,			
					(ppu_address_t)childShape0.m_childShape, 
					(ppu_address_t)childShape1.m_childShape,false); // bug fix: changed index to j.
			}
		}
	}
	else if (btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType0) )
	{
		/* Don't load the same shape twice. */
		if (collisionShape0Ptr != lsMem.gInternalShapes[0].m_ppuConvexShapePtr)
		{
			lsMem.gInternalShapes[0].dmaShapeData (collisionShape0Ptr, collisionPairInput.m_shapeType0, 1);
		}

		/* Don't load the same shape twice. */
		if (collisionShape1Ptr != lsMem.gInternalShapes[1].m_ppuConvexShapePtr)
		{
			lsMem.gInternalShapes[1].dmaShapeData (collisionShape1Ptr, collisionPairInput.m_shapeType1, 2);
		}

		if (collisionPairInput.m_shapeType1 == CONVEX_HULL_SHAPE_PROXYTYPE)
		{
			btConvexHullShape* convexHull = (btConvexHullShape*)lsMem.gInternalShapes[1].m_convexShape;
			cellDmaWaitTagStatusAll (DMA_MASK(2));
			/* Don't load the same verts twice */
			if ((ppu_address_t)convexHull->getPoints() != lsMem.gInternalConvexHull[1].m_ppuPointsPtr)
			{
				lsMem.gInternalConvexHull[1].dmaPointsData (lsMem.gInternalShapes[1], 2);
			} 
		}

		cellDmaWaitTagStatusAll (DMA_MASK(1));
		btCompoundShape* spuCompoundShape0 = (btCompoundShape*)lsMem.gInternalShapes[0].m_collisionShape;
		lsMem.compoundShapeData[0].dmaChildShapeInfo (spuCompoundShape0, 1);
		cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
		int childShapeCount0 = spuCompoundShape0->getNumChildShapes();

		for (int i = 0; i < childShapeCount0; ++i)
		{
			btCompoundShapeChild& childShape = lsMem.compoundShapeData[0].m_subshapes[i];

			lsMem.compoundShapeData[0].dmaChildShape (i, &lsMem.gInternalShapes[0], &lsMem.gInternalConvexHull[0], 1);
			SpuCollisionPairInput cinput (collisionPairInput);
			cinput.m_worldTransform0 = collisionPairInput.m_worldTransform0 * childShape.m_transform;
			cinput.m_shapeType0 = childShape.m_childShapeType;
			cinput.m_collisionMargin0 = childShape.m_childMargin;

			cellDmaWaitTagStatusAll(DMA_MASK(1));

			handleCollisionPair(cinput,
								lsMem,
								spuContacts,			
								(ppu_address_t)childShape.m_childShape,
								collisionShape1Ptr, false);
		}
	}
	else if (btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType1) )
	{
		//snPause();

		/* Don't load the same shape twice. */
		if (collisionShape0Ptr != lsMem.gInternalShapes[0].m_ppuConvexShapePtr)
		{
			lsMem.gInternalShapes[0].dmaShapeData (collisionShape0Ptr, collisionPairInput.m_shapeType0, 1);
		}

		/* Don't load the same shape twice. */
		if (collisionShape1Ptr != lsMem.gInternalShapes[1].m_ppuConvexShapePtr)
		{
			lsMem.gInternalShapes[1].dmaShapeData (collisionShape1Ptr, collisionPairInput.m_shapeType1, 2);
		}

		cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));

		// object 0 non-compound, object 1 compound
		btCompoundShape* spuCompoundShape = (btCompoundShape*)lsMem.gInternalShapes[1].m_collisionShape;
		lsMem.compoundShapeData[1].dmaChildShapeInfo (spuCompoundShape, 1);
		cellDmaWaitTagStatusAll(DMA_MASK(1));
		
		int childShapeCount = spuCompoundShape->getNumChildShapes();
		for (int i = 0; i < childShapeCount; ++i)
		{
			btCompoundShapeChild& childShape = lsMem.compoundShapeData[1].m_subshapes[i];
			// Dma the child shape
			lsMem.compoundShapeData[1].dmaChildShape (i, &lsMem.gInternalShapes[1], &lsMem.gInternalConvexHull[1], 1);
			cellDmaWaitTagStatusAll(DMA_MASK(1));

			SpuCollisionPairInput cinput (collisionPairInput);
			cinput.m_worldTransform1 = collisionPairInput.m_worldTransform1 * childShape.m_transform;
			cinput.m_shapeType1 = childShape.m_childShapeType;
			cinput.m_collisionMargin1 = childShape.m_childMargin;
			handleCollisionPair(cinput,
								lsMem,
								spuContacts,
								collisionShape0Ptr,
								(ppu_address_t)childShape.m_childShape, false);
		}
	}
	else
	{
		//we only support convex v. concave
		//a non-convex shape is involved									
		bool handleConvexConcave = false;

		//concave v. convex
		//swap into convex v. concave
		if (btBroadphaseProxy::isConcave(collisionPairInput.m_shapeType0) &&
			btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType1))
		{
			// Swap stuff
			DoSwap(collisionShape0Ptr, collisionShape1Ptr);
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
				/* Don't load the same shape twice. */
				if (collisionShape0Ptr != lsMem.gInternalShapes[0].m_ppuConvexShapePtr)
				{
					lsMem.gInternalShapes[0].dmaShapeData (collisionShape0Ptr, collisionPairInput.m_shapeType0, 1);
				}

				/* Don't load the same shape twice. */
				if (collisionShape1Ptr != lsMem.gInternalShapes[1].m_ppuConvexShapePtr)
				{
					lsMem.gInternalShapes[1].dmaShapeData (collisionShape1Ptr, collisionPairInput.m_shapeType1, 2);
				} 

				if (collisionPairInput.m_shapeType0 == CONVEX_HULL_SHAPE_PROXYTYPE)
				{
					btConvexHullShape* convexHull = (btConvexHullShape*)lsMem.gInternalShapes[0].m_convexShape;
					cellDmaWaitTagStatusAll (DMA_MASK(1));
					/* Don't load the same verts twice */
					if ((ppu_address_t)convexHull->getPoints() != lsMem.gInternalConvexHull[0].m_ppuPointsPtr)
					{
						lsMem.gInternalConvexHull[0].dmaPointsData (lsMem.gInternalShapes[0], 1);
					}
				}

				/* This can't happen because shape1 is the optimized Bvh shape */
#if 0
				if (collisionPairInput.m_shapeType1 == CONVEX_HULL_SHAPE_PROXYTYPE)
				{
					btConvexHullShape* convexHull = (btConvexHullShape*)lsMem.gInternalShapes[1].m_convexShape;
					cellDmaWaitTagStatusAll (DMA_MASK(2));
					/* Don't load the same verts twice */
					if ((ppu_address_t)convexHull->getPoints() != lsMem.gInternalConvexHull[1].m_ppuPointsPtr)
					{
						lsMem.gInternalConvexHull[1].dmaPointsData (lsMem.gInternalShapes[1], 2);
					} 
				}
#endif

				cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
			}
			
			SpuConvexConcaveCollisionAlgorithm (collisionPairInput,
												lsMem, 
												spuContacts,
												&lsMem.gInternalShapes[0],
												&lsMem.gInternalShapes[1],
												&lsMem.gInternalConvexHull[0],
												&lsMem.gInternalConvexHull[1]);
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

	gUseEpa = taskDesc.m_useEpa;

	//	spu_printf("taskDescPtr=%llx\n",taskDescPtr);

	SpuContactResult spuContacts;

	////////////////////

	ppu_address_t dmaInPtr = taskDesc.inPtr;
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
	register ppu_address_t	dmaPpuAddress;
	register ppu_address_t	dmaPpuAddress2;

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
				// DMA: broadphase pairs
				dmaSize = numPairs*sizeof(btBroadphasePair);
				dmaPpuAddress = wuInputs[j].m_pairArrayPtr+wuInputs[j].m_startIndex * sizeof(btBroadphasePair);
				cellDmaGet(&lsMem.gBroadphasePairs, dmaPpuAddress  , dmaSize, DMA_TAG(1), 0, 0);
				cellDmaWaitTagStatusAll(DMA_MASK(1));	

				//for each broadphase pair, do something
				for (p=0;p<numPairs;p++)
				{

					btBroadphasePair& pair = lsMem.gBroadphasePairs[p];

#ifdef DEBUG_SPU_COLLISION_DETECTION
					spu_printf("pair->m_userInfo = %d\n",pair.m_userInfo);
					spu_printf("pair->m_algorithm = %d\n",pair.m_algorithm);
					spu_printf("pair->m_pProxy0 = %d\n",pair.m_pProxy0);
					spu_printf("pair->m_pProxy1 = %d\n",pair.m_pProxy1);
#endif //DEBUG_SPU_COLLISION_DETECTION

					userInfo = int(pair.m_userInfo);

					// skip pairs we don't support
					if (!pair.m_algorithm || !pair.m_pProxy0 ||
						!pair.m_pProxy1 || userInfo != 2)
						continue;

					
					// DMA: SpuContactManifoldCollisionAlgorithm
					// SpuContactManifoldCollisionAlgorithm:
					// A dummy collision algorithm that gives us the
					// collision types, and contact manifold pointers
					dmaSize = sizeof(SpuContactManifoldCollisionAlgorithm);
					dmaPpuAddress2 = (ppu_address_t)pair.m_algorithm;
					cellDmaGet(&lsMem.gSpuContactManifoldAlgo, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
					//cellDmaWaitTagStatusAll(DMA_MASK(1));
						
					//snPause();

#ifdef DEBUG_SPU_COLLISION_DETECTION
					spu_printf("SPU: manifoldPtr: %llx",collisionPairInput.m_persistentManifoldPtr);
#endif //DEBUG_SPU_COLLISION_DETECTION

						
					// DMA: btBroadphaseProxy for object 0
					dmaSize = sizeof(btBroadphaseProxy);
					dmaPpuAddress2 = (ppu_address_t)pair.m_pProxy0;
					lsMem.gProxyPtr0 = (btBroadphaseProxy*) lsMem.bufferProxy0;
					stallingUnalignedDmaSmallGet(lsMem.gProxyPtr0, dmaPpuAddress2  , dmaSize);
					// NOTE: The DMA initiated for SpuContactManifoldCollisionAlgorithm is synced in
					// stallingUnalignedDmaSmallGet
					collisionPairInput.m_persistentManifoldPtr = (ppu_address_t) lsMem.gSpuContactManifoldAlgo.getContactManifoldPtr();
					collisionPairInput.m_isSwapped = false;

					// DMA: btPersistentManifold
					dmaSize = sizeof(btPersistentManifold);
					dmaPpuAddress2 = collisionPairInput.m_persistentManifoldPtr;
					cellDmaGet(&lsMem.gPersistentManifold, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);

					// DMA: btBroadphaseProxy for object 0
					dmaSize = sizeof(btBroadphaseProxy);
					dmaPpuAddress2 = (ppu_address_t)pair.m_pProxy1;
					lsMem.gProxyPtr1 = (btBroadphaseProxy*) lsMem.bufferProxy1;
					stallingUnalignedDmaSmallGet(lsMem.gProxyPtr1, dmaPpuAddress2  , dmaSize);
					// NOTE: The DMA initiated for btPersistentManifold is synced in
					// stallingUnalignedDmaSmallGet

#ifdef DEBUG_SPU_COLLISION_DETECTION
					spu_printf("SPU collisionPairInput->m_shapeType0 = %d\n",collisionPairInput.m_shapeType0);
					spu_printf("SPU collisionPairInput->m_shapeType1 = %d\n",collisionPairInput.m_shapeType1);
#endif //DEBUG_SPU_COLLISION_DETECTION

					// Construct the collision Pair 
					collisionPairInput.m_shapeType0 = lsMem.gSpuContactManifoldAlgo.getShapeType0();
					collisionPairInput.m_shapeType1 = lsMem.gSpuContactManifoldAlgo.getShapeType1();
					collisionPairInput.m_collisionMargin0 = lsMem.gSpuContactManifoldAlgo.getCollisionMargin0();
					collisionPairInput.m_collisionMargin1 = lsMem.gSpuContactManifoldAlgo.getCollisionMargin1();
									
					cellDmaWaitTagStatusAll(DMA_MASK(1)); // might not be necessary
					dmaAndSetupCollisionObjects(collisionPairInput, lsMem);

					if (lsMem.getColObj0()->isActive() || lsMem.getColObj1()->isActive())
					{

						handleCollisionPair(collisionPairInput,
											lsMem,
											spuContacts,
											(ppu_address_t)lsMem.getColObj0()->getCollisionShape(),
											(ppu_address_t)lsMem.getColObj1()->getCollisionShape());
					}
				}
			}
		} //end for (j = 0; j < numOnPage; j++)
	}//	for 

#ifdef DEBUG_SPU_COLLISION_DETECTION
	spu_printf("processCollisionTask %d / %d\n",numPages, numOnLastPage);
#endif //DEBUG_SPU_COLLISION_DETECTION

	return;
}
