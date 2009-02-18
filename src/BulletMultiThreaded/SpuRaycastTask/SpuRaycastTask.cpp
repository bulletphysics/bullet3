

#include "../PlatformDefinitions.h"
#include "SpuRaycastTask.h"
#include "../SpuCollisionObjectWrapper.h"
#include "../SpuNarrowPhaseCollisionTask/SpuCollisionShapes.h"
#include "SpuSubSimplexConvexCast.h"
#include "LinearMath/btAabbUtil2.h"


/* Future optimization strategies: 
1. BBOX prune before loading shape data
2. Could reduce number of dmas for ray output data to a single read and write.
   By sharing the temporary work unit output structures across objects.
3. The reason SpuRaycastNodeCallback1 is slower is because the triangle data isn't
   being cached across calls. Fix that by doing the final ray pruning inside the callback.
*/

/* Future work:
1. support first hit, closest hit, etc rather than just closest hit.
2. support compound objects
*/

#define CALLBACK_ALL

struct RaycastTask_LocalStoreMemory
{
	ATTRIBUTE_ALIGNED16(char gColObj [sizeof(btCollisionObject)+16]);
	btCollisionObject* getColObj()
	{
		return (btCollisionObject*) gColObj;
	}

	ATTRIBUTE_ALIGNED16(SpuCollisionObjectWrapper gCollisionObjectWrapper);
	SpuCollisionObjectWrapper* getCollisionObjectWrapper ()
	{
		return &gCollisionObjectWrapper;
	}

	CollisionShape_LocalStoreMemory gCollisionShape;
	ATTRIBUTE_ALIGNED16(int	spuIndices[16]);

	bvhMeshShape_LocalStoreMemory bvhShapeData;
	SpuConvexPolyhedronVertexData convexVertexData;
	CompoundShape_LocalStoreMemory compoundShapeData;
};

#ifdef WIN32
void* createRaycastLocalStoreMemory()
{
	return new RaycastTask_LocalStoreMemory;
};
#elif defined(__CELLOS_LV2__)
ATTRIBUTE_ALIGNED16(RaycastTask_LocalStoreMemory gLocalStoreMemory);
void* createRaycastLocalStoreMemory()
{
	return &gLocalStoreMemory;
}
#endif

void GatherCollisionObjectAndShapeData (RaycastGatheredObjectData* gatheredObjectData, RaycastTask_LocalStoreMemory* lsMemPtr, ppu_address_t objectWrapper)
{
	register int dmaSize;
	register ppu_address_t	dmaPpuAddress2;
	/* DMA Collision object wrapper into local store */
	dmaSize = sizeof(SpuCollisionObjectWrapper);
	dmaPpuAddress2 = objectWrapper;
	cellDmaGet(&lsMemPtr->gCollisionObjectWrapper, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
	cellDmaWaitTagStatusAll(DMA_MASK(1));

	/* DMA Collision object into local store */
	dmaSize = sizeof(btCollisionObject);
	dmaPpuAddress2 = lsMemPtr->getCollisionObjectWrapper()->getCollisionObjectPtr();
	cellDmaGet(&lsMemPtr->gColObj, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);
	cellDmaWaitTagStatusAll(DMA_MASK(2));
	
	/* Gather information about collision object and shape */
	gatheredObjectData->m_worldTransform = lsMemPtr->getColObj()->getWorldTransform();
	gatheredObjectData->m_collisionMargin = lsMemPtr->getCollisionObjectWrapper()->getCollisionMargin ();
	gatheredObjectData->m_shapeType = lsMemPtr->getCollisionObjectWrapper()->getShapeType ();
	gatheredObjectData->m_collisionShape = (ppu_address_t)lsMemPtr->getColObj()->getCollisionShape();
	gatheredObjectData->m_spuCollisionShape = (void*)&lsMemPtr->gCollisionShape.collisionShape;

	/* DMA shape data */
	dmaCollisionShape (gatheredObjectData->m_spuCollisionShape, gatheredObjectData->m_collisionShape, 1, gatheredObjectData->m_shapeType);
	cellDmaWaitTagStatusAll(DMA_MASK(1));
	if (btBroadphaseProxy::isConvex (gatheredObjectData->m_shapeType))
	{
		btConvexInternalShape* spuConvexShape = (btConvexInternalShape*)gatheredObjectData->m_spuCollisionShape;
		gatheredObjectData->m_primitiveDimensions = spuConvexShape->getImplicitShapeDimensions ();
	} else {
		gatheredObjectData->m_primitiveDimensions = btVector3(1.0, 1.0, 1.0);
	}

}

void dmaLoadRayOutput (ppu_address_t rayOutputAddr, SpuRaycastTaskWorkUnitOut* rayOutput, uint32_t dmaTag)
{
	cellDmaGet(rayOutput, rayOutputAddr, sizeof(*rayOutput), DMA_TAG(dmaTag), 0, 0);
}

void dmaStoreRayOutput (ppu_address_t rayOutputAddr, const SpuRaycastTaskWorkUnitOut* rayOutput, uint32_t dmaTag)
{
	cellDmaLargePut (rayOutput, rayOutputAddr, sizeof(*rayOutput), DMA_TAG(dmaTag), 0, 0);
}

#if 0
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
#endif

void small_cache_read_triple(	void* ls0, ppu_address_t ea0,
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
		tmpTarget0 = (char*)cellDmaSmallGetReadOnly(tmpTarget0,ea0,size,DMA_TAG(1),0,0);


		char* localStore1 = (char*)ls1;
		last4BitsOffset = ea1 & 0x0f;
		char* tmpTarget1 = tmpBuffer1 + last4BitsOffset;
		tmpTarget1 = (char*)cellDmaSmallGetReadOnly(tmpTarget1,ea1,size,DMA_TAG(1),0,0);
		
		char* localStore2 = (char*)ls2;
		last4BitsOffset = ea2 & 0x0f;
		char* tmpTarget2 = tmpBuffer2 + last4BitsOffset;
		tmpTarget2 = (char*)cellDmaSmallGetReadOnly(tmpTarget2,ea2,size,DMA_TAG(1),0,0);
		
		
		cellDmaWaitTagStatusAll( DMA_MASK(1) );

		//this is slowish, perhaps memcpy on SPU is smarter?
		for (i=0; btLikely( i<size );i++)
		{
			localStore0[i] = tmpTarget0[i];
			localStore1[i] = tmpTarget1[i];
			localStore2[i] = tmpTarget2[i];
		}
}

void performRaycastAgainstConvex (RaycastGatheredObjectData* gatheredObjectData, const SpuRaycastTaskWorkUnit& workUnit, SpuRaycastTaskWorkUnitOut* workUnitOut, RaycastTask_LocalStoreMemory* lsMemPtr);

class spuRaycastNodeCallback1 : public btNodeOverlapCallback
{
	RaycastGatheredObjectData* m_gatheredObjectData;
	const SpuRaycastTaskWorkUnit* m_workUnits;
	SpuRaycastTaskWorkUnitOut* m_workUnitsOut;
	int m_workUnit;
	RaycastTask_LocalStoreMemory* m_lsMemPtr;

	ATTRIBUTE_ALIGNED16(btVector3	spuTriangleVertices[3]);
	ATTRIBUTE_ALIGNED16(btScalar	spuUnscaledVertex[4]);
	//ATTRIBUTE_ALIGNED16(int	spuIndices[16]);
public:
	spuRaycastNodeCallback1(RaycastGatheredObjectData* gatheredObjectData,const SpuRaycastTaskWorkUnit* workUnits, SpuRaycastTaskWorkUnitOut* workUnitsOut, RaycastTask_LocalStoreMemory* lsMemPtr)
		: m_gatheredObjectData(gatheredObjectData),
		  m_workUnits(workUnits),
		  m_workUnitsOut(workUnitsOut),
		  m_workUnit(0),
		  m_lsMemPtr (lsMemPtr)
	{
	}

	void setWorkUnit (int workUnit) { m_workUnit = workUnit; }
	virtual void processNode(int subPart, int triangleIndex)
	{
		///Create a triangle on the stack, call process collision, with GJK
		///DMA the vertices, can benefit from software caching

		//		spu_printf("processNode with triangleIndex %d\n",triangleIndex);

			// ugly solution to support both 16bit and 32bit indices
		if (m_lsMemPtr->bvhShapeData.gIndexMesh.m_indexType == PHY_SHORT)
		{
			short int* indexBasePtr = (short int*)(m_lsMemPtr->bvhShapeData.gIndexMesh.m_triangleIndexBase+triangleIndex*m_lsMemPtr->bvhShapeData.gIndexMesh.m_triangleIndexStride);
			ATTRIBUTE_ALIGNED16(short int tmpIndices[3]);

			small_cache_read_triple(&tmpIndices[0],(ppu_address_t)&indexBasePtr[0],
									&tmpIndices[1],(ppu_address_t)&indexBasePtr[1],
									&tmpIndices[2],(ppu_address_t)&indexBasePtr[2],
									sizeof(short int));

			m_lsMemPtr->spuIndices[0] = int(tmpIndices[0]);
			m_lsMemPtr->spuIndices[1] = int(tmpIndices[1]);
			m_lsMemPtr->spuIndices[2] = int(tmpIndices[2]);
		} else
		{
			int* indexBasePtr = (int*)(m_lsMemPtr->bvhShapeData.gIndexMesh.m_triangleIndexBase+triangleIndex*m_lsMemPtr->bvhShapeData.gIndexMesh.m_triangleIndexStride);

			small_cache_read_triple(&m_lsMemPtr->spuIndices[0],(ppu_address_t)&indexBasePtr[0],
								&m_lsMemPtr->spuIndices[1],(ppu_address_t)&indexBasePtr[1],
								&m_lsMemPtr->spuIndices[2],(ppu_address_t)&indexBasePtr[2],
								sizeof(int));
		}

		//printf("%d %d %d\n", m_lsMemPtr->spuIndices[0], m_lsMemPtr->spuIndices[1], m_lsMemPtr->spuIndices[2]);
		//		spu_printf("SPU index0=%d ,",spuIndices[0]);
		//		spu_printf("SPU index1=%d ,",spuIndices[1]);
		//		spu_printf("SPU index2=%d ,",spuIndices[2]);
		//		spu_printf("SPU: indexBasePtr=%llx\n",indexBasePtr);

		const btVector3& meshScaling = m_lsMemPtr->bvhShapeData.gTriangleMeshInterfacePtr->getScaling();
	
		for (int j=2;btLikely( j>=0 );j--)
		{
			int graphicsindex = m_lsMemPtr->spuIndices[j];

						//spu_printf("SPU index=%d ,",graphicsindex);
			btScalar* graphicsbasePtr = (btScalar*)(m_lsMemPtr->bvhShapeData.gIndexMesh.m_vertexBase+graphicsindex*m_lsMemPtr->bvhShapeData.gIndexMesh.m_vertexStride);
			
			//			spu_printf("SPU graphicsbasePtr=%llx\n",graphicsbasePtr);


			///handle un-aligned vertices...

			//another DMA for each vertex
			small_cache_read_triple(&spuUnscaledVertex[0],(ppu_address_t)&graphicsbasePtr[0],
									&spuUnscaledVertex[1],(ppu_address_t)&graphicsbasePtr[1],
									&spuUnscaledVertex[2],(ppu_address_t)&graphicsbasePtr[2],
									sizeof(btScalar));
			
			//printf("%f %f %f\n", spuUnscaledVertex[0],spuUnscaledVertex[1],spuUnscaledVertex[2]);
			spuTriangleVertices[j] = btVector3(
				spuUnscaledVertex[0]*meshScaling.getX(),
				spuUnscaledVertex[1]*meshScaling.getY(),
				spuUnscaledVertex[2]*meshScaling.getZ());

				//spu_printf("SPU:triangle vertices:%f,%f,%f\n",spuTriangleVertices[j].x(),spuTriangleVertices[j].y(),spuTriangleVertices[j].z());
		}
		
		RaycastGatheredObjectData triangleGatheredObjectData (*m_gatheredObjectData);
		triangleGatheredObjectData.m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
		triangleGatheredObjectData.m_spuCollisionShape = &spuTriangleVertices[0];

		//printf("%f %f %f\n", spuTriangleVertices[0][0],spuTriangleVertices[0][1],spuTriangleVertices[0][2]);
		//printf("%f %f %f\n", spuTriangleVertices[1][0],spuTriangleVertices[1][1],spuTriangleVertices[1][2]);
		//printf("%f %f %f\n", spuTriangleVertices[2][0],spuTriangleVertices[2][1],spuTriangleVertices[2][2]);
		SpuRaycastTaskWorkUnitOut out;
		out.hitFraction = 1.0;
		performRaycastAgainstConvex (&triangleGatheredObjectData, m_workUnits[m_workUnit], &out, m_lsMemPtr);
		/* XXX: For now only take the closest hit */
		if (out.hitFraction < m_workUnitsOut[m_workUnit].hitFraction)
		{
			m_workUnitsOut[m_workUnit].hitFraction = out.hitFraction;
			m_workUnitsOut[m_workUnit].hitNormal = out.hitNormal;
		}
	}

};

class spuRaycastNodeCallback : public btNodeOverlapCallback
{
	RaycastGatheredObjectData* m_gatheredObjectData;
	const SpuRaycastTaskWorkUnit* m_workUnits;
	SpuRaycastTaskWorkUnitOut* m_workUnitsOut;
	int m_numWorkUnits;
	RaycastTask_LocalStoreMemory* m_lsMemPtr;

	ATTRIBUTE_ALIGNED16(btVector3	spuTriangleVertices[3]);
	ATTRIBUTE_ALIGNED16(btScalar	spuUnscaledVertex[4]);
	//ATTRIBUTE_ALIGNED16(int	spuIndices[16]);
public:
	spuRaycastNodeCallback(RaycastGatheredObjectData* gatheredObjectData,const SpuRaycastTaskWorkUnit* workUnits, SpuRaycastTaskWorkUnitOut* workUnitsOut, int numWorkUnits, RaycastTask_LocalStoreMemory* lsMemPtr)
		: m_gatheredObjectData(gatheredObjectData),
		  m_workUnits(workUnits),
		  m_workUnitsOut(workUnitsOut),
		  m_numWorkUnits(numWorkUnits),
		  m_lsMemPtr (lsMemPtr)
	{
	}

	virtual void processNode(int subPart, int triangleIndex)
	{
		///Create a triangle on the stack, call process collision, with GJK
		///DMA the vertices, can benefit from software caching

		//		spu_printf("processNode with triangleIndex %d\n",triangleIndex);

			// ugly solution to support both 16bit and 32bit indices
		if (m_lsMemPtr->bvhShapeData.gIndexMesh.m_indexType == PHY_SHORT)
		{
			short int* indexBasePtr = (short int*)(m_lsMemPtr->bvhShapeData.gIndexMesh.m_triangleIndexBase+triangleIndex*m_lsMemPtr->bvhShapeData.gIndexMesh.m_triangleIndexStride);
			ATTRIBUTE_ALIGNED16(short int tmpIndices[3]);

			small_cache_read_triple(&tmpIndices[0],(ppu_address_t)&indexBasePtr[0],
									&tmpIndices[1],(ppu_address_t)&indexBasePtr[1],
									&tmpIndices[2],(ppu_address_t)&indexBasePtr[2],
									sizeof(short int));

			m_lsMemPtr->spuIndices[0] = int(tmpIndices[0]);
			m_lsMemPtr->spuIndices[1] = int(tmpIndices[1]);
			m_lsMemPtr->spuIndices[2] = int(tmpIndices[2]);
		} else
		{
			int* indexBasePtr = (int*)(m_lsMemPtr->bvhShapeData.gIndexMesh.m_triangleIndexBase+triangleIndex*m_lsMemPtr->bvhShapeData.gIndexMesh.m_triangleIndexStride);

			small_cache_read_triple(&m_lsMemPtr->spuIndices[0],(ppu_address_t)&indexBasePtr[0],
								&m_lsMemPtr->spuIndices[1],(ppu_address_t)&indexBasePtr[1],
								&m_lsMemPtr->spuIndices[2],(ppu_address_t)&indexBasePtr[2],
								sizeof(int));
		}

		//printf("%d %d %d\n", m_lsMemPtr->spuIndices[0], m_lsMemPtr->spuIndices[1], m_lsMemPtr->spuIndices[2]);
		//		spu_printf("SPU index0=%d ,",spuIndices[0]);
		//		spu_printf("SPU index1=%d ,",spuIndices[1]);
		//		spu_printf("SPU index2=%d ,",spuIndices[2]);
		//		spu_printf("SPU: indexBasePtr=%llx\n",indexBasePtr);

		const btVector3& meshScaling = m_lsMemPtr->bvhShapeData.gTriangleMeshInterfacePtr->getScaling();
	
		for (int j=2;btLikely( j>=0 );j--)
		{
			int graphicsindex = m_lsMemPtr->spuIndices[j];

						//spu_printf("SPU index=%d ,",graphicsindex);
			btScalar* graphicsbasePtr = (btScalar*)(m_lsMemPtr->bvhShapeData.gIndexMesh.m_vertexBase+graphicsindex*m_lsMemPtr->bvhShapeData.gIndexMesh.m_vertexStride);
			
			//			spu_printf("SPU graphicsbasePtr=%llx\n",graphicsbasePtr);


			///handle un-aligned vertices...

			//another DMA for each vertex
			small_cache_read_triple(&spuUnscaledVertex[0],(ppu_address_t)&graphicsbasePtr[0],
									&spuUnscaledVertex[1],(ppu_address_t)&graphicsbasePtr[1],
									&spuUnscaledVertex[2],(ppu_address_t)&graphicsbasePtr[2],
									sizeof(btScalar));
			
			//printf("%f %f %f\n", spuUnscaledVertex[0],spuUnscaledVertex[1],spuUnscaledVertex[2]);
			spuTriangleVertices[j] = btVector3(
				spuUnscaledVertex[0]*meshScaling.getX(),
				spuUnscaledVertex[1]*meshScaling.getY(),
				spuUnscaledVertex[2]*meshScaling.getZ());

				//spu_printf("SPU:triangle vertices:%f,%f,%f\n",spuTriangleVertices[j].x(),spuTriangleVertices[j].y(),spuTriangleVertices[j].z());
		}
		
		RaycastGatheredObjectData triangleGatheredObjectData (*m_gatheredObjectData);
		triangleGatheredObjectData.m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
		triangleGatheredObjectData.m_spuCollisionShape = &spuTriangleVertices[0];

		//printf("%f %f %f\n", spuTriangleVertices[0][0],spuTriangleVertices[0][1],spuTriangleVertices[0][2]);
		//printf("%f %f %f\n", spuTriangleVertices[1][0],spuTriangleVertices[1][1],spuTriangleVertices[1][2]);
		//printf("%f %f %f\n", spuTriangleVertices[2][0],spuTriangleVertices[2][1],spuTriangleVertices[2][2]);
		for (int i = 0; i < m_numWorkUnits; i++)
		{
			SpuRaycastTaskWorkUnitOut out;
			out.hitFraction = 1.0;
			performRaycastAgainstConvex (&triangleGatheredObjectData, m_workUnits[i], &out, m_lsMemPtr);
			/* XXX: For now only take the closest hit */
			if (out.hitFraction < m_workUnitsOut[i].hitFraction)
			{
				m_workUnitsOut[i].hitFraction = out.hitFraction;
				m_workUnitsOut[i].hitNormal = out.hitNormal;
			}
		}
	}

};


void	spuWalkStacklessQuantizedTreeAgainstRays(RaycastTask_LocalStoreMemory* lsMemPtr, 
						 btNodeOverlapCallback* nodeCallback,
						 const btVector3* rayFrom,
						 const btVector3* rayTo,
						 int numWorkUnits,
						 unsigned short int* quantizedQueryAabbMin,
						 unsigned short int* quantizedQueryAabbMax,
						 const btQuantizedBvhNode* rootNode,
						 int startNodeIndex,int endNodeIndex)
{
	int curIndex = startNodeIndex;
	int walkIterations = 0;
	int subTreeSize = endNodeIndex - startNodeIndex;

	int escapeIndex;

	unsigned int boxBoxOverlap, rayBoxOverlap, anyRayBoxOverlap;
	unsigned int isLeafNode;

#define RAYAABB2
#ifdef RAYAABB2
	unsigned int sign[SPU_RAYCAST_WORK_UNITS_PER_TASK][3];
	btVector3 rayInvDirection[SPU_RAYCAST_WORK_UNITS_PER_TASK];
	btScalar lambda_max[SPU_RAYCAST_WORK_UNITS_PER_TASK];
	for (int i = 0; i < numWorkUnits; i++)
	{
		btVector3 rayDirection = (rayTo[i]-rayFrom[i]);
		rayDirection.normalize ();
		lambda_max[i] = rayDirection.dot(rayTo[i]-rayFrom[i]);
		rayInvDirection[i][0] = btScalar(1.0) / rayDirection[0];
		rayInvDirection[i][1] = btScalar(1.0) / rayDirection[1];
		rayInvDirection[i][2] = btScalar(1.0) / rayDirection[2];
		sign[i][0] = rayDirection[0] < 0.0;
		sign[i][1] = rayDirection[1] < 0.0;
		sign[i][2] = rayDirection[2] < 0.0;
	}
#endif

	while (curIndex < endNodeIndex)
	{
		//catch bugs in tree data
		assert (walkIterations < subTreeSize);

		walkIterations++;

		isLeafNode = rootNode->isLeafNode();

		anyRayBoxOverlap = 0;

		for (int i = 0; i < numWorkUnits; i++)
		{
			unsigned short int* quamin = (quantizedQueryAabbMin + 3 * i);
			unsigned short int* quamax = (quantizedQueryAabbMax + 3 * i);
			boxBoxOverlap = spuTestQuantizedAabbAgainstQuantizedAabb(quamin,quamax,rootNode->m_quantizedAabbMin,rootNode->m_quantizedAabbMax);
			if (!boxBoxOverlap)
				continue;

			rayBoxOverlap = 0;
			btScalar param = 1.0;
			btVector3 normal;
			btVector3 bounds[2];
			bounds[0] = lsMemPtr->bvhShapeData.getOptimizedBvh()->unQuantize(rootNode->m_quantizedAabbMin);
			bounds[1] = lsMemPtr->bvhShapeData.getOptimizedBvh()->unQuantize(rootNode->m_quantizedAabbMax);
#ifdef RAYAABB2
			rayBoxOverlap = btRayAabb2 (rayFrom[i], rayInvDirection[i], sign[i], bounds, param, 0.0, lambda_max[i]);
#else
			rayBoxOverlap = btRayAabb(rayFrom[i], rayTo[i], bounds[0], bounds[1], param, normal);
#endif

#ifndef CALLBACK_ALL
			anyRayBoxOverlap = rayBoxOverlap || anyRayBoxOverlap;
			/* If we have any ray vs. box overlap and this isn't a leaf node
			   we know that we need to dig deeper
			*/
			if (!isLeafNode && anyRayBoxOverlap)
				break;

			if (isLeafNode && rayBoxOverlap)
			{
				spuRaycastNodeCallback1* callback = (spuRaycastNodeCallback1*)nodeCallback;
				callback->setWorkUnit (i);
				nodeCallback->processNode (0, rootNode->getTriangleIndex());
			}
#else
			/* If we have any ray vs. box overlap and this isn't a leaf node
			   we know that we need to dig deeper
			*/
			if (rayBoxOverlap)
			{
				anyRayBoxOverlap = 1;
				break;
			}
#endif
		}

#ifdef CALLBACK_ALL
		if (isLeafNode && anyRayBoxOverlap)
		{
			nodeCallback->processNode (0, rootNode->getTriangleIndex());
		}
#endif

		if (anyRayBoxOverlap || isLeafNode)
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


void performRaycastAgainstConcave (RaycastGatheredObjectData* gatheredObjectData, const SpuRaycastTaskWorkUnit* workUnits, SpuRaycastTaskWorkUnitOut* workUnitsOut, int numWorkUnits, RaycastTask_LocalStoreMemory* lsMemPtr)
{
	//order: first collision shape is convex, second concave. m_isSwapped is true, if the original order was opposite
	register int dmaSize;
	register ppu_address_t	dmaPpuAddress2;

	
	btBvhTriangleMeshShape*	trimeshShape = (btBvhTriangleMeshShape*)gatheredObjectData->m_spuCollisionShape;

	//need the mesh interface, for access to triangle vertices
	dmaBvhShapeData (&(lsMemPtr->bvhShapeData), trimeshShape);

	unsigned short int quantizedQueryAabbMin[SPU_RAYCAST_WORK_UNITS_PER_TASK][3];
	unsigned short int quantizedQueryAabbMax[SPU_RAYCAST_WORK_UNITS_PER_TASK][3];
	btVector3 rayFromInTriangleSpace[SPU_RAYCAST_WORK_UNITS_PER_TASK];
	btVector3 rayToInTriangleSpace[SPU_RAYCAST_WORK_UNITS_PER_TASK];

	/* Calculate the AABB for the ray in the triangle mesh shape */
	btTransform rayInTriangleSpace;
	rayInTriangleSpace = gatheredObjectData->m_worldTransform.inverse();

	for (int i = 0; i < numWorkUnits; i++)
	{
		btVector3 aabbMin;
		btVector3 aabbMax;

		rayFromInTriangleSpace[i] = rayInTriangleSpace(workUnits[i].rayFrom);
		rayToInTriangleSpace[i] = rayInTriangleSpace(workUnits[i].rayTo);

		aabbMin = rayFromInTriangleSpace[i];
		aabbMin.setMin (rayToInTriangleSpace[i]);
		aabbMax = rayFromInTriangleSpace[i];
		aabbMax.setMax (rayToInTriangleSpace[i]);

		lsMemPtr->bvhShapeData.getOptimizedBvh()->quantizeWithClamp(quantizedQueryAabbMin[i],aabbMin,0);
		lsMemPtr->bvhShapeData.getOptimizedBvh()->quantizeWithClamp(quantizedQueryAabbMax[i],aabbMax,1);
	}

	QuantizedNodeArray&	nodeArray = lsMemPtr->bvhShapeData.getOptimizedBvh()->getQuantizedNodeArray();
	//spu_printf("SPU: numNodes = %d\n",nodeArray.size());

	BvhSubtreeInfoArray& subTrees = lsMemPtr->bvhShapeData.getOptimizedBvh()->getSubtreeInfoArray();	

#ifdef CALLBACK_ALL
	spuRaycastNodeCallback nodeCallback (gatheredObjectData, workUnits, workUnitsOut, numWorkUnits, lsMemPtr);
#else
	spuRaycastNodeCallback1 nodeCallback (gatheredObjectData, workUnits, workUnitsOut, lsMemPtr);
#endif
	
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
				
				unsigned int overlap = 1;
				for (int boxId = 0; boxId < numWorkUnits; boxId++)
				{
					overlap = spuTestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin[boxId],quantizedQueryAabbMax[boxId],subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
					if (overlap)
						break;
				}

				if (overlap)
				{
					btAssert(subtree.m_subtreeSize);

					//dma the actual nodes of this subtree
					dmaBvhSubTreeNodes (&lsMemPtr->bvhShapeData.gSubtreeNodes[0], subtree, nodeArray, 2);

					cellDmaWaitTagStatusAll(DMA_MASK(2));

					/* Walk this subtree */
					
					{

						spuWalkStacklessQuantizedTreeAgainstRays(lsMemPtr,
										        &nodeCallback,
										        &rayFromInTriangleSpace[0],
											&rayToInTriangleSpace[0],
											numWorkUnits,
											&quantizedQueryAabbMin[0][0],&quantizedQueryAabbMax[0][0],
											&lsMemPtr->bvhShapeData.gSubtreeNodes[0], 0, subtree.m_subtreeSize);
					}
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

void performRaycastAgainstCompound (RaycastGatheredObjectData* gatheredObjectData, const SpuRaycastTaskWorkUnit& workUnit, SpuRaycastTaskWorkUnitOut* workUnitOut, RaycastTask_LocalStoreMemory* lsMemPtr)
{
	//XXX spu_printf ("Currently no support for ray. vs compound objects. Support coming soon.\n");
}

void
performRaycastAgainstConvex (RaycastGatheredObjectData* gatheredObjectData, const SpuRaycastTaskWorkUnit& workUnit, SpuRaycastTaskWorkUnitOut* workUnitOut, RaycastTask_LocalStoreMemory* lsMemPtr)
{
	SpuVoronoiSimplexSolver simplexSolver;

	btTransform rayFromTrans, rayToTrans;
	rayFromTrans.setIdentity ();
	rayFromTrans.setOrigin (workUnit.rayFrom);
	rayToTrans.setIdentity ();
	rayToTrans.setOrigin (workUnit.rayTo);

	SpuCastResult result;

	/* Load the vertex data if the shape is a convex hull */
	/* XXX: We might be loading the shape twice */
	ATTRIBUTE_ALIGNED16(char convexHullShape[sizeof(btConvexHullShape)]);
	if (gatheredObjectData->m_shapeType == CONVEX_HULL_SHAPE_PROXYTYPE)
	{
		register int dmaSize;
		register ppu_address_t	dmaPpuAddress2;
		dmaSize = sizeof(btConvexHullShape);
		dmaPpuAddress2 = gatheredObjectData->m_collisionShape;
		cellDmaGet(&convexHullShape, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
		cellDmaWaitTagStatusAll(DMA_MASK(1));
		dmaConvexVertexData (&lsMemPtr->convexVertexData, (btConvexHullShape*)&convexHullShape);
		cellDmaWaitTagStatusAll(DMA_MASK(2)); // dmaConvexVertexData uses dma channel 2!
		lsMemPtr->convexVertexData.gSpuConvexShapePtr = gatheredObjectData->m_spuCollisionShape;
		lsMemPtr->convexVertexData.gConvexPoints = &lsMemPtr->convexVertexData.g_convexPointBuffer[0];
	}

	/* performRaycast */
	SpuSubsimplexRayCast caster (gatheredObjectData->m_spuCollisionShape, &lsMemPtr->convexVertexData, gatheredObjectData->m_shapeType, gatheredObjectData->m_collisionMargin, &simplexSolver);
	bool r = caster.calcTimeOfImpact (rayFromTrans, rayToTrans, gatheredObjectData->m_worldTransform, gatheredObjectData->m_worldTransform,result);

	if (r)
	{
		workUnitOut->hitFraction = result.m_fraction;
		workUnitOut->hitNormal = result.m_normal;
	}
}

void	processRaycastTask(void* userPtr, void* lsMemory)
{
	RaycastTask_LocalStoreMemory* localMemory = (RaycastTask_LocalStoreMemory*)lsMemory;

	SpuRaycastTaskDesc* taskDescPtr = (SpuRaycastTaskDesc*)userPtr;
	SpuRaycastTaskDesc& taskDesc = *taskDescPtr;

	SpuCollisionObjectWrapper* cows = (SpuCollisionObjectWrapper*)taskDesc.spuCollisionObjectsWrappers;

	//spu_printf("in processRaycastTask %d\n", taskDesc.numSpuCollisionObjectWrappers);
	/* for each object */
	RaycastGatheredObjectData gatheredObjectData;
	for (int objectId = 0; objectId < taskDesc.numSpuCollisionObjectWrappers; objectId++)
	{
		//spu_printf("%d / %d\n", objectId, taskDesc.numSpuCollisionObjectWrappers);
		
		/* load initial collision shape */
		GatherCollisionObjectAndShapeData (&gatheredObjectData, localMemory, (ppu_address_t)&cows[objectId]);

		if (btBroadphaseProxy::isConcave (gatheredObjectData.m_shapeType))
		{
			SpuRaycastTaskWorkUnitOut tWorkUnitsOut[SPU_RAYCAST_WORK_UNITS_PER_TASK];
			for (int rayId = 0; rayId < taskDesc.numWorkUnits; rayId++)
			{
				tWorkUnitsOut[rayId].hitFraction = 1.0;
			}

			performRaycastAgainstConcave (&gatheredObjectData, &taskDesc.workUnits[0], &tWorkUnitsOut[0], taskDesc.numWorkUnits, localMemory);

			for (int rayId = 0; rayId < taskDesc.numWorkUnits; rayId++)
			{
				const SpuRaycastTaskWorkUnit& workUnit = taskDesc.workUnits[rayId];
				if (tWorkUnitsOut[rayId].hitFraction == 1.0)
					continue;

				ATTRIBUTE_ALIGNED16(SpuRaycastTaskWorkUnitOut workUnitOut);
				dmaLoadRayOutput ((ppu_address_t)workUnit.output, &workUnitOut, 1);
				cellDmaWaitTagStatusAll(DMA_MASK(1));
				
				
				/* XXX Only support taking the closest hit for now */
				if (tWorkUnitsOut[rayId].hitFraction < workUnitOut.hitFraction)
				{
					workUnitOut.hitFraction = tWorkUnitsOut[rayId].hitFraction;
					workUnitOut.hitNormal = tWorkUnitsOut[rayId].hitNormal;
				}

				/* write ray cast data back */
				dmaStoreRayOutput ((ppu_address_t)workUnit.output, &workUnitOut, 1);
				cellDmaWaitTagStatusAll(DMA_MASK(1));
			}
		} else if (btBroadphaseProxy::isConvex (gatheredObjectData.m_shapeType)) {

			btVector3 objectBoxMin, objectBoxMax;
			computeAabb (objectBoxMin, objectBoxMax, (btConvexInternalShape*)gatheredObjectData.m_spuCollisionShape, gatheredObjectData.m_collisionShape, gatheredObjectData.m_shapeType, gatheredObjectData.m_worldTransform);
			for (unsigned int rayId = 0; rayId < taskDesc.numWorkUnits; rayId++)
			{
				const SpuRaycastTaskWorkUnit& workUnit = taskDesc.workUnits[rayId];
			
				btScalar ignored_param = 1.0;
				btVector3 ignored_normal;
				if (btRayAabb(workUnit.rayFrom, workUnit.rayTo, objectBoxMin, objectBoxMax, ignored_param, ignored_normal))
				{
					ATTRIBUTE_ALIGNED16(SpuRaycastTaskWorkUnitOut workUnitOut);
					SpuRaycastTaskWorkUnitOut tWorkUnitOut;
					tWorkUnitOut.hitFraction = 1.0;

					performRaycastAgainstConvex (&gatheredObjectData, workUnit, &tWorkUnitOut, localMemory);
					if (tWorkUnitOut.hitFraction == 1.0)
						continue;
	
					dmaLoadRayOutput ((ppu_address_t)workUnit.output, &workUnitOut, 1);
					cellDmaWaitTagStatusAll(DMA_MASK(1));

					/* XXX Only support taking the closest hit for now */
					if (tWorkUnitOut.hitFraction < workUnitOut.hitFraction)
					{
						workUnitOut.hitFraction = tWorkUnitOut.hitFraction;
						workUnitOut.hitNormal = tWorkUnitOut.hitNormal;
						/* write ray cast data back */
						dmaStoreRayOutput ((ppu_address_t)workUnit.output, &workUnitOut, 1);
						cellDmaWaitTagStatusAll(DMA_MASK(1));
					}
				}
			}

		} else if (btBroadphaseProxy::isCompound (gatheredObjectData.m_shapeType)) {
			for (unsigned int rayId = 0; rayId < taskDesc.numWorkUnits; rayId++)
			{
				const SpuRaycastTaskWorkUnit& workUnit = taskDesc.workUnits[rayId];
				ATTRIBUTE_ALIGNED16(SpuRaycastTaskWorkUnitOut workUnitOut);
				SpuRaycastTaskWorkUnitOut tWorkUnitOut;
				tWorkUnitOut.hitFraction = 1.0;

				performRaycastAgainstCompound (&gatheredObjectData, workUnit, &tWorkUnitOut, localMemory);
				if (tWorkUnitOut.hitFraction == 1.0)
					continue;

				dmaLoadRayOutput ((ppu_address_t)workUnit.output, &workUnitOut, 1);
				cellDmaWaitTagStatusAll(DMA_MASK(1));
				/* XXX Only support taking the closest hit for now */
				if (tWorkUnitOut.hitFraction < workUnitOut.hitFraction)
				{
					workUnitOut.hitFraction = tWorkUnitOut.hitFraction;
					workUnitOut.hitNormal = tWorkUnitOut.hitNormal;
				}

				/* write ray cast data back */
				dmaStoreRayOutput ((ppu_address_t)workUnit.output, &workUnitOut, 1);
				cellDmaWaitTagStatusAll(DMA_MASK(1));
			}
		}
	}
}
