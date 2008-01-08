#include <stdio.h>

#include "SpuRaycastTask.h"
#include "SpuCollisionObjectWrapper.h"
#include "SpuNarrowPhaseCollisionTask/SpuCollisionShapes.h"



struct RaycastTask_LocalStoreMemory
{
	ATTRIBUTE_ALIGNED16(char gColObj [sizeof(btCollisionObject)+16]);
	btCollisionObject* getColObj()
	{
		return (btCollisionObject*) gColObj;
	}

	SpuCollisionObjectWrapper gCollisionObjectWrapper;
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

void GatherCollisionObjectAndShapeData (RaycastGatheredObjectData& gatheredObjectData, RaycastTask_LocalStoreMemory& lsMem, ppu_address_t objectWrapper)
{
	register int dmaSize;
	register ppu_address_t	dmaPpuAddress2;

	/* DMA Collision object wrapper into local store */
	dmaSize = sizeof(SpuCollisionObjectWrapper);
	dmaPpuAddress2 = objectWrapper;
	cellDmaGet(&lsMem.gCollisionObjectWrapper, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
	cellDmaWaitTagStatusAll(DMA_MASK(1));

	/* DMA Collision object into local store */
	dmaSize = sizeof(btCollisionObject);
	dmaPpuAddress2 = lsMem.getCollisionObjectWrapper()->getCollisionObjectPtr();
	cellDmaGet(&lsMem.gColObj, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);
	cellDmaWaitTagStatusAll(DMA_MASK(2));
	
	/* Gather information about collision object and shape */
	gatheredObjectData.m_worldTransform = lsMem.getColObj()->getWorldTransform();
	gatheredObjectData.m_collisionMargin = lsMem.getCollisionObjectWrapper()->getCollisionMargin ();
	gatheredObjectData.m_shapeType = lsMem.getCollisionObjectWrapper()->getShapeType ();
	gatheredObjectData.m_collisionShape = (ppu_address_t)lsMem.getColObj()->getCollisionShape();
	gatheredObjectData.m_spuCollisionShape = (void*)&lsMem.gCollisionShape.collisionShape[0];

	/* DMA shape data */
	dmaCollisionShape (gatheredObjectData.m_spuCollisionShape, gatheredObjectData.m_collisionShape, 1, gatheredObjectData.m_shapeType);
	cellDmaWaitTagStatusAll(DMA_MASK(1));
	btConvexInternalShape* spuConvexShape = (btConvexInternalShape*)gatheredObjectData.m_spuCollisionShape;
	gatheredObjectData.m_primitiveDimensions = spuConvexShape->getImplicitShapeDimensions ();
}

void dmaLoadRayOutput (ppu_address_t rayOutputAddr, SpuRaycastTaskWorkUnitOut* rayOutput, uint32_t dmaTag)
{
	cellDmaGet(rayOutput, rayOutputAddr, sizeof(*rayOutput), DMA_TAG(dmaTag), 0, 0);
}

void dmaStoreRayOutput (ppu_address_t rayOutputAddr, const SpuRaycastTaskWorkUnitOut* rayOutput, uint32_t dmaTag)
{
	cellDmaLargePut (rayOutput, rayOutputAddr, sizeof(*rayOutput), DMA_TAG(dmaTag), 0, 0);
}

void	processRaycastTask(void* userPtr, void* lsMemory)
{
	RaycastTask_LocalStoreMemory* localMemory = (RaycastTask_LocalStoreMemory*)lsMemory;

	SpuRaycastTaskDesc* taskDescPtr = (SpuRaycastTaskDesc*)userPtr;
	SpuRaycastTaskDesc& taskDesc = *taskDescPtr;

	SpuCollisionObjectWrapper* cows = (SpuCollisionObjectWrapper*)taskDesc.spuCollisionObjectsWrappers;

	/* for each object */
	for (int objectId = 0; objectId < taskDesc.numSpuCollisionObjectWrappers; objectId++)
	{
		RaycastGatheredObjectData gatheredObjectData;
		GatherCollisionObjectAndShapeData (gatheredObjectData, *localMemory, (ppu_address_t)&cows[objectId]);
		/* load initial collision shape */
		for (int rayId = 0; rayId < taskDesc.numWorkUnits; rayId++)
		{
			SpuRaycastTaskWorkUnitOut rayOut;

			dmaLoadRayOutput ((ppu_address_t)taskDesc.workUnits[rayId].output, &rayOut, 1);
			cellDmaWaitTagStatusAll(DMA_MASK(1));
			
			float t = (float)rayId/(float)taskDesc.numWorkUnits;
			/* performRaycast */
			rayOut.hitFraction = 0.1f * t;
			rayOut.hitNormal = btVector3(1.0, 0.0, 0.0);

			/* write ray cast data back */
			dmaStoreRayOutput ((ppu_address_t)taskDesc.workUnits[rayId].output, &rayOut, 1);
			cellDmaWaitTagStatusAll(DMA_MASK(1));
		}
	}
	
}
