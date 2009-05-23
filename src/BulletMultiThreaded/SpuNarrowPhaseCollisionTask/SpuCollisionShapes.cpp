/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "SpuCollisionShapes.h"

///not supported on IBM SDK, until we fix the alignment of btVector3
#if defined (__CELLOS_LV2__) && defined (__SPU__)
#include <spu_intrinsics.h>
static inline vec_float4 vec_dot3( vec_float4 vec0, vec_float4 vec1 )
{
    vec_float4 result;
    result = spu_mul( vec0, vec1 );
    result = spu_madd( spu_rlqwbyte( vec0, 4 ), spu_rlqwbyte( vec1, 4 ), result );
    return spu_madd( spu_rlqwbyte( vec0, 8 ), spu_rlqwbyte( vec1, 8 ), result );
}
#endif //__SPU__

btVector3 localGetSupportingVertexWithoutMargin(int shapeType, void* shape, const btVector3& localDir,struct	SpuConvexPolyhedronVertexData* convexVertexData)//, int *featureIndex)
{
    switch (shapeType)
    {
    case SPHERE_SHAPE_PROXYTYPE:
        {
            return btVector3(0,0,0);
        }
	case BOX_SHAPE_PROXYTYPE:
		{
//			spu_printf("SPU: getSupport BOX_SHAPE_PROXYTYPE\n");
			btConvexInternalShape* convexShape = (btConvexInternalShape*)shape;
			const btVector3& halfExtents = convexShape->getImplicitShapeDimensions();
			
			return btVector3(
				localDir.getX() < 0.0f ? -halfExtents.x() : halfExtents.x(),
							localDir.getY() < 0.0f ? -halfExtents.y() : halfExtents.y(),
							localDir.getZ() < 0.0f ? -halfExtents.z() : halfExtents.z());
		}

	case TRIANGLE_SHAPE_PROXYTYPE:
		{

			btVector3 dir(localDir.getX(),localDir.getY(),localDir.getZ());
			btVector3* vertices = (btVector3*)shape;
			btVector3 dots(dir.dot(vertices[0]), dir.dot(vertices[1]), dir.dot(vertices[2]));
	  		btVector3 sup = vertices[dots.maxAxis()];
			return btVector3(sup.getX(),sup.getY(),sup.getZ());
			break;
		}

	case CYLINDER_SHAPE_PROXYTYPE:
		{
			btCylinderShape* cylShape = (btCylinderShape*)shape;

			//mapping of halfextents/dimension onto radius/height depends on how cylinder local orientation is (upAxis)

			btVector3 halfExtents = cylShape->getImplicitShapeDimensions();
			btVector3 v(localDir.getX(),localDir.getY(),localDir.getZ());
			
			int cylinderUpAxis = cylShape->getUpAxis();
			int XX(1),YY(0),ZZ(2);

			switch (cylinderUpAxis)
			{
			case 0:
				{
					XX = 1;
					YY = 0;
					ZZ = 2;
					break;
				}
			case 1:
				{
					XX = 0;
					YY = 1;
					ZZ = 2;
				break;
				}
			case 2:
				{
					XX = 0;
					YY = 2;
					ZZ = 1;
					break;
				}
			default:
				btAssert(0);
				//printf("SPU:localGetSupportingVertexWithoutMargin unknown Cylinder up-axis\n");
			};

			btScalar radius = halfExtents[XX];
			btScalar halfHeight = halfExtents[cylinderUpAxis];

			btVector3 tmp;
			btScalar d ;

			btScalar s = btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
			if (s != btScalar(0.0))
			{
				d = radius / s;  
				tmp[XX] = v[XX] * d;
				tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				tmp[ZZ] = v[ZZ] * d;
				return btVector3(tmp.getX(),tmp.getY(),tmp.getZ());
			}
			else
			{
				tmp[XX] = radius;
				tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				tmp[ZZ] = btScalar(0.0);
				return btVector3(tmp.getX(),tmp.getY(),tmp.getZ());
			}
		}

	case CAPSULE_SHAPE_PROXYTYPE:
	{
		//spu_printf("SPU: todo: getSupport CAPSULE_SHAPE_PROXYTYPE\n");
		btVector3 vec0(localDir.getX(),localDir.getY(),localDir.getZ());

		btCapsuleShape* capsuleShape = (btCapsuleShape*)shape;
		btVector3 halfExtents = capsuleShape->getImplicitShapeDimensions();
		btScalar halfHeight = capsuleShape->getHalfHeight();
		int capsuleUpAxis = capsuleShape->getUpAxis();

		btScalar radius = capsuleShape->getRadius();
		btVector3 supVec(0,0,0);

		btScalar maxDot(btScalar(-BT_LARGE_FLOAT));

		btVector3 vec = vec0;
		btScalar lenSqr = vec.length2();
		if (lenSqr < btScalar(0.0001))
		{
			vec.setValue(1,0,0);
		} else
		{
			btScalar rlen = btScalar(1.) / btSqrt(lenSqr );
			vec *= rlen;
		}
		btVector3 vtx;
		btScalar newDot;
		{
			btVector3 pos(0,0,0);
			pos[capsuleUpAxis] = halfHeight;

			vtx = pos +vec*(radius);
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supVec = vtx;
			}
		}
		{
			btVector3 pos(0,0,0);
			pos[capsuleUpAxis] = -halfHeight;

			vtx = pos +vec*(radius);
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supVec = vtx;
			}
		}
		return btVector3(supVec.getX(),supVec.getY(),supVec.getZ());
		break;
	};

	case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			//spu_printf("SPU: todo: getSupport CONVEX_HULL_SHAPE_PROXYTYPE\n");

#if defined (__CELLOS_LV2__) && defined (__SPU__)
			vec_float4 v_distMax = {-FLT_MAX,0,0,0};
			vec_int4 v_idxMax = {-999,0,0,0};
			int v=0;
			int numverts = convexVertexData->gNumConvexPoints;
			btVector3* points = convexVertexData->gConvexPoints;

			for(;v<(int)numverts-4;v+=4) {
				vec_float4 p0 = vec_dot3(points[v  ].get128(),localDir.get128());
				vec_float4 p1 = vec_dot3(points[v+1].get128(),localDir.get128());
				vec_float4 p2 = vec_dot3(points[v+2].get128(),localDir.get128());
				vec_float4 p3 = vec_dot3(points[v+3].get128(),localDir.get128());
				const vec_int4 i0 = {v  ,0,0,0};
				const vec_int4 i1 = {v+1,0,0,0};
				const vec_int4 i2 = {v+2,0,0,0};
				const vec_int4 i3 = {v+3,0,0,0};
				vec_uint4  retGt01 = spu_cmpgt(p0,p1);
				vec_float4 pmax01 = spu_sel(p1,p0,retGt01);
				vec_int4   imax01 = spu_sel(i1,i0,retGt01);
				vec_uint4  retGt23 = spu_cmpgt(p2,p3);
				vec_float4 pmax23 = spu_sel(p3,p2,retGt23);
				vec_int4   imax23 = spu_sel(i3,i2,retGt23);
				vec_uint4  retGt0123 = spu_cmpgt(pmax01,pmax23);
				vec_float4 pmax0123 = spu_sel(pmax23,pmax01,retGt0123);
				vec_int4   imax0123 = spu_sel(imax23,imax01,retGt0123);
				vec_uint4  retGtMax = spu_cmpgt(v_distMax,pmax0123);
				v_distMax = spu_sel(pmax0123,v_distMax,retGtMax);
				v_idxMax = spu_sel(imax0123,v_idxMax,retGtMax);
			}
			for(;v<(int)numverts;v++) {
				vec_float4 p = vec_dot3(points[v].get128(),localDir.get128());
				const vec_int4 i = {v,0,0,0};
				vec_uint4  retGtMax = spu_cmpgt(v_distMax,p);
				v_distMax = spu_sel(p,v_distMax,retGtMax);
				v_idxMax = spu_sel(i,v_idxMax,retGtMax);
			}
			int ptIndex = spu_extract(v_idxMax,0);
			const btVector3& supVec= points[ptIndex];
#else

			btVector3* points = 0;
			int numPoints = 0;
			points = convexVertexData->gConvexPoints;
			numPoints = convexVertexData->gNumConvexPoints;

		//	spu_printf("numPoints = %d\n",numPoints);

			int ptIndex = 0;
			btScalar newDot,maxDot = btScalar(-BT_LARGE_FLOAT);

			btVector3 vec0(localDir.getX(),localDir.getY(),localDir.getZ());
			btVector3 vec = vec0;
			btScalar lenSqr = vec.length2();
			if (lenSqr < btScalar(0.0001))
			{
				vec.setValue(1,0,0);
			} else
			{
				btScalar rlen = btScalar(1.) / btSqrt(lenSqr );
				vec *= rlen;
			}


			for (int i=0;i<numPoints;i++)
			{
				const btVector3& vtx = points[i];// * m_localScaling;

				newDot = vec.dot(vtx);
				if (newDot > maxDot)
				{
					maxDot = newDot;
					ptIndex = i;
				}
			}
			const btVector3& supVec= points[ptIndex];
			
#endif
			return btVector3(supVec.getX(),supVec.getY(),supVec.getZ());

			break;
		};

    default:

		//spu_printf("SPU:(type %i) missing support function\n",shapeType);

		
#if __ASSERT
       // spu_printf("localGetSupportingVertexWithoutMargin() - Unsupported bound type: %d.\n", shapeType);
#endif // __ASSERT
        return btVector3(0.f, 0.f, 0.f);
    }
}

void computeAabb (btVector3& aabbMin, btVector3& aabbMax, btConvexInternalShape* convexShape, ppu_address_t convexShapePtr, int shapeType, const btTransform& xform)
{
	//calculate the aabb, given the types...
	switch (shapeType)
	{
	case CYLINDER_SHAPE_PROXYTYPE:
		/* fall through */
	case BOX_SHAPE_PROXYTYPE:
	{
		btScalar margin=convexShape->getMarginNV();
		btVector3 halfExtents = convexShape->getImplicitShapeDimensions();
		halfExtents += btVector3(margin,margin,margin);
		const btTransform& t = xform;
		btMatrix3x3 abs_b = t.getBasis().absolute();  
		btVector3 center = t.getOrigin();
		btVector3 extent = btVector3(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));
		
		aabbMin = center - extent;
		aabbMax = center + extent;
		break;
	}
	case CAPSULE_SHAPE_PROXYTYPE:
	{
		btScalar margin=convexShape->getMarginNV();
		btVector3 halfExtents = convexShape->getImplicitShapeDimensions();
		//add the radius to y-axis to get full height
		btScalar radius = halfExtents[0];
		halfExtents[1] += radius;
		halfExtents += btVector3(margin,margin,margin);
#if 0
		int capsuleUpAxis = convexShape->getUpAxis();
		btScalar halfHeight = convexShape->getHalfHeight();
		btScalar radius = convexShape->getRadius();
		halfExtents[capsuleUpAxis] = radius + halfHeight;
#endif
		const btTransform& t = xform;
		btMatrix3x3 abs_b = t.getBasis().absolute();  
		btVector3 center = t.getOrigin();
		btVector3 extent = btVector3(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));
		
		aabbMin = center - extent;
		aabbMax = center + extent;
		break;
	}
	case SPHERE_SHAPE_PROXYTYPE:
	{
		btScalar radius = convexShape->getImplicitShapeDimensions().getX();// * convexShape->getLocalScaling().getX();
		btScalar margin = radius + convexShape->getMarginNV();
		const btTransform& t = xform;
		const btVector3& center = t.getOrigin();
		btVector3 extent(margin,margin,margin);
		aabbMin = center - extent;
		aabbMax = center + extent;
		break;
	}
	case CONVEX_HULL_SHAPE_PROXYTYPE:
	{
		ATTRIBUTE_ALIGNED16(char convexHullShape0[sizeof(btConvexHullShape)]);
		cellDmaGet(&convexHullShape0, convexShapePtr  , sizeof(btConvexHullShape), DMA_TAG(1), 0, 0);
		cellDmaWaitTagStatusAll(DMA_MASK(1));
		btConvexHullShape* localPtr = (btConvexHullShape*)&convexHullShape0;
		const btTransform& t = xform;
		btScalar margin = convexShape->getMarginNV();
		localPtr->getNonvirtualAabb(t,aabbMin,aabbMax,margin);
		//spu_printf("SPU convex aabbMin=%f,%f,%f=\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
		//spu_printf("SPU convex aabbMax=%f,%f,%f=\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());
		break;
	}
	default:
		{
	//	spu_printf("SPU: unsupported shapetype %d in AABB calculation\n");
		}
	};
}

void dmaBvhShapeData (bvhMeshShape_LocalStoreMemory* bvhMeshShape, btBvhTriangleMeshShape* triMeshShape)
{
	register int dmaSize;
	register ppu_address_t	dmaPpuAddress2;

	dmaSize = sizeof(btTriangleIndexVertexArray);
	dmaPpuAddress2 = reinterpret_cast<ppu_address_t>(triMeshShape->getMeshInterface());
	//	spu_printf("trimeshShape->getMeshInterface() == %llx\n",dmaPpuAddress2);
#ifdef __SPU__
	cellDmaGet(&bvhMeshShape->gTriangleMeshInterfaceStorage, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
	bvhMeshShape->gTriangleMeshInterfacePtr = &bvhMeshShape->gTriangleMeshInterfaceStorage;
#else
	bvhMeshShape->gTriangleMeshInterfacePtr = (btTriangleIndexVertexArray*)cellDmaGetReadOnly(&bvhMeshShape->gTriangleMeshInterfaceStorage, dmaPpuAddress2  , dmaSize, DMA_TAG(1), 0, 0);
#endif

	//cellDmaWaitTagStatusAll(DMA_MASK(1));
	
	///now DMA over the BVH
	
	dmaSize = sizeof(btOptimizedBvh);
	dmaPpuAddress2 = reinterpret_cast<ppu_address_t>(triMeshShape->getOptimizedBvh());
	//spu_printf("trimeshShape->getOptimizedBvh() == %llx\n",dmaPpuAddress2);
	cellDmaGet(&bvhMeshShape->gOptimizedBvh, dmaPpuAddress2  , dmaSize, DMA_TAG(2), 0, 0);
	//cellDmaWaitTagStatusAll(DMA_MASK(2));
	cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
}

void dmaBvhIndexedMesh (btIndexedMesh* IndexMesh, IndexedMeshArray& indexArray, int index, uint32_t dmaTag)
{		
	cellDmaGet(IndexMesh, (ppu_address_t)&indexArray[index]  , sizeof(btIndexedMesh), DMA_TAG(dmaTag), 0, 0);
	
}

void dmaBvhSubTreeHeaders (btBvhSubtreeInfo* subTreeHeaders, ppu_address_t subTreePtr, int batchSize, uint32_t dmaTag)
{
	cellDmaGet(subTreeHeaders, subTreePtr, batchSize * sizeof(btBvhSubtreeInfo), DMA_TAG(dmaTag), 0, 0);
}

void dmaBvhSubTreeNodes (btQuantizedBvhNode* nodes, const btBvhSubtreeInfo& subtree, QuantizedNodeArray&	nodeArray, int dmaTag)
{
	cellDmaGet(nodes, reinterpret_cast<ppu_address_t>(&nodeArray[subtree.m_rootNodeIndex]) , subtree.m_subtreeSize* sizeof(btQuantizedBvhNode), DMA_TAG(2), 0, 0);
}

///getShapeTypeSize could easily be optimized, but it is not likely a bottleneck
int		getShapeTypeSize(int shapeType)
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

void dmaConvexVertexData (SpuConvexPolyhedronVertexData* convexVertexData, btConvexHullShape* convexShapeSPU)
{
	convexVertexData->gNumConvexPoints = convexShapeSPU->getNumPoints();
	if (convexVertexData->gNumConvexPoints>MAX_NUM_SPU_CONVEX_POINTS)
	{
		btAssert(0);
	//	spu_printf("SPU: Error: MAX_NUM_SPU_CONVEX_POINTS(%d) exceeded: %d\n",MAX_NUM_SPU_CONVEX_POINTS,convexVertexData->gNumConvexPoints);
		return;
	}
			
	register int dmaSize = convexVertexData->gNumConvexPoints*sizeof(btVector3);
	ppu_address_t pointsPPU = (ppu_address_t) convexShapeSPU->getUnscaledPoints();
	cellDmaGet(&convexVertexData->g_convexPointBuffer[0], pointsPPU  , dmaSize, DMA_TAG(2), 0, 0);
}

void dmaCollisionShape (void* collisionShapeLocation, ppu_address_t collisionShapePtr, uint32_t dmaTag, int shapeType)
{
	register int dmaSize = getShapeTypeSize(shapeType);
	cellDmaGet(collisionShapeLocation, collisionShapePtr  , dmaSize, DMA_TAG(dmaTag), 0, 0);
	//cellDmaWaitTagStatusAll(DMA_MASK(dmaTag));
}

void dmaCompoundShapeInfo (CompoundShape_LocalStoreMemory* compoundShapeLocation, btCompoundShape* spuCompoundShape, uint32_t dmaTag)
{
	register int dmaSize;
	register	ppu_address_t	dmaPpuAddress2;
	int childShapeCount = spuCompoundShape->getNumChildShapes();
	dmaSize = childShapeCount * sizeof(btCompoundShapeChild);
	dmaPpuAddress2 = (ppu_address_t)spuCompoundShape->getChildList();
	cellDmaGet(&compoundShapeLocation->gSubshapes[0], dmaPpuAddress2, dmaSize, DMA_TAG(dmaTag), 0, 0);
}

void dmaCompoundSubShapes (CompoundShape_LocalStoreMemory* compoundShapeLocation, btCompoundShape* spuCompoundShape, uint32_t dmaTag)
{
	int childShapeCount = spuCompoundShape->getNumChildShapes();
	int i;
	// DMA all the subshapes 
	for ( i = 0; i < childShapeCount; ++i)
	{
		btCompoundShapeChild& childShape = compoundShapeLocation->gSubshapes[i];
		dmaCollisionShape (&compoundShapeLocation->gSubshapeShape[i],(ppu_address_t)childShape.m_childShape, dmaTag, childShape.m_childShapeType);
	}
}


void	spuWalkStacklessQuantizedTree(btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax,const btQuantizedBvhNode* rootNode,int startNodeIndex,int endNodeIndex)
{

	int curIndex = startNodeIndex;
	int walkIterations = 0;
#ifdef BT_DEBUG
	int subTreeSize = endNodeIndex - startNodeIndex;
#endif

	int escapeIndex;

	unsigned int aabbOverlap, isLeafNode;

	while (curIndex < endNodeIndex)
	{
		//catch bugs in tree data
		btAssert (walkIterations < subTreeSize);

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
