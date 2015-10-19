#define BLAAT
#include "RealTimeBullet3CollisionSdk.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3Collidable.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3ConvexPolyhedronData.h"

enum RTB3ShapeTypes
{
	RTB3_SHAPE_SPHERE=0,
	RTB3_SHAPE_PLANE,
//	RTB3_SHAPE_CAPSULE,
	MAX_NUM_SINGLE_SHAPE_TYPES,
	RTB3_SHAPE_COMPOUND_INTERNAL

	
};

//we start at 1, so that the 0 index is 'invalid' just like a nullptr
#define START_COLLIDABLE_INDEX 1

struct RTB3CollisionWorld
{
	b3AlignedObjectArray<void*> m_collidableUserPointers;
	b3AlignedObjectArray<int> m_collidableUserIndices;
	b3AlignedObjectArray<b3Float4> m_collidablePositions;
	b3AlignedObjectArray<b3Quaternion> m_collidableOrientations;
	b3AlignedObjectArray<b3Collidable> m_collidables;
	b3AlignedObjectArray<b3GpuChildShape> m_childShapes;
	b3AlignedObjectArray<b3GpuFace> m_planeFaces;
	b3AlignedObjectArray<b3CompoundOverlappingPair> m_compoundOverlappingPairs;
	int m_nextFreeShapeIndex;
	int m_nextFreeCollidableIndex;

	RTB3CollisionWorld()
	:m_nextFreeCollidableIndex(START_COLLIDABLE_INDEX),
	m_nextFreeShapeIndex(START_COLLIDABLE_INDEX)
	{
	}
};

struct RealTimeBullet3CollisionSdkInternalData
{
	b3AlignedObjectArray<RTB3CollisionWorld*> m_collisionWorlds;
};

RealTimeBullet3CollisionSdk::RealTimeBullet3CollisionSdk()
{
	int szCol = sizeof(b3Collidable);
	int szShap = sizeof(b3GpuChildShape);
	int szComPair = sizeof(b3CompoundOverlappingPair);
	m_internalData = new RealTimeBullet3CollisionSdkInternalData;
}
	
RealTimeBullet3CollisionSdk::~RealTimeBullet3CollisionSdk()
{
	delete m_internalData;
	m_internalData=0;
}
	
plCollisionWorldHandle RealTimeBullet3CollisionSdk::createCollisionWorld(int maxNumObjsCapacity, int maxNumShapesCapacity, int maxNumPairsCapacity)
{
	RTB3CollisionWorld* world = new RTB3CollisionWorld();
	world->m_collidables.resize(maxNumObjsCapacity);
	world->m_collidablePositions.resize(maxNumObjsCapacity);
	world->m_collidableOrientations.resize(maxNumObjsCapacity);
	world->m_collidableUserPointers.resize(maxNumObjsCapacity);
	world->m_collidableUserIndices.resize(maxNumObjsCapacity);
	world->m_childShapes.resize(maxNumShapesCapacity);
	world->m_compoundOverlappingPairs.resize(maxNumPairsCapacity);

	m_internalData->m_collisionWorlds.push_back(world);
	return (plCollisionWorldHandle) world;
}
	
void RealTimeBullet3CollisionSdk::deleteCollisionWorld(plCollisionWorldHandle worldHandle)
{
	RTB3CollisionWorld* world = (RTB3CollisionWorld*) worldHandle;
	int loc = m_internalData->m_collisionWorlds.findLinearSearch(world);
	b3Assert(loc >=0 && loc<m_internalData->m_collisionWorlds.size());
	if (loc >=0 && loc<m_internalData->m_collisionWorlds.size())
	{
		m_internalData->m_collisionWorlds.remove(world);
		delete world;
	}
}

plCollisionShapeHandle RealTimeBullet3CollisionSdk::createSphereShape(plCollisionWorldHandle worldHandle, plReal radius)
{
	RTB3CollisionWorld* world = (RTB3CollisionWorld*) worldHandle;
	if (world->m_nextFreeShapeIndex<world->m_childShapes.size())
	{
		b3GpuChildShape& shape = world->m_childShapes[world->m_nextFreeShapeIndex];
		shape.m_childPosition.setZero();
		shape.m_childOrientation.setValue(0,0,0,1);
		shape.m_radius = radius;
		shape.m_shapeType = RTB3_SHAPE_SPHERE;
		return (plCollisionShapeHandle) world->m_nextFreeShapeIndex++;
	}
	return 0;
}
	
void RealTimeBullet3CollisionSdk::deleteShape(plCollisionWorldHandle worldHandle, plCollisionShapeHandle shape)
{
	///todo
	//deleting shapes would involve a garbage collection phase, and mess up all user indices
	//this would be solved by one more in-direction, at some performance penalty for certain operations
	//for now, we don't delete and eventually run out-of-shapes
}
	
void RealTimeBullet3CollisionSdk::addCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object)
{
	///createCollisionObject already adds it to the world
}

void RealTimeBullet3CollisionSdk::removeCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object)
{
	///todo, see deleteShape
}
	
plCollisionObjectHandle RealTimeBullet3CollisionSdk::createCollisionObject(  plCollisionWorldHandle worldHandle, void* userPointer, 
														int userIndex,  plCollisionShapeHandle shapeHandle ,
                                                        plVector3 startPosition,plQuaternion startOrientation )
{
	RTB3CollisionWorld* world = (RTB3CollisionWorld*) worldHandle;
	if (world->m_nextFreeCollidableIndex < world->m_collidables.size())
	{
		b3Collidable& collidable = world->m_collidables[world->m_nextFreeCollidableIndex];
		world->m_collidablePositions[world->m_nextFreeCollidableIndex].setValue(startPosition[0],startPosition[1],startPosition[2]);
		world->m_collidableOrientations[world->m_nextFreeCollidableIndex].setValue(startOrientation[0],startOrientation[1],startOrientation[2],startOrientation[3]);
		world->m_collidableUserPointers[world->m_nextFreeCollidableIndex] = userPointer;
		world->m_collidableUserIndices[world->m_nextFreeCollidableIndex] = userIndex;
		int shapeIndex = (int)shapeHandle;
		collidable.m_shapeIndex = shapeIndex;
		b3GpuChildShape& shape = world->m_childShapes[shapeIndex];
		collidable.m_shapeType = shape.m_shapeType;
		collidable.m_numChildShapes = 1;

		switch (collidable.m_shapeType)
		{
		case RTB3_SHAPE_SPHERE:
			{
				break;
			}
		case RTB3_SHAPE_PLANE:
			{
				b3Assert(0);
				break;
			}
		case RTB3_SHAPE_COMPOUND_INTERNAL:
			{
				b3Assert(0);
				break;
			}
		default:
			{
				b3Assert(0);
			}
		}

		/*case SHAPE_COMPOUND_OF_CONVEX_HULLS:
		case SHAPE_COMPOUND_OF_SPHERES:
		case SHAPE_COMPOUND_OF_CAPSULES:
			{
				collidable.m_numChildShapes = shape.m_numChildShapes;
				collidable.m_shapeIndex = shape.m_shapeIndex;
				break;
		*/
		return (plCollisionObjectHandle)world->m_nextFreeCollidableIndex++;
	}
	return 0;
}
  
void RealTimeBullet3CollisionSdk::deleteCollisionObject(plCollisionObjectHandle body)
{
	///todo, see deleteShape
}
	
void RealTimeBullet3CollisionSdk::setCollisionObjectTransform(plCollisionWorldHandle world, plCollisionObjectHandle body,
											plVector3 position,plQuaternion orientation )
{
}

struct plContactCache
{
	lwContactPoint* pointsOut;
	int pointCapacity;
	int numAddedPoints;
};

typedef void (*plDetectCollisionFunc)(RTB3CollisionWorld* world,int colA, int shapeIndexA, int colB, int shapeIndexB,
									plContactCache* contactCache);

void detectCollisionDummy(RTB3CollisionWorld* world,int colA, int shapeIndexA, int colB, int shapeIndexB,
									plContactCache* contactCache)
{
	(void)world;
	(void)colA,(void)colB;
	(void)contactCache;
}

void plVecCopy(b3Scalar* dst,const b3Vector3& src)
{
	dst[0] = src.x;
	dst[1] = src.y;
	dst[2] = src.z;
}

void ComputeClosestPointsPlaneSphere(const b3Vector3& planeNormalWorld, b3Scalar planeConstant, const b3Vector3& spherePosWorld,b3Scalar sphereRadius,  plContactCache* contactCache) 
{
	if (contactCache->numAddedPoints < contactCache->pointCapacity)
	{
		lwContactPoint& pointOut = contactCache->pointsOut[contactCache->numAddedPoints];
		b3Scalar t = -(spherePosWorld.dot(-planeNormalWorld)+planeConstant);
		b3Vector3 intersectionPoint = spherePosWorld+t*-planeNormalWorld;
		b3Scalar distance = t-sphereRadius;
		if (distance<=0)
		{
			pointOut.m_distance = distance;
			plVecCopy(pointOut.m_ptOnBWorld,intersectionPoint);
			plVecCopy(pointOut.m_ptOnAWorld,spherePosWorld+sphereRadius*-planeNormalWorld);
			plVecCopy(pointOut.m_normalOnB,planeNormalWorld);
			contactCache->numAddedPoints++;
		}
	}
}

void ComputeClosestPointsSphereSphere(b3Scalar sphereARadius, const b3Vector3& sphereAPosWorld, b3Scalar sphereBRadius, const b3Vector3& sphereBPosWorld, plContactCache* contactCache) {

	if (contactCache->numAddedPoints < contactCache->pointCapacity)
	{
		lwContactPoint& pointOut = contactCache->pointsOut[contactCache->numAddedPoints];
		b3Vector3 diff = sphereAPosWorld-sphereBPosWorld;
		b3Scalar len = diff.length();
		pointOut.m_distance = len - (sphereARadius+sphereBRadius);
		if (pointOut.m_distance<=0)
		{
			b3Vector3 normOnB = b3MakeVector3(1,0,0);
			if (len > B3_EPSILON) {
				normOnB = diff / len;
			}
			plVecCopy(pointOut.m_normalOnB,normOnB);
			b3Vector3 ptAWorld = sphereAPosWorld - sphereARadius*normOnB;
			plVecCopy(pointOut.m_ptOnAWorld,ptAWorld);
			plVecCopy(pointOut.m_ptOnBWorld,ptAWorld-normOnB*pointOut.m_distance);
			contactCache->numAddedPoints++;
		}
	}
}

void detectCollisionSphereSphere(RTB3CollisionWorld* world,int colA, int shapeIndexA, int colB, int shapeIndexB,
									plContactCache* contactCache)
{
	b3Scalar radiusA = world->m_childShapes[shapeIndexA].m_radius;
	b3Scalar radiusB = world->m_childShapes[shapeIndexB].m_radius;
	b3Transform trA(world->m_collidableOrientations[colA],world->m_collidablePositions[colA]);
	const b3Vector3& sphereALocalPos = world->m_childShapes[shapeIndexA].m_childPosition;
	b3Vector3 spherePosAWorld = trA(sphereALocalPos);
	b3Transform trB(world->m_collidableOrientations[colB],world->m_collidablePositions[colB]);
	const b3Vector3& sphereBLocalPos = world->m_childShapes[shapeIndexB].m_childPosition;
	b3Vector3 spherePosBWorld = trB(sphereALocalPos);
	ComputeClosestPointsSphereSphere(radiusA,spherePosAWorld,radiusB,spherePosBWorld,contactCache);
}

void detectCollisionSpherePlane(RTB3CollisionWorld* world,int colA, int shapeIndexA, int colB, int shapeIndexB,
									plContactCache* contactCache)
{
	(void)world;
	(void)colA,(void)shapeIndexA,(void)colB,(void)shapeIndexB;
	(void)contactCache;
}

void detectCollisionPlaneSphere(RTB3CollisionWorld* world,int colA, int shapeIndexA, int colB, int shapeIndexB,
									plContactCache* contactCache)
{
	(void)world;
	(void)colA,(void)shapeIndexA,(void)colB,(void)shapeIndexB;
	(void)contactCache;
}



#ifdef RTB3_SHAPE_CAPSULE
plDetectCollisionFunc funcTbl_detectCollision[MAX_NUM_SINGLE_SHAPE_TYPES,][MAX_NUM_SINGLE_SHAPE_TYPES,] = {
	{detectCollisionSphereSphere	,detectCollisionSpherePlane	,detectCollisionSphereCapsule},
	{detectCollisionPlaneSphere		,detectCollisionDummy		,detectCollisionPlaneCapsule},
	{detectCollisionCapsuleSphere	,detectCollisionCapsulePlane	,detectCollisionCapsuleCapsule},
};
#else
plDetectCollisionFunc funcTbl_detectCollision[MAX_NUM_SINGLE_SHAPE_TYPES][MAX_NUM_SINGLE_SHAPE_TYPES] = {
	{detectCollisionSphereSphere	,detectCollisionSpherePlane},
	{detectCollisionPlaneSphere		,detectCollisionDummy		},
};

#endif

int RealTimeBullet3CollisionSdk::collide(plCollisionWorldHandle worldHandle,plCollisionObjectHandle colAHandle, plCollisionObjectHandle colBHandle,
                    lwContactPoint* pointsOutOrg, int pointCapacity)
{
	RTB3CollisionWorld* world = (RTB3CollisionWorld*) worldHandle;
	int colAIndex = (int) colAHandle;
	int colBIndex = (int) colBHandle;
	const b3Collidable& colA = world->m_collidables[colAIndex];
	const b3Collidable& colB = world->m_collidables[colBIndex];

	plContactCache contactCache;
	contactCache.pointCapacity = pointCapacity;
	contactCache.pointsOut = pointsOutOrg;
	contactCache.numAddedPoints = 0;
	int remainingCapacity = pointCapacity;

	for (int i=0;i<colA.m_numChildShapes;i++)
	{
		for (int j=0;j<colB.m_numChildShapes;j++)
		{
			if (contactCache.numAddedPoints<pointCapacity)
			{
				lwContactPoint* curPointsOut = &pointsOutOrg[contactCache.numAddedPoints];
				funcTbl_detectCollision[world->m_childShapes[colA.m_shapeIndex+i].m_shapeType]
									   [world->m_childShapes[colB.m_shapeIndex+j].m_shapeType](world,colAIndex,colA.m_shapeIndex+i,colBIndex,colB.m_shapeIndex+j,&contactCache);
			}
		}
		return contactCache.numAddedPoints;
	}

	return 0;
}
	
void RealTimeBullet3CollisionSdk::collideWorld( plCollisionWorldHandle worldHandle,
                            plNearCallback filter, void* userData)
{
	RTB3CollisionWorld* world = (RTB3CollisionWorld*) worldHandle;
	if (filter)
	{
		for (int i=START_COLLIDABLE_INDEX;i<world->m_nextFreeCollidableIndex;i++)
		{
			for (int j=i+1;j<world->m_nextFreeCollidableIndex;j++)
			{
				plCollisionObjectHandle colA = (plCollisionObjectHandle)i;
				plCollisionObjectHandle colB = (plCollisionObjectHandle)j;
				filter((plCollisionSdkHandle)this,worldHandle,userData,colA,colB);
			}
		}
	}
}

plCollisionSdkHandle RealTimeBullet3CollisionSdk::createRealTimeBullet3CollisionSdkHandle()
{
	return (plCollisionSdkHandle) new RealTimeBullet3CollisionSdk();
}