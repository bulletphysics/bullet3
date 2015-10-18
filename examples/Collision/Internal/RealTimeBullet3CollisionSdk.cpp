#include "RealTimeBullet3CollisionSdk.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3Collidable.h"

struct RTB3CollisionWorld
{
	b3AlignedObjectArray<b3Collidable> m_collidables;
	b3AlignedObjectArray<b3GpuChildShape> m_childShapes;
	b3AlignedObjectArray<b3CompoundOverlappingPair> m_compoundOverlappingPairs;
	int m_nextFreeShapeIndex;
	int m_nextFreeCollidableIndex;

	RTB3CollisionWorld()
	:m_nextFreeCollidableIndex(0),
	m_nextFreeShapeIndex(0)
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
	int index = 10;
	return (plCollisionShapeHandle) index;
}
	
void RealTimeBullet3CollisionSdk::deleteShape(plCollisionWorldHandle worldHandle, plCollisionShapeHandle shape)
{
	//deleting shapes would involve a garbage collection phase, and mess up all user indices
	//this would be solved by one more in-direction, at some performance penalty for certain operations
	//for now, we don't delete and eventually run out-of-shapes
}
	
void RealTimeBullet3CollisionSdk::addCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object)
{
}

void RealTimeBullet3CollisionSdk::removeCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object)
{
}
	
plCollisionObjectHandle RealTimeBullet3CollisionSdk::createCollisionObject(  void* userPointer, int userIndex,  plCollisionShapeHandle cshape ,
                                                        plVector3 startPosition,plQuaternion startOrientation )
{
	return 0;
}
  
void RealTimeBullet3CollisionSdk::deleteCollisionObject(plCollisionObjectHandle body)
{
}
	
void RealTimeBullet3CollisionSdk::setCollisionObjectTransform(plCollisionObjectHandle body,
											plVector3 position,plQuaternion orientation )
{
}
	
int RealTimeBullet3CollisionSdk::collide(plCollisionWorldHandle world,plCollisionObjectHandle colA, plCollisionObjectHandle colB,
                    lwContactPoint* pointsOut, int pointCapacity)
{
	return 0;
}
	
void RealTimeBullet3CollisionSdk::collideWorld( plCollisionWorldHandle world,
                            plNearCallback filter, void* userData)
{
}

plCollisionSdkHandle RealTimeBullet3CollisionSdk::createRealTimeBullet3CollisionSdkHandle()
{
	return (plCollisionSdkHandle) new RealTimeBullet3CollisionSdk();
}