#ifndef __SPU_RAYCAST_TASK_H
#define __SPU_RAYCAST_TASK_H

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "LinearMath/btVector3.h"
#include "../PlatformDefinitions.h"

ATTRIBUTE_ALIGNED16(struct) RaycastGatheredObjectData
{
	ppu_address_t m_collisionShape;
	void* m_spuCollisionShape;
	btVector3	m_primitiveDimensions;
	int		m_shapeType;
	float	m_collisionMargin;
	btTransform	m_worldTransform;
};


ATTRIBUTE_ALIGNED16(struct) SpuRaycastTaskWorkUnitOut
{
	btVector3 hitNormal; /* out */
	btScalar hitFraction; /* out */
	btCollisionWorld::LocalShapeInfo shapeInfo; /* out */
};

/* Perform a raycast on collision object */
ATTRIBUTE_ALIGNED16(struct) SpuRaycastTaskWorkUnit
{
	btVector3 rayFrom; /* in */
	btVector3 rayTo; /* in */
	SpuRaycastTaskWorkUnitOut* output; /* out */
};

#define SPU_RAYCAST_WORK_UNITS_PER_TASK 16

ATTRIBUTE_ALIGNED128(struct) SpuRaycastTaskDesc
{
	SpuRaycastTaskWorkUnit workUnits[SPU_RAYCAST_WORK_UNITS_PER_TASK];
	unsigned int numWorkUnits;
	void* spuCollisionObjectsWrappers;
	unsigned int numSpuCollisionObjectWrappers;
	int taskId;
};


void	processRaycastTask (void* userPtr, void* lsMemory);
void*	createRaycastLocalStoreMemory ();

#endif
