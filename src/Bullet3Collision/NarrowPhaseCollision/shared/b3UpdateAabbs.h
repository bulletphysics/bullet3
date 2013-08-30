#ifndef B3_UPDATE_AABBS_H
#define B3_UPDATE_AABBS_H



#include "Bullet3Collision/BroadPhaseCollision/shared/b3Aabb.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3CollidableData.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"



void b3ComputeWorldAabb(  int bodyId, b3RigidBodyData* body, b3CollidableData* collidables, b3Aabb* localShapeAABB, b3Aabb* worldAabbs)
{
	b3Float4 position = body->m_pos;
	b3Quat	orientation = body->m_quat;
	
	int collidableIndex = body->m_collidableIdx;
	int shapeIndex = collidables[collidableIndex].m_shapeIndex;
		
	if (shapeIndex>=0)
	{
				
		b3Aabb localAabb = localShapeAABB[shapeIndex];
		b3Aabb worldAabb;
		
		b3TransformAabb2(localAabb.m_minVec,localAabb.m_maxVec,margin,position,orientation,&worldAabb.m_minVec,&worldAabb.m_maxVec);		
		worldAabbs[bodyId] = worldAabb;
	}
}

#endif //B3_UPDATE_AABBS_H
