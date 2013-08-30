
#include "b3CpuCollisionWorld.h"
#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.h"


b3CpuCollisionWorld::b3CpuCollisionWorld(b3DynamicBvhBroadphase* bp, b3CpuNarrowPhase* np)
	:m_bp(bp),
	m_np(np)
{

}

b3CpuCollisionWorld::~b3CpuCollisionWorld()
{

}

void	b3CpuCollisionWorld::addCollidable(int bodyIndex, int collidableIndex,const b3Vector3& position, const b3Quaternion& orientation)
{
	b3Vector3 aabbMinWorld, aabbMaxWorld;

	if (collidableIndex>=0)
	{
		b3Aabb localAabb = m_np->getLocalSpaceAabb(collidableIndex);
		b3Vector3 localAabbMin=b3MakeVector3(localAabb.m_min[0],localAabb.m_min[1],localAabb.m_min[2]);
		b3Vector3 localAabbMax=b3MakeVector3(localAabb.m_max[0],localAabb.m_max[1],localAabb.m_max[2]);
		
		b3Scalar margin = 0.01f;
		b3Transform t;
		t.setIdentity();
		t.setOrigin(b3MakeVector3(position[0],position[1],position[2]));
		t.setRotation(b3Quaternion(orientation[0],orientation[1],orientation[2],orientation[3]));
		b3TransformAabb(localAabbMin,localAabbMax, margin,t,aabbMinWorld,aabbMaxWorld);

		m_bp->createProxy(aabbMinWorld,aabbMaxWorld,bodyIndex,0,1,1);
		
		b3Vector3 aabbMin,aabbMax;
		m_bp->getAabb(bodyIndex,aabbMin,aabbMax);
		

	} else
	{
		b3Error("registerPhysicsInstance using invalid collidableIndex\n");
	}
}