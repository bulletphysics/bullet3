#include "b3CpuDynamicsWorld.h"

#include "b3BulletDynamicsCommon.h"

b3CpuDynamicsWorld::b3CpuDynamicsWorld()
	:b3DiscreteDynamicsWorld(
			new b3CollisionDispatcher(new b3DefaultCollisionConfiguration()),
			new b3DynamicBvhBroadphase(),new b3SequentialImpulseConstraintSolver(),
			new b3DefaultCollisionConfiguration()//todo: remove this!
			)
{
}
	
b3CpuDynamicsWorld::~b3CpuDynamicsWorld()
{
	
}
