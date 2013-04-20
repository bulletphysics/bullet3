#include "btCpuDynamicsWorld.h"

#include "btBulletDynamicsCommon.h"

btCpuDynamicsWorld::btCpuDynamicsWorld()
	:btDiscreteDynamicsWorld(
			new btCollisionDispatcher(new btDefaultCollisionConfiguration()),
			new b3DynamicBvhBroadphase(),new btSequentialImpulseConstraintSolver(),
			new btDefaultCollisionConfiguration()//todo: remove this!
			)
{
}
	
btCpuDynamicsWorld::~btCpuDynamicsWorld()
{
	
}
