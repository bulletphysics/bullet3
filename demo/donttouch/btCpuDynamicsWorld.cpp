#include "btCpuDynamicsWorld.h"

#include "btBulletDynamicsCommon.h"

btCpuDynamicsWorld::btCpuDynamicsWorld()
	:btDiscreteDynamicsWorld(
			new btCollisionDispatcher(new btDefaultCollisionConfiguration()),
			new btDbvtBroadphase(),new btSequentialImpulseConstraintSolver(),
			new btDefaultCollisionConfiguration()//todo: remove this!
			)
{
}
	
btCpuDynamicsWorld::~btCpuDynamicsWorld()
{
	
}
