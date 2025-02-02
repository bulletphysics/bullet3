#include "PhysicsLoopBackC_API.h"

#include "PhysicsLoopBack.h"

//think more about naming. The b3ConnectPhysicsLoopback
b3PhysicsClientHandle b3ConnectPhysicsLoopback(int key)
{
	PhysicsLoopBack* loopBack = new PhysicsLoopBack();
	loopBack->setSharedMemoryKey(key);
	bool connected;
	connected = loopBack->connect();
	(void)connected;
	return (b3PhysicsClientHandle)loopBack;
}
