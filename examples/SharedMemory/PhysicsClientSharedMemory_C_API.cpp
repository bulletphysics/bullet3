#include "PhysicsClientSharedMemory_C_API.h"

#include "PhysicsClientSharedMemory.h"

B3_SHARED_API b3PhysicsClientHandle b3ConnectSharedMemory(int key)
{
	PhysicsClientSharedMemory* cl = new PhysicsClientSharedMemory();
	cl->setSharedMemoryKey(key);
	cl->connect();
	return (b3PhysicsClientHandle)cl;
}
