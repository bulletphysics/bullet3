#include "PhysicsClientSharedMemory2_C_API.h"

#include "PhysicsDirect.h"
#include "SharedMemoryCommandProcessor.h"

b3PhysicsClientHandle b3ConnectSharedMemory2(int key)
{

	SharedMemoryCommandProcessor* cmdProc = new SharedMemoryCommandProcessor();
	cmdProc->setSharedMemoryKey(key);
	PhysicsDirect* cl = new PhysicsDirect(cmdProc,true);

	cl->setSharedMemoryKey(key);

	cl->connect();

	return (b3PhysicsClientHandle)cl;
}

