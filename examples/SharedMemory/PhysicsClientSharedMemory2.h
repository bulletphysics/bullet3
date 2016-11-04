#ifndef PHYSICS_CLIENT_SHARED_MEMORY2_H
#define PHYSICS_CLIENT_SHARED_MEMORY2_H

#include "PhysicsDirect.h"

class PhysicsClientSharedMemory2 : public PhysicsDirect
{
	class SharedMemoryCommandProcessor* m_proc;

public:
	PhysicsClientSharedMemory2(SharedMemoryCommandProcessor* proc);
	virtual ~PhysicsClientSharedMemory2();

	void setSharedMemoryInterface(class SharedMemoryInterface* sharedMem);

};

#endif //PHYSICS_CLIENT_SHARED_MEMORY2_H