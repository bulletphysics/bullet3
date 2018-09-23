
#include "PhysicsClientSharedMemory2.h"
#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"
#include "Bullet3Common/b3Logging.h"
#include "Bullet3Common/b3Scalar.h"

#include "SharedMemoryCommandProcessor.h"

PhysicsClientSharedMemory2::PhysicsClientSharedMemory2(SharedMemoryCommandProcessor* proc)
	: PhysicsDirect(proc, false)
{
	m_proc = proc;
}
PhysicsClientSharedMemory2::~PhysicsClientSharedMemory2()
{
}

void PhysicsClientSharedMemory2::setSharedMemoryInterface(class SharedMemoryInterface* sharedMem)
{
	if (m_proc)
	{
		m_proc->setSharedMemoryInterface(sharedMem);
	}
}
