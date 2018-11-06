#ifndef IN_PROCESS_MEMORY_H
#define IN_PROCESS_MEMORY_H

#include "SharedMemoryInterface.h"

class InProcessMemory : public SharedMemoryInterface
{
	struct InProcessMemoryInternalData* m_data;

public:
	InProcessMemory();
	virtual ~InProcessMemory();

	virtual void* allocateSharedMemory(int key, int size, bool allowCreation);
	virtual void releaseSharedMemory(int key, int size);
};

#endif
