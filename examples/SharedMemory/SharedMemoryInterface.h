#ifndef SHARED_MEMORY_INTERFACE_H
#define SHARED_MEMORY_INTERFACE_H

class SharedMemoryInterface
{
	public:
	virtual ~SharedMemoryInterface()
	{
	}
	
	virtual void*	allocateSharedMemory(int key, int size, bool allowCreation) =0;
	virtual void releaseSharedMemory(int key, int size) =0;
};

#endif

