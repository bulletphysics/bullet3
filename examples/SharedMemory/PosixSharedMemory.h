#ifndef POSIX_SHARED_MEMORY_H
#define POSIX_SHARED_MEMORY_H

#include "SharedMemoryInterface.h"



class PosixSharedMemory : public SharedMemoryInterface
{

	struct PosixSharedMemoryInteralData* m_internalData;
    
public:
    PosixSharedMemory();
    virtual ~PosixSharedMemory();

    virtual void*   allocateSharedMemory(int key, int size);
    virtual void releaseSharedMemory(int key, int size);
};

#endif //
