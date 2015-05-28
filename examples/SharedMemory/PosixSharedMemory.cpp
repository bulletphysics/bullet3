#include "PosixSharedMemory.h"
#include "Bullet3Common/b3Logging.h"
#include "LinearMath/btScalar.h" //for btAssert
#ifdef __APPLE__
//#define TEST_SHARED_MEMORY
#endif

#include <stddef.h>

#ifdef TEST_SHARED_MEMORY

#include <sys/shm.h>
#include <sys/ipc.h>

#endif

struct PosixSharedMemoryInteralData
{

};
PosixSharedMemory::PosixSharedMemory()
{
    
    m_internalData = new PosixSharedMemoryInteralData;
}

PosixSharedMemory::~PosixSharedMemory()
{
    delete m_internalData;
}

struct btPointerCaster
{
    union
    {
        void* ptr;
        ptrdiff_t integer;
    };
};

void*   PosixSharedMemory::allocateSharedMemory(int key, int size)
{
#ifdef TEST_SHARED_MEMORY
    int flags = IPC_CREAT | 0666;
    int id = shmget((key_t) key, (size_t) size,flags);
    if (id < 0)
    {
        b3Error("shmget error");
    } else
    {
        btPointerCaster result;
        result.ptr = shmat(id,0,0);
        if (result.integer == -1)
        {
            b3Error("shmat returned -1");
        } else
        {
            return result.ptr;
        }
    }
#else
    //not implemented yet
    btAssert(0);
#endif
    return 0;
}
void PosixSharedMemory::releaseSharedMemory(int key, int size)
{
#ifdef TEST_SHARED_MEMORY
    int flags = 0666;
    int id = shmget((key_t) key, (size_t) size,flags);
    if (id < 0)
    {
        b3Error("shmget error");
    } else
    {
        int result = shmctl(id,IPC_RMID,0);
        if (result == -1)
        {
            b3Error("shmat returned -1");
        }
    }
#endif
}