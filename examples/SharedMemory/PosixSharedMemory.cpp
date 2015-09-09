#include "PosixSharedMemory.h"
#include "Bullet3Common/b3Logging.h"
#include "LinearMath/btScalar.h" //for btAssert

//haven't implemented shared memory on Windows yet, just Linux and Mac
#ifndef _WIN32
#define TEST_SHARED_MEMORY
#endif//_WIN32

#include <stddef.h>

#ifdef TEST_SHARED_MEMORY

#include <sys/shm.h>
#include <sys/ipc.h>

#endif

struct PosixSharedMemoryInteralData
{
	bool m_createdSharedMemory;
	int m_sharedMemoryId;
	void* m_sharedMemoryPtr;
	
	PosixSharedMemoryInteralData()
	:m_createdSharedMemory(false),
	m_sharedMemoryId(-1),
	m_sharedMemoryPtr(0)
	{
	}
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

void*   PosixSharedMemory::allocateSharedMemory(int key, int size,  bool allowCreation)
{
#ifdef TEST_SHARED_MEMORY
    int flags = (allowCreation ? IPC_CREAT : 0) | 0666;
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
			m_internalData->m_createdSharedMemory = allowCreation;
			m_internalData->m_sharedMemoryId = id;
			m_internalData->m_sharedMemoryPtr = result.ptr;
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
    if (m_internalData->m_sharedMemoryId < 0)
    {
        b3Error("PosixSharedMemory::releaseSharedMemory: shared memory id is not set");
    } else
    {
		if (m_internalData->m_createdSharedMemory)
		{
			int result = shmctl(m_internalData->m_sharedMemoryId,IPC_RMID,0);
			if (result == -1)
			{
				b3Error("PosixSharedMemory::releaseSharedMemory: shmat returned -1");
			} else
			{
				b3Printf("PosixSharedMemory::releaseSharedMemory removed shared memory");
			}
			m_internalData->m_createdSharedMemory = false;
			m_internalData->m_sharedMemoryId = -1;
		}
		if (m_internalData->m_sharedMemoryPtr)
		{
			shmdt(m_internalData->m_sharedMemoryPtr);
			m_internalData->m_sharedMemoryPtr  = 0;
			b3Printf("PosixSharedMemory::releaseSharedMemory detached shared memory\n");
		}
    }
#endif
}
