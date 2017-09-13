#include "PosixSharedMemory.h"
#include "Bullet3Common/b3Logging.h"
#include "LinearMath/btScalar.h" //for btAssert
#include "LinearMath/btAlignedObjectArray.h"

//Windows implementation is in Win32SharedMemory.cpp
#ifndef _WIN32
#define TEST_SHARED_MEMORY
#endif//_WIN32

#include <stddef.h>

#ifdef TEST_SHARED_MEMORY

#include <sys/shm.h>
#include <sys/ipc.h>

#endif

struct btSharedMemorySegment
{
    int m_key;
    int m_sharedMemoryId;
    void* m_sharedMemoryPtr;
    bool m_createdSharedMemory;
    
    btSharedMemorySegment()
    :    m_sharedMemoryId(-1),
    m_sharedMemoryPtr(0),
    m_createdSharedMemory(true)
    {
    }
    
};

struct PosixSharedMemoryInteralData
{
    btAlignedObjectArray<btSharedMemorySegment> m_segments;
    
		
	PosixSharedMemoryInteralData()
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
    
    {
        btSharedMemorySegment* seg = 0;
        int i=0;
        
        for (i=0;i<m_internalData->m_segments.size();i++)
        {
            if (m_internalData->m_segments[i].m_key == key)
            {
                seg = &m_internalData->m_segments[i];
                break;
            }
        }
        if (seg)
        {
            b3Error("already created shared memory segment using same key");
            return seg->m_sharedMemoryPtr;
        }

    }
    
    int flags = (allowCreation ? IPC_CREAT : 0) | 0666;
    int id = shmget((key_t) key, (size_t) size,flags);
    if (id < 0)
    {
        //b3Warning("shmget error1");
    } else
    {
        btPointerCaster result;
        result.ptr = shmat(id,0,0);
        if (result.integer == -1)
        {
            b3Error("shmat returned -1");
        } else
        {
            btSharedMemorySegment seg;
            seg.m_key = key;
            seg.m_createdSharedMemory = allowCreation;
            seg.m_sharedMemoryId = id;
            seg.m_sharedMemoryPtr = result.ptr;
            m_internalData->m_segments.push_back(seg);
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
    
    btSharedMemorySegment* seg = 0;
    int i=0;
    
    for (i=0;i<m_internalData->m_segments.size();i++)
    {
        if (m_internalData->m_segments[i].m_key == key)
        {
            seg = &m_internalData->m_segments[i];
            break;
        }
    }

    if (0==seg)
    {
        b3Error("PosixSharedMemory::releaseSharedMemory: shared memory key not found");
        return;
    }
    
    if (seg->m_sharedMemoryId < 0)
    {
        b3Error("PosixSharedMemory::releaseSharedMemory: shared memory id is not set");
    } else
    {
		if (seg->m_createdSharedMemory)
		{
			int result = shmctl(seg->m_sharedMemoryId,IPC_RMID,0);
			if (result == -1)
			{
				b3Error("PosixSharedMemory::releaseSharedMemory: shmat returned -1");
			} else
			{
				b3Printf("PosixSharedMemory::releaseSharedMemory removed shared memory");
			}
			seg->m_createdSharedMemory = false;
			seg->m_sharedMemoryId = -1;
		}
		if (seg->m_sharedMemoryPtr)
		{
			shmdt(seg->m_sharedMemoryPtr);
			seg->m_sharedMemoryPtr  = 0;
			b3Printf("PosixSharedMemory::releaseSharedMemory detached shared memory\n");
		}
    }

    m_internalData->m_segments.removeAtIndex(i);
    
#endif
}
