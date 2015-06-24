#ifndef WIN32_SHARED_MEMORY_H
#define WIN32_SHARED_MEMORY_H

#include "SharedMemoryInterface.h"



class Win32SharedMemory : public SharedMemoryInterface
{

	struct Win32SharedMemoryInteralData* m_internalData;
    
public:
    Win32SharedMemory();
    virtual ~Win32SharedMemory();

    virtual void*   allocateSharedMemory(int key, int size);
    virtual void releaseSharedMemory(int key, int size);
	virtual bool isServer() const = 0;
};

class Win32SharedMemoryServer : public Win32SharedMemory
{
public:
	Win32SharedMemoryServer();
	virtual ~Win32SharedMemoryServer();
	virtual bool isServer() const
	{
		return true;
	}
};

class Win32SharedMemoryClient : public Win32SharedMemory
{
public:
	Win32SharedMemoryClient();
	virtual ~Win32SharedMemoryClient();
	virtual bool isServer() const
	{
		return false;
	}
};


#endif //WIN32_SHARED_MEMORY_H
