#include "InProcessMemory.h"

#include "LinearMath/btHashMap.h"

struct InProcessMemoryInternalData
{
	btHashMap<btHashInt, void*> m_memoryPointers;
};

InProcessMemory::InProcessMemory()
{
	m_data = new InProcessMemoryInternalData;
}

InProcessMemory::~InProcessMemory()
{
	for (int i = 0; i < m_data->m_memoryPointers.size(); i++)
	{
		void** ptrptr = m_data->m_memoryPointers.getAtIndex(i);
		if (ptrptr)
		{
			void* ptr = *ptrptr;
			free(ptr);
		}
	}
	delete m_data;
}

void* InProcessMemory::allocateSharedMemory(int key, int size, bool allowCreation)
{
	void** ptrptr = m_data->m_memoryPointers[key];
	if (ptrptr)
	{
		return *ptrptr;
	}

	void* ptr = malloc(size);
	m_data->m_memoryPointers.insert(key, ptr);
	return ptr;
}

void InProcessMemory::releaseSharedMemory(int /*key*/, int /*size*/)
{
	//we don't release the memory here, but in the destructor instead,
	//so multiple users could 'share' the memory given some key
}