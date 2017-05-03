
#ifndef B3_RESIZABLE_POOL_H
#define B3_RESIZABLE_POOL_H

#include "Bullet3Common/b3AlignedObjectArray.h"

template <typename U>
struct b3PoolBodyHandle : public U
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	int m_nextFreeHandle;
	void SetNextFree(int next)
	{
		m_nextFreeHandle = next;
	}
	int	GetNextFree() const
	{
		return m_nextFreeHandle;
	}
};

template <typename T> 
class b3ResizablePool
{

protected:
	b3AlignedObjectArray<T>	m_bodyHandles;
	int m_numUsedHandles;						// number of active handles
	int	m_firstFreeHandle;		// free handles list

public:
	
	b3ResizablePool()
	{
	}
	
	virtual ~b3ResizablePool()
	{
	}
///handle management

	int getNumHandles() const
	{
		return m_bodyHandles.size();
	}

	T* getHandle(int handle)
	{
		btAssert(handle>=0);
		btAssert(handle<m_bodyHandles.size());
		if ((handle<0) || (handle>=m_bodyHandles.size()))
		{
			return 0;
		}
		return &m_bodyHandles[handle];
	}
	const T* getHandle(int handle) const
	{
		return &m_bodyHandles[handle];
	}

	void increaseHandleCapacity(int extraCapacity)
	{
		int curCapacity = m_bodyHandles.size();
		btAssert(curCapacity == m_numUsedHandles);
		int newCapacity = curCapacity + extraCapacity;
		m_bodyHandles.resize(newCapacity);

		{
			for (int i = curCapacity; i < newCapacity; i++)
				m_bodyHandles[i].SetNextFree(i + 1);


			m_bodyHandles[newCapacity - 1].SetNextFree(-1);
		}
		m_firstFreeHandle = curCapacity;
	}
	void initHandles()
	{
		m_numUsedHandles = 0;
		m_firstFreeHandle = -1;

		increaseHandleCapacity(1);
	}

	void exitHandles()
	{
		m_bodyHandles.resize(0);
		m_firstFreeHandle = -1;
		m_numUsedHandles = 0;
	}

	int allocHandle()
	{
		btAssert(m_firstFreeHandle>=0);

		int handle = m_firstFreeHandle;
		m_firstFreeHandle = getHandle(handle)->GetNextFree();
		m_numUsedHandles++;

		if (m_firstFreeHandle<0)
		{
			//int curCapacity = m_bodyHandles.size();
			int additionalCapacity= m_bodyHandles.size();
			increaseHandleCapacity(additionalCapacity);


			getHandle(handle)->SetNextFree(m_firstFreeHandle);
		}


		return handle;
	}


	void freeHandle(int handle)
	{
		btAssert(handle >= 0);

		getHandle(handle)->SetNextFree(m_firstFreeHandle);
		m_firstFreeHandle = handle;

		m_numUsedHandles--;
	}
};
	///end handle management
	
	#endif //B3_RESIZABLE_POOL_H
	