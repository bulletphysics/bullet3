#ifndef SHARED_MEMORY_USER_DATA_H
#define SHARED_MEMORY_USER_DATA_H

#include <string>
#include "LinearMath/btAlignedObjectArray.h"
#include "SharedMemoryPublic.h"

struct SharedMemoryUserData 
{
	std::string m_key;
	int m_type;

	btAlignedObjectArray<char> m_bytes;
	
	SharedMemoryUserData()
		:m_type(-1)
	{
	}

	// Takes ownership of the passed key and value arguments.
	SharedMemoryUserData(const char* key)
	:m_key(key)
	{
	}

	// Takes ownership of the data pointed to by newValue.
	void replaceValue(const char* bytes, int len, int type)
	{
		m_type = type;
		m_bytes.resize(len);
		for (int i=0;i<len;i++)
		{
			m_bytes[i] = bytes[i];
		}
	}

	virtual ~SharedMemoryUserData()
	{
	}

	void clear()
	{
		m_bytes.clear();
		m_type = -1;
	}
};

#endif //SHARED_MEMORY_USER_DATA_H
