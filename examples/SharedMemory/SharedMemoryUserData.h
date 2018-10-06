#ifndef SHARED_MEMORY_USER_DATA_H
#define SHARED_MEMORY_USER_DATA_H

#include <string>
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"
#include "SharedMemoryPublic.h"

struct SharedMemoryUserData
{
	std::string m_key;
	int m_type;

	int m_bodyUniqueId;
	int m_linkIndex;
	int m_visualShapeIndex;

	btAlignedObjectArray<char> m_bytes;

	SharedMemoryUserData()
		: m_type(-1), m_bodyUniqueId(-1), m_linkIndex(-1), m_visualShapeIndex(-1)
	{
	}

	SharedMemoryUserData(const char* key, int bodyUniqueId, int linkIndex, int visualShapeIndex)
		: m_key(key), m_type(-1), m_bodyUniqueId(bodyUniqueId), m_linkIndex(linkIndex), m_visualShapeIndex(visualShapeIndex)
	{
	}

	void replaceValue(const char* bytes, int len, int type)
	{
		m_type = type;
		m_bytes.resize(len);
		for (int i = 0; i < len; i++)
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

struct SharedMemoryUserDataHashKey
{
	unsigned int m_hash;

	btHashString m_key;
	btHashInt m_bodyUniqueId;
	btHashInt m_linkIndex;
	btHashInt m_visualShapeIndex;

	SIMD_FORCE_INLINE unsigned int getHash() const
	{
		return m_hash;
	}

	SharedMemoryUserDataHashKey() : m_hash(0) {}

	SharedMemoryUserDataHashKey(const struct SharedMemoryUserData* userData)
		: m_key(userData->m_key.c_str()),
		  m_bodyUniqueId(userData->m_bodyUniqueId),
		  m_linkIndex(userData->m_linkIndex),
		  m_visualShapeIndex(userData->m_visualShapeIndex)
	{
		calculateHash();
	}

	SharedMemoryUserDataHashKey(const char* key, int bodyUniqueId, int linkIndex, int visualShapeIndex)
		: m_key(key), m_bodyUniqueId(bodyUniqueId), m_linkIndex(linkIndex), m_visualShapeIndex(visualShapeIndex)
	{
		calculateHash();
	}

	void calculateHash()
	{
		m_hash = m_key.getHash() ^ m_bodyUniqueId.getHash() ^ m_linkIndex.getHash() ^ m_visualShapeIndex.getHash();
	}

	bool equals(const SharedMemoryUserDataHashKey& other) const
	{
		return m_bodyUniqueId.equals(other.m_bodyUniqueId) &&
			   m_linkIndex.equals(other.m_linkIndex) &&
			   m_visualShapeIndex.equals(other.m_visualShapeIndex) &&
			   m_key.equals(other.m_key);
	}
};

#endif  //SHARED_MEMORY_USER_DATA_H
