
//tinyRendererPlugin implements the TinyRenderer as a plugin
//it is statically linked when using preprocessor #define STATIC_LINK_VR_PLUGIN
//otherwise you can dynamically load it using pybullet.loadPlugin

#include "collisionFilterPlugin.h"
#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>
#include "Bullet3Common/b3HashMap.h"

#include "../b3PluginCollisionInterface.h"

struct b3CustomCollisionFilter
{
	int m_objectUniqueIdA;
	int m_linkIndexA;
	int m_objectUniqueIdB;
	int m_linkIndexB;
	bool m_enableCollision;

	B3_FORCE_INLINE unsigned int getHash() const
	{
		int obA = (m_objectUniqueIdA & 0xff);
		int obB = ((m_objectUniqueIdB & 0xf) << 8);
		int linkA = ((m_linkIndexA & 0xff) << 16);
		int linkB = ((m_linkIndexB & 0xff) << 24);
		int key = obA + obB + linkA + linkB;
		// Thomas Wang's hash
		key += ~(key << 15);
		key ^= (key >> 10);
		key += (key << 3);
		key ^= (key >> 6);
		key += ~(key << 11);
		key ^= (key >> 16);
		return key;
	}
	bool equals(const b3CustomCollisionFilter& other) const
	{
		return m_objectUniqueIdA == other.m_objectUniqueIdA &&
			   m_objectUniqueIdB == other.m_objectUniqueIdB &&
			   m_linkIndexA == other.m_linkIndexA &&
			   m_linkIndexB == other.m_linkIndexB;
	}
};

struct DefaultPluginCollisionInterface : public b3PluginCollisionInterface
{
	b3HashMap<b3CustomCollisionFilter, b3CustomCollisionFilter> m_customCollisionFilters;

	virtual void setBroadphaseCollisionFilter(
		int objectUniqueIdA, int objectUniqueIdB,
		int linkIndexA, int linkIndexB,
		bool enableCollision)
	{
		b3CustomCollisionFilter keyValue;
		keyValue.m_objectUniqueIdA = objectUniqueIdA;
		keyValue.m_linkIndexA = linkIndexA;
		keyValue.m_objectUniqueIdB = objectUniqueIdB;
		keyValue.m_linkIndexB = linkIndexB;
		keyValue.m_enableCollision = enableCollision;

		if (objectUniqueIdA > objectUniqueIdB)
		{
			b3Swap(keyValue.m_objectUniqueIdA, keyValue.m_objectUniqueIdB);
			b3Swap(keyValue.m_linkIndexA, keyValue.m_linkIndexB);
		}
		if (objectUniqueIdA == objectUniqueIdB)
		{
			if (keyValue.m_linkIndexA > keyValue.m_linkIndexB)
			{
				b3Swap(keyValue.m_linkIndexA, keyValue.m_linkIndexB);
			}
		}

		m_customCollisionFilters.insert(keyValue, keyValue);
	}

	virtual void removeBroadphaseCollisionFilter(
		int objectUniqueIdA, int objectUniqueIdB,
		int linkIndexA, int linkIndexB)
	{
		b3CustomCollisionFilter keyValue;
		keyValue.m_objectUniqueIdA = objectUniqueIdA;
		keyValue.m_linkIndexA = linkIndexA;
		keyValue.m_objectUniqueIdB = objectUniqueIdB;
		keyValue.m_linkIndexB = linkIndexB;

		if (objectUniqueIdA > objectUniqueIdB)
		{
			b3Swap(keyValue.m_objectUniqueIdA, keyValue.m_objectUniqueIdB);
			b3Swap(keyValue.m_linkIndexA, keyValue.m_linkIndexB);
		}
		if (objectUniqueIdA == objectUniqueIdB)
		{
			if (keyValue.m_linkIndexA > keyValue.m_linkIndexB)
			{
				b3Swap(keyValue.m_linkIndexA, keyValue.m_linkIndexB);
			}
		}

		m_customCollisionFilters.remove(keyValue);
	}

	virtual int getNumRules() const
	{
		return m_customCollisionFilters.size();
	}

	virtual void resetAll()
	{
		m_customCollisionFilters.clear();
	}

	virtual int needsBroadphaseCollision(int objectUniqueIdA, int linkIndexA,
										 int collisionFilterGroupA, int collisionFilterMaskA,
										 int objectUniqueIdB, int linkIndexB,
										 int collisionFilterGroupB, int collisionFilterMaskB,
										 int filterMode)
	{
		//check and apply any custom rules for those objects/links
		b3CustomCollisionFilter keyValue;
		keyValue.m_objectUniqueIdA = objectUniqueIdA;
		keyValue.m_linkIndexA = linkIndexA;
		keyValue.m_objectUniqueIdB = objectUniqueIdB;
		keyValue.m_linkIndexB = linkIndexB;

		if (objectUniqueIdA > objectUniqueIdB)
		{
			b3Swap(keyValue.m_objectUniqueIdA, keyValue.m_objectUniqueIdB);
			b3Swap(keyValue.m_linkIndexA, keyValue.m_linkIndexB);
		}
		if (objectUniqueIdA == objectUniqueIdB)
		{
			if (keyValue.m_linkIndexA > keyValue.m_linkIndexB)
			{
				b3Swap(keyValue.m_linkIndexA, keyValue.m_linkIndexB);
			}
		}

		b3CustomCollisionFilter* filter = m_customCollisionFilters.find(keyValue);
		if (filter)
		{
			return filter->m_enableCollision;
		}

		//otherwise use the default fallback

		if (filterMode == B3_FILTER_GROUPAMASKB_AND_GROUPBMASKA)
		{
			bool collides = (collisionFilterGroupA & collisionFilterMaskB) != 0;
			collides = collides && (collisionFilterGroupB & collisionFilterMaskA);
			return collides;
		}

		if (filterMode == B3_FILTER_GROUPAMASKB_OR_GROUPBMASKA)
		{
			bool collides = (collisionFilterGroupA & collisionFilterMaskB) != 0;
			collides = collides || (collisionFilterGroupB & collisionFilterMaskA);
			return collides;
		}
		return false;
	}
};

struct CollisionFilterMyClass
{
	int m_testData;

	DefaultPluginCollisionInterface m_collisionFilter;

	CollisionFilterMyClass()
		: m_testData(42)
	{
	}
	virtual ~CollisionFilterMyClass()
	{
	}
};

B3_SHARED_API int initPlugin_collisionFilterPlugin(struct b3PluginContext* context)
{
	CollisionFilterMyClass* obj = new CollisionFilterMyClass();
	context->m_userPointer = obj;
	return SHARED_MEMORY_MAGIC_NUMBER;
}

B3_SHARED_API struct b3PluginCollisionInterface* getCollisionInterface_collisionFilterPlugin(struct b3PluginContext* context)
{
	CollisionFilterMyClass* obj = (CollisionFilterMyClass*)context->m_userPointer;
	return &obj->m_collisionFilter;
}

B3_SHARED_API int executePluginCommand_collisionFilterPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	return 0;
}

B3_SHARED_API void exitPlugin_collisionFilterPlugin(struct b3PluginContext* context)
{
	CollisionFilterMyClass* obj = (CollisionFilterMyClass*)context->m_userPointer;
	delete obj;
	context->m_userPointer = 0;
}
