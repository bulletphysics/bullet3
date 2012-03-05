/*
Physics Effects Copyright(C) 2011 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/
 
///btLowLevelBroadphase implementation
#ifndef BT_LOW_LEVEL_BROADPHASE_H
#define BT_LOW_LEVEL_BROADPHASE_H

#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"

#include "physics_effects/base_level/broadphase/pfx_broadphase_pair.h"
#include "LinearMath/btHashMap.h"

struct btLowLevelBroadphase;
struct btLowLevelData;

namespace sce
{
	namespace PhysicsEffects
	{
		struct PfxSortData32;
	};
};

struct	btMyClientData
{
	btDispatcher* m_dispatcher;
	btLowLevelBroadphase*	m_bp;
};

struct btLowLevelBroadphaseProxy : public btBroadphaseProxy
{
	int			m_nextFree;
	
//	int			m_handleId;

	
	btLowLevelBroadphaseProxy() {};

	btLowLevelBroadphaseProxy(const btVector3& minpt,const btVector3& maxpt,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask,void* multiSapProxy)
	:btBroadphaseProxy(minpt,maxpt,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy)
	{
		(void)shapeType;
	}
	
	
	SIMD_FORCE_INLINE void SetNextFree(int next) {m_nextFree = next;}
	SIMD_FORCE_INLINE int GetNextFree() const {return m_nextFree;}

	


};

///btLowLevelBroadphase is a binding between Open Physics low-level broadphase and Bullet, through the btBroadphaseInterface

struct	btLowLevelBroadphase : btBroadphaseInterface
{
	
		int		m_numHandles;						// number of active handles
	int		m_maxHandles;						// max number of handles
	int		m_LastHandleIndex;							
	
	btLowLevelBroadphaseProxy* m_pHandles;						// handles pool

	void* m_pHandlesRawPtr;
	int		m_firstFreeHandle;		// free handles list
	
	btOverlappingPairCache*	m_paircache;				// Pair cache
	bool					m_releasepaircache;			// Release pair cache on delete
	int						m_guidGenerator;
	btVector3				m_broadphaseAabbMin;
	btVector3				m_broadphaseAabbMax;

	btMyClientData			m_clientData;

	btLowLevelData*	m_lowLevelData;
	btHashMap<btHashInt,void*> m_uid2ptr;


	int allocHandle()
	{
		btAssert(m_numHandles < m_maxHandles);
		int freeHandle = m_firstFreeHandle;
		m_firstFreeHandle = m_pHandles[freeHandle].GetNextFree();
		m_numHandles++;
		if(freeHandle > m_LastHandleIndex)
		{
			m_LastHandleIndex = freeHandle;
		}
		return freeHandle;
	}

	void freeHandle(btLowLevelBroadphaseProxy* proxy)
	{
		int handle = int(proxy-m_pHandles);
		btAssert(handle >= 0 && handle < m_maxHandles);
		if(handle == m_LastHandleIndex)
		{
			m_LastHandleIndex--;
		}
		proxy->SetNextFree(m_firstFreeHandle);
		m_firstFreeHandle = handle;

		proxy->m_clientObject = 0;

		m_numHandles--;
	}

	inline btLowLevelBroadphaseProxy*	getLowLevelProxyFromProxy(btBroadphaseProxy* proxy)
	{
		btLowLevelBroadphaseProxy* proxy0 = static_cast<btLowLevelBroadphaseProxy*>(proxy);
		return proxy0;
	}

	inline const btLowLevelBroadphaseProxy*	getLowLevelProxyFromProxy(btBroadphaseProxy* proxy) const
	{
		const btLowLevelBroadphaseProxy* proxy0 = static_cast<const btLowLevelBroadphaseProxy*>(proxy);
		return proxy0;
	}



	inline bool testAabbOverlap(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
	{
		btLowLevelBroadphaseProxy* p0 = getLowLevelProxyFromProxy(proxy0);
		btLowLevelBroadphaseProxy* p1 = getLowLevelProxyFromProxy(proxy1);
		return aabbOverlap(p0,p1);
	}

	static bool	aabbOverlap(btLowLevelBroadphaseProxy* proxy0,btLowLevelBroadphaseProxy* proxy1);

	void broadphase(sce::PhysicsEffects::PfxSortData32* proxies, int numRigidBodies, int axis, btDispatcher* dispatcher);

	sce::PhysicsEffects::PfxBroadphasePair*		getCurrentPairs();
	const sce::PhysicsEffects::PfxBroadphasePair*		getCurrentPairs() const;
	int	getNumCurrentPairs() const;

	

	/* Methods		*/ 
	btLowLevelBroadphase(btLowLevelData* lowLevelData, btOverlappingPairCache* paircache, int maxProxies = 16384);//,class PfxAllocator* allocator,class PfxStackAllocator* stackPool,void* spursInstance);
	virtual ~btLowLevelBroadphase();

	/* btBroadphaseInterface Implementation	*/ 
	btBroadphaseProxy*				createProxy(const btVector3& aabbMin,const btVector3& aabbMax,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask,btDispatcher* dispatcher,void* multiSapProxy);
	void							destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);
	void							setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax,btDispatcher* dispatcher);
	virtual void	rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback, const btVector3& aabbMin=btVector3(0,0,0), const btVector3& aabbMax = btVector3(0,0,0))
	{
	}
	
	virtual void	aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback)
	{
	}


	virtual void	getAabb(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const;
	void							calculateOverlappingPairs(btDispatcher* dispatcher);
	btOverlappingPairCache*			getOverlappingPairCache();
	const btOverlappingPairCache*	getOverlappingPairCache() const;
	void							getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const;
	virtual void	printStats();

	virtual void	setNumTasks(int numTasks);
	
};

#endif
