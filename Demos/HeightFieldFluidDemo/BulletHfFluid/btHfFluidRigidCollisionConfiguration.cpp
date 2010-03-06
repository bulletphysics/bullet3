/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Experimental Buoyancy fluid demo written by John McCutchan
*/

#include "btHfFluidRigidCollisionConfiguration.h"
#include "btHfFluidRigidCollisionAlgorithm.h"
#include "btHfFluidBuoyantShapeCollisionAlgorithm.h"
#include "LinearMath/btPoolAllocator.h"

btHfFluidRigidCollisionConfiguration::btHfFluidRigidCollisionConfiguration(const btDefaultCollisionConstructionInfo& constructionInfo)
:btDefaultCollisionConfiguration(constructionInfo)
{
	void* mem;

	mem = btAlignedAlloc(sizeof(btHfFluidRigidCollisionAlgorithm::CreateFunc),16);
	m_hfFluidRigidConvexCreateFunc = new(mem) btHfFluidRigidCollisionAlgorithm::CreateFunc;

	mem = btAlignedAlloc(sizeof(btHfFluidRigidCollisionAlgorithm::CreateFunc),16);
	m_swappedHfFluidRigidConvexCreateFunc = new(mem) btHfFluidRigidCollisionAlgorithm::CreateFunc;
	m_swappedHfFluidRigidConvexCreateFunc->m_swapped = true;

	mem = btAlignedAlloc(sizeof(btHfFluidBuoyantShapeCollisionAlgorithm::CreateFunc),16);
	m_hfFluidBuoyantShapeCollisionCreateFunc = new(mem) btHfFluidBuoyantShapeCollisionAlgorithm::CreateFunc(m_simplexSolver, m_pdSolver);

	if (m_ownsCollisionAlgorithmPool && m_collisionAlgorithmPool)
	{
		int curElemSize = m_collisionAlgorithmPool->getElementSize();
		///calculate maximum element size, big enough to fit any collision algorithm in the memory pool

		int maxSize0 = sizeof(btHfFluidRigidCollisionAlgorithm);
		int maxSize1 = 0;
		int maxSize2 = 0;

		int	collisionAlgorithmMaxElementSize = btMax(maxSize0,maxSize1);
		collisionAlgorithmMaxElementSize = btMax(collisionAlgorithmMaxElementSize,maxSize2);
		
		if (collisionAlgorithmMaxElementSize > curElemSize)
		{
			m_collisionAlgorithmPool->~btPoolAllocator();
			btAlignedFree(m_collisionAlgorithmPool);
			void* mem = btAlignedAlloc(sizeof(btPoolAllocator),16);
			m_collisionAlgorithmPool = new(mem) btPoolAllocator(collisionAlgorithmMaxElementSize,constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize);
		}
	}
}

btHfFluidRigidCollisionConfiguration::~btHfFluidRigidCollisionConfiguration()
{
	m_hfFluidRigidConvexCreateFunc->~btCollisionAlgorithmCreateFunc();
	m_swappedHfFluidRigidConvexCreateFunc->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(m_hfFluidRigidConvexCreateFunc);
	btAlignedFree(m_swappedHfFluidRigidConvexCreateFunc);
}

btCollisionAlgorithmCreateFunc* btHfFluidRigidCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1)
{
	if ((proxyType0 == HFFLUID_SHAPE_PROXYTYPE) && (proxyType1 == HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE))
	{
		return	m_hfFluidRigidConvexCreateFunc;
	}

	if ((proxyType0 == HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE) && (proxyType1 == HFFLUID_SHAPE_PROXYTYPE))
	{
		return	m_swappedHfFluidRigidConvexCreateFunc;
	}

	if ((proxyType0 == HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE) && (proxyType1 == HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE))
	{
		return m_hfFluidBuoyantShapeCollisionCreateFunc;
	}

	///fallback to the regular rigid collision shape
	return btDefaultCollisionConfiguration::getCollisionAlgorithmCreateFunc(proxyType0,proxyType1);
}

