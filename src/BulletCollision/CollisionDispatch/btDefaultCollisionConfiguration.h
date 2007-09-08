/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_DEFAULT_COLLISION_CONFIGURATION
#define BT_DEFAULT_COLLISION_CONFIGURATION

#include "btCollisionConfiguration.h"

///btCollisionConfiguration allows to configure Bullet collision detection
///stack allocator size, default collision algorithms and persistent manifold pool size
///todo: describe the meaning
class	btDefaultCollisionConfiguration : public btCollisionConfiguration
{

	int	m_persistentManifoldPoolSize;
	
	int	m_stackAllocatorSize;

	int m_collisionAlgorithmPoolSize;

	int m_collisionAlgorithmMaxElementSize;

	//default CreationFunctions, filling the m_doubleDispatch table
	btCollisionAlgorithmCreateFunc*	m_convexConvexCreateFunc;
	btCollisionAlgorithmCreateFunc*	m_convexConcaveCreateFunc;
	btCollisionAlgorithmCreateFunc*	m_swappedConvexConcaveCreateFunc;
	btCollisionAlgorithmCreateFunc*	m_compoundCreateFunc;
	btCollisionAlgorithmCreateFunc*	m_swappedCompoundCreateFunc;
	btCollisionAlgorithmCreateFunc* m_emptyCreateFunc;
	btCollisionAlgorithmCreateFunc* m_sphereSphereCF;
	btCollisionAlgorithmCreateFunc* m_sphereBoxCF;
	btCollisionAlgorithmCreateFunc* m_boxSphereCF;

public:

	btDefaultCollisionConfiguration();

	virtual ~btDefaultCollisionConfiguration();

	///pool size for the persistent contact manifold
	virtual int	getPersistentManifoldPoolSize();

	virtual int	getStackAllocatorSize();

	virtual int	getCollisionAlgorithmPoolSize();

	virtual int	getCollisionAlgorithmMaxElementSize();

	btCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1);

	void	setStackAllocatorSize(int size)
	{
		m_stackAllocatorSize = size;
	}

	void	setPersistentManifoldPoolSize(int size)
	{
		m_persistentManifoldPoolSize = size;
	}

	void	setCollisionAlgorithmPoolSize(int size)
	{
		m_collisionAlgorithmPoolSize = size;
	}
};

#endif //BT_DEFAULT_COLLISION_CONFIGURATION