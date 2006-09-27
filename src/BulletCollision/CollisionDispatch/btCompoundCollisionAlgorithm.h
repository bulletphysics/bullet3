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

#ifndef COMPOUND_COLLISION_ALGORITHM_H
#define COMPOUND_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"

#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include <vector>
#include "btCollisionCreateFunc.h"

/// btCompoundCollisionAlgorithm  supports collision between CompoundCollisionShapes and other collision shapes
/// Place holder, not fully implemented yet
class btCompoundCollisionAlgorithm  : public btCollisionAlgorithm
{
	btDispatcher*	m_dispatcher;
	btBroadphaseProxy	m_compoundProxy;
	btBroadphaseProxy	m_otherProxy;
	std::vector<btBroadphaseProxy> m_childProxies;
	std::vector<btCollisionAlgorithm*> m_childCollisionAlgorithms;

	btBroadphaseProxy m_compound;

	btBroadphaseProxy m_other;

	
public:

	btCompoundCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci,btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);

	virtual ~btCompoundCollisionAlgorithm();

	virtual void ProcessCollision (btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,const btDispatcherInfo& dispatchInfo);

	float	CalculateTimeOfImpact(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,const btDispatcherInfo& dispatchInfo);

	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
		{
			return new btCompoundCollisionAlgorithm(ci,proxy0,proxy1);
		}
	};

	struct SwappedCreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
		{
			return new btCompoundCollisionAlgorithm(ci,proxy1,proxy0);
		}
	};

};

#endif //COMPOUND_COLLISION_ALGORITHM_H
