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

#ifndef HF_FLUID_RIGID_COLLISION_ALGORITHM_H
#define HF_FLUID_RIGID_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h"

#include "LinearMath/btVector3.h"
class btHfFluid;

///experimental buyancy fluid demo
/// btHfFluidRigidCollisionAlgorithm  provides collision detection between btHfFluid and btRigidBody
class btHfFluidRigidCollisionAlgorithm : public btCollisionAlgorithm
{
	btPersistentManifold*	m_manifoldPtr;

	btHfFluid*				m_hfFluid;
	btCollisionObject*		m_rigidCollisionObject;

	///for rigid versus fluid (instead of fluid versus rigid), we use this swapped boolean
	bool	m_isSwapped;

	btConvexTriangleCallback m_convexTrianglecallback;

	void processGround (const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);
	void applyFluidFriction (btScalar mu, btScalar submerged_percentage);
	btScalar processFluid (const btDispatcherInfo& dispatchInfo, btScalar density, btScalar floatyness);
public:

	btHfFluidRigidCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci,btCollisionObject* col0,btCollisionObject* col1, bool isSwapped);

	virtual ~btHfFluidRigidCollisionAlgorithm();

	virtual void processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray)
	{
		manifoldArray.push_back (m_manifoldPtr);
	}


	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btHfFluidRigidCollisionAlgorithm));
			if (!m_swapped)
			{
				return new(mem) btHfFluidRigidCollisionAlgorithm(ci,body0,body1,false);
			} else
			{
				return new(mem) btHfFluidRigidCollisionAlgorithm(ci,body0,body1,true);
			}
		}
	};
};

#endif //HF_FLUID_RIGID_COLLISION_ALGORITHM_H
