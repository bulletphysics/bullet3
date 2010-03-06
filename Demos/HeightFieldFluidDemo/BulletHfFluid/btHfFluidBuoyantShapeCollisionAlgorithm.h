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

#ifndef HF_FLUID_BUOYANT_SHAPE_COLLISION_ALGORITHM_H
#define HF_FLUID_BUOYANT_SHAPE_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h"

#include "LinearMath/btVector3.h"
class btHfFluid;

class btConvexConvexAlgorithm;
class btConvexPenetrationDepthSolver;
class btSimplexSolverInterface;

///experimental buyancy fluid demo
/// btHfFluidBuoyantShapeCollisionAlgorithm  provides collision detection between btHfFluidBuoyantConvexShape and btHfFluidBuoyantConvexShape
class btHfFluidBuoyantShapeCollisionAlgorithm : public btCollisionAlgorithm
{
	btCollisionObject*		m_collisionObject0;
	btCollisionObject*		m_collisionObject1;

	btConvexConvexAlgorithm m_convexConvexAlgorithm;
public:

	btHfFluidBuoyantShapeCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci,btCollisionObject* col0,btCollisionObject* col1, btSimplexSolverInterface* simplexSolver, btConvexPenetrationDepthSolver* pdSolver);

	virtual ~btHfFluidBuoyantShapeCollisionAlgorithm();

	virtual void processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray)
	{
		m_convexConvexAlgorithm.getAllContactManifolds (manifoldArray);
	}


	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		btConvexPenetrationDepthSolver*		m_pdSolver;
		btSimplexSolverInterface*			m_simplexSolver;
		
		CreateFunc(btSimplexSolverInterface*			simplexSolver, btConvexPenetrationDepthSolver* pdSolver)
		{
			m_simplexSolver = simplexSolver;
			m_pdSolver = pdSolver;
		}
		
		virtual ~CreateFunc() {}
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btHfFluidBuoyantShapeCollisionAlgorithm));
			if (!m_swapped)
			{
				return new(mem) btHfFluidBuoyantShapeCollisionAlgorithm(ci,body0,body1, m_simplexSolver, m_pdSolver);
			} else
			{
				return new(mem) btHfFluidBuoyantShapeCollisionAlgorithm(ci,body0,body1, m_simplexSolver, m_pdSolver);
			}
		}
	};
};

#endif //HF_FLUID_BUOYANT_SHAPE_COLLISION_ALGORITHM_H
