/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

#ifndef CUSTOM_CONVEX_CONVEX_PAIR_COLLISION_H
#define CUSTOM_CONVEX_CONVEX_PAIR_COLLISION_H


#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"

class CustomConvexConvexPairCollision : public btConvexConvexAlgorithm
{
	public:

	CustomConvexConvexPairCollision(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,btCollisionObject* body0,btCollisionObject* body1, btSimplexSolverInterface* simplexSolver, btConvexPenetrationDepthSolver* pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold);
	virtual ~CustomConvexConvexPairCollision();

	virtual void processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	btPersistentManifold*	getManifoldPtr()
	{
		return m_manifoldPtr;
	}

	void	createManifoldPtr(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo);

	struct CreateFunc :public 	btConvexConvexAlgorithm::CreateFunc
	{

		CreateFunc(btSimplexSolverInterface*			simplexSolver, btConvexPenetrationDepthSolver* pdSolver);
		
		virtual ~CreateFunc();

		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(CustomConvexConvexPairCollision));
			return new(mem) CustomConvexConvexPairCollision(ci.m_manifold,ci,body0,body1,m_simplexSolver,m_pdSolver,m_numPerturbationIterations,m_minimumPointsPerturbationThreshold);
		}
	};
	

};


#endif //CUSTOM_CONVEX_CONVEX_PAIR_COLLISION_H
