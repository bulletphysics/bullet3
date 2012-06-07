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
#include <stdio.h>

#include "btHfFluidBuoyantShapeCollisionAlgorithm.h"
#include "btHfFluidBuoyantConvexShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btHfFluid.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

btHfFluidBuoyantShapeCollisionAlgorithm::btHfFluidBuoyantShapeCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci,const btCollisionObjectWrapper* col0Wrap,const btCollisionObjectWrapper* col1Wrap,btSimplexSolverInterface* simplexSolver, btConvexPenetrationDepthSolver* pdSolver)
: btCollisionAlgorithm(ci), m_convexConvexAlgorithm(NULL, ci, col0Wrap, col1Wrap, simplexSolver, pdSolver,0,0) 
{
	m_collisionObject0 = col0Wrap->getCollisionObject();
	m_collisionObject1 = col1Wrap->getCollisionObject();
}

btHfFluidBuoyantShapeCollisionAlgorithm::~btHfFluidBuoyantShapeCollisionAlgorithm()
{
}

void btHfFluidBuoyantShapeCollisionAlgorithm::processCollision (const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	const btHfFluidBuoyantConvexShape* tmpShape0 = (const btHfFluidBuoyantConvexShape*)body0Wrap->getCollisionShape();
	const btHfFluidBuoyantConvexShape* tmpShape1 = (const btHfFluidBuoyantConvexShape*)body1Wrap->getCollisionShape();
	const btConvexShape* convexShape0 = tmpShape0->getConvexShape();
	const btConvexShape* convexShape1 = tmpShape1->getConvexShape();

	//body0->setCollisionShape (convexShape0);
	//body1->setCollisionShape (convexShape1);

	btCollisionObjectWrapper ob0(body0Wrap,convexShape0,body0Wrap->getCollisionObject(),body0Wrap->getWorldTransform());
	btCollisionObjectWrapper ob1(body1Wrap,convexShape1,body1Wrap->getCollisionObject(),body1Wrap->getWorldTransform());
	m_convexConvexAlgorithm.processCollision (&ob0, &ob1, dispatchInfo,resultOut);

	
}

btScalar btHfFluidBuoyantShapeCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	btAssert(0);

	btHfFluidBuoyantConvexShape* tmpShape0 = (btHfFluidBuoyantConvexShape*)body0->getCollisionShape();
	btHfFluidBuoyantConvexShape* tmpShape1 = (btHfFluidBuoyantConvexShape*)body1->getCollisionShape();
	const btConvexShape* convexShape0 = tmpShape0->getConvexShape();
	const btConvexShape* convexShape1 = tmpShape1->getConvexShape();

	
	btScalar toi = btScalar(0.0f);

	toi = m_convexConvexAlgorithm.calculateTimeOfImpact (body0, body1, dispatchInfo, resultOut);

	
	return toi;
}
