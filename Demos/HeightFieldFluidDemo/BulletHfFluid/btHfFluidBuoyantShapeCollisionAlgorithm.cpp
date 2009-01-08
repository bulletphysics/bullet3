#include <stdio.h>

#include "btHfFluidBuoyantShapeCollisionAlgorithm.h"
#include "btHfFluidBuoyantConvexShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btHfFluid.h"

btHfFluidBuoyantShapeCollisionAlgorithm::btHfFluidBuoyantShapeCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci,btCollisionObject* col0,btCollisionObject* col1,btSimplexSolverInterface* simplexSolver, btConvexPenetrationDepthSolver* pdSolver)
: btCollisionAlgorithm(ci), m_convexConvexAlgorithm(NULL, ci, col0, col1, simplexSolver, pdSolver) 
{
	m_collisionObject0 = col0;
	m_collisionObject1 = col1;
}

btHfFluidBuoyantShapeCollisionAlgorithm::~btHfFluidBuoyantShapeCollisionAlgorithm()
{
}

void btHfFluidBuoyantShapeCollisionAlgorithm::processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	btHfFluidBuoyantConvexShape* tmpShape0 = (btHfFluidBuoyantConvexShape*)body0->getCollisionShape();
	btHfFluidBuoyantConvexShape* tmpShape1 = (btHfFluidBuoyantConvexShape*)body1->getCollisionShape();
	btConvexShape* convexShape0 = tmpShape0->getConvexShape();
	btConvexShape* convexShape1 = tmpShape1->getConvexShape();

	body0->setCollisionShape (convexShape0);
	body1->setCollisionShape (convexShape1);

	m_convexConvexAlgorithm.processCollision (body0, body1, dispatchInfo,resultOut);

	body0->setCollisionShape (tmpShape0);
	body1->setCollisionShape (tmpShape1);
}

btScalar btHfFluidBuoyantShapeCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	btHfFluidBuoyantConvexShape* tmpShape0 = (btHfFluidBuoyantConvexShape*)body0->getCollisionShape();
	btHfFluidBuoyantConvexShape* tmpShape1 = (btHfFluidBuoyantConvexShape*)body1->getCollisionShape();
	btConvexShape* convexShape0 = tmpShape0->getConvexShape();
	btConvexShape* convexShape1 = tmpShape1->getConvexShape();

	body0->setCollisionShape (convexShape0);
	body1->setCollisionShape (convexShape1);

	btScalar toi = btScalar(0.0f);

	toi = m_convexConvexAlgorithm.calculateTimeOfImpact (body0, body1, dispatchInfo, resultOut);

	body0->setCollisionShape (tmpShape0);
	body1->setCollisionShape (tmpShape1);

	return toi;
}
