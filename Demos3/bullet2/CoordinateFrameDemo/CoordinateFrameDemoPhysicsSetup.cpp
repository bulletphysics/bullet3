

#include "CoordinateFrameDemoPhysicsSetup.h"
#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5


bool showRigidBodyCenterOfMass = true;

void    CoordinateFrameDemoPhysicsSetup::debugDraw()
{
	/*
	for (int i=0;i<m_dynamicsWorld->getCollisionObjectArray().size();i++)
	{
		const btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
		if (showRigidBodyCenterOfMass)
		{
			m_dynamicsWorld->getDebugDrawer()->drawTransform(colObj->getWorldTransform(),1);
		}
	}
	*/
	m_dynamicsWorld->debugDrawWorld();
}

void CoordinateFrameDemoPhysicsSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,0,0));
	gfxBridge.createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	m_dynamicsWorld->getDebugDrawer()->setDebugMode(m_dynamicsWorld->getDebugDrawer()->getDebugMode() + btIDebugDraw::DBG_DrawFrames);

	btScalar sqr2 = btSqrt(2);
	btVector3 tetraVerts[] = {
		btVector3(1.f,	0.f,  -1/sqr2),
		btVector3(-1.f,	0.f,  -1/sqr2),
		btVector3(0, 1.f,	1/sqr2),
		btVector3(0, -1.f,	1/sqr2),
	};

	

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btCompoundShape* hull = new btCompoundShape();
		btConvexHullShape* childHull = new btConvexHullShape(&tetraVerts[0].getX(),sizeof(tetraVerts)/sizeof(btVector3),sizeof(btVector3));
		
		childHull->initializePolyhedralFeatures();
		btTransform childTrans;
		childTrans.setIdentity();
		childTrans.setOrigin(btVector3(2,0,0));
		hull->addChildShape(childTrans,childHull);
		gfxBridge.createCollisionShapeGraphicsObject(hull);
		m_collisionShapes.push_back(hull);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			hull->calculateLocalInertia(mass,localInertia);


		startTransform.setOrigin(btVector3(0,0,0));
		
		btRigidBody* body = createRigidBody(mass,startTransform,hull);
		gfxBridge.createRigidBodyGraphicsObject(body, btVector3(1, 1, 0));
	}

}










