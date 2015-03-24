#include "GyroscopicSetup.h"

void GyroscopicSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
	gfxBridge.setUpAxis(2);
	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
	gfxBridge.createPhysicsDebugDrawer(m_dynamicsWorld);

	//btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(0.5)));
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);

	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, 0));
	btRigidBody* groundBody;
	groundBody = createRigidBody(0, groundTransform, groundShape);
	groundBody->setFriction(btSqrt(2));
	btVector3 positions[4] = {
		btVector3(0.8, -5, 4),
		btVector3(0.8, -2, 4),
		btVector3(0.8, 2, 4),
		btVector3(0.8, 5, 4)

	};
	int gyroflags[4] = {
		0,//none, no gyroscopic term
		BT_ENABLE_GYROPSCOPIC_FORCE_EXPLICIT,
		BT_ENABLE_GYROPSCOPIC_FORCE_IMPLICIT_EWERT,
		BT_ENABLE_GYROPSCOPIC_FORCE_IMPLICIT_COOPER,
	};

	for (int i = 0; i<4; i++)
	{
		btCylinderShapeZ* top = new btCylinderShapeZ(btVector3(1, 1, 0.125));
		btCapsuleShapeZ* pin = new btCapsuleShapeZ(0.05, 1.5);
		top->setMargin(0.01);
		pin->setMargin(0.01);
		btCompoundShape* compound = new btCompoundShape();
		compound->addChildShape(btTransform::getIdentity(), top);
		compound->addChildShape(btTransform::getIdentity(), pin);
		btVector3 localInertia;
		top->calculateLocalInertia(1, localInertia);
		btRigidBody* body = new btRigidBody(1, 0, compound, localInertia);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(positions[i]);
		body->setCenterOfMassTransform(tr);
		body->setAngularVelocity(btVector3(1, 17, 3));
		body->setLinearVelocity(btVector3(0, 0, 0));
		body->setFriction(btSqrt(1));
		m_dynamicsWorld->addRigidBody(body);
		body->setFlags(gyroflags[i]);
		
		body->setDamping(0.00001f, 0.0001f);


	}

	gfxBridge.autogenerateGraphicsObjects(m_dynamicsWorld);
}
	
