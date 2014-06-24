
#include "CcdPhysicsSetup.h"
#include "btBulletDynamicsCommon.h"
#define CUBE_HALF_EXTENTS 1.f
#define EXTRA_HEIGHT 1.f


void KinematicObjectSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
	createEmptyDynamicsWorld();
	{
		btBoxShape* box = new btBoxShape(btVector3(btScalar(10.), btScalar(1.), btScalar(10.)));
		gfxBridge.createCollisionShapeGraphicsObject(box);
		btTransform startTrans;
		startTrans.setIdentity();
		startTrans.setOrigin(btVector3(0, -1, 0));
		btRigidBody* body = createRigidBody(0, startTrans, box);
		body->setMotionState(0);
		body->setFriction(1);
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		body->setActivationState(DISABLE_DEACTIVATION);

		gfxBridge.createRigidBodyGraphicsObject(body, btVector3(0,1,0));
	}
	{
		btBoxShape* box = new btBoxShape(btVector3(btScalar(1.), btScalar(1.), btScalar(1.)));
		gfxBridge.createCollisionShapeGraphicsObject(box);
		btTransform startTrans;
		startTrans.setIdentity();
		startTrans.setOrigin(btVector3(0, 1, 0));
		btRigidBody* body = createRigidBody(1, startTrans, box);
		body->setFriction(1);
		body->setActivationState(DISABLE_DEACTIVATION);
		gfxBridge.createRigidBodyGraphicsObject(body, btVector3(1, 1, 0));
	}
}

void KinematicObjectSetup::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[0];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			btMotionState* ms = body->getMotionState();

			btTransform startTrans;
			startTrans.setIdentity();
			static float time = 0.f;
			time += 0.01f;
			static float xPos = 0.f;
			xPos = sinf(time)*10.f;
			startTrans.setOrigin(btVector3(xPos, -1, 0));
			if (ms)
			{

				ms->setWorldTransform(startTrans);
			}
			 else
			 {
				body->setWorldTransform(startTrans);
			 }
		}
		m_dynamicsWorld->stepSimulation(deltaTime);
	}
}

void CcdPhysicsSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
	createEmptyDynamicsWorld();


	///create a few basic rigid bodies
	btBoxShape* box = new btBoxShape(btVector3(btScalar(110.), btScalar(1.), btScalar(110.)));
	gfxBridge.createCollisionShapeGraphicsObject(box);
	//	box->initializePolyhedralFeatures();
	btCollisionShape* groundShape = box;


	m_collisionShapes.push_back(groundShape);
	//m_collisionShapes.push_back(new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
	m_collisionShapes.push_back(new btBoxShape(btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS)));

	btTransform groundTransform;
	groundTransform.setIdentity();
	//groundTransform.setOrigin(btVector3(5,5,5));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		gfxBridge.createRigidBodyGraphicsObject(body, btVector3(0, 1, 0));
		body->setFriction(0.5);
		//body->setRollingFriction(0.3);
		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btCollisionShape* colShape = new btBoxShape(btVector3(1, 1, 1));
		gfxBridge.createCollisionShapeGraphicsObject(colShape);
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		int gNumObjects = 120;//120;
		int i;
		for (i = 0; i<gNumObjects; i++)
		{
			btCollisionShape* shape = colShape;// m_collisionShapes[1];

			btTransform trans;
			trans.setIdentity();

			//stack them
			int colsize = 10;
			int row = (i*CUBE_HALF_EXTENTS * 2) / (colsize * 2 * CUBE_HALF_EXTENTS);
			int row2 = row;
			int col = (i) % (colsize)-colsize / 2;


			if (col>3)
			{
				col = 11;
				row2 |= 1;
			}

			btVector3 pos(col * 2 * CUBE_HALF_EXTENTS + (row2 % 2)*CUBE_HALF_EXTENTS,
				row * 2 * CUBE_HALF_EXTENTS + CUBE_HALF_EXTENTS + EXTRA_HEIGHT, 0);

			trans.setOrigin(pos);

			float mass = 1.f;

			btRigidBody* body = createRigidBody(mass, trans, shape);
			gfxBridge.createRigidBodyGraphicsObject(body, btVector3(1, 1, 0));

			body->setAnisotropicFriction(shape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
			body->setFriction(0.5);

			//body->setRollingFriction(.3);	
			///when using m_ccdMode
			//if (m_ccdMode == USE_CCD)
			{
				body->setCcdMotionThreshold(CUBE_HALF_EXTENTS);
				body->setCcdSweptSphereRadius(0.9*CUBE_HALF_EXTENTS);
			}
		}
	}

}
