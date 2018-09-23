
#include "MultiBodySoftContact.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointFeedback.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "../Utils/b3ResourcePath.h"

//static btScalar radius(0.2);

struct MultiBodySoftContact : public CommonMultiBodyBase
{
	btMultiBody* m_multiBody;
	btAlignedObjectArray<btMultiBodyJointFeedback*> m_jointFeedbacks;

	bool m_once;

public:
	MultiBodySoftContact(struct GUIHelperInterface* helper);
	virtual ~MultiBodySoftContact();

	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);

	virtual void resetCamera()
	{
		float dist = 5;
		float pitch = -21;
		float yaw = 270;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

MultiBodySoftContact::MultiBodySoftContact(struct GUIHelperInterface* helper)
	: CommonMultiBodyBase(helper),
	  m_once(true)
{
}

MultiBodySoftContact::~MultiBodySoftContact()
{
}

void MultiBodySoftContact::initPhysics()
{
	int upAxis = 2;

	m_guiHelper->setUpAxis(upAxis);

	btVector4 colors[4] =
		{
			btVector4(1, 0, 0, 1),
			btVector4(0, 1, 0, 1),
			btVector4(0, 1, 1, 1),
			btVector4(1, 1, 0, 1),
		};
	int curColor = 0;

	this->createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0, 0, -10));

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(
		//btIDebugDraw::DBG_DrawConstraints
		+btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawAabb);  //+btIDebugDraw::DBG_DrawConstraintLimits);

	//create a static ground object
	if (1)
	{
		btVector3 groundHalfExtents(50, 50, 50);
		btBoxShape* box = new btBoxShape(groundHalfExtents);
		box->initializePolyhedralFeatures();

		m_guiHelper->createCollisionShapeGraphicsObject(box);
		btTransform start;
		start.setIdentity();
		btVector3 groundOrigin(0, 0, -50.5);
		start.setOrigin(groundOrigin);
		//	start.setRotation(groundOrn);
		btRigidBody* body = createRigidBody(0, start, box);

		//setContactStiffnessAndDamping will enable compliant rigid body contact
		body->setContactStiffnessAndDamping(300, 10);
		btVector4 color = colors[curColor];
		curColor++;
		curColor &= 3;
		m_guiHelper->createRigidBodyGraphicsObject(body, color);
	}

	{
		btCollisionShape* childShape = new btSphereShape(btScalar(0.5));
		m_guiHelper->createCollisionShapeGraphicsObject(childShape);

		btScalar mass = 1;
		btVector3 baseInertiaDiag;
		bool isFixed = (mass == 0);
		childShape->calculateLocalInertia(mass, baseInertiaDiag);
		btMultiBody* pMultiBody = new btMultiBody(0, 1, baseInertiaDiag, false, false);
		btTransform startTrans;
		startTrans.setIdentity();
		startTrans.setOrigin(btVector3(0, 0, 3));

		pMultiBody->setBaseWorldTransform(startTrans);

		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
		col->setCollisionShape(childShape);
		pMultiBody->setBaseCollider(col);
		bool isDynamic = (mass > 0 && !isFixed);
		int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
		int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

		m_dynamicsWorld->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);  //, 2,1+2);

		pMultiBody->finalizeMultiDof();

		m_dynamicsWorld->addMultiBody(pMultiBody);

		btAlignedObjectArray<btQuaternion> scratch_q;
		btAlignedObjectArray<btVector3> scratch_m;
		pMultiBody->forwardKinematics(scratch_q, scratch_m);
		btAlignedObjectArray<btQuaternion> world_to_local;
		btAlignedObjectArray<btVector3> local_origin;
		pMultiBody->updateCollisionObjectWorldTransforms(world_to_local, local_origin);
	}
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void MultiBodySoftContact::stepSimulation(float deltaTime)
{
	if (0)  //m_once)
	{
		m_once = false;
		m_multiBody->addJointTorque(0, 10.0);

		btScalar torque = m_multiBody->getJointTorque(0);
		b3Printf("t = %f,%f,%f\n", torque, torque, torque);  //[0],torque[1],torque[2]);
	}

	m_dynamicsWorld->stepSimulation(deltaTime);
}

class CommonExampleInterface* MultiBodySoftContactCreateFunc(struct CommonExampleOptions& options)
{
	return new MultiBodySoftContact(options.m_guiHelper);
}
