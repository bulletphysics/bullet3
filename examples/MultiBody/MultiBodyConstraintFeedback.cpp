#include "MultiBodyConstraintFeedback.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointFeedback.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

static btScalar radius(0.2);

struct MultiBodyConstraintFeedbackSetup : public CommonMultiBodyBase
{
	btMultiBody* m_multiBody;
	btAlignedObjectArray<btMultiBodyJointFeedback*> m_jointFeedbacks;
	btMultiBodyJointMotor* m_motor;
	bool m_once;

public:
	MultiBodyConstraintFeedbackSetup(struct GUIHelperInterface* helper);
	virtual ~MultiBodyConstraintFeedbackSetup();

	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);

	virtual void resetCamera()
	{
		float dist = 5;
		float pitch = -21;
		float yaw = 270;
		float targetPos[3] = {-1.34, 3.4, -0.44};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

MultiBodyConstraintFeedbackSetup::MultiBodyConstraintFeedbackSetup(struct GUIHelperInterface* helper)
	: CommonMultiBodyBase(helper),
	  m_motor(0),
	  m_once(true)
{
}

MultiBodyConstraintFeedbackSetup::~MultiBodyConstraintFeedbackSetup()
{
}

void MultiBodyConstraintFeedbackSetup::initPhysics()
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
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(
		//btIDebugDraw::DBG_DrawConstraints
		+btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawAabb);  //+btIDebugDraw::DBG_DrawConstraintLimits);


	m_dynamicsWorld->getSolverInfo().m_jointFeedbackInWorldSpace = true;
	m_dynamicsWorld->getSolverInfo().m_jointFeedbackInJointFrame = true;

	//create a static ground object
	if (1)
	{
		btVector3 groundHalfExtents(10, 10, 0.2);
		btBoxShape* box = new btBoxShape(groundHalfExtents);
		box->initializePolyhedralFeatures();

		m_guiHelper->createCollisionShapeGraphicsObject(box);
		btTransform start;
		start.setIdentity();
		btVector3 groundOrigin(-0.4f, 3.f, 0.f);
		//btVector3 basePosition = btVector3(-0.4f, 3.f, 0.f);
		groundOrigin[upAxis] -= .5;
		groundOrigin[2] -= 0.6;
		start.setOrigin(groundOrigin);
		btQuaternion groundOrn(btVector3(0, 1, 0), 0.25 * SIMD_PI);

		//	start.setRotation(groundOrn);
		btRigidBody* body = createRigidBody(0, start, box);
		body->setFriction(0);
		btVector4 color = colors[curColor];
		curColor++;
		curColor &= 3;
		m_guiHelper->createRigidBodyGraphicsObject(body, color);
	}

	{
		bool floating = false;
		bool damping = false;
		bool gyro = false;
		int numLinks = 2;
		bool spherical = false;  //set it ot false -to use 1DoF hinges instead of 3DoF sphericals
		bool canSleep = false;
		bool selfCollide = false;
		btVector3 linkHalfExtents(0.05, 0.5, 0.1);
		btVector3 baseHalfExtents(0.05, 0.5, 0.1);

		btVector3 basePosition = btVector3(-0.4f, 3.f, 0.f);
		//mbC->forceMultiDof();							//if !spherical, you can comment this line to check the 1DoF algorithm
		//init the base
		btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
		float baseMass = 0.01f;

		if (baseMass)
		{
			//btCollisionShape *shape = new btSphereShape(baseHalfExtents[0]);// btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
			btCollisionShape* shape = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
			shape->calculateLocalInertia(baseMass, baseInertiaDiag);
			delete shape;
		}

		btMultiBody* pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);

		m_multiBody = pMultiBody;
		btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
		//	baseOriQuat.setEulerZYX(-.25*SIMD_PI,0,-1.75*SIMD_PI);
		pMultiBody->setBasePos(basePosition);
		pMultiBody->setWorldToBaseRot(baseOriQuat);
		btVector3 vel(0, 0, 0);
		//	pMultiBody->setBaseVel(vel);

		//init the links
		btVector3 hingeJointAxis(1, 0, 0);

		//y-axis assumed up
		btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] * 2.f, 0);                      //par body's COM to cur body's COM offset
		btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);                         //cur body's COM to cur body's PIV offset
		btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;  //par body's COM to cur body's PIV offset

		//////
		btScalar q0 = 0.f * SIMD_PI / 180.f;
		btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
		quat0.normalize();
		/////

		for (int i = 0; i < numLinks; ++i)
		{
			float linkMass = i == 0 ? 0.0001 : 1.f;
			//if (i==3 || i==2)
			//	linkMass= 1000;
			btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

			btCollisionShape* shape = 0;
			if (i == 0)
			{
				shape = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));  //
			}
			else
			{
				shape = new btSphereShape(radius);
			}
			shape->calculateLocalInertia(linkMass, linkInertiaDiag);
			delete shape;

			if (!spherical)
			{
				//pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, false);

				if (i == 0)
				{
					pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1,
											  btQuaternion(0.f, 0.f, 0.f, 1.f),
											  hingeJointAxis,
											  parentComToCurrentPivot,
											  currentPivotToCurrentCom, false);
				}
				else
				{
					btVector3 parentComToCurrentCom(0, -linkHalfExtents[1], 0);  //par body's COM to cur body's COM offset
					btVector3 currentPivotToCurrentCom(0, 0, 0);                 //cur body's COM to cur body's PIV offset
					//btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;	//par body's COM to cur body's PIV offset

					pMultiBody->setupFixed(i, linkMass, linkInertiaDiag, i - 1,
										   btQuaternion(0.f, 0.f, 0.f, 1.f),
										   parentComToCurrentPivot,
										   currentPivotToCurrentCom);
				}

				//pMultiBody->setupFixed(i,linkMass,linkInertiaDiag,i-1,btQuaternion(0,0,0,1),parentComToCurrentPivot,currentPivotToCurrentCom,false);
			}
			else
			{
				//pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f)/*quat0*/, btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
				pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, false);
			}
		}

		pMultiBody->finalizeMultiDof();

		//for (int i=pMultiBody->getNumLinks()-1;i>=0;i--)//
		for (int i = 0; i < pMultiBody->getNumLinks(); i++)
		{
			btMultiBodyJointFeedback* fb = new btMultiBodyJointFeedback();
			pMultiBody->getLink(i).m_jointFeedback = fb;
			m_jointFeedbacks.push_back(fb);
			//break;
		}
		btMultiBodyDynamicsWorld* world = m_dynamicsWorld;

		///
		world->addMultiBody(pMultiBody);
		btMultiBody* mbC = pMultiBody;
		mbC->setCanSleep(canSleep);
		mbC->setHasSelfCollision(selfCollide);
		mbC->setUseGyroTerm(gyro);
		//
		if (!damping)
		{
			mbC->setLinearDamping(0.f);
			mbC->setAngularDamping(0.f);
		}
		else
		{
			mbC->setLinearDamping(0.1f);
			mbC->setAngularDamping(0.9f);
		}
		//
		m_dynamicsWorld->setGravity(btVector3(0, 0, -10));

		//////////////////////////////////////////////
		if (0)  //numLinks > 0)
		{
			btScalar q0 = 45.f * SIMD_PI / 180.f;
			if (!spherical)
			{
				mbC->setJointPosMultiDof(0, &q0);
			}
			else
			{
				btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
				quat0.normalize();
				mbC->setJointPosMultiDof(0, quat0);
			}
		}
		///

		btAlignedObjectArray<btQuaternion> world_to_local;
		world_to_local.resize(pMultiBody->getNumLinks() + 1);

		btAlignedObjectArray<btVector3> local_origin;
		local_origin.resize(pMultiBody->getNumLinks() + 1);
		world_to_local[0] = pMultiBody->getWorldToBaseRot();
		local_origin[0] = pMultiBody->getBasePos();
		//  double friction = 1;
		{
			//	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
			// float quat[4]={-world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w()};

			if (1)
			{
				btCollisionShape* shape = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));  //new btSphereShape(baseHalfExtents[0]);
				m_guiHelper->createCollisionShapeGraphicsObject(shape);

				btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
				col->setCollisionShape(shape);

				btTransform tr;
				tr.setIdentity();
				//if we don't set the initial pose of the btCollisionObject, the simulator will do this
				//when syncing the btMultiBody link transforms to the btMultiBodyLinkCollider

				tr.setOrigin(local_origin[0]);
				btQuaternion orn(btVector3(0, 0, 1), 0.25 * 3.1415926538);

				tr.setRotation(orn);
				col->setWorldTransform(tr);

				bool isDynamic = (baseMass > 0 && floating);
				int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
				int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

				world->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);  //, 2,1+2);

				btVector3 color(0.0, 0.0, 0.5);
				m_guiHelper->createCollisionObjectGraphicsObject(col, color);

				//                col->setFriction(friction);
				pMultiBody->setBaseCollider(col);
			}
		}

		for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
		{
			const int parent = pMultiBody->getParent(i);
			world_to_local[i + 1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
			local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), pMultiBody->getRVector(i)));
		}

		for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
		{
			btVector3 posr = local_origin[i + 1];
			//	float pos[4]={posr.x(),posr.y(),posr.z(),1};

			const btScalar quat[4] = {-world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w()};
			btCollisionShape* shape = 0;

			if (i == 0)
			{
				shape = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));  //btSphereShape(linkHalfExtents[0]);
			}
			else
			{
				shape = new btSphereShape(radius);
			}

			m_guiHelper->createCollisionShapeGraphicsObject(shape);
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

			col->setCollisionShape(shape);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(posr);
			tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
			col->setWorldTransform(tr);
			//       col->setFriction(friction);
			bool isDynamic = 1;  //(linkMass > 0);
			int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
			int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

			//if (i==0||i>numLinks-2)
			{
				world->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);  //,2,1+2);
				btVector4 color = colors[curColor];
				curColor++;
				curColor &= 3;
				m_guiHelper->createCollisionObjectGraphicsObject(col, color);

				pMultiBody->getLink(i).m_collider = col;
			}
		}
		int link = 0;
		int targetVelocity = 0.f;
		btScalar maxForce = 100000;
		m_motor = new btMultiBodyJointMotor(pMultiBody, link, targetVelocity, maxForce);
		m_dynamicsWorld->addMultiBodyConstraint(m_motor);
	}
}

void MultiBodyConstraintFeedbackSetup::stepSimulation(float deltaTime)
{
	//m_multiBody->addLinkForce(0,btVector3(100,100,100));
	if (0)  //m_once)
	{
		m_once = false;
		m_multiBody->addJointTorque(0, 10.0);

		btScalar torque = m_multiBody->getJointTorque(0);
		b3Printf("t = %f,%f,%f\n", torque, torque, torque);  //[0],torque[1],torque[2]);
	}
	btScalar timeStep = 1. / 240.f;

	m_dynamicsWorld->stepSimulation(timeStep, 0);

	static int count = 0;
	if ((count & 0x0f) == 0)
	{
		if (m_motor)
		{
			float force = m_motor->getAppliedImpulse(0) / timeStep;
			b3Printf("motor applied force = %f\n", force);
		}

		for (int i = 0; i < m_jointFeedbacks.size(); i++)
		{
			b3Printf("F_reaction[%i] linear:%f,%f,%f, angular:%f,%f,%f",
					 i,
					 m_jointFeedbacks[i]->m_reactionForces.m_topVec[0],
					 m_jointFeedbacks[i]->m_reactionForces.m_topVec[1],
					 m_jointFeedbacks[i]->m_reactionForces.m_topVec[2],

					 m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[0],
					 m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[1],
					 m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[2]

			);
		}
	}
	count++;

	/*
    b3Printf("base angvel = %f,%f,%f",m_multiBody->getBaseOmega()[0],
             m_multiBody->getBaseOmega()[1],
             m_multiBody->getBaseOmega()[2]
             );
    */
	// btScalar jointVel =m_multiBody->getJointVel(0);

	//    b3Printf("child angvel = %f",jointVel);
}

class CommonExampleInterface* MultiBodyConstraintFeedbackCreateFunc(struct CommonExampleOptions& options)
{
	return new MultiBodyConstraintFeedbackSetup(options.m_guiHelper);
}
