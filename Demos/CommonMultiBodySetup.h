
#ifndef COMMON_MULTI_BODY_SETUP_H
#define COMMON_MULTI_BODY_SETUP_H

//todo: replace this 'btBulletDynamicsCommon.h' header with specific used header files
#include "btBulletDynamicsCommon.h"
#include "CommonPhysicsSetup.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"

struct CommonMultiBodySetup : public CommonPhysicsSetup
{
		//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btMultiBodyConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btMultiBodyDynamicsWorld* m_dynamicsWorld;

	//data for picking objects
	class btRigidBody*	m_pickedBody;
	class btTypedConstraint* m_pickedConstraint;
	class btMultiBodyPoint2Point*		m_pickingMultiBodyPoint2Point;

	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btScalar m_oldPickingDist;
	bool m_prevCanSleep;

	CommonMultiBodySetup()
	:m_broadphase(0),
		m_dispatcher(0),
		m_solver(0),
		m_collisionConfiguration(0),
		m_dynamicsWorld(0),
		m_pickedBody(0),
		m_pickedConstraint(0),
		m_pickingMultiBodyPoint2Point(0),
		m_prevCanSleep(false)
	{
	}

	virtual void createEmptyDynamicsWorld()
	{
		///collision configuration contains default setup for memory, collision setup
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		//m_collisionConfiguration->setConvexConvexMultipointIterations();

		///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

		m_broadphase = new btDbvtBroadphase();

		m_solver = new btMultiBodyConstraintSolver;

		m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

		m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	}


	virtual void stepSimulation(float deltaTime)
	{
		if (m_dynamicsWorld)
		{
			m_dynamicsWorld->stepSimulation(deltaTime);
		}
	}


	virtual void exitPhysics()
	{
		removePickingConstraint();
		//cleanup in the reverse order of creation/initialization

		//remove the rigidbodies from the dynamics world and delete them

		if (m_dynamicsWorld)
		{

            int i;
            for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
            {
                m_dynamicsWorld->removeConstraint(m_dynamicsWorld->getConstraint(i));
            }
			for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
			{
				btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
				btRigidBody* body = btRigidBody::upcast(obj);
				if (body && body->getMotionState())
				{
					delete body->getMotionState();
				}
				m_dynamicsWorld->removeCollisionObject(obj);
				delete obj;
			}
		}
		//delete collision shapes
		for (int j = 0; j<m_collisionShapes.size(); j++)
		{
			btCollisionShape* shape = m_collisionShapes[j];
			delete shape;
		}
		m_collisionShapes.clear();

		delete m_dynamicsWorld;

		delete m_solver;

		delete m_broadphase;

		delete m_dispatcher;

		delete m_collisionConfiguration;
	}

	virtual void syncPhysicsToGraphics(GraphicsPhysicsBridge& gfxBridge)
	{
		if (m_dynamicsWorld)
		{
			gfxBridge.syncPhysicsToGraphics(m_dynamicsWorld);
		}
	}

    virtual void    debugDraw()
    {
     	if (m_dynamicsWorld)
        {
            m_dynamicsWorld->debugDrawWorld();
        }

    }

	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
	{
		if (m_dynamicsWorld==0)
			return false;

		btCollisionWorld::ClosestRayResultCallback rayCallback(rayFromWorld, rayToWorld);

		m_dynamicsWorld->rayTest(rayFromWorld, rayToWorld, rayCallback);
		if (rayCallback.hasHit())
		{

			btVector3 pickPos = rayCallback.m_hitPointWorld;
			btRigidBody* body = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
			if (body)
			{
				//other exclusions?
				if (!(body->isStaticObject() || body->isKinematicObject()))
				{
					m_pickedBody = body;
					m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
					//printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
					btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
					btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
					m_dynamicsWorld->addConstraint(p2p, true);
					m_pickedConstraint = p2p;
					btScalar mousePickClamping = 30.f;
					p2p->m_setting.m_impulseClamp = mousePickClamping;
					//very weak constraint for picking
					p2p->m_setting.m_tau = 0.001f;
				}
			} else
			{
				btMultiBodyLinkCollider* multiCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(rayCallback.m_collisionObject);
				if (multiCol && multiCol->m_multiBody)
				{
						
					m_prevCanSleep = multiCol->m_multiBody->getCanSleep();
					multiCol->m_multiBody->setCanSleep(false);

					btVector3 pivotInA = multiCol->m_multiBody->worldPosToLocal(multiCol->m_link, pickPos);

					btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(multiCol->m_multiBody,multiCol->m_link,0,pivotInA,pickPos);
					//if you add too much energy to the system, causing high angular velocities, simulation 'explodes'
					//see also http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=949
					//so we try to avoid it by clamping the maximum impulse (force) that the mouse pick can apply
					//it is not satisfying, hopefully we find a better solution (higher order integrator, using joint friction using a zero-velocity target motor with limited force etc?)
					btScalar scaling=1;
					p2p->setMaxAppliedImpulse(2*scaling);
		
					btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_dynamicsWorld;
					world->addMultiBodyConstraint(p2p);
					m_pickingMultiBodyPoint2Point =p2p; 
				}
			}



			//					pickObject(pickPos, rayCallback.m_collisionObject);
			m_oldPickingPos = rayToWorld;
			m_hitPos = pickPos;
			m_oldPickingDist = (pickPos - rayFromWorld).length();
			//					printf("hit !\n");
			//add p2p
		}
		return false;
	}
	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
	{
		if (m_pickedBody  && m_pickedConstraint)
		{
			btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickedConstraint);
			if (pickCon)
			{
				//keep it at the same picking distance
		
				btVector3 dir = rayToWorld-rayFromWorld;
				dir.normalize();
				dir *= m_oldPickingDist;

				btVector3 newPivotB = rayFromWorld + dir;
				pickCon->setPivotB(newPivotB);
			}
		}
		
		if (m_pickingMultiBodyPoint2Point)
		{
			//keep it at the same picking distance

		
			btVector3 dir = rayToWorld-rayFromWorld;
			dir.normalize();
			dir *= m_oldPickingDist;

			btVector3 newPivotB = rayFromWorld + dir;
			
			m_pickingMultiBodyPoint2Point->setPivotInB(newPivotB);
		}
		
		return false;
	}
	virtual void removePickingConstraint()
	{
		if (m_pickedConstraint)
		{
			m_dynamicsWorld->removeConstraint(m_pickedConstraint);
			delete m_pickedConstraint;
			m_pickedConstraint = 0;
			m_pickedBody = 0;
		}
		if (m_pickingMultiBodyPoint2Point)
		{
			m_pickingMultiBodyPoint2Point->getMultiBodyA()->setCanSleep(m_prevCanSleep);
			btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_dynamicsWorld;
			world->removeMultiBodyConstraint(m_pickingMultiBodyPoint2Point);
			delete m_pickingMultiBodyPoint2Point;
			m_pickingMultiBodyPoint2Point = 0;
		}
	}



	btBoxShape* createBoxShape(const btVector3& halfExtents)
	{
		btBoxShape* box = new btBoxShape(halfExtents);
		return box;
	}

	btRigidBody*	createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape,  const btVector4& color = btVector4(1, 0, 0, 1))
	{
		btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

		btRigidBody* body = new btRigidBody(cInfo);
		//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);
#endif//

		body->setUserIndex(-1);
		m_dynamicsWorld->addRigidBody(body);
		return body;
	}
};

#endif //COMMON_MULTI_BODY_SETUP_H

