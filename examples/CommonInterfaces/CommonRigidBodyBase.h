
#ifndef COMMON_RIGID_BODY_BASE_H
#define COMMON_RIGID_BODY_BASE_H


#include "btBulletDynamicsCommon.h"
#include "CommonExampleInterface.h"
#include "CommonGUIHelperInterface.h"
#include "CommonRenderInterface.h"
#include "CommonCameraInterface.h"

#include "CommonGraphicsAppInterface.h"
#include "CommonWindowInterface.h"

struct CommonRigidBodyBase : public CommonExampleInterface
{
		//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btDiscreteDynamicsWorld* m_dynamicsWorld;

	//data for picking objects
	class btRigidBody*	m_pickedBody;
	class btTypedConstraint* m_pickedConstraint;
	int	m_savedState;
	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btScalar m_oldPickingDist;
	struct GUIHelperInterface* m_guiHelper;

	CommonRigidBodyBase(struct GUIHelperInterface* helper)
	:m_broadphase(0),
		m_dispatcher(0),
		m_solver(0),
		m_collisionConfiguration(0),
		m_dynamicsWorld(0),
		m_pickedBody(0),
		m_pickedConstraint(0),
		m_guiHelper(helper)
	{
	}
	virtual ~CommonRigidBodyBase()
	{
	}


	btDiscreteDynamicsWorld*	getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}

	virtual void createEmptyDynamicsWorld()
	{
		///collision configuration contains default setup for memory, collision setup
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		//m_collisionConfiguration->setConvexConvexMultipointIterations();

		///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

		m_broadphase = new btDbvtBroadphase();

		///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
		m_solver = sol;

		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

		m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	}


	virtual void stepSimulation(float deltaTime)
	{
		if (m_dynamicsWorld)
		{
			m_dynamicsWorld->stepSimulation(deltaTime);
		}
	}

	virtual void physicsDebugDraw(int debugFlags)
	{
		if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
		{
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
			m_dynamicsWorld->debugDrawWorld();
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
		m_dynamicsWorld=0;

		delete m_solver;
		m_solver=0;

		delete m_broadphase;
		m_broadphase=0;

		delete m_dispatcher;
		m_dispatcher=0;

		delete m_collisionConfiguration;
		m_collisionConfiguration=0;
	}

	
    virtual void    debugDraw(int debugDrawFlags)
    {
     	if (m_dynamicsWorld)
        {
			if (m_dynamicsWorld->getDebugDrawer())
			{
				m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugDrawFlags);
			}
            m_dynamicsWorld->debugDrawWorld();
        }

    }

	virtual bool	keyboardCallback(int key, int state)
	{
		if ((key==B3G_F3) && state && m_dynamicsWorld)
		{
			btDefaultSerializer*	serializer = new btDefaultSerializer();
			m_dynamicsWorld->serialize(serializer);

			FILE* file = fopen("testFile.bullet","wb");
			fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1, file);
			fclose(file);
			//b3Printf("btDefaultSerializer wrote testFile.bullet");
			delete serializer;
			return true;

		}
		return false;//don't handle this key
	}

	
	btVector3	getRayTo(int x,int y)
	{
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		
		if (!renderer)
		{
			btAssert(0);
			return btVector3(0,0,0);
		}

		float top = 1.f;
		float bottom = -1.f;
		float nearPlane = 1.f;
		float tanFov = (top-bottom)*0.5f / nearPlane;
		float fov = btScalar(2.0) * btAtan(tanFov);

		btVector3 camPos,camTarget;
		
		renderer->getActiveCamera()->getCameraPosition(camPos);
		renderer->getActiveCamera()->getCameraTargetPosition(camTarget);

		btVector3	rayFrom = camPos;
		btVector3 rayForward = (camTarget-camPos);
		rayForward.normalize();
		float farPlane = 10000.f;
		rayForward*= farPlane;

		btVector3 rightOffset;
		btVector3 cameraUp=btVector3(0,0,0);
		cameraUp[m_guiHelper->getAppInterface()->getUpAxis()]=1;

		btVector3 vertical = cameraUp;

		btVector3 hor;
		hor = rayForward.cross(vertical);
		hor.normalize();
		vertical = hor.cross(rayForward);
		vertical.normalize();

		float tanfov = tanf(0.5f*fov);


		hor *= 2.f * farPlane * tanfov;
		vertical *= 2.f * farPlane * tanfov;

		btScalar aspect;
		float width = float(renderer->getScreenWidth());
		float height = float (renderer->getScreenHeight());

		aspect =  width / height;

		hor*=aspect;


		btVector3 rayToCenter = rayFrom + rayForward;
		btVector3 dHor = hor * 1.f/width;
		btVector3 dVert = vertical * 1.f/height;


		btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
		rayTo += btScalar(x) * dHor;
		rayTo -= btScalar(y) * dVert;
		return rayTo;
	}

	virtual bool	mouseMoveCallback(float x,float y)
	{
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		
		if (!renderer)
		{
			btAssert(0);
			return false;
		}

		btVector3 rayTo = getRayTo(int(x), int(y));
		btVector3 rayFrom;
		renderer->getActiveCamera()->getCameraPosition(rayFrom);
		movePickedBody(rayFrom,rayTo);

		return false;
	}

	virtual bool	mouseButtonCallback(int button, int state, float x, float y)
	{
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		
		if (!renderer)
		{
			btAssert(0);
			return false;
		}
		
		CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;

#if 0
		if (window->isModifierKeyPressed(B3G_ALT))
		{
			printf("ALT pressed\n");
		} else
		{
			printf("NO ALT pressed\n");
		}
		
		if (window->isModifierKeyPressed(B3G_SHIFT))
		{
			printf("SHIFT pressed\n");
		} else
		{
			printf("NO SHIFT pressed\n");
		}
		
		if (window->isModifierKeyPressed(B3G_CONTROL))
		{
			printf("CONTROL pressed\n");
		} else
		{
			printf("NO CONTROL pressed\n");
		}
#endif
		
		
		if (state==1)
		{
			if(button==0 && (!window->isModifierKeyPressed(B3G_ALT) && !window->isModifierKeyPressed(B3G_CONTROL) ))
			{
				btVector3 camPos;
				renderer->getActiveCamera()->getCameraPosition(camPos);

				btVector3 rayFrom = camPos;
				btVector3 rayTo = getRayTo(int(x),int(y));

				pickBody(rayFrom, rayTo);


			}
		} else
		{
			if (button==0)
			{
				removePickingConstraint();
				//remove p2p
			}
		}

		//printf("button=%d, state=%d\n",button,state);
		return false;
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
					m_savedState = m_pickedBody->getActivationState();
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

				btVector3 newPivotB;

				btVector3 dir = rayToWorld - rayFromWorld;
				dir.normalize();
				dir *= m_oldPickingDist;

				newPivotB = rayFromWorld + dir;
				pickCon->setPivotB(newPivotB);
				return true;
			}
		}
		return false;
	}
	virtual void removePickingConstraint()
	{
		if (m_pickedConstraint)
		{
			m_pickedBody->forceActivationState(m_savedState);
			m_pickedBody->activate();
			m_dynamicsWorld->removeConstraint(m_pickedConstraint);
			delete m_pickedConstraint;
			m_pickedConstraint = 0;
			m_pickedBody = 0;
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


	
	virtual void renderScene()
	{
		m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
		
		m_guiHelper->render(m_dynamicsWorld);
	}
};

#endif //COMMON_RIGID_BODY_SETUP_H

