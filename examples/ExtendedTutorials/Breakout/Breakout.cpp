/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "Breakout.h"
#include <math.h>
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "Ball.h"
#include "Brick.h"
#include "Border.h"
#include "Paddle.h"

#define BRICKS_PER_COL 8
#define BRICKS_PER_ROW 9

//TODO: Cleanup objects in exitPhysics

struct BreakoutExample : public CommonRigidBodyBase
{
	BreakoutExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper), m_gameArea(27,48,0),m_border(NULL),m_ball(NULL),m_paddle(NULL),m_desiredVelocity(0)
	{
	}
	virtual ~BreakoutExample(){}
	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);
	virtual void renderScene();
	virtual bool keyboardCallback(int key, int state);
	void resetCamera()
	{
		float dist = m_gameArea.y()/2.0f / 1.191753593f;// tanf(100.0/2);
		float pitch = 0;
		float yaw = 0;
		float targetPos[3]={m_gameArea.x()/2, m_gameArea.y()/2, -dist};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

	Border* m_border;
	Ball* m_ball;
	Paddle* m_paddle;
	float m_desiredVelocity;

	btVector3 m_gameArea;

	btVector3 m_last_ball_direction;
};

bool objectContactProcessedCallback(btManifoldPoint& cp,
                                void* body0, void* body1)
{
    btCollisionObject* obA = static_cast<btCollisionObject*>(body0);
    btCollisionObject* obB = static_cast<btCollisionObject*>(body1);

	//6
	GameObject* pnA = (GameObject*)obA->getUserPointer();
	GameObject* pnB = (GameObject*)obB->getUserPointer();

	//
	btTransform removePosition = btTransform(btQuaternion(),btVector3(100,100,100));
	if (pnA && pnA->m_tag == GameObject::BRICK) {
		pnA->m_body->setWorldTransform(removePosition); //TODO: Fix this with correct graphical removal
//	                	m_dynamicsWorld->removeRigidBody(pnA->m_body);
	}

	//8
	if (pnB && pnB->m_tag == GameObject::BRICK){
		pnB->m_body->setWorldTransform(removePosition); //TODO: Fix this with correct graphical removal
//	                    m_dynamicsWorld->removeRigidBody(pnB->m_body);
	}


    return false;
}

void BreakoutExample::initPhysics()
{

//	gContactProcessedCallback = objectContactProcessedCallback;

	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,0,0)); // we do not need gravity
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	{
		m_border = new Border();
	    m_border->setPosition(btVector3(btScalar(m_gameArea.x()/2.0), btScalar(m_gameArea.y()/2.0), btScalar(0)));
	    m_dynamicsWorld->addRigidBody(m_border->m_body);

		m_paddle = new Paddle();
	    m_paddle->setPosition(btVector3(btScalar(m_gameArea.x()/2.0), btScalar(m_gameArea.y() * 0.05), btScalar(0)));
	    m_dynamicsWorld->addRigidBody(m_paddle->m_body);

		m_ball = new Ball();
	    m_ball->setPosition(btVector3(btScalar(m_gameArea.x()/2.0), btScalar(m_gameArea.y() * 0.1), btScalar(0)));
	    m_ball->m_body->setActivationState(DISABLE_DEACTIVATION);
	    m_dynamicsWorld->addRigidBody(m_ball->m_body); //Adding ball to world
		m_ball->m_body->setLinearVelocity(btVector3(0,0,0));
		m_last_ball_direction = btVector3(0,0,0);
	    m_desiredVelocity = 20;

		// Generate array of bricks
		for (int j = 0; j < BRICKS_PER_COL; ++j) {
			for (int i = 0; i < BRICKS_PER_ROW; ++i) {
				Brick* brick = new Brick();
				float margin = m_gameArea.x() * 0.1;
				float startY = m_gameArea.y() * 0.5;
				brick->setPosition(
					btVector3(btScalar(margin + (margin * i)),
						btScalar(startY + (margin * j)), btScalar(0)));
				m_dynamicsWorld->addRigidBody(brick->m_body);
			}
		}

	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BreakoutExample::stepSimulation(float deltaTime){
	CommonRigidBodyBase::stepSimulation(deltaTime);

	   if (m_ball->m_body->getWorldTransform().getOrigin().y() < 0)
	    {
		   m_ball->setPosition(btVector3(btScalar(m_gameArea.x()/2.0), btScalar(m_gameArea.y() * 0.1), btScalar(0)));
		   m_ball->m_body->setLinearVelocity(btVector3(0,0,0));
		   return;
	    }

	    //1
	    int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
	    for (int i=0;i<numManifolds;i++)
	    {
	        //2
	        btPersistentManifold* contactManifold =  m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

	        //3
	        int numContacts = contactManifold->getNumContacts();
	        if (numContacts > 0)
	        {
	                //5
	                const btCollisionObject* obA = contactManifold->getBody0();
	                const btCollisionObject* obB = contactManifold->getBody1();

	                //6
	                GameObject* pnA = (GameObject*)obA->getUserPointer();
	                GameObject* pnB = (GameObject*)obB->getUserPointer();

	                //
	                btTransform removePosition = btTransform(btQuaternion(),btVector3(100,100,100));
	                if (pnA && pnA->m_tag == GameObject::BRICK) {
	                	pnA->m_body->setWorldTransform(removePosition); //TODO: Fix this with correct graphical removal
//	                	m_dynamicsWorld->removeRigidBody(pnA->m_body);
	                }

	                //8
	                if (pnB && pnB->m_tag == GameObject::BRICK){
	                	pnB->m_body->setWorldTransform(removePosition); //TODO: Fix this with correct graphical removal
//	                    m_dynamicsWorld->removeRigidBody(pnB->m_body);
	                }
	        }
	    }

	btVector3 newVelocity(0,0,0);
	btVector3 normalizedCurrentBallDirection(0,0,0);
	if(!m_ball->m_body->getLinearVelocity().isZero()){
		normalizedCurrentBallDirection = m_ball->m_body->getLinearVelocity().normalized();
	}
	else if(!m_last_ball_direction.isZero()){
		normalizedCurrentBallDirection = -m_last_ball_direction.normalized();
	}

	newVelocity = normalizedCurrentBallDirection* m_desiredVelocity;
	m_last_ball_direction = m_ball->m_body->getLinearVelocity();
	m_ball->m_body->setLinearVelocity(newVelocity);

	m_ball->m_body->setAngularVelocity(btVector3(0,0,0)); // cancel rotation
}

bool BreakoutExample::keyboardCallback(int key, int state) {
//	b3Printf("Key pressed: %d in state %d ",key,state);

	//key 1, key 2, key 3
	switch (key) {
	case 32 /*ASCII for space */:{
		if(m_ball->m_body->getLinearVelocity().isZero()){
			b3Printf("Shooting ball!");
			m_ball->m_body->setLinearVelocity(btVector3((rand() % 50)-25,rand() % 15,0));
		}
		return true;
	}
	case 110 /*ASCII for n*/: {
		btTransform oldTransform = m_paddle->m_body->getWorldTransform();
		oldTransform.setOrigin(oldTransform.getOrigin()+btVector3(0.5,0,0));
		m_paddle->m_body->setWorldTransform(oldTransform);
		return true;
	}
	case 109 /*ASCII for m*/: {
		btTransform oldTransform = m_paddle->m_body->getWorldTransform();
		oldTransform.setOrigin(oldTransform.getOrigin()+btVector3(-0.5,0,0));
		m_paddle->m_body->setWorldTransform(oldTransform);
		return true;
	}
	}

	return false;
}


void BreakoutExample::renderScene()
{
	CommonRigidBodyBase::renderScene();	
}

CommonExampleInterface*    ET_BreakoutCreateFunc(CommonExampleOptions& options)
{
	return new BreakoutExample(options.m_guiHelper);
}



