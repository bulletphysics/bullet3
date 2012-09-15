/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"

#include "GLDebugFont.h"
#include <stdio.h> //printf debugging

#include "GyroscopicDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"


#include "GLDebugDrawer.h"
static GLDebugDrawer	gDebugDrawer;





void	GyroscopicDemo::setupEmptyDynamicsWorld()
{
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_overlappingPairCache = new btDbvtBroadphase();
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
}

void	GyroscopicDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}
void	GyroscopicDemo::initPhysics()
{
	m_azi=90;
	m_ele = 20;

	setTexturing(true);
	setShadows(true);
	setCameraUp(btVector3(0,0,1));
	setCameraForwardAxis(1);
	m_sundirection.setValue(0,-1,-1);
	setCameraDistance(7.f);

	setupEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,0,-9.8));
	m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);


	//btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(0.5)));
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,0,1),0);
	
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,0,0));
	btRigidBody* groundBody;
	groundBody= localCreateRigidBody(0, groundTransform, groundShape);
	groundBody->setFriction(btSqrt(2));
	btVector3 positions[2] = {
		btVector3(0.8,-2,2),
		btVector3(0.8,2,2)
	};
	bool gyro[2] = {
		true,
		false
	};

	for (int i=0;i<2;i++)
	{
		btCylinderShapeZ* top  = new btCylinderShapeZ(btVector3(1,1,0.125));
		btCapsuleShapeZ* pin  = new btCapsuleShapeZ(0.05,1.5);
		top->setMargin(0.01);
		pin->setMargin(0.01);
		btCompoundShape* compound = new btCompoundShape();
		compound->addChildShape(btTransform::getIdentity(),top);
		compound->addChildShape(btTransform::getIdentity(),pin);
		btVector3 localInertia;
		top->calculateLocalInertia(1,localInertia);
		btRigidBody* body = new btRigidBody(1,0,compound,localInertia);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(positions[i]);
		body->setCenterOfMassTransform(tr);
		body->setAngularVelocity(btVector3(0,0,15));
		body->setLinearVelocity(btVector3(0,.2,0));
		body->setFriction(btSqrt(1));
		m_dynamicsWorld->addRigidBody(body);
		if (gyro[i])
		{
			body->setFlags(BT_ENABLE_GYROPSCOPIC_FORCE);
		} else
		{
			body->setFlags(0);
		}
		body->setDamping(0.00001f,0.0001f);

		
	}

}

void	GyroscopicDemo::exitPhysics()
{

		int i;

	//removed/delete constraints
	for (i=m_dynamicsWorld->getNumConstraints()-1; i>=0 ;i--)
	{
		btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
		m_dynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}

	//remove the rigidbodies from the dynamics world and delete them
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}




	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	m_collisionShapes.clear();

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

}

GyroscopicDemo::GyroscopicDemo()
{
}
GyroscopicDemo::~GyroscopicDemo()
{
	//cleanup in the reverse order of creation/initialization

	exitPhysics();

}


void GyroscopicDemo::clientMoveAndDisplay()
{
	
 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

 	float dt = float(getDeltaTimeMicroseconds()) * 0.000001f;
	//printf("dt = %f: ",dt);

	{
		static bool once = true;
		if ( m_dynamicsWorld->getDebugDrawer() && once)
		{
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
			once=false;
		}
	}

	
	{
	 	//during idle mode, just run 1 simulation step maximum

		int numSimSteps = m_dynamicsWorld->stepSimulation(dt,100,1./1000.f);

		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	
		
	}
	renderme();


    glFlush();
    swapBuffers();
}




void GyroscopicDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();


	renderme();

    glFlush();
    swapBuffers();
}


