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



#include "Bridge.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../CommonInterfaces/CommonRigidBodyBase.h"

const int TOTAL_PLANKS = 10;
struct BridgeExample : public CommonRigidBodyBase
{
	BridgeExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~BridgeExample(){}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 41;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}
};

void BridgeExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0)); 
	{
		btScalar mass(0.);
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1));
	}

	//create two fixed boxes to hold the planks



	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btScalar plankWidth = 0.4;
		btScalar plankHeight = 0.2;
		btScalar plankBreadth = 1;
		btScalar plankOffset = plankWidth; //distance between two planks
		btScalar bridgeWidth = plankWidth*TOTAL_PLANKS + plankOffset*(TOTAL_PLANKS-1);
		btScalar bridgeHeight = 5;
		btScalar halfBridgeWidth = bridgeWidth*0.5f;

        btBoxShape* colShape = createBoxShape(btVector3(plankWidth,plankHeight,plankBreadth));
		 
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		//create a set of boxes to represent bridge 
		btAlignedObjectArray<btRigidBody*> boxes;
		int lastBoxIndex = TOTAL_PLANKS-1;
		for(int i=0;i<TOTAL_PLANKS;++i) {
			float t =   float(i)/lastBoxIndex;
			t = -(t*2-1.0f) * halfBridgeWidth;
			startTransform.setOrigin(btVector3(									 
									 btScalar(t),
									 bridgeHeight,
									 btScalar(0)
									 )
									 );
			boxes.push_back(createRigidBody((i==0 || i==lastBoxIndex)?0:mass,startTransform,colShape));		  
		} 
		 
		
		//add N-1 spring constraints
		for(int i=0;i<TOTAL_PLANKS-1;++i) {
			btRigidBody* b1 = boxes[i];
			btRigidBody* b2 = boxes[i+1];
			 
			btPoint2PointConstraint* leftSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(-0.5,0,-0.5), btVector3(0.5,0,-0.5));
			m_dynamicsWorld->addConstraint(leftSpring);
			 
			btPoint2PointConstraint* rightSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(-0.5,0,0.5), btVector3(0.5,0,0.5));
			m_dynamicsWorld->addConstraint(rightSpring);
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void BridgeExample::renderScene()
{
	CommonRigidBodyBase::renderScene();	
}







CommonExampleInterface*    ET_BridgeCreateFunc(CommonExampleOptions& options)
{
	return new BridgeExample(options.m_guiHelper);
}



