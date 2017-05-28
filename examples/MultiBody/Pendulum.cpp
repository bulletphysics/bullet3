/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2014 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///Original author: Erwin Coumans, January 2016
///Compare the simulation of a pendulum with 

#ifdef USE_GTEST
#include <gtest/gtest.h>
#include "pendulum_gold.h"

#endif

#include "../CommonInterfaces/CommonMultiBodyBase.h"

static btScalar radius(0.05);

struct Pendulum : public CommonMultiBodyBase
{
	btMultiBody* m_multiBody;
	btAlignedObjectArray<btMultiBodyJointFeedback*> m_jointFeedbacks;
	
public:
	
	Pendulum(struct GUIHelperInterface* helper);
	virtual ~Pendulum();
	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);
	virtual void resetCamera()
	{
		float dist = 5;
		float pitch = 270;
		float yaw = 21;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
	
	
};

Pendulum::Pendulum(struct GUIHelperInterface* helper)
:CommonMultiBodyBase(helper)
{
}

Pendulum::~Pendulum()
{
}



void Pendulum::initPhysics()
{
	int upAxis = 1;
	
	m_guiHelper->setUpAxis(upAxis);
	
	this->createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	if (m_dynamicsWorld->getDebugDrawer())
	{
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(
													//btIDebugDraw::DBG_DrawConstraints
													+btIDebugDraw::DBG_DrawWireframe
													+btIDebugDraw::DBG_DrawContactPoints
													+btIDebugDraw::DBG_DrawAabb
													);//+btIDebugDraw::DBG_DrawConstraintLimits);
	}
	{
		bool floating = false;
		bool damping = false;
		bool gyro = false;
		int numLinks = 1;
		bool canSleep = false;
		bool selfCollide = false;
		btVector3 linkHalfExtents(0.05, 0.5, 0.1);
		btVector3 baseHalfExtents(0.05, 0.5, 0.1);
		
		btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
		float baseMass = 0.f;
		
		btMultiBody *pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);
		//pMultiBody->useRK4Integration(true);
		m_multiBody = pMultiBody;
		pMultiBody->setBaseWorldTransform(btTransform::getIdentity());
		
		//init the links
		btVector3 hingeJointAxis(1, 0, 0);
		
		//y-axis assumed up
		btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] , 0);
		btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);	
		btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;
		
		for(int i = 0; i < numLinks; ++i)
		{
			float linkMass = 10.f;
			btVector3 linkInertiaDiag(0.f, 0.f, 0.f);
			btCollisionShape* shape = 0;
			{
				shape = new btSphereShape(radius);
			}
			shape->calculateLocalInertia(linkMass, linkInertiaDiag);
			delete shape;
			
			pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, 
									  btQuaternion(0.f, 0.f, 0.f, 1.f), 
									  hingeJointAxis, 
									  parentComToCurrentPivot, 
									  currentPivotToCurrentCom, false);
			
		}
		
		pMultiBody->finalizeMultiDof();
		
		btMultiBodyDynamicsWorld* world = m_dynamicsWorld;
		
		world->addMultiBody(pMultiBody);
		pMultiBody->setCanSleep(canSleep);
		pMultiBody->setHasSelfCollision(selfCollide);
		pMultiBody->setUseGyroTerm(gyro);
		//
		
		if(!damping)
		{
			pMultiBody->setLinearDamping(0.f);
			pMultiBody->setAngularDamping(0.f);
		}else
		{	pMultiBody->setLinearDamping(0.1f);
			pMultiBody->setAngularDamping(0.9f);
		}
		m_dynamicsWorld->setGravity(btVector3(0,-9.81,0));
		
		
		for (int i=0; i < pMultiBody->getNumLinks(); ++i)
		{
			btCollisionShape* shape =new btSphereShape(radius);
			m_guiHelper->createCollisionShapeGraphicsObject(shape);
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);
			col->setCollisionShape(shape);
			bool isDynamic = 1;
			int collisionFilterGroup = isDynamic? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
			int collisionFilterMask = isDynamic? 	int(btBroadphaseProxy::AllFilter) : 	int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
			world->addCollisionObject(col,collisionFilterGroup,collisionFilterMask);//,2,1+2);
			btVector4 color(1,0,0,1);
			m_guiHelper->createCollisionObjectGraphicsObject(col,color);
			pMultiBody->getLink(i).m_collider=col;
		}
		
		btAlignedObjectArray<btQuaternion> scratch_q;
		btAlignedObjectArray<btVector3> scratch_m;
		pMultiBody->forwardKinematics(scratch_q,scratch_m);
		btAlignedObjectArray<btQuaternion> world_to_local;
		btAlignedObjectArray<btVector3> local_origin;
		pMultiBody->updateCollisionObjectWorldTransforms(world_to_local,local_origin);
	}
	
	
}

void Pendulum::stepSimulation(float deltaTime)
{
	m_multiBody->addJointTorque(0, 20.0);
#ifdef USE_GTEST	
	m_dynamicsWorld->stepSimulation(1./1000.0,0);
#else
	m_dynamicsWorld->stepSimulation(deltaTime);	
#endif
	btVector3 from = m_multiBody->getBaseWorldTransform().getOrigin();
	btVector3 to = m_multiBody->getLink(0).m_collider->getWorldTransform().getOrigin();
	btVector4 color(1,0,0,1);
	if (m_guiHelper->getRenderInterface())
	{
		m_guiHelper->getRenderInterface()->drawLine(from,to,color,btScalar(1));
	}
}

#ifdef USE_GTEST


TEST(BulletDynamicsTest, pendulum) 
{
	DummyGUIHelper noGfx;
	Pendulum* setup = new Pendulum(&noGfx);
	setup->initPhysics();
	int numGoldValues = sizeof(sPendulumGold)/sizeof(float);
	for (int i=0;i<2000;i++)
	{
		setup->stepSimulation(0.001);
		int index = i*2+1;
		ASSERT_LE(index,numGoldValues);
		ASSERT_NEAR(setup->m_multiBody->getJointPos(0),sPendulumGold[index],0.005);
		
	}
	setup->exitPhysics();
	delete setup;
	
}


int main(int argc, char **argv) {
#if _MSC_VER
        _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
        //void *testWhetherMemoryLeakDetectionWorks = malloc(1);
#endif
        ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}

#endif //USE_GTEST

class CommonExampleInterface*    TestPendulumCreateFunc(struct CommonExampleOptions& options)
{
	return new Pendulum(options.m_guiHelper);
}
