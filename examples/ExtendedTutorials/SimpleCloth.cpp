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



#include "SimpleCloth.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

struct SimpleClothExample : public CommonRigidBodyBase
{
	SimpleClothExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~SimpleClothExample(){}
	virtual void initPhysics();
	virtual void renderScene();
	void createEmptyDynamicsWorld()
	{
		m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration(); 
        m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration); 
		 
		m_broadphase = new btDbvtBroadphase();
	 
		m_solver = new btSequentialImpulseConstraintSolver; 
		
		m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
		m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

		softBodyWorldInfo.m_broadphase = m_broadphase;
		softBodyWorldInfo.m_dispatcher = m_dispatcher;
		softBodyWorldInfo.m_gravity = m_dynamicsWorld->getGravity();
		softBodyWorldInfo.m_sparsesdf.Initialize();
	}
	virtual btSoftRigidDynamicsWorld*	getSoftDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}
	void resetCamera()
	{
		float dist = 41;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

	void createSoftBody(const btScalar size, const int num_x, const int num_z, const int fixed=1+2);
	btSoftBodyWorldInfo softBodyWorldInfo;
};

void SimpleClothExample::initPhysics()
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


	{
		const btScalar s=4; //size of cloth patch
		const int NUM_X=31; //vertices on X axis
		const int NUM_Z=31; //vertices on Z axis
		createSoftBody(s,NUM_X, NUM_Z);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void SimpleClothExample::createSoftBody(const btScalar s,
										const int numX,
										const int numY, 
										const int fixed) {
	
	
	 
	btSoftBody* cloth=btSoftBodyHelpers::CreatePatch(softBodyWorldInfo,
		btVector3(-s/2,s+1,0),
		btVector3(+s/2,s+1,0),
		btVector3(-s/2,s+1,+s),
		btVector3(+s/2,s+1,+s),
		numX,numY, 
		fixed,true);
	
	cloth->getCollisionShape()->setMargin(0.001f);
	cloth->generateBendingConstraints(2,cloth->appendMaterial());
	cloth->setTotalMass(10); 
	//cloth->m_cfg.citerations = 10;
//	cloth->m_cfg.diterations = 10;
	cloth->m_cfg.piterations = 5;
	cloth->m_cfg.kDP = 0.005f;
	getSoftDynamicsWorld()->addSoftBody(cloth);

}

void SimpleClothExample::renderScene()
{
	CommonRigidBodyBase::renderScene();	
	btSoftRigidDynamicsWorld* softWorld = getSoftDynamicsWorld();

		for (  int i=0;i<softWorld->getSoftBodyArray().size();i++)
		{
			btSoftBody*	psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
			//if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
			{
				btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
				btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());
			}
		}
}







CommonExampleInterface*    ET_SimpleClothCreateFunc(CommonExampleOptions& options)
{
	return new SimpleClothExample(options.m_guiHelper);
}



