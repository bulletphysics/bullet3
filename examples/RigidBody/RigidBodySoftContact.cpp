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



#include "RigidBodySoftContact.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_X 1
#define ARRAY_SIZE_Z 1

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"




struct RigidBodySoftContact : public CommonRigidBodyBase
{
	RigidBodySoftContact(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~RigidBodySoftContact()
    {
        
    }
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 3;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
};





void RigidBodySoftContact::initPhysics()
{
    
    
	m_guiHelper->setUpAxis(1);

	//createEmptyDynamicsWorld();
	{
		///collision configuration contains default setup for memory, collision setup
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		//m_collisionConfiguration->setConvexConvexMultipointIterations();

		///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

		m_broadphase = new btDbvtBroadphase();

		///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
		//btMLCPSolver* sol = new btMLCPSolver(new btSolveProjectedGaussSeidel());
		m_solver = sol;

		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

		m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	}
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);
	m_dynamicsWorld->getSolverInfo().m_erp2 = 0.f;
	m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.f;
	m_dynamicsWorld->getSolverInfo().m_numIterations = 3;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD;// | SOLVER_RANDMIZE_ORDER;
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = false;
	
	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	

	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	{
		btScalar mass(0.);
		btRigidBody* body = createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1));
        
        body->setContactStiffnessAndDamping(300,10);
		
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		//btBoxShape* colShape = createBoxShape(btVector3(1,1,1));
		

		btCollisionShape* childShape = new btSphereShape(btScalar(0.5));
		btCompoundShape* colShape = new btCompoundShape();
		colShape->addChildShape(btTransform::getIdentity(),childShape);

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();
		
		startTransform.setRotation(btQuaternion(btVector3(1,1,1),SIMD_PI/10.));
		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);


		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(btVector3(
										btScalar(2.0*i+0.1),
										btScalar(3+2.0*k),
										btScalar(2.0*j)));

			
					btRigidBody* body;
					body = createRigidBody(mass,startTransform,colShape);
					//body->setAngularVelocity(btVector3(1,1,1));
					

				}
			}
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void RigidBodySoftContact::renderScene()
{
	CommonRigidBodyBase::renderScene();
	
}







CommonExampleInterface*    RigidBodySoftContactCreateFunc(CommonExampleOptions& options)
{
	return new RigidBodySoftContact(options.m_guiHelper);
}



