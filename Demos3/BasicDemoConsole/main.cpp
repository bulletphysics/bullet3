
#include <stdio.h>
#include "../../Demos/BasicDemo/BasicDemoPhysicsSetup.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"

int main(int argc, char* argv[])
{

	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btDiscreteDynamicsWorld* m_dynamicsWorld;


	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	//Use the btMultiBodyConstraintSolver for Featherstone btMultiBody support
	btMultiBodyConstraintSolver* sol = new btMultiBodyConstraintSolver;
	m_solver = sol;

	//use btMultiBodyDynamicsWorld for Featherstone btMultiBody support
	btMultiBodyDynamicsWorld* world = new btMultiBodyDynamicsWorld(m_dispatcher,m_broadphase,sol,m_collisionConfiguration);
	m_dynamicsWorld = world;
//	m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btVector3 groundHalfExtents(50,50,50);
	btCollisionShape* groundShape = new btBoxShape(groundHalfExtents);
	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,00));

	/////////////////////////////////////////////////////////////////	
	/////////////////////////////////////////////////////////////////
	bool floating = false;
	bool damping = true;
	bool gyro = true;
	int numLinks = 5;
	bool spherical = false;					//set it ot false -to use 1DoF hinges instead of 3DoF sphericals		
	bool canSleep = true;
	bool selfCollide = false;	
	btVector3 linkHalfExtents(0.05, 0.37, 0.1);
	btVector3 baseHalfExtents(0.05, 0.37, 0.1);

	btVector3 basePosition = btVector3(-0.4f, 3.f, 0.f);
	//mbC->forceMultiDof();							//if !spherical, you can comment this line to check the 1DoF algorithm		
	//init the base	
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;
	
	if(baseMass)
	{
		btCollisionShape *pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	bool isMultiDof = false;
	btMultiBody *pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep, isMultiDof);

	btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
	pMultiBody->setBasePos(basePosition);
	pMultiBody->setWorldToBaseRot(baseOriQuat);
	btVector3 vel(0, 0, 0);
//	pMultiBody->setBaseVel(vel);

	//init the links	
	btVector3 hingeJointAxis(1, 0, 0);
	float linkMass = 1.f;
	btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

	btCollisionShape *pTempBox = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));
	pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
	delete pTempBox;

	//y-axis assumed up
	btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] * 2.f, 0);						//par body's COM to cur body's COM offset	
	btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);							//cur body's COM to cur body's PIV offset
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;	//par body's COM to cur body's PIV offset

	//////
	btScalar q0 = 0.f * SIMD_PI/ 180.f;
	btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
	quat0.normalize();	
	/////

	for(int i = 0; i < numLinks; ++i)
	{
		if(!spherical)			
			pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, false);
		else
			//pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f)/*quat0*/, btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
			pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, false);		
	}

	//pMultiBody->finalizeMultiDof();

	///
	world->addMultiBody(pMultiBody);
	btMultiBody* mbC = pMultiBody;
	mbC->setCanSleep(canSleep);	
	mbC->setHasSelfCollision(selfCollide);
	mbC->setUseGyroTerm(gyro);
	//
	if(!damping)
	{
		mbC->setLinearDamping(0.f);
		mbC->setAngularDamping(0.f);
	}else
	{	mbC->setLinearDamping(0.1f);
		mbC->setAngularDamping(0.9f);
	}
	//
	m_dynamicsWorld->setGravity(btVector3(0, -9.81 ,0));	
	//////////////////////////////////////////////
	if(numLinks > 0)
	{			
		btScalar q0 = 45.f * SIMD_PI/ 180.f;
		if(!spherical)
			if(mbC->isMultiDof())
				mbC->setJointPosMultiDof(0, &q0);
			else
				mbC->setJointPos(0, q0);
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
	double friction = 1;
	{
			
	//	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
		float quat[4]={-world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w()};

			
		if (1)
		{
			btCollisionShape* box = new btBoxShape(baseHalfExtents);			
			btMultiBodyLinkCollider* col= new btMultiBodyLinkCollider(pMultiBody, -1);			
			col->setCollisionShape(box);
								
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(local_origin[0]);
			tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));			
			col->setWorldTransform(tr);
				
			world->addCollisionObject(col, 2,1+2);

			col->setFriction(friction);
			pMultiBody->setBaseCollider(col);
				
		}
	}


	for (int i=0; i < pMultiBody->getNumLinks(); ++i)
	{
		const int parent = pMultiBody->getParent(i);
		world_to_local[i+1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent+1];
		local_origin[i+1] = local_origin[parent+1] + (quatRotate(world_to_local[i+1].inverse() , pMultiBody->getRVector(i)));
	}

		
	for (int i=0; i < pMultiBody->getNumLinks(); ++i)
	{
		
		btVector3 posr = local_origin[i+1];
	//	float pos[4]={posr.x(),posr.y(),posr.z(),1};
			
		float quat[4]={-world_to_local[i+1].x(),-world_to_local[i+1].y(),-world_to_local[i+1].z(),world_to_local[i+1].w()};

		btCollisionShape* box = new btBoxShape(linkHalfExtents);
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

		col->setCollisionShape(box);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
		col->setWorldTransform(tr);
		col->setFriction(friction);
		world->addCollisionObject(col,2,1+2);

			
		pMultiBody->getLink(i).m_collider=col;		
	}

	double a = pMultiBody->getJointPos(0);
	for( int i=0; i<10; i++ )
	{
		m_dynamicsWorld->stepSimulation(1./60.);
	}
	double b = pMultiBody->getJointPos(0);
	for( int i=0; i<10; i++ )
	{
		pMultiBody->addJointTorque(0, 10.0);
		m_dynamicsWorld->stepSimulation(1./60.);
	}
	double c = pMultiBody->getJointPos(0);

	printf("hello\n");
}