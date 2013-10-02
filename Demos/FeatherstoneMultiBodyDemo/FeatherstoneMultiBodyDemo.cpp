/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///experimental support for Featherstone multi body (articulated hierarchies)


///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
//#define START_POS_Y 12
#define START_POS_Y 2
#define START_POS_Z -3

#include "FeatherstoneMultiBodyDemo.h"

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"



#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"
#include "LinearMath/btAabbUtil2.h"

static GLDebugDrawer gDebugDraw;


void FeatherstoneMultiBodyDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();

		btVector3 aabbMin(1,1,1);
		btVector3 aabbMax(2,2,2);

		
	}
		
	renderme(); 

	glFlush();

	swapBuffers();

}



void FeatherstoneMultiBodyDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}





void	FeatherstoneMultiBodyDemo::initPhysics()
{
	//m_idle=true;
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*50.));

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
	m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);//,1,1+2);
	}

	if (1)
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
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

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
										btScalar(3.0*i + start_x),
										btScalar(3.0*k + start_y),
										btScalar(3.0*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					
					m_dynamicsWorld->addRigidBody(body);//,1,1+2);
				}
			}
		}
	}

	
	createFeatherstoneMultiBody(world, 5, btVector3 (20,29.5,-2), true, true);
	
	createFeatherstoneMultiBody(world, 5, btVector3 (0,29.5,-2), false,false);
	
	

}

void FeatherstoneMultiBodyDemo::createFeatherstoneMultiBody(class btMultiBodyDynamicsWorld* world, int numLinks, const btVector3& basePosition,bool isFixedBase, bool usePrismatic)
{
	{
		int n_links = numLinks;
		float mass = 13.5;
		btVector3 inertia(91,344,253);
		bool canSleep = true;//false;
	
		btMultiBody * bod = new btMultiBody(n_links, mass, inertia, isFixedBase, canSleep);
		
		//btQuaternion orn(btVector3(0,0,1),-0.25*SIMD_HALF_PI);//0,0,0,1);
		btQuaternion orn(0,0,0,1);
		bod->setBasePos(basePosition);
		bod->setWorldToBaseRot(orn);
		btVector3 vel(0,0,0);
		bod->setBaseVel(vel);

		{
			
			btVector3 joint_axis_hinge(1,0,0);
			btVector3 joint_axis_prismatic(0,0,1);
			btQuaternion parent_to_child = orn.inverse();
			btVector3 joint_axis_child_prismatic = quatRotate(parent_to_child ,joint_axis_prismatic);
			btVector3 joint_axis_child_hinge = quatRotate(parent_to_child , joint_axis_hinge);
        
			int this_link_num = -1;
			int link_num_counter = 0;

		

			btVector3 pos(0,0,9.0500002);

			btVector3 joint_axis_position(0,0,4.5250001);

			for (int i=0;i<n_links;i++)
			{
				float initial_joint_angle=0.3;
				if (i>0)
					initial_joint_angle = -0.06f;

				const int child_link_num = link_num_counter++;

				if (usePrismatic && i==(n_links-1))
				{
					 bod->setupPrismatic(child_link_num, mass, inertia, this_link_num,
											parent_to_child, joint_axis_child_prismatic, quatRotate(parent_to_child , pos));

				} else
				{
					bod->setupRevolute(child_link_num, mass, inertia, this_link_num,parent_to_child, joint_axis_child_hinge,
										  joint_axis_position,quatRotate(parent_to_child , (pos - joint_axis_position)));
				}
				bod->setJointPos(child_link_num, initial_joint_angle);
				this_link_num = i;
			}

			//add some constraint limit
			if (usePrismatic)
			{
				btMultiBodyConstraint* limit = new btMultiBodyJointLimitConstraint(bod,n_links-1,2,3);
				world->addMultiBodyConstraint(limit);
			}
		}

		//add a collider for the base
		{
			
			btAlignedObjectArray<btQuaternion> world_to_local;
			world_to_local.resize(n_links+1);

			btAlignedObjectArray<btVector3> local_origin;
			local_origin.resize(n_links+1);
			world_to_local[0] = bod->getWorldToBaseRot();
			local_origin[0] = bod->getBasePos();
			//float halfExtents[3]={7.5,0.05,4.5};
			float halfExtents[3]={7.5,0.45,4.5};
			{
			
				float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
				float quat[4]={-world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w()};

			
				if (1)
				{
					btCollisionShape* box = new btBoxShape(btVector3(halfExtents[0],halfExtents[1],halfExtents[2]));
					btRigidBody* body = new btRigidBody(mass,0,box,inertia);
					btMultiBodyLinkCollider* multiBody= new btMultiBodyLinkCollider(bod,-1);

					body->setCollisionShape(box);
					multiBody->setCollisionShape(box);
								
					btTransform tr;
					tr.setIdentity();
					tr.setOrigin(local_origin[0]);
					tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
					body->setWorldTransform(tr);
					multiBody->setWorldTransform(tr);
				
					world->addCollisionObject(multiBody, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
					multiBody->setFriction(1);
					bod->addLinkCollider(multiBody);
				
				}
			}


			for (int i=0;i<bod->getNumLinks();i++)
			{
				const int parent = bod->getParent(i);
				world_to_local[i+1] = bod->getParentToLocalRot(i) * world_to_local[parent+1];
				local_origin[i+1] = local_origin[parent+1] + (quatRotate(world_to_local[i+1].inverse() , bod->getRVector(i)));
			}

		
			for (int i=0;i<bod->getNumLinks();i++)
			{
		
				btVector3 posr = local_origin[i+1];
				float pos[4]={posr.x(),posr.y(),posr.z(),1};
			
				float quat[4]={-world_to_local[i+1].x(),-world_to_local[i+1].y(),-world_to_local[i+1].z(),world_to_local[i+1].w()};

				btCollisionShape* box = new btBoxShape(btVector3(halfExtents[0],halfExtents[1],halfExtents[2]));
				btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bod,i);

				col->setCollisionShape(box);
				btTransform tr;
				tr.setIdentity();
				tr.setOrigin(posr);
				tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
				col->setWorldTransform(tr);
				col->setFriction(1);
				world->addCollisionObject(col,btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);//,2,1);
				
				bod->addLinkCollider(col);
				//app->drawBox(halfExtents, pos,quat);
			}

		}
		world->addMultiBody(bod);
	}

}


void	FeatherstoneMultiBodyDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}
	

void	FeatherstoneMultiBodyDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
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

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




