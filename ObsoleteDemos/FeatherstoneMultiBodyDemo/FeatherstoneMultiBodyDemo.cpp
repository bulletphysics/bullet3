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
float friction = 1.;


//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)


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
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"


#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"
#include "LinearMath/btAabbUtil2.h"

static GLDebugDrawer gDebugDraw;
//btVector3 scaling(0.1,0.1,0.1);
float scaling = 0.4f;


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

	setCameraDistance(btScalar(100.*scaling));
	this->m_azi = 130;
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
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,00));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	if (1)
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = new btBoxShape(btVector3(1,1,1));
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
					startTransform.setOrigin(btVector3(
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

	btMultiBodySettings settings;
	settings.m_numLinks = 2;
	settings.m_basePosition =  btVector3 (60,29.5,-2)*scaling;
	settings.m_isFixedBase = false;
	settings.m_disableParentCollision = true;//the self-collision has conflicting/non-resolvable contact normals
	
	settings.m_usePrismatic = true;
	settings.m_canSleep = true;
	settings.m_createConstraints = true;

	//btMultiBody* createFeatherstoneMultiBody(class btMultiBodyDynamicsWorld* world, int numLinks, const btVector3& basePosition,bool isFixedBase, bool usePrismatic, bool canSleep, bool createConstraints);

	btMultiBody* mbA = createFeatherstoneMultiBody(world, settings);
		
	settings.m_numLinks = 10;
	settings.m_basePosition = btVector3 (0,29.5,-settings.m_numLinks*4.f);
	settings.m_isFixedBase = true;
	settings.m_usePrismatic = false;

	btMultiBody* mbB = createFeatherstoneMultiBody(world, settings);
	settings.m_basePosition = btVector3 (-20*scaling,29.5*scaling,-settings.m_numLinks*4.f*scaling);
	settings.m_isFixedBase = false;
	btMultiBody* mbC = createFeatherstoneMultiBody(world, settings);

	settings.m_basePosition = btVector3 (-20,9.5,-settings.m_numLinks*4.f);
	settings.m_isFixedBase = true;
	settings.m_usePrismatic = true;
	settings.m_disableParentCollision = true;
	
	btMultiBody* mbPrim= createFeatherstoneMultiBody(world, settings);

	//btMultiBody* mbB = createFeatherstoneMultiBody(world, 15, btVector3 (0,29.5,-2), false,true,true);
#if 0
	if (0)//!useGroundShape && i==4)
	{
		//attach two multibody using a point2point constraint

		//btVector3 pivotInAworld(0,20,46);
		btVector3 pivotInAworld(-0.3,29,-3.5);

		int linkA = -1;
		int linkB = -1;

		btVector3 pivotInAlocal = mbA->worldPosToLocal(linkA, pivotInAworld);
		btVector3 pivotInBlocal = mbB->worldPosToLocal(linkB, pivotInAworld);
		btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(mbA,linkA,mbB,linkB,pivotInAlocal,pivotInBlocal);
		world->addMultiBodyConstraint(p2p);
	}
#endif
	bool testRemoveLinks = false;
	if (testRemoveLinks)
	{
		while (mbA->getNumLinks())
		{
			btCollisionObject* col = mbA->getLink(mbA->getNumLinks()-1).m_collider;
			m_dynamicsWorld->removeCollisionObject(col);
			delete col;
			mbA->setNumLinks(mbA->getNumLinks()-1);
		}
	}
	
	if (1)//useGroundShape
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
		m_dynamicsWorld->addRigidBody(body,1,1+2);//,1,1+2);
	}


}

btMultiBody* FeatherstoneMultiBodyDemo::createFeatherstoneMultiBody(class btMultiBodyDynamicsWorld* world, const btMultiBodySettings& settings)
{
	
	int n_links = settings.m_numLinks;
	float mass = 13.5*scaling;
	btVector3 inertia = btVector3 (91,344,253)*scaling*scaling;
	
	
	btMultiBody * bod = new btMultiBody(n_links, mass, inertia, settings.m_isFixedBase, settings.m_canSleep);
//		bod->setHasSelfCollision(false);

	//btQuaternion orn(btVector3(0,0,1),-0.25*SIMD_HALF_PI);//0,0,0,1);
	btQuaternion orn(0,0,0,1);
	bod->setBasePos(settings.m_basePosition);
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

		

		btVector3 pos = btVector3 (0,0,9.0500002)*scaling;

		btVector3 joint_axis_position = btVector3 (0,0,4.5250001)*scaling;

		for (int i=0;i<n_links;i++)
		{
			float initial_joint_angle=0.3;
			if (i>0)
				initial_joint_angle = -0.06f;

			const int child_link_num = link_num_counter++;

			

			if (settings.m_usePrismatic)// && i==(n_links-1))
			{
					bod->setupPrismatic(child_link_num, mass, inertia, this_link_num,
						parent_to_child, joint_axis_child_prismatic, quatRotate(parent_to_child , pos),settings.m_disableParentCollision);

			} else
			{
				bod->setupRevolute(child_link_num, mass, inertia, this_link_num,parent_to_child, joint_axis_child_hinge,
										joint_axis_position,quatRotate(parent_to_child , (pos - joint_axis_position)),settings.m_disableParentCollision);
			}
			bod->setJointPos(child_link_num, initial_joint_angle);
			this_link_num = i;
		
			if (0)//!useGroundShape && i==4)
			{
				btVector3 pivotInAworld(0,20,46);
				btVector3 pivotInAlocal = bod->worldPosToLocal(i, pivotInAworld);
				btVector3 pivotInBworld = pivotInAworld;
				btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(bod,i,&btTypedConstraint::getFixedBody(),pivotInAlocal,pivotInBworld);
				world->addMultiBodyConstraint(p2p);
			}
			//add some constraint limit
			if (settings.m_usePrismatic)
			{
	//			btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(bod,n_links-1,2,3);
			
				if (settings.m_createConstraints)
				{	
					btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(bod,i,-1,1);
					world->addMultiBodyConstraint(con);
				}
			
			} else
			{
				if (settings.m_createConstraints)
				{	
					if (1)
					{
						btMultiBodyJointMotor* con = new btMultiBodyJointMotor(bod,i,0,500000); 
						world->addMultiBodyConstraint(con);
					}

					btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(bod,i,-1,1);
					world->addMultiBodyConstraint(con);
				}

			}
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
				btCollisionShape* box = new btBoxShape(btVector3(halfExtents[0],halfExtents[1],halfExtents[2])*scaling);
				btRigidBody* body = new btRigidBody(mass,0,box,inertia);
				btMultiBodyLinkCollider* col= new btMultiBodyLinkCollider(bod,-1);

				body->setCollisionShape(box);
				col->setCollisionShape(box);
								
				btTransform tr;
				tr.setIdentity();
				tr.setOrigin(local_origin[0]);
				tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
				body->setWorldTransform(tr);
				col->setWorldTransform(tr);
				
				world->addCollisionObject(col, 2,1+2);
				col->setFriction(friction);
				bod->setBaseCollider(col);
				
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

			btCollisionShape* box = new btBoxShape(btVector3(halfExtents[0],halfExtents[1],halfExtents[2])*scaling);
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bod,i);

			col->setCollisionShape(box);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(posr);
			tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
			col->setWorldTransform(tr);
			col->setFriction(friction);
			world->addCollisionObject(col,2,1+2);
			
			bod->getLink(i).m_collider=col;
			//app->drawBox(halfExtents, pos,quat);
		}

	}
	world->addMultiBody(bod);

	return bod;
}

extern btScalar gOldPickingDist;
void	FeatherstoneMultiBodyDemo::mouseMotionFunc(int x,int y)
{
	if (m_pickingMultiBodyPoint2Point)
	{
		//keep it at the same picking distance

		btVector3 newRayTo = getRayTo(x,y);
		btVector3 rayFrom;
		btVector3 oldPivotInB = m_pickingMultiBodyPoint2Point->getPivotInB();
		btVector3 newPivotB;
		if (m_ortho)
		{
			newPivotB = oldPivotInB;
			newPivotB.setX(newRayTo.getX());
			newPivotB.setY(newRayTo.getY());
		} else
		{
			rayFrom = m_cameraPosition;
			btVector3 dir = newRayTo-rayFrom;
			dir.normalize();
			dir *= gOldPickingDist;

			newPivotB = rayFrom + dir;
		}
		m_pickingMultiBodyPoint2Point->setPivotInB(newPivotB);
	}
	DemoApplication::mouseMotionFunc(x,y);
}

void FeatherstoneMultiBodyDemo::removePickingConstraint()
{
	if (m_pickingMultiBodyPoint2Point)
	{
		m_pickingMultiBodyPoint2Point->getMultiBodyA()->setCanSleep(true);
		
		btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_dynamicsWorld;
		world->removeMultiBodyConstraint(m_pickingMultiBodyPoint2Point);
		delete m_pickingMultiBodyPoint2Point;
		m_pickingMultiBodyPoint2Point = 0;
	}
	
	DemoApplication::removePickingConstraint();

}

void FeatherstoneMultiBodyDemo::pickObject(const btVector3& pickPos, const class btCollisionObject* hitObj)
{
	btVector3 pivotInA(0,0,0);
	btMultiBodyLinkCollider* multiCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(hitObj);
	if (multiCol && multiCol->m_multiBody)
	{
		multiCol->m_multiBody->setCanSleep(false);

		btVector3 pivotInA = multiCol->m_multiBody->worldPosToLocal(multiCol->m_link, pickPos);

		btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(multiCol->m_multiBody,multiCol->m_link,0,pivotInA,pickPos);
		//if you add too much energy to the system, causing high angular velocities, simulation 'explodes'
		//see also http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=949
		//so we try to avoid it by clamping the maximum impulse (force) that the mouse pick can apply
		//it is not satisfying, hopefully we find a better solution (higher order integrator, using joint friction using a zero-velocity target motor with limited force etc?)

		p2p->setMaxAppliedImpulse(200*scaling);
		
		btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_dynamicsWorld;
		world->addMultiBodyConstraint(p2p);
		m_pickingMultiBodyPoint2Point =p2p; 
	} else
	{
		DemoApplication::pickObject(pickPos,hitObj);
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




