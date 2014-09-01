

#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5
static float scaling = 1.f;
static float friction = 1.;

#include "BulletMultiBodyDemos.h"


#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Vector3.h"
#include "assert.h"
#include <stdio.h>

#include "btBulletDynamicsCommon.h"

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

#define CONSTRAINT_DEBUG_SIZE 0.2f
static bool prevCanSleep = false;

struct GraphicsVertex
{
	float pos[4];
	float normal[3];
	float texcoord[2];
};

static btVector4 colors[4] =
{
	btVector4(1,0,0,1),
	btVector4(0,1,0,1),
	btVector4(0,1,1,1),
	btVector4(1,1,0,1),
};



Bullet2MultiBodyDemo::Bullet2MultiBodyDemo(SimpleOpenGL3App* app)
:m_glApp(app),
m_pickedBody(0),
m_pickedConstraint(0),
m_pickingMultiBodyPoint2Point(0)

{
	m_collisionConfiguration = 0;
	m_dispatcher = 0;
	m_broadphase = 0;
	m_solver = 0;
	m_dynamicsWorld = 0;
}
void Bullet2MultiBodyDemo::initPhysics()
{
	m_collisionConfiguration = new btDefaultCollisionConfiguration;
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();
	m_solver = new btMultiBodyConstraintSolver();
	m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
}

void Bullet2MultiBodyDemo::exitPhysics()
{
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

Bullet2MultiBodyDemo::~Bullet2MultiBodyDemo()
{
	btAssert(m_collisionConfiguration == 0);
	btAssert(m_dispatcher == 0);
	btAssert(m_broadphase == 0);
	btAssert(m_solver == 0);
	btAssert(m_dynamicsWorld == 0);
}



btVector3	Bullet2MultiBodyDemo::getRayTo(int x,int y)
{
	if (!m_glApp->m_instancingRenderer)
	{
		btAssert(0);
		return btVector3(0,0,0);
	}

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = b3Scalar(2.0) * b3Atan(tanFov);

	btVector3 camPos,camTarget;
	m_glApp->m_instancingRenderer->getCameraPosition(camPos);
	m_glApp->m_instancingRenderer->getCameraTargetPosition(camTarget);

	btVector3	rayFrom = camPos;
	btVector3 rayForward = (camTarget-camPos);
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward*= farPlane;

	btVector3 rightOffset;
	btVector3 m_cameraUp=btVector3(0,1,0);
	btVector3 vertical = m_cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);


	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	b3Scalar aspect;
	float width = m_glApp->m_instancingRenderer->getScreenWidth();
	float height = m_glApp->m_instancingRenderer->getScreenHeight();

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

	
bool	Bullet2MultiBodyDemo::mouseMoveCallback(float x,float y)
{
//		if (m_data->m_altPressed!=0 || m_data->m_controlPressed!=0)
//		return false;
		
	if (m_pickedBody  && m_pickedConstraint)
	{
		btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickedConstraint);
		if (pickCon)
		{
			//keep it at the same picking distance
			btVector3 newRayTo = getRayTo(x,y);
			btVector3 rayFrom;
//			btVector3 oldPivotInB = pickCon->getPivotInB();
			btVector3 newPivotB;
			m_glApp->m_instancingRenderer->getCameraPosition(rayFrom);
			btVector3 dir = newRayTo-rayFrom;
			dir.normalize();
			dir *= m_oldPickingDist;

			newPivotB = rayFrom + dir;
			pickCon->setPivotB(newPivotB);
		}
	}
	if (m_pickingMultiBodyPoint2Point)
	{
		//keep it at the same picking distance

		btVector3 newRayTo = getRayTo(x,y);
		btVector3 rayFrom;
	//	btVector3 oldPivotInB = m_pickingMultiBodyPoint2Point->getPivotInB();
		btVector3 newPivotB;
		btVector3 camPos;
		m_glApp->m_instancingRenderer->getCameraPosition(camPos);
		rayFrom = camPos;
		btVector3 dir = newRayTo-rayFrom;
		dir.normalize();
		dir *= m_oldPickingDist;

		newPivotB = rayFrom + dir;
			
		m_pickingMultiBodyPoint2Point->setPivotInB(newPivotB);
	}
		
	return false;
}
bool	Bullet2MultiBodyDemo::mouseButtonCallback(int button, int state, float x, float y)
{

	if (state==1)
	{
		if(button==0)// && (m_data->m_altPressed==0 && m_data->m_controlPressed==0))
		{
			btVector3 camPos;
			m_glApp->m_instancingRenderer->getCameraPosition(camPos);

			btVector3 rayFrom = camPos;
			btVector3 rayTo = getRayTo(x,y);

			btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);
			m_dynamicsWorld->rayTest(rayFrom,rayTo,rayCallback);
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
						m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
						//printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
						btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
						btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,localPivot);
						m_dynamicsWorld->addConstraint(p2p,true);
						m_pickedConstraint = p2p;
						btScalar mousePickClamping = 10.f;
						p2p->m_setting.m_impulseClamp = mousePickClamping;
						//very weak constraint for picking
						p2p->m_setting.m_tau = 0.001f;
					}
				} else
				{
					btMultiBodyLinkCollider* multiCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(rayCallback.m_collisionObject);
					if (multiCol && multiCol->m_multiBody)
					{
						
						prevCanSleep = multiCol->m_multiBody->getCanSleep();
						multiCol->m_multiBody->setCanSleep(false);

						btVector3 pivotInA = multiCol->m_multiBody->worldPosToLocal(multiCol->m_link, pickPos);

						btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(multiCol->m_multiBody,multiCol->m_link,0,pivotInA,pickPos);
						//if you add too much energy to the system, causing high angular velocities, simulation 'explodes'
						//see also http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=949
						//so we try to avoid it by clamping the maximum impulse (force) that the mouse pick can apply
						//it is not satisfying, hopefully we find a better solution (higher order integrator, using joint friction using a zero-velocity target motor with limited force etc?)

						p2p->setMaxAppliedImpulse(2*scaling);
		
						btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_dynamicsWorld;
						world->addMultiBodyConstraint(p2p);
						m_pickingMultiBodyPoint2Point =p2p; 
					}
				}


//					pickObject(pickPos, rayCallback.m_collisionObject);
				m_oldPickingPos = rayTo;
				m_hitPos = pickPos;
				m_oldPickingDist  = (pickPos-rayFrom).length();
//					printf("hit !\n");
			//add p2p
			}
				
		}
	} else
	{
		if (button==0)
		{
			if (m_pickedConstraint)
			{
				m_dynamicsWorld->removeConstraint(m_pickedConstraint);
				delete m_pickedConstraint;
				m_pickedConstraint=0;
				m_pickedBody = 0;
			}

			if (m_pickingMultiBodyPoint2Point)
			{
				m_pickingMultiBodyPoint2Point->getMultiBodyA()->setCanSleep(prevCanSleep);
				btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_dynamicsWorld;
				world->removeMultiBodyConstraint(m_pickingMultiBodyPoint2Point);
				delete m_pickingMultiBodyPoint2Point;
				m_pickingMultiBodyPoint2Point = 0;
			}
			//remove p2p
		}
	}

	//printf("button=%d, state=%d\n",button,state);
	return false;
}













FeatherstoneDemo1::FeatherstoneDemo1(SimpleOpenGL3App* app)
:Bullet2MultiBodyDemo(app)
{
}
FeatherstoneDemo1::~FeatherstoneDemo1()
{
}



btMultiBody* FeatherstoneDemo1::createFeatherstoneMultiBody(class btMultiBodyDynamicsWorld* world, const btMultiBodySettings& settings)
{
	int curColor=0;
					
	
	int cubeShapeId = m_glApp->registerCubeShape();
		
	int n_links = settings.m_numLinks;
	float mass = 13.5*scaling;
	btVector3 inertia = btVector3 (91,344,253)*scaling*scaling;
	
	
	bool isMultiDof = false;
	btMultiBody * bod = new btMultiBody(n_links, mass, inertia, settings.m_isFixedBase, settings.m_canSleep, isMultiDof);
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
						btMultiBodyJointMotor* con = new btMultiBodyJointMotor(bod,i,0,0,500000); 
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
		btVector4 halfExtents(7.5,0.45,4.5,1);
    	{
			
		//	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
			float quat[4]={-world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w()};

			
			if (1)
			{
				btCollisionShape* box = new btBoxShape(btVector3(halfExtents[0],halfExtents[1],halfExtents[2])*scaling);
				btMultiBodyLinkCollider* col= new btMultiBodyLinkCollider(bod,-1);

				col->setCollisionShape(box);
								
				btTransform tr;
				tr.setIdentity();
				tr.setOrigin(local_origin[0]);
				tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
							col->setWorldTransform(tr);
				
				btVector4 color = colors[curColor++];
				curColor&=3;

				int index = m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,tr.getOrigin(),tr.getRotation(),color,halfExtents);
				col->setUserIndex(index);




				world->addCollisionObject(col,short(btBroadphaseProxy::DefaultFilter),short(btBroadphaseProxy::AllFilter));
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
			//float pos[4]={posr.x(),posr.y(),posr.z(),1};
			
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

								
			btVector4 color = colors[curColor++];
			curColor&=3;

			int index = m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,tr.getOrigin(),tr.getRotation(),color,halfExtents);
			col->setUserIndex(index);



			world->addCollisionObject(col,short(btBroadphaseProxy::DefaultFilter),short(btBroadphaseProxy::AllFilter));
			
			bod->getLink(i).m_collider=col;
			//app->drawBox(halfExtents, pos,quat);
		}

	}
	world->addMultiBody(bod);

	return bod;
}
	
void FeatherstoneDemo1::addColliders_testMultiDof(btMultiBody *pMultiBody, btMultiBodyDynamicsWorld *pWorld, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents)
{
}
void FeatherstoneDemo1::addBoxes_testMultiDof()
{
}

void FeatherstoneDemo1::createGround()
{
	//create ground
	int cubeShapeId = m_glApp->registerCubeShape();
	//float pos[]={0,0,0};
	//float orn[]={0,0,0,1};
		

	{
		btVector4 color(0.3,0.3,1,1);
		btVector4 halfExtents(50,50,50,1);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-50,0));
		btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(halfExtents[0]),btScalar(halfExtents[1]),btScalar(halfExtents[2])));
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

			int index = m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,groundTransform.getOrigin(),groundTransform.getRotation(),color,halfExtents);
			body ->setUserIndex(index);

			//add the body to the dynamics world
			m_dynamicsWorld->addRigidBody(body);
		}
	}
}

void	FeatherstoneDemo1::initPhysics()
{

	Bullet2MultiBodyDemo::initPhysics();

	createGround();

	btMultiBodySettings settings;
	settings.m_isFixedBase = false;
	settings.m_basePosition.setValue(0,10,0);
	settings.m_numLinks = 10;
	//btMultiBody* mb = 
	createFeatherstoneMultiBody(m_dynamicsWorld,settings);


	m_glApp->m_instancingRenderer->writeTransforms();
}



void	FeatherstoneDemo1::exitPhysics()
{
	Bullet2MultiBodyDemo::exitPhysics();
}

void	FeatherstoneDemo1::renderScene()
{
	//sync graphics -> physics world transforms
	{
		for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
		{
			btCollisionObject* col = m_dynamicsWorld->getCollisionObjectArray()[i];

			btVector3 pos = col->getWorldTransform().getOrigin();
			btQuaternion orn = col->getWorldTransform().getRotation();
			int index = col->getUserIndex();
            if (index>=0)
            {
                m_glApp->m_instancingRenderer->writeSingleInstanceTransformToCPU(pos,orn,index);
            }
		}
		m_glApp->m_instancingRenderer->writeTransforms();
	}

	m_glApp->m_instancingRenderer->renderScene();
}

void	FeatherstoneDemo1::physicsDebugDraw()
{
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->debugDrawWorld();	
	}
}

void	FeatherstoneDemo1::stepSimulation(float deltaTime)
{
	m_dynamicsWorld->stepSimulation(deltaTime);//,0);
//		CProfileManager::dumpAll();
	/*
	for (int i=0;i<m_dynamicsWorld->getDispatcher()->getNumManifolds();i++)
	{
		btPersistentManifold* contact = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		for (int c=0;c<contact->getNumContacts();c++)
		{
			btManifoldPoint& pt = contact->getContactPoint(c);
			btScalar dist = pt.getDistance();
			if (dist< contact->getContactProcessingThreshold())
			{
				printf("normalImpulse[%d.%d] = %f\n",i,c,pt.m_appliedImpulse);
				
			} else
			{
				printf("?\n");
			}
		}
	}
	*/
}



FeatherstoneDemo2::FeatherstoneDemo2(SimpleOpenGL3App* app)
:FeatherstoneDemo1(app)
{
}

FeatherstoneDemo2::~FeatherstoneDemo2()
{
}



class RagDoll2
{
	enum
	{
		BODYPART_PELVIS = 0,
		BODYPART_SPINE,
		BODYPART_HEAD,
		
		BODYPART_LEFT_UPPER_LEG,
		BODYPART_LEFT_LOWER_LEG,
		
		BODYPART_RIGHT_UPPER_LEG,
		BODYPART_RIGHT_LOWER_LEG,
		
		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,
		
		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,
		
		BODYPART_COUNT
	};
	
	enum
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,
		
		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,
		
		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,
		
		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,
		
		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,
		
		JOINT_COUNT
	};
	
	btDynamicsWorld* m_ownerWorld;
	btConvexShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];
	SimpleOpenGL3App* m_app;//used to create graphics shapes
	
	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btConvexShape* shape, const btVector3& color=btVector3(1,0,0) )
	{
		bool isDynamic = (mass != 0.f);
		
		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);
		
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		
		m_ownerWorld->addRigidBody(body);
		
		
		btVector3 scaling(1,1,1);
		
		btShapeHull* hull = new btShapeHull(shape);
		hull->buildHull(0.01);
		
		{
		//	int strideInBytes = 9*sizeof(float);
			int numVertices = hull->numVertices();
			int numIndices =hull->numIndices();
			
			btAlignedObjectArray<GraphicsVertex> gvertices;
			
			for (int i=0;i<numVertices;i++)
			{
				GraphicsVertex vtx;
				btVector3 pos =hull->getVertexPointer()[i];
				vtx.pos[0] = pos.x();
				vtx.pos[1] = pos.y();
				vtx.pos[2] = pos.z();
				vtx.pos[3] = 1.f;
				pos.normalize();
				vtx.normal[0] =pos.x();
				vtx.normal[1] =pos.y();
				vtx.normal[2] =pos.z();
				vtx.texcoord[0] = 0.5f;
				vtx.texcoord[1] = 0.5f;
				gvertices.push_back(vtx);
			}
			
			btAlignedObjectArray<int> indices;
			for (int i=0;i<numIndices;i++)
				indices.push_back(hull->getIndexPointer()[i]);
			
			int shapeId = m_app->m_instancingRenderer->registerShape(&gvertices[0].pos[0],numVertices,&indices[0],numIndices);
			
			int index = m_app->m_instancingRenderer->registerGraphicsInstance(shapeId,body->getWorldTransform().getOrigin(),body->getWorldTransform().getRotation(),color,scaling);
			body ->setUserIndex(index);
		}
		delete hull;
		
		
		
		return body;
	}
	
public:
	RagDoll2 (btDynamicsWorld* ownerWorld, const btVector3& positionOffset,SimpleOpenGL3App* app)
	: m_ownerWorld (ownerWorld),
	m_app(app)
	{
		
		// Setup the geometry
		m_shapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
		m_shapes[BODYPART_SPINE] = new btCapsuleShape(btScalar(0.15), btScalar(0.28));
		m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(0.10), btScalar(0.05));
		m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
		m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
		

		// Setup all the rigid bodies
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);
		
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
		//m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS], btVector3(0,1,0));
		
		//		btMultiBody * bod = new btMultiBody(n_links, mass, inertia, settings.m_isFixedBase, settings.m_canSleep);
		int n_links = 0;
		float mass = 1.f;
		btVector3 localInertia;
		m_shapes[BODYPART_PELVIS]->calculateLocalInertia(mass,localInertia);
		
		bool isFixedBase = true;
		bool canSleep = true;
		bool isMultiDof = false;
		btMultiBody * bod = new btMultiBody(n_links, mass, localInertia, isFixedBase, canSleep, isMultiDof);
		
		btTransform tr;
		tr = offset*transform;
		
		
		bod->setBasePos(tr.getOrigin());
		bod->setWorldToBaseRot(tr.getRotation());
		btVector3 vel(0,0,0);
		bod->setBaseVel(vel);
		
#if 0
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.2), btScalar(0.)));
		m_bodies[BODYPART_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_SPINE],btVector3(0,0,1));

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.6), btScalar(0.)));
		m_bodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_HEAD]);
		
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_LEG]);
		
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_LEG]);
		
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_LEG]);
		
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_LEG]);
		
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);
		
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);
		
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);
		
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);
		
		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}
#endif
#if 0
		// Now setup the constraints
		btHingeConstraint* hingeC;
		btConeTwistConstraint* coneC;
		
		btTransform localA, localB;
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
		hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
		m_joints[JOINT_PELVIS_SPINE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);
		
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_2); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
		m_joints[JOINT_SPINE_HEAD] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);
		
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,-M_PI_4*5); localA.setOrigin(btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,-M_PI_4*5); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_LEFT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);
		
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_4); localA.setOrigin(btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_4); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_RIGHT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);
		
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI); localA.setOrigin(btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_joints[JOINT_LEFT_SHOULDER] = coneC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
		//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);
		
		
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		m_joints[JOINT_RIGHT_SHOULDER] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);
		
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
		//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
#endif
		
	}
	
	virtual	~RagDoll2 ()
	{
		//int i;
		/*
		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}
		 */
		/*
		// Remove all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			
			delete m_bodies[i]->getMotionState();
			
			delete m_bodies[i]; m_bodies[i] = 0;
			delete m_shapes[i]; m_shapes[i] = 0;
		}
		 */
	}
};



void	FeatherstoneDemo2::initPhysics()
{
	Bullet2MultiBodyDemo::initPhysics();

	createGround();

/*	btMultiBodySettings settings;
	settings.m_isFixedBase = false;
	settings.m_basePosition.setValue(0,20,0);
	settings.m_numLinks = 3;
	settings.m_usePrismatic = true;
	btMultiBody* mb = createFeatherstoneMultiBody(m_dynamicsWorld,settings);
*/
//	btVector3 offset(0,2,0);
	//RagDoll2* doll = new RagDoll2(m_dynamicsWorld,offset,m_glApp);

	
	m_glApp->m_instancingRenderer->writeTransforms();
}






