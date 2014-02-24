
#include "HingeDemo.h"

#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "MyDebugDrawer.h"

#include "btBulletDynamicsCommon.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

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

static float scaling = 1.f;
static float friction = 1.f;

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


HingeDemo::HingeDemo(SimpleOpenGL3App* app, HINGE_CREATION_METHOD hingeMethod)
	:BasicDemo(app),
	m_hingeMethod(hingeMethod)
{
}

struct btMultiBodySettings2
{
	btMultiBodySettings2()
	{
		m_numLinks = 0;
		m_basePosition.setZero();
		m_isFixedBase = true;
		m_usePrismatic = false;
		m_canSleep = true;
		m_createConstraints = false;
		m_disableParentCollision = false;
	}
	int			m_numLinks;
	btVector3	m_basePosition;
	bool		m_isFixedBase;
	bool		m_usePrismatic;
	bool		m_canSleep;
	bool		m_createConstraints;
	bool		m_disableParentCollision;
};


btMultiBody* HingeDemo::createFeatherstoneHinge(class btMultiBodyDynamicsWorld* world, const btMultiBodySettings2& settings)
{
	int curColor=0;
					
	
	int cubeShapeId = m_glApp->registerCubeShape();
		
	int n_links = settings.m_numLinks;
	float mass = 1;
	btVector3 inertia;
	btVector3 halfExtents(1,1,1);
	btCollisionShape* box = new btBoxShape(btVector3(halfExtents[0],halfExtents[1],halfExtents[2])*scaling);
	box->calculateLocalInertia(mass,inertia);
	
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
			
		btVector3 joint_axis_hinge(0,1,0);
		btVector3 joint_axis_prismatic(2,0,2);
		btQuaternion parent_to_child = orn.inverse();
		btVector3 joint_axis_child_prismatic = quatRotate(parent_to_child ,joint_axis_prismatic);
		btVector3 joint_axis_child_hinge = quatRotate(parent_to_child , joint_axis_hinge);
        
		int this_link_num = -1;
		int link_num_counter = 0;

		

		btVector3 pos = btVector3 (0,0,0);//.0500002)*scaling;

		btVector3 joint_axis_position = btVector3 (1,0,1);//-2)*scaling;

		for (int i=0;i<n_links;i++)
		{
			float initial_joint_angle=0;//0.3;
//			if (i>0)
//				initial_joint_angle = -0.06f;

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
#if 0
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
#endif
			
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
		
		{
			
			float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
			float quat[4]={-world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w()};

			
			if (0)
			{
				
				btMultiBodyLinkCollider* col= new btMultiBodyLinkCollider(bod,-1);

				col->setCollisionShape(box);
								
				btTransform tr;
				tr.setIdentity();
				tr.setOrigin(local_origin[0]);
				tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
							col->setWorldTransform(tr);
				
				btVector4 color = colors[curColor++];
				curColor&=3;

				int index = m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,tr.getOrigin(),tr.getRotation(),color,halfExtents*scaling);
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

								
			btVector4 color = colors[curColor++];
			curColor&=3;

			int index = m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,tr.getOrigin(),tr.getRotation(),color,halfExtents*scaling);
			col->setUserIndex(index);



			world->addCollisionObject(col,short(btBroadphaseProxy::DefaultFilter),short(btBroadphaseProxy::AllFilter));
			
			bod->getLink(i).m_collider=col;
			//app->drawBox(halfExtents, pos,quat);
		}

	}
	world->addMultiBody(bod);

	return bod;
}

void	HingeDemo::initPhysics()
{
	m_config = new btDefaultCollisionConfiguration;
	m_dispatcher = new btCollisionDispatcher(m_config);
	m_bp = new btDbvtBroadphase();

	
	switch (m_hingeMethod)
	{
		case FEATHERSTONE_HINGE:
		{
			btMultiBodyConstraintSolver* solver = new btMultiBodyConstraintSolver();
			btMultiBodyDynamicsWorld* world = new btMultiBodyDynamicsWorld(m_dispatcher,m_bp,solver,m_config);
			m_dynamicsWorld = world;
			btMultiBodySettings2 settings;
			settings.m_basePosition.setValue(0,0,0);
			settings.m_numLinks = 1;
			btMultiBody* multibody = createFeatherstoneHinge(world, settings);
			
			break;
		}
		case DANTZIG_HINGE:
		{
			btDantzigSolver* mlcp = new btDantzigSolver();
			m_solver = new btMLCPSolver(mlcp);
			break;
		}
		case LEMKE_HINGE:
		{
			btLemkeSolver* mlcp = new btLemkeSolver();
			m_solver = new btMLCPSolver(mlcp);
			
			break;
		}

		case PGS_HINGE:
		{
			btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
			m_solver = new btMLCPSolver(mlcp);
			break;
		}
		case SI_HINGE:
		{
			m_solver = new btSequentialImpulseConstraintSolver();
			break;
		}
		case INERTIA_HINGE:
		{
			m_solver = new btSequentialImpulseConstraintSolver();
			break;
		}
		default:
		{
			
		}
	}
	
	int cubeShapeId = m_glApp->registerCubeShape();

	
	if (m_hingeMethod!=FEATHERSTONE_HINGE)
	{
		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_bp,m_solver,m_config);
	
		
		
		m_dynamicsWorld->setDebugDrawer(new MyDebugDrawer(m_glApp));
		


		
		if (1)
		{
			btVector4 color(0,1,0,1);
			btTransform startTransform;
			startTransform.setIdentity();
			startTransform.setOrigin(btVector3(0,0,0));

		//startTransform.setRotation(btQuaternion(btVector3(0,1,0),0.2*SIMD_HALF_PI));
			btVector3 halfExtents(1,1,1);
			int index = m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,startTransform.getOrigin(),startTransform.getRotation(),color,halfExtents);
			btBoxShape* box1 = new btBoxShape(halfExtents);
			btCompoundShape* box = new btCompoundShape();
			btTransform shiftTrans;shiftTrans.setIdentity();
			btVector3 centerOfMassShift(0,0,0);//1.5,1.5,1.5);

			if (m_hingeMethod==INERTIA_HINGE)
			{
				centerOfMassShift.setValue(-1,1,-1);
			}
			
			shiftTrans.setOrigin(centerOfMassShift);
	//		shiftTrans.setRotation(btQuaternion(btVector3(0,1,0),0.2*SIMD_HALF_PI));
			box->addChildShape(shiftTrans,box1);

			float mass = 1.f;
			btVector3 localInertia;
			box->calculateLocalInertia(mass,localInertia);
			
			if (m_hingeMethod==INERTIA_HINGE)
			{
				//localInertia[0] = 0;
				//localInertia[1] = 0;
			}
			
			btDefaultMotionState* motionState = new btDefaultMotionState();
			startTransform.setOrigin(-centerOfMassShift);

			motionState->m_centerOfMassOffset = shiftTrans;
			motionState->setWorldTransform(startTransform);

			btRigidBody* body = new btRigidBody(mass,motionState,box,localInertia);
			body->setUserIndex(index);
			m_dynamicsWorld->addRigidBody(body);
			m_dynamicsWorld->getSolverInfo().m_splitImpulse = false;//true;
	//		m_dynamicsWorld->getSolverInfo().m_erp2 = 1;
	//		m_dynamicsWorld->getSolverInfo().m_erp = 1;
			m_dynamicsWorld->getSolverInfo().m_numIterations = 10;
			
			if (m_hingeMethod!=INERTIA_HINGE)
			{
				btVector3 pivotInA(1,0,1);
				btVector3 axisInA(0,1,0);
				btHingeConstraint* hinge = new btHingeConstraint(*body,pivotInA,axisInA);
				hinge->setOverrideNumSolverIterations(10);
				m_dynamicsWorld->addConstraint(hinge);
			} else
			{
				body->setLinearFactor(btVector3(0,0,0));

				btVector3 ax = btVector3(0,1,0);

				body->setAngularFactor(ax);

			}
		}
		
		
	}

	
	if (1)
	{
		btVector4 color(0,0,1,1);
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0,2,0));
		btVector3 halfExtents(1,1,1);
		int index = m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,startTransform.getOrigin(),startTransform.getRotation(),color,halfExtents);
		btBoxShape* box = new btBoxShape(halfExtents);
		
		float mass = 1.f;
		btVector3 localInertia;
		box->calculateLocalInertia(mass,localInertia);
		btRigidBody* body = new btRigidBody(mass,0,box,localInertia);
		body->getMotionState();
		body->setWorldTransform(startTransform);
		body->setUserIndex(index);
		body->setAngularVelocity(btVector3(0,1,0));
		body->setActivationState(DISABLE_DEACTIVATION);
		m_dynamicsWorld->addRigidBody(body);
	}

	
	m_glApp->m_instancingRenderer->writeTransforms();
}


void	HingeDemo::renderScene()
{
//	m_glApp->drawGrid();
//	m_glApp->drawText("test",10,10);
	//sync graphics -> physics world transforms
	
	{
		
		for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
		{
			btRigidBody* body =  btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[i]);
			const btDefaultMotionState* state = (btDefaultMotionState*)(body ? body->getMotionState() : 0);

			if (state)
			{
				btTransform tr = state->m_graphicsWorldTrans;
		
				btVector3 pos = tr.getOrigin();
				btQuaternion orn = tr.getRotation();
				m_glApp->m_instancingRenderer->writeSingleInstanceTransformToCPU(pos,orn,i);

			} else
			{
				btVector3 pos = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().getOrigin();
				btQuaternion orn = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().getRotation();
				m_glApp->m_instancingRenderer->writeSingleInstanceTransformToCPU(pos,orn,i);
			}
		}
		m_glApp->m_instancingRenderer->writeTransforms();
	}


	static bool debugRender = false;
	{
		m_dynamicsWorld->debugDrawWorld();
	}
	debugRender  = !debugRender ;
	{
		m_glApp->m_instancingRenderer->renderScene();
	}

//	if (debugRender)
	
// else
	
}

void	HingeDemo::exitPhysics()
{
	Bullet2RigidBodyDemo::exitPhysics();

}