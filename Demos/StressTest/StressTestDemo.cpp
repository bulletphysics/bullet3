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
#include "StressTestDemo.h"

#define SHOW_NUM_DEEP_PENETRATIONS

#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btDefaultMotionState.h"
#include "BulletCollision/BroadphaseCollision/btMultiSapBroadphase.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"

//#define REGISTER_BOX_BOX 1
#ifdef REGISTER_BOX_BOX
#include "../Extras/AlternativeCollisionAlgorithms/BoxBoxCollisionAlgorithm.h"
#endif //REGISTER_BOX_BOX


//#define SHOW_MEMORY_DETAILS 1

//#ifdef SHOW_MEMORY_DETAILS
//	#include "LinearMath/btGenericPoolAllocator.h"
//#endif

/// Including GIMPACT here



#include "BMF_Api.h"

#include "GLDebugDrawer.h"

#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"


#ifdef SHOW_NUM_DEEP_PENETRATIONS
extern int gNumDeepPenetrationChecks;
extern int gNumGjkChecks;
#endif //



GLDebugDrawer	debugDrawer;
//Real			dts = 0.000001f;
btScalar			dts = 1.0 / 60.0;


///**************************************************************************************
///	GIMPACT Test Demo made by DevO
///
///**************************************************************************************



//------------------------------------------------------------------------------
///User can override this material combiner by implementing gContactAddedCallback and setting body0->m_collisionFlags |= btCollisionObject::customMaterialCallback;
inline btScalar	calculateCombinedFriction(float friction0,float friction1)
{
	btScalar friction = friction0 * friction1;

	const btScalar MAX_FRICTION  = 10.f;
	if (friction < -MAX_FRICTION)
		friction = -MAX_FRICTION;
	if (friction > MAX_FRICTION)
		friction = MAX_FRICTION;
	return friction;

}

//------------------------------------------------------------------------------
inline btScalar	calculateCombinedRestitution(float restitution0,float restitution1)
{
	return restitution0 * restitution1;
}


//------------------------------------------------------------------------------
bool CustomMaterialCombinerCallback(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
{

	float friction0 = colObj0->getFriction();
	float friction1 = colObj1->getFriction();
	float restitution0 = colObj0->getRestitution();
	float restitution1 = colObj1->getRestitution();

	if (colObj0->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)
	{
		friction0 = 1.0;//partId0,index0
		restitution0 = 0.f;
	}
	if (colObj1->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)
	{
		if (index1&1)
		{
			friction1 = 1.0f;//partId1,index1
		} else
		{
			friction1 = 0.f;
		}
		restitution1 = 0.f;
	}

	cp.m_combinedFriction = calculateCombinedFriction(friction0,friction1);
	cp.m_combinedRestitution = calculateCombinedRestitution(restitution0,restitution1);

	//this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
	return true;
}

extern ContactAddedCallback		gContactAddedCallback;



//################################## main #####################################
int main(int argc,char** argv)
{
	//gContactAddedCallback = CustomMaterialCombinerCallback;

	int sizeofbp = sizeof(btBroadphaseProxy);

	StressTestDemo* concaveDemo = new StressTestDemo();  /// This will not be Deleted!!!
	concaveDemo->initPhysics();
	concaveDemo->setCameraDistance(45.f);

//cannot run stepFront yet, the OpenGL context is not opened (stepFront updates camera...)
//	concaveDemo->stepFront();
//	concaveDemo->stepFront();
//	concaveDemo->stepFront();
//	concaveDemo->stepFront();

	return glutmain(argc, argv,640,480,"DevO,s GIMPACT Test Demo",concaveDemo);
}

//--------------------------
//------------------------------------------------------------------------------
void	StressTestDemo::initPhysics()
{
	/// Init Bullet
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

#ifdef REGISTER_BOX_BOX
	m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,new BoxBoxCollisionAlgorithm::CreateFunc);
#endif //REGISTER_BOX_BOX
	

	LONG maxProxies = 4096;
	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax( 1000, 1000, 1000);
	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
	//m_broadphase = new btMultiSapBroadphase();
	//m_broadphase = new btSimpleBroadphase();

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();;
	m_constraintSolver = solver;

	btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_constraintSolver);
	world->getSolverInfo().m_numIterations = 4;
//	solver->setSolverMode(btSequentialImpulseConstraintSolver::SOLVER_CACHE_FRIENDLY);//btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
//	solver->setSolverMode(btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);

	m_dynamicsWorld = world;
	m_dynamicsWorld->setDebugDrawer(&debugDrawer);

	

	// register algorithm
	if(0)
	{
		m_spheresphere_collisionCreateFunc = new btSphereSphereCollisionAlgorithm::CreateFunc;  /// NEW

		m_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,m_spheresphere_collisionCreateFunc);

		m_spherebox_collisionCreateFunc = new btSphereBoxCollisionAlgorithm::CreateFunc;

		m_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,m_spherebox_collisionCreateFunc);

		m_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,m_spherebox_collisionCreateFunc);

	}



	//create trimesh model and shape




	/// Create Scene
	float mass = 0.f;
	btTransform	startTransform;
	startTransform.setIdentity();

	btCollisionShape* staticboxShape1 = new btBoxShape(btVector3(20,1,20));//floor
	btCollisionShape* staticboxShape2 = new btBoxShape(btVector3(1,50,200));//left wall
	btCollisionShape* staticboxShape3 = new btBoxShape(btVector3(1,50,200));//right wall
	btCollisionShape* staticboxShape4 = new btBoxShape(btVector3(200,50,1));//front wall
	btCollisionShape* staticboxShape5 = new btBoxShape(btVector3(200,50,1));//back wall
#ifdef USE_COMPOUND

	btCompoundShape* staticScenario = new btCompoundShape();//static scenario

	startTransform.setOrigin(btVector3(0,-10,0));
	staticScenario->addChildShape(startTransform,staticboxShape1);
	startTransform.setOrigin(btVector3(-200,15,0));
	staticScenario->addChildShape(startTransform,staticboxShape2);
	startTransform.setOrigin(btVector3(200,15,0));
	staticScenario->addChildShape(startTransform,staticboxShape3);
	startTransform.setOrigin(btVector3(0,15,200));
	staticScenario->addChildShape(startTransform,staticboxShape4);
	startTransform.setOrigin(btVector3(0,15,-200));
	staticScenario->addChildShape(startTransform,staticboxShape5);

	startTransform.setOrigin(btVector3(0,0,0));

	btRigidBody* staticBody = localCreateRigidBody(mass, startTransform,staticScenario);
#else
	startTransform.setOrigin(btVector3(0,-7,0));//;//-10,0));
	btRigidBody* staticBody = localCreateRigidBody(mass, startTransform,staticboxShape1);
	/*startTransform.setOrigin(btVector3(-200,15,0));
	staticBody = localCreateRigidBody(mass, startTransform,staticboxShape2);
	startTransform.setOrigin(btVector3(200,15,0));
	staticBody = localCreateRigidBody(mass, startTransform,staticboxShape3);
	startTransform.setOrigin(btVector3(0,15,200));
	staticBody = localCreateRigidBody(mass, startTransform,staticboxShape4);
	startTransform.setOrigin(btVector3(0,15,-200));
	staticBody = localCreateRigidBody(mass, startTransform,staticboxShape5);
	*/

#endif //USE_COMPOUND

	//enable custom material callback
//	staticBody->setCollisionFlags(staticBody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

	startTransform.setIdentity();
	btCollisionShape* sphShape = new btSphereShape(1);
	btScalar radii[1] = {btScalar(1)};
	btVector3 poss[1]={btVector3(0,0,0)};

//	btCollisionShape* sphShape = new btMultiSphereShape(btVector3(1,1,1),&poss[0],&radii[0],1);

	btScalar margin(0.04);
	btCollisionShape* boxShape = new btBoxShape(btVector3(1,1,1));//1.-margin,1.-margin,1.-margin));
	boxShape->setMargin(margin);


	/// Create Dynamic Boxes
	{

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
										2.0*i + start_x,
										2.0*k + start_y,
										2.0*j + start_z));

					#ifdef SPHERES
					
					localCreateRigidBody(1, startTransform,sphShape);
					#else					
					localCreateRigidBody(1, startTransform,boxShape);
					#endif



				}
			}
		}
	}

	//m_debugMode |= btIDebugDraw::DBG_DrawWireframe;




}


//------------------------------------------------------------------------------
void StressTestDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	int start, end;

    start = glutGet(GLUT_ELAPSED_TIME);

	float dt = float(m_clock.getTimeMicroseconds()) * 0.000001f;

	m_clock.reset();
	m_dynamicsWorld->stepSimulation(dt,1);
	m_steps_done++;

	end = glutGet(GLUT_ELAPSED_TIME);
	printf("Time %d \n", end-start);

	renderme();

	glFlush();
	glutSwapBuffers();

}

//------------------------------------------------------------------------------
void StressTestDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	glFlush();
	glutSwapBuffers();
}

//------------------------------------------------------------------------------
void StressTestDemo::clientResetScene()
{
	m_steps_done = 0;
	DemoApplication::clientResetScene();
}

#define KEY_ESCAPE     0x1B


//------------------------------------------------------------------------------
void StressTestDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '.':
		{
			break;
		}

	case '2':
		{
			dts += 0.000001f;
			break;
		}
	case '3':
		{
			dts -= 0.000001f; if(dts<0.000001f) dts = 0.000001f;
			break;
		}

	default:
		DemoApplication::keyboardCallback(key, x, y);
	}
}


