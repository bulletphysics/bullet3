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

//#define USER_DEFINED_FRICTION_MODEL 1
//#define PRINT_CONTACT_STATISTICS 1
#define REGISTER_CUSTOM_COLLISION_ALGORITHM 1

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"

#include "PHY_Pro.h"
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;

#include "CcdPhysicsDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"


extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;
const int maxOverlap = 65535;

bool createConstraint = true;//false;
bool useCompound = false;//true;//false;


#ifdef _DEBUG
const int gNumObjects = 120;
#else
const int gNumObjects = 120;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif

const int maxNumObjects = 32760;

int	shapeIndex[maxNumObjects];


#define CUBE_HALF_EXTENTS 1

#define EXTRA_HEIGHT -20.f
//GL_LineSegmentShape shapeE(btPoint3(-50,0,0),
//						   btPoint3(50,0,0));
static const int numShapes = 4;

btCollisionShape* shapePtr[numShapes] = 
{
	///Please don't make the box sizes larger then 1000: the collision detection will be inaccurate.
	///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=346

//#define USE_GROUND_PLANE 1
#ifdef USE_GROUND_PLANE
	new btStaticPlaneShape(btVector3(0,1,0),10),
#else
	new btBoxShape (btVector3(50,10,50)),
#endif
		
		new btBoxShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)),
		new btSphereShape (CUBE_HALF_EXTENTS- 0.05f),

		//new btConeShape(CUBE_HALF_EXTENTS,2.f*CUBE_HALF_EXTENTS),
		//new btBU_Simplex1to4(btPoint3(-1,-1,-1),btPoint3(1,-1,-1),btPoint3(-1,1,-1),btPoint3(0,0,1)),

		//new btEmptyShape(),

		new btBoxShape (btVector3(0.4,1,0.8))

};



////////////////////////////////////






GLDebugDrawer debugDrawer;

int main(int argc,char** argv)
{

	CcdPhysicsDemo* ccdDemo = new CcdPhysicsDemo();

	ccdDemo->initPhysics();

	ccdDemo->setCameraDistance(26.f);

	return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://www.continuousphysics.com/Bullet/phpBB2/",ccdDemo);
}



extern int gNumManifold;
extern int gOverlappingPairs;
extern int gTotalContactPoints;

void CcdPhysicsDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	if (m_dynamicsWorld)
		m_dynamicsWorld->stepSimulation(deltaTime);
	
#ifdef USE_QUICKPROF 
        btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 
	
	renderme(); 

#ifdef USE_QUICKPROF 
        btProfiler::endBlock("render"); 
#endif 
	glFlush();
	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num manifolds: %i\n",gNumManifold);
	printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	printf("num gTotalContactPoints : %i\n",gTotalContactPoints );
#endif //PRINT_CONTACT_STATISTICS

	gTotalContactPoints = 0;
	glutSwapBuffers();

}



void CcdPhysicsDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	/*
	if (m_physicsEnvironmentPtr)
	{
		m_physicsEnvironmentPtr->UpdateAabbs(deltaTime);
		//draw contactpoints
		m_physicsEnvironmentPtr->CallbackTriggers();
	}
	*/


	renderme();

	glFlush();
	glutSwapBuffers();
}



///make this positive to show stack falling from a distance
///this shows the penalty tresholds in action, springy/spungy look

void CcdPhysicsDemo::clientResetScene()
{

/*	
	int i;
	int numObjects = m_physicsEnvironmentPtr->GetNumControllers();

	for (i=0;i<numObjects;i++)
	{
		//skip the first object (static ground)
		if (i>0)
		{
			CcdPhysicsController* ctrl = m_physicsEnvironmentPtr->GetPhysicsController(i);

			if ((getDebugMode() & btIDebugDraw::DBG_NoHelpText))
			{
				if (ctrl->GetRigidBody()->GetCollisionShape()->GetShapeType() != SPHERE_SHAPE_PROXYTYPE)
				{
					ctrl->GetRigidBody()->SetCollisionShape(shapePtr[2]);
				} else
				{
					ctrl->GetRigidBody()->SetCollisionShape(shapePtr[1]);
				}

				btBroadphaseProxy* bpproxy = ctrl->GetRigidBody()->m_broadphaseHandle;
				m_physicsEnvironmentPtr->GetBroadphase()->CleanProxyFromPairs(bpproxy);
			}

			//stack them
			int colsize = 10;
			int row = (i*CUBE_HALF_EXTENTS*2)/(colsize*2*CUBE_HALF_EXTENTS);
			int row2 = row;
			int col = (i)%(colsize)-colsize/2;


			if (col>3)
			{
				col=11;
				row2 |=1;
			}
			ctrl->setPosition(col*2*CUBE_HALF_EXTENTS + (row2%2)*CUBE_HALF_EXTENTS,
				row*2*CUBE_HALF_EXTENTS+CUBE_HALF_EXTENTS+EXTRA_HEIGHT,0);
			ctrl->setOrientation(0,0,0,1);
			ctrl->SetLinearVelocity(0,0,0,false);
			ctrl->SetAngularVelocity(0,0,0,false);
		} 
	}
	*/

}


///User-defined friction model, the most simple friction model available: no friction
float myFrictionModel(	btRigidBody& body1,	btRigidBody& body2,	btManifoldPoint& contactPoint,	const btContactSolverInfo& solverInfo	)
{
	//don't do any friction
	return 0.f;
}

void	CcdPhysicsDemo::initPhysics()
{

	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher();

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);

	btOverlappingPairCache* broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
//	btOverlappingPairCache* broadphase = new btSimpleBroadphase;
	
#ifdef REGISTER_CUSTOM_COLLISION_ALGORITHM
	dispatcher->RegisterCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,new btSphereSphereCollisionAlgorithm::CreateFunc);
#endif //REGISTER_CUSTOM_COLLISION_ALGORITHM

		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

		m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver);


#ifdef USER_DEFINED_FRICTION_MODEL
	btSequentialImpulseConstraintSolver* solver = (btSequentialImpulseConstraintSolver*) m_physicsEnvironmentPtr->GetConstraintSolver();
	//solver->SetContactSolverFunc(ContactSolverFunc func,USER_CONTACT_SOLVER_TYPE1,DEFAULT_CONTACT_SOLVER_TYPE);
	solver->SetFrictionSolverFunc(myFrictionModel,USER_CONTACT_SOLVER_TYPE1,DEFAULT_CONTACT_SOLVER_TYPE);
	solver->SetFrictionSolverFunc(myFrictionModel,DEFAULT_CONTACT_SOLVER_TYPE,USER_CONTACT_SOLVER_TYPE1);
	solver->SetFrictionSolverFunc(myFrictionModel,USER_CONTACT_SOLVER_TYPE1,USER_CONTACT_SOLVER_TYPE1);
	//m_physicsEnvironmentPtr->setNumIterations(2);
#endif //USER_DEFINED_FRICTION_MODEL


	int i;

	btTransform tr;
	tr.setIdentity();

	
	for (i=0;i<gNumObjects;i++)
	{
		if (i>0)
		{
			shapeIndex[i] = 1;//sphere
		}
		else
			shapeIndex[i] = 0;
	}

	if (useCompound)
	{
		btCompoundShape* compoundShape = new btCompoundShape();
		btCollisionShape* oldShape = shapePtr[1];
		shapePtr[1] = compoundShape;

		btTransform ident;
		ident.setIdentity();
		ident.setOrigin(btVector3(0,0,0));	
		compoundShape->AddChildShape(ident,oldShape);//
		ident.setOrigin(btVector3(0,0,2));	
		compoundShape->AddChildShape(ident,new btSphereShape(0.9));//
	}

	for (i=0;i<gNumObjects;i++)
	{
		btCollisionShape* shape = shapePtr[shapeIndex[i]];
		shape->SetMargin(0.05f);

		bool isDyna = i>0;

		btTransform trans;
		trans.setIdentity();
		
		if (i>0)
		{
			//stack them
			int colsize = 10;
			int row = (i*CUBE_HALF_EXTENTS*2)/(colsize*2*CUBE_HALF_EXTENTS);
			int row2 = row;
			int col = (i)%(colsize)-colsize/2;


			if (col>3)
			{
				col=11;
				row2 |=1;
			}

			btVector3 pos(col*2*CUBE_HALF_EXTENTS + (row2%2)*CUBE_HALF_EXTENTS,
				row*2*CUBE_HALF_EXTENTS+CUBE_HALF_EXTENTS+EXTRA_HEIGHT,0);

			trans.setOrigin(pos);
		} else
		{
			trans.setOrigin(btVector3(0,-30,0));
		}

		float mass = 1.f;

		if (!isDyna)
			mass = 0.f;
	
		btRigidBody* body = LocalCreateRigidBody(isDyna,mass,trans,shape);
		
		m_dynamicsWorld->AddCollisionObject(body);
		
		// Only do CCD if  motion in one timestep (1.f/60.f) exceeds CUBE_HALF_EXTENTS
		body->m_ccdSquareMotionTreshold = CUBE_HALF_EXTENTS;
		
		//Experimental: better estimation of CCD Time of Impact:
		body->m_ccdSweptShereRadius = 0.2*CUBE_HALF_EXTENTS;

#ifdef USER_DEFINED_FRICTION_MODEL	
		///Advanced use: override the friction solver
		ctrl->GetRigidBody()->m_frictionSolverType = USER_CONTACT_SOLVER_TYPE1;
#endif //USER_DEFINED_FRICTION_MODEL

	}


	clientResetScene();


}
	



