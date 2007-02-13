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
//#define USE_KINEMATIC_GROUND 1
//#define PRINT_CONTACT_STATISTICS 1
//#define REGISTER_CUSTOM_COLLISION_ALGORITHM 1
//#define USER_DEFINED_FRICTION_MODEL 1
#define USE_CUSTOM_NEAR_CALLBACK 1

//following define allows to compare/replace Bullet's constraint solver with ODE quickstep
//this define requires to either add the libquickstep library (win32, see msvc/8/libquickstep.vcproj) or manually add the files from Extras/quickstep
//#define COMPARE_WITH_QUICKSTEP 1


#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
//#include "../Extras/AlternativeCollisionAlgorithms/BoxBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"

#ifdef COMPARE_WITH_QUICKSTEP
#include "../Extras/quickstep/OdeConstraintSolver.h"
#endif //COMPARE_WITH_QUICKSTEP

#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"

#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;
float	gCollisionMargin = 0.05f;
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
		
		new btCylinderShape (btVector3(CUBE_HALF_EXTENTS-gCollisionMargin,CUBE_HALF_EXTENTS-gCollisionMargin,CUBE_HALF_EXTENTS-gCollisionMargin)),
		//new btCapsuleShape(0.5*CUBE_HALF_EXTENTS-gCollisionMargin,CUBE_HALF_EXTENTS-gCollisionMargin),
		//new btCylinderShape (btVector3(1-gCollisionMargin,CUBE_HALF_EXTENTS-gCollisionMargin,1-gCollisionMargin)),
		//new btBoxShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)),
		//new btConeShape(CUBE_HALF_EXTENTS-gCollisionMargin,2.f*CUBE_HALF_EXTENTS-gCollisionMargin),
		
		new btSphereShape (CUBE_HALF_EXTENTS),
		
		//new btBU_Simplex1to4(btPoint3(-1,-1,-1),btPoint3(1,-1,-1),btPoint3(-1,1,-1),btPoint3(0,0,1)),

		//new btEmptyShape(),

		new btBoxShape (btVector3(0.4,1,0.8))

};



////////////////////////////////////



//by default, Bullet will use its own nearcallback, but you can override it using dispatcher->setNearCallback()
void customNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, btDispatcherInfo& dispatchInfo)
{
		btCollisionObject* colObj0 = (btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
		btCollisionObject* colObj1 = (btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

		if (dispatcher.needsCollision(colObj0,colObj1))
		{
			//dispatcher will keep algorithms persistent in the collision pair
			if (!collisionPair.m_algorithm)
			{
				collisionPair.m_algorithm = dispatcher.findAlgorithm(colObj0,colObj1);
			}

			if (collisionPair.m_algorithm)
			{
				btManifoldResult contactPointResult(colObj0,colObj1);
				
				if (dispatchInfo.m_dispatchFunc == 		btDispatcherInfo::DISPATCH_DISCRETE)
				{
					//discrete collision detection query
					collisionPair.m_algorithm->processCollision(colObj0,colObj1,dispatchInfo,&contactPointResult);
				} else
				{
					//continuous collision detection query, time of impact (toi)
					float toi = collisionPair.m_algorithm->calculateTimeOfImpact(colObj0,colObj1,dispatchInfo,&contactPointResult);
					if (dispatchInfo.m_timeOfImpact > toi)
						dispatchInfo.m_timeOfImpact = toi;

				}
			}
		}

}





GLDebugDrawer debugDrawer;

//experimental jitter damping (1 = no damping, 0 = total damping once motion below threshold)
extern btScalar gJitterVelocityDampingFactor;

int main(int argc,char** argv)
{
	gJitterVelocityDampingFactor = 0.7;

	CcdPhysicsDemo* ccdDemo = new CcdPhysicsDemo();

	ccdDemo->initPhysics();

	ccdDemo->setCameraDistance(26.f);

	return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bullet.sf.net",ccdDemo);
}



extern int gNumManifold;
extern int gOverlappingPairs;
extern int gTotalContactPoints;

void CcdPhysicsDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


#ifdef USE_KINEMATIC_GROUND
	//btQuaternion kinRotation(btVector3(0,0,1),0.);
	btVector3 kinTranslation(-0.01,0,0);
	//kinematic object
	btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[0];
	//is this a rigidbody with a motionstate? then use the motionstate to update positions!
	if (btRigidBody::upcast(colObj) && btRigidBody::upcast(colObj)->getMotionState())
	{
		btTransform newTrans;
		btRigidBody::upcast(colObj)->getMotionState()->getWorldTransform(newTrans);
		newTrans.getOrigin()+=kinTranslation;
		btRigidBody::upcast(colObj)->getMotionState()->setWorldTransform(newTrans);
	} else
	{
		m_dynamicsWorld->getCollisionObjectArray()[0]->getWorldTransform().getOrigin() += kinTranslation;
	}

#endif //USE_KINEMATIC_GROUND

	float dt = m_clock.getTimeMicroseconds() * 0.000001f;
	m_clock.reset();
	printf("dt = %f: ",dt);
	
	if (m_dynamicsWorld)
	{

		//swap solver
#ifdef COMPARE_WITH_QUICKSTEP
	   if (m_debugMode & btIDebugDraw::DBG_DisableBulletLCP)
	   {
		   m_dynamicsWorld->setConstraintSolver( new OdeConstraintSolver());
	   } else
	   {
		   m_dynamicsWorld->setConstraintSolver( new btSequentialImpulseConstraintSolver());
	   }
#endif //COMPARE_WITH_QUICKSTEP


		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			dt = 1.0/420.f;

		int numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
	}
	
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

	m_dynamicsWorld->updateAabbs();

	
	renderme();

	glFlush();
	glutSwapBuffers();
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

#ifdef USE_CUSTOM_NEAR_CALLBACK
	//this is optional
	dispatcher->setNearCallback(customNearCallback);
#endif

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);

	btOverlappingPairCache* broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
//	btOverlappingPairCache* broadphase = new btSimpleBroadphase;
	
#ifdef REGISTER_CUSTOM_COLLISION_ALGORITHM
	dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,new btSphereSphereCollisionAlgorithm::CreateFunc);
	//box-box is in Extras/AlternativeCollisionAlgorithms:it requires inclusion of those files
	//dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,new BoxBoxCollisionAlgorithm::CreateFunc);
	dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,TRIANGLE_SHAPE_PROXYTYPE,new btSphereTriangleCollisionAlgorithm::CreateFunc);
	btSphereTriangleCollisionAlgorithm::CreateFunc* triangleSphereCF = new btSphereTriangleCollisionAlgorithm::CreateFunc;
	triangleSphereCF->m_swapped = true;
	dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,TRIANGLE_SHAPE_PROXYTYPE,triangleSphereCF);
#endif //REGISTER_CUSTOM_COLLISION_ALGORITHM

#ifdef COMPARE_WITH_QUICKSTEP
	btConstraintSolver* solver = new OdeConstraintSolver();
#else
	//default constraint solver
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	//default solverMode is SOLVER_RANDMIZE_ORDER. Warmstarting seems not to improve convergence, see 
	//solver->setSolverMode(btSequentialImpulseConstraintSolver::SOLVER_USE_WARMSTARTING | btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
	
#endif
		

		m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver);
		m_dynamicsWorld->setGravity(btVector3(0,-10,0));

		m_dynamicsWorld->setDebugDrawer(&debugDrawer);
		

#ifdef USER_DEFINED_FRICTION_MODEL
	{
		//solver->setContactSolverFunc(ContactSolverFunc func,USER_CONTACT_SOLVER_TYPE1,DEFAULT_CONTACT_SOLVER_TYPE);
		solver->SetFrictionSolverFunc(myFrictionModel,USER_CONTACT_SOLVER_TYPE1,DEFAULT_CONTACT_SOLVER_TYPE);
		solver->SetFrictionSolverFunc(myFrictionModel,DEFAULT_CONTACT_SOLVER_TYPE,USER_CONTACT_SOLVER_TYPE1);
		solver->SetFrictionSolverFunc(myFrictionModel,USER_CONTACT_SOLVER_TYPE1,USER_CONTACT_SOLVER_TYPE1);
		//m_physicsEnvironmentPtr->setNumIterations(2);
	}
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
		compoundShape->addChildShape(ident,oldShape);//
		ident.setOrigin(btVector3(0,0,2));	
		compoundShape->addChildShape(ident,new btSphereShape(0.9));//
	}

	for (i=0;i<gNumObjects;i++)
	{
		btCollisionShape* shape = shapePtr[shapeIndex[i]];
		shape->setMargin(gCollisionMargin);

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
	
		btRigidBody* body = localCreateRigidBody(mass,trans,shape);
#ifdef USE_KINEMATIC_GROUND
		if (mass == 0.f)
		{
			body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState(DISABLE_DEACTIVATION);
		}
#endif //USE_KINEMATIC_GROUND
		
		
		// Only do CCD if  motion in one timestep (1.f/60.f) exceeds CUBE_HALF_EXTENTS
		body->setCcdSquareMotionThreshold( CUBE_HALF_EXTENTS );
		
		//Experimental: better estimation of CCD Time of Impact:
		body->setCcdSweptSphereRadius( 0.2*CUBE_HALF_EXTENTS );

#ifdef USER_DEFINED_FRICTION_MODEL	
		///Advanced use: override the friction solver
		body->m_frictionSolverType = USER_CONTACT_SOLVER_TYPE1;
#endif //USER_DEFINED_FRICTION_MODEL

	}


	clientResetScene();


}
	



