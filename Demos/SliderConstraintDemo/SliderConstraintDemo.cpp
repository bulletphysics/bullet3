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

/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

Added support for ODE sover
April 24, 2008
*/




#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"


#include <stdio.h> //printf debugging

#include "SliderConstraintDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"



#define SLIDER_DEMO_USE_ODE_SOLVER 0
#define SLIDER_DEMO_USE_6DOF 0

#define CUBE_HALF_EXTENTS 1.f

#define SLIDER_ENABLE_ALL_DEMOS 1



// A couple of sliders
#if SLIDER_DEMO_USE_6DOF
	static btGeneric6DofConstraint *spSlider1, *spSlider2;
#else
	static btSliderConstraint *spSlider1, *spSlider2;
#endif

static btPoint2PointConstraint* spP2PConst;
static btHingeConstraint* spHingeConst;



static void draw_axes(const btRigidBody& rb, const btTransform& frame)
{
	glBegin(GL_LINES);
	// draw world transform
	btVector3 from = rb.getWorldTransform().getOrigin();
	btVector3 to = from + rb.getWorldTransform().getBasis() * btVector3(5,0,0);
	// X in red
	glColor3f(255.0F, 0.0F, 0.0F);
	glVertex3d(from.getX(), from.getY(), from.getZ());
	glVertex3d(to.getX(), to.getY(), to.getZ());
	to = from + rb.getWorldTransform().getBasis() * btVector3(0,5,0);
	// Y in green
	glColor3f(0.0F, 255.0F, 0.0F);
	glVertex3d(from.getX(), from.getY(), from.getZ());
	glVertex3d(to.getX(), to.getY(), to.getZ());
	to = from + rb.getWorldTransform().getBasis() * btVector3(0,0,5);
	// Z in blue
	glColor3f(0.0F, 0.0F, 255.0F);
	glVertex3d(from.getX(), from.getY(), from.getZ());
	glVertex3d(to.getX(), to.getY(), to.getZ());
	// draw slider frame
	btTransform calc_frame = rb.getWorldTransform() * frame;
	from = calc_frame.getOrigin();
	to = from + calc_frame.getBasis() * btVector3(10,0,0);
	// X in red
	glColor3f(255.0F, 0.0F, 0.0F);
	glVertex3d(from.getX(), from.getY(), from.getZ());
	glVertex3d(to.getX(), to.getY(), to.getZ());
	to = from + calc_frame.getBasis() * btVector3(0,10,0);
	// Y in green
	glColor3f(0.0F, 255.0F, 0.0F);
	glVertex3d(from.getX(), from.getY(), from.getZ());
	glVertex3d(to.getX(), to.getY(), to.getZ());
	to = from + calc_frame.getBasis() * btVector3(0,0,10);
	// Z in blue
	glColor3f(0.0F, 0.0F, 255.0F);
	glVertex3d(from.getX(), from.getY(), from.getZ());
	glVertex3d(to.getX(), to.getY(), to.getZ());
	glEnd();
} // draw_axes()



#if SLIDER_DEMO_USE_6DOF
static void	drawSlider(btGeneric6DofConstraint* pSlider)
{
	draw_axes(pSlider->getRigidBodyA(), pSlider->getFrameOffsetA());
	draw_axes(pSlider->getRigidBodyB(), pSlider->getFrameOffsetB());
} // drawSlider()
#else
static void	drawSlider(btSliderConstraint* pSlider)
{
	draw_axes(pSlider->getRigidBodyA(), pSlider->getFrameOffsetA());
	draw_axes(pSlider->getRigidBodyB(), pSlider->getFrameOffsetB());
	// draw limits in white
	btVector3 from(pSlider->getLowerLinLimit(), 0, 0);
	btVector3 to(pSlider->getUpperLinLimit(), 0, 0);
	btTransform trans;
	if(pSlider->getUseLinearReferenceFrameA())
	{
		trans = pSlider->getRigidBodyA().getWorldTransform() * pSlider->getFrameOffsetA();
	}
	else
	{
		trans = pSlider->getRigidBodyB().getWorldTransform() * pSlider->getFrameOffsetB();
	}
	from = trans * from;
	to = trans * to;
	glBegin(GL_LINES);
	glColor3f(255.0F, 255.0F, 255.0F);
	glVertex3d(from.getX(), from.getY(), from.getZ());
	glVertex3d(to.getX(), to.getY(), to.getZ());
	glEnd();
} // drawSlider()
#endif



void SliderConstraintDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(26.f);

	// init world
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);

#if SLIDER_DEMO_USE_ODE_SOLVER
	m_constraintSolver = new btOdeQuickstepConstraintSolver();
#else
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
#endif

	btDiscreteDynamicsWorld* wp = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
	//	wp->getSolverInfo().m_numIterations = 20; // default is 10
	m_dynamicsWorld = wp;
//	wp->getSolverInfo().m_erp = 0.8;

	// add floor
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-76,0));
	btRigidBody* groundBody;
	groundBody = localCreateRigidBody(0, groundTransform, groundShape);
	
	// add box shape (will be reused for all bodies)
	btCollisionShape* shape = new btBoxShape(btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
	m_collisionShapes.push_back(shape);
	// mass of dymamic bodies
	btScalar mass = btScalar(1.);
	
	// add dynamic rigid body A1
	btTransform trans;
	trans.setIdentity();
	btVector3 worldPos(-20,0,0);
	trans.setOrigin(worldPos);
	btTransform frameInA, frameInB;
	frameInA = btTransform::getIdentity();
	frameInB = btTransform::getIdentity();

#if SLIDER_ENABLE_ALL_DEMOS
	btRigidBody* pRbA1 = localCreateRigidBody(mass, trans, shape);
//	btRigidBody* pRbA1 = localCreateRigidBody(0.f, trans, shape);
	pRbA1->setActivationState(DISABLE_DEACTIVATION);

	// add dynamic rigid body B1
	worldPos.setValue(-30,0,0);
	trans.setOrigin(worldPos);
	btRigidBody* pRbB1 = localCreateRigidBody(mass, trans, shape);
//	btRigidBody* pRbB1 = localCreateRigidBody(0.f, trans, shape);
	pRbB1->setActivationState(DISABLE_DEACTIVATION);

	// create slider constraint between A1 and B1 and add it to world
	
#if SLIDER_DEMO_USE_6DOF
	spSlider1 = new btGeneric6DofConstraint(*pRbA1, *pRbB1, frameInA, frameInB, true);
	btVector3 lowerSliderLimit = btVector3(-20,0,0);
	btVector3 hiSliderLimit = btVector3(-10,0,0);
//	btVector3 lowerSliderLimit = btVector3(-20,-5,-5);
//	btVector3 hiSliderLimit = btVector3(-10,5,5);
	spSlider1->setLinearLowerLimit(lowerSliderLimit);
	spSlider1->setLinearUpperLimit(hiSliderLimit);
	spSlider1->setAngularLowerLimit(btVector3(0,0,0));
	spSlider1->setAngularUpperLimit(btVector3(0,0,0));
#else
	spSlider1 = new btSliderConstraint(*pRbA1, *pRbB1, frameInA, frameInB, true);
//	spSlider1 = new btSliderConstraint(*pRbA1, *pRbB1, frameInA, frameInB, false);
	spSlider1->setLowerLinLimit(-15.0F);
	spSlider1->setUpperLinLimit(-5.0F);
//	spSlider1->setLowerLinLimit(5.0F);
//	spSlider1->setUpperLinLimit(15.0F);
//	spSlider1->setLowerLinLimit(-10.0F);
//	spSlider1->setUpperLinLimit(-10.0F);

	spSlider1->setLowerAngLimit(-SIMD_PI / 3.0F);
	spSlider1->setUpperAngLimit( SIMD_PI / 3.0F);
#endif

	m_dynamicsWorld->addConstraint(spSlider1, true);
	spSlider1->setDbgDrawSize(btScalar(5.f));
#endif

#if SLIDER_ENABLE_ALL_DEMOS
	// add kinematic rigid body A2
//	worldPos.setValue(20,4,0);
	worldPos.setValue(5,-20,0);
	trans.setOrigin(worldPos);
	btRigidBody* pRbA2 = localCreateRigidBody(0., trans, shape);
//	btRigidBody* pRbA2 = localCreateRigidBody(mass, trans, shape);
//	btRigidBody* pRbA2 = localCreateRigidBody(mass * 10000, trans, shape);
	pRbA2->setActivationState(DISABLE_DEACTIVATION);

	// add dynamic rigid body B2
//	worldPos.setValue(-20,4,0);
	worldPos.setValue(-5,-20,0);
	trans.setOrigin(worldPos);
//	btRigidBody* pRbB2 = localCreateRigidBody(0., trans, shape);
	btRigidBody* pRbB2 = localCreateRigidBody(mass, trans, shape);
//	btRigidBody* pRbB2 = localCreateRigidBody(mass * 10000, trans, shape);
	pRbB2->setActivationState(DISABLE_DEACTIVATION);

//	frameInA.getBasis().setEulerZYX(1.f, 1.f, 1.f);
//	frameInB.getBasis().setEulerZYX(1.f, 1.f, 1.f);
//	frameInA.getBasis().setEulerZYX(1.f, 1.f, 1.f);
//	frameInB.getBasis().setEulerZYX(1.f, 1.f, 1.f);


//	frameInA.setOrigin(btVector3(-20., 5., 0));
//	frameInB.setOrigin(btVector3( 20., 5., 0));
	frameInA.setOrigin(btVector3(-5., 20., 0));
	frameInB.setOrigin(btVector3( 5., 20., 0));


	// create slider constraint between A2 and B2 and add it to world
#if SLIDER_DEMO_USE_6DOF
	spSlider2 = new btGeneric6DofConstraint(*pRbA2, *pRbB2, frameInA, frameInB, true);
	spSlider2->setLinearLowerLimit(lowerSliderLimit);
	spSlider2->setLinearUpperLimit(hiSliderLimit);
	spSlider2->setAngularLowerLimit(btVector3(0,0,0));
	spSlider2->setAngularUpperLimit(btVector3(0,0,0));
#else
	spSlider2 = new btSliderConstraint(*pRbA2, *pRbB2, frameInA, frameInB, true);
//	spSlider2 = new btSliderConstraint(*pRbA2, *pRbB2, frameInA, frameInB, false);
//	spSlider2->setLowerLinLimit(0.0F);
//	spSlider2->setUpperLinLimit(0.0F);
	spSlider2->setLowerLinLimit(-2.0F);
	spSlider2->setUpperLinLimit(2.0F);
//	spSlider2->setLowerLinLimit(5.0F);
//	spSlider2->setUpperLinLimit(25.0F);
//	spSlider2->setUpperLinLimit(-5.0F);
//	spSlider2->setUpperLinLimit(-9.99F);


//	spSlider2->setLowerAngLimit(SIMD_PI / 2.0F);
//	spSlider2->setUpperAngLimit(-SIMD_PI / 2.0F);

	//	spSlider2->setLowerAngLimit(-SIMD_PI / 2.0F);
//	spSlider2->setUpperAngLimit(SIMD_PI / 2.0F);

//	spSlider2->setLowerAngLimit(-SIMD_PI);
//	spSlider2->setUpperAngLimit(SIMD_PI *0.8F);


//	spSlider2->setLowerAngLimit(-0.01F);
//	spSlider2->setUpperAngLimit(0.01F);
	spSlider2->setLowerAngLimit(-1.570796326F * 0.5f);
	spSlider2->setUpperAngLimit(1.570796326F * 0.5f);
//	spSlider2->setLowerAngLimit(1.F);
//	spSlider2->setUpperAngLimit(-1.F);


//	spSlider2->setDampingLimLin(0.5f);

#if 0
	// add motors
	spSlider2->setPoweredLinMotor(true);
	spSlider2->setMaxLinMotorForce(0.1);
	spSlider2->setTargetLinMotorVelocity(-5.0);

	spSlider2->setPoweredAngMotor(true);
//	spSlider2->setMaxAngMotorForce(0.01);
	spSlider2->setMaxAngMotorForce(10.0);
	spSlider2->setTargetAngMotorVelocity(1.0);

	// change default damping and restitution

	spSlider2->setDampingDirLin(0.005F);
	spSlider2->setRestitutionLimLin(1.1F);
#endif

	// various ODE tests
//	spSlider2->setDampingLimLin(0.1F); // linear bounce factor for ODE == 1.0 - DampingLimLin;
//	spSlider2->setDampingLimAng(0.1F); // angular bounce factor for ODE == 1.0 - DampingLimAng;
//	spSlider2->setSoftnessOrthoAng(0.1);
//	spSlider2->setSoftnessOrthoLin(0.1);
//	spSlider2->setSoftnessLimLin(0.1);
//	spSlider2->setSoftnessLimAng(0.1);
#endif
	m_dynamicsWorld->addConstraint(spSlider2, true);
	spSlider2->setDbgDrawSize(btScalar(5.f));
#endif

#if SLIDER_ENABLE_ALL_DEMOS
{
	// add dynamic rigid body A1
	trans.setIdentity();
	worldPos.setValue(20,0,0);
	trans.setOrigin(worldPos);
	btRigidBody* pRbA3 = localCreateRigidBody(0.0F, trans, shape);
	pRbA3->setActivationState(DISABLE_DEACTIVATION);

	// add dynamic rigid body B1
	worldPos.setValue(25,0,0);
	trans.setOrigin(worldPos);
	btRigidBody* pRbB3 = localCreateRigidBody(mass, trans, shape);
	pRbB3->setActivationState(DISABLE_DEACTIVATION);

	btVector3 pivA( 2.5, 0., 0.);
	btVector3 pivB(-2.5, 0., 0.);
	spP2PConst = new btPoint2PointConstraint(*pRbA3, *pRbB3, pivA, pivB);
	m_dynamicsWorld->addConstraint(spP2PConst, true);
	spP2PConst->setDbgDrawSize(btScalar(5.f));

}
#endif

#if 0 // SLIDER_ENABLE_ALL_DEMOS
	// add dynamic rigid body A4
	trans.setIdentity();
	worldPos.setValue(20,10,0);
	trans.setOrigin(worldPos);
	btRigidBody* pRbA4 = localCreateRigidBody(0.0F, trans, shape);
	pRbA4->setActivationState(DISABLE_DEACTIVATION);

	// add dynamic rigid body B1
	worldPos.setValue(27,10,0);
	trans.setOrigin(worldPos);
	btRigidBody* pRbB4 = localCreateRigidBody(mass, trans, shape);
	pRbB1->setActivationState(DISABLE_DEACTIVATION);


	btVector3 pivA( 2., 0., 0.);
	btVector3 pivB(-5., 0., 0.);
	btVector3 axisA(0., 0., 1.);
	btVector3 axisB(0., 0., 1.);

	spHingeConst = new btHingeConstraint(*pRbA4, *pRbB4, pivA, pivB, axisA, axisB);
//	spHingeConst->setLimit(-1.57, 1.57);
	spHingeConst->setLimit(1.57, -1.57);
	spHingeConst->enableAngularMotor(true, 10.0, 0.19);

	m_dynamicsWorld->addConstraint(spHingeConst, true);
	spHingeConst->setDbgDrawSize(btScalar(5.f));

#endif

} // SliderConstraintDemo::initPhysics()



SliderConstraintDemo::~SliderConstraintDemo()
{
	//cleanup in the reverse order of creation/initialization
	int i;
	//removed/delete constraints
	for (i=m_dynamicsWorld->getNumConstraints()-1; i>=0 ;i--)
	{
		btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
		m_dynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}
	//remove the rigidbodies from the dynamics world and delete them
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
	//delete dynamics world
	delete m_dynamicsWorld;
	//delete solver
	delete m_constraintSolver;
	//delete broadphase
	delete m_overlappingPairCache;
	//delete dispatcher
	delete m_dispatcher;
	delete m_collisionConfiguration;
} // SliderConstraintDemo::~SliderConstraintDemo()



void SliderConstraintDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
 	float dt = float(getDeltaTimeMicroseconds()) * 0.000001f;
 	//during idle mode, just run 1 simulation step maximum
	int maxSimSubSteps = m_idle ? 1 : 1;
	if(m_idle)
	{
		dt = 1.0/420.f;
	}
	int numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
	//optional but useful: debug drawing
	m_dynamicsWorld->debugDrawWorld();
	bool verbose = false;
	if (verbose)
	{
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
	renderme();
//	drawSlider(spSlider1);
//	drawSlider(spSlider2);
    glFlush();
    glutSwapBuffers();
} // SliderConstraintDemo::clientMoveAndDisplay()



void SliderConstraintDemo::displayCallback(void) 
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	if(m_dynamicsWorld)
	{
		m_dynamicsWorld->debugDrawWorld();
	}
//	drawSlider(spSlider1);
//	drawSlider(spSlider2);
	renderme();
    glFlush();
    glutSwapBuffers();
} // SliderConstraintDemo::displayCallback()


void SliderConstraintDemo::keyboardCallback(unsigned char key, int x, int y)
{
	(void)x;
	(void)y;
	switch (key) 
	{
		case 'O' :
			{
				bool offectOnOff;
				offectOnOff = spSlider1->getUseFrameOffset();
				offectOnOff = !offectOnOff;
				spSlider1->setUseFrameOffset(offectOnOff);
				printf("Slider1 %s frame offset\n", offectOnOff ? "uses" : "does not use");
				offectOnOff = spSlider2->getUseFrameOffset();
				offectOnOff = !offectOnOff;
				spSlider2->setUseFrameOffset(offectOnOff);
				printf("Slider2 %s frame offset\n", offectOnOff ? "uses" : "does not use");
			}
			break;
		default : 
			{
				DemoApplication::keyboardCallback(key, x, y);
			}
			break;
	}
}

