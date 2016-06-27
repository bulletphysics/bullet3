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

#include "TimeWarp.h"

#include <string>

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btQuickprof.h" // Use your own timer, this timer is only used as we lack another timer

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <BulletDynamics/MLCPSolvers/btLemkeSolver.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>

#include "BulletUtils.hpp" // ERP/CFM setting utils

// sample scenery
static btRigidBody* gCube = NULL; // a sample cube to drop onto the ground

static btRigidBody* gSupport1 = NULL; // support box to put girder onto

static btRigidBody* gSupport2 = NULL; // support box to put girder onto

static btRigidBody* gGirder = NULL; // girder on top of support boxes

static btRigidBody* gSmallBox = NULL; // smaller low mass box attached to static box

static btRigidBody* gLargeBox = NULL; // larger high mass box attached to low mass box

static btRigidBody* gBouncingSphere = NULL; // bouncing sphere with restitution 1

static btRigidBody* gMotorizedBox = NULL; // motorized hinge-box

static btScalar gSimulationSpeed = 1; // default simulation speed at startup

// the current simulation speeds to choose from (the slider will snap to those using a custom form of snapping)
namespace SimulationSpeeds {
static double/*0*/PAUSE = 0;
static double/*1*/QUARTER_SPEED = 0.25;
static double/*2*/HALF_SPEED = 0.5;
static double/*3*/NORMAL_SPEED = 1;
static double/*4*/DOUBLE_SPEED = 2;
static double/*5*/QUADRUPLE_SPEED = 4;
static double/*6*/DECUPLE_SPEED = 10;
static double/*7*/CENTUPLE_SPEED = 100;
static double/*8*/QUINCENTUPLE_SPEED = 500;
static double /*9*/ MILLITUPLE_SPEED = 1000;
static double/*0*/MAX_SPEED = MILLITUPLE_SPEED;
static double /**/NUM_SPEEDS = 11;
};

// add speeds from the namespace here
static double speeds[] = { SimulationSpeeds::PAUSE,
		SimulationSpeeds::QUARTER_SPEED, SimulationSpeeds::HALF_SPEED,
		SimulationSpeeds::NORMAL_SPEED, SimulationSpeeds::DOUBLE_SPEED,
		SimulationSpeeds::QUADRUPLE_SPEED, SimulationSpeeds::DECUPLE_SPEED,
		SimulationSpeeds::CENTUPLE_SPEED,SimulationSpeeds::QUINCENTUPLE_SPEED,
		SimulationSpeeds::MILLITUPLE_SPEED};

static btScalar gSolverIterations = 10; // default number of solver iterations for the iterative solvers

static bool gIsHeadless = false; // demo runs with graphics by default

static int gMinSpeed = SimulationSpeeds::PAUSE; // the minimum simulation speed

static int gMaxSpeed = SimulationSpeeds::MAX_SPEED; // the maximum simulation speed

static bool gMaximumSpeed = false; // the demo does not try to achieve maximum stepping speed by default

static bool gInterpolate = false; // the demo does not use any bullet interpolated physics substeps

static bool useSplitImpulse = true; // split impulse fixes issues with restitution in Baumgarte stabilization
// http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=7117&p=24631&hilit=Baumgarte#p24631
// disabling continuous collision detection can also fix issues with restitution, though CCD is disabled by default an only kicks in at higher speeds
// set CCD speed threshold and testing sphere radius per rigidbody (rb->setCCDSpeedThreshold())

// all supported solvers by bullet
enum SolverEnumType {
	SEQUENTIALIMPULSESOLVER = 0,
	GAUSSSEIDELSOLVER = 1,
	NNCGSOLVER = 2,
	DANZIGSOLVER = 3,
	LEMKESOLVER = 4,
	FSSOLVER = 5,
	NUM_SOLVERS = 6
};


//TODO: In case the solver should be changeable by drop down menu
namespace SolverType {
static char SEQUENTIALIMPULSESOLVER[] = "Sequential Impulse Solver";
static char GAUSSSEIDELSOLVER[] = "Gauss-Seidel Solver";
static char NNCGSOLVER[] = "NNCG Solver";
static char DANZIGSOLVER[] = "Danzig Solver";
static char LEMKESOLVER[] = "Lemke Solver";
static char FSSOLVER[] = "FeatherStone Solver";
};

static const char* solverTypes[NUM_SOLVERS];

static SolverEnumType SOLVER_TYPE = SEQUENTIALIMPULSESOLVER; // You can switch the solver here

//TODO:s===
//TODO: Give specific explanations about solver values

/**
 * Step size of the bullet physics simulator (solverAccuracy). Accuracy versus speed.
 */
// Choose an appropriate number of steps per second for your needs
static btScalar gPhysicsStepsPerSecond = 60.0f; // Default number of steps
//static btScalar gPhysicsStepsPerSecond = 120.0f; // Double steps for more accuracy
//static btScalar gPhysicsStepsPerSecond = 240.0f; // For high accuracy
//static btScalar gPhysicsStepsPerSecond = 1000.0f; // Very high accuracy

// appropriate inverses for seconds and milliseconds
static double fixedPhysicsStepSizeSec = 1.0f / gPhysicsStepsPerSecond; // steps size in seconds
static double fixedPhysicsStepSizeMilli = 1000.0f / gPhysicsStepsPerSecond; // step size in milliseconds

static btScalar gApplicationFrequency = 120.0f; // number of internal application ticks per second
static int gApplicationTick = 1000.0f / gApplicationFrequency; //ms

static btScalar gFpsPerSecond = 30.0f; // number of frames per second

static btScalar gERPSpringK = 10;
static btScalar gERPDamperC = 1;

static btScalar gCFMSpringK = 10;
static btScalar gCFMDamperC = 1;
static btScalar gCFMSingularityAvoidance = 0;
/**
 * @link: Gaffer on Games - Fix your timestep: http://gafferongames.com/game-physics/fix-your-timestep/
 */
struct TimeWarpExample: public CommonRigidBodyBase {
	TimeWarpExample(struct GUIHelperInterface* helper) :
		CommonRigidBodyBase(helper) {

		// main frame timer initialization
		mApplicationStart = mLoopTimer.getTimeMilliseconds(); /**!< Initialize when the application started running */
		mInputClock = mApplicationStart; /**!< Initialize the last time the input was updated */
		mPreviousModelIteration = mApplicationStart;
		mThisModelIteration = mApplicationStart;
		mApplicationRuntime = mThisModelIteration - mApplicationStart; /**!< Initialize the application runtime */

		// sub frame time initializations
		mGraphicsStart = mApplicationStart; /** !< Initialize the last graphics start */
		mModelStart = mApplicationStart; /** !< Initialize the last model start */
		mInputStart = mApplicationStart; /** !< Initialize the last input start */

		mPhysicsStepStart = mApplicationStart; /**!< Initialize the physics step start */
		mPhysicsStepEnd = mApplicationStart; /**!< Initialize the physics step end */

		//durations
		mLastGraphicsTick = 0;
		mLastModelTick = 0;
		mLastInputTick = 0;
		mPhysicsTick = 0;

		mInputDt = 0;
		mModelAccumulator = 0;
		mFrameTime = 0;

		fpsTimeStamp = mLoopTimer.getTimeMilliseconds(); // to time the fps
		fpsStep = 1000.0f/gFpsPerSecond;

		// performance measurements for this demo
		performanceTimestamp = 0;
		performedTime = 0; // time the physics steps consumed
		speedUpPrintTimeStamp = mLoopTimer.getTimeSeconds(); // timer to print the speed up periodically
	}

	virtual ~TimeWarpExample() {
	}

	virtual void initPhysics(); // initialize the demo

	virtual void setupParameterInterface(); // setup the adjustable sliders and button for parameters

	virtual void createEmptyDynamicsWorld(); // create a custom dynamic world according to the solver settings etc.

	virtual void renderScene(); // render the scene to screen

	virtual void resetScene(); // reset scene to make is run constantly

	virtual void stepSimulation(float deltaTime); // customly step the simulation

	virtual void changePhysicsStepsPerSecond(float physicsStepsPerSecond); // change the simulation accuracy

	virtual void changeERPCFM(); // Change ERP/CFM appropriately to the timestep and the ERP/CFM parameters above

	virtual void changeSolverIterations(int iterations); // change the number of iterations

	virtual void changeFPS(float framesPerSecond); // change the frames per second

	virtual void performTrueSteps(btScalar timeStep); // physics stepping without interpolated substeps

	virtual void performInterpolatedSteps(btScalar timeStep); // physics stepping with interpolated substeps

	void performMaxStep(); // perform as many steps as possible

	void performSpeedStep(); // force-perform the number of steps needed to achieve a certain speed (safe to too high speeds, meaning the application will lose time, not the physics)

	bool keyboardCallback(int key, int state); // keyboard callbacks

	void resetCamera() { // reset the camera to its original position
		float dist = 41;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3] = { 0, 0.46, 0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1],
			targetPos[2]);
	}

	// loop timing components ###################
	//# loop timestamps
	btClock mLoopTimer; /**!< The loop timer to time the loop correctly */
	unsigned long int mApplicationStart; /**!< The time the application was started (absolute, in Milliseconds) */
	unsigned long int mPreviousModelIteration; /**!< The previous model iteration timestamp (absolute, in Milliseconds) */
	unsigned long int mThisModelIteration; /**!< This model iteration timestamp (absolute, in Milliseconds) */

	//# loop durations
	long int mModelAccumulator; /**!< The time to forward the model in this loop iteration (relative, in Milliseconds) */
	unsigned long int mFrameTime; /**!< The time to render a frame (relative, in Milliseconds) */
	unsigned long int mApplicationRuntime; /**!< The total application runtime (relative, in Milliseconds) */

	long int mInputDt; /**!< The time difference of input that has to be fed in */
	unsigned long int mInputClock;

	long int mLastGraphicsTick; /*!< The time it took the graphics rendering last time (relative, in Milliseconds) */
	unsigned long int mGraphicsStart;

	long int mLastInputTick; /**!< The time it took the input to process last time (relative, in Milliseconds) */
	unsigned long int mInputStart;

	long int mLastModelTick; /**!<  The time it took the model to update last time
	 This includes the bullet physics update */
	unsigned long int mModelStart; /**!< The timestamp the model started updating last (absolute, in Milliseconds)*/

	long int mPhysicsTick; /**!< The time remaining in the loop to update the physics (relative, in Milliseconds)*/
	unsigned long int mPhysicsStepStart; /**!< The physics start timestamp (absolute, in Milliseconds) */
	unsigned long int mPhysicsStepEnd; /**!< The last physics step end (absolute, in Milliseconds) */

	// to measure the performance of the demo
	double performedTime;
	unsigned long int performanceTimestamp;

	unsigned long int speedUpPrintTimeStamp;

	unsigned long int fpsTimeStamp; /**!< FPS timing variables */
	double fpsStep;
};


// GUI related parameter changing helpers

void twxChangePhysicsStepsPerSecond(float accuracy); // function to change simulation accuracy

void twxChangeERPCFM(float notUsed); // function to change ERP/CFM appropriately

void twxChangeSolverIterations(float notUsed); // change the solver iterations

void twxChangeFPS(float fpsPerSecond); // change frames per second

void clampToCustomSpeedNotches(float speed); // function to clamp to custom speed notches

void switchInterpolated(int buttonId, bool buttonState, void* userPointer); // toggle if interpolation steps are taken

void switchHeadless(int buttonId, bool buttonState, void* userPointer); // toggle if the demo should run headless

void switchMaximumSpeed(int buttonId, bool buttonState, void* userPointer); // toggle it the demo should run as fast as possible

void setApplicationTick(float frequency); // set internal application tick

void changeSolver(int solverType, const char* item, void* userPointer); // change solver type

void floorSliderValues(float notUsed); // floor values that should be ints

static TimeWarpExample* twx = NULL; // just a handle to the demo to access it via functions from outside (not use in your application!)

//==Problems occurring with step size and ERP:==
//[list]
//[*] Tunneling
//[*] Restitution scaling
//[*] Constraint displacement
//[*] Constraint motor explosion
//[/list]
// We look at every issue in detail below:

// Demonstration settings

// Normal mode
void normalMode(int buttonId, bool buttonState, void* userPointer){
	if(twx){
		twx->resetScene(); // reset the scene objects to their original position
	}

	gApplicationFrequency = 120.0f; // number of internal application ticks per second
	gApplicationTick = 1000.0f / gApplicationFrequency; //ms

	gFpsPerSecond = 30.0f; // number of frames per second

	gSimulationSpeed = 1.0f;
	gERPSpringK = 10; // the constraint behaves like a strong spring
	gERPDamperC = 1; // the constraint behaves like a strong damper
	gCFMSpringK = 10;
	gCFMDamperC = 1;
	gCFMSingularityAvoidance = 0;
	gPhysicsStepsPerSecond = 60.0f; // the default setting of the bullet steps per second
	fixedPhysicsStepSizeSec = 1.0f/ gPhysicsStepsPerSecond; // appropriate time steps in seconds
	fixedPhysicsStepSizeMilli = 1000.0f/gPhysicsStepsPerSecond; // and milliseconds
}


// How to blow up this demo
/**
 * =Tunneling=
 * If the time steps per second are decreased, the objects begin to tunnel through each other instead of properly colliding.
 * Counter measure: Keep your steps per second at a reasonable 60, for more complex scenes, increase to 100-300 until tunneling problems no longer occur
 * In some cases, it also works to increase the solver iterations instead. Solver iterations increase the number of steps to converge to a solution.
 */
void showTunneling(int buttonId, bool buttonState, void* userPointer) {
	if(twx){
		twx->resetScene(); // reset the scene objects to their original position
	}

	float lowNumberOfTimeStepsPerSecond = 5.0f; // setting a low number of time steps per second causes the steps to be huge,
												// meaning that collisions are no checked often enough for a falling object to collide with the ground

	gPhysicsStepsPerSecond = lowNumberOfTimeStepsPerSecond;
	fixedPhysicsStepSizeSec = 1.0f / lowNumberOfTimeStepsPerSecond;
	fixedPhysicsStepSizeMilli = 1000.0f / lowNumberOfTimeStepsPerSecond;
}

/**
 * =Restitution scaling=
 * If the time steps per seconds are increased, a ball with restitution 1 does no longer recover to its original height
 * Counter measure: Scale restitution with time step (*steps/60.0f)?
 */
void showRestitutionScaling(int buttonId, bool buttonState, void* userPointer) {
	if(twx){
		twx->resetScene(); // reset the scene objects to their original position
	}

	float highNumberOfTimeStepsPerSecond = 200.0f; // setting a high number of time steps per second causes the steps to be very small
												   // TODO: Simple reason for restitution scaling?

	gPhysicsStepsPerSecond = highNumberOfTimeStepsPerSecond;
	fixedPhysicsStepSizeSec = 1.0f / highNumberOfTimeStepsPerSecond;
	fixedPhysicsStepSizeMilli = 1000.0f / highNumberOfTimeStepsPerSecond;
}

/**
 * =Constraint Displacement=
 * If the Constraint Error Reduction Parameter (ERP) is set to a small value, the constrained objects start to fall apart
 * Counter measure: Keep ERP at a level where the constraints have the right springiness (k_spring) and damping (k_damping) and scale ERP with time step (Check out Bullet Utils).
 */
void showConstraintDisplacement(int buttonId, bool buttonState,
	void* userPointer) {
	if(twx){
		twx->resetScene(); // reset the scene objects to their original position
	}

	gERPSpringK = 0; // the constraint acts like a spring with 0 k, meaning that pulling force is not proportional to displacement distance
//	gERP_c_damper = 100; // You can achieve a similar effect by increasing the damper
}

/**
 * =Constraint Motor Explosion=
 * If the constraint motor is set to a constant impulse, increasing the number of time steps per second causes it to apply the impulse a higher number of times.
 * This causes the motor force to scale with the number of time steps.
 * Counter measure: Scale impulse with the number of time steps (steps/60.0f).
 */
void showConstraintMotorExplosion(int buttonId, bool buttonState, void* userPointer){
	if(twx){
		twx->resetScene(); // reset the scene objects to their original position
	}

	float highNumberOfTimeStepsPerSecond = 200.0f; // setting a high number of time steps per second causes the steps to be very small
												   // The motor impulse is applied more often, resulting in a higher total force/torque applied

	gPhysicsStepsPerSecond = highNumberOfTimeStepsPerSecond;
	fixedPhysicsStepSizeSec = 1.0f / highNumberOfTimeStepsPerSecond;
	fixedPhysicsStepSizeMilli = 1000.0f / highNumberOfTimeStepsPerSecond;
}

void TimeWarpExample::initPhysics() {

	setupParameterInterface(); // setup adjustable sliders and buttons for parameters

	mLoopTimer.reset(); // reset the loop timer to time the loop performance

	m_guiHelper->setUpAxis(1); // Set Y axis as Up axis

	createEmptyDynamicsWorld(); // create an empty dynamic world

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld); // create a physics debug drawer to see what is going on under the hood

	if (m_dynamicsWorld->getDebugDrawer()) // create and setup debug drawer
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(
			btIDebugDraw::DBG_DrawWireframe
				+ btIDebugDraw::DBG_DrawContactPoints);

	// Create sample setups to show how the simulation behaves for different parameters

	{// create a ground shape - several things are standing on it
		btBoxShape* groundShape = createBoxShape(
			btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform; // position and orientation of the static ground
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));

		btRigidBody* ground = createRigidBody(btScalar(0.), groundTransform,
			groundShape, btVector4(0, 0, 1, 1));
		ground->setRestitution(btScalar(1.0f)); // the ground needs to have restitution 1
	}

	{ // create a falling box - used to show tunneling

		btBoxShape* fallingBoxShape = createBoxShape(btVector3(1, 1, 1));
		m_collisionShapes.push_back(fallingBoxShape);

		btTransform fallingBoxTransform; // initial position and orientation of the falling box
		fallingBoxTransform.setIdentity();
		fallingBoxTransform.setOrigin(
			btVector3(btScalar(0), btScalar(20), btScalar(0)));

		btScalar fallingBoxMass(1.f); // mass of the falling box

		gCube = createRigidBody(fallingBoxMass, fallingBoxTransform, fallingBoxShape); // create falling box

		gCube->setActivationState(DISABLE_DEACTIVATION); // with very high simulation speed and repetitive cube trajectory, it can happen that the cube starts to sleep
	}

	{ // create a bench out of two supports and a girder on top of it - used to show tunneling

		// That is what it looks like:
		//
		// =======
		// ||   ||

		btBoxShape* supportShape = createBoxShape(btVector3(2, 2, 2));

		btBoxShape* girderShape = createBoxShape(btVector3(10, 0.1, 2));

		m_collisionShapes.push_back(supportShape);
		m_collisionShapes.push_back(girderShape);

		btScalar mass(1.f); // girder and supports share the same mass

		btTransform startTransform;
		startTransform.setIdentity();

		//create supports
		startTransform.setOrigin(
			btVector3(btScalar(10), btScalar(2), btScalar(0))); // initial position and orientation of the first support

		gSupport1 = createRigidBody(mass, startTransform, supportShape);
		gSupport1->setActivationState(DISABLE_DEACTIVATION); // with very high simulation speed and no motion, it can happen that the cube starts to sleep


		startTransform.setOrigin(
			btVector3(btScalar(-10), btScalar(2), btScalar(0))); // initial position and orientation of the second support

		gSupport2 = createRigidBody(mass, startTransform, supportShape);
		gSupport2->setActivationState(DISABLE_DEACTIVATION); // with very high simulation speed and no motion, it can happen that the cube starts to sleep


		//create girder
		startTransform.setOrigin(
			btVector3(btScalar(0), btScalar(5), btScalar(0))); // initial position and orientation of the girder

		gGirder = createRigidBody(mass, startTransform, girderShape);
		gGirder->setActivationState(DISABLE_DEACTIVATION); // with very high simulation speed and no motion,  it can happen that the cube starts to sleep


	}

	{ // create a chain of a 100kg box hanging from a 1kg box hanging from another static box - used to show constraint displacement
	  // constraints soften with lower spring k or lower time step

		btBoxShape* smallBoxShape = createBoxShape(btVector3(1,1,1));
		m_collisionShapes.push_back(smallBoxShape);

		btTransform startTransform;
		startTransform.setIdentity();

		//create static box
		startTransform.setOrigin(
			btVector3(btScalar(10), btScalar(30), btScalar(10))); // initial position and orientation of the static box

		btRigidBody* staticBox = createRigidBody(btScalar(0), startTransform, smallBoxShape);

		// small box
		btScalar smallMass(1.f); // the small box has a mass of 1kg

		startTransform.setOrigin(
			btVector3(btScalar(8), btScalar(28), btScalar(12))); // initial position and orientation of the small box

		gSmallBox = createRigidBody(smallMass, startTransform, smallBoxShape);
		gSmallBox->setActivationState(DISABLE_DEACTIVATION); // with very high simulation speed and no motion, it can happen that the cube starts to sleep

		//create constraint between boxes
		// this is represented by the constraint pivot in the local frames of reference of both constrained boxes
		btTransform constraintPivotInStaticBoxRF, constraintPivotInSmallBoxRF;

		constraintPivotInStaticBoxRF.setIdentity();
		constraintPivotInSmallBoxRF.setIdentity();

		// we want the constraint pivot to be at the center of mass of the static box

		// the orientation of a point-to-point constraint does not matter, as is has no rotational limits

		// The constraint pivot in the local reference frame of the static box is therefore at its own origin (where the origin is at the center of mass)
		constraintPivotInStaticBoxRF.setOrigin(btVector3(0,0,0));

		// To obtain the position of the static box center of mass in local reference frame of the small box
		// we transform the origin of the static box using the inverse of the small box world transform
		btVector3 staticBoxOriginInSmallBoxRF =
			(gSmallBox->getWorldTransform().inverse()(
				staticBox->getWorldTransform().getOrigin()));
		constraintPivotInSmallBoxRF.setOrigin(staticBoxOriginInSmallBoxRF);

		btPoint2PointConstraint* p2pconst = new btPoint2PointConstraint(
			*staticBox,*gSmallBox,constraintPivotInStaticBoxRF.getOrigin(), constraintPivotInSmallBoxRF.getOrigin());

		p2pconst->setDbgDrawSize(btScalar(5.f)); // indicates how large the constraint should be drawn in case of debug output

		m_dynamicsWorld->addConstraint(p2pconst);


		// large box
		btBoxShape* largeBoxShape = createBoxShape(btVector3(5,5,5));
		m_collisionShapes.push_back(largeBoxShape);

		btScalar largeMass(100.f); // the large box has a mass of 100kg

		startTransform.setOrigin(
			btVector3(btScalar(3), btScalar(21), btScalar(17))); // initial position and orientation of the large box (below of and next to the small box)

		gLargeBox = createRigidBody(largeMass, startTransform, largeBoxShape);
		gLargeBox->setActivationState(DISABLE_DEACTIVATION); // with very high simulation speed and no motion, it can happen that the cube starts to sleep

		//create constraint between boxes
		btTransform constraintPivotInLargeBoxRF;

		constraintPivotInSmallBoxRF.setIdentity();
		constraintPivotInLargeBoxRF.setIdentity();

		// we want the constraint pivot to be at the center of mass of the small box

		// the orientation of a point-to-point constraint does not matter, as is has no rotational limits

		// To obtain the position of the small box center of mass in local reference frame of the large box
		// we transform the origin of the small box using the inverse of the large box world transform
		btVector3 largeBoxOriginInSmallBoxRF =
			(gLargeBox->getWorldTransform().inverse()(
				gSmallBox->getWorldTransform().getOrigin()));
		constraintPivotInLargeBoxRF.setOrigin(largeBoxOriginInSmallBoxRF);

		p2pconst = new btPoint2PointConstraint(
			*gSmallBox,*gLargeBox,constraintPivotInSmallBoxRF.getOrigin(), constraintPivotInLargeBoxRF.getOrigin());

		p2pconst->setDbgDrawSize(btScalar(5.f)); // indicates how large the constraint should be drawn in case of debug output

		m_dynamicsWorld->addConstraint(p2pconst);

	}

	{ // create a bouncing ball - used to show restitution scaling
	  // restitution drops with higher time step -
		btVector3 pos(0,0,0);
		btScalar radius(1.0);
		btMultiSphereShape* colShape = new btMultiSphereShape(&pos, &radius,1);
//		btSphereShape* colShape = new btSphereShape(2);
		m_collisionShapes.push_back(colShape);

		btScalar mass(1.f); // bouncing ball has a mass of 1 kg

		btTransform startTransform;
		startTransform.setIdentity();

		startTransform.setOrigin(
			btVector3(btScalar(-10), btScalar(10), btScalar(-10))); // position the sphere above ground

		gBouncingSphere = createRigidBody(mass, startTransform, colShape);

		gBouncingSphere->setRestitution(btScalar(1.0f)); // restitution 1 should make the sphere restore its original height when bouncing (given that the ground is restitution 1 too)

		gBouncingSphere->setActivationState(DISABLE_DEACTIVATION); // with very high simulation speed and repetitive cube trajectory, it can happen that the sphere starts to sleep
	}

	{// motorized box - used to show constraint motor explosion
	 // motor moves constrained bodies using impulses (F*s) hence more force is applied with a higher time step

		// small box to be attached to the world statically with a motorized constraint
		btBoxShape* smallBoxShape = createBoxShape(btVector3(1,1,1));

		m_collisionShapes.push_back(smallBoxShape);

		btScalar smallMass(1.f); // the box has a mass of 1kg

		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(
			btVector3(btScalar(-8), btScalar(20), btScalar(-12))); // initial position and orientation of the box

		gMotorizedBox = createRigidBody(smallMass, startTransform,
			smallBoxShape);
		gMotorizedBox->setActivationState(DISABLE_DEACTIVATION); // with very high simulation speed and repetitive cube trajectory, it can happen that the cube starts to sleep


		//create hinge constraint
		btTransform constraintPivotAndOrientationInSmallBoxRF;

		constraintPivotAndOrientationInSmallBoxRF.setIdentity();

		btQuaternion qt; // The hinge axis is z aligned per default, but for demonstration we want it to be x-aligned,
		qt.setEuler(SIMD_HALF_PI, 0, 0);  // so we rotate it around the y axis (check the setEuler method description, the order is Y, X, Z)
		constraintPivotAndOrientationInSmallBoxRF.setRotation(qt);

		constraintPivotAndOrientationInSmallBoxRF.setOrigin(btVector3(1,1,0)); // we want the hinge axis to be along one of the top edges of the box, so we set the local pivot to (1,1,0)

		btHingeConstraint* motorizedHinge = new btHingeConstraint(
			*gMotorizedBox, constraintPivotAndOrientationInSmallBoxRF,true); // reference frame A is the motorized box reference frame, therefore set useReferenceFrameA to true

		motorizedHinge->enableAngularMotor(true,5,0.1f); // enable the constraint motor to tune the speed to 5 with a maximum force of 0.1f

		motorizedHinge->setDbgDrawSize(btScalar(5.f)); // indicates how large the constraint should be drawn in case of debug output

		m_dynamicsWorld->addConstraint(motorizedHinge);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void TimeWarpExample::setupParameterInterface(){

	solverTypes[0] = SolverType::SEQUENTIALIMPULSESOLVER;
	solverTypes[1] = SolverType::GAUSSSEIDELSOLVER;
	solverTypes[2] = SolverType::NNCGSOLVER;
	solverTypes[3] = SolverType::DANZIGSOLVER;
	solverTypes[4] = SolverType::LEMKESOLVER;
	solverTypes[5] = SolverType::FSSOLVER;

	 {
	   ComboBoxParams comboParams;
	   comboParams.m_comboboxId = 0;
	   comboParams.m_numItems = NUM_SOLVERS;
	   comboParams.m_startItem = SOLVER_TYPE;
	   comboParams.m_callback = changeSolver;

	   comboParams.m_items=solverTypes;
	   m_guiHelper->getParameterInterface()->registerComboBox(comboParams);
	}

	{ // create a slider to adjust the simulation speed
	  // Force increase the simulation speed to run the simulation with the same accuracy but a higher speed
		SliderParams slider("Simulation speed",
			&gSimulationSpeed);
		slider.m_minVal = gMinSpeed;
		slider.m_maxVal = gMaxSpeed;
		slider.m_callback = clampToCustomSpeedNotches;
		slider.m_clampToNotches = false;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
				slider);
	}

	{ // create a button to switch to headless simulation
	  // This turns off the graphics update and therefore results in more time for the model update
	    ButtonParams button("Run headless",0,true);
	    button.m_callback = switchHeadless;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerButtonParameter(
				button);
	}

	{ // create a slider to adjust the number of internal application ticks
	  // The set application tick should contain enough time to perform a full cycle of model update (physics and input)
	  // and view update (graphics) with average application load. The graphics and input update determine the remaining time
	  // for the physics update
		SliderParams slider("Application Ticks",
			&gApplicationFrequency);
		slider.m_minVal = gMinSpeed;
		slider.m_maxVal = gMaxSpeed;
		slider.m_callback = setApplicationTick;
		slider.m_clampToNotches = false;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
				slider);
	}

	{ // create a button to switch to maximum speed simulation (fully deterministic)
	  // Interesting to test the maximal achievable speed on this hardware
	    ButtonParams button("Run maximum speed",0,true);
	    button.m_callback = switchMaximumSpeed;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerButtonParameter(
				button);
	}

	{ // create a slider to adjust the number of physics steps per second
	  // The default number of steps is at 60, which is appropriate for most general simulations
	  // For simulations with higher complexity or if you experience undesired behavior, try increasing the number of steps per second
	  // Alternatively, try increasing the number of solver iterations if you experience jittering constraints due to non-converging solutions
		SliderParams slider("Physics steps per second", &gPhysicsStepsPerSecond);
		slider.m_minVal = 0;
		slider.m_maxVal = 1000;
		slider.m_callback = twxChangePhysicsStepsPerSecond;
		slider.m_clampToNotches = false;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
				slider);
	}

	{ // create a slider to adjust the number of frames per second
		SliderParams slider("Frames per second", &gFpsPerSecond);
		slider.m_minVal = 0;
		slider.m_maxVal = 200;
		slider.m_callback = twxChangeFPS;
		slider.m_clampToNotches = false;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
				slider);
	}

	{ // create a button to switch bullet to perform interpolated substeps to speed up simulation
	  // generally, interpolated steps are a good speed-up and should only be avoided if higher accuracy is needed (research purposes etc.)
	    ButtonParams button("Perform interpolated substeps",0,true);
	    button.m_callback = switchInterpolated;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerButtonParameter(
				button);
	}

	{ // create a slider to adjust the number of solver iterations to converge to a solution
	  // more complex simulations might need a higher number of iterations to converge, it also
	  // depends on the type of solver. 
		SliderParams slider(
			"Solver interations",
			&gSolverIterations);
		slider.m_minVal = 0;
		slider.m_maxVal = 1000;
		slider.m_callback = twxChangePhysicsStepsPerSecond;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	// ERP/CFM sliders
	// Advanced users: Check descriptions of ERP/CFM in BulletUtils.cpp

	{ // create a slider to adjust ERP Spring k constant
		SliderParams slider("Global ERP Spring k (F=k*x)", &gERPSpringK);
		slider.m_minVal = 0;
		slider.m_maxVal = 10;
		slider.m_callback = twxChangeERPCFM;
		slider.m_clampToNotches = false;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
				slider);
	}

	{ // create a slider to adjust ERP damper c constant
		SliderParams slider("Global ERP damper c (F=c*xdot)", &gERPDamperC);
		slider.m_minVal = 0;
		slider.m_maxVal = 10;
		slider.m_callback = twxChangeERPCFM;
		slider.m_clampToNotches = false;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
				slider);
	}

	{ // create a slider to adjust CFM Spring k constant
		SliderParams slider("Global CFM Spring k (F=k*x)", &gCFMSpringK);
		slider.m_minVal = 0;
		slider.m_maxVal = 10;
		slider.m_callback = twxChangeERPCFM;
		slider.m_clampToNotches = false;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
				slider);
	}

	{ // create a slider to adjust CFM damper c constant
		SliderParams slider("Global CFM damper c (F=c*xdot)", &gCFMDamperC);
		slider.m_minVal = 0;
		slider.m_maxVal = 10;
		slider.m_callback = twxChangeERPCFM;
		slider.m_clampToNotches = false;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
				slider);
	}

	{ // create a slider to adjust CFM damper c constant
		SliderParams slider("Global CFM singularity avoidance", &gCFMSingularityAvoidance);
		slider.m_minVal = 0;
		slider.m_maxVal = 10;
		slider.m_callback = twxChangeERPCFM;
		slider.m_clampToNotches = false;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
				slider);
	}

	// Demo Blow-Up scenario

	{ // create a button to set to normal mode
	    ButtonParams button("Back to normal",0,true);
	    button.m_callback = normalMode;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerButtonParameter(
				button);
	}

	{ // create a button to show tunneling
	    ButtonParams button("Show tunneling",0,true);
	    button.m_callback = showTunneling;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerButtonParameter(
				button);
	}

	{ // create a button to show restitution drop
	    ButtonParams button("Show Restitution Scaling",0,true);
	    button.m_callback = showRestitutionScaling;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerButtonParameter(
				button);
	}

	{ // create a button to show constraint displacement
	    ButtonParams button("Show Constraint Displacement",0,true);
	    button.m_callback = showConstraintDisplacement;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerButtonParameter(
				button);
	}

	{ // create a button to show constraint motor explosion
	    ButtonParams button("Show Constraint Motor Explosion",0,true);
	    button.m_callback = showConstraintMotorExplosion;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerButtonParameter(
				button);
	}

}

void TimeWarpExample::createEmptyDynamicsWorld(){ // create an empty dynamics worlds according to the chosen settings via statics (top section of code)

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	// default broadphase
	m_broadphase = new btDbvtBroadphase();

	// different solvers require different settings
	switch (SOLVER_TYPE) {
	case SEQUENTIALIMPULSESOLVER: {
		b3Printf("=%s=",SolverType::SEQUENTIALIMPULSESOLVER);
		m_solver = new btSequentialImpulseConstraintSolver();
		break;
	}
	case NNCGSOLVER: {
		b3Printf("=%s=",SolverType::NNCGSOLVER);
		m_solver = new btNNCGConstraintSolver();
		break;
	}
	case DANZIGSOLVER: {
		b3Printf("=%s=",SolverType::DANZIGSOLVER);
		btDantzigSolver* mlcp = new btDantzigSolver();
		m_solver = new btMLCPSolver(mlcp);
		break;
	}
	case GAUSSSEIDELSOLVER: {
		b3Printf("=%s=",SolverType::GAUSSSEIDELSOLVER);
		btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel();
		m_solver = new btMLCPSolver(mlcp);
		break;
	}
	case LEMKESOLVER: {
		b3Printf("=%s=",SolverType::LEMKESOLVER);
		btLemkeSolver* mlcp = new btLemkeSolver();
		m_solver = new btMLCPSolver(mlcp);
		break;
	}
	case FSSOLVER: {
		b3Printf("=%s=",SolverType::FSSOLVER);
		//Use the btMultiBodyConstraintSolver for Featherstone btMultiBody support
		m_solver = new btMultiBodyConstraintSolver;

		break;
	}
	default:
		break;
	}

	if (SOLVER_TYPE != FSSOLVER) {
		//TODO: Set parameters for other solvers

		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,
			m_broadphase, m_solver, m_collisionConfiguration);

		if (SOLVER_TYPE == DANZIGSOLVER || SOLVER_TYPE == GAUSSSEIDELSOLVER) {
			m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1; //for mlcp solver it is better to have a small A matrix
		} else {
			m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 128; //for direct solver, it is better to solve multiple objects together, small batches have high overhead
		}

		m_dynamicsWorld->getDispatchInfo().m_useContinuous = true; // set continuous collision

	}
	else{
		//use btMultiBodyDynamicsWorld for Featherstone btMultiBody support
		m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,
			m_broadphase, (btMultiBodyConstraintSolver*) m_solver,
			m_collisionConfiguration);
	}

	changeERPCFM(); // set appropriate ERP/CFM values according to the string and damper properties of the constraint

	if (useSplitImpulse) { // If you experience strong repulsion forces in your constraints, it might help to enable the split impulse feature
		m_dynamicsWorld->getSolverInfo().m_splitImpulse = 1; //enable split impulse feature
//		m_dynamicsWorld->getSolverInfo().m_splitImpulsePenetrationThreshold =
//			-0.02;
//		m_dynamicsWorld->getSolverInfo().m_erp2 = BulletUtils::getERP(
//			fixedPhysicsStepSizeSec, 10, 1);
//		m_dynamicsWorld->getSolverInfo().m_splitImpulseTurnErp =
//			BulletUtils::getERP(fixedPhysicsStepSizeSec, 10, 1);
		b3Printf("Using split impulse feature with ERP/TurnERP: (%f,%f)",
			m_dynamicsWorld->getSolverInfo().m_erp2,
			m_dynamicsWorld->getSolverInfo().m_splitImpulseTurnErp);
	}

	m_dynamicsWorld->getSolverInfo().m_numIterations = gSolverIterations; // set the number of solver iterations for iteration based solvers

	m_dynamicsWorld->setGravity(btVector3(0, -9.81f, 0)); // set gravity to -9.81

}

void TimeWarpExample::resetScene() { // reset the cube to its original position, clearing its forces and velocities

	{
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(
			btVector3(btScalar(0), btScalar(20), btScalar(0)));
		gCube->setWorldTransform(startTransform);

		gCube->clearForces();
		btVector3 zeroVector(0, 0, 0);
		gCube->setLinearVelocity(zeroVector);
		gCube->setAngularVelocity(zeroVector);
	}

	{
		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//create supports
		startTransform.setOrigin(
			btVector3(btScalar(10), btScalar(2), btScalar(0)));
		gSupport1->setWorldTransform(startTransform);

		gSupport1->clearForces();
		btVector3 zeroVector(0, 0, 0);
		gSupport1->setLinearVelocity(zeroVector);
		gSupport1->setAngularVelocity(zeroVector);

		startTransform.setOrigin(
			btVector3(btScalar(-10), btScalar(2), btScalar(0)));
		gSupport2->setWorldTransform(startTransform);

		gSupport2->clearForces();
		gSupport2->setLinearVelocity(zeroVector);
		gSupport2->setAngularVelocity(zeroVector);

		//create girder
		startTransform.setOrigin(
			btVector3(btScalar(0), btScalar(5), btScalar(0)));
		gGirder->setWorldTransform(startTransform);

		gGirder->clearForces();
		gGirder->setLinearVelocity(zeroVector);
		gGirder->setAngularVelocity(zeroVector);

	}

	{
		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		// small box
		startTransform.setOrigin(
			btVector3(btScalar(8), btScalar(28), btScalar(12)));
		gSmallBox->setWorldTransform(startTransform);

		gSmallBox->clearForces();
		btVector3 zeroVector(0, 0, 0);
		gSmallBox->setLinearVelocity(zeroVector);
		gSmallBox->setAngularVelocity(zeroVector);

		// large box
		startTransform.setOrigin(
			btVector3(btScalar(3), btScalar(21), btScalar(17)));
		gLargeBox->setWorldTransform(startTransform);

		gLargeBox->clearForces();
		gLargeBox->setLinearVelocity(zeroVector);
		gLargeBox->setAngularVelocity(zeroVector);
	}

	{
		// the sphere
		btTransform startTransform;
		startTransform.setIdentity();

		// position the top sphere above ground with a moving x position
		startTransform.setOrigin(
			btVector3(btScalar(-10), btScalar(10), btScalar(-10)));
		gBouncingSphere->setWorldTransform(startTransform);

		gBouncingSphere->clearForces();
		btVector3 zeroVector(0, 0, 0);
		gBouncingSphere->setLinearVelocity(zeroVector);
		gBouncingSphere->setAngularVelocity(zeroVector);
	}

	{
		//create motorized box
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(
			btVector3(btScalar(-8), btScalar(20), btScalar(-12))); // initial position and orientation of the box
		gMotorizedBox->setWorldTransform(startTransform);

		gMotorizedBox->clearForces();
		btVector3 zeroVector(0, 0, 0);
		gMotorizedBox->setLinearVelocity(zeroVector);
		gMotorizedBox->setAngularVelocity(zeroVector);
	}


	// on reset, we calculate the performed speed up
	double speedUp = ((double)performedTime*1000.0)/((double)(mLoopTimer.getTimeMilliseconds()-performanceTimestamp));
	b3Printf("Avg Effective speedup: %f",speedUp);
	performedTime = 0;
	performanceTimestamp = mLoopTimer.getTimeMilliseconds();


}

void TimeWarpExample::stepSimulation(float deltaTime) {

	do{

	// structure according to the canonical game loop
	// http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Canonical_Game_Loop

	//##############
	// breaking conditions - if the loop should stop, then check it here

	//#############
	// model update - here you perform updates of your model, be it the physics model, the game or simulation state or anything not related to graphics and input
	if(mLoopTimer.getTimeSeconds() - speedUpPrintTimeStamp > 1){
		// on reset, we calculate the performed speed up
		double speedUp = ((double)performedTime*1000.0)/((double)(mLoopTimer.getTimeMilliseconds()-performanceTimestamp));
		b3Printf("Avg Effective speedup: %f",speedUp);
		performedTime = 0;
		performanceTimestamp = mLoopTimer.getTimeMilliseconds();
		speedUpPrintTimeStamp = mLoopTimer.getTimeSeconds();
	}

	//TODO: Cleanup
//	b3Printf("Height: %f",gBouncingSphere->getWorldTransform().getOrigin().y());

	// update timers
	mThisModelIteration = mLoopTimer.getTimeMilliseconds();
	mFrameTime = mThisModelIteration - mPreviousModelIteration; /**!< Calculate the frame time (in Milliseconds) */
	mPreviousModelIteration = mThisModelIteration;

//	b3Printf("Current Frame time: % u", mFrameTime);

	mApplicationRuntime = mThisModelIteration - mApplicationStart; /**!< Update main frame timer (in Milliseconds) */

	mModelStart = mLoopTimer.getTimeMilliseconds(); /**!< Begin with the model update (in Milliseconds)*/
	mLastGraphicsTick = mModelStart - mGraphicsStart; /**!< Update graphics timer (in Milliseconds) */

	if (gMaximumSpeed /** If maximum speed is enabled*/) {
		performMaxStep();
	} else { /**!< This mode tries to progress as much time as it is expected from the game loop*/
		performSpeedStep();
	}

	mInputStart = mLoopTimer.getTimeMilliseconds(); /**!< Start the input update */
	mLastModelTick = mInputStart - mModelStart; /**!< Calculate the time the model update took */

	//#############
	// Input update - Game Clock part of the loop
	/** This runs once every gApplicationTick milliseconds on average */
	mInputDt = mThisModelIteration - mInputClock;
	if (mInputDt >= gApplicationTick) {
		mInputClock = mThisModelIteration;
//	         mInputHandler.injectInput(); /**!< Inject input into handlers */
//	         mInputHandler.update(mInputClock); /**!< update elements that work on the current input state */
	}

	mGraphicsStart = mLoopTimer.getTimeMilliseconds(); /**!< Start the graphics update */
	mLastInputTick = mGraphicsStart - mInputStart; /**!< Calculate the time the input injection took */

	//#############
	// Graphics update - Here you perform the representation of your model, meaning graphics rendering according to what your game or simulation model describes
	// In the example browser, there is a separate method called renderScene() for this

	// Uncomment this for some detailed output about the application ticks
//	b3Printf(
//		"Physics time: %u milliseconds / Graphics time: %u milliseconds / Input time: %u milliseconds / Total time passed: %u milliseconds",
//		mLastModelTick, mLastGraphicsTick, mLastInputTick, mApplicationRuntime);

	}while(mLoopTimer.getTimeMilliseconds() - fpsTimeStamp < fpsStep); // escape the loop if it is time to render
	// Unfortunately, the input is not included in the loop, therefore the input update frequency is equal to the fps

	fpsTimeStamp = mLoopTimer.getTimeMilliseconds();

}

void TimeWarpExample::performMaxStep() { // perform as many update steps as we can
	if(gApplicationTick >= mLastGraphicsTick + mLastInputTick){ // if the remaining time for graphics is going to be positive
	mPhysicsTick = gApplicationTick /**!< calculate the remaining time for physics (in Milliseconds) */
	- mLastGraphicsTick - mLastInputTick;
	}
	else{
		mPhysicsTick = 0; // no time for physics left / The internal application step is too high
	}

//	b3Printf("Application tick: %u",gApplicationTick);
//	b3Printf("Graphics tick: %u",mLastGraphicsTick);
//	b3Printf("Input tick: %u",mLastInputTick);
//	b3Printf("Physics tick: %u",mPhysicsTick);

	if (mPhysicsTick > 0) { // with positive physics tick we perform as many update steps until the time for it is used up

		mPhysicsStepStart = mLoopTimer.getTimeMilliseconds(); /**!< The physics updates start (in Milliseconds)*/
		mPhysicsStepEnd = mPhysicsStepStart;

		while (mPhysicsTick > mPhysicsStepEnd - mPhysicsStepStart) { /**!< Update the physics until we run out of time (in Milliseconds) */
//			b3Printf("Physics passed: %u", mPhysicsStepEnd - mPhysicsStepStart);
			double timeStep = fixedPhysicsStepSizeSec; /**!< update the world (in Seconds) */

			if (gInterpolate) {
				performInterpolatedSteps(timeStep);
			} else {
				performTrueSteps(timeStep);
			}
			performedTime += timeStep;
			mPhysicsStepEnd = mLoopTimer.getTimeMilliseconds(); /**!< Update the last physics step end to stop updating in time (in Milliseconds) */
		}
	}
}

void TimeWarpExample::performSpeedStep() { // force perform the number of update steps suggested by the speed-up set by the slider
	if (mFrameTime > gApplicationTick) { /** cap frametime to make the application lose time, not the physics (in Milliseconds) */
		mFrameTime = gApplicationTick; // This prevents the physics time accumulator to sum up too much time
	}				       // The simulation therefore gets slower, but still performs all requested physics steps

	mModelAccumulator += mFrameTime; /**!< Accumulate the time the physics simulation has to perform in order to stay in real-time (in Milliseconds) */
//	b3Printf("Model time accumulator: %u", mModelAccumulator);

	int steps = floor(mModelAccumulator / fixedPhysicsStepSizeMilli); /**!< Calculate the number of time steps we can take */
//	b3Printf("Next steps: %i", steps);

	if (steps > 0) { /**!< Update if we can take at least one step */

		double timeStep = gSimulationSpeed * steps * fixedPhysicsStepSizeSec; /**!< update the universe (in Seconds) */

		if (gInterpolate) {
			performInterpolatedSteps(timeStep); // perform interpolated steps
		} else {
			performTrueSteps(timeStep); // perform full steps
		}
		performedTime += timeStep;  // sum up the performed time for measuring the speed up
		mModelAccumulator -= steps * fixedPhysicsStepSizeMilli; /**!< Remove the time performed by the physics simulation from the accumulator, the remaining time carries over to the next cycle  (in Milliseconds) */
	}
}

void TimeWarpExample::performTrueSteps(btScalar timeStep) { // perform uninterpolated steps by performing the amount of steps fitting into the assigned time step
	int subSteps = round(timeStep / fixedPhysicsStepSizeSec); /**!< Calculate the number of full normal time steps we can take */

	for (int i = 0; i < subSteps; i++) { /**!< Perform the number of substeps to reach the timestep*/
		if (timeStep && m_dynamicsWorld) {
			// since we want to perform all proper steps, we perform no interpolated substeps
			int subSteps = 1;

			m_dynamicsWorld->stepSimulation(btScalar(timeStep),
				btScalar(subSteps), btScalar(fixedPhysicsStepSizeSec));
		}
	}
}

void TimeWarpExample::performInterpolatedSteps(btScalar timeStep) { // perform interpolated steps 
	int subSteps = 1 + round(timeStep / fixedPhysicsStepSizeSec); /**!< Calculate the number of full normal time steps we can take, plus 1 for safety of not losing time */
	if (timeStep && m_dynamicsWorld) {

		m_dynamicsWorld->stepSimulation(btScalar(timeStep), btScalar(subSteps),
			btScalar(fixedPhysicsStepSizeSec)); /**!< Perform the number of substeps to reach the timestep*/
	}
}

void TimeWarpExample::renderScene() { // render the scene
	if(!gIsHeadless){ // while the simulation is not running headlessly, render to screen
		CommonRigidBodyBase::renderScene();
	}
}

void TimeWarpExample::changePhysicsStepsPerSecond(float physicsStepsPerSecond) { // Change the number of physics time steps per second
	if (m_dynamicsWorld && physicsStepsPerSecond) {
		fixedPhysicsStepSizeSec = 1.0f / physicsStepsPerSecond;
		fixedPhysicsStepSizeMilli = 1000.0f / physicsStepsPerSecond;

		changeERPCFM();
	}
}

void TimeWarpExample::changeFPS(float framesPerSecond) { // change the number of frames per second
	fpsStep = 1000.0f / gFpsPerSecond;
}

void TimeWarpExample::changeERPCFM(){
	if(m_dynamicsWorld){
		m_dynamicsWorld->getSolverInfo().m_erp = BulletUtils::getERP( // set the error reduction parameter
			fixedPhysicsStepSizeSec, // step size per second
			gERPSpringK, // k of a spring in the equation F = k * x (x:position)
			gERPDamperC); // k of a damper in the equation F = k * v (v:velocity)

		m_dynamicsWorld->getSolverInfo().m_globalCfm = BulletUtils::getCFM( // set the constraint force mixing according to the time step
			gCFMSingularityAvoidance, // singularity avoidance (if you experience unsolvable constraints, increase this value
			fixedPhysicsStepSizeSec, // steps size per second
			gCFMSpringK, // k of a spring in the equation F = k * x (x:position)
			gCFMDamperC); // k of a damper in the equation F = k * v (v:velocity)

			b3Printf("Bullet DynamicsWorld ERP: %f",
				m_dynamicsWorld->getSolverInfo().m_erp);

			b3Printf("Bullet DynamicsWorld CFM: %f",
				m_dynamicsWorld->getSolverInfo().m_globalCfm);
	}
}

void TimeWarpExample::changeSolverIterations(int iterations){ // change the number of iterations per second
	m_dynamicsWorld->getSolverInfo().m_numIterations = iterations;
}

bool TimeWarpExample::keyboardCallback(int key, int state) {
//	b3Printf("Key pressed: %d in state %d \n",key,state);

	switch (key) {
	case 32 /*ASCII for space*/: {
		resetScene();
		break;
	}
	}

	return false;
}

//GUI related parameter changing helpers

void twxChangePhysicsStepsPerSecond(float physicsStepsPerSecond) { // function to change simulation physics steps per second
	if (twx){
		twx->changePhysicsStepsPerSecond(physicsStepsPerSecond);
	}
}

void twxChangeFPS(float fpsPerSecond) {
	if(twx){
		twx->changeFPS(fpsPerSecond);
	}
}

void twxChangeERPCFM(float notUsed) { // function to change ERP/CFM appropriately
	if (twx){
		twx->changeERPCFM();
	}
}

void changeSolver(int comboboxId, const char* item, void* userPointer) { // function to change the solver
	for(int i = 0; i < NUM_SOLVERS;i++){
		if(strcmp(solverTypes[i], item) == 0){ // if the strings are equal
			SOLVER_TYPE = ((SolverEnumType)i);
			b3Printf("=%s=\n Reset the simulation by double clicking it in the menu list.",item);
			return;
		}
	}
	b3Printf("No Change");
}


void twxChangeSolverIterations(float notUsed){ // change the solver iterations

	floorSliderValues(0); // floor the values set by slider

	if(twx)
		twx->changeSolverIterations(gSolverIterations);

}

void clampToCustomSpeedNotches(float speed) { // function to clamp to custom speed notches
	double minSpeed = 0;
	double minSpeedDist = SimulationSpeeds::MAX_SPEED;
	for (int i = 0; i < SimulationSpeeds::NUM_SPEEDS; i++) {
		double speedDist = (speeds[i]-speed >= 0)?speeds[i]-speed:speed-speeds[i]; // float absolute

		if (minSpeedDist > speedDist) {
			minSpeedDist = speedDist;
			minSpeed = speeds[i];
		}
	}
	gSimulationSpeed = minSpeed;
}

void switchInterpolated(int buttonId, bool buttonState, void* userPointer){ // toggle if interpolation steps are taken
	gInterpolate=!gInterpolate;
   b3Printf("Interpolate substeps %s", gInterpolate?"on":"off");
}

void switchHeadless(int buttonId, bool buttonState, void* userPointer){ // toggle if the demo should run headless
	gIsHeadless=!gIsHeadless;
   b3Printf("Run headless %s", gIsHeadless?"on":"off");
}

void switchMaximumSpeed(int buttonId, bool buttonState, void* userPointer){ // toggle it the demo should run as fast as possible
	gMaximumSpeed=!gMaximumSpeed;
   b3Printf("Run maximum speed %s", gMaximumSpeed?"on":"off");
}

void setApplicationTick(float frequency){ // set internal application tick
	gApplicationTick = 1000.0f/frequency;
}

void floorSliderValues(float notUsed) { // floor values that should be ints
	gSolverIterations = floor(gSolverIterations);
}

CommonExampleInterface* ET_TimeWarpCreateFunc(CommonExampleOptions& options) {
	twx = new TimeWarpExample(options.m_guiHelper);
	return twx;
}

