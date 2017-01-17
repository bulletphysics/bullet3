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

#ifndef NN3D_WALKERS_TIME_WARP_BASE_H
#define NN3D_WALKERS_TIME_WARP_BASE_H

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btQuickprof.h" // Use your own timer, this timer is only used as we lack another timer

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

//Solvers
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

#include "../Utils/b3ERPCFMHelper.hpp" // ERP/CFM setting utils

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
static double speeds[] = {
	SimulationSpeeds::PAUSE,
	SimulationSpeeds::QUARTER_SPEED, 	SimulationSpeeds::HALF_SPEED,
	SimulationSpeeds::NORMAL_SPEED, 	SimulationSpeeds::DOUBLE_SPEED,
	SimulationSpeeds::QUADRUPLE_SPEED, 	SimulationSpeeds::DECUPLE_SPEED,
	SimulationSpeeds::CENTUPLE_SPEED,	SimulationSpeeds::QUINCENTUPLE_SPEED,
	SimulationSpeeds::MILLITUPLE_SPEED};

static btScalar gSolverIterations = 10; // default number of solver iterations for the iterative solvers

static bool gIsHeadless = false; // demo runs with graphics by default

static bool gChangeErpCfm = false; // flag to make recalculation of ERP/CFM

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


// solvers can be changed by drop down menu
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

static btScalar gApplicationFrequency = 60.0f; // number of internal application ticks per second
static int gApplicationTick = 1000.0f / gApplicationFrequency; //ms

static btScalar gFramesPerSecond = 30.0f; // number of frames per second

static btScalar gERPSpringK = 10;
static btScalar gERPDamperC = 1;

static btScalar gCFMSpringK = 10;
static btScalar gCFMDamperC = 1;
static btScalar gCFMSingularityAvoidance = 0;

//GUI related parameter changing helpers

inline void twxChangePhysicsStepsPerSecond(float physicsStepsPerSecond, void*) { // function to change simulation physics steps per second
	gPhysicsStepsPerSecond = physicsStepsPerSecond;
}

inline void twxChangeFPS(float framesPerSecond, void*) {
	gFramesPerSecond = framesPerSecond;
}

inline void twxChangeERPCFM(float notUsed, void*) { // function to change ERP/CFM appropriately
	gChangeErpCfm = true;
}

inline void changeSolver(int comboboxId, const char* item, void* userPointer) { // function to change the solver
	for(int i = 0; i < NUM_SOLVERS;i++){
		if(strcmp(solverTypes[i], item) == 0){ // if the strings are equal
			SOLVER_TYPE = ((SolverEnumType)i);
			b3Printf("=%s=\n Reset the simulation by double clicking it in the menu list.",item);
			return;
		}
	}
	b3Printf("No Change");
}


inline void twxChangeSolverIterations(float notUsed, void* userPtr) { // change the solver iterations


}

inline void clampToCustomSpeedNotches(float speed, void*) { // function to clamp to custom speed notches
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

inline void switchInterpolated(int buttonId, bool buttonState, void* userPointer){ // toggle if interpolation steps are taken
	gInterpolate=!gInterpolate;
//	b3Printf("Interpolate substeps %s", gInterpolate?"on":"off");
}

inline void switchHeadless(int buttonId, bool buttonState, void* userPointer){ // toggle if the demo should run headless
	gIsHeadless=!gIsHeadless;
//	b3Printf("Run headless %s", gIsHeadless?"on":"off");
}

inline void switchMaximumSpeed(int buttonId, bool buttonState, void* userPointer){ // toggle it the demo should run as fast as possible
//	b3Printf("Run maximum speed %s", gMaximumSpeed?"on":"off");
}

inline void setApplicationTick(float frequency, void*){ // set internal application tick
	gApplicationTick = 1000.0f/frequency;
}

/**
 * @link: Gaffer on Games - Fix your timestep: http://gafferongames.com/game-physics/fix-your-timestep/
 */
struct NN3DWalkersTimeWarpBase: public CommonRigidBodyBase {

	NN3DWalkersTimeWarpBase(struct GUIHelperInterface* helper):
		CommonRigidBodyBase(helper),
		mPhysicsStepsPerSecondUpdated(false),
		mFramesPerSecondUpdated(false),
		mSolverIterationsUpdated(false) {

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
		 fpsStep = 1000.0f/gFramesPerSecond;

		 // performance measurements for this demo
		 performanceTimestamp = 0;
		 performedTime = 0; // time the physics steps consumed
		 speedUpPrintTimeStamp = mLoopTimer.getTimeSeconds(); // timer to print the speed up periodically
		 mLoopTimer.reset();
	 }

	~NN3DWalkersTimeWarpBase(){

	}


	void initPhysics(){ // initialize the demo

		setupBasicParamInterface(); // setup adjustable sliders and buttons for parameters

		m_guiHelper->setUpAxis(1); // Set Y axis as Up axis

		createEmptyDynamicsWorld(); // create an empty dynamic world

		m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	}

	void setupBasicParamInterface(){ // setup the adjustable sliders and button for parameters

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



		{ // create a button to switch to maximum speed simulation (fully deterministic)
			// Interesting to test the maximal achievable speed on this hardware
			ButtonParams button("Run maximum speed",0,true);
			button.m_callback = switchMaximumSpeed;
			if (m_guiHelper->getParameterInterface())
				m_guiHelper->getParameterInterface()->registerButtonParameter(
					button);
		}



		{ // create a button to switch bullet to perform interpolated substeps to speed up simulation
			// generally, interpolated steps are a good speed-up and should only be avoided if higher accuracy is needed (research purposes etc.)
			ButtonParams button("Perform interpolated substeps",0,true);
			button.m_callback = switchInterpolated;
			if (m_guiHelper->getParameterInterface())
				m_guiHelper->getParameterInterface()->registerButtonParameter(
					button);
		}

	}

	void setupAdvancedParamInterface(){

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
			SliderParams slider("Frames per second", &gFramesPerSecond);
			slider.m_minVal = 0;
			slider.m_maxVal = 200;
			slider.m_callback = twxChangeFPS;
			slider.m_clampToNotches = false;
			if (m_guiHelper->getParameterInterface())
				m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
					slider);
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
            slider.m_clampToIntegers = true;
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


	}

	void createEmptyDynamicsWorld(){ // create an empty dynamics worlds according to the chosen settings via statics (top section of code)

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
//			b3Printf("=%s=",SolverType::SEQUENTIALIMPULSESOLVER);
			m_solver = new btSequentialImpulseConstraintSolver();
			break;
		}
		case NNCGSOLVER: {
//			b3Printf("=%s=",SolverType::NNCGSOLVER);
			m_solver = new btNNCGConstraintSolver();
			break;
		}
		case DANZIGSOLVER: {
//			b3Printf("=%s=",SolverType::DANZIGSOLVER);
			btDantzigSolver* mlcp = new btDantzigSolver();
			m_solver = new btMLCPSolver(mlcp);
			break;
		}
		case GAUSSSEIDELSOLVER: {
//			b3Printf("=%s=",SolverType::GAUSSSEIDELSOLVER);
			btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel();
			m_solver = new btMLCPSolver(mlcp);
			break;
		}
		case LEMKESOLVER: {
//			b3Printf("=%s=",SolverType::LEMKESOLVER);
			btLemkeSolver* mlcp = new btLemkeSolver();
			m_solver = new btMLCPSolver(mlcp);
			break;
		}
		case FSSOLVER: {
//			b3Printf("=%s=",SolverType::FSSOLVER);
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
//			b3Printf("Using split impulse feature with ERP/TurnERP: (%f,%f)",
//				m_dynamicsWorld->getSolverInfo().m_erp2,
//				m_dynamicsWorld->getSolverInfo().m_splitImpulseTurnErp);
		}

		m_dynamicsWorld->getSolverInfo().m_numIterations = gSolverIterations; // set the number of solver iterations for iteration based solvers

		m_dynamicsWorld->setGravity(btVector3(0, -9.81f, 0)); // set gravity to -9.81

	}

	btScalar calculatePerformedSpeedup() { // calculate performed speedup
		// we calculate the performed speed up
		btScalar speedUp = ((double)performedTime*1000.0)/((double)(mLoopTimer.getTimeMilliseconds()-performanceTimestamp));
//		b3Printf("Avg Effective speedup: %f",speedUp);
		performedTime = 0;
		performanceTimestamp = mLoopTimer.getTimeMilliseconds();
		return speedUp;
	}



	void timeWarpSimulation(float deltaTime) // Override this
	{

	}

	void stepSimulation(float deltaTime){ // customly step the simulation
		do{

//			// settings
			if(mPhysicsStepsPerSecondUpdated){
				changePhysicsStepsPerSecond(gPhysicsStepsPerSecond);
				mPhysicsStepsPerSecondUpdated = false;
			}

			if(mFramesPerSecondUpdated){
				changeFPS(gFramesPerSecond);
				mFramesPerSecondUpdated = false;
			}

			if(gChangeErpCfm){
				changeERPCFM();
				gChangeErpCfm = false;
			}

			if(mSolverIterationsUpdated){
				changeSolverIterations(gSolverIterations);
				mSolverIterationsUpdated = false;
			}


			// structure according to the canonical game loop
			// http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Canonical_Game_Loop

			//##############
			// breaking conditions - if the loop should stop, then check it here

			//#############
			// model update - here you perform updates of your model, be it the physics model, the game or simulation state or anything not related to graphics and input

			timeWarpSimulation(deltaTime);
			if(mLoopTimer.getTimeSeconds() - speedUpPrintTimeStamp > 1){
				// on reset, we calculate the performed speed up
				//double speedUp = ((double)performedTime*1000.0)/((double)(mLoopTimer.getTimeMilliseconds()-performanceTimestamp));
//				b3Printf("Avg Effective speedup: %f",speedUp);
				performedTime = 0;
				performanceTimestamp = mLoopTimer.getTimeMilliseconds();
				speedUpPrintTimeStamp = mLoopTimer.getTimeSeconds();
			}

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

	virtual bool	keyboardCallback(int key, int state)
	{
		switch(key)
		{
		case '1':{
			gSimulationSpeed = SimulationSpeeds::QUARTER_SPEED;
			gMaximumSpeed = false;
			return true;
		}
		case '2':{
			gSimulationSpeed = SimulationSpeeds::HALF_SPEED;
			gMaximumSpeed = false;
			return true;
		}
		case '3':{
			gSimulationSpeed = SimulationSpeeds::NORMAL_SPEED;
			gMaximumSpeed = false;
			return true;
		}
		case '4':{
			gSimulationSpeed = SimulationSpeeds::DOUBLE_SPEED;
			gMaximumSpeed = false;
			return true;
		}
		case '5':{
			gSimulationSpeed = SimulationSpeeds::QUADRUPLE_SPEED;
			gMaximumSpeed = false;
			return true;
		}
		case '6':{
			gSimulationSpeed = SimulationSpeeds::DECUPLE_SPEED;
			gMaximumSpeed = false;
			return true;
		}
		case '7':{
			gSimulationSpeed = SimulationSpeeds::CENTUPLE_SPEED;
			gMaximumSpeed = false;
			return true;
		}
		case '8':{
			gSimulationSpeed = SimulationSpeeds::QUINCENTUPLE_SPEED;
			gMaximumSpeed = false;
			return true;
		}
		case '9':{
			gSimulationSpeed = SimulationSpeeds::MILLITUPLE_SPEED;
			gMaximumSpeed = false;
			return true;
		}
		case '0':{
			gSimulationSpeed = SimulationSpeeds::MAX_SPEED;
			gMaximumSpeed = true;
			return true;
		}
		}
		return CommonRigidBodyBase::keyboardCallback(key,state);
	}


	void changePhysicsStepsPerSecond(float physicsStepsPerSecond){ // change the simulation accuracy
		if (m_dynamicsWorld && physicsStepsPerSecond) {
			fixedPhysicsStepSizeSec = 1.0f / physicsStepsPerSecond;
			fixedPhysicsStepSizeMilli = 1000.0f / physicsStepsPerSecond;

			changeERPCFM();
		}
	}

	void changeERPCFM(){ // Change ERP/CFM appropriately to the timestep and the ERP/CFM parameters above
		if(m_dynamicsWorld){
			m_dynamicsWorld->getSolverInfo().m_erp = b3ERPCFMHelper::getERP( // set the error reduction parameter
				fixedPhysicsStepSizeSec, // step size per second
				gERPSpringK, // k of a spring in the equation F = k * x (x:position)
				gERPDamperC); // k of a damper in the equation F = k * v (v:velocity)

			m_dynamicsWorld->getSolverInfo().m_globalCfm = b3ERPCFMHelper::getCFM( // set the constraint force mixing according to the time step
				gCFMSingularityAvoidance, // singularity avoidance (if you experience unsolvable constraints, increase this value
				fixedPhysicsStepSizeSec, // steps size per second
				gCFMSpringK, // k of a spring in the equation F = k * x (x:position)
				gCFMDamperC); // k of a damper in the equation F = k * v (v:velocity)

//			b3Printf("Bullet DynamicsWorld ERP: %f",
//				m_dynamicsWorld->getSolverInfo().m_erp);

//			b3Printf("Bullet DynamicsWorld CFM: %f",
//				m_dynamicsWorld->getSolverInfo().m_globalCfm);
		}
	}

	void changeSolverIterations(int iterations){ // change the number of iterations
		m_dynamicsWorld->getSolverInfo().m_numIterations = iterations;
	}

	void changeFPS(float framesPerSecond){ // change the frames per second
		fpsStep = 1000.0f / gFramesPerSecond;
	}

	void performTrueSteps(btScalar timeStep){ // physics stepping without interpolated substeps
		int subSteps = floor((timeStep / fixedPhysicsStepSizeSec)+0.5); /**!< Calculate the number of full normal time steps we can take */

		for (int i = 0; i < subSteps; i++) { /**!< Perform the number of substeps to reach the timestep*/
			if (timeStep && m_dynamicsWorld) {
				// since we want to perform all proper steps, we perform no interpolated substeps
				int subSteps = 1;

				m_dynamicsWorld->stepSimulation(btScalar(timeStep),
					btScalar(subSteps), btScalar(fixedPhysicsStepSizeSec));
			}
		}
	}

	void performInterpolatedSteps(btScalar timeStep){ // physics stepping with interpolated substeps
		int subSteps = 1 + floor((timeStep / fixedPhysicsStepSizeSec)+0.5); /**!< Calculate the number of full normal time steps we can take, plus 1 for safety of not losing time */
		if (timeStep && m_dynamicsWorld) {

			m_dynamicsWorld->stepSimulation(btScalar(timeStep), btScalar(subSteps),
				btScalar(fixedPhysicsStepSizeSec)); /**!< Perform the number of substeps to reach the timestep*/
		}
	}

	void performMaxStep(){ // perform as many steps as possible
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


	void performSpeedStep(){ // force-perform the number of steps needed to achieve a certain speed (safe to too high speeds, meaning the application will lose time, not the physics)
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

	void renderScene() { // render the scene
		if(!gIsHeadless){ // while the simulation is not running headlessly, render to screen
			CommonRigidBodyBase::renderScene();

			if(m_dynamicsWorld->getDebugDrawer()){
				debugDraw(m_dynamicsWorld->getDebugDrawer()->getDebugMode());
			}
		}
		mIsHeadless = gIsHeadless;
	}
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

	//store old values
	bool mPhysicsStepsPerSecondUpdated;
	bool mFramesPerSecondUpdated;
	bool mSolverIterationsUpdated;
	bool mIsHeadless;
};

#endif //NN3D_WALKERS_TIME_WARP_BASE_H

