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

#include "NN3DWalkers.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class NNWalker;
class NNWalkerBrain;

#include "NN3DWalkersTimeWarpBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "../Utils/b3ReferenceFrameHelper.hpp"
#include "../Utils/b3Clock.h"
#include "../RenderingExamples/TimeSeriesCanvas.h"

// #### configurable parameters ####
#ifndef NUM_WALKER_LEGS
#define NUM_WALKER_LEGS 6 // the number of walker legs
#endif

#ifndef POPULATION_SIZE
#define POPULATION_SIZE 10 // number of walkers in the population
#endif

#ifndef EVALUATION_DURATION
#define EVALUATION_DURATION 10 // s (duration of one single evaluation)
#endif

#ifndef TIME_SERIES_MAX_Y
#define TIME_SERIES_MAX_Y 20.0f // chart maximum y
#endif

#ifndef TIME_SERIES_MIN_Y
#define TIME_SERIES_MIN_Y 0.0f // chart minimum y
#endif

// walker dimensions and properties
static btScalar gWalkerMotorStrength = 0.5f;
static btScalar gWalkerLegTargetFrequency = 3;
static btScalar gRootBodyRadius  = 0.25f;
static btScalar gRootBodyHeight = 0.1f;
static btScalar gLegRadius = 0.1f;
static btScalar gLegLength = 0.45f;
static btScalar gForeLegLength = 0.75f;
static btScalar gForeLegRadius = 0.08f;
static btScalar gParallelEvaluations = 10.0f;

// Evaluation configurable parameters
#ifndef REAP_QTY
#define REAP_QTY 0.3f 												// number of walkers reaped based on their bad performance
#endif

#ifndef SOW_CROSSOVER_QTY
#define SOW_CROSSOVER_QTY 0.2f 										// number of walkers recreated via crossover
#endif

// this means the rest of them is randomly created: REAP_QTY-SOW_CROSSOVER_QTY = NEW_RANDOM_BREED_QTY

#ifndef SOW_ELITE_QTY
#define SOW_ELITE_QTY 0.2f 											// number of walkers kept using an elitist strategy (the best performing creatures are NOT mutated at all)
#endif

#ifndef SOW_MUTATION_QTY
#define SOW_MUTATION_QTY 0.5f 										// SOW_ELITE_QTY + SOW_MUTATION_QTY + REAP_QTY = 1
#endif

#ifndef MUTATION_RATE
#define MUTATION_RATE 0.5f 											// the mutation rate of for the walker with the worst performance
#endif

#ifndef SOW_ELITE_PARTNER
#define SOW_ELITE_PARTNER 0.8f 										// the chance an elite partner is chosen for breeding
#endif

// #### debugging ####
#ifndef DRAW_INTERPENETRATIONS
#define DRAW_INTERPENETRATIONS false 								// DEBUG toggle: draw interpenetrations of a walker body
#endif

#ifndef REBUILD_WALKER
#define REBUILD_WALKER true											// if the walker should be rebuilt on mutation
#endif

#ifndef TIMESTAMP_TIME
#define TIMESTAMP_TIME 2000.0f 										// delay between speed up timestamps
#endif

// #### not to be reconfigured ####
#define BODYPART_COUNT (2 * NUM_WALKER_LEGS + 1)
#define JOINT_COUNT (BODYPART_COUNT - 1)

void* GROUND_ID = (void*)1;

#ifndef SIMD_PI_4
#define SIMD_PI_4     0.5 * SIMD_HALF_PI
#endif

#ifndef SIMD_PI_8
#define SIMD_PI_8     0.25 * SIMD_HALF_PI
#endif

class NN3DWalkersExample : public NN3DWalkersTimeWarpBase
{
	btScalar m_SimulationTime; 										// the current simulation time
	btScalar m_LastSpeedupPrintTimestamp;
	btScalar m_bestWalkerFitness; 									// to keep track of the best fitness

	btVector3 m_resetPosition; 										// initial position of an evaluation

	int 	 m_walkersInEvaluation; 								// number of walkers in evaluation
	int		 m_nextReapedIndex; 									// index of the next reaped walker
	
	btAlignedObjectArray<class NNWalker*> m_walkersInPopulation;	// walkers in the population
	btAlignedObjectArray<class NNWalkerBrain*> m_walkerBrains;		// walker cores in the population
	
	bool m_rebuildWorldNeeded; 										// if the world should be rebuilt (for determinism)

	btRigidBody* m_ground; 											// reference to ground to readd if world is rebuilt

	btOverlapFilterCallback * m_filterCallback; 					// the collision filter callback

	TimeSeriesCanvas* m_timeSeriesCanvas; 							// A plotting canvas for the walker fitnesses
public:
	NN3DWalkersExample(struct GUIHelperInterface* helper)
	:NN3DWalkersTimeWarpBase(helper),
	 // others
	 m_resetPosition(0,0,0),
	 m_SimulationTime(0),
	 m_bestWalkerFitness(0),
	 m_LastSpeedupPrintTimestamp(0),
	 m_walkersInEvaluation(0),
	 m_nextReapedIndex(0),
	 m_timeSeriesCanvas(NULL),
	 m_ground(NULL),
	 m_filterCallback(NULL)
	{
		b3Clock clock;
		srand(clock.getSystemTimeMilliseconds());					// Set random milliseconds based on system time
	}

	virtual ~NN3DWalkersExample()
	{
		//m_walkersInPopulation deallocates itself
		//m_walkerBrains deallocates itself
		delete m_timeSeriesCanvas;
	}

	/**
	 * Setup physics scene.
	 */
	void initPhysics();
	
	/**
	 * Recreate the world if necessary.
	 * @param deltaTime
	 */
	virtual void performModelUpdate(float deltaTime);

	/**
	 * Delete the world and recreate it anew.
	 */
	void recreateWorld();

	/**
	 * Shutdown physics scene.
	 */
	virtual void exitPhysics();
	
	/**
	 * Handle keyboard inputs.
	 * @param key
	 * @param state
	 * @return
	 */
	virtual bool keyboardCallback(int key, int state);

	/**
	 * Detect collisions within simulation. Used to avoid collisions happening at startup.
	 * @return
	 */
	bool detectCollisions();

	/**
	 * Reset the camera to a certain position and orientation.
	 */
	void resetCamera()
	{
		float dist = 11;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

	// Evaluation

	/**
	 * Update the simulation.
	 * @param timeSinceLastTick
	 */
	void update(const btScalar timeSinceLastTick);

	/**
	 * Update all evaluations.
	 * @param timeSinceLastTick
	 */
	void updateEvaluations(const btScalar timeSinceLastTick);

	/**
	 * Schedule new evaluations and tear down old ones.
	 */
	void scheduleEvaluations();

	/**
	 * Draw distance markings on ground.
	 */
	void drawMarkings();

	/**
	 * Reset a walker by deleting and rebuilding it.
	 * @param i
	 * @param resetPosition
	 */
	void resetWalkerAt(int i, const btVector3& resetPosition);

	// Reaper

	/**
	 * Rate all evaluations via fitness function.
	 */
	void rateEvaluations();

	/**
	 * Reap the worst performing walkers.
	 */
	void reap();

	/**
	 * Sow new walkers.
	 */
	void sow();

	/**
	 * Crossover two walkers to create an offspring.
	 * @param mother
	 * @param father
	 * @param offspring
	 */
	void crossover(NNWalker* mother, NNWalker* father, NNWalker* offspring);

	/**
	 * Mutate a walker.
	 * @param mutant
	 * @param mutationRate
	 */
	void mutate(NNWalker* mutant, btScalar mutationRate);

	/**
	 * Get a random elite walker.
	 * @return
	 */
	NNWalker* getRandomElite();

	/**
	 * Get a random non elite walker.
	 * @return
	 */
	NNWalker* getRandomNonElite();

	/**
	 * Get the next walker to be reaped.
	 * @return
	 */
	NNWalker* getNextReaped();

	/**
	 * Print walker configurations to console.
	 */
	void printWalkerConfigs();
};

static NN3DWalkersExample* nn3DWalkers = NULL;

/**
 * Neural Network Brain Weights.
 */
class NNWalkerBrain
{
	btScalar m_sensoryMotorWeights[BODYPART_COUNT*JOINT_COUNT]; // Neural Network connection weights

public:

	NNWalkerBrain(){
		randomizeSensoryMotorWeights();
	}

	void randomizeSensoryMotorWeights(){
		//initialize random weights
		for(int i = 0;i < BODYPART_COUNT;i++){
			for(int j = 0;j < JOINT_COUNT;j++){
				m_sensoryMotorWeights[i+j*BODYPART_COUNT] = ((double) rand() / (RAND_MAX))*2.0f-1.0f;
			}
		}
	}

	btScalar* getSensoryMotorWeights() {
		return m_sensoryMotorWeights;
	}
};

/**
 * Neural Networks Walker. A 6 legged robot moving controlled by a one-layer neural network which maps from leg ground touch sensors to leg joint positions.
 */
class NNWalker
{
	btDynamicsWorld*			m_ownerWorld;
	btCollisionShape*			m_shapes[BODYPART_COUNT];
	btRigidBody*				m_bodies[BODYPART_COUNT];
	btTransform					m_bodyRelativeTransforms[BODYPART_COUNT];
	btTypedConstraint*			m_joints[JOINT_COUNT];
	btHashMap<btHashPtr,int>	m_bodyTouchSensorIndexMap;
	bool 						m_touchSensors[BODYPART_COUNT];
	NNWalkerBrain*				m_brain;


	bool						m_inEvaluation;
	btScalar					m_evaluationTime;
	bool						m_reaped;
	btVector3					m_startPosition;
	int							m_index;
	btScalar					m_legUpdateAccumulator;

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* motionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,motionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		return body;
	}


public:

	NNWalker(int index,
		btDynamicsWorld* ownerWorld,
		NNWalkerBrain* core,
		const btVector3& startingPosition,
		const btScalar& rootBodyRadius,
		const btScalar& rootBodyHeight,
		const btScalar& legRadius,
		const btScalar& legLength,
		const btScalar& foreLegRadius,
		const btScalar& foreLegLength,
		bool fixedBodyPosition)
		: m_ownerWorld(ownerWorld), 												// the world the walker walks in
		  m_brain(core), 															// the evolution core
		  m_inEvaluation(false), 													// the walker is not in evaluation
		  m_evaluationTime(0), 														// reset evaluation time
		  m_reaped(false), 															// the walker is not reaped
		  m_startPosition(startingPosition), 										// the starting position of the walker
		  m_legUpdateAccumulator(0),												// variable to apply rhythmic leg position updates
		  m_index(index)
	{
		btVector3 vUp(0, 1, 0); 													// up direction in local reference frame

		NN3DWalkersExample* nnWalkersDemo = (NN3DWalkersExample*)m_ownerWorld->getWorldUserInfo();

		// Setup geometry
		m_shapes[0] = new btCapsuleShape(rootBodyRadius, rootBodyHeight); 			// torso capsule
		int i;
		for (i = 0; i < NUM_WALKER_LEGS; i++)
		{
			m_shapes[1 + 2*i] = new btCapsuleShape(legRadius, legLength); 			// leg  capsule
			m_shapes[2 + 2*i] = new btCapsuleShape(foreLegRadius, foreLegLength); 	// fore leg capsule
		}

		// Setup rigid bodies
		btScalar torsoAboveGroundHeight = foreLegLength;
		btTransform bodyOffset; bodyOffset.setIdentity();
		bodyOffset.setOrigin(startingPosition);		

		// torso
		btVector3 localTorsoPosition = btVector3(btScalar(0.), torsoAboveGroundHeight, btScalar(0.)); // torso position in local reference frame
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(localTorsoPosition);

		btTransform originTransform = transform;

		m_bodies[0] = localCreateRigidBody(btScalar(fixedBodyPosition?0.:1.), bodyOffset*transform, m_shapes[0]);
		m_ownerWorld->addRigidBody(m_bodies[0]);
		m_bodyRelativeTransforms[0] = btTransform::getIdentity();
		m_bodies[0]->setUserPointer(this);
		m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[0]), 0);

		btHingeConstraint* hingeC;

		btTransform localA, localB, localC;

		// legs
		for (i = 0; i < NUM_WALKER_LEGS; i++)
		{
			float footAngle = 2 * SIMD_PI * i / NUM_WALKER_LEGS; 					// legs are uniformly distributed around the root body
			float footYUnitPosition = sin(footAngle); 								// y position of the leg on the unit circle
			float footXUnitPosition = cos(footAngle); 								// x position of the leg on the unit circle

			transform.setIdentity();
			btVector3 legCOM = btVector3(btScalar(footXUnitPosition*(rootBodyRadius+0.5*legLength)),
										 btScalar(torsoAboveGroundHeight),
										 btScalar(footYUnitPosition*(rootBodyRadius+0.5*legLength)));
			transform.setOrigin(legCOM);

			// thigh
			btVector3 legDirection = (legCOM - localTorsoPosition).normalize();
			btVector3 kneeAxis = legDirection.cross(vUp);			
			transform.setRotation(btQuaternion(kneeAxis, SIMD_HALF_PI));
			m_bodies[1+2*i] = localCreateRigidBody(btScalar(1.), bodyOffset*transform, m_shapes[1+2*i]);
			m_bodyRelativeTransforms[1+2*i] = transform;
			m_bodies[1+2*i]->setUserPointer(this);
			m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[1+2*i]),1+2*i);

			// shin
			transform.setIdentity();
			transform.setOrigin(btVector3(btScalar(footXUnitPosition*(rootBodyRadius+legLength)),
										  btScalar(torsoAboveGroundHeight-0.5*foreLegLength),
										  btScalar(footYUnitPosition*(rootBodyRadius+legLength))));
			m_bodies[2+2*i] = localCreateRigidBody(btScalar(1.), bodyOffset*transform, m_shapes[2+2*i]);
			m_bodyRelativeTransforms[2+2*i] = transform;
			m_bodies[2+2*i]->setUserPointer(this);
			m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[2+2*i]),2+2*i);

			// hip joints
			localA.setIdentity(); localB.setIdentity();
			localA.getBasis().setEulerZYX(0,-footAngle,0);	localA.setOrigin(btVector3(btScalar(footXUnitPosition*rootBodyRadius),
																					   btScalar(0.),
																					   btScalar(footYUnitPosition*rootBodyRadius)));
			localB = b3ReferenceFrameHelper::getTransformWorldToLocal(m_bodies[1+2*i]->getWorldTransform(), b3ReferenceFrameHelper::getTransformLocalToWorld(m_bodies[0]->getWorldTransform(),localA));
			hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[1+2*i], localA, localB);
			hingeC->setLimit(btScalar(-0.75 * SIMD_PI_4), btScalar(SIMD_PI_8));
			m_joints[2*i] = hingeC;

			// knee joints
			localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
			localA.getBasis().setEulerZYX(0,-footAngle,0);	localA.setOrigin(btVector3(btScalar(footXUnitPosition*(rootBodyRadius+legLength)),
																					   btScalar(0.),
																					   btScalar(footYUnitPosition*(rootBodyRadius+legLength))));
			localB = b3ReferenceFrameHelper::getTransformWorldToLocal(m_bodies[1+2*i]->getWorldTransform(), b3ReferenceFrameHelper::getTransformLocalToWorld(m_bodies[0]->getWorldTransform(),localA));
			localC = b3ReferenceFrameHelper::getTransformWorldToLocal(m_bodies[2+2*i]->getWorldTransform(), b3ReferenceFrameHelper::getTransformLocalToWorld(m_bodies[0]->getWorldTransform(),localA));
			hingeC = new btHingeConstraint(*m_bodies[1+2*i], *m_bodies[2+2*i], localB, localC);
			hingeC->setLimit(btScalar(-SIMD_PI_8), btScalar(0.2));
			m_joints[1+2*i] = hingeC;

			//test if we cause a collision with priorly inserted bodies. This prevents the walkers from having to resolve collisions on startup

			m_ownerWorld->addRigidBody(m_bodies[1+2*i]); 								// add thigh bone
			m_ownerWorld->addConstraint(m_joints[2*i], true); 							// connect thigh bone with root

			if(nnWalkersDemo->detectCollisions()){ 										// if thigh bone causes collision, remove it again
				m_ownerWorld->removeConstraint(m_joints[2*i]); 							// disconnect thigh bone from root
				delete m_joints[2*i];
				m_joints[2*i] = NULL;

				m_ownerWorld->removeRigidBody(m_bodies[1+2*i]);
				delete m_bodies[1+2*i];
				m_bodies[1+2*i] = NULL;

				delete m_joints[1+2*i];
				m_joints[1+2*i] = NULL;
				delete m_bodies[2+2*i];
				m_bodies[2+2*i] = NULL;
			}
			else{
				m_ownerWorld->addRigidBody(m_bodies[2+2*i]); 							// add shin bone
				m_ownerWorld->addConstraint(m_joints[1+2*i], true); 					// connect shin bone with thigh

				if(nnWalkersDemo->detectCollisions()){ 									// if shin bone causes collision, remove it again
					m_ownerWorld->removeConstraint(m_joints[1+2*i]); 					// disconnect shin bone from thigh
					delete m_joints[1+2*i];
					m_joints[1+2*i] = NULL;

					m_ownerWorld->removeRigidBody(m_bodies[2+2*i]);
					delete m_bodies[2+2*i];
					m_bodies[2+2*i] = NULL;
				}
			}
		}

		// Setup some damping on the m_bodies
		for (i = 0; i < BODYPART_COUNT; ++i)
		{
			if(m_bodies[i] != NULL){
				m_bodies[i]->setDamping(0.05, 0.85);
				m_bodies[i]->setDeactivationTime(0.8);
				m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
				m_bodies[i]->setActivationState(DISABLE_DEACTIVATION);
			}
		}

		// Properly remove rigidbodies and joints
		removeJoints();
		removeRigidbodies();

		clearTouchSensors(); 															// set touch sensors to zero

		randomizeSensoryMotorWeights(); 												// set random sensory motor weights for neural network layer
	}

	virtual	~NNWalker ()
	{
		int i;

		removeFromWorld();							// remove walker from world

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			if(m_joints[i] != NULL){
				delete m_joints[i]; m_joints[i] = 0;
			}
		}

		// Remove all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			if(m_bodies[i] != NULL){
				delete m_bodies[i]->getMotionState();

				delete m_bodies[i]; m_bodies[i] = 0;
				delete m_shapes[i]; m_shapes[i] = 0;
			}
		}

		m_ownerWorld = NULL;
	}

	btTypedConstraint** getJoints() {
		return &m_joints[0];
	}

	void setTouchSensor(void* bodyPointer){
		m_touchSensors[*m_bodyTouchSensorIndexMap.find(btHashPtr(bodyPointer))] = true;
	}

	void clearTouchSensors(){
		for(int i = 0 ; i < BODYPART_COUNT;i++){
			m_touchSensors[i] = false;
		}
	}

	btScalar* getSensoryMotorWeights() {
		return m_brain->getSensoryMotorWeights();
	}

	void randomizeSensoryMotorWeights(){
		m_brain->randomizeSensoryMotorWeights();
	}

	void addToWorld() {
		if(!isInEvaluation()){

			addRigidbodies();
			addJoints();

			setInEvaluation(true);
		}
	}

	void removeFromWorld() {
		if(isInEvaluation()){

			removeJoints();
			removeRigidbodies();

			setInEvaluation(false);
		}
	}

	btVector3 getPosition() const {
		btVector3 finalPosition(0,0,0);

		int j = 0;
		for(int i = 0; i < BODYPART_COUNT;i++)
		{
			if(m_bodies[i] != NULL){
				finalPosition += m_bodies[i]->getCenterOfMassPosition();
				j++;
			}
		}

		finalPosition /= btScalar(j);
		return finalPosition;
	}

	btScalar getDistanceFitness() const
	{
		btScalar distance = 0;

		distance = (getPosition() - m_startPosition).length2();

		return distance;
	}

	btScalar getFitness() const
	{
		return getDistanceFitness(); // for now it is only distance
	}

	/**
	 * Reset walker to a position. Does not work deterministically.
	 * @param position
	 */
	void resetAt(const btVector3& position) {
		removeFromWorld();
		btTransform resetPosition(btQuaternion::getIdentity(), position);

		for (int i = 0; i < JOINT_COUNT; i++)
		{
			btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(getJoints()[i]);
			hingeC->enableAngularMotor(false,0,0);
		}

		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->clearForces();
			m_bodies[i]->setAngularVelocity(btVector3(0,0,0));
			m_bodies[i]->setLinearVelocity(btVector3(0,0,0));

			m_bodies[i]->setWorldTransform(resetPosition*m_bodyRelativeTransforms[i]);
			if(m_bodies[i]->getMotionState()){
				m_bodies[i]->getMotionState()->setWorldTransform(resetPosition*m_bodyRelativeTransforms[i]);
			}
		}

		m_startPosition = position; // the starting position of the walker

		m_legUpdateAccumulator = 0;

		clearTouchSensors();
	}

	btScalar getEvaluationTime() const {
		return m_evaluationTime;
	}

	void setEvaluationTime(btScalar evaluationTime) {
		m_evaluationTime = evaluationTime;
	}

	bool isInEvaluation() const {
		return m_inEvaluation;
	}

	void setInEvaluation(bool inEvaluation) {
		m_inEvaluation = inEvaluation;
	}

	bool isReaped() const {
		return m_reaped;
	}

	void setReaped(bool reaped) {
		m_reaped = reaped;
	}

	int getIndex() const {
		return m_index;
	}

	bool getTouchSensor(int i){
		return m_touchSensors[i];
	}

	NNWalkerBrain* getBrain(){
		return m_brain;
	}

	btRigidBody** getBodies(){
		return &m_bodies[0];
	}

	btScalar getLegUpdateAccumulator() const {
		return m_legUpdateAccumulator;
	}

	void setLegUpdateAccumulator(btScalar legUpdateAccumulator) {
		m_legUpdateAccumulator = legUpdateAccumulator;
	}

	void setOwnerWorld(btDynamicsWorld* ownerWorld) {
		m_ownerWorld = ownerWorld;
	}

private:
	void addJoints(){
		// add all constraints
		for (int i = 0; i < JOINT_COUNT; ++i)
		{
			if(m_joints[i] != NULL){
				m_ownerWorld->addConstraint(m_joints[i], true); // important! If you add constraints back, you must set bullet physics to disable collision between constrained bodies
			}
		}
	}

	void addRigidbodies() {
		// add all bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			if(m_bodies[i] != NULL){
				m_ownerWorld->addRigidBody(m_bodies[i]);
			}
		}
	}

	void removeJoints() {
		// remove all constraints
		for (int i = 0; i < JOINT_COUNT; ++i)
		{
			if(m_joints[i] != NULL){
				m_ownerWorld->removeConstraint(m_joints[i]);
			}
		}
	}

	void removeRigidbodies() {
		// remove all bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			if(m_bodies[i] != NULL){
				m_ownerWorld->removeRigidBody(m_bodies[i]);
			}
		}
	}
};

void evaluationUpdatePreTickCallback(btDynamicsWorld *world, btScalar timeStep);

/**
 * Draw leg contacts on ground.
 * @param cp
 * @param body0
 * @param body1
 * @return
 */
bool legContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1)
{
    btCollisionObject* o1 = static_cast<btCollisionObject*>(body0);
    btCollisionObject* o2 = static_cast<btCollisionObject*>(body1);

    void* ID1 = o1->getUserPointer();
    void* ID2 = o2->getUserPointer();

	if (ID1 != GROUND_ID || ID2 != GROUND_ID) { // if one of the contacts is not ground
	    // Make a circle with a 0.9 radius at (0,0,0)
	    // with RGB color (1,0,0).
		if(nn3DWalkers->m_dynamicsWorld->getDebugDrawer() != NULL && !nn3DWalkers->mIsHeadless){
			nn3DWalkers->m_dynamicsWorld->getDebugDrawer()->drawSphere(cp.getPositionWorldOnA(), 0.1, btVector3(1., 0., 0.));
		}

		if(ID1 != GROUND_ID && ID1){ // if ID1 is not ground
			((NNWalker*)ID1)->setTouchSensor(o1);
		}

		if(ID2 != GROUND_ID && ID2){ // if ID2 is not ground
			((NNWalker*)ID2)->setTouchSensor(o2);
		}
	}
    return false;
}

/**
 * A filter avoiding collision between walkers.
 */
struct WalkerFilterCallback : public btOverlapFilterCallback // avoids collisions between walkers
{
	// return true when pairs need collision
	virtual bool	needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
	{
		btCollisionObject* obj0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
		btCollisionObject* obj1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);

		if (obj0->getUserPointer() == GROUND_ID || obj1->getUserPointer() == GROUND_ID) { // everything collides with ground
			return true;
		}
		else if((NNWalker*)obj0->getUserPointer() && (NNWalker*)obj1->getUserPointer()){
			return ((NNWalker*)obj0->getUserPointer())->getIndex() == ((NNWalker*)obj1->getUserPointer())->getIndex();
		}
		return true;
	}
};

void NN3DWalkersExample::initPhysics()
{

	setupBasicParamInterface(); // parameter interface to use timewarp

	gContactProcessedCallback = legContactProcessedCallback;

	m_guiHelper->setUpAxis(1);

	// Setup the basic world

	m_SimulationTime = 0;

	gWalkerLegTargetFrequency = 3; // Hz

	m_walkersInPopulation.resize(POPULATION_SIZE, NULL);
	m_walkerBrains.resize(POPULATION_SIZE, NULL);

	recreateWorld(); // setup world and add ground

	{ // Setup a big ground box
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));
		m_ground = createRigidBody(btScalar(0.),groundTransform,groundShape);
		m_ground->setFriction(5);
		m_ground->setUserPointer(GROUND_ID);
	}

	// new SIMD solver for joints clips accumulated impulse, so the new limits for the motor
	// should be (numberOfsolverIterations * oldLimits)
	gWalkerMotorStrength = 0.05f * m_dynamicsWorld->getSolverInfo().m_numIterations;

	{ // create a slider to change the motor update frequency
		SliderParams slider("Motor update frequency", &gWalkerLegTargetFrequency);
		slider.m_minVal = 0;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	{ // create a slider to change the motor torque
		SliderParams slider("Motor force", &gWalkerMotorStrength);
		slider.m_minVal = 1;
		slider.m_maxVal = 50;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	
	{ // create a slider to change the root body radius
		SliderParams slider("Root body radius", &gRootBodyRadius);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	{ // create a slider to change the root body height
		SliderParams slider("Root body height", &gRootBodyHeight);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	{ // create a slider to change the leg radius
		SliderParams slider("Leg radius", &gLegRadius);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	{ // create a slider to change the leg length
		SliderParams slider("Leg length", &gLegLength);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	{ // create a slider to change the fore leg radius
		SliderParams slider("Fore Leg radius", &gForeLegRadius);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	{ // create a slider to change the fore leg length
		SliderParams slider("Fore Leg length", &gForeLegLength);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	if(POPULATION_SIZE > 1)
	{ // create a slider to change the number of parallel evaluations
		SliderParams slider("Parallel evaluations", &gParallelEvaluations);
		slider.m_minVal = 1;
		slider.m_maxVal = POPULATION_SIZE;
		slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	// setup data sources for walkers in time series canvas
	m_timeSeriesCanvas = new TimeSeriesCanvas(m_guiHelper->getAppInterface()->m_2dCanvasInterface,400,300, "Fitness Performance");
	m_timeSeriesCanvas->setupTimeSeries(TIME_SERIES_MIN_Y, TIME_SERIES_MAX_Y, 10, 0);
	for(int i = 0; i < POPULATION_SIZE ; i++){
		m_timeSeriesCanvas->addDataSource(" ", 100*i/POPULATION_SIZE, 100*(POPULATION_SIZE-i)/POPULATION_SIZE, 100*i/POPULATION_SIZE);
	}
}

void NN3DWalkersExample::performModelUpdate(float deltaTime){
	if(m_rebuildWorldNeeded){ // rebuilding the world must be performed in sync to the update cycle of the example browser
		recreateWorld();
		m_rebuildWorldNeeded = false;
	}
}

/**
 * This method should really delete all remainings of the simulation except for the walker brains.
 */
void NN3DWalkersExample::recreateWorld(){

	for(int i = 0; i < POPULATION_SIZE ; i++){ 												// remove and delete walkers
		if(m_walkersInPopulation[i] != NULL){
			m_walkersInPopulation[i]->removeFromWorld();

			for(int j = 0;j < BODYPART_COUNT;j++){ 											// remove all body graphics
				btRigidBody* rb = static_cast<btRigidBody*>(m_walkersInPopulation[i]->getBodies()[j]);
				if (rb != NULL) {
					int graphicsUid = rb->getUserIndex();
					m_guiHelper->removeGraphicsInstance(graphicsUid);
				}
			}

			delete m_walkersInPopulation[i];
			m_walkersInPopulation[i] = NULL;
		}
	}

	if(m_dynamicsWorld != NULL && m_ground != NULL){
		m_dynamicsWorld->removeRigidBody(m_ground); 										// remove ground
	}

	// delete world etc.
	if(m_dynamicsWorld != NULL){
		delete m_dynamicsWorld;
		m_dynamicsWorld = 0;
	}

	if(m_solver != NULL){
		delete m_solver;
		m_solver = 0;
	}

	if(m_broadphase != NULL){
		delete m_broadphase;
		m_broadphase = 0;
	}

	if(m_dispatcher != NULL){
		delete m_dispatcher;
		m_dispatcher = 0;
	}

	if(m_collisionConfiguration != NULL){
		delete m_collisionConfiguration;
		m_collisionConfiguration = 0;
	}

	createEmptyDynamicsWorld(); // create new world

	// add all filters and callbacks
	m_dynamicsWorld->setInternalTickCallback(evaluationUpdatePreTickCallback, this, true); 	// set evolution update pretick callback
	m_filterCallback = new WalkerFilterCallback();
	m_dynamicsWorld->getPairCache()->setOverlapFilterCallback(m_filterCallback); 			// avoid collisions between walkers
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(0);

	if(m_dynamicsWorld != NULL && m_ground != NULL){
		m_dynamicsWorld->addRigidBody(m_ground); 											// readd ground
	}
}

bool NN3DWalkersExample::detectCollisions()
{
	bool collisionDetected = false;
	if (m_dynamicsWorld) {
		m_dynamicsWorld->performDiscreteCollisionDetection(); 							// let the collisions be calculated

		int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
		for (int i = 0; i < numManifolds; i++) 												// for each collision
		{
			btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			// get collision objects of collision
			const btCollisionObject* obA = contactManifold->getBody0();
			const btCollisionObject* obB = contactManifold->getBody1();

			if (obA->getUserPointer() != GROUND_ID && obB->getUserPointer() != GROUND_ID && obA->getUserPointer() == obB->getUserPointer()) { 	// if one of them is not ground

				int numContacts = contactManifold->getNumContacts();
				for (int j = 0; j < numContacts; j++) 											// for each collision contact
				{
					collisionDetected = true;
					btManifoldPoint& pt = contactManifold->getContactPoint(j);
					if (pt.getDistance() < 0.f) 											// if collision is penetrating
					{
						const btVector3& ptA = pt.getPositionWorldOnA();
						const btVector3& ptB = pt.getPositionWorldOnB();
						const btVector3& normalOnB = pt.m_normalWorldOnB;

						if (!DRAW_INTERPENETRATIONS) {										// if no interpenetrations are drawn, break
							break;
						}

						if (m_dynamicsWorld->getDebugDrawer() && !mIsHeadless) { 				// draw self collisions
							m_dynamicsWorld->getDebugDrawer()->drawSphere(pt.getPositionWorldOnA(), 0.1, btVector3(0., 0., 1.));
							m_dynamicsWorld->getDebugDrawer()->drawSphere(pt.getPositionWorldOnB(), 0.1, btVector3(0., 0., 1.));
						}
					}
				}
			}
		}
	}

	return collisionDetected;
}

bool NN3DWalkersExample::keyboardCallback(int key, int state)
{
	switch (key)
	{
	case '[':
		gWalkerMotorStrength /= 1.1f;
		return true;
	case ']':
		gWalkerMotorStrength *= 1.1f;
		return true;
	case 'l':
//		printWalkerConfigs();
		return true;
	default:
		break;
	}

	return NN3DWalkersTimeWarpBase::keyboardCallback(key,state);
}

void NN3DWalkersExample::exitPhysics()
{

	gContactProcessedCallback = NULL; // clear contact processed callback on exiting

	for (int i = 0;i < POPULATION_SIZE;i++)
	{
		NNWalker* walker = m_walkersInPopulation[i];
		delete walker;
	}

	CommonRigidBodyBase::exitPhysics();
}

class CommonExampleInterface* ET_NN3DWalkersCreateFunc(struct CommonExampleOptions& options)
{
	nn3DWalkers = new NN3DWalkersExample(options.m_guiHelper);
	return nn3DWalkers;
}

bool fitnessComparator (const NNWalker* a, const NNWalker* b)
{
    return a->getFitness() > b->getFitness(); // sort walkers descending
}

void NN3DWalkersExample::rateEvaluations(){

	m_walkersInPopulation.quickSort(fitnessComparator); // Sort walkers by fitness

	b3Printf("Best performing walker: %f meters", btSqrt(m_walkersInPopulation[0]->getDistanceFitness()));

	// if not all walkers are reaped and the best walker is worse than is had been in the previous round
	if((POPULATION_SIZE-1)*(1-REAP_QTY) != 0 && btSqrt(m_walkersInPopulation[0]->getDistanceFitness()) < m_bestWalkerFitness){
		b3Printf("################Simulation not deterministic###########################");
	}
	else{
		m_bestWalkerFitness = btSqrt(m_walkersInPopulation[0]->getDistanceFitness());
	}

	for(int i = 0; i < POPULATION_SIZE;i++){ // plot walker fitnesses for this round
		m_timeSeriesCanvas->insertDataAtCurrentTime(btSqrt(m_walkersInPopulation[i]->getDistanceFitness()),i,true);
	}
	m_timeSeriesCanvas->nextTick(); // move to next tick of graph

	for(int i = 0; i < POPULATION_SIZE;i++){ // reset all walkers
		m_walkersInPopulation[i]->setEvaluationTime(0);
	}
	m_nextReapedIndex = 0;
}

void NN3DWalkersExample::reap() {
	int reaped = 0;
	for(int i = POPULATION_SIZE-1;i >=(POPULATION_SIZE-1)*(1-REAP_QTY); i--){ // reap a certain percentage of walkers to replace them afterwards
		m_walkersInPopulation[i]->setReaped(true);
		reaped++;
	}
	b3Printf("%i Walker(s) reaped.",reaped);
}

/**
 * Return a random elitist walker (one that is not mutated at all because it performs well).
 * @return Random elitist walker.
 */
NNWalker* NN3DWalkersExample::getRandomElite(){
	return m_walkersInPopulation[((POPULATION_SIZE-1) * SOW_ELITE_QTY) * (rand()/RAND_MAX)];
}

/**
 * Return a random non-elitist walker (a mutated walker).
 * @return
 */
NNWalker* NN3DWalkersExample::getRandomNonElite(){
	return m_walkersInPopulation[(POPULATION_SIZE-1) * SOW_ELITE_QTY + (POPULATION_SIZE-1) * (1.0f-SOW_ELITE_QTY) * (rand()/RAND_MAX)];
}

/**
 * Get the next reaped walker to be replaced.
 * @return
 */
NNWalker* NN3DWalkersExample::getNextReaped() {
	if((POPULATION_SIZE-1) - m_nextReapedIndex >= (POPULATION_SIZE-1) * (1-REAP_QTY)){
		m_nextReapedIndex++;
	}

	if(m_walkersInPopulation[(POPULATION_SIZE-1) - m_nextReapedIndex+1]->isReaped()){
		return m_walkersInPopulation[(POPULATION_SIZE-1) - m_nextReapedIndex+1];
	}
	else{
		return NULL; // we asked for too many
	}

}

/**
 * Sow new walkers.
 */
void NN3DWalkersExample::sow() {
	int sow = 0;
	for(int i = 0; i < POPULATION_SIZE * (SOW_CROSSOVER_QTY);i++){ // create number of new crossover creatures
		sow++;
		NNWalker* mother = getRandomElite(); // Get elite partner (mother)
		NNWalker* father = (SOW_ELITE_PARTNER < rand()/RAND_MAX)?getRandomElite():getRandomNonElite(); //Get elite or random partner (father)
		NNWalker* offspring = getNextReaped();
		crossover(mother,father, offspring);
	}

	for(int i = POPULATION_SIZE*SOW_ELITE_QTY; i < POPULATION_SIZE*(SOW_ELITE_QTY+SOW_MUTATION_QTY);i++){ // create mutants
		mutate(m_walkersInPopulation[i], btScalar(MUTATION_RATE / (POPULATION_SIZE * SOW_MUTATION_QTY) * (i-POPULATION_SIZE*SOW_ELITE_QTY)));
	}

	for(int i = 0; i < (POPULATION_SIZE-1) * (REAP_QTY-SOW_CROSSOVER_QTY);i++){
		sow++;
		NNWalker* reaped = getNextReaped();
		reaped->setReaped(false);
		reaped->randomizeSensoryMotorWeights();
	}
	b3Printf("%i Walker(s) sown.",sow);
}

/**
 * Crossover mother and father into the child.
 * @param mother
 * @param father
 * @param child
 */
void NN3DWalkersExample::crossover(NNWalker* mother, NNWalker* father, NNWalker* child) {
	for(int i = 0; i < BODYPART_COUNT*JOINT_COUNT;i++){
		btScalar random = ((double) rand() / (RAND_MAX));

		if(random >= 0.5f){
			child->getSensoryMotorWeights()[i] = mother->getSensoryMotorWeights()[i];
		}
		else
		{
			child->getSensoryMotorWeights()[i] = father->getSensoryMotorWeights()[i];
		}
	}
}

/**
 * Mutate the mutant.
 * @param mutant
 * @param mutationRate
 */
void NN3DWalkersExample::mutate(NNWalker* mutant, btScalar mutationRate) {
	for(int i = 0; i < BODYPART_COUNT*JOINT_COUNT;i++){
		btScalar random = ((double) rand() / (RAND_MAX));

		if(random >= mutationRate){
			mutant->getSensoryMotorWeights()[i] = ((double) rand() / (RAND_MAX))*2.0f-1.0f;
		}
	}
}

/**
 * Update the demo via pretick callback to be precise.
 * @param world
 * @param timeStep
 */
void evaluationUpdatePreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	NN3DWalkersExample* nnWalkersDemo = (NN3DWalkersExample*)world->getWorldUserInfo();
	nnWalkersDemo->update(timeStep);
}

/**
 * Update cycle.
 * @param timeSinceLastTick
 */
void NN3DWalkersExample::update(const btScalar timeSinceLastTick) {
	updateEvaluations(timeSinceLastTick); /**!< We update all evaluations that are in the loop */

	scheduleEvaluations(); /**!< Start new evaluations and finish the old ones. */

	drawMarkings(); /**!< Draw markings on the ground */

	if(m_SimulationTime > m_LastSpeedupPrintTimestamp + TIMESTAMP_TIME){ // print effective speedup every x seconds
		b3Printf("Avg Effective speedup: %f real time",calculatePerformedSpeedup());
		m_LastSpeedupPrintTimestamp = m_SimulationTime;
	}
}

/**
 * Update the evaluations.
 * @param timeSinceLastTick
 */
void NN3DWalkersExample::updateEvaluations(const btScalar timeSinceLastTick) {
	btScalar delta = timeSinceLastTick;
	btScalar minFPS = 1.f/60.f;
	if (delta > minFPS){
		delta = minFPS;
	}

	m_SimulationTime += delta;

	for(int r = 0; r < POPULATION_SIZE;r++) // evaluation time passes
	{
		if(m_walkersInPopulation[r] != NULL && m_walkersInPopulation[r]->isInEvaluation()){
			m_walkersInPopulation[r]->setEvaluationTime(m_walkersInPopulation[r]->getEvaluationTime()+delta); // increase evaluation time

			m_walkersInPopulation[r]->setLegUpdateAccumulator(m_walkersInPopulation[r]->getLegUpdateAccumulator() + delta);

			if(m_walkersInPopulation[r]->getLegUpdateAccumulator() >= btScalar(1.0f) /gWalkerLegTargetFrequency) // we update the leg movement with leg target frequency
			{
				m_walkersInPopulation[r]->setLegUpdateAccumulator(0);

				for (int i = 0; i < 2*NUM_WALKER_LEGS; i++)
				{
					btScalar targetAngle = 0; // angle in range [0,1]
					btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_walkersInPopulation[r]->getJoints()[i]);


					if(hingeC != NULL){ // if hinge exists
						for(int j = 0; j < JOINT_COUNT;j++){ // accumulate sensor inputs with weights (summate inputs)
							targetAngle += m_walkersInPopulation[r]->getSensoryMotorWeights()[i+j*BODYPART_COUNT] * m_walkersInPopulation[r]->getTouchSensor(i);
						}

						targetAngle = (tanh(targetAngle)+1.0f)*0.5f; // apply the activation function (threshold) [0;1]

						btScalar targetLimitAngle 	= hingeC->getLowerLimit() + targetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit()); // [lowerLimit;upperLimit]
						btScalar currentAngle      	= hingeC->getHingeAngle();
						btScalar angleError  		= targetLimitAngle - currentAngle; // target current delta
						btScalar desiredAngularVel = angleError/((delta>0)?delta:btScalar(0.0001f)); // division by zero safety

						hingeC->enableAngularMotor(true, desiredAngularVel, gWalkerMotorStrength); // set new target velocity
					}
				}
			}

			// clear sensor signals after usage
			m_walkersInPopulation[r]->clearTouchSensors();
		}
	}
}

/**
 * Schedule the walker evaluations.
 */
void NN3DWalkersExample::scheduleEvaluations() {
	for(int i = 0; i < POPULATION_SIZE;i++){

		if(m_walkersInPopulation[i] != NULL && m_walkersInPopulation[i]->isInEvaluation() && m_walkersInPopulation[i]->getEvaluationTime() >= EVALUATION_DURATION){ /**!< tear down evaluations */
			b3Printf("An evaluation finished at %f s. Distance: %f m", m_SimulationTime, btSqrt(m_walkersInPopulation[i]->getDistanceFitness()));
			m_walkersInPopulation[i]->removeFromWorld();
			m_walkersInEvaluation--;
		}

		if(m_walkersInEvaluation < gParallelEvaluations && (m_walkersInPopulation[i] == NULL || (!m_walkersInPopulation[i]->isInEvaluation() && m_walkersInPopulation[i]->getEvaluationTime() == 0))){ /**!< Setup the new evaluations */
			b3Printf("An evaluation started at %f s.", m_SimulationTime);
			m_walkersInEvaluation++;

			if(REBUILD_WALKER){ 									// deletes and recreates the walker in the position
				m_guiHelper->removeAllGraphicsInstances();
				m_ground->setUserIndex(-1); 						// reset to get a new graphics object
				m_ground->setUserIndex2(-1); 						// reset to get a new graphics object
				m_ground->getCollisionShape()->setUserIndex(-1); 	// reset to get a new graphics object

				resetWalkerAt(i, m_resetPosition);
			}
			else{ 													// resets the position of the walker without deletion
				m_walkersInPopulation[i]->resetAt(m_resetPosition);
			}
			m_walkersInPopulation[i]->addToWorld();
		}
	}

	if(!mIsHeadless){ 												// after all changes, regenerate graphics objects
		m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	}

	if(m_walkersInEvaluation == 0){ 								// if there are no more evaluations possible
		if(REBUILD_WALKER){
			m_rebuildWorldNeeded = true;
		}

		rateEvaluations(); 											// rate evaluations by sorting them based on their fitness

		reap(); 													// reap worst performing walkers

		sow(); 														// crossover, mutate and sow new walkers
		b3Printf("### A new generation started. ###");
	}
}

/**
 * Reset the walker at position.
 * @param i
 * @param resetPosition
 */
void NN3DWalkersExample::resetWalkerAt(int i, const btVector3& resetPosition){

	// either copy core or create a new one if no previous existing
	m_walkerBrains[i] = (m_walkerBrains[i] != NULL)?m_walkerBrains[i]:new NNWalkerBrain();

	NNWalker* newWalker = new NNWalker(i,
		m_dynamicsWorld,
		m_walkerBrains[i],
		resetPosition,
		gRootBodyRadius,
		gRootBodyHeight,
		gLegRadius,
		gLegLength,
		gForeLegRadius,
		gForeLegLength,
		false);

	if(m_walkersInPopulation[i] != NULL){
		delete m_walkersInPopulation[i];
	}

	m_walkersInPopulation[i] = newWalker;
}

/**
 * Draw distance markings on the ground.
 */
void NN3DWalkersExample::drawMarkings() {
	if(!mIsHeadless){												// if the system is not running headless
		for(int i = 0; i < POPULATION_SIZE;i++) 					// draw current distance plates of moving walkers
		{
			if(m_walkersInPopulation[i]->isInEvaluation()){
				btVector3 walkerPosition = m_walkersInPopulation[i]->getPosition();
				char performance[20];
				sprintf(performance, "%.2f m", btSqrt(m_walkersInPopulation[i]->getDistanceFitness()));
				m_guiHelper->drawText3D(performance,
					walkerPosition.x(),
					walkerPosition.y() + 1,
					walkerPosition.z(),
					1);
			}
		}

		if(m_dynamicsWorld->getDebugDrawer()){
			for(int i = 2; i < 50; i += 2){ 						// draw distance circles
				m_dynamicsWorld->getDebugDrawer()->drawArc(
					btVector3(0,0,0),
					btVector3(0,1,0),
					btVector3(1,0,0),
					btScalar(i), btScalar(i), btScalar(0),
					btScalar(SIMD_2_PI),
					btVector3(10*i,0,0),
					false);
			}
		}
	}
}

/**
 * Print walker neural network layer configurations.
*/
//void NN3DWalkersExample::printWalkerConfigs(){
//	char configString[25 + POPULATION_SIZE*BODYPART_COUNT*JOINT_COUNT*(3+15+1) + POPULATION_SIZE*4 + 1]; // 15 precision + [],\n
//	char* runner = configString;
//	sprintf(runner,"Population configuration:");
//	runner +=25;
//	for(int i = 0;i < POPULATION_SIZE;i++) {
//		runner[0] = '\n';
//		runner++;
//		runner[0] = '[';
//		runner++;
//		for(int j = 0; j < BODYPART_COUNT*JOINT_COUNT;j++) {
//			sprintf(runner,"%.15f", m_walkersInPopulation[i]->getSensoryMotorWeights()[j]);
//			runner +=15;
//			if(j + 1 < BODYPART_COUNT*JOINT_COUNT){
//				runner[0] = ',';
//			}
//			else{
//				runner[0] = ']';
//			}
//			runner++;
//		}
//	}
//	runner[0] = '\0';
//	b3Printf(configString);
//}
