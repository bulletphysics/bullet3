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

#include "NN3DWalkersTimeWarpBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "../Utils/b3ReferenceFrameHelper.hpp"
#include "../RenderingExamples/TimeSeriesCanvas.h"

static btScalar gRootBodyRadius  = 0.25f;
static btScalar gRootBodyHeight = 0.1f;
static btScalar gLegRadius = 0.1f;
static btScalar gLegLength = 0.45f;
static btScalar gForeLegLength = 0.75f;
static btScalar gForeLegRadius = 0.08f;

static btScalar gParallelEvaluations = 10.0f;

#ifndef SIMD_PI_4
#define SIMD_PI_4     0.5 * SIMD_HALF_PI
#endif

#ifndef SIMD_PI_8
#define SIMD_PI_8     0.25 * SIMD_HALF_PI
#endif

#ifndef RANDOM_MOVEMENT
#define RANDOM_MOVEMENT false
#endif

#ifndef RANDOMIZE_DIMENSIONS
#define RANDOMIZE_DIMENSIONS false
#endif

#ifndef NUM_WALKERS
#define NUM_WALKERS 50
#endif

#ifndef EVALUATION_TIME
#define EVALUATION_TIME 10 // s
#endif

#ifndef REAP_QTY
#define REAP_QTY 0.3f // number of walkers reaped based on their bad performance
#endif

#ifndef SOW_CROSSOVER_QTY
#define SOW_CROSSOVER_QTY 0.2f // this means REAP_QTY-SOW_CROSSOVER_QTY = NEW_RANDOM_BREED_QTY
#endif

#ifndef SOW_ELITE_QTY
#define SOW_ELITE_QTY 0.2f // number of walkers kept using an elitist strategy
#endif

#ifndef SOW_MUTATION_QTY
#define SOW_MUTATION_QTY 0.5f // SOW_ELITE_QTY + SOW_MUTATION_QTY + REAP_QTY = 1
#endif

#ifndef MUTATION_RATE
#define MUTATION_RATE 0.5f // the mutation rate of for the walker with the worst performance
#endif

#ifndef SOW_ELITE_PARTNER
#define SOW_ELITE_PARTNER 0.8f
#endif

#define NUM_LEGS 6
#define BODYPART_COUNT (2 * NUM_LEGS + 1)
#define JOINT_COUNT (BODYPART_COUNT - 1)
#define DRAW_INTERPENETRATIONS false

void* GROUND_ID = (void*)1;

class NN3DWalkersExample : public NN3DWalkersTimeWarpBase
{
	btScalar m_Time;
	btScalar m_SpeedupTimestamp;
	btScalar m_targetAccumulator;
	btScalar m_targetFrequency;
	btScalar m_motorStrength;
	int 	 m_evaluationsQty;
	int		 m_nextReaped;
	
	btAlignedObjectArray<class NNWalker*> m_walkersInPopulation;
	
	TimeSeriesCanvas* m_timeSeriesCanvas;

public:
	NN3DWalkersExample(struct GUIHelperInterface* helper)
	:NN3DWalkersTimeWarpBase(helper),
	 m_Time(0),
	 m_SpeedupTimestamp(0),
	 m_targetAccumulator(0),
	m_targetFrequency(3),
	m_motorStrength(0.5f),
	m_evaluationsQty(0),
	 m_nextReaped(0),
	 m_timeSeriesCanvas(0)
	{
	}

	virtual ~NN3DWalkersExample()
	{
		delete m_timeSeriesCanvas;
	}

	void initPhysics();
	
	virtual void exitPhysics();
	
	void spawnWalker(int index, const btVector3& startOffset, bool bFixed);
	
	virtual bool	keyboardCallback(int key, int state);

	bool detectCollisions();

	void resetCamera()
	{
		float dist = 11;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

	// Evaluation

	void update(const btScalar timeSinceLastTick);

	void updateEvaluations(const btScalar timeSinceLastTick);

	void scheduleEvaluations();

	void drawMarkings();

	// Reaper

	void rateEvaluations();

	void reap();

	void sow();

	void crossover(NNWalker* mother, NNWalker* father, NNWalker* offspring);

	void mutate(NNWalker* mutant, btScalar mutationRate);

	NNWalker* getRandomElite();

	NNWalker* getRandomNonElite();

	NNWalker* getNextReaped();

	void printWalkerConfigs();

};

static NN3DWalkersExample* nn3DWalkers = NULL;

class NNWalker
{
	btDynamicsWorld*	m_ownerWorld;
	btCollisionShape*	m_shapes[BODYPART_COUNT];
	btRigidBody*		m_bodies[BODYPART_COUNT];
	btTransform			m_bodyRelativeTransforms[BODYPART_COUNT];
	btTypedConstraint*	m_joints[JOINT_COUNT];
	btHashMap<btHashPtr,int>	m_bodyTouchSensorIndexMap;
	bool 				m_touchSensors[BODYPART_COUNT];
	btScalar			m_sensoryMotorWeights[BODYPART_COUNT*JOINT_COUNT];

	bool				m_inEvaluation;
	btScalar			m_evaluationTime;
	bool				m_reaped;
	btVector3			m_startPosition;
	int					m_index;

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

	void randomizeSensoryMotorWeights(){
		//initialize random weights
		for(int i = 0;i < BODYPART_COUNT;i++){
			for(int j = 0;j < JOINT_COUNT;j++){
				m_sensoryMotorWeights[i+j*BODYPART_COUNT] = ((double) rand() / (RAND_MAX))*2.0f-1.0f;
			}
		}
	}

	NNWalker(int index, btDynamicsWorld* ownerWorld, const btVector3& positionOffset, bool bFixed)
		: m_ownerWorld (ownerWorld),
		  m_inEvaluation(false),
		  m_evaluationTime(0),
		  m_reaped(false)
	{
		m_index = index;
		btVector3 vUp(0, 1, 0); // up in local reference frame

		NN3DWalkersExample* nnWalkersDemo = (NN3DWalkersExample*)m_ownerWorld->getWorldUserInfo();

		randomizeSensoryMotorWeights();

		//
		// Setup geometry
		m_shapes[0] = new btCapsuleShape(gRootBodyRadius, gRootBodyHeight); // root body capsule
		int i;
		for ( i=0; i<NUM_LEGS; i++)
		{
			m_shapes[1 + 2*i] = new btCapsuleShape(gLegRadius, gLegLength); // leg  capsule
			m_shapes[2 + 2*i] = new btCapsuleShape(gForeLegRadius, gForeLegLength); // fore leg capsule
		}

		//
		// Setup rigid bodies
		btScalar rootAboveGroundHeight = gForeLegLength;
		btTransform bodyOffset; bodyOffset.setIdentity();
		bodyOffset.setOrigin(positionOffset);		

		// root body
		btVector3 localRootBodyPosition = btVector3(btScalar(0.), rootAboveGroundHeight, btScalar(0.)); // root body position in local reference frame
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(localRootBodyPosition);

		btTransform originTransform = transform;

		m_bodies[0] = localCreateRigidBody(btScalar(bFixed?0.:1.), bodyOffset*transform, m_shapes[0]);
		m_ownerWorld->addRigidBody(m_bodies[0]);
		m_bodyRelativeTransforms[0] = btTransform::getIdentity();
		m_bodies[0]->setUserPointer(this);
		m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[0]), 0);

		btHingeConstraint* hingeC;
		//btConeTwistConstraint* coneC;

		btTransform localA, localB, localC;

		// legs
		for (i = 0; i < NUM_LEGS; i++)
		{
			float footAngle = 2 * SIMD_PI * i / NUM_LEGS; // legs are uniformly distributed around the root body
			float footYUnitPosition = sin(footAngle); // y position of the leg on the unit circle
			float footXUnitPosition = cos(footAngle); // x position of the leg on the unit circle

			transform.setIdentity();
			btVector3 legCOM = btVector3(btScalar(footXUnitPosition*(gRootBodyRadius+0.5*gLegLength)), btScalar(rootAboveGroundHeight), btScalar(footYUnitPosition*(gRootBodyRadius+0.5*gLegLength)));
			transform.setOrigin(legCOM);

			// thigh
			btVector3 legDirection = (legCOM - localRootBodyPosition).normalize();
			btVector3 kneeAxis = legDirection.cross(vUp);			
			transform.setRotation(btQuaternion(kneeAxis, SIMD_HALF_PI));
			m_bodies[1+2*i] = localCreateRigidBody(btScalar(1.), bodyOffset*transform, m_shapes[1+2*i]);
			m_bodyRelativeTransforms[1+2*i] = transform;
			m_bodies[1+2*i]->setUserPointer(this);
			m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[1+2*i]),1+2*i);

			// shin
			transform.setIdentity();
			transform.setOrigin(btVector3(btScalar(footXUnitPosition*(gRootBodyRadius+gLegLength)), btScalar(rootAboveGroundHeight-0.5*gForeLegLength), btScalar(footYUnitPosition*(gRootBodyRadius+gLegLength))));
			m_bodies[2+2*i] = localCreateRigidBody(btScalar(1.), bodyOffset*transform, m_shapes[2+2*i]);
			m_bodyRelativeTransforms[2+2*i] = transform;
			m_bodies[2+2*i]->setUserPointer(this);
			m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[2+2*i]),2+2*i);

			// hip joints
			localA.setIdentity(); localB.setIdentity();
			localA.getBasis().setEulerZYX(0,-footAngle,0);	localA.setOrigin(btVector3(btScalar(footXUnitPosition*gRootBodyRadius), btScalar(0.), btScalar(footYUnitPosition*gRootBodyRadius)));
			localB = b3ReferenceFrameHelper::getTransformWorldToLocal(m_bodies[1+2*i]->getWorldTransform(), b3ReferenceFrameHelper::getTransformLocalToWorld(m_bodies[0]->getWorldTransform(),localA));
			hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[1+2*i], localA, localB);
			hingeC->setLimit(btScalar(-0.75 * SIMD_PI_4), btScalar(SIMD_PI_8));
			//hingeC->setLimit(btScalar(-0.1), btScalar(0.1));
			m_joints[2*i] = hingeC;

			// knee joints
			localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
			localA.getBasis().setEulerZYX(0,-footAngle,0);	localA.setOrigin(btVector3(btScalar(footXUnitPosition*(gRootBodyRadius+gLegLength)), btScalar(0.), btScalar(footYUnitPosition*(gRootBodyRadius+gLegLength))));
			localB = b3ReferenceFrameHelper::getTransformWorldToLocal(m_bodies[1+2*i]->getWorldTransform(), b3ReferenceFrameHelper::getTransformLocalToWorld(m_bodies[0]->getWorldTransform(),localA));
			localC = b3ReferenceFrameHelper::getTransformWorldToLocal(m_bodies[2+2*i]->getWorldTransform(), b3ReferenceFrameHelper::getTransformLocalToWorld(m_bodies[0]->getWorldTransform(),localA));
			hingeC = new btHingeConstraint(*m_bodies[1+2*i], *m_bodies[2+2*i], localB, localC);
			//hingeC->setLimit(btScalar(-0.01), btScalar(0.01));
			hingeC->setLimit(btScalar(-SIMD_PI_8), btScalar(0.2));
			m_joints[1+2*i] = hingeC;

			m_ownerWorld->addRigidBody(m_bodies[1+2*i]); // add thigh bone

			m_ownerWorld->addConstraint(m_joints[2*i], true); // connect thigh bone with root

			if(nnWalkersDemo->detectCollisions()){ // if thigh bone causes collision, remove it again
				m_ownerWorld->removeRigidBody(m_bodies[1+2*i]);
				m_ownerWorld->removeConstraint(m_joints[2*i]); // disconnect thigh bone from root
			}
			else{

				m_ownerWorld->addRigidBody(m_bodies[2+2*i]); // add shin bone
				m_ownerWorld->addConstraint(m_joints[1+2*i], true); // connect shin bone with thigh

				if(nnWalkersDemo->detectCollisions()){ // if shin bone causes collision, remove it again
					m_ownerWorld->removeRigidBody(m_bodies[2+2*i]);
					m_ownerWorld->removeConstraint(m_joints[1+2*i]); // disconnect shin bone from thigh
				}
			}
		}

		// Setup some damping on the m_bodies
		for (i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			//m_bodies[i]->setSleepingThresholds(1.6, 2.5);
			m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
		}

		removeFromWorld(); // it should not yet be in the world
	}

	virtual	~NNWalker ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			
			delete m_bodies[i]->getMotionState();

			delete m_bodies[i]; m_bodies[i] = 0;
			delete m_shapes[i]; m_shapes[i] = 0;
		}
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

	bool getTouchSensor(int i){
		return m_touchSensors[i];
	}

	btScalar* getSensoryMotorWeights() {
		return m_sensoryMotorWeights;
	}

	void addToWorld() {
		int i;
		// add all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->addRigidBody(m_bodies[i]);
		}

		// add all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->addConstraint(m_joints[i], true); // important! If you add constraints back, you must set bullet physics to disable collision between constrained bodies
		}
		m_startPosition = getPosition();
	}

	void removeFromWorld(){
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
		}

		// Remove all bodies
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
		}
	}

	btVector3 getPosition() const {
		btVector3 finalPosition(0,0,0);

		for(int i = 0; i < BODYPART_COUNT;i++)
		{
			finalPosition += m_bodies[i]->getCenterOfMassPosition();
		}

		finalPosition /= BODYPART_COUNT;
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

	void resetAt(const btVector3& position) {
		btTransform resetPosition(btQuaternion::getIdentity(), position);
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setWorldTransform(resetPosition*m_bodyRelativeTransforms[i]);
			if(m_bodies[i]->getMotionState()){
				m_bodies[i]->getMotionState()->setWorldTransform(resetPosition*m_bodyRelativeTransforms[i]);
			}
			m_bodies[i]->clearForces();
			m_bodies[i]->setAngularVelocity(btVector3(0,0,0));
			m_bodies[i]->setLinearVelocity(btVector3(0,0,0));

		}

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
};

void evaluationUpdatePreTickCallback(btDynamicsWorld *world, btScalar timeStep);

bool legContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1)
{
    btCollisionObject* o1 = static_cast<btCollisionObject*>(body0);
    btCollisionObject* o2 = static_cast<btCollisionObject*>(body1);

    void* ID1 = o1->getUserPointer();
    void* ID2 = o2->getUserPointer();

	if (ID1 != GROUND_ID || ID2 != GROUND_ID) {
	    // Make a circle with a 0.9 radius at (0,0,0)
	    // with RGB color (1,0,0).
		if(nn3DWalkers->m_dynamicsWorld->getDebugDrawer() != NULL){
			if(!nn3DWalkers->mIsHeadless){
				nn3DWalkers->m_dynamicsWorld->getDebugDrawer()->drawSphere(cp.getPositionWorldOnA(), 0.1, btVector3(1., 0., 0.));
			}
		}

		if(ID1 != GROUND_ID && ID1){
			((NNWalker*)ID1)->setTouchSensor(o1);
		}

		if(ID2 != GROUND_ID && ID2){
			((NNWalker*)ID2)->setTouchSensor(o2);
		}
	}
    return false;
}

struct WalkerFilterCallback : public btOverlapFilterCallback
{
	// return true when pairs need collision
	virtual bool	needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
	{
		btCollisionObject* obj0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
		btCollisionObject* obj1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);

		if (obj0->getUserPointer() == GROUND_ID || obj1->getUserPointer() == GROUND_ID) { // everything collides with ground
			return true;
		}
		else{
			return ((NNWalker*)obj0->getUserPointer())->getIndex() == ((NNWalker*)obj1->getUserPointer())->getIndex();
		}
	}
};

void NN3DWalkersExample::initPhysics()
{

	setupBasicParamInterface(); // parameter interface to use timewarp

	gContactProcessedCallback = legContactProcessedCallback;

	m_guiHelper->setUpAxis(1);

	// Setup the basic world

	m_Time = 0;

	createEmptyDynamicsWorld();

	m_dynamicsWorld->setInternalTickCallback(evaluationUpdatePreTickCallback, this, true);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	m_targetFrequency = 3;

	// new SIMD solver for joints clips accumulated impulse, so the new limits for the motor
	// should be (numberOfsolverIterations * oldLimits)
	m_motorStrength = 0.05f * m_dynamicsWorld->getSolverInfo().m_numIterations;


	{ // create a slider to change the motor update frequency
		SliderParams slider("Motor update frequency", &m_targetFrequency);
		slider.m_minVal = 0;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the motor torque
		SliderParams slider("Motor force", &m_motorStrength);
		slider.m_minVal = 1;
		slider.m_maxVal = 50;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}
	
	{ // create a slider to change the root body radius
		SliderParams slider("Root body radius", &gRootBodyRadius);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the root body height
		SliderParams slider("Root body height", &gRootBodyHeight);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the leg radius
		SliderParams slider("Leg radius", &gLegRadius);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the leg length
		SliderParams slider("Leg length", &gLegLength);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the fore leg radius
		SliderParams slider("Fore Leg radius", &gForeLegRadius);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the fore leg length
		SliderParams slider("Fore Leg length", &gForeLegLength);
		slider.m_minVal = 0.01f;
		slider.m_maxVal = 10;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the number of parallel evaluations
		SliderParams slider("Parallel evaluations", &gParallelEvaluations);
		slider.m_minVal = 1;
		slider.m_maxVal = NUM_WALKERS;
        slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}


	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));
		btRigidBody* ground = createRigidBody(btScalar(0.),groundTransform,groundShape);
		ground->setFriction(5);
		ground->setUserPointer(GROUND_ID);
	}

	for(int i = 0; i < NUM_WALKERS ; i++){
		if(RANDOMIZE_DIMENSIONS){
			float maxDimension = 0.2f;

			// randomize the dimensions
			gRootBodyRadius  = ((double) rand() / (RAND_MAX)) * (maxDimension-0.01f) + 0.01f;
			gRootBodyHeight = ((double) rand() / (RAND_MAX)) * (maxDimension-0.01f) + 0.01f;
			gLegRadius = ((double) rand() / (RAND_MAX)) * (maxDimension-0.01f) + 0.01f;
			gLegLength = ((double) rand() / (RAND_MAX)) * (maxDimension-0.01f) + 0.01f;
			gForeLegLength = ((double) rand() / (RAND_MAX)) * (maxDimension-0.01f) + 0.01f;
			gForeLegRadius = ((double) rand() / (RAND_MAX)) * (maxDimension-0.01f) + 0.01f;
		}

		// Spawn one walker
		btVector3 offset(0,0,0);
		spawnWalker(i, offset, false);
	}

	btOverlapFilterCallback * filterCallback = new WalkerFilterCallback();
	m_dynamicsWorld->getPairCache()->setOverlapFilterCallback(filterCallback);

	m_timeSeriesCanvas = new TimeSeriesCanvas(m_guiHelper->getAppInterface()->m_2dCanvasInterface,300,200, "Fitness Performance");
	m_timeSeriesCanvas ->setupTimeSeries(40, NUM_WALKERS*EVALUATION_TIME, 0);
	for(int i = 0; i < NUM_WALKERS ; i++){
		m_timeSeriesCanvas->addDataSource(" ", 100*i/NUM_WALKERS,100*(NUM_WALKERS-i)/NUM_WALKERS,100*(i)/NUM_WALKERS);
	}
}


void NN3DWalkersExample::spawnWalker(int index, const btVector3& startOffset, bool bFixed)
{
	NNWalker* walker = new NNWalker(index, m_dynamicsWorld, startOffset, bFixed);
	m_walkersInPopulation.push_back(walker);
}

bool NN3DWalkersExample::detectCollisions()
{
	bool collisionDetected = false;
	if(m_dynamicsWorld){
		m_dynamicsWorld->performDiscreteCollisionDetection(); // let the collisions be calculated
	}

	int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0;i < numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();

		if(obA->getUserPointer() != GROUND_ID && obB->getUserPointer() != GROUND_ID){

			int numContacts = contactManifold->getNumContacts();
			for (int j=0;j<numContacts;j++)
			{
				collisionDetected = true;
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				if (pt.getDistance()<0.f)
				{
					//const btVector3& ptA = pt.getPositionWorldOnA();
					//const btVector3& ptB = pt.getPositionWorldOnB();
					//const btVector3& normalOnB = pt.m_normalWorldOnB;

					if(!DRAW_INTERPENETRATIONS){
						return collisionDetected;
					}

					if(m_dynamicsWorld->getDebugDrawer()){
						m_dynamicsWorld->getDebugDrawer()->drawSphere(pt.getPositionWorldOnA(), 0.1, btVector3(0., 0., 1.));
						m_dynamicsWorld->getDebugDrawer()->drawSphere(pt.getPositionWorldOnB(), 0.1, btVector3(0., 0., 1.));
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
		m_motorStrength /= 1.1f;
		return true;
	case ']':
		m_motorStrength *= 1.1f;
		return true;
	case 'l':
		printWalkerConfigs();
		return true;
	default:
		break;
	}

	return NN3DWalkersTimeWarpBase::keyboardCallback(key,state);
}

void NN3DWalkersExample::exitPhysics()
{

	gContactProcessedCallback = NULL; // clear contact processed callback on exiting

	int i;

	for (i = 0;i < NUM_WALKERS;i++)
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

	for(int i = 0; i < NUM_WALKERS;i++){
		m_timeSeriesCanvas->insertDataAtCurrentTime(btSqrt(m_walkersInPopulation[i]->getDistanceFitness()),0,true);
	}
	m_timeSeriesCanvas->nextTick();

	for(int i = 0; i < NUM_WALKERS;i++){
		m_walkersInPopulation[i]->setEvaluationTime(0);
	}
	m_nextReaped = 0;
}

void NN3DWalkersExample::reap() {
	int reaped = 0;
	for(int i = NUM_WALKERS-1;i >=(NUM_WALKERS-1)*(1-REAP_QTY); i--){ // reap a certain percentage
		m_walkersInPopulation[i]->setReaped(true);
		reaped++;
		b3Printf("%i Walker(s) reaped.",reaped);
	}
}

NNWalker* NN3DWalkersExample::getRandomElite(){
	return m_walkersInPopulation[((NUM_WALKERS-1) * SOW_ELITE_QTY) * (rand()/RAND_MAX)];
}

NNWalker* NN3DWalkersExample::getRandomNonElite(){
	return m_walkersInPopulation[(NUM_WALKERS-1) * SOW_ELITE_QTY + (NUM_WALKERS-1) * (1.0f-SOW_ELITE_QTY) * (rand()/RAND_MAX)];
}

NNWalker* NN3DWalkersExample::getNextReaped() {
	if((NUM_WALKERS-1) - m_nextReaped >= (NUM_WALKERS-1) * (1-REAP_QTY)){
		m_nextReaped++;
	}

	if(m_walkersInPopulation[(NUM_WALKERS-1) - m_nextReaped+1]->isReaped()){
		return m_walkersInPopulation[(NUM_WALKERS-1) - m_nextReaped+1];
	}
	else{
		return NULL; // we asked for too many
	}

}

void NN3DWalkersExample::sow() {
	int sow = 0;
	for(int i = 0; i < NUM_WALKERS * (SOW_CROSSOVER_QTY);i++){ // create number of new crossover creatures
		sow++;
		b3Printf("%i Walker(s) sown.",sow);
		NNWalker* mother = getRandomElite(); // Get elite partner (mother)
		NNWalker* father = (SOW_ELITE_PARTNER < rand()/RAND_MAX)?getRandomElite():getRandomNonElite(); //Get elite or random partner (father)
		NNWalker* offspring = getNextReaped();
		crossover(mother,father, offspring);
	}

	for(int i = NUM_WALKERS*SOW_ELITE_QTY; i < NUM_WALKERS*(SOW_ELITE_QTY+SOW_MUTATION_QTY);i++){ // create mutants
		mutate(m_walkersInPopulation[i], btScalar(MUTATION_RATE / (NUM_WALKERS * SOW_MUTATION_QTY) * (i-NUM_WALKERS*SOW_ELITE_QTY)));
	}

	for(int i = 0; i < (NUM_WALKERS-1) * (REAP_QTY-SOW_CROSSOVER_QTY);i++){
		sow++;
		b3Printf("%i Walker(s) sown.",sow);
		NNWalker* reaped = getNextReaped();
		reaped->setReaped(false);
		reaped->randomizeSensoryMotorWeights();
	}
}

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

void NN3DWalkersExample::mutate(NNWalker* mutant, btScalar mutationRate) {
	for(int i = 0; i < BODYPART_COUNT*JOINT_COUNT;i++){
		btScalar random = ((double) rand() / (RAND_MAX));

		if(random >= mutationRate){
			mutant->getSensoryMotorWeights()[i] = ((double) rand() / (RAND_MAX))*2.0f-1.0f;
		}
	}
}

void evaluationUpdatePreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	NN3DWalkersExample* nnWalkersDemo = (NN3DWalkersExample*)world->getWorldUserInfo();

	nnWalkersDemo->update(timeStep);
}

void NN3DWalkersExample::update(const btScalar timeSinceLastTick) {
	updateEvaluations(timeSinceLastTick); /**!< We update all evaluations that are in the loop */

	scheduleEvaluations(); /**!< Start new evaluations and finish the old ones. */

	drawMarkings(); /**!< Draw markings on the ground */

	if(m_Time > m_SpeedupTimestamp + 2.0f){ // print effective speedup
		b3Printf("Avg Effective speedup: %f real time",calculatePerformedSpeedup());
		m_SpeedupTimestamp = m_Time;
	}
}

void NN3DWalkersExample::updateEvaluations(const btScalar timeSinceLastTick) {
	btScalar delta = timeSinceLastTick;
	btScalar minFPS = 1.f/60.f;
	if (delta > minFPS){
		delta = minFPS;
	}

	m_Time += delta;

	m_targetAccumulator += delta;

	for(int i = 0; i < NUM_WALKERS;i++) // evaluation time passes
	{
		if(m_walkersInPopulation[i]->isInEvaluation()){
			m_walkersInPopulation[i]->setEvaluationTime(m_walkersInPopulation[i]->getEvaluationTime()+delta); // increase evaluation time
		}
	}

	if(m_targetAccumulator >= 1.0f /((double)m_targetFrequency))
	{
		m_targetAccumulator = 0;

		for (int r=0; r<NUM_WALKERS; r++)
		{
			if(m_walkersInPopulation[r]->isInEvaluation())
			{
				for (int i = 0; i < 2*NUM_LEGS; i++)
				{
					btScalar targetAngle = 0;
					btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_walkersInPopulation[r]->getJoints()[i]);

					if(RANDOM_MOVEMENT){
						targetAngle   = ((double) rand() / (RAND_MAX));
					}
					else{ // neural network movement

						// accumulate sensor inputs with weights
						for(int j = 0; j < JOINT_COUNT;j++){
							targetAngle += m_walkersInPopulation[r]->getSensoryMotorWeights()[i+j*BODYPART_COUNT] * m_walkersInPopulation[r]->getTouchSensor(i);
						}

						// apply the activation function
						targetAngle = (tanh(targetAngle)+1.0f)*0.5f;
					}
					btScalar targetLimitAngle 	= hingeC->getLowerLimit() + targetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
					btScalar currentAngle      	= hingeC->getHingeAngle();
					btScalar angleError  		= targetLimitAngle - currentAngle;
					btScalar desiredAngularVel = 0;
					if(delta){
						desiredAngularVel = angleError/delta;
					}
					else{
						desiredAngularVel = angleError/0.0001f;
					}
					hingeC->enableAngularMotor(true, desiredAngularVel, m_motorStrength);
				}

				// clear sensor signals after usage
				m_walkersInPopulation[r]->clearTouchSensors();
			}
		}
	}
}

void NN3DWalkersExample::scheduleEvaluations() {
	for(int i = 0; i < NUM_WALKERS;i++){

		if(m_walkersInPopulation[i]->isInEvaluation() && m_walkersInPopulation[i]->getEvaluationTime() >= EVALUATION_TIME){ /**!< tear down evaluations */
			b3Printf("An evaluation finished at %f s. Distance: %f m", m_Time, btSqrt(m_walkersInPopulation[i]->getDistanceFitness()));
			m_walkersInPopulation[i]->setInEvaluation(false);
			m_walkersInPopulation[i]->removeFromWorld();
			m_evaluationsQty--;
		}

		if(m_evaluationsQty < gParallelEvaluations && !m_walkersInPopulation[i]->isInEvaluation() && m_walkersInPopulation[i]->getEvaluationTime() == 0){ /**!< Setup the new evaluations */
			b3Printf("An evaluation started at %f s.",m_Time);
			m_evaluationsQty++;
			m_walkersInPopulation[i]->setInEvaluation(true);

			if(m_walkersInPopulation[i]->getEvaluationTime() == 0){ // reset to origin if the evaluation did not yet run
				m_walkersInPopulation[i]->resetAt(btVector3(0,0,0));
			}

			m_walkersInPopulation[i]->addToWorld();
			m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
		}
	}

	if(m_evaluationsQty == 0){ // if there are no more evaluations possible
		rateEvaluations(); // rate evaluations by sorting them based on their fitness

		reap(); // reap worst performing walkers

		sow(); // crossover & mutate and sow new walkers
		b3Printf("### A new generation started. ###");
	}
}

void NN3DWalkersExample::drawMarkings() {
	if(!mIsHeadless){
		for(int i = 0; i < NUM_WALKERS;i++) // draw current distance plates of moving walkers
		{
			if(m_walkersInPopulation[i]->isInEvaluation()){
				btVector3 walkerPosition = m_walkersInPopulation[i]->getPosition();
				char performance[20];
				sprintf(performance, "%.2f m", btSqrt(m_walkersInPopulation[i]->getDistanceFitness()));
				m_guiHelper->drawText3D(performance,walkerPosition.x(),walkerPosition.y()+1,walkerPosition.z(),1);
			}
		}

		for(int i = 2; i < 50; i+=2){ // draw distance circles
			if(m_dynamicsWorld->getDebugDrawer()){
				m_dynamicsWorld->getDebugDrawer()->drawArc(btVector3(0,0,0),btVector3(0,1,0),btVector3(1,0,0),btScalar(i), btScalar(i),btScalar(0),btScalar(SIMD_2_PI),btVector3(10*i,0,0),false);
			}
		}
	}
}

void NN3DWalkersExample::printWalkerConfigs(){
#if 0
	char configString[25 + NUM_WALKERS*BODYPART_COUNT*JOINT_COUNT*(3+15+1) + NUM_WALKERS*4 + 1]; // 15 precision + [],\n
	char* runner = configString;
	sprintf(runner,"Population configuration:");
	runner +=25;
	for(int i = 0;i < NUM_WALKERS;i++) {
		runner[0] = '\n';
		runner++;
		runner[0] = '[';
		runner++;
		for(int j = 0; j < BODYPART_COUNT*JOINT_COUNT;j++) {
			sprintf(runner,"%.15f", m_walkersInPopulation[i]->getSensoryMotorWeights()[j]);
			runner +=15;
			if(j + 1 < BODYPART_COUNT*JOINT_COUNT){
				runner[0] = ',';
			}
			else{
				runner[0] = ']';
			}
			runner++;
		}
	}
	runner[0] = '\0';
	b3Printf(configString);
#endif
}
