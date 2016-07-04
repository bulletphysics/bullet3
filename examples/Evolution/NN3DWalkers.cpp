/*
Bullet Continuous Collision Detection and Physics Library Copyright (c) 2007 Erwin Coumans
Motor Demo

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
#include <map>

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

//TODO: Maybe add pointworldToLocal and AxisWorldToLocal etc. to a helper class

btVector3 getPointWorldToLocal(btTransform localObjectCenterOfMassTransform, btVector3 point);
btVector3 getAxisWorldToLocal(btTransform localObjectCenterOfMassTransform, btVector3 axis);

btVector3 getPointLocalToWorld(btTransform localObjectCenterOfMassTransform, btVector3 point);
btVector3 getAxisLocalToWorld(btTransform localObjectCenterOfMassTransform, btVector3 axis);

btTransform getTransformLocalToWorld(btTransform localObjectCenterOfMassTransform, btTransform transform);
btTransform getTransformWorldToLocal(btTransform localObjectCenterOfMassTransform, btTransform transform);

class NN3DWalkers : public CommonRigidBodyBase
{
	btScalar m_Time;
	btScalar m_targetAccumulator;
	btScalar m_targetFrequency;
	btScalar m_motorStrength;
	
	btAlignedObjectArray<class NNWalker*> m_walkers;
	
	
public:
	NN3DWalkers(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper), m_Time(0),m_motorStrength(0.5f),m_targetFrequency(3),m_targetAccumulator(0)
	{

	}
	void initPhysics();
	
	virtual void exitPhysics();

	virtual ~NN3DWalkers()
	{
	}
	
	void spawnWalker(const btVector3& startOffset, bool bFixed);
	
	virtual bool	keyboardCallback(int key, int state);
	
	void setMotorTargets(btScalar deltaTime);
	
	void resetCamera()
	{
		float dist = 11;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

	virtual void renderScene();
};

static NN3DWalkers* nn3DWalkers = NULL;

#ifndef SIMD_PI_4
#define SIMD_PI_4     0.5 * SIMD_HALF_PI
#endif

#ifndef SIMD_PI_8
#define SIMD_PI_8     0.25 * SIMD_HALF_PI
#endif

void* GROUND_ID = (void*)1;
bool RANDOM_MOVEMENT = false;

#define NUM_LEGS 6
#define BODYPART_COUNT (2 * NUM_LEGS + 1)
#define JOINT_COUNT (BODYPART_COUNT - 1)

class NNWalker
{
	btDynamicsWorld*	m_ownerWorld;
	btCollisionShape*	m_shapes[BODYPART_COUNT];
	btRigidBody*		m_bodies[BODYPART_COUNT];
	btTypedConstraint*	m_joints[JOINT_COUNT];
	std::map<void*,int>			m_body_index_map;
	bool 				m_touch_sensors[BODYPART_COUNT];
	float 				m_sensory_motor_weights[BODYPART_COUNT*JOINT_COUNT];

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* motionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,motionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}


public:
	NNWalker(btDynamicsWorld* ownerWorld, const btVector3& positionOffset, bool bFixed)
		: m_ownerWorld (ownerWorld)
	{
		btVector3 vUp(0, 1, 0); // up in local reference frame

		//initialize random weights
		for(int i = 0;i < BODYPART_COUNT;i++){
			for(int j = 0;j < JOINT_COUNT;j++){
				m_sensory_motor_weights[i+j*BODYPART_COUNT] = ((double) rand() / (RAND_MAX))*2.0f-1.0f;
			}
		}

		//
		// Setup geometry
		//
		float rootBodyRadius  = 0.25f;
		float rootBodyHeight = 0.1f;
		float legRadius = 0.1f;
		float legLength = 0.45f;
		float foreLegLength = 0.75f;
		float foreLegRadius = 0.08f;
		m_shapes[0] = new btCapsuleShape(btScalar(rootBodyRadius), btScalar(rootBodyHeight));
		int i;
		for ( i=0; i<NUM_LEGS; i++)
		{
			m_shapes[1 + 2*i] = new btCapsuleShape(btScalar(legRadius), btScalar(legLength)); // leg  capsule
			m_shapes[2 + 2*i] = new btCapsuleShape(btScalar(foreLegRadius), btScalar(foreLegLength)); // fore leg capsule
		}

		//
		// Setup rigid bodies
		//
		float footHeight = 0.5;
		btTransform bodyOffset; bodyOffset.setIdentity();
		bodyOffset.setOrigin(positionOffset);		

		// root body
		btVector3 localRootBodyPosition = btVector3(btScalar(0.), btScalar(footHeight), btScalar(0.)); // root body position in local reference frame
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(localRootBodyPosition);
		if (bFixed) // if fixed body
		{
			m_bodies[0] = localCreateRigidBody(btScalar(0.), bodyOffset*transform, m_shapes[0]);
		} else
		{
			m_bodies[0] = localCreateRigidBody(btScalar(1.), bodyOffset*transform, m_shapes[0]);
		}

		btHingeConstraint* hingeC;
		//btConeTwistConstraint* coneC;

		btTransform localA, localB, localC;

		// legs
		for ( i=0; i<NUM_LEGS; i++)
		{
			float footAngle = 2 * SIMD_PI * i / NUM_LEGS; // legs are uniformly distributed around the root body
			float footYUnitPosition = sin(footAngle); // y position of the leg on the unit circle
			float footXUnitPosition = cos(footAngle); // x position of the leg on the unit circle

			transform.setIdentity();
			btVector3 legCOM = btVector3(btScalar(footXUnitPosition*(rootBodyRadius+0.5*legLength)), btScalar(footHeight), btScalar(footYUnitPosition*(rootBodyRadius+0.5*legLength)));
			transform.setOrigin(legCOM);

			// thigh
			btVector3 legDirection = (legCOM - localRootBodyPosition).normalize();
			btVector3 kneeAxis = legDirection.cross(vUp);			
			transform.setRotation(btQuaternion(kneeAxis, SIMD_HALF_PI));
			m_bodies[1+2*i] = localCreateRigidBody(btScalar(1.), bodyOffset*transform, m_shapes[1+2*i]);

			// shin
			transform.setIdentity();
			transform.setOrigin(btVector3(btScalar(footXUnitPosition*(rootBodyRadius+legLength)), btScalar(footHeight-0.5*foreLegLength), btScalar(footYUnitPosition*(rootBodyRadius+legLength))));
			m_bodies[2+2*i] = localCreateRigidBody(btScalar(1.), bodyOffset*transform, m_shapes[2+2*i]);

			//
			// Setup the constraints
			//

			// hip joints
			localA.setIdentity(); localB.setIdentity();
			localA.getBasis().setEulerZYX(0,-footAngle,0);	localA.setOrigin(btVector3(btScalar(footXUnitPosition*rootBodyRadius), btScalar(0.), btScalar(footYUnitPosition*rootBodyRadius)));
			localB = getTransformWorldToLocal(m_bodies[1+2*i]->getWorldTransform(), getTransformLocalToWorld(m_bodies[0]->getWorldTransform(),localA));
			hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[1+2*i], localA, localB);
			hingeC->setLimit(btScalar(-0.75 * SIMD_PI_4), btScalar(SIMD_PI_8));
			//hingeC->setLimit(btScalar(-0.1), btScalar(0.1));
			m_joints[2*i] = hingeC;
			m_ownerWorld->addConstraint(m_joints[2*i], true);

			// knee joints
			localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
			localA.getBasis().setEulerZYX(0,-footAngle,0);	localA.setOrigin(btVector3(btScalar(footXUnitPosition*(rootBodyRadius+legLength)), btScalar(0.), btScalar(footYUnitPosition*(rootBodyRadius+legLength))));
			localB = getTransformWorldToLocal(m_bodies[1+2*i]->getWorldTransform(), getTransformLocalToWorld(m_bodies[0]->getWorldTransform(),localA));
			localC = getTransformWorldToLocal(m_bodies[2+2*i]->getWorldTransform(), getTransformLocalToWorld(m_bodies[0]->getWorldTransform(),localA));
			hingeC = new btHingeConstraint(*m_bodies[1+2*i], *m_bodies[2+2*i], localB, localC);
			//hingeC->setLimit(btScalar(-0.01), btScalar(0.01));
			hingeC->setLimit(btScalar(-SIMD_PI_8), btScalar(0.2));
			m_joints[1+2*i] = hingeC;
			m_ownerWorld->addConstraint(m_joints[1+2*i], true);
		}

		// Setup some damping on the m_bodies
		for (i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			//m_bodies[i]->setSleepingThresholds(1.6, 2.5);
			m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
			m_bodies[i]->setUserPointer(this);
			m_body_index_map.insert(std::pair<void*,int>(m_bodies[i],i));

		}
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

	btTypedConstraint** getJoints() {return &m_joints[0];}

	void setTouchSensor(void* bodyPointer){
		m_touch_sensors[m_body_index_map.at(bodyPointer)] = true;
	}

	void clearTouchSensors(){
		for(int i = 0 ; i < BODYPART_COUNT;i++){
			m_touch_sensors[i] = false;
		}
	}

	bool getTouchSensor(int i){ return m_touch_sensors[i];}

	const float* getSensoryMotorWeights() const {
		return m_sensory_motor_weights;
	}
};



void legMotorPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	NN3DWalkers* motorDemo = (NN3DWalkers*)world->getWorldUserInfo();

	motorDemo->setMotorTargets(timeStep);
	
}

bool legContactProcessedCallback(btManifoldPoint& cp,
                                void* body0, void* body1)
{
    btCollisionObject* o1 = static_cast<btCollisionObject*>(body0);
    btCollisionObject* o2 = static_cast<btCollisionObject*>(body1);

    void* ID1 = o1->getUserPointer();
    void* ID2 = o2->getUserPointer();

	if (ID2 != GROUND_ID || ID1 != GROUND_ID) {
	    // Make a circle with a 0.9 radius at (0,0,0)
	    // with RGB color (1,0,0).
		if(nn3DWalkers->m_dynamicsWorld->getDebugDrawer() != NULL)
			nn3DWalkers->m_dynamicsWorld->getDebugDrawer()->drawSphere(cp.getPositionWorldOnA(), 0.1, btVector3(1., 0., 0.));

		if(ID1 != GROUND_ID){
			((NNWalker*)ID1)->setTouchSensor(o1);
		}

		if(ID2 != GROUND_ID){
			((NNWalker*)ID2)->setTouchSensor(o2);
		}
	}
    return false;
}



void NN3DWalkers::initPhysics()
{
	gContactProcessedCallback = legContactProcessedCallback;

	m_guiHelper->setUpAxis(1);

	// Setup the basic world

	m_Time = 0;

	createEmptyDynamicsWorld();

	m_dynamicsWorld->setInternalTickCallback(legMotorPreTickCallback,this,true);
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

	for(int i = 0; i < 5 ; i++){
		for(int j = 0; j < 5; j++){
			// Spawn one walker
			btVector3 spacing(10.0f,0.8f,10.0f);
			btVector3 startOffset(spacing * btVector3(i,0,j));
			spawnWalker(startOffset, false);
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void NN3DWalkers::spawnWalker(const btVector3& startOffset, bool bFixed)
{
	NNWalker* walker = new NNWalker(m_dynamicsWorld, startOffset, bFixed);
	m_walkers.push_back(walker);
}

void NN3DWalkers::setMotorTargets(btScalar deltaTime)
{

	float ms = deltaTime*1000000.;
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	m_Time += ms;

	m_targetAccumulator +=ms;

	if(m_targetAccumulator >= 1000000.0f /((double)m_targetFrequency))
	{
		m_targetAccumulator = 0;
		//
		// set per-frame sinusoidal position targets using angular motor (hacky?)
		//
		for (int r=0; r<m_walkers.size(); r++)
		{
			for (int i=0; i<2*NUM_LEGS; i++)
			{
				btScalar targetAngle = 0;
				btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_walkers[r]->getJoints()[i]);

				if(RANDOM_MOVEMENT){
					targetAngle   = ((double) rand() / (RAND_MAX));//0.5 * (1 + sin(2 * SIMD_PI * fTargetPercent+ i* SIMD_PI/NUM_LEGS));
				}
				else{

					// accumulate sensor inputs with weights
					for(int j = 0; j < JOINT_COUNT;j++){
						targetAngle += m_walkers[r]->getSensoryMotorWeights()[i+j*BODYPART_COUNT] * m_walkers[r]->getTouchSensor(i);
					}

					// apply the activation function
					targetAngle = (tanh(targetAngle)+1.0f)*0.5f;
				}
				btScalar targetLimitAngle = hingeC->getLowerLimit() + targetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
				btScalar currentAngle      = hingeC->getHingeAngle();
				btScalar angleError  = targetLimitAngle - currentAngle;
				btScalar desiredAngularVel = 1000000.f * angleError/ms;
				hingeC->enableAngularMotor(true, desiredAngularVel, m_motorStrength);
			}

			// clear sensor signals after usage
			m_walkers[r]->clearTouchSensors();
		}
	}
}

bool NN3DWalkers::keyboardCallback(int key, int state)
{
	switch (key)
	{
	case '[':
		m_motorStrength /= 1.1f;
		return true;
		break;
	case ']':
		m_motorStrength *= 1.1f;
		return true;
		break;
	default:
		break;
	}

	return false;
}



void NN3DWalkers::exitPhysics()
{

	int i;

	for (i=0;i<m_walkers.size();i++)
	{
		NNWalker* walker = m_walkers[i];
		delete walker;
	}

	CommonRigidBodyBase::exitPhysics();
}

void NN3DWalkers::renderScene()
	{
		m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);

		m_guiHelper->render(m_dynamicsWorld);

		debugDraw(m_dynamicsWorld->getDebugDrawer()->getDebugMode());
	}

class CommonExampleInterface*    ET_NN3DWalkersCreateFunc(struct CommonExampleOptions& options)
{
	nn3DWalkers = new NN3DWalkers(options.m_guiHelper);
	return nn3DWalkers;
}


btVector3 getPointWorldToLocal( btTransform localObjectCenterOfMassTransform, btVector3 point) {
	  return localObjectCenterOfMassTransform.inverse() * point; // transforms the point from the world frame into the local frame
}

btVector3 getPointLocalToWorld( btTransform localObjectCenterOfMassTransform, btVector3 point) {
	  return localObjectCenterOfMassTransform * point; // transforms the point from the world frame into the local frame
}

btVector3 getAxisWorldToLocal(btTransform localObjectCenterOfMassTransform, btVector3 axis) {
  btTransform local1 = localObjectCenterOfMassTransform.inverse(); // transforms the axis from the local frame into the world frame
  btVector3 zero(0,0,0);
  local1.setOrigin(zero);
  return local1 * axis;
}

btVector3 getAxisLocalToWorld(btTransform localObjectCenterOfMassTransform, btVector3 axis) {
  btTransform local1 = localObjectCenterOfMassTransform; // transforms the axis from the local frame into the world frame
  btVector3 zero(0,0,0);
  local1.setOrigin(zero);
  return local1 * axis;
}

btTransform getTransformWorldToLocal(btTransform localObjectCenterOfMassTransform, btTransform transform) {
  return localObjectCenterOfMassTransform.inverse() * transform; // transforms the axis from the local frame into the world frame
}

btTransform getTransformLocalToWorld(btTransform localObjectCenterOfMassTransform, btTransform transform) {
  return localObjectCenterOfMassTransform * transform; // transforms the axis from the local frame into the world frame
}
