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


#include "btBulletDynamicsCommon.h"

#include "LinearMath/btIDebugDraw.h"
#include "MotorDemo.h"


#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

#include "../CommonInterfaces/CommonRigidBodyBase.h"

class MotorDemo : public CommonRigidBodyBase
{
	float m_Time;
	float m_fCyclePeriod; // in milliseconds
	float m_fMuscleStrength;
	
	btAlignedObjectArray<class TestRig*> m_rigs;
	
	
public:
	MotorDemo(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper)
	{
	}

	void initPhysics();
	
	void exitPhysics();
	
	virtual ~MotorDemo()
	{
	}
	
	void spawnTestRig(const btVector3& startOffset, bool bFixed);
	
//	virtual void keyboardCallback(unsigned char key, int x, int y);
	
	void setMotorTargets(btScalar deltaTime);
	
	void resetCamera()
	{
		float dist = 11;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
};


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

#ifndef M_PI_8
#define M_PI_8     0.5 * M_PI_4
#endif


// /LOCAL FUNCTIONS



#define NUM_LEGS 6
#define BODYPART_COUNT 2 * NUM_LEGS + 1
#define JOINT_COUNT BODYPART_COUNT - 1

class TestRig
{
	btDynamicsWorld*	m_ownerWorld;
	btCollisionShape*	m_shapes[BODYPART_COUNT];
	btRigidBody*		m_bodies[BODYPART_COUNT];
	btTypedConstraint*	m_joints[JOINT_COUNT];

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}


public:
	TestRig (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, bool bFixed)
		: m_ownerWorld (ownerWorld)
	{
		btVector3 vUp(0, 1, 0);

		//
		// Setup geometry
		//
		float fBodySize  = 0.25f;
		float fLegLength = 0.45f;
		float fForeLegLength = 0.75f;
		m_shapes[0] = new btCapsuleShape(btScalar(fBodySize), btScalar(0.10));
		int i;
		for ( i=0; i<NUM_LEGS; i++)
		{
			m_shapes[1 + 2*i] = new btCapsuleShape(btScalar(0.10), btScalar(fLegLength));
			m_shapes[2 + 2*i] = new btCapsuleShape(btScalar(0.08), btScalar(fForeLegLength));
		}

		//
		// Setup rigid bodies
		//
		float fHeight = 0.5;
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);		

		// root
		btVector3 vRoot = btVector3(btScalar(0.), btScalar(fHeight), btScalar(0.));
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(vRoot);
		if (bFixed)
		{
			m_bodies[0] = localCreateRigidBody(btScalar(0.), offset*transform, m_shapes[0]);
		} else
		{
			m_bodies[0] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[0]);
		}
		// legs
		for ( i=0; i<NUM_LEGS; i++)
		{
			float fAngle = 2 * M_PI * i / NUM_LEGS;
			float fSin = sin(fAngle);
			float fCos = cos(fAngle);

			transform.setIdentity();
			btVector3 vBoneOrigin = btVector3(btScalar(fCos*(fBodySize+0.5*fLegLength)), btScalar(fHeight), btScalar(fSin*(fBodySize+0.5*fLegLength)));
			transform.setOrigin(vBoneOrigin);

			// thigh
			btVector3 vToBone = (vBoneOrigin - vRoot).normalize();
			btVector3 vAxis = vToBone.cross(vUp);			
			transform.setRotation(btQuaternion(vAxis, M_PI_2));
			m_bodies[1+2*i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[1+2*i]);

			// shin
			transform.setIdentity();
			transform.setOrigin(btVector3(btScalar(fCos*(fBodySize+fLegLength)), btScalar(fHeight-0.5*fForeLegLength), btScalar(fSin*(fBodySize+fLegLength))));
			m_bodies[2+2*i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[2+2*i]);
		}

		// Setup some damping on the m_bodies
		for (i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			//m_bodies[i]->setSleepingThresholds(1.6, 2.5);
			m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
		}


		//
		// Setup the constraints
		//
		btHingeConstraint* hingeC;
		//btConeTwistConstraint* coneC;

		btTransform localA, localB, localC;

		for ( i=0; i<NUM_LEGS; i++)
		{
			float fAngle = 2 * M_PI * i / NUM_LEGS;
			float fSin = sin(fAngle);
			float fCos = cos(fAngle);

			// hip joints
			localA.setIdentity(); localB.setIdentity();
			localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*fBodySize), btScalar(0.), btScalar(fSin*fBodySize)));
			localB = m_bodies[1+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
			hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[1+2*i], localA, localB);
			hingeC->setLimit(btScalar(-0.75 * M_PI_4), btScalar(M_PI_8));
			//hingeC->setLimit(btScalar(-0.1), btScalar(0.1));
			m_joints[2*i] = hingeC;
			m_ownerWorld->addConstraint(m_joints[2*i], true);

			// knee joints
			localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
			localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*(fBodySize+fLegLength)), btScalar(0.), btScalar(fSin*(fBodySize+fLegLength))));
			localB = m_bodies[1+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
			localC = m_bodies[2+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
			hingeC = new btHingeConstraint(*m_bodies[1+2*i], *m_bodies[2+2*i], localB, localC);
			//hingeC->setLimit(btScalar(-0.01), btScalar(0.01));
			hingeC->setLimit(btScalar(-M_PI_8), btScalar(0.2));
			m_joints[1+2*i] = hingeC;
			m_ownerWorld->addConstraint(m_joints[1+2*i], true);
		}
	}

	virtual	~TestRig ()
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

	btTypedConstraint** GetJoints() {return &m_joints[0];}

};



void motorPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	MotorDemo* motorDemo = (MotorDemo*)world->getWorldUserInfo();

	motorDemo->setMotorTargets(timeStep);
	
}



void MotorDemo::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	// Setup the basic world

	m_Time = 0;
	m_fCyclePeriod = 2000.f; // in milliseconds

//	m_fMuscleStrength = 0.05f;
	// new SIMD solver for joints clips accumulated impulse, so the new limits for the motor
	// should be (numberOfsolverIterations * oldLimits)
	// currently solver uses 10 iterations, so:
	m_fMuscleStrength = 0.5f;


	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	m_dynamicsWorld->setInternalTickCallback(motorPreTickCallback,this,true);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	

	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));
		createRigidBody(btScalar(0.),groundTransform,groundShape);
	}

	// Spawn one ragdoll
	btVector3 startOffset(1,0.5,0);
	spawnTestRig(startOffset, false);
	startOffset.setValue(-2,0.5,0);
	spawnTestRig(startOffset, true);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void MotorDemo::spawnTestRig(const btVector3& startOffset, bool bFixed)
{
	TestRig* rig = new TestRig(m_dynamicsWorld, startOffset, bFixed);
	m_rigs.push_back(rig);
}

void	PreStep()
{

}




void MotorDemo::setMotorTargets(btScalar deltaTime)
{

	float ms = deltaTime*1000000.;
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	m_Time += ms;

	//
	// set per-frame sinusoidal position targets using angular motor (hacky?)
	//	
	for (int r=0; r<m_rigs.size(); r++)
	{
		for (int i=0; i<2*NUM_LEGS; i++)
		{
			btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_rigs[r]->GetJoints()[i]);
			btScalar fCurAngle      = hingeC->getHingeAngle();
			
			btScalar fTargetPercent = (int(m_Time / 1000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
			btScalar fTargetAngle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent));
			btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
			btScalar fAngleError  = fTargetLimitAngle - fCurAngle;
			btScalar fDesiredAngularVel = 1000000.f * fAngleError/ms;
			hingeC->enableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);
		}
	}

	
}

#if 0
void MotorDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '+': case '=':
		m_fCyclePeriod /= 1.1f;
		if (m_fCyclePeriod < 1.f)
			m_fCyclePeriod = 1.f;
		break;
	case '-': case '_':
		m_fCyclePeriod *= 1.1f;
		break;
	case '[':
		m_fMuscleStrength /= 1.1f;
		break;
	case ']':
		m_fMuscleStrength *= 1.1f;
		break;
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}	
}
#endif



void MotorDemo::exitPhysics()
{

	int i;

	for (i=0;i<m_rigs.size();i++)
	{
		TestRig* rig = m_rigs[i];
		delete rig;
	}

	//cleanup in the reverse order of creation/initialization

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
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;	
}


class CommonExampleInterface*    MotorControlCreateFunc(struct CommonExampleOptions& options)
{
	return new MotorDemo(options.m_guiHelper);
}