/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#include "Ragdoll.h"

//#define RIGID 1

RagDoll::RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset,
	btScalar scale_ragdoll)	: m_ownerWorld (ownerWorld)
{


	// Setup the geometry
	m_shapes[BODYPART_PELVIS] = new btCapsuleShape(
	btScalar(scale_ragdoll*0.15), btScalar(scale_ragdoll*0.20));
	m_shapes[BODYPART_SPINE] = new btCapsuleShape(
	btScalar(scale_ragdoll*0.15), btScalar(scale_ragdoll*0.28));
	m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(scale_ragdoll*0.10), btScalar(scale_ragdoll*0.05));
	m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape(btScalar(scale_ragdoll*0.07), btScalar(scale_ragdoll*0.45));
	m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape(btScalar(scale_ragdoll*0.05), btScalar(scale_ragdoll*0.37));
	m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(btScalar(scale_ragdoll*0.07), btScalar(scale_ragdoll*0.45));
	m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(btScalar(scale_ragdoll*0.05), btScalar(scale_ragdoll*0.37));
	m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(scale_ragdoll*0.05), btScalar(scale_ragdoll*0.33));
	m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(scale_ragdoll*0.04), btScalar(scale_ragdoll*0.25));
	m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(scale_ragdoll*0.05), btScalar(scale_ragdoll*0.33));
	m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(scale_ragdoll*0.04), btScalar(scale_ragdoll*0.25));

	// Setup all the rigid bodies
	btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(scale_ragdoll*1.), btScalar(0.)));
	m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(scale_ragdoll*1.2), btScalar(0.)));
	m_bodies[BODYPART_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_SPINE]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(scale_ragdoll*1.6), btScalar(0.)));
	m_bodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_HEAD]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.18*scale_ragdoll), btScalar(0.65*scale_ragdoll),
btScalar(0.)));
	m_bodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.18*scale_ragdoll), btScalar(0.2*scale_ragdoll), btScalar(0.)));
	m_bodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.18*scale_ragdoll), btScalar(0.65*scale_ragdoll), btScalar(0.)));
	m_bodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.18*scale_ragdoll), btScalar(0.2*scale_ragdoll), btScalar(0.)));
	m_bodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.35*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,SIMD_HALF_PI);
	m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.7*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,SIMD_HALF_PI);
	m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.35*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-SIMD_HALF_PI);
	m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.7*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-SIMD_HALF_PI);
	m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);

	// Setup some damping on the m_bodies
	for (int i = 0; i < BODYPART_COUNT; ++i)
	{
		m_bodies[i]->setDamping(0.05f, 0.85f);
		m_bodies[i]->setDeactivationTime(0.8f);
		m_bodies[i]->setSleepingThresholds(1.6f, 2.5f);
	}

///////////////////////////// SETTING THE CONSTRAINTS /////////////////////////////////////////////7777
	// Now setup the constraints
	btGeneric6DofConstraint * joint6DOF;
	btTransform localA, localB;
	bool useLinearReferenceFrameA = true;
/// ******* SPINE HEAD ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30*scale_ragdoll), btScalar(0.)));

		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14*scale_ragdoll), btScalar(0.)));

		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.3f,-SIMD_EPSILON,-SIMD_PI*0.3f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.5f,SIMD_EPSILON,SIMD_PI*0.3f));
#endif
		m_joints[JOINT_SPINE_HEAD] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);
	}
/// *************************** ///




/// ******* LEFT SHOULDER ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(-0.2*scale_ragdoll), btScalar(0.15*scale_ragdoll), btScalar(0.)));

		localB.getBasis().setEulerZYX(SIMD_HALF_PI,0,-SIMD_HALF_PI);
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18*scale_ragdoll), btScalar(0.)));

		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.8f,-SIMD_EPSILON,-SIMD_PI*0.5f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.8f,SIMD_EPSILON,SIMD_PI*0.5f));
#endif
		m_joints[JOINT_LEFT_SHOULDER] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);
	}
/// *************************** ///


/// ******* RIGHT SHOULDER ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.2*scale_ragdoll), btScalar(0.15*scale_ragdoll), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,SIMD_HALF_PI);
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18*scale_ragdoll), btScalar(0.)));
		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else

		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.8f,-SIMD_EPSILON,-SIMD_PI*0.5f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.8f,SIMD_EPSILON,SIMD_PI*0.5f));
#endif
		m_joints[JOINT_RIGHT_SHOULDER] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);
	}
/// *************************** ///

/// ******* LEFT ELBOW ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14*scale_ragdoll), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7f,SIMD_EPSILON,SIMD_EPSILON));
#endif
		m_joints[JOINT_LEFT_ELBOW] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);
	}
/// *************************** ///

/// ******* RIGHT ELBOW ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14*scale_ragdoll), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7,SIMD_EPSILON,SIMD_EPSILON));
#endif

		m_joints[JOINT_RIGHT_ELBOW] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
	}
/// *************************** ///


/// ******* PELVIS ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.getBasis().setEulerZYX(0,SIMD_HALF_PI,0);
		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15*scale_ragdoll), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,SIMD_HALF_PI,0);
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15*scale_ragdoll), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.2,-SIMD_EPSILON,-SIMD_PI*0.3));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.2,SIMD_EPSILON,SIMD_PI*0.6));
#endif
		m_joints[JOINT_PELVIS_SPINE] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);
	}
/// *************************** ///

/// ******* LEFT HIP ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(-0.18*scale_ragdoll), btScalar(-0.10*scale_ragdoll), btScalar(0.)));

		localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225*scale_ragdoll), btScalar(0.)));

		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_HALF_PI*0.5,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_HALF_PI*0.8,SIMD_EPSILON,SIMD_HALF_PI*0.6f));
#endif
		m_joints[JOINT_LEFT_HIP] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);
	}
/// *************************** ///


/// ******* RIGHT HIP ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.18*scale_ragdoll), btScalar(-0.10*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225*scale_ragdoll), btScalar(0.)));

		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_HALF_PI*0.5,-SIMD_EPSILON,-SIMD_HALF_PI*0.6f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_HALF_PI*0.8,SIMD_EPSILON,SIMD_EPSILON));
#endif
		m_joints[JOINT_RIGHT_HIP] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);
	}
/// *************************** ///


/// ******* LEFT KNEE ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185*scale_ragdoll), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB,useLinearReferenceFrameA);
//
#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7f,SIMD_EPSILON,SIMD_EPSILON));
#endif
		m_joints[JOINT_LEFT_KNEE] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);
	}
/// *************************** ///

/// ******* RIGHT KNEE ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185*scale_ragdoll), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7f,SIMD_EPSILON,SIMD_EPSILON));
#endif
		m_joints[JOINT_RIGHT_KNEE] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);
	}
/// *************************** ///

}


RagDoll::~RagDoll()
{
	int i;

	// Remove all constraints
	for (i = 0; i < JOINT_COUNT; ++i)
	{
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = 0;
	}

	// Remove all bodies and shapes
	for (i = 0; i < BODYPART_COUNT; ++i)
	{
		m_ownerWorld->removeRigidBody(m_bodies[i]);

		delete m_bodies[i]->getMotionState();

		delete m_bodies[i]; m_bodies[i] = 0;
		delete m_shapes[i]; m_shapes[i] = 0;
	}
}


btRigidBody* RagDoll::localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	rbInfo.m_additionalDamping = true;
	btRigidBody* body = new btRigidBody(rbInfo);

	m_ownerWorld->addRigidBody(body);

	return body;
}
