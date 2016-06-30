/*
 * Ball.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: leviathan
 */

#include "Ball.h"

#include "Breakout.h"
#include "btBulletDynamicsCommon.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

Ball::Ball() {
	m_tag = BALL;
	m_shape = new btSphereShape(btScalar(1));

	btScalar sphereMass(1.f);

	createBodyWithMass(sphereMass);
}

Ball::~Ball() {
	// TODO Auto-generated destructor stub
}

