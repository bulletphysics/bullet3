/*
 * Paddle.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: leviathan
 */

#include "Paddle.h"

Paddle::Paddle() {
	m_tag = PADDLE;
	m_shape = new btBoxShape(btVector3(4,1,1));

	btScalar paddleMass(0.f);

	createBodyWithMass(paddleMass);

}

Paddle::~Paddle() {
	// TODO Auto-generated destructor stub
}

