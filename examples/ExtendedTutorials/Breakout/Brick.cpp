/*
 * Brick.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: leviathan
 */

#include "Brick.h"

Brick::Brick() {
	m_tag = BRICK;
	m_shape = new btBoxShape(btVector3(1.0f,0.5f,0.5f));

	btScalar brickMass(1.f);

	createBodyWithMass(brickMass);
}

Brick::~Brick() {
	// TODO Auto-generated destructor stub
}

