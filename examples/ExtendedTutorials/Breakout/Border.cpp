/*
 * Border.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: leviathan
 */

#include "Border.h"

Border::Border() {
	m_tag = BORDER;
	createShapeWithVertices(m_border_vertices, 132, false);

	createBodyWithMass(0);

}

Border::~Border() {
	// TODO Auto-generated destructor stub
}

