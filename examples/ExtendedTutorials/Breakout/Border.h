/*
 * Border.h
 *
 *  Created on: Jun 28, 2016
 *      Author: leviathan
 */

#ifndef EXAMPLES_SRC_EXTENDEDTUTORIALS_BREAKOUT_BORDER_H_
#define EXAMPLES_SRC_EXTENDEDTUTORIALS_BREAKOUT_BORDER_H_

#include "GameObject.h"

#include "LinearMath/btVector3.h"

/*
 *
 */
class Border: public GameObject {
public:
	Border();
	virtual ~Border();

	static Vertex m_border_vertices[132];
};

#endif /* EXAMPLES_SRC_EXTENDEDTUTORIALS_BREAKOUT_BORDER_H_ */
