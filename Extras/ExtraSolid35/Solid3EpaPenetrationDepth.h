/*
 * SOLID - Software Library for Interference Detection
 * 
 * Copyright (C) 2001-2003  Dtecta.  All rights reserved.
 *
 * This library may be distributed under the terms of the Q Public License
 * (QPL) as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE.QPL included in the packaging of this file.
 *
 * This library may be distributed and/or modified under the terms of the
 * GNU bteral Public License (GPL) version 2 as published by the Free Software
 * Foundation and appearing in the file LICENSE.GPL included in the
 * packaging of this file.
 *
 * This library is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Commercial use or any other use of this library not covered by either 
 * the QPL or the GPL requires an additional license from Dtecta. 
 * Please contact info@dtecta.com for enquiries about the terms of commercial
 * use of this library.
 */

#ifndef SOLID3_EPA_PENETRATION_DEPTH_H
#define SOLID3_EPA_PENETRATION_DEPTH_H


#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"

/// Solid3EpaPenetrationDepth contains the 'Expanding Polytope Algorithm' from Solid 3.5
class Solid3EpaPenetrationDepth : public btConvexPenetrationDepthSolver
{
public:

	virtual bool calcPenDepth(btSimplexSolverInterface& simplexSolver,
		btConvexShape* convexA,btConvexShape* convexB,
		const btTransform& transformA,const btTransform& transformB,
				btVector3& v, btPoint3& pa, btPoint3& pb);

};

#endif //SOLID3_EPA_PENETRATION_DEPTH_H