/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

EPA Copyright (c) Ricardo Padrela 2006 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef EPA_PENETRATION_DEPTH_H
#define EPA_PENETRATION_DEPTH_H

/**
*	EpaPenetrationDepthSolver uses the Expanding Polytope Algorithm to
*	calculate the penetration depth between two convex shapes.
*/

extern btScalar	g_GJKMaxRelError;
extern btScalar	g_GJKMaxRelErrorSqrd;

//! Note : This class is not supposed to be a base class
class EpaPenetrationDepthSolver : public btConvexPenetrationDepthSolver
{
	public :

		bool			calcPenDepth( btSimplexSolverInterface& simplexSolver,
									  btConvexShape* pConvexA, btConvexShape* pConvexB,
									  const btTransform& transformA, const btTransform& transformB,
									  btVector3& v, btPoint3& wWitnessOnA, btPoint3& wWitnessOnB,
									  class btIDebugDraw* debugDraw );

	private :

#ifdef EPA_USE_HYBRID
		bool			HybridPenDepth( btSimplexSolverInterface& simplexSolver,
										btConvexShape* pConvexA, btConvexShape* pConvexB,
										const btTransform& transformA, const btTransform& transformB,
										btPoint3& wWitnessOnA, btPoint3& wWitnessOnB,
										btScalar& penDepth, btVector3& v );
#endif

		btScalar		EpaPenDepth( btSimplexSolverInterface& simplexSolver,
									 btConvexShape* pConvexA, btConvexShape* pConvexB,
									 const btTransform& transformA, const btTransform& transformB,
									 btPoint3& wWitnessOnA, btPoint3& wWitnessOnB );
};

#endif	// EPA_PENETRATION_DEPTH_H

