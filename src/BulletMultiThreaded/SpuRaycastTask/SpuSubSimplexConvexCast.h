/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef SPU_SUBSIMPLEX_RAY_CAST_H
#define SPU_SUBSIMPLEX_RAY_CAST_H

#include "../SpuNarrowPhaseCollisionTask/SpuVoronoiSimplexSolver.h"
#include "../SpuNarrowPhaseCollisionTask/SpuCollisionShapes.h"
#include "SpuRaycastTask.h"

class btConvexShape;

struct SpuCastResult
{
	float m_fraction;
	btVector3 m_normal;
};

/// btSubsimplexConvexCast implements Gino van den Bergens' paper
///"Ray Casting against bteral Convex Objects with Application to Continuous Collision Detection"
/// GJK based Ray Cast, optimized version
/// Objects should not start in overlap, otherwise results are not defined.
class SpuSubsimplexRayCast
{
	SpuVoronoiSimplexSolver* m_simplexSolver;
	void* m_shapeB;
	SpuConvexPolyhedronVertexData* m_convexDataB;
	int m_shapeTypeB;
	float m_marginB;

public:
	SpuSubsimplexRayCast (void* shapeB, SpuConvexPolyhedronVertexData* convexDataB, int shapeTypeB, float marginB,
						  SpuVoronoiSimplexSolver* simplexSolver);

	//virtual ~btSubsimplexConvexCast();

	///SimsimplexConvexCast calculateTimeOfImpact calculates the time of impact+normal for the linear cast (sweep) between two moving objects.
	///Precondition is that objects should not penetration/overlap at the start from the interval. Overlap can be tested using btGjkPairDetector.
	bool calcTimeOfImpact(const btTransform& fromRay,
						  const btTransform& toRay,
						  const btTransform& fromB,
						  const btTransform& toB,
						  SpuCastResult& result);

};

#endif //SUBSIMPLEX_RAY_CAST_H
