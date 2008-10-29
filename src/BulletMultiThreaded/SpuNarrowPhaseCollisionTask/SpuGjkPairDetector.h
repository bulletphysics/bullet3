
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




#ifndef SPU_GJK_PAIR_DETECTOR_H
#define SPU_GJK_PAIR_DETECTOR_H



#include "SpuContactResult.h"


#include "SpuVoronoiSimplexSolver.h"
class SpuConvexPenetrationDepthSolver;

/// btGjkPairDetector uses GJK to implement the btDiscreteCollisionDetectorInterface
class SpuGjkPairDetector 
{
	

	btVector3	m_cachedSeparatingAxis;
	btScalar	m_cachedSeparatingDistance;
	const SpuConvexPenetrationDepthSolver*	m_penetrationDepthSolver;
	SpuVoronoiSimplexSolver* m_simplexSolver;
	void* m_minkowskiA;
	void* m_minkowskiB;
    int m_shapeTypeA;
    int m_shapeTypeB;
    float m_marginA;
    float m_marginB;
	bool		m_ignoreMargin;
	

public:

	//some debugging to fix degeneracy problems
	int			m_lastUsedMethod;
	int			m_curIter;
	int			m_degenerateSimplex;
	int			m_catchDegeneracies;


	SpuGjkPairDetector(void* objectA,void* objectB,int m_shapeTypeA, int m_shapeTypeB, float marginA, float marginB, SpuVoronoiSimplexSolver* simplexSolver, const SpuConvexPenetrationDepthSolver*	penetrationDepthSolver);
	virtual ~SpuGjkPairDetector() {};

	virtual void	getClosestPoints(const SpuClosestPointInput& input,SpuContactResult& output);

	void setMinkowskiA(void* minkA)
	{
		m_minkowskiA = minkA;
	}

	void setMinkowskiB(void* minkB)
	{
		m_minkowskiB = minkB;
	}

	void setCachedSeperatingAxis(const btVector3& seperatingAxis)
	{
		m_cachedSeparatingAxis = seperatingAxis;
	}

	const btVector3&	getCachedSeparatingAxis() const
	{
		return m_cachedSeparatingAxis;
	}
	btScalar getCachedSeparatingDistance() const
	{
		return m_cachedSeparatingDistance;
	}

	void	setPenetrationDepthSolver(SpuConvexPenetrationDepthSolver*	penetrationDepthSolver)
	{
		m_penetrationDepthSolver = penetrationDepthSolver;
	}

	///don't use setIgnoreMargin, it's for Bullet's internal use
	void	setIgnoreMargin(bool ignoreMargin)
	{
		m_ignoreMargin = ignoreMargin;
	}


};



#endif //SPU_GJK_PAIR_DETECTOR_H
