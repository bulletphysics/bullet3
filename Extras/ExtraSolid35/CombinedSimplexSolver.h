/*
 * Copyright (c) 2005 Erwin Coumans http://www.erwincoumans.com
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/

#ifndef COMBINED_SIMPLEX_SOLVER
#define COMBINED_SIMPLEX_SOLVER

#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "Solid3JohnsonSimplexSolver.h"

/// CombinedSimplexSolver runs both Solid and Voronoi Simplex Solver for comparison
class CombinedSimplexSolver: public SimplexSolverInterface
{
	VoronoiSimplexSolver	m_voronoiSolver;
//	VoronoiSimplexSolver	m_johnsonSolver;

	Solid3JohnsonSimplexSolver	m_johnsonSolver;
	
	bool	m_useVoronoiSolver;

	void	debugPrint();

	public:
		CombinedSimplexSolver();
	
	virtual ~CombinedSimplexSolver() {};

	virtual void reset();

	virtual void addVertex(const SimdVector3& w, const SimdPoint3& p, const SimdPoint3& q);
	
	virtual bool closest(SimdVector3& v);

	virtual SimdScalar maxVertex();

	virtual bool fullSimplex() const;

	virtual int getSimplex(SimdPoint3 *pBuf, SimdPoint3 *qBuf, SimdVector3 *yBuf) const;

	virtual bool inSimplex(const SimdVector3& w);
	
	virtual void backup_closest(SimdVector3& v) ;

	virtual bool emptySimplex() const;

	virtual void compute_points(SimdPoint3& p1, SimdPoint3& p2);

	virtual int numVertices() const;

	
};


#endif //COMBINED_SIMPLEX_SOLVER