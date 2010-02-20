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
class CombinedSimplexSolver: public btSimplexSolverInterface
{
	btVoronoiSimplexSolver	m_voronoiSolver;
//	btVoronoiSimplexSolver	m_johnsonSolver;

	Solid3JohnsonSimplexSolver	m_johnsonSolver;
	
	bool	m_useVoronoiSolver;

	void	debugPrint();

	public:
		CombinedSimplexSolver();
	
	virtual ~CombinedSimplexSolver() {};

	virtual void reset();

	virtual void addVertex(const btVector3& w, const btPoint3& p, const btPoint3& q);
	
	virtual bool closest(btVector3& v);

	virtual btScalar maxVertex();

	virtual bool fullSimplex() const;

	virtual int getSimplex(btPoint3 *pBuf, btPoint3 *qBuf, btVector3 *yBuf) const;

	virtual bool inSimplex(const btVector3& w);
	
	virtual void backup_closest(btVector3& v) ;

	virtual bool emptySimplex() const;

	virtual void compute_points(btPoint3& p1, btPoint3& p2);

	virtual int numVertices() const;

	
};


#endif //COMBINED_SIMPLEX_SOLVER