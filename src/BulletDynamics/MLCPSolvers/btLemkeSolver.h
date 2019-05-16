/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///original version written by Erwin Coumans, October 2013

#ifndef BT_LEMKE_SOLVER_H
#define BT_LEMKE_SOLVER_H

#include "btMLCPSolverInterface.h"
#include "btLemkeAlgorithm.h"

///The btLemkeSolver is based on "Fast Implementation of Lemke’s Algorithm for Rigid Body Contact Simulation (John E. Lloyd) "
///It is a slower but more accurate solver. Increase the m_maxLoops for better convergence, at the cost of more CPU time.
///The original implementation of the btLemkeAlgorithm was done by Kilian Grundl from the MBSim team
class btLemkeSolver : public btMLCPSolverInterface
{
protected:
public:
	btScalar m_maxValue;
	int m_debugLevel;
	int m_maxLoops;
	bool m_useLoHighBounds;

	btLemkeSolver()
		: m_maxValue(100000),
		  m_debugLevel(0),
		  m_maxLoops(1000),
		  m_useLoHighBounds(true)
	{
	}
	virtual bool solveMLCP(const btMatrixXu& A, const btVectorXu& b, btVectorXu& x, const btVectorXu& lo, const btVectorXu& hi, const btAlignedObjectArray<int>& limitDependency, int numIterations, bool useSparsity = true);
};

#endif  //BT_LEMKE_SOLVER_H
