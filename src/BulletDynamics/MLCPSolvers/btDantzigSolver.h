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

#ifndef BT_DANTZIG_SOLVER_H
#define BT_DANTZIG_SOLVER_H

#include "btMLCPSolverInterface.h"
#include "btDantzigLCP.h"


class btDantzigSolver : public btMLCPSolverInterface
{
protected:

	btScalar m_acceptableUpperLimitSolution;

	btAlignedObjectArray<char>	m_tempBuffer;
public:

	btDantzigSolver()
		:m_acceptableUpperLimitSolution(btScalar(1000))
	{
	}

	virtual bool solveMLCP(const btMatrixXu & A, const btVectorXu & b, btVectorXu& x, const btVectorXu & lo,const btVectorXu & hi,const btAlignedObjectArray<int>& limitDependency, int numIterations, bool useSparsity = true)
	{
		bool result = true;
		int n = b.rows();
		if (n)
		{
			btScalar* AA = (btScalar*) A.getBufferPointer();
			btScalar* bb = (btScalar* ) b.getBufferPointer();
			btScalar* xx = (btScalar*) x.getBufferPointer();
			btScalar* llo = (btScalar*) lo.getBufferPointer();
			btScalar* hhi = (btScalar*) hi.getBufferPointer();
			int* findex = (int*) &limitDependency[0];
			int nub = 0;
			btAlignedObjectArray<btScalar> ww;
			ww.resize(n);
		


			const btScalar* Aptr = A.getBufferPointer();

			for (int i=0;i<n*n;i++)
			{
				AA[i] = Aptr[i];
			}
			for (int i=0;i<n;i++)
			{
				llo[i] = lo[i];
				hhi[i] = hi[i];
				bb[i] = b[i];
				xx[i] = x[i];
			}

			extern int numAllocas;
			numAllocas = 0;

			result = btSolveDantzigLCP (n,AA,xx,bb,&ww[0],nub,llo,hhi,findex);
			
//			printf("numAllocas = %d\n",numAllocas);
			for (int i=0;i<n;i++)
			{
				x[i] = xx[i];

				//test for NAN
				if (x[i] != xx[i])
				{
					return false;
				}
				if (x[i] >= m_acceptableUpperLimitSolution)
				{
					return false;
				}

				if (x[i] <= -m_acceptableUpperLimitSolution)
				{
					return false;
				}
				
			}
			
		}

		return result;
	}
};

#endif //BT_DANTZIG_SOLVER_H
