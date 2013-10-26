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


class btLemkeSolver : public btMLCPSolverInterface
{
public:
	virtual bool solveMLCP(const btMatrixXu & A, const btVectorXu & b, btVectorXu& x, const btVectorXu & lo,const btVectorXu & hi,const btAlignedObjectArray<int>& limitDependency, int numIterations, bool useSparsity = true)
	{
		int dimension = A.rows();
		if (0==dimension)
			return true;
		
//		printf("================ solving using Lemke/Newton/Fixpoint\n");

		btVectorXu q;
		q.resize(dimension);
		for (int row=0;row<dimension;row++)
		{
			q[row] = -b[row];
		}
		
		int debugLevel=0;
		btLemkeAlgorithm lemke(A,q,debugLevel);
		
		
		lemke.setSystem(A,q);
		
		int maxloops = 10000;
		btVectorXu solution = lemke.solve(maxloops);
		
		//check solution
		
		bool fail = false;
		int errorIndexMax = -1;
		int errorIndexMin = -1;
		float errorValueMax = -1e30;
		float errorValueMin = 1e30;
		
		for (int i=0;i<dimension;i++)
		{
			x[i] = solution[i+dimension];
			volatile btScalar check = x[i];
			if (x[i] != check)
			{
				x.setZero();
				return false;
			}
			
			//this is some hack/safety mechanism, to discard invalid solutions from the Lemke solver 
			//we need to figure out why it happens, and fix it, or detect it properly)
			if (x[i]>100000)
			{
				if (x[i]> errorValueMax)
				{
					fail = true;
					errorIndexMax = i;
					errorValueMax = x[i];
				}
				////printf("x[i] = %f,",x[i]);
			}
			if (x[i]<-10000)
			{
				if (x[i]<errorValueMin)
				{
					errorIndexMin = i;
					errorValueMin = x[i];
					fail = true;
					//printf("x[i] = %f,",x[i]);
				}
			}
		}
		if (fail)
		{
			static int errorCountTimes = 0;
			if (errorIndexMin<0)
				errorValueMin = 0.f;
			if (errorIndexMax<0)
				errorValueMax = 0.f;
			printf("Error (x[%d] = %f, x[%d] = %f), resetting %d times\n", errorIndexMin,errorValueMin, errorIndexMax, errorValueMax, errorCountTimes++);
			for (int i=0;i<dimension;i++)
			{
				x[i]=0.f;
			}
		}
#if 0
		if (lemke.getInfo()<0)
		{
			printf("Lemke found no solution, info = %d\n",lemke.getInfo());
		} else
		{
			printf("Lemke info = %d, found a solution in %d steps\n",lemke.getInfo(),lemke.getSteps());
			//printf("Lemke found a solution\n");
			for (int i=0;i<dimension;i++)
			{
				x[i] = solution(i+dimension);
			}
		}
#endif

		return !fail;
		
	}

};

#endif //BT_LEMKE_SOLVER_H
