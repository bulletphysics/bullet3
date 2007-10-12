/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser bteral Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       bteral Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#define USE_SOR_SOLVER
#ifdef USE_SOR_SOLVER

#ifndef SOR_LCP_H
#define SOR_LCP_H
struct	OdeSolverBody;
class BU_Joint;
#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btStackAlloc.h"

struct btContactSolverInfo;


//=============================================================================
class SorLcpSolver //Remotion: 11.10.2007
{
public:
	SorLcpSolver()
	{
		dRand2_seed = 0;
	}

	void SolveInternal1 (float global_cfm,
		float global_erp,
		const btAlignedObjectArray<OdeSolverBody*> &body, int nb,
		btAlignedObjectArray<BU_Joint*> &joint, 
		int nj, const btContactSolverInfo& solverInfo,
		btStackAlloc* stackAlloc
		);

public: //data
	unsigned long dRand2_seed;

protected: //typedef
	typedef const btScalar *dRealPtr;
	typedef btScalar *dRealMutablePtr;

protected: //members
	//------------------------------------------------------------------------------
	SIMD_FORCE_INLINE unsigned long dRand2()
	{
		dRand2_seed = (1664525L*dRand2_seed + 1013904223L) & 0xffffffff;
		return dRand2_seed;
	}
	//------------------------------------------------------------------------------
	SIMD_FORCE_INLINE int dRandInt2 (int n)
	{
		float a = float(n) / 4294967296.0f;
		return (int) (float(dRand2()) * a);
	}
	//------------------------------------------------------------------------------
	void SOR_LCP(int m, int nb, dRealMutablePtr J, int *jb, 
		const btAlignedObjectArray<OdeSolverBody*> &body,
		dRealPtr invI, dRealMutablePtr lambda, dRealMutablePtr invMforce, dRealMutablePtr rhs,
		dRealMutablePtr lo, dRealMutablePtr hi, dRealPtr cfm, int *findex,
		int numiter,float overRelax,
		btStackAlloc* stackAlloc
		);
};


//=============================================================================
class AutoBlockSa //Remotion: 10.10.2007
{
	btStackAlloc* stackAlloc;
	btBlock*	  saBlock;
public:
	AutoBlockSa(btStackAlloc* stackAlloc_)
	{
		stackAlloc = stackAlloc_;
		saBlock = stackAlloc->beginBlock();
	}
	~AutoBlockSa()
	{
		stackAlloc->endBlock(saBlock);
	}
	//operator btBlock* () { return saBlock; }
};
// //Usage
//void function(btStackAlloc* stackAlloc)
//{
//	AutoBlockSa(stackAlloc);
//   ...
//	if(...) return;
//	return;
//}
//------------------------------------------------------------------------------


#endif //SOR_LCP_H

#endif //USE_SOR_SOLVER

