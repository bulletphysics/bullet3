//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef DY_SOLVERCONSTRAINT1D4_H
#define DY_SOLVERCONSTRAINT1D4_H

#include "foundation/PxVec3.h"
#include "PxvConfig.h"
#include "DyArticulationUtils.h"
#include "DySolverConstraint1D.h"

namespace physx
{

namespace Dy
{

struct SolverConstraint1DHeader4
{
	PxU8	type;			// enum SolverConstraintType - must be first byte
	PxU8	pad0[3];	
	//These counts are the max of the 4 sets of data.
	//When certain pairs have fewer constraints than others, they are padded with 0s so that no work is performed but 
	//calculations are still shared (afterall, they're computationally free because we're doing 4 things at a time in SIMD)
	PxU32	count;
	PxU8	count0, count1, count2, count3;
	PxU8	break0, break1, break2, break3;

	Vec4V	linBreakImpulse;
	Vec4V	angBreakImpulse;
	Vec4V	invMass0D0;
	Vec4V	invMass1D1;
	Vec4V	angD0;
	Vec4V	angD1;

	Vec4V	body0WorkOffsetX;
	Vec4V	body0WorkOffsetY;
	Vec4V	body0WorkOffsetZ;
};

struct SolverConstraint1DBase4 
{
public:
	Vec4V		lin0X;
	Vec4V		lin0Y;
	Vec4V		lin0Z;
	Vec4V		ang0X;
	Vec4V		ang0Y;
	Vec4V		ang0Z;
	Vec4V		ang0WritebackX;
	Vec4V		ang0WritebackY;
	Vec4V		ang0WritebackZ;
	Vec4V		constant;
	Vec4V		unbiasedConstant;
	Vec4V		velMultiplier;
	Vec4V		impulseMultiplier;
	Vec4V		minImpulse;
	Vec4V		maxImpulse;
	Vec4V		appliedForce;
	PxU32		flags[4];
};

PX_COMPILE_TIME_ASSERT(sizeof(SolverConstraint1DBase4) == 272);

struct SolverConstraint1DDynamic4 : public SolverConstraint1DBase4
{
	Vec4V		lin1X;
	Vec4V		lin1Y;
	Vec4V		lin1Z;
	Vec4V		ang1X;
	Vec4V		ang1Y;
	Vec4V		ang1Z;
};
PX_COMPILE_TIME_ASSERT(sizeof(SolverConstraint1DDynamic4) == 368);

}

}

#endif //DY_SOLVERCONSTRAINT1D4_H
